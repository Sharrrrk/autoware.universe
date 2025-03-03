// Copyright 2022 The Autoware Contributors
// SPDX-License-Identifier: Apache-2.0

#include "pointcloud_preprocessor/polgon_remover/polgon_remover.hpp"

using K = CGAL::Exact_predicates_inexact_constructions_kernel;
using PointCgal = K::Point_2;

namespace pointcloud_preprocessor
{
PolgonRemoverComponent::PolgonRemoverComponent(const rclcpp::NodeOptions & options)
: Filter("PolgonRemover", options)
{
  pub_marker_ptr_ = this->create_publisher<visualization_msgs::msg::Marker>("Removed_polygon", 10);

	this->declare_parameter<std::vector<double>>("polygon_vertices");
	this->get_parameter("polygon_vertices", param);
  this->declare_parameter<bool>("will_visualize");
  this->get_parameter("will_visualize", will_visualize_);
	polygon_vertices_ = param.as_double_array();    
	if (polygon_vertices_.size() % 2 != 0) 
  {
    throw std::length_error(
        "polygon_vertices has odd number of elements. "
        "It must have a list of x,y double pairs.");
  }

  auto make_point = [](float x, float y, float z) {
    geometry_msgs::msg::Point32 point_32;
    point_32.set__x(x);
    point_32.set__y(y);
    point_32.set__z(z);
    return point_32;
  };
  polygon_ = std::make_shared<geometry_msgs::msg::Polygon>();
  for (size_t i = 0UL; i < polygon_vertices_.size(); i += 2) 
  {
    auto p_x = static_cast<float>(polygon_vertices_.at(i));
    auto p_y = static_cast<float>(polygon_vertices_.at(i + 1));
    polygon_->points.emplace_back(make_point(p_x, p_y, 0.0F));
  }
  this->update_polygon(polygon_);
}

void PolgonRemoverComponent::filter(
  const PointCloud2ConstPtr & input, 
  [[maybe_unused]] const IndicesPtr & indices,
  PointCloud2 & output)
{
  if (!this->polygon_is_initialized_) 
  {
    RCLCPP_INFO_STREAM(get_logger(), "Polygon is not initialized, publishing incoming cloud.");
    output = *input;
    return;
  }

  output = this->remove_updated_polygon_from_cloud(input);
  if (will_visualize_) 
  {
    pub_marker_ptr_->publish(marker_);
  }

}

void PolgonRemoverComponent::update_polygon(const geometry_msgs::msg::Polygon::ConstSharedPtr & polygon_in)
{
	polygon_cgal_ = polygon_geometry_to_cgal(polygon_in);
	if (will_visualize_) 
	{
		marker_.ns = "";
		marker_.id = 0;
		marker_.type = visualization_msgs::msg::Marker::LINE_LIST;
		marker_.action = visualization_msgs::msg::Marker::ADD;
		marker_.pose.orientation.w = 1.0;
		marker_.scale.x = 0.2;
		marker_.color.a = 1.0;
		marker_.color.r = 1.0;
		marker_.color.g = 1.0;
		marker_.color.b = 1.0;

		auto make_point = [](float x, float y, float z) {
			geometry_msgs::msg::Point point;
			point.x = static_cast<double>(x);
			point.y = static_cast<double>(y);
			point.z = static_cast<double>(z);
			return point;
		};
		for (size_t index_cur = 0; index_cur < polygon_cgal_.size(); ++index_cur) 
		{
			const auto & vertex = polygon_cgal_.at(index_cur);

			// Take the last segment into consideration to connect the loop
			size_t index_next = index_cur == polygon_cgal_.size() - 1 ? 0UL : index_cur + 1;
			const auto & vertex_next = polygon_cgal_.at(index_next);

			// Build upper ring 
			auto vertex_up_cur = make_point(
			static_cast<float>(vertex.x()),
			static_cast<float>(vertex.y()), 5.0F);
			auto vertex_up_next =
			make_point(
			static_cast<float>(vertex_next.x()),
			static_cast<float>(vertex_next.y()), 5.0F);
			marker_.points.emplace_back(vertex_up_cur);
			marker_.points.emplace_back(vertex_up_next);

			// Build lower ring
			auto vertex_down_cur =
			make_point(static_cast<float>(vertex.x()), static_cast<float>(vertex.y()), -5.0F);
			auto vertex_down_next =
			make_point(
			static_cast<float>(vertex_next.x()),
			static_cast<float>(vertex_next.y()), -5.0F);
			marker_.points.emplace_back(vertex_down_cur);
			marker_.points.emplace_back(vertex_down_next);

			// Connect up and down vertices 
			marker_.points.emplace_back(vertex_up_cur);
			marker_.points.emplace_back(vertex_down_cur);
		}
	}
	polygon_is_initialized_ = true;
}

sensor_msgs::msg::PointCloud2 PolgonRemoverComponent::remove_updated_polygon_from_cloud(const PointCloud2ConstPtr & cloud_in)
{
  if (will_visualize_) {
      marker_.header.frame_id = cloud_in->header.frame_id;
  }
  if (!polygon_is_initialized_) {
      throw std::runtime_error("Polygon is not initialized. Please use `update_polygon` first.");
  }

  return remove_polygon_cgal_from_cloud(cloud_in, polygon_cgal_);
}

sensor_msgs::msg::PointCloud2 PolgonRemoverComponent::remove_polygon_cgal_from_cloud(
        const sensor_msgs::msg::PointCloud2::ConstSharedPtr & cloud_in_ptr,
        const std::vector<PointCgal> & polyline_polygon)
{
  sensor_msgs::msg::PointCloud2 output;
  pcl::PointCloud<pcl::PointXYZ> pcl_output;

  sensor_msgs::msg::PointCloud2 transformed_cluster = *cloud_in_ptr;

  for(sensor_msgs::PointCloud2ConstIterator<float> iter_x(transformed_cluster, "x"), iter_y(transformed_cluster, "y"), iter_z(transformed_cluster, "z");
        iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z)
  {
    if(CGAL::bounded_side_2(polyline_polygon.begin(), polyline_polygon.end(),PointCgal(*iter_x, *iter_y), K()) == CGAL::ON_UNBOUNDED_SIDE)
    {
      pcl::PointXYZ p;
      p.x = *iter_x;
      p.y = *iter_y;
      p.z = *iter_z;
      pcl_output.emplace_back(p);
    }
  }
  pcl::toROSMsg(pcl_output, output);
  output.header = cloud_in_ptr->header;
  return output;
}
std::vector<PointCgal> PolgonRemoverComponent::polygon_geometry_to_cgal( const geometry_msgs::msg::Polygon::ConstSharedPtr & polygon_in)
{
  std::vector<PointCgal> polyline_polygon;
  if (polygon_in->points.size() < 3) {
      throw std::length_error("Polygon vertex count should be larger than 2.");
  }
  const auto & vertices_in = polygon_in->points;
  polyline_polygon.resize(vertices_in.size());
  std::transform(
      polygon_in->points.begin(),
      polygon_in->points.end(),
      polyline_polygon.begin(),
      [](const geometry_msgs::msg::Point32 & p_in) {
      return PointCgal(p_in.x, p_in.y);
      });
  return polyline_polygon;
}
}  // namespace pointcloud_preprocessor

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(pointcloud_preprocessor::PolgonRemoverComponent)
