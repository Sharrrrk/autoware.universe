// Copyright 2022 The Autoware Contributors
// SPDX-License-Identifier: Apache-2.0

#ifndef POINTCLOUD_PREPROCESSOR__CROP_BOX_FILTER__CROP_BOX_FILTER_NODELET_HPP_
#define POINTCLOUD_PREPROCESSOR__CROP_BOX_FILTER__CROP_BOX_FILTER_NODELET_HPP_

#include "pointcloud_preprocessor/filter.hpp"
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Polygon_2_algorithms.h>

using K = CGAL::Exact_predicates_inexact_constructions_kernel;
using PointCgal = K::Point_2;

namespace pointcloud_preprocessor
{
class PolgonRemoverComponent : public pointcloud_preprocessor::Filter
{
protected:
  virtual void filter(const PointCloud2ConstPtr & input, const IndicesPtr & indices, PointCloud2 & output);

  void publishRemovedPolygon();

  void update_polygon(const geometry_msgs::msg::Polygon::ConstSharedPtr & polygon_in);
  static std::vector<PointCgal> polygon_geometry_to_cgal(
          const geometry_msgs::msg::Polygon::ConstSharedPtr & polygon_in);
  PointCloud2 remove_updated_polygon_from_cloud(const PointCloud2ConstPtr & cloud_in);
  PointCloud2 remove_polygon_cgal_from_cloud(
    const PointCloud2::ConstSharedPtr & cloud_in_ptr,
    const std::vector<PointCgal> & polyline_polygon);

private:
  rclcpp::Parameter param;
  std::vector<double> polygon_vertices_;
  geometry_msgs::msg::Polygon::SharedPtr polygon_;

  bool polygon_is_initialized_;
  bool will_visualize_;
  std::vector<PointCgal> polygon_cgal_;
  visualization_msgs::msg::Marker marker_;

  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_marker_ptr_;

public:
  PCL_MAKE_ALIGNED_OPERATOR_NEW
  explicit PolgonRemoverComponent(const rclcpp::NodeOptions & options);
};
}  // namespace pointcloud_preprocessor

#endif  // POINTCLOUD_PREPROCESSOR__CROP_BOX_FILTER__POLGON_REMOVER_NODELET_HPP_
