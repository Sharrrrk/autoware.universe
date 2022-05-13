// Copyright 2020 Tier IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/*
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id: polgonremover.cpp
 *
 */

#ifndef POINTCLOUD_PREPROCESSOR__CROP_BOX_FILTER__CROP_BOX_FILTER_NODELET_HPP_
#define POINTCLOUD_PREPROCESSOR__CROP_BOX_FILTER__CROP_BOX_FILTER_NODELET_HPP_

#include "pointcloud_preprocessor/filter.hpp"
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <vector>

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
