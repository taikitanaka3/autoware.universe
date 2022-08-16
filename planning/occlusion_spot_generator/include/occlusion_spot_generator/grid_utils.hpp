// Copyright 2021 Tier IV, Inc.
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

#ifndef OCCLUSION_SPOT_GENERATOR__GRID_UTILS_HPP_
#define OCCLUSION_SPOT_GENERATOR__GRID_UTILS_HPP_

#include <grid_map_core/GridMap.hpp>
#include <grid_map_core/iterators/LineIterator.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <grid_map_utils/polygon_iterator.hpp>
#include <opencv2/opencv.hpp>
#include <tier4_autoware_utils/geometry/geometry.hpp>
#include <tier4_autoware_utils/math/normalization.hpp>

#include <autoware_auto_perception_msgs/msg/object_classification.hpp>
#include <autoware_auto_perception_msgs/msg/predicted_object.hpp>
#include <autoware_auto_perception_msgs/msg/predicted_objects.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/map_meta_data.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

#include <boost/assign/list_of.hpp>
#include <boost/geometry.hpp>
#include <boost/geometry/algorithms/area.hpp>
#include <boost/geometry/algorithms/disjoint.hpp>
#include <boost/geometry/algorithms/distance.hpp>
#include <boost/geometry/algorithms/equals.hpp>
#include <boost/geometry/algorithms/intersection.hpp>
#include <boost/geometry/geometries/linestring.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/geometries/register/point.hpp>
#include <boost/geometry/geometries/segment.hpp>

#include <lanelet2_core/primitives/Polygon.h>
#include <tf2/utils.h>

#include <algorithm>
#include <vector>

#ifdef ROS_DISTRO_GALACTIC
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#endif

// cppcheck-suppress unknownMacro
BOOST_GEOMETRY_REGISTER_POINT_3D(geometry_msgs::msg::Point, double, cs::cartesian, x, y, z)

using geometry_msgs::msg::Pose;
using geometry_msgs::msg::PoseStamped;
using Point2d = boost::geometry::model::d2::point_xy<double>;
using Polygon2d = boost::geometry::model::polygon<Point2d, false, false>;
using Polygons2d = std::vector<Polygon2d>;

namespace grid_utils
{
using autoware_auto_perception_msgs::msg::PredictedObject;
using autoware_auto_perception_msgs::msg::PredictedObjects;
using geometry_msgs::msg::Point;
using geometry_msgs::msg::TransformStamped;
using nav_msgs::msg::MapMetaData;
using nav_msgs::msg::OccupancyGrid;
namespace occlusion_cost_value
{
static constexpr unsigned char FREE_SPACE = 0;
static constexpr unsigned char UNKNOWN = 50;
static constexpr unsigned char OCCUPIED = 100;
static constexpr unsigned char UNKNOWN_IMAGE = 100;
static constexpr unsigned char OCCUPIED_IMAGE = 255;
}  // namespace occlusion_cost_value

struct PolarCoordinates
{
  double radius;
  double theta;
};

inline PolarCoordinates toPolarCoordinates(const Point2d & origin, const Point2d & point)
{
  const double dy = point.y() - origin.y();
  const double dx = point.x() - origin.x();
  const double radius = std::hypot(dy, dx);
  const double theta = std::atan2(dy, dx);
  return {radius, theta};
}

struct GridParam
{
  int free_space_max;  // maximum value of a freespace cell in the occupancy grid
  int occupied_min;    // minimum value of an occupied cell in the occupancy grid
};

void imageToOccupancyGrid(const cv::Mat & cv_image, nav_msgs::msg::OccupancyGrid * occupancy_grid);
void toQuantizedImage(
  const nav_msgs::msg::OccupancyGrid & occupancy_grid, cv::Mat * border_image,
  cv::Mat * occlusion_image, const GridParam & param);
void denoiseOccupancyGridCV(
  const OccupancyGrid::ConstSharedPtr occupancy_grid_ptr, grid_map::GridMap & grid_map,
  const GridParam & param, const bool is_show_debug_window, const int num_iter,
  const bool use_object_footprints, const bool use_object_ray_casts);
void addObjectsToGridMap(const std::vector<PredictedObject> & objs, grid_map::GridMap & grid);
}  // namespace grid_utils

#endif  // OCCLUSION_SPOT_GENERATOR__GRID_UTILS_HPP_
