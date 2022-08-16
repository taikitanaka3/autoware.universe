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

#ifndef OCCLUSION_SPOT_GENERATOR__OCCLUSION_SPOT_GENERATOR_HPP_
#define OCCLUSION_SPOT_GENERATOR__OCCLUSION_SPOT_GENERATOR_HPP_

#include <autoware_auto_perception_msgs/msg/object_classification.hpp>
#include <autoware_auto_perception_msgs/msg/predicted_object.hpp>
#include <autoware_auto_perception_msgs/msg/predicted_objects.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>

// boost
#include <boost/assign/list_of.hpp>
#include <boost/geometry.hpp>
#include <boost/geometry/algorithms/intersection.hpp>
#include <boost/geometry/geometries/linestring.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/geometries/register/point.hpp>
#include <boost/geometry/geometries/segment.hpp>
#include <boost/geometry/strategies/strategies.hpp>

// tf
#include <tf2/utils.h>
#ifdef ROS_DISTRO_GALACTIC
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#endif

#include <boost/optional.hpp>

#include <algorithm>
#include <chrono>
#include <string>
#include <utility>
#include <vector>

using geometry_msgs::msg::Pose;
using Point2d = boost::geometry::model::d2::point_xy<double>;
using Polygon2d = boost::geometry::model::polygon<Point2d, false, false>;
using Polygons2d = std::vector<Polygon2d>;

namespace occlusion_spot_generator
{
using autoware_auto_perception_msgs::msg::ObjectClassification;
using autoware_auto_perception_msgs::msg::PredictedObject;
using autoware_auto_perception_msgs::msg::PredictedObjects;
using autoware_auto_perception_msgs::msg::Shape;
using geometry_msgs::msg::Point;
using geometry_msgs::msg::Pose;

bool isStuckVehicle(const PredictedObject & obj, const double min_vel);
bool isMovingVehicle(const PredictedObject & obj, const double min_vel);
std::vector<PredictedObject> extractVehicles(
  const PredictedObjects::ConstSharedPtr objects_ptr, const Point ego_position,
  const double distance);
std::vector<PredictedObject> filterVehiclesByDetectionArea(
  const std::vector<PredictedObject> & objs, const Polygons2d & polys);
bool isVehicle(const ObjectClassification & obj_class);
void categorizeVehicles(
  const std::vector<PredictedObject> & vehicles, Polygons2d & stuck_vehicle_foot_prints,
  Polygons2d & moving_vehicle_foot_prints, const double stuck_vehicle_vel);

}  // namespace occlusion_spot_generator

#endif  // OCCLUSION_SPOT_GENERATOR__OCCLUSION_SPOT_GENERATOR_HPP_
