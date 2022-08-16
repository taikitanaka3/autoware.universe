// Copyright 2019 Autoware Foundation
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

#ifndef OCCLUSION_SPOT_GENERATOR__PLANNER_DATA_HPP_
#define OCCLUSION_SPOT_GENERATOR__PLANNER_DATA_HPP_

#include <autoware_auto_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_auto_planning_msgs/msg/path_with_lane_id.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/header.hpp>

#include <boost/optional.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_routing/RoutingGraphContainer.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>
#include <tf2_ros/transform_listener.h>

#include <algorithm>
#include <deque>
#include <map>
#include <memory>
#include <vector>

namespace occlusion_spot_generator
{
using autoware_auto_perception_msgs::msg::PredictedObjects;
using autoware_auto_planning_msgs::msg::PathPointWithLaneId;
using autoware_auto_planning_msgs::msg::PathWithLaneId;
using geometry_msgs::msg::PoseStamped;
using geometry_msgs::msg::TwistStamped;
using nav_msgs::msg::OccupancyGrid;

class OcclusionSpotGeneratorNode;
struct PerceptionData
{
  // tf
  PoseStamped current_pose;
  // msgs from callbacks that are used for data-ready
  TwistStamped::ConstSharedPtr current_velocity;
  PredictedObjects::ConstSharedPtr predicted_objects;
  // occupancy grid
  OccupancyGrid::ConstSharedPtr occupancy_grid;
  // path
  PathWithLaneId::ConstSharedPtr path;

  friend OcclusionSpotGeneratorNode;
};

}  // namespace occlusion_spot_generator

#endif  // OCCLUSION_SPOT_GENERATOR__PLANNER_DATA_HPP_
