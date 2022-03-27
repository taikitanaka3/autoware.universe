// Copyright 2015-2021 Autoware Foundation
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

#ifndef OCCLUSION_SPOT_GENERATOR__OCCLUSION_SPOT_GENERATOR_NODE_HPP_
#define OCCLUSION_SPOT_GENERATOR__OCCLUSION_SPOT_GENERATOR_NODE_HPP_

#include "occlusion_spot_generator/occlusion_spot_generator.hpp"
#include "occlusion_spot_generator/planner_data.hpp"

#include <rclcpp/rclcpp.hpp>

#include <autoware_auto_mapping_msgs/msg/had_map_bin.hpp>
#include <autoware_auto_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_auto_planning_msgs/msg/path.hpp>
#include <autoware_auto_planning_msgs/msg/path_with_lane_id.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <memory>
#include <mutex>
#include <string>

namespace occlusion_spot_generator
{
using autoware_auto_perception_msgs::msg::PredictedObjects;
using autoware_auto_planning_msgs::msg::PathWithLaneId;
using geometry_msgs::msg::PoseStamped;
using geometry_msgs::msg::TwistStamped;
using nav_msgs::msg::OccupancyGrid;
using nav_msgs::msg::Odometry;

/// This is a convenience class for saving you from declaring all parameters
/// manually and calculating derived parameters.
/// This class supposes that necessary parameters are set when the node is launched.
class OcclusionSpotGeneratorNode : public rclcpp::Node
{
public:
  /// Constructor
  explicit OcclusionSpotGeneratorNode(const rclcpp::NodeOptions & node_options);

private:
  // tf
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  // subscriber
  rclcpp::Subscription<PathWithLaneId>::SharedPtr trigger_sub_path_with_lane_id_;
  rclcpp::Subscription<PredictedObjects>::SharedPtr sub_predicted_objects_;

  rclcpp::Subscription<Odometry>::SharedPtr sub_vehicle_odometry_;
  rclcpp::Subscription<OccupancyGrid>::SharedPtr sub_occupancy_grid_;
  void onPathWithLaneId(const PathWithLaneId::ConstSharedPtr input_path_msg);
  void onPredictedObjects(const PredictedObjects::ConstSharedPtr msg);
  void onVehicleVelocity(const Odometry::ConstSharedPtr msg);
  void onOccupancyGrid(const OccupancyGrid::ConstSharedPtr msg);
  bool isDataReady(const PlannerData planner_data) const;
  void onTimer();

  std::unique_ptr<OcclusionSpotGenerator> occlusion_spot_generator_;
  // mutex for planner_data_
  std::mutex mutex_;
  PlannerData planner_data_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace occlusion_spot_generator

#endif  // OCCLUSION_SPOT_GENERATOR__OCCLUSION_SPOT_GENERATOR_NODE_HPP_
