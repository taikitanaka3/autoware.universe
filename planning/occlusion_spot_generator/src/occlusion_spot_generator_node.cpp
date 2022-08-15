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

#include "occlusion_spot_generator/occlusion_spot_generator_node.hpp"

#include <functional>
#include <memory>
#include <string>

namespace
{
template <class T>
T getParameter(rclcpp::Node & node, const std::string & name)
{
  if (node.has_parameter(name)) {
    return node.get_parameter(name).get_value<T>();
  }

  try {
    return node.declare_parameter<T>(name);
  } catch (const rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(
      node.get_logger(), "Failed to get parameter `%s`, please set it when you launch the node.",
      name.c_str());
    throw(ex);
  }
}
}  // namespace

namespace occlusion_spot_generator
{
geometry_msgs::msg::PoseStamped transform2pose(
  const geometry_msgs::msg::TransformStamped & transform)
{
  geometry_msgs::msg::PoseStamped pose;
  pose.header = transform.header;
  pose.pose.position.x = transform.transform.translation.x;
  pose.pose.position.y = transform.transform.translation.y;
  pose.pose.position.z = transform.transform.translation.z;
  pose.pose.orientation = transform.transform.rotation;
  return pose;
}

OcclusionSpotGeneratorNode::OcclusionSpotGeneratorNode(const rclcpp::NodeOptions & node_options)
: Node("occlusion_spot_generator_node", node_options),
  tf_buffer_(this->get_clock()),
  tf_listener_(tf_buffer_)
{
  using std::placeholders::_1;
  using std::chrono_literals::operator""ms;
  // Trigger Subscriber
  trigger_sub_path_with_lane_id_ =
    this->create_subscription<autoware_auto_planning_msgs::msg::PathWithLaneId>(
      "~/input/path_with_lane_id", 1,
      std::bind(&OcclusionSpotGeneratorNode::onPathWithLaneId, this, _1),
      createSubscriptionOptions(this));

  // Subscribers
  sub_predicted_objects_ = this->create_subscription<PredictedObjects>(
    "~/input/dynamic_objects", 1,
    std::bind(&OcclusionSpotGeneratorNode::onPredictedObjects, this, _1),
    createSubscriptionOptions(this));
  sub_vehicle_odometry_ = this->create_subscription<Odometry>(
    "~/input/vehicle_odometry", 1,
    std::bind(&OcclusionSpotGeneratorNode::onVehicleVelocity, this, _1),
    createSubscriptionOptions(this));
  sub_occupancy_grid_ = this->create_subscription<OccupancyGrid>(
    "~/input/occupancy_grid", 1, std::bind(&OcclusionSpotGeneratorNode::onOccupancyGrid, this, _1),
    createSubscriptionOptions(this));
  timer_ = rclcpp::create_timer(
    this, get_clock(), 500ms, std::bind(&OcclusionSpotGeneratorNode::onTimer, this));
  occlusion_spot_generator_ = std::make_unique<OcclusionSpotGenerator>(*this);
}

void OcclusionSpotGeneratorNode::onTimer()
{
  mutex_.lock();  // for planner_data_
  // Check ready
  try {
    planner_data_.current_pose =
      transform2pose(tf_buffer_.lookupTransform("map", "base_link", tf2::TimePointZero));
  } catch (tf2::TransformException & e) {
    RCLCPP_INFO(get_logger(), "waiting for transform from `map` to `base_link`");
    rclcpp::sleep_for(std::chrono::milliseconds(5000));
    return;
  }
  if (!isDataReady(planner_data_)) {
    return;
  }
  // NOTE: planner_data must not be referenced for multithreading
  const auto planner_data = planner_data_;
  occlusion_spot_generator_->generateOcclusionSpot();
}

bool OcclusionSpotGeneratorNode::isDataReady(const PlannerData planner_data) const
{
  const auto & d = planner_data;

  // from tf
  if (d.current_pose.header.frame_id == "") {
    return false;
  }

  // from callbacks
  if (!d.current_velocity) {
    return false;
  }
  if (!d.predicted_objects) {
    return false;
  }
  if (!d.occupancy_grid) {
    return false;
  }
  if (!d.path) {
    return false;
  }
  return true;
}

void OcclusionSpotGeneratorNode::onPathWithLaneId(
  const PathWithLaneId::ConstSharedPtr input_path_msg)
{
  planner_data_.path = input_path_msg;
}

void OcclusionSpotGeneratorNode::onOccupancyGrid(
  const nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg)
{
  planner_data_.occupancy_grid = msg;
}

void OcclusionSpotGeneratorNode::onPredictedObjects(
  const autoware_auto_perception_msgs::msg::PredictedObjects::ConstSharedPtr msg)
{
  planner_data_.predicted_objects = msg;
}

void OcclusionSpotGeneratorNode::onVehicleVelocity(
  const nav_msgs::msg::Odometry::ConstSharedPtr msg)
{
  auto current_velocity = std::make_shared<geometry_msgs::msg::TwistStamped>();
  current_velocity->header = msg->header;
  current_velocity->twist = msg->twist.twist;
  planner_data_.current_velocity = current_velocity;
}

}  // namespace occlusion_spot_generator

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(occlusion_spot_generator::OcclusionSpotGeneratorNode)
