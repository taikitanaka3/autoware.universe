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

#include "tier4_autoware_utils/ros/update_param.hpp"

#include <occlusion_spot_generator/grid_utils.hpp>

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

rcl_interfaces::msg::SetParametersResult OcclusionSpotGeneratorNode::paramCallback(
  const std::vector<rclcpp::Parameter> & parameters)
{
  using tier4_autoware_utils::updateParam;

  {  // option parameter
    auto & op = occlusion_param_;
    updateParam<int>(parameters, "grid.free_space_max", op.free_space_max);
    updateParam<int>(parameters, "grid.occupied_min", op.occupied_min);
    updateParam<double>(
      parameters, "grid.occlusion_size_of_pedestrian", op.occlusion_size_of_pedestrian);
    updateParam<double>(parameters, "grid.occlusion_size_of_bicycle", op.occlusion_size_of_bicycle);
    updateParam<double>(parameters, "grid.occlusion_size_of_car", op.occlusion_size_of_car);
  }
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";
  return result;
}

OcclusionSpotGeneratorNode::OcclusionSpotGeneratorNode(const rclcpp::NodeOptions & node_options)
: Node("occlusion_spot_generator_node", node_options),
  tf_buffer_(this->get_clock()),
  tf_listener_(tf_buffer_)
{
  using std::placeholders::_1;
  using std::chrono_literals::operator""ms;

  // get parameters
  auto & op = occlusion_param_;
  {
    op.occupancy_grid_resolusion = declare_parameter<double>("grid.occupancy_grid_resolusion");
    op.free_space_max = declare_parameter<int>("grid.free_space_max");
    op.occupied_min = declare_parameter<int>("grid.occupied_min");
    op.occlusion_size_of_pedestrian =
      declare_parameter<double>("grid.occlusion_size_of_pedestrian");
    op.occlusion_size_of_bicycle = declare_parameter<double>("grid.occlusion_size_of_bicycle");
    op.occlusion_size_of_car = declare_parameter<double>("grid.occlusion_size_of_car");
  }
  // Trigger Subscriber
  trigger_sub_path_with_lane_id_ =
    this->create_subscription<autoware_auto_planning_msgs::msg::PathWithLaneId>(
      "~/input/path_with_lane_id", 1,
      std::bind(&OcclusionSpotGeneratorNode::onPathWithLaneId, this, _1));
  // Subscribers
  sub_predicted_objects_ = this->create_subscription<PredictedObjects>(
    "~/input/dynamic_objects", 1, [this](const PredictedObjects::ConstSharedPtr msg) {
      perception_data_.predicted_objects = msg;
    });
  sub_vehicle_odometry_ = create_subscription<Odometry>(
    "~/input/vehicle_odometry", 1, [this](const Odometry::SharedPtr msg) {
      auto current_velocity = std::make_shared<geometry_msgs::msg::TwistStamped>();
      current_velocity->header = msg->header;
      current_velocity->twist = msg->twist.twist;
      perception_data_.current_velocity = current_velocity;
    });
  sub_occupancy_grid_ = this->create_subscription<OccupancyGrid>(
    "~/input/occupancy_grid", 1,
    [this](const OccupancyGrid::ConstSharedPtr msg) { perception_data_.occupancy_grid = msg; });

  // set parameter callback
  param_callback_ = this->add_on_set_parameters_callback(
    std::bind(&OcclusionSpotGeneratorNode::paramCallback, this, std::placeholders::_1));

  timer_ = rclcpp::create_timer(
    this, get_clock(), 500ms, std::bind(&OcclusionSpotGeneratorNode::onTimer, this));
}

void OcclusionSpotGeneratorNode::onTimer()
{
  try {
    perception_data_.current_pose =
      transform2pose(tf_buffer_.lookupTransform("map", "base_link", tf2::TimePointZero));
  } catch (tf2::TransformException & e) {
    RCLCPP_INFO(get_logger(), "waiting for transform from `map` to `base_link`");
    rclcpp::sleep_for(std::chrono::milliseconds(5000));
    return;
  }
  if (!isDataReady(perception_data_)) {
    return;
  }
  generateOcclusionSpot();
}

void OcclusionSpotGeneratorNode::generateOcclusionSpot()
{
  const auto & ego_pose = perception_data_.current_pose.pose;
  const auto & obj = perception_data_.predicted_objects;
  const bool use_object_footprints = true;
  const bool use_object_ray_casts = true;
  const auto & occ_grid_ptr = perception_data_.occupancy_grid;
  grid_map::GridMap grid_map;
  OccupancyGrid occupancy_grid = *occ_grid_ptr;
  Polygons2d stuck_vehicle_foot_prints = {};
  Polygons2d moving_vehicle_foot_prints = {};
  {
    const auto vehicles = occlusion_spot_generator::extractVehicles(obj, ego_pose.position, 100);
    occlusion_spot_generator::vehiclesToFootprint(
      vehicles, stuck_vehicle_foot_prints, moving_vehicle_foot_prints, 1.0);
  }
  grid_utils::GridParam grid_param = {
    occlusion_param_.free_space_max, occlusion_param_.occupied_min};
  const bool is_show_debug_window = true;
  cv::Mat border_image(
    occupancy_grid.info.width, occupancy_grid.info.height, CV_8UC1,
    cv::Scalar(grid_utils::occlusion_cost_value::FREE_SPACE));
  cv::Mat occlusion_image(
    occupancy_grid.info.width, occupancy_grid.info.height, CV_8UC1,
    cv::Scalar(grid_utils::occlusion_cost_value::FREE_SPACE));
  grid_utils::toQuantizedImage(occupancy_grid, &border_image, &occlusion_image, grid_param);

  //! show original occupancy grid to compare difference
  if (is_show_debug_window) {
    cv::namedWindow("occlusion_image", cv::WINDOW_NORMAL);
    cv::imshow("occlusion_image", occlusion_image);
    cv::moveWindow("occlusion_image", 0, 0);
  }

  //! raycast object shadow using vehicle
  if (use_object_footprints || use_object_ray_casts) {
    // generateOccupiedImage(
    //   occupancy_grid, border_image, stuck_vehicle_foot_prints, moving_vehicle_foot_prints,
    //   use_object_footprints, use_object_ray_casts);
    if (is_show_debug_window) {
      cv::namedWindow("object ray shadow", cv::WINDOW_NORMAL);
      cv::imshow("object ray shadow", border_image);
      cv::moveWindow("object ray shadow", 300, 0);
    }
  }

  //!< @brief erode occlusion to make sure occlusion candidates are big enough
  auto & op = occlusion_param_;
  const double res = op.occupancy_grid_resolusion;
  const int num_iter_for_pedestrian = static_cast<int>(op.occlusion_size_of_pedestrian / res) - 1;
  const int num_iter_for_bicycle = static_cast<int>(op.occlusion_size_of_bicycle / res) - 1;
  const int num_iter_for_car = static_cast<int>(op.occlusion_size_of_car / res) - 1;
  cv::Mat kernel(2, 2, CV_8UC1, cv::Scalar(1));
  cv::erode(occlusion_image, occlusion_image, kernel, cv::Point(-1, -1), num_iter_for_pedestrian);
  if (is_show_debug_window) {
    cv::namedWindow("morph", cv::WINDOW_NORMAL);
    cv::imshow("morph", occlusion_image);
    cv::moveWindow("morph", 0, 300);
  }
  {
    cv::Mat bicycle_image(
      occupancy_grid.info.width, occupancy_grid.info.height, CV_8UC1,
      cv::Scalar(grid_utils::occlusion_cost_value::FREE_SPACE));
    cv::Mat car_image(
      occupancy_grid.info.width, occupancy_grid.info.height, CV_8UC1,
      cv::Scalar(grid_utils::occlusion_cost_value::FREE_SPACE));
    int filter_size = num_iter_for_bicycle - num_iter_for_pedestrian;
    cv::erode(occlusion_image, bicycle_image, kernel, cv::Point(-1, -1), filter_size);
    if (is_show_debug_window) {
      cv::namedWindow("bicycle_image", cv::WINDOW_NORMAL);
      cv::imshow("bicycle_image", bicycle_image);
      cv::moveWindow("bicycle_image", 0, 600);
    }
    filter_size = num_iter_for_car - num_iter_for_bicycle;
    cv::erode(bicycle_image, car_image, kernel, cv::Point(-1, -1), filter_size);
    if (is_show_debug_window) {
      cv::namedWindow("car_image", cv::WINDOW_NORMAL);
      cv::imshow("car_image", car_image);
      cv::moveWindow("car_image", 0, 900);
    }
  }

  border_image += occlusion_image;
  if (is_show_debug_window) {
    cv::namedWindow("merge", cv::WINDOW_NORMAL);
    cv::imshow("merge", border_image);
    cv::moveWindow("merge", 300, 300);
    cv::waitKey(1);
  }
  grid_utils::imageToOccupancyGrid(border_image, &occupancy_grid);
  grid_map::GridMapRosConverter::fromOccupancyGrid(occupancy_grid, "layer", grid_map);
}

bool OcclusionSpotGeneratorNode::isDataReady(const PerceptionData planner_data) const
{
  const auto & d = planner_data;

  if (d.current_pose.header.frame_id == "") {
    return false;
  }
  if (!d.current_velocity) {
    return false;
  }
  if (!d.predicted_objects) {
    return false;
  }
  if (!d.occupancy_grid) {
    return false;
  }
  return true;
}

void OcclusionSpotGeneratorNode::onPathWithLaneId(
  const PathWithLaneId::ConstSharedPtr input_path_msg)
{
  perception_data_.path = input_path_msg;
}

}  // namespace occlusion_spot_generator

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(occlusion_spot_generator::OcclusionSpotGeneratorNode)
