// Copyright 2020 Tier IV, Inc. All rights reserved.
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

#include <lanelet2_extension/utility/query.hpp>
#include <lanelet2_extension/utility/utilities.hpp>
#include <scene_module/occlusion_spot/geometry.hpp>
#include <scene_module/occlusion_spot/occlusion_spot_utils.hpp>
#include <scene_module/occlusion_spot/risk_predictive_braking.hpp>
#include <scene_module/occlusion_spot/scene_occlusion_spot.hpp>
#include <utilization/boost_geometry_helper.hpp>
#include <utilization/path_utilization.hpp>
#include <utilization/util.hpp>

#include <lanelet2_core/primitives/BasicRegulatoryElements.h>

#include <memory>
#include <vector>

namespace behavior_velocity_planner
{
namespace bg = boost::geometry;
namespace lg = lanelet::geometry;
namespace utils = occlusion_spot_utils;

OcclusionSpotModule::OcclusionSpotModule(
  const int64_t module_id, [[maybe_unused]] std::shared_ptr<const PlannerData> planner_data,
  const PlannerParam & planner_param, const rclcpp::Logger logger,
  const rclcpp::Clock::SharedPtr clock,
  const rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr publisher)
: SceneModuleInterface(module_id, logger, clock), publisher_(publisher)
{
  param_ = planner_param;
  debug_data_.detection_type = "occupancy";
  const lanelet::LaneletMapConstPtr & ll = planner_data->route_handler_->getLaneletMapPtr();
  const lanelet::ConstLineStrings3d partitions = lanelet::utils::query::getAllPartitions(ll);
  for (const auto & partition : partitions) {
    lanelet::BasicLineString2d line;
    for (const auto & p : partition) {
      line.emplace_back(lanelet::BasicPoint2d{p.x(), p.y()});
    }
    debug_data_.partition_lanelets.emplace_back(lanelet::BasicPolygon2d(line));
  }
}

bool OcclusionSpotModule::modifyPathVelocity(
  autoware_auto_planning_msgs::msg::PathWithLaneId * path,
  [[maybe_unused]] tier4_planning_msgs::msg::StopReason * stop_reason)
{
  debug_data_.resetData();
  if (path->points.size() < 2) {
    return true;
  }
  // set planner data
  {
    param_.v.max_stop_jerk = planner_data_->max_stop_jerk_threshold;
    param_.v.max_stop_accel = planner_data_->max_stop_acceleration_threshold;
    param_.v.v_ego = planner_data_->current_velocity->twist.linear.x;
    param_.v.a_ego = planner_data_->current_accel.get();
    param_.detection_area_max_length = planning_utils::calcJudgeLineDistWithJerkLimit(
      param_.v.v_ego, param_.v.a_ego, param_.v.non_effective_accel, param_.v.non_effective_jerk,
      0.0);
  }
  const geometry_msgs::msg::Pose ego_pose = planner_data_->current_pose.pose;
  const auto & occ_grid_ptr = planner_data_->occupancy_grid;
  if (!occ_grid_ptr) {
    return true;
  }

  const double max_range = 50.0;
  PathWithLaneId clipped_path;
  utils::clipPathByLength(*path, clipped_path, max_range);
  PathWithLaneId interp_path;
  //! never change this interpolation interval(will affect module accuracy)
  splineInterpolate(clipped_path, 1.0, &interp_path, logger_);
  debug_data_.interp_path = interp_path;
  int closest_idx = -1;
  if (!planning_utils::calcClosestIndex<PathWithLaneId>(
        interp_path, ego_pose, closest_idx, param_.dist_thr, param_.angle_thr)) {
    return true;
  }
  // return if ego is final point of interpolated path
  if (closest_idx == static_cast<int>(interp_path.points.size()) - 1) return true;
  nav_msgs::msg::OccupancyGrid occ_grid = *occ_grid_ptr;
  grid_map::GridMap grid_map;
  grid_utils::denoiseOccupancyGridCV(occ_grid, grid_map, param_.grid);
  double offset_from_start_to_ego = utils::offsetFromStartToEgo(interp_path, ego_pose, closest_idx);
  using Slice = occlusion_spot_utils::Slice;
  std::vector<Slice> detection_area_polygons;
  utils::buildDetectionAreaPolygon(
    detection_area_polygons, interp_path, offset_from_start_to_ego, param_);
  std::vector<utils::PossibleCollisionInfo> possible_collisions;
  // Note: Don't consider offset from path start to ego here
  utils::createPossibleCollisionsInDetectionArea(
    detection_area_polygons, possible_collisions, grid_map, interp_path, offset_from_start_to_ego,
    param_, debug_data_.occlusion_points);
  if (detection_area_polygons.empty()) return true;
  RCLCPP_DEBUG_STREAM_THROTTLE(
    logger_, *clock_, 3000, "num possible collision:" << possible_collisions.size());
  utils::calcSlowDownPointsForPossibleCollision(0, interp_path, 0.0, possible_collisions);
  // Note: Consider offset from path start to ego here
  utils::handleCollisionOffset(possible_collisions, offset_from_start_to_ego);
  // apply safe velocity using ebs and pbs deceleration
  utils::applySafeVelocityConsideringPossibleCollision(path, possible_collisions, param_);
  // these debug topics needs computation resource
  if (param_.debug) {
    publisher_->publish(occ_grid);
    for (const auto & p : detection_area_polygons) {
      debug_data_.detection_areas.emplace_back(p.polygon);
    }
  } else {
    debug_data_.occlusion_points.clear();
  }
  debug_data_.z = path->points.front().point.pose.position.z;
  debug_data_.possible_collisions = possible_collisions;
  debug_data_.path_raw = *path;
  return true;
}

}  // namespace behavior_velocity_planner
