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

#include "occlusion_spot_generator/occlusion_spot_generator.hpp"

#include <string>

namespace occlusion_spot_generator
{
OcclusionSpotGenerator::OcclusionSpotGenerator(rclcpp::Node & node), logger_{node.get_logger()},
  clock_{node.get_clock()}
{
}

void OcclusionSpotGenerator::generateOcclusionSpot()
{
  const geometry_msgs::msg::Pose ego_pose = planner_data_->current_pose.pose;
  PathWithLaneId clipped_path;
  utils::clipPathByLength(*path, clipped_path, param_.detection_area_length);
  PathWithLaneId interp_path;
  const auto & occ_grid_ptr = planner_data_->occupancy_grid;
  if (!occ_grid_ptr) return;  // mo data
  grid_map::GridMap grid_map;
  grid_utils::denoiseOccupancyGridCV(
    occ_grid_ptr, grid_map, param_.grid, param_.is_show_cv_window, param_.filter_occupancy_grid);
}

}  // namespace occlusion_spot_generator
