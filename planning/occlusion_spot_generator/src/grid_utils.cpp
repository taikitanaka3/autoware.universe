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

#include <occlusion_spot_generator/grid_utils.hpp>

#include <algorithm>
#include <stdexcept>
#include <vector>

namespace grid_utils
{
void imageToOccupancyGrid(const cv::Mat & cv_image, nav_msgs::msg::OccupancyGrid * occupancy_grid)
{
  const int width = cv_image.cols;
  const int height = cv_image.rows;
  occupancy_grid->data.clear();
  occupancy_grid->data.resize(width * height);
  for (int x = width - 1; x >= 0; x--) {
    for (int y = height - 1; y >= 0; y--) {
      const int idx = (height - 1 - y) + (width - 1 - x) * height;
      unsigned char intensity = cv_image.at<unsigned char>(y, x);
      if (intensity == grid_utils::occlusion_cost_value::FREE_SPACE) {
        intensity = grid_utils::occlusion_cost_value::FREE_SPACE;
      } else if (intensity == grid_utils::occlusion_cost_value::UNKNOWN_IMAGE) {
        intensity = grid_utils::occlusion_cost_value::UNKNOWN;
      } else if (intensity == grid_utils::occlusion_cost_value::OCCUPIED_IMAGE) {
        intensity = grid_utils::occlusion_cost_value::OCCUPIED;
      } else {
        throw std::logic_error("behavior_velocity[occlusion_spot_grid]: invalid if clause");
      }
      occupancy_grid->data.at(idx) = intensity;
    }
  }
}
void toQuantizedImage(
  const nav_msgs::msg::OccupancyGrid & occupancy_grid, cv::Mat * border_image,
  cv::Mat * occlusion_image, const GridParam & param)
{
  const int width = border_image->cols;
  const int height = border_image->rows;
  for (int x = width - 1; x >= 0; x--) {
    for (int y = height - 1; y >= 0; y--) {
      const int idx = (height - 1 - y) + (width - 1 - x) * height;
      unsigned char intensity = occupancy_grid.data.at(idx);
      if (intensity <= param.free_space_max) {
        continue;
      } else if (param.free_space_max < intensity && intensity < param.occupied_min) {
        intensity = grid_utils::occlusion_cost_value::UNKNOWN_IMAGE;
        occlusion_image->at<unsigned char>(y, x) = intensity;
      } else if (param.occupied_min <= intensity) {
        intensity = grid_utils::occlusion_cost_value::OCCUPIED_IMAGE;
        border_image->at<unsigned char>(y, x) = intensity;
      } else {
        throw std::logic_error("behavior_velocity[occlusion_spot_grid]: invalid if clause");
      }
    }
  }
}

void denoiseOccupancyGridCV(
  const OccupancyGrid::ConstSharedPtr occupancy_grid_ptr, grid_map::GridMap & grid_map,
  const GridParam & param, const bool is_show_debug_window, const int num_iter,
  const bool use_object_footprints, const bool use_object_ray_casts)
{
  OccupancyGrid occupancy_grid = *occupancy_grid_ptr;
  cv::Mat border_image(
    occupancy_grid.info.width, occupancy_grid.info.height, CV_8UC1,
    cv::Scalar(grid_utils::occlusion_cost_value::FREE_SPACE));
  cv::Mat occlusion_image(
    occupancy_grid.info.width, occupancy_grid.info.height, CV_8UC1,
    cv::Scalar(grid_utils::occlusion_cost_value::FREE_SPACE));
  toQuantizedImage(occupancy_grid, &border_image, &occlusion_image, param);

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
  cv::Mat kernel(2, 2, CV_8UC1, cv::Scalar(1));
  cv::erode(occlusion_image, occlusion_image, kernel, cv::Point(-1, -1), num_iter);
  if (is_show_debug_window) {
    cv::namedWindow("morph", cv::WINDOW_NORMAL);
    cv::imshow("morph", occlusion_image);
    cv::moveWindow("morph", 0, 300);
  }

  border_image += occlusion_image;
  if (is_show_debug_window) {
    cv::namedWindow("merge", cv::WINDOW_NORMAL);
    cv::imshow("merge", border_image);
    cv::moveWindow("merge", 300, 300);
    cv::waitKey(1);
  }
  imageToOccupancyGrid(border_image, &occupancy_grid);
  grid_map::GridMapRosConverter::fromOccupancyGrid(occupancy_grid, "layer", grid_map);
}
}  // namespace grid_utils
