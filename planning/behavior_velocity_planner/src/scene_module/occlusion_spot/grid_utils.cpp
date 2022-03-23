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

#include <scene_module/occlusion_spot/grid_utils.hpp>

#include <algorithm>
#include <stdexcept>
#include <vector>

namespace behavior_velocity_planner
{
namespace grid_utils
{

Polygon2d pointsToPoly(const Point2d p0, const Point2d p1, const double radius)
{
  LineString2d line = {p0, p1};
  const double angle = atan2(p0.y() - p1.y(), p0.x() - p1.x());
  const double r = radius;
  Polygon2d line_poly;
  // add polygon counter clockwise
  line_poly.outer().emplace_back(p0.x() + r * sin(angle), p0.y() - r * cos(angle));
  line_poly.outer().emplace_back(p1.x() + r * sin(angle), p1.y() - r * cos(angle));
  line_poly.outer().emplace_back(p1.x() - r * sin(angle), p1.y() + r * cos(angle));
  line_poly.outer().emplace_back(p0.x() - r * sin(angle), p0.y() + r * cos(angle));
  // std::cout << boost::geometry::wkt(line_poly) << std::endl;
  // std::cout << boost::geometry::wkt(line) << std::endl;
  return line_poly;
}

void addObjectsToGridMap(const PredictedObjects & objs, grid_map::GridMap & grid)
{
  auto & grid_data = grid["layer"];
  for (const auto & obj : objs.objects) {
    Polygon2d foot_print_polygon = planning_utils::toFootprintPolygon(obj);
    grid_map::Polygon grid_polygon;
    const auto & pos = obj.kinematics.initial_pose_with_covariance.pose.position;
    if (grid.isInside(grid_map::Position(pos.x, pos.y))) continue;
    try {
      for (const auto & point : foot_print_polygon.outer()) {
        grid_polygon.addVertex({point.x(), point.y()});
      }
      for (grid_map::PolygonIterator iterator(grid, grid_polygon); !iterator.isPastEnd();
           ++iterator) {
        const grid_map::Index & index = *iterator;
        if (!grid.isValid(index)) continue;
        grid_data(index.x(), index.y()) = grid_utils::occlusion_cost_value::OCCUPIED;
      }
    } catch (const std::invalid_argument & e) {
      std::cerr << e.what() << std::endl;
    }
  }
}

bool isOcclusionSpotSquare(
  OcclusionSpotSquare & occlusion_spot, const grid_map::Matrix & grid_data,
  const grid_map::Index & cell, int min_occlusion_size, const grid_map::Size & grid_size)
{
  const int offset = (min_occlusion_size != 1) ? (min_occlusion_size - 1) : min_occlusion_size;
  const int cell_max_x = grid_size.x() - 1;
  const int cell_max_y = grid_size.y() - 1;
  // Calculate ranges to check
  int min_x = cell.x() - offset;
  int max_x = cell.x() + offset;
  int min_y = cell.y() - offset;
  int max_y = cell.y() + offset;
  if (min_x < 0) max_x += std::abs(min_x);
  if (max_x > cell_max_x) min_x -= std::abs(max_x - cell_max_x);
  if (min_y < 0) max_y += std::abs(min_y);
  if (max_y > cell_max_y) min_y -= std::abs(max_y - cell_max_y);
  // No occlusion_spot with size 0
  if (min_occlusion_size == 0) {
    return false;
  }
  /**
   * @brief
   *   (min_x,min_y)...(max_x,min_y)
   *        .               .
   *   (min_x,max_y)...(max_x,max_y)
   */
  // Ensure we stay inside the grid
  min_x = std::max(0, min_x);
  max_x = std::min(cell_max_x, max_x);
  min_y = std::max(0, min_y);
  max_y = std::min(cell_max_y, max_y);
  int not_unknown_count = 0;
  if (grid_data(cell.x(), cell.y()) != grid_utils::occlusion_cost_value::UNKNOWN) {
    return false;
  }
  for (int x = min_x; x <= max_x; ++x) {
    for (int y = min_y; y <= max_y; ++y) {
      // if the value is not unknown value return false
      if (grid_data(x, y) != grid_utils::occlusion_cost_value::UNKNOWN) {
        not_unknown_count++;
      }
      /**
       * @brief case pass o: unknown x: freespace or occupied
       *   oxx oxo oox xxx oxo oxo
       *   oox oxx oox ooo oox oxo ... etc
       *   ooo ooo oox ooo xoo oxo
       */
      if (not_unknown_count > min_occlusion_size + 1) return false;
    }
  }
  occlusion_spot.min_occlusion_size = min_occlusion_size;
  occlusion_spot.index = cell;
  return true;
}

void findOcclusionSpots(
  std::vector<grid_map::Position> & occlusion_spot_positions, const grid_map::GridMap & grid,
  const Polygon2d & polygon, double min_size)
{
  const grid_map::Matrix & grid_data = grid["layer"];
  const int min_occlusion_spot_size = std::max(0.0, std::floor(min_size / grid.getResolution()));
  grid_map::Polygon grid_polygon;
  for (const auto & point : polygon.outer()) {
    grid_polygon.addVertex({point.x(), point.y()});
  }
  for (grid_map::PolygonIterator iterator(grid, grid_polygon); !iterator.isPastEnd(); ++iterator) {
    OcclusionSpotSquare occlusion_spot_square;
    if (isOcclusionSpotSquare(
          occlusion_spot_square, grid_data, *iterator, min_occlusion_spot_size, grid.getSize())) {
      if (!grid.getPosition(occlusion_spot_square.index, occlusion_spot_square.position)) {
        continue;
      }
      std::vector<grid_map::Position> corner_positions;
      getCornerPositions(corner_positions, grid, occlusion_spot_square);
      for (const grid_map::Position & corner : corner_positions) {
        occlusion_spot_positions.emplace_back(corner);
      }
    }
  }
}

bool isCollisionFree(
  const grid_map::GridMap & grid, const grid_map::Position & p1, const grid_map::Position & p2,
  const double radius)
{
  const grid_map::Matrix & grid_data = grid["layer"];
  Point2d occlusion_p = {p1.x(), p1.y()};
  Point2d collision_p = {p2.x(), p2.y()};
  Polygon2d polygon = pointsToPoly(occlusion_p, collision_p, radius);
  grid_map::Polygon grid_polygon;
  try {
    for (const auto & point : polygon.outer()) {
      grid_polygon.addVertex({point.x(), point.y()});
    }
    for (grid_map::PolygonIterator iterator(grid, grid_polygon); !iterator.isPastEnd();
         ++iterator) {
      const grid_map::Index & index = *iterator;
      if (grid_data(index.x(), index.y()) == grid_utils::occlusion_cost_value::OCCUPIED) {
        return false;
      }
    }
  } catch (const std::invalid_argument & e) {
    std::cerr << e.what() << std::endl;
  }
  return true;
}

void getCornerPositions(
  std::vector<grid_map::Position> & corner_positions, const grid_map::GridMap & grid,
  const OcclusionSpotSquare & occlusion_spot_square)
{
  // Special case with size = 1: only one cell
  if (occlusion_spot_square.min_occlusion_size == 1) {
    corner_positions.emplace_back(occlusion_spot_square.position);
    return;
  }
  std::vector<grid_map::Index> corner_indexes;
  const int offset = (occlusion_spot_square.min_occlusion_size - 1) / 2;
  /**
   * @brief relation of each grid position
   *    bl br
   *    tl tr
   */
  corner_indexes = {// bl
                    grid_map::Index(
                      std::max(0, occlusion_spot_square.index.x() - offset),
                      std::max(0, occlusion_spot_square.index.y() - offset)),
                    // br
                    grid_map::Index(
                      std::min(grid.getSize().x() - 1, occlusion_spot_square.index.x() + offset),
                      std::max(0, occlusion_spot_square.index.y() - offset)),
                    // tl
                    grid_map::Index(
                      std::max(0, occlusion_spot_square.index.x() - offset),
                      std::min(grid.getSize().y() - 1, occlusion_spot_square.index.y() + offset)),
                    // tr
                    grid_map::Index(
                      std::min(grid.getSize().x() - 1, occlusion_spot_square.index.x() + offset),
                      std::min(grid.getSize().y() - 1, occlusion_spot_square.index.y() + offset))};
  for (const grid_map::Index & corner_index : corner_indexes) {
    grid_map::Position corner_position;
    grid.getPosition(corner_index, corner_position);
    corner_positions.emplace_back(corner_position);
  }
}
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
      if (intensity == grid_utils::occlusion_cost_value::FREE_SPACE_IMAGE) {
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
  const nav_msgs::msg::OccupancyGrid & occupancy_grid, cv::Mat * cv_image, const GridParam & param)
{
  const int width = cv_image->cols;
  const int height = cv_image->rows;
  for (int x = width - 1; x >= 0; x--) {
    for (int y = height - 1; y >= 0; y--) {
      const int idx = (height - 1 - y) + (width - 1 - x) * height;
      unsigned char intensity = occupancy_grid.data.at(idx);
      if (intensity <= param.free_space_max) {
        intensity = grid_utils::occlusion_cost_value::FREE_SPACE_IMAGE;
      } else if (param.free_space_max < intensity && intensity < param.occupied_min) {
        intensity = grid_utils::occlusion_cost_value::UNKNOWN_IMAGE;
      } else if (param.occupied_min <= intensity) {
        intensity = grid_utils::occlusion_cost_value::OCCUPIED_IMAGE;
      } else {
        throw std::logic_error("behavior_velocity[occlusion_spot_grid]: invalid if clause");
      }
      cv_image->at<unsigned char>(y, x) = intensity;
    }
  }
}

void denoiseOccupancyGridCV(
  const nav_msgs::msg::OccupancyGrid::ConstSharedPtr occupancy_grid_ptr,
  grid_map::GridMap & grid_map, const GridParam & param, const bool is_show_debug_window,
  const bool filter_occupancy_grid)
{
  OccupancyGrid occupancy_grid = *occupancy_grid_ptr;
  cv::Mat cv_image(
    occupancy_grid.info.width, occupancy_grid.info.height, CV_8UC1,
    cv::Scalar(grid_utils::occlusion_cost_value::OCCUPIED));
  toQuantizedImage(occupancy_grid, &cv_image, param);
  constexpr int num_iter = 2;
  //!< @brief opening & closing to remove noise in occupancy grid
  if (filter_occupancy_grid) {
    cv::morphologyEx(cv_image, cv_image, cv::MORPH_CLOSE, cv::Mat(), cv::Point(-1, -1), num_iter);
  }
  if (is_show_debug_window) {
    cv::namedWindow("morph", cv::WINDOW_NORMAL);
    cv::imshow("morph", cv_image);
    cv::waitKey(1);
  }
  imageToOccupancyGrid(cv_image, &occupancy_grid);
  grid_map::GridMapRosConverter::fromOccupancyGrid(occupancy_grid, "layer", grid_map);
}
}  // namespace grid_utils
}  // namespace behavior_velocity_planner
