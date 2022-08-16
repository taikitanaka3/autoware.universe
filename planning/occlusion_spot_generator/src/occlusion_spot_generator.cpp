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

inline Polygon2d toBoostPoly(const geometry_msgs::msg::Polygon & polygon)
{
  Polygon2d boost_poly;
  for (const auto & point : polygon.points) {
    const Point2d point2d(point.x, point.y);
    boost_poly.outer().push_back(point2d);
  }
  return boost_poly;
}

Polygon2d obj2Polygon(
  const geometry_msgs::msg::Pose & pose, const geometry_msgs::msg::Vector3 & shape)
{
  // rename
  const double x = pose.position.x;
  const double y = pose.position.y;
  const double h = shape.x;
  const double w = shape.y;
  const double yaw = tf2::getYaw(pose.orientation);

  // create base polygon
  Polygon2d obj_poly;
  boost::geometry::exterior_ring(obj_poly) = boost::assign::list_of<Point2d>(h / 2.0, w / 2.0)(
    -h / 2.0, w / 2.0)(-h / 2.0, -w / 2.0)(h / 2.0, -w / 2.0)(h / 2.0, w / 2.0);

  // rotate polygon(yaw)
  boost::geometry::strategy::transform::rotate_transformer<boost::geometry::radian, double, 2, 2>
    rotate(-yaw);  // anti-clockwise -> :clockwise rotation
  Polygon2d rotate_obj_poly;
  boost::geometry::transform(obj_poly, rotate_obj_poly, rotate);

  // translate polygon(x, y)
  boost::geometry::strategy::transform::translate_transformer<double, 2, 2> translate(x, y);
  Polygon2d translate_obj_poly;
  boost::geometry::transform(rotate_obj_poly, translate_obj_poly, translate);

  return translate_obj_poly;
}

Polygon2d toFootprintPolygon(const PredictedObject & object)
{
  Polygon2d obj_footprint;
  if (object.shape.type == Shape::POLYGON) {
    obj_footprint = toBoostPoly(object.shape.footprint);
  } else {
    // cylinder type is treated as square-polygon
    obj_footprint =
      obj2Polygon(object.kinematics.initial_pose_with_covariance.pose, object.shape.dimensions);
  }

  namespace bg = boost::geometry;
  namespace trans = bg::strategy::transform;
  // TODO add velocity buffer
  // {
  //   const double vel = object.kinematics.initial_twist_with_covariance.twist.linear.x;
  //   const double yaw = tf2::getYaw(kinematics.initial_pose_with_covariance.pose.orientation);
  //   const double delay_time = 0.2; // [sec]
  //   // translate polygon(x, y)
  //   const double t_x = vel*delay_time*std::cos(yaw);
  //   const double t_y = vel*delay_time*std::sin(yaw);
  //   boost::geometry::strategy::transform::translate_transformer<double, 2, 2> translate(t_x,
  //   t_y); Polygon2d translate_obj_poly; boost::geometry::transform(obj_footprint,
  //   translate_obj_poly, translate);
  //   // add poly
  //   // convex hull
  // }

  // up scale transform
  {
    trans::scale_transformer<double, 2, 2> translate(1.1);
    bg::transform(obj_footprint, obj_footprint, translate);
  }
  return obj_footprint;
}

bool isVehicle(const PredictedObject & obj)
{
  const auto & label = obj.classification.at(0).label;
  return (
    label == ObjectClassification::CAR || label == ObjectClassification::TRUCK ||
    label == ObjectClassification::BUS || label == ObjectClassification::TRAILER);
}

bool isStuckVehicle(const PredictedObject & obj, const double min_vel)
{
  if (!isVehicle(obj)) return false;
  const auto & obj_vel = obj.kinematics.initial_twist_with_covariance.twist.linear.x;
  return std::abs(obj_vel) <= min_vel;
}

bool isMovingVehicle(const PredictedObject & obj, const double min_vel)
{
  if (!isVehicle(obj)) return false;
  const auto & obj_vel = obj.kinematics.initial_twist_with_covariance.twist.linear.x;
  return std::abs(obj_vel) > min_vel;
}

std::vector<PredictedObject> extractVehicles(
  const PredictedObjects::ConstSharedPtr objects_ptr, const Point ego_position,
  const double distance)
{
  std::vector<PredictedObject> vehicles;
  for (const auto & obj : objects_ptr->objects) {
    if (occlusion_spot_generator::isVehicle(obj)) {
      const auto & o = obj.kinematics.initial_pose_with_covariance.pose.position;
      const auto & p = ego_position;
      // Don't consider far vehicle
      if (std::hypot(p.x - o.x, p.y - o.y) > distance) continue;
      vehicles.emplace_back(obj);
    }
  }
  return vehicles;
}

void vehiclesToFootprint(
  const std::vector<PredictedObject> & vehicles, Polygons2d & stuck_vehicle_foot_prints,
  Polygons2d & moving_vehicle_foot_prints, const double stuck_vehicle_vel)
{
  moving_vehicle_foot_prints.clear();
  stuck_vehicle_foot_prints.clear();
  for (const auto & vehicle : vehicles) {
    if (isMovingVehicle(vehicle, stuck_vehicle_vel)) {
      moving_vehicle_foot_prints.emplace_back(toFootprintPolygon(vehicle));
    } else if (isStuckVehicle(vehicle, stuck_vehicle_vel)) {
      stuck_vehicle_foot_prints.emplace_back(toFootprintPolygon(vehicle));
    }
  }
  return;
}
}  // namespace occlusion_spot_generator
