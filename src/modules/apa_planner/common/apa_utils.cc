#include "apa_planner/common/apa_utils.h"

#include "apa_planner/common/apa_cos_sin.h"

namespace planning {

using planning::planning_math::Polygon2d;
using planning::planning_math::Vec2d;

Polygon2d ConstructVehiclePolygon(
    const PlanningPoint &rear_center, const double half_width,
    const double front_edge_to_rear_center,
    const double rear_edge_to_rear_center,
    const double front_shrink_dis, const double front_side_shrink_dis,
    const double rear_shrink_dis, const double rear_side_shrink_dis) {
  std::vector<Vec2d> points;
  points.reserve(8);
  const double sin_heading = apa_sin(rear_center.theta);
  const double cos_heading = apa_cos(rear_center.theta);
  const double dx1 = cos_heading * front_edge_to_rear_center;
  const double dy1 = sin_heading * front_edge_to_rear_center;
  const double dx2 =
      cos_heading * (front_edge_to_rear_center - front_side_shrink_dis);
  const double dy2 =
      sin_heading * (front_edge_to_rear_center - front_side_shrink_dis);

  const double dx3 = sin_heading * half_width;
  const double dy3 = cos_heading * half_width;
  const double dx4 = sin_heading * (half_width - front_shrink_dis);
  const double dy4 = cos_heading * (half_width - front_shrink_dis);

  const double dx5 = cos_heading * rear_edge_to_rear_center;
  const double dy5 = sin_heading * rear_edge_to_rear_center;
  const double dx6 =
      cos_heading * (rear_edge_to_rear_center - rear_side_shrink_dis);
  const double dy6 =
      sin_heading * (rear_edge_to_rear_center - rear_side_shrink_dis);

  const double dx7 =
      sin_heading * (half_width - rear_shrink_dis);
  const double dy7 =
      cos_heading * (half_width - rear_shrink_dis);

  points.emplace_back(
      rear_center.x + dx2 + dx3, rear_center.y + dy2 - dy3);
  points.emplace_back(
      rear_center.x + dx1 + dx4, rear_center.y + dy1 - dy4);
  points.emplace_back(
      rear_center.x + dx1 - dx4, rear_center.y + dy1 + dy4);
  points.emplace_back(
      rear_center.x + dx2 - dx3, rear_center.y + dy2 + dy3);
  points.emplace_back(
      rear_center.x - dx6 - dx3, rear_center.y - dy6 + dy3);
  points.emplace_back(
      rear_center.x - dx5 - dx7, rear_center.y - dy5 + dy7);
  points.emplace_back(
      rear_center.x - dx5 + dx7, rear_center.y - dy5 - dy7);
  points.emplace_back(
      rear_center.x - dx6 + dx3, rear_center.y - dy6 - dy3);

  return Polygon2d(points);
}

} // namespace  planning