#include "apa_planner/apa_speed_decider/apa_speed_limit_decider.h"

#include <cassert>
#include <limits>

#include "apa_planner/common/apa_cos_sin.h"
#include "apa_planner/common/apa_utils.h"
#include "apa_planner/common/geometry_planning_io.h"
#include "common/math/linear_interpolation.h"
#include "common/math/math_utils.h"
#include "common/math/polygon2d.h"

namespace planning {

using ::PlanningOutput::PlanningOutput;
using planning_math::LineSegment2d;
using planning_math::Polygon2d;
using planning_math::Vec2d;

namespace {
constexpr double kEps = 1e-6;
constexpr double kLatBuffer = 0.1;
constexpr double kLonBuffer = 0.1;
constexpr double kMaxLatAcc = 1.0;
constexpr double kMaxSpd = 1.0;
constexpr double kMinSpd = 0.3;
constexpr double kMaxObsDis = 1.0;
constexpr double kMinObsDis = 0.2;
}

bool ApaSpeedLimitDecider::GetSpeedLimits(
    const PlanningOutput& planning_output,
    const std::vector<LineSegment2d>& obstacles,
    ApaSpeedLimit* const speed_limit_data) const {
  assert(speed_limit_data != nullptr);

  const int traj_num = planning_output.trajectory().trajectory_points_size();
  if (traj_num == 0) {
    return false;
  }

  const auto& traj_points = planning_output.trajectory().trajectory_points();
  const double motion_sign = traj_points[0].v() > 0.0 ? 1.0 : -1.0;
  const double front_buffer = motion_sign > 0.0 ? kLonBuffer : 0.0;
  const double rear_buffer = motion_sign < 0.0 ? kLonBuffer : 0.0;
  const double lat_buffer = kLatBuffer;

  PlanningPoint point_0(
      traj_points[0].x(), traj_points[0].y(), traj_points[0].heading_yaw());
  Polygon2d init_ego_polygon = std::move(ConstructVehiclePolygonWithBuffer(
        point_0, front_buffer, rear_buffer, lat_buffer));
  for (int i = 0; i < traj_num; ++i) {
    const auto& traj_point_i =
        planning_output.trajectory().trajectory_points()[i];
    PlanningPoint point_i(traj_point_i.x(), traj_point_i.y(),
        traj_point_i.heading_yaw());
    Polygon2d ego_polygon(init_ego_polygon);
    const double rotate_angle = point_i.theta - point_0.theta;
    ego_polygon.RotateAndTranslate(Vec2d(point_0.x, point_0.y),
      apa_sin(rotate_angle), apa_cos(rotate_angle),
      Vec2d(point_i.x - point_0.x, point_i.y - point_0.y));
    const double abs_curvatrue = std::fabs(traj_point_i.curvature());
    // 1. speed limit from curvature
    const double speed_limit_from_lat_acc =
        std::sqrt(kMaxLatAcc / std::fmax(kEps, abs_curvatrue));

    // 2. speed limit from obstacles
    double min_dis_from_obs = std::numeric_limits<double>::infinity();
    for (const auto& obs : obstacles) {
      const double dis_from_obs = ego_polygon.DistanceTo(obs);
      min_dis_from_obs = std::fmin(min_dis_from_obs, dis_from_obs);
    }
    const double speed_limit_from_obs =
        CalSpeedLimitFromObstacleDistance(min_dis_from_obs);

    const double speed_limit = std::min(
        {kMaxSpd, speed_limit_from_lat_acc, speed_limit_from_obs});

    speed_limit_data->AppendSpeedLimit(traj_point_i.distance(), speed_limit);
  }

  return true;
}

double ApaSpeedLimitDecider::CalSpeedLimitFromObstacleDistance(
    const double dis_from_obstacle) const {
  const double speed = planning_math::lerp(
      kMinSpd, kMinObsDis, kMaxSpd, kMaxObsDis, dis_from_obstacle);
  return planning_math::Clamp(speed, kMinSpd, kMaxSpd);
}

}  // namespace planning
