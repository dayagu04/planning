#include "hpp_general_lateral_decider_utils.h"
#include <cassert>
#include <cmath>
#include "common/math/linear_interpolation.h"
#include "reference_path_manager.h"
#include "utils/kd_path.h"
#include "utils/pose2d_utils.h"

namespace planning {
namespace hpp_general_lateral_decider_utils {
double CalDesireLateralDistance(const double ego_vel, const double pred_ts,
                                const double agent_lateral_relative_speed,
                                const std::shared_ptr<FrenetObstacle> obstacle,
                                const bool is_nudge_left, bool in_intersection,
                                HppGeneralLateralDeciderConfig &config) {
  double base_dis = 0.8;
  if (IsVRU(obstacle->type())) {
    base_dis = 1.0;
  } else if (IsTruck(obstacle)) {
    base_dis = 0.8;
  }
  if (in_intersection) {
    base_dis += config.nudge_extra_buffer_in_intersection;
  }

  return std::fmax(base_dis + 0.015 * ego_vel, 0.);
}

double CalDesireLonDistance(double ego_vel, double agent_vel, double base_dist) {
  return base_dist + std::fmax(0., (ego_vel - agent_vel) * 0.2) + ego_vel * 0.2;
}

double CalDesireStaticLateralDistance(const double base_distance,
                                      const double ego_vel, const double ego_l,
                                      iflyauto::ObjectType type,
                                      bool is_update_hard_bound) {
  const double kStaticVRUMaxExtraLateralBuffer = 0.65;
  const double kConeMaxExtraLateralBuffer = 0.15;
  const double kStaticOtherMaxExtraLateralBuffer = 0.45;
  const double kMaxEgoLCoeff = 0.5;

  if (is_update_hard_bound) {
    return base_distance;
  }

  double max_extra_lateral_buffer = 0;
  if (IsVRU(type)) {
    max_extra_lateral_buffer = kStaticVRUMaxExtraLateralBuffer;
  } else if (IsCone(type)) {
    max_extra_lateral_buffer = kConeMaxExtraLateralBuffer;
  } else {
    max_extra_lateral_buffer = kStaticOtherMaxExtraLateralBuffer;
  }

  double min_extra_lateral_buffer =
      std::fmin(0.1 * ego_vel, max_extra_lateral_buffer);

  double clip_ego_l = clip(fabs(ego_l), kMaxEgoLCoeff, 0.0);
  double lateral_extra_buffer =
      min_extra_lateral_buffer +
      clip_ego_l * (max_extra_lateral_buffer - min_extra_lateral_buffer) /
          kMaxEgoLCoeff;
  return base_distance + lateral_extra_buffer;
}

double GetBoundWeight(
    BoundType type,
    const std::unordered_map<BoundType, double> &map_bound_weight) {
  if (map_bound_weight.find(type) != map_bound_weight.end()) {
    return map_bound_weight.at(type);
  } else {
    return 0.1;
  }
}

int GetBoundTypePriority(BoundType type) {
  // higher priority, larger value
  switch (type) {
    // the same level
    case BoundType::DEFAULT:
      return 0;
    // the same level
    case BoundType::LANE:
      return 1;
    case BoundType::EGO_POSITION:
      return 1;
    //  the same level
    case BoundType::DYNAMIC_AGENT:
      return 3;
    //  the same level
    case BoundType::AGENT:
      return 3;
    case BoundType::ADJACENT_AGENT:
      return 3;
    case BoundType::ROAD_BORDER:
      return 3;
    case BoundType::GROUNDLINE:
      return 3;
    case BoundType::PARKING_SPACE:
      return 3;
    //  the same level
    // case BoundType::PURNE_VEHICLE_WIDTH:
    //   return 4;
    default:
      return 0;
  }
}

std::vector<int> MatchRefTrajPoints(int s,
                                    const TrajectoryPoints &ref_traj_points) {
  assert(ref_traj_points.size() >= 1);
  int left_index = 0;
  int right_index = ref_traj_points.size() - 1;
  std::vector<int> index;
  while (left_index <= right_index) {
    int mid_index = left_index + (right_index - left_index) / 2;
    if (fabs(ref_traj_points[mid_index].s - s) < 1e-6) {
      index.emplace_back(mid_index);
      return index;
    } else if (ref_traj_points[mid_index].s > s) {
      right_index = mid_index - 1;
    } else {
      left_index = mid_index + 1;
    }
  }

  if (left_index >= ref_traj_points.size()) {
    index.emplace_back(right_index);
    return index;
  } else if (right_index < 0) {
    index.emplace_back(left_index);
    return index;
  } else {
    index.emplace_back(left_index);
    index.emplace_back(right_index);
    return index;
    if (abs(ref_traj_points[left_index].s - s) <
        abs(ref_traj_points[right_index].s - s)) {
      index.emplace_back(left_index);
      return index;
    } else {
      index.emplace_back(right_index);
      return index;
    }
  }
}

TrajectoryPoint GetTrajectoryPointAtTime(
    const TrajectoryPoints trajectory_points, const double relative_time) {
  const auto &points = trajectory_points;
  if (trajectory_points.size() < 2) {
    TrajectoryPoint point;
    return point;
  } else {
    auto comp = [](const TrajectoryPoint &p, const double time) {
      return p.t < time;
    };

    auto it_lower =
        std::lower_bound(points.begin(), points.end(), relative_time, comp);

    if (it_lower == points.begin()) {
      return *points.begin();
    } else if (it_lower == points.end()) {
      return *points.rbegin();
    }
    return planning_math::InterpolateUsingLinearApproximation(
        *(it_lower - 1), *it_lower, relative_time);
  }
}

bool IsVRU(iflyauto::ObjectType type) {
  return type == iflyauto::ObjectType::OBJECT_TYPE_PEDESTRIAN ||
         type == iflyauto::ObjectType::OBJECT_TYPE_BICYCLE ||
         type == iflyauto::ObjectType::OBJECT_TYPE_MOTORCYCLE ||
         type == iflyauto::ObjectType::OBJECT_TYPE_TRICYCLE ||
         type == iflyauto::ObjectType::OBJECT_TYPE_CYCLE_RIDING ||
         type == iflyauto::ObjectType::OBJECT_TYPE_MOTORCYCLE_RIDING ||
         type == iflyauto::ObjectType::OBJECT_TYPE_TRICYCLE_RIDING ||
         type == iflyauto::ObjectType::OBJECT_TYPE_ANIMAL;
}

bool IsCone(iflyauto::ObjectType type) {
  // TODO(clren):other type
  return type == iflyauto::ObjectType::OBJECT_TYPE_TRAFFIC_CONE;
}

bool IsTruck(const std::shared_ptr<FrenetObstacle> obstacle) {
  return (obstacle->type() == iflyauto::ObjectType::OBJECT_TYPE_BUS ||
          (obstacle->type() == iflyauto::ObjectType::OBJECT_TYPE_TRUCK &&
          obstacle->length() > 6));
}

ObstacleBorderInfo GetNearestObstacleBorder(
    const planning_math::Polygon2d &care_polygon, double care_area_s_start,
    double care_area_s_end,
    const std::vector<std::pair<int, planning_math::Polygon2d>>
        &obstacle_frenet_polygons,
    bool is_left, bool is_sorted, bool is_curve, int index,
    const TrajectoryPoints &traj_points) {
  static constexpr double kMaxLaneBound = 100.0;
  double l_care_width = 10.;
  double nearest_border = is_left ? kMaxLaneBound : -kMaxLaneBound;
  // planning_math::Polygon2d overlap_polygon;
  ObstacleBorderInfo nearest_obstacle_border;
  nearest_obstacle_border.obstacle_id = -100;
  nearest_obstacle_border.obstacle_border = nearest_border;

  for (auto &polygon : obstacle_frenet_polygons) {
    if (polygon.second.max_x() < care_area_s_start) {
      continue;
    }
    if (polygon.second.min_x() > care_area_s_end) {
      if (is_sorted) {
        break;
      }
      continue;
    }
    if (!is_sorted && polygon.second.max_y() * polygon.second.min_y() < 0) {
      continue;
    }
    if (std::min(std::fabs(polygon.second.min_y()),
                 std::fabs(polygon.second.max_y())) > l_care_width)
      continue;
    nearest_border = is_left
                         ? std::fmin(nearest_border, polygon.second.min_y())
                         : std::fmax(nearest_border, polygon.second.max_y());
    if (nearest_obstacle_border.obstacle_border != nearest_border) {
      nearest_obstacle_border.obstacle_id = polygon.first;
      nearest_obstacle_border.obstacle_border = nearest_border;
    }
  }
  return nearest_obstacle_border;
}

bool ConstructLinePolygons(const std::vector<planning_math::Vec2d> &line,
                           double y_width,
                           std::vector<planning_math::Polygon2d> &polygons) {
  for (size_t i = 0; i < line.size() - 1; i++) {
    auto x_start = line[i].x();
    auto y_start = line[i].y();
    auto x_end = line[i + 1].x();
    auto y_end = line[i + 1].y();

    auto y_edge = y_width;
    if (y_width > 0) {
      y_edge += fmax(y_start, y_end);
    } else if (y_width < 0) {
      y_edge += fmin(y_start, y_end);
    }

    std::vector<planning_math::Vec2d> points{
        planning_math::Vec2d{x_start, y_start},
        planning_math::Vec2d{x_end, y_end}, planning_math::Vec2d{x_end, y_edge},
        planning_math::Vec2d{x_start, y_edge}};

    planning_math::Polygon2d polygon;
    if (not planning_math::Polygon2d::ComputeConvexHull(points, &polygon)) {
      continue;
    }
    polygons.emplace_back(polygon);
  }

  return true;
}

bool OnLeftSide(const std::vector<planning_math::Vec2d> &vec2ds) {
  for (auto &vec2d : vec2ds) {
    if (vec2d.y() <= 0) {
      return false;
    }
  }
  return true;
}

bool Vec2dsToFrenet2ds(const std::shared_ptr<KDPath> &frenet_coord,
                       const std::vector<planning_math::Vec2d> &pts,
                       std::vector<planning_math::Vec2d> &frenet_pts) {
  const double kDistanceToRefThreshold = 1.0;
  if (pts.size() == 0) {
    ILOG_INFO << "vec2ds_to_frenet2ds empty input";
    return false;
  }
  Point2D carte_point, frenet_point;
  for (size_t i = 0; i < pts.size(); i++) {
    carte_point.x = pts[i].x();
    carte_point.y = pts[i].y();
    if (!frenet_coord->XYToSL(carte_point, frenet_point) ||
        std::isnan(frenet_point.x) || std::isnan(frenet_point.y)) {
      return false;
    }
    frenet_pts.emplace_back(
        planning_math::Vec2d{frenet_point.x, frenet_point.y});
  }

  // [hack]: just effective for groundline close to road center
  for (size_t i = 0; i < frenet_pts.size(); i++) {
    if (std::fabs(frenet_pts.at(i).y()) < kDistanceToRefThreshold) {
      return false;
    }
  }

  bool is_same_lat_direction = true;
  for (size_t i = 0; i + 1 < frenet_pts.size(); i++) {
    if (frenet_pts.at(i).y() * frenet_pts.at(i + 1).y() < 0) {
      is_same_lat_direction = false;
      break;
    }
  }
  if (!is_same_lat_direction) {
    ILOG_INFO << "stupid ground line obstacle!!!";
  }
  return is_same_lat_direction;
}

void MakeLinePolygons(
    const Obstacle *const &obstacle,
    const std::shared_ptr<ReferencePath> &reference_path_ptr,
    std::vector<std::pair<int, planning_math::Polygon2d>> &left_polygons,
    std::vector<std::pair<int, planning_math::Polygon2d>> &right_polygons) {
  for (auto &frenet_obstacle : reference_path_ptr->get_obstacles()) {
    if (frenet_obstacle->id() == obstacle->id() &&
        frenet_obstacle->b_frenet_valid()) {
      planning_math::Polygon2d tmp_polygon(frenet_obstacle->corner_points());
      if (frenet_obstacle->frenet_l() > 0) {
        left_polygons.emplace_back(obstacle->id(), tmp_polygon);
      } else {
        right_polygons.emplace_back(obstacle->id(), tmp_polygon);
      }
    }
  }
}

void MakePolygon(
    const int obstacle_id, const std::shared_ptr<KDPath> &frenet_coord,
    const planning_math::Polygon2d &polygon,
    std::vector<std::pair<int, planning_math::Polygon2d>> &left_polygons,
    std::vector<std::pair<int, planning_math::Polygon2d>> &right_polygons) {
  const std::vector<planning_math::Vec2d> &points = polygon.points();
  std::vector<planning_math::Vec2d> frenet_points;
  frenet_points.reserve(points.size());
  bool trans_success = Vec2dsToFrenet2ds(frenet_coord, points, frenet_points);
  if (!trans_success) {
    return;
  }
  planning_math::Polygon2d tmp_polygon(frenet_points);
  if (OnLeftSide(frenet_points) > 0) {
    left_polygons.emplace_back(obstacle_id, tmp_polygon);
  } else {
    right_polygons.emplace_back(obstacle_id, tmp_polygon);
  }
}

}  // namespace hpp_general_lateral_decider_utils
}  // namespace planning