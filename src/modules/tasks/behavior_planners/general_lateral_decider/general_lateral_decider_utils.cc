#include "general_lateral_decider_utils.h"
#include <cassert>
#include <cmath>
#include "common/math/linear_interpolation.h"
#include "utils/pose2d_utils.h"

namespace planning {
namespace general_lateral_decider_utils {
double CalDesireLateralDistance(const double ego_vel, const double pred_ts,
                                const double agent_lateral_relative_speed,
                                iflyauto::ObjectType type,
                                const bool is_nudge_left, bool in_intersection,
                                GeneralLateralDeciderConfig& config) {
  double base_dis = 0.8;
  if (IsVRU(type)) {
    base_dis = 1.0;
  } else if (IsTruck(type)) {
    base_dis = 0.8;
  }
  if (in_intersection) {
    base_dis += config.nudge_extra_buffer_in_intersection;
  }

  return std::fmax(base_dis + 0.015 * ego_vel, 0.);
}

double CalDesireLonDistance(double ego_vel, double agent_vel) {
  return 3.0 + std::fmax(0., (ego_vel - agent_vel) * 0.2) + ego_vel * 0.2;
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
      std::fmin(0.15 * ego_vel, max_extra_lateral_buffer);

  double clip_ego_l = clip(fabs(ego_l), kMaxEgoLCoeff, 0.0);
  double lateral_extra_buffer =
      min_extra_lateral_buffer +
      clip_ego_l * (max_extra_lateral_buffer - min_extra_lateral_buffer) /
          kMaxEgoLCoeff;
  return base_distance + lateral_extra_buffer;
}

double GetBoundWeight(BoundType type, const std::unordered_map<BoundType, double>& map_bound_weight) {
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
  if (trajectory_points.size() == 0) {
  } else if (trajectory_points.size() == 1) {
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

bool IsTruck(iflyauto::ObjectType type) {
  return (type == iflyauto::ObjectType::OBJECT_TYPE_BUS ||
          type == iflyauto::ObjectType::OBJECT_TYPE_TRUCK);
}

}  // namespace general_lateral_decider_utils
}  // namespace planning