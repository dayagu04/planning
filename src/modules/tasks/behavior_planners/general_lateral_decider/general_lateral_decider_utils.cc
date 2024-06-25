#include "general_lateral_decider_utils.h"
#include <cassert>

namespace planning {
namespace general_lateral_decider_utils {
double CalDesireLateralDistance(const double ego_vel, const double pred_ts,
                              const double agent_lateral_relative_speed,
                              const bool is_vru_agent,
                              const bool is_nudge_left) {
  const double max_base_dis = is_vru_agent ? 1.0 : 0.8;
  double base_dis = max_base_dis;

  return std::fmax(base_dis + 0.015 * ego_vel - 0.1 * pred_ts, 0.);
}

double CalDesireLonDistance(double ego_vel, double agent_vel) {
  return 3.0 + std::fmax(0., (ego_vel - agent_vel) * 0.2) + ego_vel * 0.2;
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
    return 2;
  //  the same level
  case BoundType::AGENT:
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


std::vector<int> match_ref_traj_points(int s, const TrajectoryPoints &ref_traj_points) {
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
    if (abs(ref_traj_points[left_index].s - s) < abs(ref_traj_points[right_index].s - s)) {
      index.emplace_back(left_index);
      return index;
    } else {
      index.emplace_back(right_index);
      return index;
    }
  }

}

}
}