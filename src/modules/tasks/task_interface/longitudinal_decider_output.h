#pragma once

#include <unordered_map>
#include <utility>
#include <vector>

#include "task_basic_types.h"

namespace planning {
struct LongitudinalDeciderOutput {
  std::vector<double> t_list;
  std::vector<std::pair<double, double>> s_refs;  // <offset, weight>
  std::vector<std::pair<double, double>> ds_refs;
  std::vector<std::pair<double, double>> dds_refs;

  std::vector<WeightedBounds> hard_bounds;             // s hard bounds
  WeightedBounds hard_bounds_v3;                       // s hard bounds for v3
  std::vector<WeightedBounds> soft_bounds;             // s soft bounds
  WeightedBounds soft_bounds_v3;                       // s soft bounds for v3
  WeightedBounds extend_bounds_v3;                     // s extend bounds for v3
  std::vector<WeightedLonLeadBounds> lon_lead_bounds;  // s lead bounds
  std::vector<LonLeadBounds> lead_bounds;
  std::unordered_map<int, std::vector<LonObstacleOverlapInfo>>
      lon_obstacle_overlap_info;
  std::vector<LonObstalceYieldInfo> lon_obstacle_yield_info;

  SVBoundary lon_sv_boundary;

  Bounds lon_bound_v;
  Bounds lon_bound_a;
  Bounds lon_bound_jerk;

  std::vector<double> s_speed_adjust_target;
  std::vector<double> s_lane_change_target;

  void Clear() {
    t_list.clear();
    s_refs.clear();
    ds_refs.clear();
    for (size_t index = 0; index < hard_bounds.size(); index++) {
      hard_bounds[index].clear();
    }
    hard_bounds.clear();

    for (size_t index = 0; index < soft_bounds.size(); index++) {
      soft_bounds[index].clear();
    }
    soft_bounds.clear();

    for (size_t index = 0; index < lon_lead_bounds.size(); index++) {
      lon_lead_bounds[index].clear();
    }
    lon_lead_bounds.clear();

    lon_obstacle_overlap_info.clear();

    lon_sv_boundary.sv_bounds.clear();
    lon_bound_v.clear();
    lon_bound_a.clear();
    lon_bound_jerk.clear();
    lon_obstacle_yield_info.clear();
    s_speed_adjust_target.clear();
    s_lane_change_target.clear();
  }
};

}  // namespace planning