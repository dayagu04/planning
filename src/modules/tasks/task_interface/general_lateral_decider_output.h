#pragma once
#include <string>
#include <utility>
#include <vector>

#include "config/basic_type.h"
#include "define/geometry.h"
#include "lateral_motion_planner.pb.h"
#include "task_basic_types.h"

namespace planning {
struct GeneralLateralDeciderOutput {
  planning::common::LateralInitState init_state;
  std::vector<WeightedBounds> soft_bounds;
  std::vector<WeightedBounds> hard_bounds;
  std::vector<std::pair<double, double>> enu_ref_path;
  std::vector<std::pair<double, double>> last_enu_ref_path;
  std::vector<std::pair<Point2D, Point2D>> soft_bounds_cart_point;
  std::vector<std::pair<Point2D, Point2D>> hard_bounds_cart_point;
  std::vector<std::pair<double, double>> soft_bounds_frenet_point;
  std::vector<std::pair<double, double>> hard_bounds_frenet_point;
  std::vector<std::pair<BoundInfo, BoundInfo>> soft_bounds_info;
  std::vector<std::pair<BoundInfo, BoundInfo>> hard_bounds_info;
  std::vector<double> enu_ref_theta;
  std::vector<double> last_enu_ref_theta;
  bool complete_follow = true;
  std::string lc_status;
  bool lane_change_scene = false;
  double v_cruise;
  bool ramp_scene = false;
  bool enable_ara_ref = false;
  bool bound_avoid = false;
  bool is_use_spatio_planner_result = false;
  void Clear() {
    complete_follow = true;
    lane_change_scene = false;
    ramp_scene = false;
    enable_ara_ref = false;
    bound_avoid = false;
    is_use_spatio_planner_result = false;
    init_state.Clear();
    enu_ref_path.clear();
    last_enu_ref_path.clear();
    soft_bounds.clear();
    hard_bounds.clear();
    soft_bounds_cart_point.clear();
    hard_bounds_cart_point.clear();
    soft_bounds_frenet_point.clear();
    hard_bounds_frenet_point.clear();
    soft_bounds_info.clear();
    hard_bounds_info.clear();
    enu_ref_theta.clear();
    last_enu_ref_theta.clear();
    lc_status.clear();
  }
};
}  // namespace planning