#pragma once
#include <string>
#include <utility>
#include <vector>

#include "config/basic_type.h"
#include "define/geometry.h"
#include "lateral_motion_planner.pb.h"
#include "modules/tasks/task_interface/potential_dangerous_agent_decider_output.h"
#include "task_basic_types.h"

namespace planning {
struct GeneralLateralDeciderOutput {
  planning::common::LateralInitState init_state;
  std::vector<WeightedBounds> second_soft_bounds; // 默认和之前一样，为第二层软约束
  std::vector<WeightedBounds> first_soft_bounds; // 稳定
  std::vector<WeightedBounds> hard_bounds;
  std::vector<std::pair<double, double>> enu_ref_path;
  std::vector<std::pair<double, double>> front_axis_enu_ref_path;
  std::vector<std::pair<double, double>> last_enu_ref_path;
  std::vector<std::pair<Point2D, Point2D>> second_soft_bounds_cart_point;
  std::vector<std::pair<Point2D, Point2D>> first_soft_bounds_cart_point;
  std::vector<std::pair<Point2D, Point2D>> hard_bounds_cart_point;
  std::vector<std::pair<double, double>> second_soft_bounds_frenet_point;
  std::vector<std::pair<double, double>> first_soft_bounds_frenet_point;
  std::vector<std::pair<double, double>> hard_bounds_frenet_point;
  std::vector<std::pair<BoundInfo, BoundInfo>> second_soft_bounds_info;
  std::vector<std::pair<BoundInfo, BoundInfo>> first_soft_bounds_info;
  std::vector<std::pair<BoundInfo, BoundInfo>> hard_bounds_info;
  std::vector<double> enu_ref_theta;
  std::vector<double> last_enu_ref_theta;
  pnc::mathlib::spline curve_s_spline;
  bool complete_follow = true;
  std::string lc_status;
  bool lane_change_scene = false;
  double v_cruise;
  bool ramp_scene = false;
  bool enable_ara_ref = false;
  bool bound_avoid = false;
  bool is_use_spatio_planner_result = false;
  double recommended_bound_avoid_jerk = 0.4;
  RiskLevel risk_level = RiskLevel::NO_RISK;
  void Clear() {
    complete_follow = true;
    lane_change_scene = false;
    ramp_scene = false;
    enable_ara_ref = false;
    bound_avoid = false;
    is_use_spatio_planner_result = false;
    recommended_bound_avoid_jerk = 0.4;
    risk_level = RiskLevel::NO_RISK;
    init_state.Clear();
    enu_ref_path.clear();
    last_enu_ref_path.clear();
    second_soft_bounds.clear();
    first_soft_bounds.clear();
    hard_bounds.clear();
    second_soft_bounds_cart_point.clear();
    first_soft_bounds_cart_point.clear();
    hard_bounds_cart_point.clear();
    second_soft_bounds_frenet_point.clear();
    first_soft_bounds_frenet_point.clear();
    hard_bounds_frenet_point.clear();
    second_soft_bounds_info.clear();
    first_soft_bounds_info.clear();
    hard_bounds_info.clear();
    enu_ref_theta.clear();
    last_enu_ref_theta.clear();
    lc_status.clear();
    curve_s_spline.get_x().clear();
    curve_s_spline.get_y().clear();
  }
};
}  // namespace planning