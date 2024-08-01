#pragma once
#include <string>
#include <utility>
#include <vector>

#include "config/basic_type.h"
#include "task_basic_types.h"
#include "define/geometry.h"
#include "lateral_motion_planner.pb.h"

namespace planning {
struct GeneralLateralDeciderOutput {
  planning::common::LateralInitState init_state;
  std::vector<WeightedBounds> soft_bounds;
  std::vector<WeightedBounds> hard_bounds;
  std::vector<std::pair<double, double>> enu_ref_path;
  std::vector<std::pair<double, double>> last_enu_ref_path;
  std::vector<std::pair<Point2D, Point2D>> soft_bounds_cart_point;
  std::vector<std::pair<Point2D, Point2D>> hard_bounds_cart_point;
  std::vector<double> enu_ref_theta;
  std::vector<double> last_enu_ref_theta;
  bool complete_follow = true;
  std::string lc_status;
  bool lane_change_scene = false;
  double v_cruise;
  void Clear() {
    init_state.Clear();
    enu_ref_path.clear();
    last_enu_ref_path.clear();
    soft_bounds.clear();
    hard_bounds.clear();
    soft_bounds_cart_point.clear();
    hard_bounds_cart_point.clear();
    enu_ref_theta.clear();
    last_enu_ref_theta.clear();
    lc_status.clear();
  }
};
}  // namespace planning