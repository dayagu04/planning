#pragma once

#include <string>
#include <utility>
#include <vector>

#include "define/geometry.h"
#include "lateral_motion_planner.pb.h"
namespace planning {
struct HppGeneralLateralDeciderOutput {
  planning::common::LateralInitState init_state;
  std::vector<std::pair<double, double>> enu_ref_path;
  std::vector<std::pair<double, double>> last_enu_ref_path;
  std::vector<std::pair<Point2D, Point2D>> path_bounds;
  std::vector<std::pair<Point2D, Point2D>> safe_bounds;
  std::vector<double> enu_ref_theta;
  std::vector<double> last_enu_ref_theta;
  double v_cruise;
  bool complete_follow = true;
  std::string lc_status;

  void Clear() {
    init_state.Clear();
    enu_ref_path.clear();
    last_enu_ref_path.clear();
    path_bounds.clear();
    safe_bounds.clear();
    enu_ref_theta.clear();
    last_enu_ref_theta.clear();
    lc_status.clear();
  }
};
}  // namespace planning