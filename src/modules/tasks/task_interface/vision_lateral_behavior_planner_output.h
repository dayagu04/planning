#pragma once
#include <array>
#include <limits>
#include <string>
#include <vector>
namespace planning {
struct VisionLateralBehaviorPlannerOutput {
  std::array<std::vector<double>, 2> avd_car_past;
  std::array<std::vector<double>, 2> avd_sp_car_past;
  bool flag_avd = true;
  double dist_rblane = 0.0;
  std::string lc_status;
  int ncar_change;
  int avd_back_cnt;
  int avd_leadone;
  int pre_leadone_id;
  double final_y_rel;
};

}  // namespace planning
