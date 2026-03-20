#pragma once

#include <vector>

#include "config/basic_type.h"
#include "ilqr_define.h"
#include "spline.h"
namespace planning {

struct LateralMotionPlannerOutput {
  pnc::mathlib::spline ref_x_s_spline_;
  pnc::mathlib::spline ref_y_s_spline_;
  pnc::mathlib::spline x_s_spline_;
  pnc::mathlib::spline y_s_spline_;
  pnc::mathlib::spline theta_s_spline_;
  pnc::mathlib::spline delta_s_spline_;
  pnc::mathlib::spline omega_s_spline_;
  pnc::mathlib::spline curv_s_spline_;
  pnc::mathlib::spline d_curv_s_spline_;
  std::vector<double> s_lat_vec_;
  ilqr_solver::ControlVec u_vec_;
  TrajectoryPoints traj_points_;
  bool lat_init_flag;
};

}  // namespace planning