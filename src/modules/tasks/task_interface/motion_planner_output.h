#pragma once
#include "ilqr_define.h"
#include "spline.h"
namespace planning {

struct MotionPlannerOutput {
  bool lat_enable_flag = false;
  bool lon_enable_flag = false;
  bool lat_init_flag = false;
  pnc::mathlib::spline ref_x_s_spline;
  pnc::mathlib::spline ref_y_s_spline;
  pnc::mathlib::spline x_s_spline;
  pnc::mathlib::spline y_s_spline;
  pnc::mathlib::spline theta_s_spline;
  pnc::mathlib::spline delta_s_spline;
  pnc::mathlib::spline omega_s_spline;
  pnc::mathlib::spline curv_s_spline;
  pnc::mathlib::spline d_curv_s_spline;

  pnc::mathlib::spline lateral_x_t_spline;
  pnc::mathlib::spline lateral_y_t_spline;
  pnc::mathlib::spline lateral_theta_t_spline;
  pnc::mathlib::spline lateral_s_t_spline;
  pnc::mathlib::spline lateral_t_s_spline;

  pnc::mathlib::spline s_t_spline;
  pnc::mathlib::spline v_t_spline;
  pnc::mathlib::spline a_t_spline;
  pnc::mathlib::spline j_t_spline;

  pnc::mathlib::spline x_t_spline;
  pnc::mathlib::spline y_t_spline;

  pnc::mathlib::spline ref_x_t_spline;
  pnc::mathlib::spline ref_y_t_spline;

  pnc::mathlib::spline theta_t_spline;
  pnc::mathlib::spline delta_t_spline;
  pnc::mathlib::spline omega_t_spline;

  std::vector<double> s_lat_vec;

  ilqr_solver::ControlVec u_vec;
};

}  // namespace planning
