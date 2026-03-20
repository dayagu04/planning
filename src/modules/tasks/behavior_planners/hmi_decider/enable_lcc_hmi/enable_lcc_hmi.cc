#include "enable_lcc_hmi.h"

#include "environmental_model.h"
#include "planning_context.h"

namespace planning {

EnableLCCHMIDecider::EnableLCCHMIDecider(framework::Session* session,
                                         const HmiDeciderConfig& config) {
  session_ = session;
  config_ = config;
}

bool EnableLCCHMIDecider::Execute() {
  if (session_ == nullptr) {
    Reset();
    return false;
  }
  auto& ad_info = session_->mutable_planning_context()
                      ->mutable_planning_hmi_info()
                      ->ad_info;
  ad_info.is_avaliable = true;
  reference_path_ptr_ = session_->planning_context()
                            .lane_change_decider_output()
                            .coarse_planning_info.reference_path;
  if (reference_path_ptr_ == nullptr) {
    ad_info.is_avaliable = false;
    last_enable_lcc_ = ad_info.is_avaliable;
    return false;
  }

  const auto& ego_planning_result =
      session_->mutable_planning_context()->mutable_planning_result();
  const auto& motion_planner_output =
      session_->mutable_planning_context()->mutable_motion_planner_output();
  const auto& speed_limit_output = session_->mutable_planning_context()
                                       ->mutable_speed_limit_decider_output();
  // const auto& traj_points = ego_planning_result.traj_points;
  const auto& traj_points = ego_planning_result.raw_traj_points;
  const double curv_factor = motion_planner_output.curv_factor;
  const double tp_init_s = traj_points.front().s;

  double lat_jerk_thr = config_.lat_jerk_thr;
  const bool ramp_scene =
      session_->planning_context().general_lateral_decider_output().ramp_scene;
  if (ramp_scene) {
    lat_jerk_thr = config_.ramp_lat_jerk_thr;
  }

  double traj_max_lat_acc = 0.0;
  double traj_max_lat_jerk = 0.0;
  // int lat_jerk_above_thr_count = 0;
  double traj_max_lon_acc = 0.0;
  double traj_max_lon_jerk = 0.0;

  for (size_t i = 0; i < traj_points.size(); i++) {
    double tp_curvature =
        motion_planner_output.curv_s_spline(traj_points[i].s - tp_init_s);
    double tp_dcurvature =
        motion_planner_output.d_curv_s_spline(traj_points[i].s - tp_init_s);
    double tp_delta =
        motion_planner_output.delta_s_spline(traj_points[i].s - tp_init_s);
    double tp_lat_acc =
        curv_factor * traj_points[i].v * traj_points[i].v * tp_delta;
    traj_max_lat_acc = std::max(std::fabs(tp_lat_acc), traj_max_lat_acc);
    // double tp_omega =
    //     motion_planner_output.omega_s_spline(traj_points[i].s - tp_init_s);
    double tp_omega = 0.0;
    if (i < traj_points.size() - 1) {
      double next_tp_delta = motion_planner_output.delta_s_spline(
          traj_points[i + 1].s - tp_init_s);
      tp_omega = (next_tp_delta - tp_delta) /
                 (traj_points[i + 1].t - traj_points[i].t);
    }
    double tp_lat_jerk =
        curv_factor * traj_points[i].v * traj_points[i].v * tp_omega;

    traj_max_lat_jerk = std::max(std::fabs(tp_lat_jerk), traj_max_lat_jerk);
    // if (std::fabs(tp_lat_jerk) > lat_jerk_thr) {
    //   lat_jerk_above_thr_count++;
    // }
    traj_max_lon_acc = std::max(std::fabs(traj_points[i].a), traj_max_lon_acc);
    traj_max_lon_jerk =
        std::max(std::fabs(traj_points[i].jerk), traj_max_lon_jerk);
  }

  // judge condition

  if (last_enable_lcc_) {
    lat_jerk_thr += config_.lat_jerk_hysteresis_value;
  }

  if (traj_max_lon_acc > config_.lon_acc_thr ||
      traj_max_lon_jerk > config_.lon_jerk_thr ||
      traj_max_lat_acc > config_.lat_acc_thr ||
      traj_max_lat_jerk > lat_jerk_thr) {
    ad_info.is_avaliable = false;
  }

  if (speed_limit_output->function_inhibited_near_roundabout()) {
    ad_info.is_avaliable = false;
  }
  last_enable_lcc_ = ad_info.is_avaliable;
  return true;
}

void EnableLCCHMIDecider::Reset() { last_enable_lcc_ = false; }

}  // namespace planning