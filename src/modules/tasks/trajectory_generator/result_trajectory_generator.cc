#include "tasks/trajectory_generator/result_trajectory_generator.h"

#include <cmath>
#include <cstddef>
#include <vector>

#include "environmental_model.h"
#include "geometry_math.h"
#include "modules/tasks/task_interface/lane_change_decider_output.h"

// #include "core/common/trace.h"
#include "debug_info_log.h"
#include "define/geometry.h"
#include "math/math_utils.h"
#include "math_lib.h"
#include "planning_context.h"

namespace planning {

using namespace std;
using namespace planning::planning_math;
using namespace pnc::mathlib;

ResultTrajectoryGenerator::ResultTrajectoryGenerator(
    const EgoPlanningConfigBuilder *config_builder, framework::Session *session)
    : Task(config_builder, session) {
  config_ = config_builder->cast<ResultTrajectoryGeneratorConfig>();
  name_ = "ResultTrajectoryGenerator";
  Init();
}

void ResultTrajectoryGenerator::Init() {
  const int N = config_.trajectory_time_length / config_.planning_dt;
  t_vec_.resize(N + 1);
  s_vec_.resize(N + 1);
  l_vec_.resize(N + 1);
  curvature_vec_.resize(N + 1);
  dkappa_vec_.resize(N + 1);
  ddkappa_vec_.resize(N + 1);
  lat_acc_vec_.resize(N + 1);
  lat_jerk_vec_.resize(N + 1);
}

bool ResultTrajectoryGenerator::Execute() {
  ILOG_DEBUG << "=======ResultTrajectoryGenerator=======";

  if (!PreCheck()) {
    ILOG_DEBUG << "PreCheck failed";
    return false;
  }
  std::fill(t_vec_.begin(), t_vec_.end(), 0);
  std::fill(s_vec_.begin(), s_vec_.end(), 0);
  std::fill(l_vec_.begin(), l_vec_.end(), 0);
  std::fill(curvature_vec_.begin(), curvature_vec_.end(), 0);
  std::fill(dkappa_vec_.begin(), dkappa_vec_.end(), 0);
  std::fill(ddkappa_vec_.begin(), ddkappa_vec_.end(), 0);
  std::fill(lat_acc_vec_.begin(), lat_acc_vec_.end(), 0);
  std::fill(lat_jerk_vec_.begin(), lat_jerk_vec_.end(), 0);

  auto start_time = IflyTime::Now_ms();

  // const auto &location_valid =
  // session_->environmental_model().location_valid();
  bool res = false;
  res = TrajectoryGenerator();
  UpdateTurnSignal();
  UpdateHMIInfo();
  auto end_time = IflyTime::Now_ms();
  JSON_DEBUG_VALUE("TrajectoryGeneratorCostTime", end_time - start_time);
  return res;
}

bool ResultTrajectoryGenerator::TrajectoryGenerator() {
  auto &ego_planning_result =
      session_->mutable_planning_context()->mutable_planning_result();

  // Step 1) get x,y of trajectory points
  auto &traj_points = ego_planning_result.traj_points;
  ego_planning_result.raw_traj_points = traj_points;
  std::copy(traj_points.begin(), traj_points.end(),
            ego_planning_result.raw_traj_points.begin());
  // const auto &num_point = traj_points.size();
  const auto &lateral_motion_planning_input =
      DebugInfoManager::GetInstance()
          .GetDebugInfoPb()
          ->lateral_motion_planning_input();
  double curv_factor =
      lateral_motion_planning_input.curv_factor();
  auto &motion_planner_output =
      session_->mutable_planning_context()->mutable_motion_planner_output();
  pnc::mathlib::spline s_t_spline;
  pnc::mathlib::spline l_t_spline;

  pnc::mathlib::spline curvature_t_spline;
  pnc::mathlib::spline dkappa_t_spline;
  pnc::mathlib::spline ddkappa_t_spline;

  auto const N = traj_points.size();
  double traj_max_lat_acc = 0.0;
  double traj_max_lat_jerk = 0.0;
  double traj_max_lon_acc = 0.0;
  double traj_max_lon_jerk = 0.0;

  const auto &reference_path_ptr = session_->planning_context()
                                       .lane_change_decider_output()
                                       .coarse_planning_info.reference_path;
  const auto &frenet_coord = reference_path_ptr->get_frenet_coord();
  const double tp_init_s = traj_points.front().s;
  for (size_t i = 0; i < traj_points.size(); i++) {
    if (config_.is_pwj_planning) {
      Point2D frenet_pt{traj_points[i].s, traj_points[i].l};
      Point2D cart_pt;
      if (!frenet_coord->SLToXY(frenet_pt, cart_pt)) {
        ILOG_ERROR << "ResultTrajectoryGenerator::execute, transform failed";
        return false;
      }

      traj_points[i].x = cart_pt.x;
      traj_points[i].y = cart_pt.y;

      ILOG_DEBUG << "result traj_point s=" << traj_points[i].s
                 << ", l=" << traj_points[i].l
                 << ", x=" << traj_points[i].x
                 << ", y=" << traj_points[i].y;
    }
    t_vec_[i] = traj_points[i].t;
    s_vec_[i] = traj_points[i].s;
    l_vec_[i] = traj_points[i].l;
    double tp_curvature =
        motion_planner_output.curv_s_spline(traj_points[i].s - tp_init_s);
    curvature_vec_[i] = tp_curvature;
    double tp_dcurvature =
        motion_planner_output.d_curv_s_spline(traj_points[i].s - tp_init_s);
    dkappa_vec_[i] = tp_dcurvature;
    ddkappa_vec_[i] = traj_points[i].ddkappa;
    double tp_delta =
        motion_planner_output.delta_s_spline(traj_points[i].s - tp_init_s);
    double tp_lat_acc =
        curv_factor * traj_points[i].v * traj_points[i].v * tp_delta;
    lat_acc_vec_[i] = tp_lat_acc;
    traj_max_lat_acc = std::max(std::fabs(tp_lat_acc), traj_max_lat_acc);
    double tp_omega =
        motion_planner_output.omega_s_spline(traj_points[i].s - tp_init_s);
    double tp_lat_jerk =
        curv_factor * traj_points[i].v * traj_points[i].v * tp_omega;
    lat_jerk_vec_[i] = tp_lat_jerk;
    traj_max_lat_jerk = std::max(std::fabs(tp_lat_jerk), traj_max_lat_jerk);
    traj_max_lon_acc = std::max(std::fabs(traj_points[i].a), traj_max_lon_acc);
    traj_max_lon_jerk =
        std::max(std::fabs(traj_points[i].jerk), traj_max_lon_jerk);
  }
  // JSON_DEBUG_VECTOR("traj_s_vec", s_vec, 3)
  s_t_spline.set_points(t_vec_, s_vec_);
  l_t_spline.set_points(t_vec_, l_vec_);

  curvature_t_spline.set_points(t_vec_, curvature_vec_);
  dkappa_t_spline.set_points(t_vec_, dkappa_vec_);
  ddkappa_t_spline.set_points(t_vec_, ddkappa_vec_);
  double lat_jerk_thr = config_.lat_jerk_thr;
  const bool &ramp_scene =
    session_->planning_context()
            .general_lateral_decider_output()
            .ramp_scene;
  if (ramp_scene) {
    lat_jerk_thr = config_.ramp_lat_jerk_thr;
  }
  // dynamic lat jerk thr
  if (config_.use_dynamic_lat_jerk_thr) {
    lat_jerk_thr =
        lateral_motion_planning_input.jerk_bound();
  }
  // judge condition
  auto &ad_info =
      session_->mutable_planning_context()
              ->mutable_planning_hmi_info()
              ->ad_info;
  if ((traj_max_lat_acc > config_.lat_acc_thr) ||
      (traj_max_lat_jerk > lat_jerk_thr) ||
      (traj_max_lon_acc > config_.lon_acc_thr) ||
      (traj_max_lon_jerk > config_.lon_jerk_thr)) {
    ad_info.is_avaliable = false;
  } else {
    ad_info.is_avaliable = true;
  }
  // Step 2) get dense trajectory points

  std::vector<TrajectoryPoint> dense_traj_points;
  int dense_num_points =
      int(traj_points.back().t / config_.planning_result_delta_time) + 1;

  for (int j = 0; j < dense_num_points; ++j) {
    TrajectoryPoint traj_pt;
    double t = j * config_.planning_result_delta_time;

    // traj_pt.t = init_point_relative_time + t;
    traj_pt.t = t;
    traj_pt.s = s_t_spline(t);
    traj_pt.x = motion_planner_output.x_t_spline(t);
    traj_pt.y = motion_planner_output.y_t_spline(t);
    traj_pt.v = motion_planner_output.v_t_spline(t);
    traj_pt.a = motion_planner_output.a_t_spline(t);
    traj_pt.jerk = motion_planner_output.j_t_spline(t);
    traj_pt.l = l_t_spline(t);
    traj_pt.heading_angle = motion_planner_output.theta_t_spline(t);
    traj_pt.curvature = curvature_t_spline(t);
    traj_pt.dkappa = dkappa_t_spline(t);
    traj_pt.ddkappa = ddkappa_t_spline(t);
    traj_pt.frenet_valid = true;
    dense_traj_points.push_back(std::move(traj_pt));
  }

  // Step 3) extends to max length if needed
  // double desired_length = planning_init_point.frenet_state.s + 1.0;

  // while (dense_traj_points.back().s < desired_length) {
  //   auto traj_pt = dense_traj_points.back();
  //   traj_pt.s += 0.2;
  //   traj_pt.t += config_.planning_result_delta_time;

  //   Point2D frenet_pt{traj_pt.s, traj_pt.l};
  //   Point2D cart_pt;
  //   if (!frenet_coord->SLToXY(frenet_pt, cart_pt)) {
  //     return false;
  //   }

  //   traj_pt.x = cart_pt.x;
  //   traj_pt.y = cart_pt.y;
  //   dense_traj_points.emplace_back(std::move(traj_pt));
  // }

  // const auto N_ext = dense_traj_points.size();
  // std::vector<double> traj_x_vec(N_ext);
  // std::vector<double> traj_y_vec(N_ext);

  // for (size_t i = 0; i < N_ext; ++i) {
  //   traj_x_vec[i] = dense_traj_points[i].x;
  //   traj_y_vec[i] = dense_traj_points[i].y;
  // }

  // JSON_DEBUG_VECTOR("traj_x_vec", traj_x_vec, 3)
  // JSON_DEBUG_VECTOR("traj_y_vec", traj_y_vec, 3)

  ego_planning_result.traj_points = std::move(dense_traj_points);

  return true;
}

bool ResultTrajectoryGenerator::RealtimeTrajectoryGenerator() {
  bool enable_lat_traj = config_.enable_lat_traj;

  auto &ego_planning_result =
      session_->mutable_planning_context()->mutable_planning_result();

  // Step 1) get x,y of trajectory points
  auto &traj_points = ego_planning_result.traj_points;
  auto &motion_planner_output =
      session_->mutable_planning_context()->mutable_motion_planner_output();

  pnc::mathlib::spline s_t_spline;
  pnc::mathlib::spline curvature_t_spline;
  pnc::mathlib::spline dkappa_t_spline;
  pnc::mathlib::spline ddkappa_t_spline;

  auto const N = traj_points.size();

  std::vector<double> t_vec(N);
  std::vector<double> s_vec(N);
  std::vector<double> curvature_vec(N);
  std::vector<double> dkappa_vec(N);
  std::vector<double> ddkappa_vec(N);

  for (size_t i = 0; i < traj_points.size(); i++) {
    t_vec[i] = traj_points[i].t;
    s_vec[i] = traj_points[i].s;
    curvature_vec[i] = traj_points[i].curvature;
    dkappa_vec[i] = traj_points[i].dkappa;
    ddkappa_vec[i] = traj_points[i].ddkappa;
  }

  s_t_spline.set_points(t_vec, s_vec);

  curvature_t_spline.set_points(t_vec, curvature_vec);
  dkappa_t_spline.set_points(t_vec, dkappa_vec);
  ddkappa_t_spline.set_points(t_vec, ddkappa_vec);

  // combination of genereted trajectory and reference without localization
  const double lat_err_norm =
      std::fabs(motion_planner_output.ref_y_t_spline(0.0));
  static const std::vector<double> lat_err_norm_tab = {0.0, 0.3, 0.5, 100.0};
  static const std::vector<double> alpha_tab = {1.0, 1.0, 0.0, 0.0};

  double alpha = 1.0;
  if (enable_lat_traj) {
    // 使能横向轨迹，采用轨迹和参考线合并的形式
    alpha = pnc::mathlib::Interp1(lat_err_norm_tab, alpha_tab, lat_err_norm);
  }

  // Step 2) get dense trajectory points
  std::vector<TrajectoryPoint> dense_traj_points;
  size_t dense_num_points =
      int(traj_points.back().t / config_.planning_result_delta_time) + 1;
  dense_traj_points.reserve(dense_num_points);

  for (int j = 0; j < dense_num_points; ++j) {
    TrajectoryPoint traj_pt;
    double t = j * config_.planning_result_delta_time;

    traj_pt.t = t;
    traj_pt.s = s_t_spline(t);

    // combination of genereted trajectory and reference considering x and y
    traj_pt.x = motion_planner_output.x_t_spline(t) * (1.0 - alpha) +
                motion_planner_output.ref_x_t_spline(t) * alpha;

    traj_pt.y = motion_planner_output.y_t_spline(t) * (1.0 - alpha) +
                motion_planner_output.ref_y_t_spline(t) * alpha;

    traj_pt.v = motion_planner_output.v_t_spline(t);
    traj_pt.a = motion_planner_output.a_t_spline(t);
    traj_pt.heading_angle = motion_planner_output.theta_t_spline(t);
    traj_pt.curvature = curvature_t_spline(t);
    traj_pt.dkappa = dkappa_t_spline(t);
    traj_pt.ddkappa = ddkappa_t_spline(t);
    traj_pt.frenet_valid = true;
    dense_traj_points.emplace_back(std::move(traj_pt));
  }

  const auto N_ext = dense_traj_points.size();
  std::vector<double> traj_x_vec(N_ext);
  std::vector<double> traj_y_vec(N_ext);

  for (size_t i = 0; i < N_ext; ++i) {
    traj_x_vec[i] = dense_traj_points[i].x;
    traj_y_vec[i] = dense_traj_points[i].y;
  }

  // JSON_DEBUG_VECTOR("traj_x_vec", traj_x_vec, 3)
  // JSON_DEBUG_VECTOR("traj_y_vec", traj_y_vec, 3)

  ego_planning_result.traj_points = dense_traj_points;

  return true;
}

void ResultTrajectoryGenerator::UpdateTurnSignal() {
  auto &planning_result =
      session_->mutable_planning_context()->mutable_planning_result();
  planning_result.turn_signal = RequestType::NO_CHANGE;
  bool active = session_->environmental_model().GetVehicleDbwStatus();
  if (!active) {
    // planning_result.turn_signal = RequestType::NO_CHANGE;
    return;
  }
  const auto &lane_borrow_decider_output =
      session_->planning_context().lane_borrow_decider_output();
  bool turn_signal_on_from_lane_borrow =
      lane_borrow_decider_output.lane_borrow_state ==
          LaneBorrowStatus::kLaneBorrowDriving ||
      lane_borrow_decider_output.lane_borrow_state ==
          LaneBorrowStatus::kLaneBorrowCrossing;
  const auto &lane_change_decider_output =
      session_->planning_context().lane_change_decider_output();
  bool turn_signal_from_ramp_direction =
      lane_change_decider_output.dir_turn_signal_road_to_ramp !=
      RampDirection::RAMP_NONE;
  bool turn_signal_from_lane_change =
      lane_change_decider_output.lc_request != 0;
  if (turn_signal_from_lane_change) {
    planning_result.turn_signal = lane_change_decider_output.lc_request == 1
                                      ? RequestType::LEFT_CHANGE
                                      : RequestType::RIGHT_CHANGE;
    return;
  }
  if (turn_signal_from_ramp_direction) {
    planning_result.turn_signal =
        lane_change_decider_output.dir_turn_signal_road_to_ramp ==
                RampDirection::RAMP_ON_LEFT
            ? RequestType::LEFT_CHANGE
            : RequestType::RIGHT_CHANGE;
    return;
  }

  if (turn_signal_on_from_lane_borrow) {
    planning_result.turn_signal = lane_borrow_decider_output.borrow_direction ==
                                          BorrowDirection::LEFT_BORROW
                                      ? RequestType::LEFT_CHANGE
                                      : RequestType::RIGHT_CHANGE;
    return;
  }
}

void ResultTrajectoryGenerator::UpdateHMIInfo() {
  const auto &lane_change_decider_output =
      session_->planning_context().lane_change_decider_output();
  const auto &route_info_output =
      session_->environmental_model().get_route_info()->get_route_info_output();
  auto &ad_info = session_->mutable_planning_context()
                      ->mutable_planning_hmi_info()
                      ->ad_info;
  const auto &ego_state_manager =
      session_->environmental_model().get_ego_state_manager();
  ad_info.cruise_speed = ego_state_manager->ego_v_cruise();
  ad_info.lane_change_direction =
      (iflyauto::LaneChangeDirection)lane_change_decider_output.lc_request;
  // update LaneChangeStatus
  const auto curr_state = lane_change_decider_output.curr_state;
  const auto lasr_frame_state = session_->planning_context()
                                    .lane_change_decider_output()
                                    .coarse_planning_info.source_state;
  static double lc_complete_to_lk_time = 0.0;
  if (curr_state == kLaneKeeping) {
    if (lasr_frame_state == kLaneChangeComplete) {
      lc_complete_to_lk_time = IflyTime::Now_ms();
      ad_info.lane_change_status =
          iflyauto::LaneChangeStatus::LC_STATE_COMPLETE;
    } else {
      ad_info.lane_change_status =
          iflyauto::LaneChangeStatus::LC_STATE_NO_CHANGE;

      if (session_->is_hpp_scene()) {
        // for HPP turn signal road to ramp
        const auto hpp_turn_signal = lane_change_decider_output.hpp_turn_signal;
        if (hpp_turn_signal == NO_CHANGE) {
          ad_info.lane_change_status =
              iflyauto::LaneChangeStatus::LC_STATE_NO_CHANGE;
        } else if (hpp_turn_signal == LEFT_CHANGE) {
          ad_info.lane_change_direction =
              iflyauto::LaneChangeDirection::LC_DIR_LEFT;
        } else if (hpp_turn_signal == RIGHT_CHANGE) {
          ad_info.lane_change_direction =
              iflyauto::LaneChangeDirection::LC_DIR_RIGHT;
        }
      } else {
        // for NOA turn signal road to ramp
        const auto dir_turn_signal_road_to_ramp =
            lane_change_decider_output.dir_turn_signal_road_to_ramp;
        if (dir_turn_signal_road_to_ramp == RAMP_NONE) {
          ad_info.lane_change_status =
              iflyauto::LaneChangeStatus::LC_STATE_NO_CHANGE;
        } else if (dir_turn_signal_road_to_ramp == RAMP_ON_LEFT) {
          ad_info.lane_change_direction =
              iflyauto::LaneChangeDirection::LC_DIR_LEFT;
        } else if (dir_turn_signal_road_to_ramp == RAMP_ON_RIGHT) {
          ad_info.lane_change_direction =
              iflyauto::LaneChangeDirection::LC_DIR_RIGHT;
        }
      }
    }
  } else if (curr_state == kLaneChangePropose) {
    ad_info.lane_change_status = iflyauto::LaneChangeStatus::LC_STATE_WAITING;
  } else if (curr_state == kLaneChangeExecution) {
    ad_info.lane_change_status = iflyauto::LaneChangeStatus::LC_STATE_STARTING;
  } else if (curr_state == kLaneChangeComplete) {
    ad_info.lane_change_status = iflyauto::LaneChangeStatus::LC_STATE_STARTING;
  } else if (curr_state == kLaneChangeCancel || curr_state == kLaneChangeHold) {
    ad_info.lane_change_status = iflyauto::LaneChangeStatus::LC_STATE_CANCELLED;
  }

  // update StatusUpdateReason
  const auto int_request_cancel_reason =
      lane_change_decider_output.int_request_cancel_reason;
  const auto lc_invalid_reason = lane_change_decider_output.lc_invalid_reason;
  const auto lc_back_reason = lane_change_decider_output.lc_back_reason;
  if (int_request_cancel_reason == SOLID_LC) {
    ad_info.status_update_reason =
        iflyauto::StatusUpdateReason::STATUS_UPDATE_REASON_SOLID_LINE;
    // 暂时为了满足实线变道时打灯合planing_hmi的提示需求
    // 在此更新变道状态和变道方向的值！！！！！！！
    //  TODO(fengwang31):在变道过程中，遇到实线取消了，是否需要发出方向？
    ad_info.lane_change_direction =
        (iflyauto::LaneChangeDirection)
            lane_change_decider_output.ilc_virtual_req;
    ad_info.lane_change_status = iflyauto::LaneChangeStatus::LC_STATE_NO_CHANGE;
  } else if (int_request_cancel_reason == MANUAL_CANCEL) {
    ad_info.status_update_reason =
        iflyauto::StatusUpdateReason::STATUS_UPDATE_REASON_MANUAL_CANCEL;
  } else if (lc_invalid_reason == "side view invalid" ||
             lc_invalid_reason == "front view invalid" ||
             lc_back_reason == "side view back" ||
             lc_back_reason == "front view back" ||
             lc_back_reason == "but back cnt below threshold") {
    ad_info.status_update_reason =
        iflyauto::StatusUpdateReason::STATUS_UPDATE_REASON_SIDE_VEH;
    iflyauto::ObstacleInfo obstacle;
    obstacle.id = lane_change_decider_output.lc_invalid_track.track_id;
    ad_info.obstacle_info[0] = obstacle;
    ad_info.obstacle_info_size = 1;
  } else {
    ad_info.status_update_reason =
        iflyauto::StatusUpdateReason::STATUS_UPDATE_REASON_NONE;
  }

  // update LaneChangeReason
  const auto lc_request_source = lane_change_decider_output.lc_request_source;
  if (lc_request_source == NO_REQUEST) {
    ad_info.lane_change_reason = iflyauto::LaneChangeReason::LC_REASON_NONE;
  } else if (lc_request_source == INT_REQUEST) {
    ad_info.lane_change_reason = iflyauto::LaneChangeReason::LC_REASON_MANUAL;
  } else if (lc_request_source == OVERTAKE_REQUEST) {
    ad_info.lane_change_reason =
        iflyauto::LaneChangeReason::LC_REASON_SLOWING_VEH;
  } else if (lc_request_source == MAP_REQUEST) {
    if (route_info_output.dis_to_ramp <
        route_info_output.distance_to_first_road_merge) {
      ad_info.lane_change_reason = iflyauto::LaneChangeReason::LC_REASON_SPLIT;
    } else {
      ad_info.lane_change_reason = iflyauto::LaneChangeReason::LC_REASON_MERGE;
    }
  } else if (lc_request_source == MERGE_REQUEST) {
    ad_info.lane_change_reason = iflyauto::LaneChangeReason::LC_REASON_MERGE;
  }
  // update route info
  if (!route_info_output.is_on_ramp) {
    ad_info.distance_to_ramp = route_info_output.dis_to_ramp;
  } else {
    ad_info.distance_to_ramp = NL_NMAX;
  }
  ad_info.distance_to_split = route_info_output.distance_to_first_road_split;
  if (route_info_output.is_ramp_merge_to_road_on_expressway) {
    ad_info.distance_to_merge = route_info_output.distance_to_first_road_merge;
  } else {
    ad_info.distance_to_merge = NL_NMAX;
  }
  ad_info.distance_to_toll_station = route_info_output.distance_to_toll_station;
  ad_info.noa_exit_warning_level_distance =
      route_info_output.distance_to_route_end;
  // ad_info.distance_to_tunnel = ;  //
  // ad_info.is_within_hdmap = ;     //
  const int ramp_direction = route_info_output.ramp_direction;
  ad_info.ramp_direction = (iflyauto::RampDirection)ramp_direction;

  const auto &fix_reference_path =
      lane_change_decider_output.coarse_planning_info.reference_path;
  if (fix_reference_path != nullptr) {
    ad_info.dis_to_reference_line =
        std::abs(fix_reference_path->get_frenet_ego_state().l() * 100);
    ad_info.angle_to_roaddirection =
        fix_reference_path->get_frenet_ego_state().heading_angle();
  }

  ad_info.is_in_sdmaproad = route_info_output.is_in_sdmaproad;
  if (route_info_output.is_ego_on_expressway_hmi) {
    ad_info.road_type = iflyauto::DrivingRoadType::DRIVING_ROAD_TYPE_HIGHWAY;
    // update RampPassSts
    // 高速主路上距离匝道距离小于200m，且不在一分二车道的场景
    if (route_info_output.dis_to_ramp < 200 &&
        lane_change_decider_output.dir_turn_signal_road_to_ramp == RAMP_NONE &&
        !route_info_output.is_on_ramp) {
      if (ramp_direction == iflyauto::RAMP_LEFT &&
          !lane_change_decider_output.is_ego_on_leftmost_lane) {
        ad_info.ramp_pass_sts = iflyauto::RAMP_PASS_STS_READYTOMISS;
      } else if (ramp_direction == iflyauto::RAMP_RIGHT &&
                 !lane_change_decider_output.is_ego_on_rightmost_lane) {
        ad_info.ramp_pass_sts = iflyauto::RAMP_PASS_STS_READYTOMISS;
      } else {
        ad_info.ramp_pass_sts = iflyauto::RAMP_PASS_STS_NONE;
      }
    } else {
      ad_info.ramp_pass_sts = iflyauto::RAMP_PASS_STS_NONE;
    }
  } else if (route_info_output.is_ego_on_city_expressway_hmi) {
    ad_info.road_type = iflyauto::DrivingRoadType::DRIVING_ROAD_TYPE_OVERPASS;
    // update RampPassSts
    // 城区主路上距离匝道距离小于50m，且不在一分二车道的场景
    if (route_info_output.dis_to_ramp < 50 &&
        lane_change_decider_output.dir_turn_signal_road_to_ramp == RAMP_NONE &&
        !route_info_output.is_on_ramp) {
      if (ramp_direction == iflyauto::RAMP_LEFT &&
          !lane_change_decider_output.is_ego_on_leftmost_lane) {
        ad_info.ramp_pass_sts = iflyauto::RAMP_PASS_STS_READYTOMISS;
      } else if (ramp_direction == iflyauto::RAMP_RIGHT &&
                 !lane_change_decider_output.is_ego_on_rightmost_lane) {
        ad_info.ramp_pass_sts = iflyauto::RAMP_PASS_STS_READYTOMISS;
      } else {
        ad_info.ramp_pass_sts = iflyauto::RAMP_PASS_STS_NONE;
      }
    } else {
      ad_info.ramp_pass_sts = iflyauto::RAMP_PASS_STS_NONE;
    }
  } else {
    ad_info.road_type = iflyauto::DrivingRoadType::DRIVING_ROAD_TYPE_NONE;
    ad_info.ramp_pass_sts = iflyauto::RAMP_PASS_STS_NONE;
  }
  ad_info.reference_line_msg = session_->environmental_model()
                                   .get_virtual_lane_manager()
                                   ->get_current_lane()
                                   ->get_reference_line_msg();
  ad_info.landing_point.heading = 0.0;
  ad_info.landing_point.relative_pos.x = 0.0;
  ad_info.landing_point.relative_pos.y = 0.0;
  ad_info.landing_point.relative_pos.z = 0.0;
  JSON_DEBUG_VALUE("ramp_pass_sts", (int)ad_info.ramp_pass_sts)
  double lc_complete_time_period = IflyTime::Now_ms() - lc_complete_to_lk_time;
  static constexpr double kMaxTimeToCompleteLaneChange = 5000.0;  // ms
  bool is_lane_keeping = curr_state == kLaneKeeping;
  bool is_time_out = lc_complete_time_period > kMaxTimeToCompleteLaneChange;
  iflyauto::LandingPoint landing_point;
  if (!is_time_out && is_lane_keeping) {
    landing_point = CalculateLandingPoint(true, lane_change_decider_output);
  } else if (curr_state == kLaneChangePropose ||
             curr_state == kLaneChangeExecution ||
             curr_state == kLaneChangeComplete ||
             curr_state == kLaneChangeCancel || curr_state == kLaneChangeHold) {
    landing_point = CalculateLandingPoint(false, lane_change_decider_output);
    // }
  } else {
    landing_point = CalculateLandingPoint(true, lane_change_decider_output);
  }
  ad_info.landing_point = landing_point;
  return;
}

iflyauto::LandingPoint ResultTrajectoryGenerator::CalculateLandingPoint(
    bool is_lane_keeping,
    const LaneChangeDeciderOutput &lane_change_decider_output) {
  iflyauto::LandingPoint landing_point;
  landing_point.relative_pos.x = 0.0;
  landing_point.relative_pos.y = 0.0;
  landing_point.relative_pos.z = 0.0;
  landing_point.heading = 0.0;
  std::shared_ptr<ReferencePath> target_reference = nullptr;
  double l_velocity = 1.4;
  auto ego_state_manager =
      session_->environmental_model().get_ego_state_manager();
  double ego_v = std::fmax(1.0, ego_state_manager->ego_v_cruise() * 0.2);
  double l_min = 0.15;
  if (is_lane_keeping) {
    target_reference = session_->environmental_model()
                           .get_reference_path_manager()
                           ->get_reference_path_by_current_lane();

  } else {
    int target_reference_virtual_id =
        lane_change_decider_output.target_lane_virtual_id;
    target_reference =
        session_->environmental_model()
            .get_reference_path_manager()
            ->get_reference_path_by_lane(target_reference_virtual_id, false);
  }
  if (target_reference != nullptr) {
    double l = std::fabs(target_reference->get_frenet_ego_state().l());
    // double t_ratio = std::fabs(l) / std::fabs(l_max);
    double t = std::fmax(0.0, l - l_min) / l_velocity;

    if (l < l_min) {
      t = 0.0;
    }

    Point2D cart_point;
    if (!target_reference->get_frenet_coord()->SLToXY(
            Point2D(target_reference->get_frenet_ego_state().s() + t * ego_v,
                    0),
            cart_point)) {
      return landing_point;
    }
    const auto &ego_pose = ego_state_manager->ego_pose();
    const double theta_ori = ego_pose.theta;
    double landing_point_theta_global = 0;
    ReferencePathPoint reference_path_point{};
    if (target_reference->get_reference_point_by_lon(
            target_reference->get_frenet_ego_state().s() + t * ego_v,
            reference_path_point)) {
      landing_point_theta_global = reference_path_point.path_point.theta();
    }
    Eigen::Vector2d pos_n_ori(ego_pose.x, ego_pose.y);
    pnc::geometry_lib::GlobalToLocalTf global_to_local_tf(pos_n_ori, theta_ori);
    Eigen::Vector2d p_n(cart_point.x, cart_point.y);
    Eigen::Vector2d landing_point_body = global_to_local_tf.GetPos(p_n);
    const double landing_point_theta_local =
        global_to_local_tf.GetHeading(landing_point_theta_global);

    landing_point.relative_pos.x = landing_point_body.x();
    landing_point.relative_pos.y = landing_point_body.y();
    landing_point.relative_pos.z = 0;
    landing_point.heading = landing_point_theta_local;
  }
  return landing_point;
}

}  // namespace planning
