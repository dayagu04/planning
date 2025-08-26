#include "parking_scenario.h"

#include <bits/stdint-uintn.h>
#include <sys/types.h>

#include <algorithm>
#include <memory>
#include <string>

#include "apa_context.h"
#include "apa_debug_data.pb.h"
#include "apa_param_config.h"
#include "apa_slot.h"
#include "apa_utils.h"
#include "collision_detection/base_collision_detector.h"
#include "common.pb.h"
#include "common_c.h"
#include "debug_info_log.h"
#include "dp_speed_common.h"
#include "geometry_math.h"
#include "jerk_limited_traj_optimizer/jerk_limited_traj_optimizer.h"
#include "log_glog.h"
#include "narrow_space_decider.h"
#include "park_speed_limit_decider.h"
#include "parking_stop_decider.h"
#include "parking_task/parking_task.h"
#include "planning_plan_c.h"
#include "pose2d.h"
#include "pwj_qp_speed_optimizer/piecewise_jerk_qp_speed_optimizer.h"
#include "rule_based_predictor/rule_based_predictor.h"
#include "speed/apa_speed_decision.h"
#include "sv_dp_optimizer/dp_speed_optimizer.h"
#include "traj_stitcher/apa_trajectory_stitcher.h"

namespace planning {
namespace apa_planner {
ParkingScenario::ParkingScenario() {}

ParkingScenario::ParkingScenario(
    const std::shared_ptr<ApaWorld>& apa_world_ptr) {
  SetApaWorldPtr(apa_world_ptr);
}

std::string ParkingScenario::GetName() { return ""; }

void ParkingScenario::Init() { return; }

void ParkingScenario::Reset() {
  memset(&planning_output_, 0, sizeof(planning_output_));
  memset(&apa_hmi_, 0, sizeof(apa_hmi_));
  frame_.Reset();
  current_path_point_global_vec_.clear();
  complete_path_point_global_vec_.clear();
  trajectory_.clear();

  return;
}

void ParkingScenario::Clear() {
  frame_.Reset();
  current_path_point_global_vec_.clear();
  complete_path_point_global_vec_.clear();
  trajectory_.clear();

  return;
}

void ParkingScenario::ScenarioRunning() {
  // run plan core
  ExcutePathPlanningTask();

  ExcuteSpeedPlanningTask();

  // generate planning output
  PublishPlanningTraj();

  GenPlanningHmiOutput();

  // log json debug
  Log();

  return;
}

void ParkingScenario::InitSimulation() {
  if (apa_world_ptr_->GetSimuParam().is_simulation &&
      apa_world_ptr_->GetSimuParam().is_reset) {
    Reset();
  }

  return;
}

void ParkingScenario::UpdateStuckTime() {
  const auto& param = apa_param.GetParam();
  const auto& measure_data_ptr = apa_world_ptr_->GetMeasureDataManagerPtr();
  const bool static_flag = measure_data_ptr->GetStaticFlag();
  const bool brake_flag = measure_data_ptr->GetBrakeFlag();
  const uint8_t plan_status = frame_.plan_stm.planning_status;
  const uint8_t pathplan_result = frame_.pathplan_result;
  const bool path_update_success =
      plan_status == ParkingStatus::PARKING_PLANNING &&
      pathplan_result == PathPlannerResult::PLAN_UPDATE;

  // update stuck time
  if (static_flag) {
    frame_.stuck_time += param.plan_time;
  } else {
    frame_.stuck_time = 0.0;
  }

  // update stuck by path time
  if (static_flag && !path_update_success) {
    frame_.stuck_path_time += param.plan_time;
  } else {
    frame_.stuck_path_time = 0.0;
  }

  if (static_flag && !path_update_success) {
    if (frame_.remain_dist_obs < param.max_replan_remain_dist) {
      // obs here
      if (frame_.stuck_by_dynamic_obs || frame_.stuck_dynamic_obs_time > 1e-3) {
        frame_.stuck_dynamic_obs_time += param.plan_time;
      } else {
        frame_.stuck_obs_time += param.plan_time;
      }
    } else {
      // obs disappear or far away
      // do nothing, keep the old value
    }
  } else {
    frame_.stuck_obs_time = 0.0;
    frame_.stuck_dynamic_obs_time = 0.0;
  }

  if (frame_.pathplan_result == PathPlannerResult::PLAN_FAILED) {
    frame_.replan_fail_time += apa_param.GetParam().plan_time;
  } else {
    frame_.replan_fail_time = 0.0;
  }

  return;
}

const bool ParkingScenario::CheckPaused() const {
  if (apa_world_ptr_->GetStateMachineManagerPtr()->GetStateMachine() ==
      ApaStateMachine::SUSPEND) {
    return true;
  } else {
    return false;
  }
}

const bool ParkingScenario::CheckPlanSkip() const {
  if ((frame_.plan_stm.planning_status == PARKING_FINISHED ||
       frame_.plan_stm.planning_status == PARKING_FAILED) &&
      !apa_world_ptr_->GetSimuParam().force_plan) {
    ILOG_INFO << "plan has been finished or failed, should skip";

    return true;
  } else {
    return false;
  }
}

void ParkingScenario::SetParkingStatus(uint8_t status) {
  if (status == PARKING_IDLE || status == PARKING_FAILED) {
    frame_.plan_stm.path_plan_success = false;
  } else if (status == PARKING_RUNNING || status == PARKING_GEARCHANGE ||
             status == PARKING_PLANNING || status == PARKING_FINISHED) {
    frame_.plan_stm.path_plan_success = true;
  }

  frame_.plan_stm.planning_status = status;
  return;
}

void ParkingScenario::PublishPlanningTraj() {
  pnc::geometry_lib::PathPoint current_ego_pose(
      apa_world_ptr_->GetMeasureDataManagerPtr()->GetPos(),
      apa_world_ptr_->GetMeasureDataManagerPtr()->GetHeading());

  ILOG_INFO << "frame_.plan_stm.planning_status = "
            << static_cast<int>(frame_.plan_stm.planning_status)
            << "  plan path pt size = "
            << current_path_point_global_vec_.size();

  if (frame_.plan_stm.planning_status == PARKING_FINISHED) {
    SetFinishedPlanningOutput(planning_output_, current_ego_pose);
  } else if (frame_.plan_stm.planning_status == PARKING_FAILED) {
    SetFailedPlanningOutput(planning_output_, current_ego_pose);
  } else if (frame_.plan_stm.planning_status == PARKING_PLANNING ||
             frame_.plan_stm.planning_status == PARKING_GEARCHANGE ||
             frame_.plan_stm.planning_status == PARKING_RUNNING ||
             frame_.plan_stm.planning_status == PARKING_PAUSED) {
    SetPlanningPath();
  } else if (frame_.plan_stm.planning_status == PARKING_IDLE) {
    SetIdlePlanningOutput(planning_output_, current_ego_pose);
  }

  if (frame_.plan_stm.planning_status == PARKING_IDLE ||
      frame_.plan_stm.planning_status == PARKING_FAILED ||
      frame_.plan_stm.planning_status == PARKING_FINISHED) {
    frame_.replan_flag = false;
    frame_.correct_path_for_limiter = false;
    frame_.ego_slot_info.Reset();
    apa_world_ptr_->GetCollisionDetectorPtr()->ClearObstacles();
  }

  ILOG_INFO << "gen plan output success.";
  return;
}

void ParkingScenario::TansformPreparePlanningTraj() {
  if (complete_path_point_global_vec_.empty()) {
    ILOG_INFO << "path is null";
    return;
  }

  auto publish_traj = &(planning_output_.trajectory);
  publish_traj->available = true;
  publish_traj->trajectory_type = iflyauto::TRAJECTORY_TYPE_TRAJECTORY_POINTS;
  publish_traj->target_reference.target_velocity = 0;

  double total_s = complete_path_point_global_vec_.back().s;
  double delta_point_s = total_s / PLANNING_TRAJ_POINTS_MAX_NUM;
  double path_s = 0.0;
  int path_point_id = 0;

  ILOG_INFO << "prepare plan path size "
            << complete_path_point_global_vec_.size() << ",s = " << total_s;

  for (const auto& pt : complete_path_point_global_vec_) {
    if (pt.s < path_s) {
      continue;
    }

    publish_traj->trajectory_points[path_point_id].x = pt.pos.x();
    publish_traj->trajectory_points[path_point_id].y = pt.pos.y();
    publish_traj->trajectory_points[path_point_id].heading_yaw = pt.heading;
    publish_traj->trajectory_points[path_point_id].curvature = pt.kappa;
    publish_traj->trajectory_points[path_point_id].t = 0;
    publish_traj->trajectory_points[path_point_id].distance = pt.s;
    publish_traj->trajectory_points[path_point_id].v = 0;
    publish_traj->trajectory_points[path_point_id].a = 0;
    publish_traj->trajectory_points[path_point_id].jerk = 0;

    path_s = pt.s + delta_point_s;
    path_point_id++;

    if (path_point_id >= PLANNING_TRAJ_POINTS_MAX_NUM) {
      break;
    }
  }

  publish_traj->trajectory_points_size =
      std::min(path_point_id, PLANNING_TRAJ_POINTS_MAX_NUM);

  publish_traj->target_reference.polynomial[0] = 0.0;

  return;
}

void ParkingScenario::GenPlanningHmiOutput() {
  memset(&apa_hmi_, 0, sizeof(apa_hmi_));

  if (frame_.plan_stm.planning_status == PARKING_PLANNING ||
      frame_.plan_stm.planning_status == PARKING_GEARCHANGE ||
      frame_.plan_stm.planning_status == PARKING_RUNNING) {
    apa_hmi_.remain_dist = frame_.remain_dist_path;
  }
  return;
}

void ParkingScenario::SetPlanningPath() {
  // planning_output_.Clear();
  memset(&planning_output_, 0, sizeof(planning_output_));
  planning_output_.planning_status.hpp_planning_status = iflyauto::HPP_RUNNING;
  planning_output_.planning_status.apa_planning_status =
      iflyauto::APA_IN_PROGRESS;

  // record last frame remain dist path
  frame_.remain_dist_path_last = frame_.remain_dist_path;

  // reset obs remain dist when gear shift
  if ((frame_.gear_command == pnc::geometry_lib::SEG_GEAR_DRIVE &&
       planning_output_.gear_command.gear_command_value ==
           iflyauto::GearCommandValue::GEAR_COMMAND_VALUE_REVERSE) ||
      (frame_.gear_command == pnc::geometry_lib::SEG_GEAR_REVERSE &&
       planning_output_.gear_command.gear_command_value ==
           iflyauto::GearCommandValue::GEAR_COMMAND_VALUE_DRIVE)) {
    frame_.remain_dist_obs = 2.68;
    frame_.remain_dist_col_det = 2.68;
  }

  // set fold and unfold mirror
  iflyauto::RearViewMirrorCommand* mirror_command =
      &planning_output_.rear_view_mirror_signal_command;
  mirror_command->available = true;
  if (frame_.mirror_command == MirrorCommand::FOLD) {
    mirror_command->rear_view_mirror_value = iflyauto::REAR_VIEW_MIRROR_FOLD;
  } else if (frame_.mirror_command == MirrorCommand::EXPAND) {
    mirror_command->rear_view_mirror_value = iflyauto::REAR_VIEW_MIRROR_UNFOLD;
  } else {
    mirror_command->rear_view_mirror_value = iflyauto::REAR_VIEW_MIRROR_NONE;
  }

  ILOG_INFO << "plan mirror = " << static_cast<int>(frame_.mirror_command);

  auto publish_traj = &(planning_output_.trajectory);
  publish_traj->available = true;
  publish_traj->trajectory_type = iflyauto::TRAJECTORY_TYPE_TRAJECTORY_POINTS;
  publish_traj->target_reference.target_velocity =
      apa_param.GetParam().speed_config.default_cruise_speed;

  if (!apa_param.GetParam().speed_config.enable_apa_speed_plan) {
    double total_s = current_path_point_global_vec_.back().s;
    double delta_point_s = total_s / PLANNING_TRAJ_POINTS_MAX_NUM;
    double path_s = 0.0;
    int path_point_id = 0;

    ILOG_INFO << " plan path size " << current_path_point_global_vec_.size()
              << ",s = " << total_s;

    for (const auto& pt : current_path_point_global_vec_) {
      if (pt.s < path_s) {
        continue;
      }

      publish_traj->trajectory_points[path_point_id].x = pt.pos.x();
      publish_traj->trajectory_points[path_point_id].y = pt.pos.y();
      publish_traj->trajectory_points[path_point_id].heading_yaw = pt.heading;
      publish_traj->trajectory_points[path_point_id].curvature = pt.kappa;
      publish_traj->trajectory_points[path_point_id].t = 0;
      publish_traj->trajectory_points[path_point_id].distance = pt.s;
      publish_traj->trajectory_points[path_point_id].v = 0;
      publish_traj->trajectory_points[path_point_id].a = 0;
      publish_traj->trajectory_points[path_point_id].jerk = 0;

      path_s = pt.s + delta_point_s;
      path_point_id++;

      if (path_point_id >= PLANNING_TRAJ_POINTS_MAX_NUM) {
        break;
      }
    }

    publish_traj->trajectory_points_size =
        std::min(path_point_id, PLANNING_TRAJ_POINTS_MAX_NUM);

    publish_traj->target_reference.polynomial[0] = 0.0;

    // send obs remain dist to control
    publish_traj->trajectory_points[0].distance =
        std::min(frame_.remain_dist_obs, frame_.remain_dist_slot_jump);

    // send slot occupation ratio to control
    publish_traj->trajectory_points[1].distance =
        apa_world_ptr_->GetSlotManagerPtr()
            ->GetEgoInfoUnderSlot()
            .slot_occupied_ratio;

    // send slot type to control
    publish_traj->trajectory_points[2].distance = static_cast<double>(
        apa_world_ptr_->GetSlotManagerPtr()->GetEgoInfoUnderSlot().slot_type);
    publish_traj->trajectory_points[3].distance = 0.0;
    publish_traj->trajectory_points[4].distance = frame_.remain_dist_col_det;
  } else {
    publish_traj->trajectory_points_size =
        std::min(trajectory_.size(), size_t(PLANNING_TRAJ_POINTS_MAX_NUM));

    for (size_t i = 0; i < publish_traj->trajectory_points_size; ++i) {
      const auto& point = trajectory_[i];
      publish_traj->trajectory_points[i].x = point.x();
      publish_traj->trajectory_points[i].y = point.y();
      publish_traj->trajectory_points[i].heading_yaw = point.theta();
      publish_traj->trajectory_points[i].curvature = point.kappa();
      publish_traj->trajectory_points[i].t = point.absolute_time();
      publish_traj->trajectory_points[i].distance = point.s();
      publish_traj->trajectory_points[i].v = point.vel();
      publish_traj->trajectory_points[i].a = point.acc();
      publish_traj->trajectory_points[i].jerk = point.jerk();
    }

    planning_output_.trajectory.target_reference.polynomial[0] =
        trajectory_.GetTerminalS();
  }

  // set plan gear cmd
  auto gear_command = &(planning_output_.gear_command);
  gear_command->available = true;
  if (frame_.gear_command == pnc::geometry_lib::SEG_GEAR_DRIVE) {
    gear_command->gear_command_value = iflyauto::GEAR_COMMAND_VALUE_DRIVE;
  } else {
    gear_command->gear_command_value = iflyauto::GEAR_COMMAND_VALUE_REVERSE;
  }
  ILOG_INFO << "gear command in planning output = "
            << static_cast<int>(gear_command->gear_command_value);

  RecordDebugPath();

  return;
}

const bool ParkingScenario::CheckStuckFailed() {
  const ApaParameters& param = apa_param.GetParam();
  if (frame_.stuck_dynamic_obs_time > 1e-3) {
    return std::max(frame_.stuck_time, frame_.stuck_dynamic_obs_time) >
           param.stuck_failed_by_dynamic_obs_time;
  } else {
    return frame_.stuck_time > param.stuck_failed_time;
  }
}

const bool ParkingScenario::CheckEgoPoseInBelieveObsArea(
    const double lat_expand, const double lon_expand,
    const double heading_err) {
  const geometry_lib::PathPoint& ego_pose =
      apa_world_ptr_->GetSlotManagerPtr()->GetEgoInfoUnderSlot().cur_pose;

  const ApaSlot& slot =
      apa_world_ptr_->GetSlotManagerPtr()->GetEgoInfoUnderSlot().slot;

  if ((slot.IsPointInCustomSlot(ego_pose.pos, lon_expand, lon_expand,
                                lat_expand, lat_expand, true) &&
       std::fabs(ego_pose.heading) * kRad2Deg < heading_err)) {
    return true;
  }

  return false;
}

const geometry_lib::PathPoint ParkingScenario::GetCarFrontPoseFromCarPose(
    const geometry_lib::PathPoint& pose) {
  geometry_lib::PathPoint front_pose = pose;
  front_pose.pos = pose.pos + (apa_param.GetParam().front_overhanging +
                               apa_param.GetParam().wheel_base) *
                                  geometry_lib::GenHeadingVec(pose.heading);
  return front_pose;
}

const double ParkingScenario::CalRemainDistFromPath() {
  double remain_dist = 15.0;

  if (frame_.is_replan_first) {
    return remain_dist;
  }

  if (frame_.spline_success) {
    double s_proj = 0.0;
    bool success = pnc::geometry_lib::CalProjFromSplineByBisection(
        0.0, frame_.current_path_length + frame_.path_extended_dist, s_proj,
        apa_world_ptr_->GetMeasureDataManagerPtr()->GetPos(), frame_.x_s_spline,
        frame_.y_s_spline);

    if (success == true) {
      remain_dist = frame_.current_path_length - s_proj;

      ILOG_INFO << "remain_dist = " << remain_dist << "  s_proj = " << s_proj
                << "  current_path_length = " << frame_.current_path_length;
    } else {
      ILOG_INFO << "remain_dist calculation error:input is error";
    }
  } else {
    ILOG_INFO << "remain_dist calculation error: path spline failed!";
  }

  return remain_dist;
}

const double ParkingScenario::CalRemainDistFromObs(
    const double static_lon_buffer, const double static_body_lat_buffer,
    const double static_mirror_lat_buffer, const double dynamic_lon_buffer,
    const double dynamic_body_lat_buffer,
    const double dynamic_mirror_lat_buffer, const bool only_check_mirror) {
  const ApaParameters& param = apa_param.GetParam();

  if (apa_world_ptr_->GetSlotManagerPtr()
          ->GetEgoInfoUnderSlot()
          .slot_disappear_flag) {
    ILOG_INFO << "slot disappear, should stop, set remain dist obs = 0.0168";
    return 0.0168;
  }

  const std::shared_ptr<UssObstacleAvoidance>& uss_obstacle_avoider_ptr =
      apa_world_ptr_->GetColDetInterfacePtr()->GetUssObsAvoidancePtr();

  const std::shared_ptr<GJKCollisionDetector>& gjk_col_det_ptr =
      apa_world_ptr_->GetColDetInterfacePtr()->GetGJKColDetPtr();

  if (only_check_mirror) {
    ColResult col_res = gjk_col_det_ptr->Update(
        apa_world_ptr_->GetPredictPathManagerPtr()->GetPredictPath(),
        static_body_lat_buffer, 0.0,
        GJKColDetRequest(false, false, CarBodyType::ONLY_MIRROR,
                         ApaObsMovementType::STATIC),
        true, static_mirror_lat_buffer);

    return col_res.remain_dist - static_lon_buffer;
  }

  if (param.enable_corner_uss_process) {
    uss_obstacle_avoider_ptr->Update();
  }

  const double uss_remain_dist =
      uss_obstacle_avoider_ptr->GetRemainDistInfo().remain_dist -
      static_lon_buffer;

  // check static obs, it can be radical
  GJKColDetRequest gjl_col_det_request(false, false, CarBodyType::NORMAL,
                                       ApaObsMovementType::STATIC);

  gjl_col_det_request.use_uss_pt = param.uss_config.use_uss_pt_cloud &&
                                   param.uss_config.use_uss_pt_for_speed;

  ColResult col_res = gjk_col_det_ptr->Update(
      apa_world_ptr_->GetPredictPathManagerPtr()->GetPredictPath(),
      static_body_lat_buffer, 0.0, gjl_col_det_request, true,
      static_mirror_lat_buffer);

  if (!col_res.col_flag) {
    col_res.remain_dist_static = frame_.remain_dist_path + 1.68;
  }

  const double obs_pt_remain_dist_static =
      col_res.remain_dist_static - static_lon_buffer;

  // check dynamic obs, it should be conservative
  gjl_col_det_request.movement_type = ApaObsMovementType::MOTION;
  gjl_col_det_request.use_uss_pt = false;
  col_res = gjk_col_det_ptr->Update(
      apa_world_ptr_->GetPredictPathManagerPtr()->GetPredictPath(),
      dynamic_body_lat_buffer, 0.0, gjl_col_det_request, true,
      dynamic_mirror_lat_buffer);

  if (!col_res.col_flag) {
    col_res.remain_dist_dynamic = frame_.remain_dist_path + 3.68;
  }

  const double obs_pt_remain_dist_dynamic =
      col_res.remain_dist_dynamic - dynamic_lon_buffer;

  JSON_DEBUG_VALUE("car_real_time_col_lat_buffer", static_body_lat_buffer)

  ILOG_INFO << "  enable_corner_uss_process = "
            << param.enable_corner_uss_process
            << "  uss remain dist = " << uss_remain_dist
            << "  obs_pt_remain_dist_static = " << obs_pt_remain_dist_static
            << "  obs_pt_remain_dist_dynamic = " << obs_pt_remain_dist_dynamic;

  if (param.enable_corner_uss_process) {
    frame_.stuck_by_dynamic_obs = false;
    return uss_remain_dist;
  } else {
    if (obs_pt_remain_dist_dynamic < obs_pt_remain_dist_static) {
      frame_.stuck_by_dynamic_obs = true;
      return obs_pt_remain_dist_dynamic;
    } else {
      frame_.stuck_by_dynamic_obs = false;
      return obs_pt_remain_dist_static;
    }
  }
}

const double ParkingScenario::CalRemainDistFromPlanPathDangerous(
    const double static_lon_buffer, const double static_lat_buffer) {
  if (current_path_point_global_vec_.empty()) {
    return 36.8;
  }

  GJKColDetRequest gjl_col_det_request(false, false, CarBodyType::NORMAL,
                                       ApaObsMovementType::STATIC);
  gjl_col_det_request.use_uss_pt = false;

  const std::shared_ptr<GJKCollisionDetector>& gjk_col_det_ptr =
      apa_world_ptr_->GetColDetInterfacePtr()->GetGJKColDetPtr();

  ColResult col_res =
      gjk_col_det_ptr->Update(current_path_point_global_vec_, static_lat_buffer,
                              static_lon_buffer, gjl_col_det_request);

  return col_res.remain_dist_static;
}

const bool ParkingScenario::PostProcessPath() {
  const size_t origin_trajectory_size = current_path_point_global_vec_.size();
  if (origin_trajectory_size < 2) {
    frame_.spline_success = false;
    ILOG_INFO << "error: origin_trajectory_size = " << origin_trajectory_size;
    frame_.plan_fail_reason = POST_PROCESS_PATH_POINT_SIZE;
    return false;
  }

  std::vector<double> x_vec;
  std::vector<double> y_vec;
  std::vector<double> heading_vec;
  std::vector<double> s_vec;
  std::vector<double> kappa_vec;
  x_vec.reserve(origin_trajectory_size + 1);
  y_vec.reserve(origin_trajectory_size + 1);
  heading_vec.reserve(origin_trajectory_size + 1);
  s_vec.reserve(origin_trajectory_size + 1);
  kappa_vec.reserve(origin_trajectory_size + 1);
  double s = 0.0;
  double ds = 0.0;
  for (size_t i = 0; i < origin_trajectory_size; ++i) {
    pnc::geometry_lib::PathPoint pt = current_path_point_global_vec_[i];
    if (i > 0) {
      pnc::geometry_lib::PathPoint pt_ = current_path_point_global_vec_[i - 1];
      ds = std::hypot(pt.pos.x() - pt_.pos.x(), pt.pos.y() - pt_.pos.y());
      if (ds < 1e-3) {
        continue;
      }
      s += ds;
    }
    x_vec.emplace_back(pt.pos.x());
    y_vec.emplace_back(pt.pos.y());
    heading_vec.emplace_back(pt.heading);
    s_vec.emplace_back(s);
    kappa_vec.emplace_back(pt.kappa);
  }

  size_t x_vec_size = x_vec.size();
  if (x_vec_size < 2) {
    frame_.spline_success = false;
    ILOG_INFO << "error: x_vec_size = " << x_vec.size();
    frame_.plan_fail_reason = POST_PROCESS_PATH_POINT_SIZE;
    return false;
  }

  // hack: insert line of 0.2m compensating control error to reduce gear change
  // num
  if (apa_world_ptr_->GetStateMachineManagerPtr()->GetStateMachine() ==
          ApaStateMachine::ACTIVE_IN_CAR_FRONT &&
      frame_.current_gear == geometry_lib::SEG_GEAR_REVERSE) {
    const Eigen::Vector2d start_point(x_vec[x_vec_size - 2],
                                      y_vec[x_vec_size - 2]);

    const Eigen::Vector2d end_point(x_vec[x_vec_size - 1],
                                    y_vec[x_vec_size - 1]);

    geometry_lib::PathPoint extend_point(end_point,
                                         heading_vec[x_vec_size - 1]);

    const double extend_length = 0.2;
    geometry_lib::CalExtendedPointByTwoPoints(start_point, end_point,
                                              extend_point.pos, 0.2);

    s += extend_length;
    x_vec.emplace_back(extend_point.pos.x());
    y_vec.emplace_back(extend_point.pos.y());
    heading_vec.emplace_back(extend_point.heading);
    s_vec.emplace_back(s);
    kappa_vec.emplace_back(0);
    x_vec_size = x_vec.size();
  }

  current_path_point_global_vec_.clear();
  current_path_point_global_vec_.resize(x_vec_size);
  pnc::geometry_lib::PathPoint point;
  for (size_t i = 0; i < x_vec_size; ++i) {
    point.pos << x_vec[i], y_vec[i];
    point.heading = heading_vec[i];
    point.s = s_vec[i];
    point.kappa = kappa_vec[i];
    current_path_point_global_vec_[i] = point;
  }

  frame_.current_path_length = s;

  // calculate the extended point and insert
  const Eigen::Vector2d start_point(x_vec[x_vec_size - 2],
                                    y_vec[x_vec_size - 2]);

  const Eigen::Vector2d end_point(x_vec[x_vec_size - 1], y_vec[x_vec_size - 1]);

  Eigen::Vector2d extended_point;

  bool success = pnc::geometry_lib::CalExtendedPointByTwoPoints(
      start_point, end_point, extended_point, frame_.path_extended_dist);

  if (success == false) {
    frame_.spline_success = false;
    ILOG_INFO << "fit line by spline error!";
    frame_.plan_fail_reason = POST_PROCESS_PATH_POINT_SAME;
    return false;
  }

  x_vec.emplace_back(extended_point.x());
  y_vec.emplace_back(extended_point.y());
  s_vec.emplace_back(s_vec.back() + frame_.path_extended_dist);

  frame_.x_s_spline.set_points(s_vec, x_vec);
  frame_.y_s_spline.set_points(s_vec, y_vec);

  frame_.spline_success = true;

  return true;
}

void ParkingScenario::CreateTasks() { return; }

void ParkingScenario::ThreadClearState() { return; }

void ParkingScenario::ScenarioTry() {
  // todo: use geometry method first, if no result, use hybrid astar.
  std::shared_ptr<ApaSlotManager> slot_manager =
      apa_world_ptr_->GetSlotManagerPtr();
  slot_manager->GetMutableEgoInfoUnderSlot()
      .slot.release_info_
      .release_state[SlotReleaseMethod::GEOMETRY_PLANNING_RELEASE] =
      SlotReleaseState::RELEASE;
  slot_manager->GetMutableEgoInfoUnderSlot()
      .slot.release_info_
      .release_state[SlotReleaseMethod::ASTAR_PLANNING_RELEASE] =
      SlotReleaseState::RELEASE;

  return;
}

void ParkingScenario::ExcuteSpeedPlanningTask() {
  if (!apa_param.GetParam().speed_config.enable_apa_speed_plan) {
    return;
  }

  // check planning status
  if (CheckPlanSkip()) {
    return;
  }

  if (current_path_point_global_vec_.empty()) {
    return;
  }

  // task: predictor
  std::shared_ptr<RuleBasedPredictor> predictor =
      std::make_shared<RuleBasedPredictor>();
  predictor->Execute(apa_world_ptr_->GetObstacleManagerPtr());

  double acc = apa_world_ptr_->GetMeasureDataManagerPtr()->GetAcceleration();
  if (apa_world_ptr_->GetMeasureDataManagerPtr()->GetVel() < 0.0) {
    acc = -apa_world_ptr_->GetMeasureDataManagerPtr()->GetAcceleration();
  }
  SVPoint ego_speed_point = SVPoint(
      0, std::fabs(apa_world_ptr_->GetMeasureDataManagerPtr()->GetVel()), acc);

  // task: update traj stitcher
  std::shared_ptr<ApaTrajectoryStitcher> traj_stitcher =
      std::make_shared<ApaTrajectoryStitcher>();
  traj_stitcher->Execute(
      apa_world_ptr_->GetMeasureDataManagerPtr()->GetPose(),
      current_path_point_global_vec_,
      apa_world_ptr_->GetMeasureDataManagerPtr()->GetFrontWheelAngle(),
      ego_speed_point, 0.0, trajectory_,
      pnc::geometry_lib::GetGearType(frame_.gear_command),
      apa_world_ptr_->GetMeasureDataManagerPtr()->GetGear());

  const SVPoint stitch_init_speed = traj_stitcher->GetStitchSpeed();

  // task: update stop decision
  std::shared_ptr<ParkingStopDecider> stop_decider =
      std::make_shared<ParkingStopDecider>(
          apa_world_ptr_->GetColDetInterfacePtr(),
          apa_world_ptr_->GetMeasureDataManagerPtr(),
          apa_world_ptr_->GetObstacleManagerPtr());

  stop_decider->Execute(
      stitch_init_speed, traj_stitcher->GetConstStitchPath(),
      apa_world_ptr_->GetPredictPathManagerPtr()->GetPredictPath(),
      pnc::geometry_lib::GetGearType(frame_.gear_command));

  // todo: will be retired
  if (apa_param.GetParam().speed_config.use_remain_dist) {
    double stop_s =
        std::min(frame_.remain_dist_obs, frame_.remain_dist_slot_jump);
    stop_decider->AddStopDecisionByDistance(
        stop_s,
        frame_.remain_dist_obs < frame_.remain_dist_slot_jump
            ? LonDecisionReason::REMAIN_DIST
            : LonDecisionReason::SLOT_POSE_CHANGE,
        traj_stitcher->GetConstStitchPath());
  }

  SpeedDecisions speed_decisions;
  const ParkLonDecision stop_decision = stop_decider->GetStopDecision();
  if (stop_decision.decision_type == LonDecisionType::STOP) {
    speed_decisions.decisions.emplace_back(stop_decision);
  }

  // task: update speed limit decision
  std::shared_ptr<ParkSpeedLimitDecider> speed_limit_decider =
      std::make_shared<ParkSpeedLimitDecider>(
          apa_world_ptr_->GetColDetInterfacePtr(),
          apa_world_ptr_->GetMeasureDataManagerPtr(),
          apa_world_ptr_->GetObstacleManagerPtr(),
          apa_world_ptr_->GetPredictPathManagerPtr());
  speed_limit_decider->Execute(traj_stitcher->GetMutableStitchPath(),
                               &speed_decisions);

  // task: generate dp speed
  std::shared_ptr<DpSpeedOptimizer> dp_speed_optimizer =
      std::make_shared<DpSpeedOptimizer>();
  dp_speed_optimizer->Excute(
      traj_stitcher->GetConstStitchPath(),
      apa_world_ptr_->GetMeasureDataManagerPtr()->GetPose(), stitch_init_speed,
      &speed_decisions, &speed_limit_decider->GetSpeedLimitProfile());

  // task: generate qp speed
  std::shared_ptr<PiecewiseJerkSpeedQPOptimizer> qp_speed_optimizer =
      std::make_shared<PiecewiseJerkSpeedQPOptimizer>();
  const SpeedData& dp_speed = dp_speed_optimizer->SpeedProfile();
  qp_speed_optimizer->Execute(stitch_init_speed,
                              &speed_limit_decider->GetSpeedLimitProfile(),
                              dp_speed, &speed_decisions);

  // task: generate jlt speed
  if (dp_speed_optimizer->GetExcuteState() != TaskExcuteState::SUCCESS ||
      qp_speed_optimizer->GetExcuteState() != TaskExcuteState::SUCCESS) {
    std::shared_ptr<JerkLimitedTrajOptimizer> jlt_optimizer =
        std::make_shared<JerkLimitedTrajOptimizer>();
    jlt_optimizer->Execute(stitch_init_speed, ego_speed_point,
                           traj_stitcher->GetConstStitchPath(),
                           &speed_decisions);
    traj_stitcher->CombineTrajBasedOnTime(jlt_optimizer->GetSpeedData(),
                                          trajectory_);
  } else {
    traj_stitcher->CombineTrajBasedOnTime(qp_speed_optimizer->GetSpeedData(),
                                          trajectory_);
  }

  trajectory_ = traj_stitcher->GetConstCombinedTraj();
  trajectory_.SetTerminalS(stop_decider->GetTerminalS());

  return;
}

const bool ParkingScenario::CheckReplan(const CheckReplanParams& check_params) {
  frame_.replan_reason = ReplanReason::NOT_REPLAN;

  if (frame_.is_replan_first) {
    ILOG_INFO << "first plan";
    frame_.replan_reason = ReplanReason::FIRST_PLAN;
    return true;
  }

  if (apa_world_ptr_->GetSimuParam().force_plan) {
    ILOG_INFO << "force plan";
    frame_.replan_reason = ReplanReason::FORCE_PLAN;
    return true;
  }

  if (CheckSegCompleted(check_params.replan_dist_path,
                        check_params.wait_time_path,
                        check_params.min_drive_dist)) {
    ILOG_INFO << "replan by current segment completed!";
    frame_.replan_reason = ReplanReason::SEG_COMPLETED_PATH;
    return true;
  }

  if (CheckObsStucked(check_params.replan_dist_obs, check_params.wait_time_obs,
                      check_params.min_drive_dist)) {
    ILOG_INFO << "replan by obs stucked!";
    frame_.replan_reason = ReplanReason::SEG_COMPLETED_OBS;
    return true;
  }

  if (CheckSlotJumpStucked(check_params.replan_dist_slot_jump,
                           check_params.wait_time_slot_jump,
                           check_params.min_drive_dist)) {
    ILOG_INFO << "replan by slot jump stucked!";
    frame_.replan_reason = ReplanReason::SEG_COMPLETED_SLOT_JUMP;
    return true;
  }

  if (CheckStuckTimeEnough(check_params.stuck_replan_time)) {
    ILOG_INFO << "replan by stuck!";
    frame_.replan_reason = ReplanReason::STUCKED;
    return true;
  }

  if (!apa_world_ptr_->GetSimuParam().sim_to_target &&
      !apa_world_ptr_->GetSlotManagerPtr()->GetEgoInfoUnderSlot().fix_slot &&
      CheckDynamicUpdate()) {
    ILOG_INFO << "replan by dynamic!";
    frame_.replan_reason = ReplanReason::DYNAMIC;
    return true;
  }

  if (!apa_world_ptr_->GetSimuParam().sim_to_target &&
      !apa_world_ptr_->GetSlotManagerPtr()->GetEgoInfoUnderSlot().fix_slot &&
      CheckPathDangerous()) {
    ILOG_INFO << "replan by path dangerous!";
    frame_.replan_reason = ReplanReason::PATH_DANGEROUS;
    return true;
  }

  if (CheckDynamicGearSwitch()) {
    ILOG_INFO << "replan by dynamic gear switch!";
    frame_.replan_reason = ReplanReason::DYNAMIC_GEAR_SWITCH;
    return true;
  }

  return false;
}

const bool ParkingScenario::CheckSegCompleted(const double replan_dist,
                                              const double wait_time,
                                              const double min_drive_dist) {
  if (frame_.remain_dist_path < replan_dist &&
      apa_world_ptr_->GetMeasureDataManagerPtr()->GetStaticFlag()) {
    ILOG_INFO << "close to target, need wait a certain time!";
    if (frame_.stuck_time > wait_time &&
        frame_.current_path_length - frame_.remain_dist_path > min_drive_dist) {
      ILOG_INFO << "wait a certain time, start plan";
      return true;
    }
  }

  return false;
}

const bool ParkingScenario::CheckObsStucked(const double replan_dist,
                                            const double wait_time,
                                            const double min_drive_dist) {
  if (frame_.stuck_by_dynamic_obs) {
    return false;
  }

  if (frame_.remain_dist_obs < replan_dist &&
      apa_world_ptr_->GetMeasureDataManagerPtr()->GetStaticFlag()) {
    ILOG_INFO << "close to obstacle!, need wait a certain time!";
    if (frame_.stuck_obs_time > wait_time &&
        frame_.current_path_length - frame_.remain_dist_path > min_drive_dist) {
      ILOG_INFO << "wait a certain time, start plan";
      return true;
    }
  }

  return false;
}

const bool ParkingScenario::CheckSlotJumpStucked(const double replan_dist,
                                                 const double wait_time,
                                                 const double min_drive_dist) {
  if (frame_.remain_dist_slot_jump < replan_dist &&
      apa_world_ptr_->GetMeasureDataManagerPtr()->GetStaticFlag()) {
    ILOG_INFO << "close to slot jumped!, need wait a certain time!";
    if (frame_.stuck_time > wait_time &&
        frame_.current_path_length - frame_.remain_dist_path > min_drive_dist) {
      ILOG_INFO << "wait a certain time, start plan";
      return true;
    }
  }

  return false;
}

const bool ParkingScenario::CheckStuckTimeEnough(
    const double stuck_replan_time) {
  if (frame_.stuck_by_dynamic_obs) {
    return false;
  }

  if (frame_.stuck_dynamic_obs_time < 1e-3) {
    return frame_.stuck_path_time > stuck_replan_time;
  } else {
    return frame_.stuck_path_time > stuck_replan_time &&
           (frame_.stuck_time - frame_.stuck_dynamic_obs_time >
            stuck_replan_time);
  }
}

const bool ParkingScenario::CheckDynamicUpdate() { return false; }

void ParkingScenario::RecordDebugObstacle(
    const std::vector<double>& obs_x, const std::vector<double>& obs_y) const {
  auto& debug = DebugInfoManager::GetInstance().GetDebugInfoPb();
  common::ApaPathDebug* path_debug = debug->mutable_apa_path_debug();
  path_debug->clear_obs_list();

  common::ObstacleDebug* obs = path_debug->mutable_obs_list()->add_obs();
  common::Point2d point;

  size_t size = std::min(obs_x.size(), obs_y.size());
  for (size_t i = 0; i < size; i++) {
    point.set_x(obs_x[i]);
    point.set_y(obs_y[i]);

    obs->add_points()->CopyFrom(point);
  }

  return;
}
const bool ParkingScenario::CheckPathDangerous() { return false; }

const ParkingScenario::CarSlotRelationship
ParkingScenario::CalCarSlotRelationship(
    const geometry_lib::PathPoint& cur_pose) {
  return CarSlotRelationship::TOUCHING;
}

const bool ParkingScenario::CheckDynamicGearSwitch() {
  auto& param = apa_param.GetParam().gear_switch_config;
  if (!param.enable_dynamic_gear_switch) {
    return false;
  }

  if (frame_.mirror_command != MirrorCommand::NONE) {
    return false;
  }

  if (frame_.remain_dist_obs < frame_.remain_dist_path) {
    ILOG_INFO << "false";
    return false;
  }

  if (frame_.current_path_length < param.dist_thresh_for_current_path) {
    ILOG_INFO << "false";
    return false;
  }

  double next_gear_path_len =
      GetSecondGearPathLength(complete_path_point_global_vec_);
  if (next_gear_path_len < param.dist_thresh_for_next_path) {
    ILOG_INFO << "false";
    return false;
  }

  if (frame_.remain_dist_path > param.dist_thresh_for_gear_switch_point) {
    ILOG_INFO << "false";
    return false;
  }

  if (std::fabs(apa_world_ptr_->GetMeasureDataManagerPtr()->GetVel()) >
      param.vel_thresh_for_gear_switch_point) {
    ILOG_INFO << "false";
    return false;
  }

  if (apa_world_ptr_->GetSlotManagerPtr()
          ->GetEgoInfoUnderSlot()
          .slot_occupied_ratio >
      param.slot_occupied_ratio_for_gear_switch_point) {
    ILOG_INFO << "false";
    return false;
  }

  if (std::fabs(apa_world_ptr_->GetPredictPathManagerPtr()->GetLatErr()) >
          param.lat_error_for_dynamic_gear_switch ||
      std::fabs(apa_world_ptr_->GetPredictPathManagerPtr()->GetPhiErr()) >
          param.theta_error_for_dynamic_gear_switch) {
    ILOG_INFO << "false";
    return false;
  }

  // check around obs
  double dist =
      CalRemainDistFromObs(param.cur_path_lon_buffer, param.cur_path_lat_buffer,
                           param.cur_path_lat_buffer);
  if (dist < frame_.remain_dist_path) {
    ILOG_INFO << "false";
    return false;
  }

  return true;
}

geometry_lib::PathPoint ParkingScenario::GetCurrentPathTerminal(
    const bool is_slot_coordinate) {
  geometry_lib::PathPoint point;
  if (current_path_point_global_vec_.empty()) {
    return point;
  }

  const pnc::geometry_lib::PathPoint& back =
      current_path_point_global_vec_.back();

  if (is_slot_coordinate) {
    const EgoInfoUnderSlot& ego_info =
        apa_world_ptr_->GetSlotManagerPtr()->GetEgoInfoUnderSlot();
    point.pos = ego_info.g2l_tf.GetPos(back.pos);
    point.heading = ego_info.g2l_tf.GetHeading(back.heading);
  } else {
    point = back;
  }

  return point;
}

void ParkingScenario::RecordDebugPath() {
  auto& debug_info = DebugInfoManager::GetInstance().GetDebugInfoPb();
  debug_info->mutable_cur_path_points()->Clear();
  debug_info->mutable_complete_path_points()->Clear();

  for (const auto& pt : current_path_point_global_vec_) {
    planning::common::ApaPathPoint* point = debug_info->add_cur_path_points();
    point->set_x(pt.GetX());
    point->set_y(pt.GetY());
    point->set_heading(pt.GetTheta());
    point->set_kappa(pt.kappa);
    point->set_lat_buffer(pt.lat_buffer);
    point->set_type(pt.type);
  }

  for (const auto& pt : complete_path_point_global_vec_) {
    planning::common::ApaPathPoint* point =
        debug_info->add_complete_path_points();
    point->set_x(pt.GetX());
    point->set_y(pt.GetY());
    point->set_heading(pt.GetTheta());
    point->set_kappa(pt.kappa);
    point->set_lat_buffer(pt.lat_buffer);
    point->set_type(pt.type);
  }

  return;
}

const bool ParkingScenario::IsStopByDynamicObs() const {
  if (!apa_world_ptr_->GetMeasureDataManagerPtr()->GetStaticFlag()) {
    return false;
  }

  if (frame_.stuck_by_dynamic_obs) {
    return true;
  }

  return false;
}

const bool ParkingScenario::IsStopByStaticMovableObs() const {
  if (!apa_world_ptr_->GetMeasureDataManagerPtr()->GetStaticFlag()) {
    return false;
  }

  if (frame_.stuck_by_dynamic_obs) {
    return false;
  }

  if (frame_.replan_fail_time < 1.0) {
    return false;
  }

  if (!(frame_.remain_dist_obs < frame_.remain_dist_path)) {
    return false;
  }

  // TODO: If the obs is static, check it could be movable, such as
  // traffic_cone, water_safety_barrier, static bicycle, static vehicle.
  ObjectDetectObsConfig od_config;
  od_config.use_dynamic_obs = false;
  od_config.use_specificationer = false;
  od_config.use_movable_static_obs = true;
  auto obs_manager = apa_world_ptr_->GetObstacleManagerPtr();
  obs_manager->GenerateObsByOD(apa_world_ptr_->GetLocalViewPtr(), od_config);

  PathSafeChecker checker(obs_manager);
  if (checker.IsCollisionByStaticMavableOD(
          apa_world_ptr_->GetMeasureDataManagerPtr()->GetPose(), 0.06, 0.3,
          current_path_point_global_vec_)) {
    return true;
  }

  return false;
}

void ParkingScenario::ScenarioSuspend() {
  ClearTimeBySuspendStatus();

  return;
}

void ParkingScenario::ClearTimeBySuspendStatus() {
  frame_.stuck_time = 0.0;
  frame_.stuck_obs_time  = 0.0;
  frame_.dynamic_plan_time = 0.0;
  frame_.replan_fail_time = 0.0;
  return;
}

}  // namespace apa_planner
}  // namespace planning
