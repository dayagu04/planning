#include "parking_scenario.h"
#include <bits/stdint-uintn.h>
#include <sys/types.h>

#include <memory>

#include "apa_param_config.h"
#include "apa_utils.h"
#include "common.pb.h"
#include "common_c.h"
#include "debug_info_log.h"
#include "log_glog.h"
#include "parking_task/deciders/narrow_space_decider.h"
#include "parking_task/parking_task.h"

namespace planning {
namespace apa_planner {

ParkingScenario::ParkingScenario()
    : scenario_status_(ParkingScenarioStatus::STATUS_UNKNOWN), name_("") {}

ParkingScenario::ParkingScenario(const std::shared_ptr<ApaWorld>& apa_world_ptr)
    : scenario_status_(ParkingScenarioStatus::STATUS_UNKNOWN),
      name_(""),
      type_(ParkingScenarioType::SCENARIO_UNKNOWN) {
  SetApaWorldPtr(apa_world_ptr);
}

std::string ParkingScenario::GetName() { return name_; }

void ParkingScenario::Init() {
  return;
}

void ParkingScenario::ScenarioRunning() {
  // run plan core
  PlanCore();

  // generate planning output
  GenPlanningOutput();

  GenPlanningHmiOutput();

  // log json debug
  Log();

  return;
}

void ParkingScenario::Reset() {
  scenario_status_ = ParkingScenarioStatus::STATUS_UNKNOWN;

  return;
}

void ParkingScenario::Process() {
  if (scenario_status_ == ParkingScenarioStatus::STATUS_RUNNING) {
    ScenarioRunning();
  }

  return;
}

void ParkingScenario::InitSimulation() {
  if (apa_world_ptr_->GetApaDataPtr()->simu_param.is_simulation &&
      apa_world_ptr_->GetApaDataPtr()->simu_param.is_reset) {
    Reset();
  }

  return;
}

const bool ParkingScenario::CheckPaused() const {
  if (apa_world_ptr_->GetApaDataPtr()->cur_state == ApaStateMachine::SUSPEND) {
    return true;
  } else {
    return false;
  }
}

const bool ParkingScenario::CheckPlanSkip() const {
  if ((frame_.plan_stm.planning_status == PARKING_FINISHED ||
       frame_.plan_stm.planning_status == PARKING_FAILED) &&
      !apa_world_ptr_->GetApaDataPtr()->simu_param.force_plan) {
    ILOG_INFO << "plan has been finished or failed, need reset";

    apa_world_ptr_->GetSlotManagerPtr()->Reset();

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

void ParkingScenario::GenPlanningOutput() {
  pnc::geometry_lib::PathPoint current_ego_pose(
      apa_world_ptr_->GetApaDataPtr()->measurement_data.pos,
      apa_world_ptr_->GetApaDataPtr()->measurement_data.heading);

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
    GenPlanningPath();
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

void ParkingScenario::GenPlanningHmiOutput() {
  memset(&apa_hmi_, 0, sizeof(apa_hmi_));

  if (frame_.plan_stm.planning_status == PARKING_PLANNING ||
      frame_.plan_stm.planning_status == PARKING_GEARCHANGE ||
      frame_.plan_stm.planning_status == PARKING_RUNNING) {
    apa_hmi_.distance_to_parking_space = frame_.remain_dist;
  }
  return;
}

void ParkingScenario::GenPlanningPath() {
  // planning_output_.Clear();
  memset(&planning_output_, 0, sizeof(planning_output_));
  planning_output_.planning_status.apa_planning_status =
      iflyauto::APA_IN_PROGRESS;

  auto trajectory = &(planning_output_.trajectory);
  trajectory->available = true;

  trajectory->trajectory_type = iflyauto::TRAJECTORY_TYPE_TRAJECTORY_POINTS;

  size_t N = current_path_point_global_vec_.size();
  if (N > PLANNING_TRAJ_POINTS_NUM - 1) {
    ILOG_INFO << "sample ds is possible err";
    N = PLANNING_TRAJ_POINTS_NUM - 1;
  }
  trajectory->trajectory_points_size = N;

  for (size_t i = 0; i < N; ++i) {
    const auto& global_point = current_path_point_global_vec_[i];
    trajectory->trajectory_points[i].x = global_point.pos.x();
    trajectory->trajectory_points[i].y = global_point.pos.y();
    trajectory->trajectory_points[i].heading_yaw = global_point.heading;
    trajectory->trajectory_points[i].v = 0.5;
  }

  // set target velocity to control as a limit
  const std::vector<double> ratio_tab = {0.0, 0.4, 0.8, 1.0};
  const std::vector<double> vel_limit_tab = {apa_param.GetParam().max_velocity,
                                             apa_param.GetParam().max_velocity,
                                             0.45, 0.35};

  const double vel_limit = pnc::mathlib::Interp1(
      ratio_tab, vel_limit_tab, frame_.ego_slot_info.slot_occupied_ratio);

  planning_output_.trajectory.target_reference.target_velocity =
      frame_.vel_target;

  // send uss remain dist to control
  planning_output_.trajectory.trajectory_points[0].distance =
      frame_.remain_dist_uss;

  // send slot occupation ratio to control
  planning_output_.trajectory.trajectory_points[1].distance =
      frame_.ego_slot_info.slot_occupied_ratio;

  // send slot type to control
  planning_output_.trajectory.trajectory_points[2].distance =
      static_cast<double>(apa_world_ptr_->GetApaDataPtr()->slot_type);

  // send remain dist col det to uss
  // only set a flag let control can use it
  const double flag = (apa_world_ptr_->GetApaDataPtr()->slot_type ==
                           iflyauto::PARKING_SLOT_TYPE_VERTICAL ||
                       apa_world_ptr_->GetApaDataPtr()->slot_type ==
                           iflyauto::PARKING_SLOT_TYPE_SLANTING)
                          ? 1.0
                          : 0.0;
  planning_output_.trajectory.trajectory_points[3].distance = 0.0;

  planning_output_.trajectory.trajectory_points[4].distance =
      frame_.remain_dist_col_det;

  // set plan gear cmd
  auto gear_command = &(planning_output_.gear_command);
  gear_command->available = true;

  if (frame_.gear_command == pnc::geometry_lib::SEG_GEAR_DRIVE) {
    gear_command->gear_command_value = iflyauto::GEAR_COMMAND_VALUE_DRIVE;
  } else {
    gear_command->gear_command_value = iflyauto::GEAR_COMMAND_VALUE_REVERSE;
  }

  return;
}

const bool ParkingScenario::CheckStuckFailed() {
  return frame_.stuck_time > apa_param.GetParam().stuck_failed_time;
}

void ParkingScenario::UpdateRemainDist(const double uss_safe_dist) {
  // 1. calculate remain dist according to plan path
  frame_.remain_dist = CalRemainDistFromPath();

  // 2.calculate remain dist uss according to uss
  frame_.remain_dist_uss = CalRemainDistFromUss(uss_safe_dist);

  return;
}

const double ParkingScenario::CalRemainDistFromPath() {
  double remain_dist = 5.01;

  if (frame_.is_replan_first) {
    return remain_dist;
  }

  if (frame_.spline_success) {
    double s_proj = 0.0;
    bool success = pnc::geometry_lib::CalProjFromSplineByBisection(
        0.0, frame_.current_path_length + frame_.path_extended_dist, s_proj,
        apa_world_ptr_->GetApaDataPtr()->measurement_data.pos,
        frame_.x_s_spline, frame_.y_s_spline);

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

const double ParkingScenario::CalRemainDistFromUss(const double safe_dist) {
  double remain_dist = 5.01;

  const auto& uss_obstacle_avoider_ptr =
      apa_world_ptr_->GetUssObstacleAvoidancePtr();

  uss_obstacle_avoider_ptr->Update(&planning_output_,
                                   apa_world_ptr_->GetApaDataPtr());

  remain_dist =
      uss_obstacle_avoider_ptr->GetRemainDistInfo().remain_dist - safe_dist;

  double obs_pt_remain_dist =
      uss_obstacle_avoider_ptr->GetRemainDistInfo().obs_pt_remain_dist -
      safe_dist;

  if (frame_.gear_command == pnc::geometry_lib::SEG_GEAR_REVERSE) {
    remain_dist -= 0.068;
    obs_pt_remain_dist -= 0.068;
  }

  ILOG_INFO << "origin_uss remain dist = "
            << uss_obstacle_avoider_ptr->GetRemainDistInfo().remain_dist
            << "  uss remain dist = " << remain_dist;

  ILOG_INFO << "origin_obs_pt remain dist = "
            << uss_obstacle_avoider_ptr->GetRemainDistInfo().obs_pt_remain_dist
            << "  obs_pt remain dist = " << obs_pt_remain_dist;

  frame_.vel_target = uss_obstacle_avoider_ptr->GetRemainDistInfo().vel_target;

  if (apa_param.GetParam().enable_corner_uss_process) {
    return remain_dist;
  } else {
    return obs_pt_remain_dist;
  }
}

const bool ParkingScenario::PostProcessPath() {
  size_t origin_trajectory_size = current_path_point_global_vec_.size();
  if (origin_trajectory_size < 2) {
    frame_.spline_success = false;
    ILOG_INFO << "error: origin_trajectory_size = " << origin_trajectory_size;
    frame_.plan_fail_reason = POST_PROCESS_PATH_POINT_SIZE;
    return false;
  }

  const size_t trajectory_size = origin_trajectory_size + 1;

  std::vector<double> x_vec;
  std::vector<double> y_vec;
  std::vector<double> s_vec;
  x_vec.clear();
  y_vec.clear();
  s_vec.clear();

  x_vec.resize(trajectory_size);
  y_vec.resize(trajectory_size);
  s_vec.resize(trajectory_size);

  double s = 0.0;
  double ds = 0.0;
  for (size_t i = 0; i < origin_trajectory_size; ++i) {
    x_vec[i] = current_path_point_global_vec_[i].pos.x();
    y_vec[i] = current_path_point_global_vec_[i].pos.y();
    if (i == 0) {
      s_vec[i] = s;
    } else {
      ds = std::hypot(x_vec[i] - x_vec[i - 1], y_vec[i] - y_vec[i - 1]);
      s += std::max(ds, 1e-3);
      s_vec[i] = s;
    }
  }

  frame_.current_path_length = s;

  // calculate the extended point and insert
  Eigen::Vector2d start_point(x_vec[origin_trajectory_size - 2],
                              y_vec[origin_trajectory_size - 2]);

  Eigen::Vector2d end_point(x_vec[origin_trajectory_size - 1],
                            y_vec[origin_trajectory_size - 1]);

  Eigen::Vector2d extended_point;

  bool success = pnc::geometry_lib::CalExtendedPointByTwoPoints(
      start_point, end_point, extended_point, frame_.path_extended_dist);

  if (success == false) {
    frame_.spline_success = false;
    ILOG_INFO << "fit line by spline error!";
    frame_.plan_fail_reason = POST_PROCESS_PATH_POINT_SAME;
    return false;
  }

  x_vec[origin_trajectory_size] = extended_point.x();
  y_vec[origin_trajectory_size] = extended_point.y();

  s_vec[origin_trajectory_size] =
      s_vec[origin_trajectory_size - 1] + frame_.path_extended_dist;

  frame_.x_s_spline.set_points(s_vec, x_vec);
  frame_.y_s_spline.set_points(s_vec, y_vec);

  frame_.spline_success = true;

  return true;
}

void ParkingScenario::CreateTasks() {

  return;
}

void ParkingScenario::Enter(const ParkingScenarioStatus status) {
  scenario_status_ = status;

  return;
}

void ParkingScenario::ThreadClear() { return; }

const ParkingScenarioStatus ParkingScenario::ScenarioTry() {

  // todo: use geometry method first, if no result, use hybrid astar.
  std::shared_ptr<SlotManager> slot_manager =
      apa_world_ptr_->GetSlotManagerPtr();
  slot_manager->SlotReleaseByScenarioTry(
      true, SlotReleaseMethod::GEOMETRY_PLANNING_RELEASE);

  return ParkingScenarioStatus::STATUS_RUNNING;
}

void ParkingScenario::Exit() {
  scenario_status_ = ParkingScenarioStatus::STATUS_DONE;

  return;
}

}  // namespace apa_planner
}  // namespace planning
