#include "apa_plan_base.h"

#include <memory>

#include "apa_utils.h"
#include "common_c.h"
#include "debug_info_log.h"

namespace planning {
namespace apa_planner {

void ApaPlannerBase::Init() {}

void ApaPlannerBase::Update() {
  // run plan core
  PlanCore();

  // generate planning output
  GenPlanningOutput();

  GenPlanningHmiOutput();

  // log json debug
  Log();
}

void ApaPlannerBase::InitSimulation() {
  if (apa_world_ptr_->GetApaDataPtr()->simu_param.is_simulation &&
      apa_world_ptr_->GetApaDataPtr()->simu_param.is_reset) {
    Reset();
  }
}

const bool ApaPlannerBase::CheckPaused() const {
  if (apa_world_ptr_->GetApaDataPtr()->cur_state == ApaStateMachine::SUSPEND) {
    return true;
  } else {
    return false;
  }
}

const bool ApaPlannerBase::CheckPlanSkip() const {
  if (frame_.plan_stm.planning_status == PARKING_FINISHED ||
      frame_.plan_stm.planning_status == PARKING_FAILED) {
    DEBUG_PRINT("plan has been finished or failed, need reset");

    if (!apa_world_ptr_->GetApaDataPtr()->simu_param.is_simulation) {
      apa_world_ptr_->GetSlotManagerPtr()->Reset();
    }

    return true;
  } else {
    return false;
  }
}

void ApaPlannerBase::SetParkingStatus(uint8_t status) {
  if (status == PARKING_IDLE || status == PARKING_FAILED) {
    frame_.plan_stm.path_plan_success = false;
  } else if (status == PARKING_RUNNING || status == PARKING_GEARCHANGE ||
             status == PARKING_PLANNING || status == PARKING_FINISHED) {
    frame_.plan_stm.path_plan_success = true;
  }

  frame_.plan_stm.planning_status = status;
}

void ApaPlannerBase::GenPlanningOutput() {
  pnc::geometry_lib::PathPoint current_ego_pose(
      apa_world_ptr_->GetApaDataPtr()->measurement_data.pos,
      apa_world_ptr_->GetApaDataPtr()->measurement_data.heading);

  DEBUG_PRINT("frame_.plan_stm.planning_status = "
              << static_cast<int>(frame_.plan_stm.planning_status)
              << "  plan path pt size = "
              << current_path_point_global_vec_.size());

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

  DEBUG_PRINT("gen plan output success.");
}

void ApaPlannerBase::GenPlanningHmiOutput() {
  memset(&apa_hmi_, 0, sizeof(apa_hmi_));

  if (frame_.plan_stm.planning_status == PARKING_PLANNING ||
      frame_.plan_stm.planning_status == PARKING_GEARCHANGE ||
      frame_.plan_stm.planning_status == PARKING_RUNNING) {
    apa_hmi_.distance_to_parking_space = frame_.remain_dist;
  }
}

void ApaPlannerBase::GenPlanningPath() {
  // planning_output_.Clear();
  memset(&planning_output_, 0, sizeof(planning_output_));
  planning_output_.planning_status.apa_planning_status =
      iflyauto::APA_IN_PROGRESS;

  auto trajectory = &(planning_output_.trajectory);
  trajectory->available = true;

  trajectory->trajectory_type = iflyauto::TRAJECTORY_TYPE_TRAJECTORY_POINTS;

  size_t N = current_path_point_global_vec_.size();
  if (N > PLANNING_TRAJ_POINTS_NUM - 1) {
    std::cout << "sample ds is possible err\n";
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

  planning_output_.trajectory.target_reference.target_velocity = vel_limit;

  // send uss remain dist to control
  planning_output_.trajectory.trajectory_points[0].distance =
      frame_.remain_dist_uss;

  if (current_path_point_global_vec_.size() <= 1) {
    return;
  }

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
  planning_output_.trajectory.trajectory_points[3].distance = flag;

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
}

const bool ApaPlannerBase::CheckStuckFailed() {
  return frame_.stuck_time > apa_param.GetParam().stuck_failed_time;
}

void ApaPlannerBase::UpdateRemainDist() {
  // 1. calculate remain dist according to plan path
  frame_.remain_dist = CalRemainDistFromPath();

  // 2.calculate remain dist uss according to uss
  frame_.remain_dist_uss = CalRemainDistFromUss();

  return;
}

const double ApaPlannerBase::CalRemainDistFromPath() {
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

      DEBUG_PRINT("remain_dist = " << remain_dist << "  s_proj = " << s_proj
                                   << "  current_path_length = "
                                   << frame_.current_path_length);
    } else {
      DEBUG_PRINT("remain_dist calculation error:input is error");
    }
  } else {
    DEBUG_PRINT("remain_dist calculation error: path spline failed!");
  }

  return remain_dist;
}

const double ApaPlannerBase::CalRemainDistFromUss() {
  double remain_dist = 5.01;

  // if (frame_.is_replan_first) {
  //   return remain_dist;
  // }
  const auto& uss_obstacle_avoider_ptr =
      apa_world_ptr_->GetUssObstacleAvoidancePtr();

  uss_obstacle_avoider_ptr->Update(&planning_output_,
                                   apa_world_ptr_->GetApaDataPtr());

  const double safe_uss_remain_dist =
      (frame_.ego_slot_info.slot_occupied_ratio < 0.05)
          ? apa_param.GetParam().safe_uss_remain_dist_out_slot
          : apa_param.GetParam().safe_uss_remain_dist_in_slot;

  remain_dist = uss_obstacle_avoider_ptr->GetRemainDistInfo().remain_dist -
                safe_uss_remain_dist;

  DEBUG_PRINT("origin_uss remain dist = "
              << uss_obstacle_avoider_ptr->GetRemainDistInfo().remain_dist
              << "  uss remain dist = " << remain_dist);

  // remain_dist = 5.01;

  return remain_dist;
}

const bool ApaPlannerBase::PostProcessPath() {
  size_t origin_trajectory_size = current_path_point_global_vec_.size();
  if (origin_trajectory_size < 2) {
    frame_.spline_success = false;
    DEBUG_PRINT("error: origin_trajectory_size = " << origin_trajectory_size);
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
    DEBUG_PRINT("fit line by spline error!");
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

}  // namespace apa_planner
}  // namespace planning
