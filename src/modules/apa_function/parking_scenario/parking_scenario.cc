#include "parking_scenario.h"

#include <bits/stdint-uintn.h>
#include <sys/types.h>

#include <algorithm>
#include <memory>
#include <string>

#include "apa_param_config.h"
#include "apa_slot.h"
#include "apa_utils.h"
#include "common.pb.h"
#include "common_c.h"
#include "debug_info_log.h"
#include "geometry_math.h"
#include "log_glog.h"
#include "narrow_space_decider.h"
#include "parking_task/parking_task.h"
#include "parking_stop_decider.h"
#include "park_speed_limit_decider.h"
#include "pose2d.h"

namespace planning {
namespace apa_planner {

void PrintApaScenarioStatus(const ParkingScenarioStatus scenario_status) {
  ILOG_INFO << "scenario_status = "
            << GetApaScenarioStatusString(scenario_status);
}

const std::string GetApaScenarioStatusString(
    const ParkingScenarioStatus scenario_status) {
  std::string status;
  switch (scenario_status) {
    case ParkingScenarioStatus::STATUS_TRY:
      status = "STATUS_TRY";
      break;
    case ParkingScenarioStatus::STATUS_RUNNING:
      status = "STATUS_RUNNING";
      break;
    case ParkingScenarioStatus::STATUS_DONE:
      status = "STATUS_DONE";
      break;
    case ParkingScenarioStatus::STATUS_FAIL:
      status = "STATUS_FAIL";
      break;
    default:
      status = "STATUS_UNKNOWN";
      break;
  }
  return status;
}

ParkingScenario::ParkingScenario() {}

ParkingScenario::ParkingScenario(
    const std::shared_ptr<ApaWorld>& apa_world_ptr) {
  SetApaWorldPtr(apa_world_ptr);
}

std::string ParkingScenario::GetName() { return ""; }

void ParkingScenario::Init() { return; }

void ParkingScenario::Reset() { return; }

void ParkingScenario::ScenarioRunning() {
  // run plan core
  ExcutePathPlanningTask();

  ExcuteSpeedPlanningTask();

  // generate planning output
  GenPlanningOutput();

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
  // update stuck by uss time  重规划清空
  if (frame_.plan_stm.planning_status == PARKING_RUNNING &&
      apa_world_ptr_->GetMeasureDataManagerPtr()->GetStaticFlag() &&
      !apa_world_ptr_->GetMeasureDataManagerPtr()->GetBrakeFlag()) {
    frame_.stuck_uss_time += apa_param.GetParam().plan_time;
  } else {
    frame_.stuck_uss_time = 0.0;
  }

  // 重规划不清空
  if ((frame_.plan_stm.planning_status == PARKING_RUNNING ||
       frame_.plan_stm.planning_status == PARKING_PLANNING) &&
      apa_world_ptr_->GetMeasureDataManagerPtr()->GetStaticFlag() &&
      !apa_world_ptr_->GetMeasureDataManagerPtr()->GetBrakeFlag()) {
    frame_.stuck_time += apa_param.GetParam().plan_time;
  } else {
    frame_.stuck_time = 0.0;
  }

  // update pause time  这个时间其实没用  因为踩刹车导致的暂停根本进不来规划器
  // 后面删除
  if (frame_.plan_stm.planning_status == PARKING_PAUSED) {
    frame_.pause_time += apa_param.GetParam().plan_time;
  } else {
    frame_.pause_time = 0.0;
  }
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

    // apa_world_ptr_->GetSlotManagerPtr()->Reset();

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

  planning_output_.trajectory.target_reference.target_velocity =
      frame_.vel_target;

  // reset obs remain dist when gear shift
  if ((frame_.gear_command == pnc::geometry_lib::SEG_GEAR_DRIVE &&
       planning_output_.gear_command.gear_command_value ==
           iflyauto::GearCommandValue::GEAR_COMMAND_VALUE_REVERSE) ||
      (frame_.gear_command == pnc::geometry_lib::SEG_GEAR_REVERSE &&
       planning_output_.gear_command.gear_command_value ==
           iflyauto::GearCommandValue::GEAR_COMMAND_VALUE_DRIVE)) {
    frame_.remain_dist_uss = 2.68;
    frame_.remain_dist_col_det = 2.68;
  }

  // send uss remain dist to control
  planning_output_.trajectory.trajectory_points[0].distance =
      frame_.remain_dist_uss;

  // send slot occupation ratio to control
  planning_output_.trajectory.trajectory_points[1].distance =
      frame_.ego_slot_info.slot_occupied_ratio;

  // send slot type to control
  planning_output_.trajectory.trajectory_points[2].distance =
      static_cast<double>(apa_world_ptr_->slot_type_);

  // send remain dist col det to uss
  // only set a flag let control can use it
  const double flag =
      (apa_world_ptr_->GetNewSlotManagerPtr()
               ->ego_info_under_slot_.slot.slot_type_ ==
           SlotType::PERPENDICULAR ||
       apa_world_ptr_->GetNewSlotManagerPtr()
               ->ego_info_under_slot_.slot.slot_type_ == SlotType::SLANT)
          ? 1.0
          : 0.0;
  planning_output_.trajectory.trajectory_points[3].distance = flag;

  planning_output_.trajectory.trajectory_points[4].distance =
      frame_.remain_dist_col_det;

  // set plan gear cmd
  auto gear_command = &(planning_output_.gear_command);
  gear_command->available = true;

  // reset obs remain dist when gear shift
  if ((frame_.gear_command == pnc::geometry_lib::SEG_GEAR_DRIVE &&
       gear_command->gear_command_value ==
           iflyauto::GearCommandValue::GEAR_COMMAND_VALUE_REVERSE) ||
      (frame_.gear_command == pnc::geometry_lib::SEG_GEAR_REVERSE &&
       gear_command->gear_command_value ==
           iflyauto::GearCommandValue::GEAR_COMMAND_VALUE_DRIVE)) {
    frame_.remain_dist_uss = 2.68;
    frame_.remain_dist_col_det = 2.68;
  }

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

const bool ParkingScenario::UpdateObstacleLocal() {
  auto& ego_slot_info = frame_.ego_slot_info;
  apa_world_ptr_->GetObstacleManagerPtr()->TransformCoordFromGlobalToLocal(
      ego_slot_info.g2l_tf);
  const auto obstacle_manager_ptr = apa_world_ptr_->GetObstacleManagerPtr();

  // 只是临时使用而已 其实不应该把障碍物转换到ego_slot_info
  // 这个函数待整体适配完需要删除 应该直接从障碍物管理器中获取障碍物

  std::vector<ApaObstacle*> obs_vec;
  std::vector<Eigen::Vector2d> obs_pt_vec;
  ego_slot_info.obs_pt_vec_slot.clear();

  if (apa_param.GetParam().use_uss_pt_clound) {
    // 获取超声波点云
    if (obstacle_manager_ptr->GetObstacle(ApaObsAttributeType::USS_POINT_CLOUD,
                                          obs_vec)) {
      for (const auto& obs : obs_vec) {
        ego_slot_info.obs_pt_vec_slot.insert(
            ego_slot_info.obs_pt_vec_slot.end(),
            obs->GetPtClout2dLocal().begin(), obs->GetPtClout2dLocal().end());
      }
    }
  }

  // 获取OCC点云
  if (obstacle_manager_ptr->GetObstacle(ApaObsAttributeType::FUSION_POINT_CLOUD,
                                        obs_vec)) {
    for (const auto& obs : obs_vec) {
      ego_slot_info.obs_pt_vec_slot.insert(ego_slot_info.obs_pt_vec_slot.end(),
                                           obs->GetPtClout2dLocal().begin(),
                                           obs->GetPtClout2dLocal().end());
    }
  }

  // 获取接地线点云
  if (obstacle_manager_ptr->GetObstacle(
          ApaObsAttributeType::GROUND_LINE_POINT_CLOUD, obs_vec)) {
    for (const auto& obs : obs_vec) {
      ego_slot_info.obs_pt_vec_slot.insert(ego_slot_info.obs_pt_vec_slot.end(),
                                           obs->GetPtClout2dLocal().begin(),
                                           obs->GetPtClout2dLocal().end());
    }
  }

  return true;
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

const double ParkingScenario::CalRemainDistFromUss(const double safe_dist) {
  double remain_dist = 5.01;

  const auto& uss_obstacle_avoider_ptr =
      apa_world_ptr_->GetUssObstacleAvoidancePtr();

  uss_obstacle_avoider_ptr->Update(apa_world_ptr_->GetMeasureDataManagerPtr(),
                                   apa_world_ptr_->GetPredictPathManagerPtr(),
                                   apa_world_ptr_->GetObstacleManagerPtr());

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
            << "  uss remain dist = " << remain_dist
            << "  enable_corner_uss_process = "
            << apa_param.GetParam().enable_corner_uss_process;

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
  x_vec.reserve(origin_trajectory_size + 1);
  y_vec.reserve(origin_trajectory_size + 1);
  heading_vec.reserve(origin_trajectory_size + 1);
  s_vec.reserve(origin_trajectory_size + 1);
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
  }

  const size_t x_vec_size = x_vec.size();
  if (x_vec_size < 2) {
    frame_.spline_success = false;
    ILOG_INFO << "error: x_vec_size = " << x_vec.size();
    frame_.plan_fail_reason = POST_PROCESS_PATH_POINT_SIZE;
    return false;
  }

  current_path_point_global_vec_.clear();
  current_path_point_global_vec_.resize(x_vec_size);
  pnc::geometry_lib::PathPoint point;
  for (size_t i = 0; i < x_vec_size; ++i) {
    point.pos << x_vec[i], y_vec[i];
    point.heading = heading_vec[i];
    point.s = s_vec[i];
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

void ParkingScenario::ThreadClear() { return; }

void ParkingScenario::ScenarioTry() {
  // todo: use geometry method first, if no result, use hybrid astar.
  std::shared_ptr<ApaSlotManager> sslot_manager =
      apa_world_ptr_->GetNewSlotManagerPtr();
  sslot_manager->ego_info_under_slot_.slot.release_info_
      .release_state[SlotReleaseMethod::GEOMETRY_PLANNING_RELEASE] =
      SlotReleaseState::RELEASE;
  sslot_manager->ego_info_under_slot_.slot.release_info_
      .release_state[SlotReleaseMethod::ASTAR_PLANNING_RELEASE] =
      SlotReleaseState::RELEASE;

  return;
}

void ParkingScenario::ExcuteSpeedPlanningTask() {
  if (!apa_param.GetParam().speed_config.enable_apa_speed_plan) {
    return;
  }

  SpeedDecisions speed_decisions;

  // todo: use apa obstacle
  ParkObstacleList obstacles;
  const std::shared_ptr<ApaObstacleManager> obs_manager =
      apa_world_ptr_->GetObstacleManagerPtr();
  if (obs_manager == nullptr) {
    return;
  }

  for (const auto& pair : obs_manager->GetObstacles()) {
    if (!apa_param.GetParam().use_uss_pt_clound &&
        pair.second.GetObsAttributeType() ==
            apa_planner::ApaObsAttributeType::USS_POINT_CLOUD) {
      continue;
    }

    planning::PointCloudObstacle obs;
    obs.points.reserve(pair.second.GetPtClout2dGlobal().size());
    for (const auto& pt : pair.second.GetPtClout2dGlobal()) {
      obs.points.emplace_back(Position2D(pt.x(), pt.y()));
    }
    obs.box = pair.second.GetBoxGlobal();
    obs.envelop_polygon = pair.second.GetPolygon2DGlobal();
    obs.obs_type = pair.second.GetObsAttributeType();
    obstacles.point_cloud_list.emplace_back(obs);
  }

  double tracking_path_collision_dist = frame_.remain_dist_uss;
  tracking_path_collision_dist =
      std::min(tracking_path_collision_dist, frame_.remain_dist_col_det);

  ILOG_INFO << "remain_dist_uss" << frame_.remain_dist_uss
            << ", frame_.remain_dist_col_det" << frame_.remain_dist_col_det;

  // update stop decision
  ParkingStopDecider stop_decider = ParkingStopDecider();
  stop_decider.Process(obstacles, apa_world_ptr_, tracking_path_collision_dist,
                       current_path_point_global_vec_, &speed_decisions);

  // update speed limit decision
  ParkSpeedLimitDecider speed_limit_decider = ParkSpeedLimitDecider();
  speed_limit_decider.Process(obstacles, current_path_point_global_vec_,
                              &speed_decisions);

  // get ego around speed limit decision
  const SpeedLimitDecision* min_speed_decision =
      speed_limit_decider.GetSpeedLimitDecisionBySRange(
          &speed_decisions, stop_decider.GetEgoPathProjectS(),
          stop_decider.GetEgoPathProjectS() + 0.6);

  // todo: add uss obs dist
  const auto& uss_obstacle_avoider_ptr =
      apa_world_ptr_->GetUssObstacleAvoidancePtr();
  if (min_speed_decision != nullptr) {
    frame_.vel_target =
        std::min(min_speed_decision->advised_speed,
                 uss_obstacle_avoider_ptr->GetRemainDistInfo().vel_target);
  } else {
    frame_.vel_target =
        std::min(apa_param.GetParam().speed_config.default_cruise_speed,
                 uss_obstacle_avoider_ptr->GetRemainDistInfo().vel_target);
  }

  return;
}

}  // namespace apa_planner
}  // namespace planning
