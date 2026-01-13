#include "parallel_park_in_scenario.h"
#include <Eigen/src/Core/Matrix.h>

#include <algorithm>
#include <cmath>
#include <complex>
#include <cstddef>
#include <cstdint>
#include <iostream>
#include <limits>
#include <queue>
#include <utility>
#include <vector>
#include <random>
#include <numeric>
#include <cmath>
#include <Eigen/Dense>

#include "apa_param_config.h"
#include "apa_slot.h"
#include "apa_slot_manager.h"
#include "apa_utils.h"
#include "apa_world.h"
#include "common_c.h"
#include "config/basic_type.h"
#include "debug_info_log.h"
#include "dubins_lib.h"
#include "func_state_machine_c.h"
#include "geometry_math.h"
#include "geometry_path_generator.h"
#include "gjk_collision_detector.h"
#include "ifly_time.h"
#include "lateral_path_optimizer.h"
#include "local_view.h"
#include "log_glog.h"
#include "math_lib.h"
#include "obstacle.h"
#include "parallel_path_generator.h"
#include "src/modules/apa_function/parking_scenario/parking_scenario.h"
#include "park_hmi_state.h"

namespace planning {
namespace apa_planner {

static double kInsertLineLonBuffer = 0.4;
static double kFrontDetaXMagWhenFrontVacant = 3.0;
static double kFrontMaxDetaXMagWhenFrontOccupied = 0.8;
static double kRearDetaXMagWhenBothSidesVacant = 0.5;
static double kRearDetaXMagWhenFrontOccupiedRearVacant = 2;
static double kRearDetaXMagWhenFrontVacantRearOccupied = 0.2;
static double kRearMaxDetaXMagWhenRearOccupied = 0.8;
static double kFrontObsLineYMagIdentification = 1.2;
static double kRearObsLineYMagIdentification = 0.6;
static double kTBoundaryXdiff = 0.5;
static double kCurbInitialOffset = 0.46;
static double kCurbYMagIdentification = 0.0;
static double kMaxDistDeleteObsToEgoInSlot = 0.3;
static double kMaxDistDeleteObsToEgoOutSlot = 0.35;
static double kMinChannelYMagIdentification = 3.3;
static double kExtendLengthOutsideSlot = 0.5;
static double kDeletedObsDistOutSlot = 0.3;
static double kDeletedObsDistInSlot = 0.10;
static double kTBoundarySampleDist = 0.38;
static double kChannelSampleDist = 0.46;
static double kEnterMultiPlanSlotRatio = 0.1;
static double kEps = 1e-5;
static double kFrontShortChannelMin = 0.5;
static double kFrontShortChannelMax = 7.5;
static double kWidthCurbOffset = 0.4;
static double kWidthSlot = 2.3;
static double kAdjustLonErr = 0.2;
static const size_t kMaxReplanTimes = 15;
static int kWallPtNumThred = 10;
static double kBigWallHeightRatioThred = 0.5;
static double kSmallWallHeightRatioThred = 0.25;

using Point = Eigen::Vector2d;

void ParallelParkInScenario::Reset() {
  frame_.Reset();
  t_lane_.Reset();
  obs_pt_local_vec_.clear();
  parallel_path_planner_.Reset();
  previous_parallel_path_planner_.Reset();
  parallel_replan_again_ = 0;
  previous_remain_dist_obs.clear();
  enable_pa_park_ = false;
  first_plan_slot.Reset();
  first_line_coeffs_ << 0.0, 0.0;
  first_plan_cur_pos.Reset();
  multi_parkin_path_vec_.clear();
  delay_check_finish_ = false;
  relative_loc_observer_manager_.Reset();
  try_bound_map_.clear();
  parent_total_count.clear();
  parent_height_count.clear();
  multi_frame_height_obs_map_.clear();

  ParkingScenario::Reset();
}

bool ParallelParkInScenario::CheckReplanParallel() {
  ILOG_INFO << "Enter CheckReplanParallel";
  if (parallel_replan_again_ != 0) {
    return false;
  }
  EgoInfoUnderSlot& ego_info_under_slot =
      apa_world_ptr_->GetSlotManagerPtr()->GetMutableEgoInfoUnderSlot();

  pnc::geometry_lib::PathSegGear cur_gear =
      apa_world_ptr_->GetMeasureDataManagerPtr()->GetGear();
  pnc::geometry_lib::PathPoint& cur_pose = ego_info_under_slot.cur_pose;

  ILOG_INFO << "cur_pose pos: " << cur_pose.GetPos().x() << " "
            << cur_pose.GetPos().y()
            << " slot length: " << ego_info_under_slot.slot.GetLength()
            << " cur_pose.gear: " << static_cast<int>(cur_gear);
  const double step_dist = 0.2;
  if (cur_pose.GetX() < ego_info_under_slot.slot.GetLength() +
                            apa_param.GetParam().parallel_replan_dist +
                            step_dist &&
      cur_pose.GetX() > ego_info_under_slot.slot.GetLength() +
                            apa_param.GetParam().parallel_replan_dist &&
      cur_gear == pnc::geometry_lib::SEG_GEAR_REVERSE) {
    ILOG_INFO << "replan once when car is coming slot";
    frame_.replan_reason = ReplanReason::DYNAMIC;
    parallel_replan_again_ = 1;
    return true;
  }
  return false;
}

void ParallelParkInScenario::CheckEgoPoseWhenPlanFaild(ParkingFailReason reason) {
  ILOG_INFO << "Enter CheckEgoPoseWhenPlanFaild!";
  const EgoInfoUnderSlot& ego_slot_info =
      apa_world_ptr_->GetSlotManagerPtr()->GetEgoInfoUnderSlot();

  if (enable_pa_park_) {
    const double finish_pa_heading = 3.0;
    const bool pa_heading_condition =
        std::fabs(ego_slot_info.terminal_err.heading) <=
        finish_pa_heading * kDeg2Rad;
    if ((reason == ParkingFailReason::PATH_PLAN_FAILED) &&
        pa_heading_condition) {
      ILOG_INFO << "parallel parking finish!";
      SetParkingStatus(PARKING_FINISHED);
    } else {
      ILOG_INFO << "parallel parking failed!";
      SetParkingStatus(PARKING_FAILED);
      frame_.plan_fail_reason = reason;
    }
    return;
  }

  const double finish_parallel_lat_err = 0.5;
  const double finish_parallel_lon_err = 0.5;

  const double finish_parallel_heading = 10.0;
  const bool heading_condition =
      std::fabs(ego_slot_info.terminal_err.heading) <=
      finish_parallel_heading * kDeg2Rad;

  ILOG_INFO << "terminal heading error = "
            << ego_slot_info.terminal_err.heading * kRad2Deg;
  ILOG_INFO << "heading_condition = " << heading_condition;

  ILOG_INFO << "lat error = " << ego_slot_info.terminal_err.pos.y();
  const bool lat_condition_1 =
      std::fabs(ego_slot_info.terminal_err.pos.y()) <= finish_parallel_lat_err;

  // lat condition 2, keep both outer wheels in slot
  const double side_sgn =
      t_lane_.slot_side == pnc::geometry_lib::SLOT_SIDE_RIGHT ? 1.0 : -1.0;

  pnc::geometry_lib::LocalToGlobalTf ego2slot;
  ego2slot.Init(ego_slot_info.cur_pose.pos, ego_slot_info.cur_pose.heading);

  const auto& front_out_wheel = ego2slot.GetPos(
      Eigen::Vector2d(apa_param.GetParam().wheel_base,
                      0.5 * apa_param.GetParam().car_width * side_sgn));

  const auto& rear_out_wheel = ego2slot.GetPos(
      Eigen::Vector2d(0.0, 0.5 * apa_param.GetParam().car_width * side_sgn));

  const double slot_outer_pt_y =
      0.5 * ego_slot_info.slot.slot_width_ * side_sgn;
  bool lat_condition_2 = false;
  if (side_sgn > 0.0) {
    const double wheel_limit_y =
        slot_outer_pt_y - apa_param.GetParam().finish_parallel_lat_err;
    lat_condition_2 = (rear_out_wheel.y() <= wheel_limit_y) &&
                      (front_out_wheel.y() <= wheel_limit_y);
  } else {
    const double wheel_limit_y =
        slot_outer_pt_y + apa_param.GetParam().finish_parallel_lat_err;
    lat_condition_2 = (rear_out_wheel.y() >= wheel_limit_y) &&
                      (front_out_wheel.y() >= wheel_limit_y);
  }
  const bool lat_condition = lat_condition_1 || lat_condition_2;

  ILOG_INFO << "terminal y error = " << ego_slot_info.terminal_err.pos.y();
  ILOG_INFO << "lat condition  = " << lat_condition;
  if (lat_condition) {
    if (lat_condition_1) {
      ILOG_INFO << "lat y err = " << ego_slot_info.terminal_err.pos.y() << " < "
                << finish_parallel_lat_err;
    } else {
      ILOG_INFO << "ego outer wheel are both in slot!";
    }
  }

  const bool lon_condition =
      std::fabs(ego_slot_info.terminal_err.pos.x()) < finish_parallel_lon_err;
  ILOG_INFO << "terminal y error = " << ego_slot_info.terminal_err.pos.x();
  ILOG_INFO << "lon_condition = " << lon_condition;

  if (lat_condition && heading_condition && lon_condition) {
    ILOG_INFO << "parallel parking finish!";
    SetParkingStatus(PARKING_FINISHED);
  } else {
    ILOG_INFO << "parallel parking failed!";
    SetParkingStatus(PARKING_FAILED);
    frame_.plan_fail_reason = reason;
  }
  return;
}

const double ParallelParkInScenario::UpdateRemainDistObs(
    const double remain_dist_path, const double remain_dist_obs) {
  ILOG_INFO << "Enter UpdateRemainDistObs!";
  const double dist_path_threshold = 0.3;
  const double dist_obs_threshold = 1.0;
  previous_remain_dist_obs.emplace_back(remain_dist_obs);
  if (previous_remain_dist_obs.size() > 12) {
    previous_remain_dist_obs.pop_front();
  }
  if (previous_remain_dist_obs.size() < 3) {
    ILOG_INFO << "size: use real calc remain_dist_obs";
    return remain_dist_obs;
  }
  if (remain_dist_path > dist_path_threshold) {
    ILOG_INFO << "path: use real calc remain_dist_obs";
    return remain_dist_obs;
  }
  int ascending_count = 0;
  double min_dist_obs = previous_remain_dist_obs.front();
  double max_dist_obs = previous_remain_dist_obs.front();
  for (size_t i = 1; i < previous_remain_dist_obs.size(); ++i) {
    min_dist_obs = std::min(min_dist_obs, previous_remain_dist_obs[i]);
    max_dist_obs = std::max(max_dist_obs, previous_remain_dist_obs[i]);
    if (previous_remain_dist_obs[i] >
        (previous_remain_dist_obs[i - 1] + 1e-3)) {
      ascending_count++;
    }
  }
  const bool is_ascending =
      ascending_count >
      static_cast<int>(previous_remain_dist_obs.size() - 1) / 2;

  const double previous_obs =
      previous_remain_dist_obs[previous_remain_dist_obs.size() - 2];
  const double cur_remain_dist_obs =
      is_ascending ? std::max(max_dist_obs, remain_dist_obs)
                   : std::min(min_dist_obs, remain_dist_obs);
  ILOG_INFO << "cur_remain_dist_obs = " << cur_remain_dist_obs
            << " previous_remain_dist_obs = " << previous_obs
            << " is_ascending = " << static_cast<int>(is_ascending);
  const double dist_diff = std::fabs(previous_obs - remain_dist_obs);
  if (dist_diff > dist_obs_threshold && previous_obs < remain_dist_obs) {
    ILOG_INFO << "diff: use real calc remain_dist_obs";
    previous_remain_dist_obs.clear();
    return remain_dist_obs;
  }
  return cur_remain_dist_obs;
}

void ParallelParkInScenario::ExcutePathPlanningTask() {
  ILOG_INFO << "Enter parallel parking planner!-----------------------";
  // init simulation
  InitSimulation();

  // check planning status
  if (CheckPlanSkip()) {
    return;
  }

  UpdateStuckTime();

  enable_pa_park_ = false;
  if (apa_world_ptr_->GetStateMachineManagerPtr()->GetParkRunningMode() ==
      ApaRunningMode::RUNNING_PA) {
    enable_pa_park_ = true;
  }

  // calculate remain dist according to plan path
  frame_.remain_dist_path = CalRemainDistFromPath();

  // calculate remain dist uss according to uss
  frame_.remain_dist_obs = CalRealTimeBrakeDist();

  // update remain dist obs by previous state
  frame_.remain_dist_obs =
      UpdateRemainDistObs(frame_.remain_dist_path, frame_.remain_dist_obs);

  if (enable_pa_park_) {
    if (!UpdatePASlotInfo()) {
      SetParkingStatus(PARKING_FAILED);
      frame_.plan_fail_reason = UPDATE_EGO_SLOT_INFO;
      return;
    }
  } else {
    // update ego slot info
    if (!UpdateEgoSlotInfo()) {
      ILOG_INFO << "update ego slot info failed";
      SetParkingStatus(PARKING_FAILED);
      frame_.plan_fail_reason = UPDATE_EGO_SLOT_INFO;
      return;
    }
  }

  // generate t-lane
  if (!GenTlane()) {
    ILOG_INFO << "GenTlane failed!";
    CheckEgoPoseWhenPlanFaild(ParkingFailReason::NO_TARGET_POSE);
    return;
  }

  // check finish
  if (enable_pa_park_) {
    if (CheckPAFinished()) {
      ILOG_INFO << "check pa finished!";
      SetParkingStatus(PARKING_FINISHED);
      UpdatePARemainDistance();
      return;
    }
  } else {
    if (CheckFinished()) {
      ILOG_INFO << "check apa finished!";
      SetParkingStatus(PARKING_FINISHED);
      return;
    }
  }

  // check failed
  if (CheckStuckFailed()) {
    ILOG_INFO << "check stuck failed!";
    CheckEgoPoseWhenPlanFaild(ParkingFailReason::STUCK_FAILED_TIME);
    return;
  }

  const double max_replan_path_dist = 0.15;
  const double obs_stuck_replan_wait_time = 1.0;

  CheckReplanParams replan_params(
      max_replan_path_dist, 0.068, apa_param.GetParam().max_replan_remain_dist,
      obs_stuck_replan_wait_time, apa_param.GetParam().max_replan_remain_dist,
      0.168, apa_param.GetParam().stuck_replan_time);
  // check replan
  if (!CheckReplan(replan_params)) {
    if (!CheckReplanParallel()) {
      ILOG_INFO << "replan is not required!";
      bool replan_by_short_trimmed_path = false;
      if (apa_param.GetParam().is_trim_limter_parallel_enable ||
          !apa_world_ptr_->GetStateMachineManagerPtr()->IsSeachingStatus()) {
        GeometryPathInput path_planner_input;
        const EgoInfoUnderSlot& ego_info_under_slot =
            apa_world_ptr_->GetSlotManagerPtr()->GetEgoInfoUnderSlot();
        path_planner_input.ego_info_under_slot = ego_info_under_slot;
        parallel_path_planner_.SetInput(path_planner_input);
        std::vector<geometry_lib::PathPoint> tmp_path_point_vec;
        tmp_path_point_vec = previous_current_path_point_global_vec_;
        parallel_path_planner_.TrimPathByLimiterPathPoint(tmp_path_point_vec,
                                                          true);
        if (tmp_path_point_vec.size() < current_path_point_global_vec_.size()) {
          current_path_point_global_vec_ = tmp_path_point_vec;
        }
        replan_by_short_trimmed_path = !PostProcessPathPara();
        // parallel_path_planner_.TrimPathByLimiterLastPathVec(false);
      }
      if (!replan_by_short_trimmed_path) {
        SetParkingStatus(PARKING_RUNNING);
        UpdatePARemainDistance();
        return;
      }
    }
  }

  // check finish
  if (enable_pa_park_) {
    if (CheckPAFinished()) {
      ILOG_INFO << "check pa finished!";
      SetParkingStatus(PARKING_FINISHED);
      UpdatePARemainDistance();
      return;
    }
  } else {
    if (CheckFinished()) {
      ILOG_INFO << "check apa finished!";
      SetParkingStatus(PARKING_FINISHED);
      return;
    }
  }

  ILOG_INFO << "replan is required!";
  // update obstacles
  GenTBoundaryObstacles();

  // path plan
  const auto pathplan_result = PathPlanOnce();
  // frame_.pathplan_result = pathplan_result;

  if (pathplan_result == PathPlannerResult::PLAN_HOLD) {
    if (PostProcessPath()) {
      SetParkingStatus(PARKING_GEARCHANGE);
      delay_check_finish_ = true;
      ILOG_INFO << "replan from PARKING_GEARCHANGE!";
    } else {
      // SetParkingStatus(PARKING_FAILED);
      // frame_.plan_fail_reason = PATH_PLAN_FAILED;
      CheckEgoPoseWhenPlanFaild(PATH_PLAN_FAILED);
      ILOG_INFO << "replan failed from PLAN_HOLD!";
    }
  } else if (pathplan_result == PathPlannerResult::PLAN_UPDATE) {
    previous_parallel_path_planner_.Reset();
    previous_parallel_path_planner_ = parallel_path_planner_;
    if (PostProcessPath()) {
      SetParkingStatus(PARKING_PLANNING);
      delay_check_finish_ = true;
      ILOG_INFO << "replan from PARKING_PLANNING!";
    } else {
      // SetParkingStatus(PARKING_FAILED);
      // frame_.plan_fail_reason = PATH_PLAN_FAILED;
      CheckEgoPoseWhenPlanFaild(PATH_PLAN_FAILED);
      ILOG_INFO << "replan failed from PARKING_PLANNING!";
    }
  } else if (pathplan_result == PathPlannerResult::PLAN_FAILED) {
    // SetParkingStatus(PARKING_FAILED);
    // frame_.plan_fail_reason = PATH_PLAN_FAILED;
    CheckEgoPoseWhenPlanFaild(PATH_PLAN_FAILED);
  }

  ILOG_INFO << "pathplan_result = " << static_cast<int>(pathplan_result);
  UpdatePARemainDistance();
}

void ParallelParkInScenario::UpdatePARemainDistance() {
  if (!enable_pa_park_) {
    return;
  }
  const double complate_distance = 0.15;
  const EgoInfoUnderSlot& ego_info_under_slot =
      apa_world_ptr_->GetSlotManagerPtr()->GetEgoInfoUnderSlot();
  Eigen::Vector2d pos_diff =
      first_plan_cur_pos.GetPos() - ego_info_under_slot.cur_pose.pos;
  const double remain_dis_y =
      complate_distance -
      std::min(std::fabs(pos_diff.y()), complate_distance) - kEps;
  ILOG_INFO << "remain_dis_y = " << remain_dis_y;
  ApaPAStateGeneral pa_state;
  pa_state.SetPARemainDistance(apa_hmi_, remain_dis_y);
  pa_state.SetPARemainDistancePercentage(
      apa_hmi_, (remain_dis_y / complate_distance));
  return;
}

const bool ParallelParkInScenario::UpdatePASlotInfo() {
  ILOG_INFO << "Enter update pa slot info!";
  ApaSlot need_move_slot;
  if (frame_.is_replan_first) {
    std::unordered_map<size_t, ApaSlot> slots_map =
        apa_world_ptr_->GetSlotManagerPtr()->GetSlotsMap();

    ApaPADirection pa_direction =
        apa_world_ptr_->GetStateMachineManagerPtr()->GetPADirection();
    if (pa_direction == ApaPADirection::PA_INVALID) {
      ILOG_ERROR << "No PA direction found";
      return false;
    }
    if (pa_direction == ApaPADirection::PA_LEFT) {
      if (slots_map.find(int(ApaPADirection::PA_LEFT)) == slots_map.end()) {
        ILOG_ERROR << "PA_LEFT found, but slot not found";
        return false;
      }
      need_move_slot = slots_map[int(ApaPADirection::PA_LEFT)];
    } else if (pa_direction == ApaPADirection::PA_RIGHT) {
      if (slots_map.find(int(ApaPADirection::PA_RIGHT)) == slots_map.end()) {
        ILOG_ERROR << "PA_RIGHT found, but slot not found";
        return false;
      }
      need_move_slot = slots_map[int(ApaPADirection::PA_RIGHT)];
    }
    first_plan_slot = need_move_slot;
  } else {
    need_move_slot = first_plan_slot;
  }
  EgoInfoUnderSlot& ego_info_under_slot =
      apa_world_ptr_->GetSlotManagerPtr()->GetMutableEgoInfoUnderSlot();
  ego_info_under_slot.slot.origin_corner_coord_global_ =
      need_move_slot.origin_corner_coord_global_;
  if (!GeneralPASlot()) {
    ILOG_ERROR << "GeneralPASlot failed!";
    return false;
  }
  return true;
}

const bool ParallelParkInScenario::ParkInTry(const ApaSlot& slot) {
  frame_.Reset();
  t_lane_.Reset();
  obs_pt_local_vec_.clear();
  parallel_path_planner_.Reset();
  previous_parallel_path_planner_.Reset();

  ILOG_INFO << "Enter ParkInTry";

  EgoInfoUnderSlot& ego_info_under_slot =
      apa_world_ptr_->GetSlotManagerPtr()->GetMutableEgoInfoUnderSlot();

  // update ego slot info
  if (enable_pa_park_) {
    ego_info_under_slot.slot.origin_corner_coord_global_ =
        slot.origin_corner_coord_global_;
    if (!GeneralPASlot()) {
      ILOG_ERROR << "GeneralPASlot failed!";
      return false;
    }
  } else {
    // update ego slot info
    if (!UpdateEgoSlotInfo()) {
      ILOG_ERROR << "UpdateEgoSlotInfo failed!";
      return false;
    }
  }

  // ExcutePathPlanningTask()
  ILOG_INFO << "Enter parallel parking try planner!";
  // init simulation
  InitSimulation();

  // generate t-lane
  if (!GenTlane()) {
    ILOG_INFO << "GenTlane failed!";
    return false;
  }

  // update obstacles
  GenTBoundaryObstacles();

  bool ret = false;
  // path plan
  const auto pathplan_result = PathPlanOnce();

  if (pathplan_result == PathPlannerResult::PLAN_HOLD) {
    if (PostProcessPath()) {
      ILOG_INFO << "replan from PARKING_GEARCHANGE!";
      ret = true;
      ILOG_INFO << "geometry path try success";
    } else {
      ILOG_INFO << "replan failed from PLAN_HOLD!";
    }
  } else if (pathplan_result == PathPlannerResult::PLAN_UPDATE) {
    if (PostProcessPath()) {
      ILOG_INFO << "replan from PARKING_PLANNING!";
      ret = true;
      ILOG_INFO << "geometry path try success";
    } else {
      ILOG_INFO << "replan failed from PARKING_PLANNING!";
    }
  } else if (pathplan_result == PathPlannerResult::PLAN_FAILED) {
    ILOG_INFO << "geometry path try fail";
  }
  frame_.is_replan_first = true;

  frame_.Reset();
  t_lane_.Reset();
  obs_pt_local_vec_.clear();
  parallel_path_planner_.Reset();
  previous_parallel_path_planner_.Reset();
  return ret;
}

void ParallelParkInScenario::ScenarioTry() {
  ILOG_INFO << "Enter ScenarioTry";
  if (apa_world_ptr_->GetStateMachineManagerPtr()->IsParkingStatus()) {
    return;
  }
  EgoInfoUnderSlot& ego_info_under_slot =
      apa_world_ptr_->GetSlotManagerPtr()->GetMutableEgoInfoUnderSlot();
  ego_info_under_slot.slot.release_info_
      .release_state[SlotReleaseMethod::GEOMETRY_PLANNING_RELEASE] =
      SlotReleaseState::NOT_RELEASE;
  if (apa_world_ptr_->GetStateMachineManagerPtr()->IsSAPAMode() &&
      !apa_world_ptr_->GetStateMachineManagerPtr()->GetFreeSlotPosDir()) {
    ego_info_under_slot.slot.release_info_
        .release_state[SlotReleaseMethod::ASTAR_PLANNING_RELEASE] =
        SlotReleaseState::NOT_RELEASE;
    ILOG_INFO
        << "free slot not support head parking posdir: "
        << apa_world_ptr_->GetStateMachineManagerPtr()->GetFreeSlotPosDir();
    return;
  }

  enable_pa_park_ = false;
  if (apa_world_ptr_->GetStateMachineManagerPtr()->GetParkRunningMode() ==
      ApaRunningMode::RUNNING_PA) {
    enable_pa_park_ = true;
  }
  if (enable_pa_park_) {
    ApaPAStateGeneral pa_state;
    pa_state.ClearPADirectionFlag(apa_hmi_);
    std::unordered_map<size_t, ApaSlot> slots_map =
        apa_world_ptr_->GetSlotManagerPtr()->GetSlotsMap();
    for (auto iter = slots_map.begin(); iter != slots_map.end(); iter++) {
      ILOG_INFO << "slot id = " << iter->first << " try begin";
      if (ParkInTry(iter->second)) {
        ILOG_INFO << "slot id = " << iter->first << " try success";
        pa_state.SetPADirectionFlag(apa_hmi_,
                                    APAPaRecommendedDirection::PaParitybit);
        if (iter->first == int(ApaPADirection::PA_LEFT)) {
          pa_state.SetPADirectionFlag(apa_hmi_,
                                      APAPaRecommendedDirection::PaLeft);
          multi_parkin_path_vec_[int(ApaPADirection::PA_LEFT)] =
              complete_path_point_global_vec_;
        } else if (iter->first == int(ApaPADirection::PA_RIGHT)) {
          pa_state.SetPADirectionFlag(apa_hmi_,
                                      APAPaRecommendedDirection::PaRight);
          multi_parkin_path_vec_[int(ApaPADirection::PA_RIGHT)] =
              complete_path_point_global_vec_;
        }
      }
    }
    ILOG_INFO << "planning_park_pa_dir = " << apa_hmi_.planning_park_pa_dir;
    pa_state.ClearRecommendPADirectionFlag(apa_hmi_);
    if (apa_hmi_.planning_park_pa_dir) {
      complete_path_point_global_vec_.clear();
      if (multi_parkin_path_vec_.find(int(ApaPADirection::PA_RIGHT)) !=
          multi_parkin_path_vec_.end()) {
        complete_path_point_global_vec_ =
            multi_parkin_path_vec_[int(ApaPADirection::PA_RIGHT)];
        pa_state.SetRecommendPADirectionFlag(
            apa_hmi_, APAPaRecommendedDirection::PaRight);
        ILOG_INFO << "complete path is PA_RIGHT";
      } else if (multi_parkin_path_vec_.find(int(ApaPADirection::PA_LEFT)) !=
                 multi_parkin_path_vec_.end()) {
        complete_path_point_global_vec_ =
            multi_parkin_path_vec_[int(ApaPADirection::PA_LEFT)];
        pa_state.SetRecommendPADirectionFlag(
            apa_hmi_, APAPaRecommendedDirection::PaLeft);
        ILOG_INFO << "complete path is PA_LEFT";
      } else {
        ILOG_INFO << "multi_parkin_path_vec_ is empty";
      }
      ego_info_under_slot.slot.release_info_
          .release_state[SlotReleaseMethod::GEOMETRY_PLANNING_RELEASE] =
          SlotReleaseState::RELEASE;
    } else {
      ego_info_under_slot.slot.release_info_
          .release_state[SlotReleaseMethod::GEOMETRY_PLANNING_RELEASE] =
          SlotReleaseState::NOT_RELEASE;
      ego_info_under_slot.slot.release_info_
          .release_state[SlotReleaseMethod::ASTAR_PLANNING_RELEASE] =
          SlotReleaseState::NOT_RELEASE;
    }
  } else {
    ApaSlot slot;
    bool ret = ParkInTry(slot);
    if (!ret) {
      ego_info_under_slot.slot.release_info_
          .release_state[SlotReleaseMethod::GEOMETRY_PLANNING_RELEASE] =
          SlotReleaseState::NOT_RELEASE;
      ego_info_under_slot.slot.release_info_
          .release_state[SlotReleaseMethod::ASTAR_PLANNING_RELEASE] =
          SlotReleaseState::NOT_RELEASE;
      ILOG_INFO << "ParkInTry failed";
    } else {
      ego_info_under_slot.slot.release_info_
          .release_state[SlotReleaseMethod::GEOMETRY_PLANNING_RELEASE] =
          SlotReleaseState::RELEASE;
      ILOG_INFO << "ParkInTry success";
    }
  }
  TansformPreparePlanningTraj();
  return;
}

const double ParallelParkInScenario::CalRealTimeBrakeDist() {
  double lat_buffer = 0.0;
  double safe_uss_remain_dist = 0.0;
  CalStaticBufferInDiffSteps(lat_buffer, safe_uss_remain_dist);
  ILOG_INFO << "parallel lat_buffer = " << lat_buffer;
  ILOG_INFO << "parallel safe_uss_remain_dist = " << safe_uss_remain_dist;

  double dynaminc_lat_buffer = 0.0;
  double dynamic_lon_buffer = 0.0;
  CalDynamicBufferInDiffSteps(dynaminc_lat_buffer, dynamic_lon_buffer);

  apa_world_ptr_->GetColDetInterfacePtr()->Init(true);

  const double remain_dist_obs = CalRemainDistFromObs(
      safe_uss_remain_dist, lat_buffer, lat_buffer, dynamic_lon_buffer,
      dynaminc_lat_buffer, dynaminc_lat_buffer, false,
      apa_param.GetParam().use_obs_height_method, false);

  ILOG_INFO << "final remain_dist_obs = " << remain_dist_obs;

  return remain_dist_obs;
}

const bool ParallelParkInScenario::IsPointInOrientedRectangle(
    const Eigen::Vector2d& point, const Eigen::Vector2d& rect_center,
    const double rect_heading, const double rect_length,
    const double rect_width) {
  // Calculate the transformation from the global to local
  Eigen::Matrix2d rotation_matrix;
  rotation_matrix << cos(-rect_heading), -sin(-rect_heading),
      sin(-rect_heading), cos(-rect_heading);

  // Convert the point to the local coordinate system of the rectangle
  Eigen::Vector2d local_point = rotation_matrix * (point - rect_center);

  // Check whether the checkpoint is within the rectangle
  return (std::abs(local_point.x()) <= rect_length * 0.5) &&
         (std::abs(local_point.y()) <= rect_width * 0.5);
}

SlotCoord ParallelParkInScenario::AlignAndMoveSlotToLine(
    const SlotCoord& slot, const Eigen::Vector2d& ego_pos,
    const Eigen::Vector2d& ego_head, double k, double b, double slot_width,
    double distance) {
  SlotCoord aligned_slot;

  // 1. Calculate the center point of the original parking space
  Eigen::Vector2d center =
      0.25 * (slot.pt_0 + slot.pt_1 + slot.pt_2 + slot.pt_3);

  // 2. Calculate the long side vector (0, 1 side)
  Eigen::Vector2d long_edge = slot.pt_0 - slot.pt_1;
  double long_edge_length = long_edge.norm();

  // 3. Calculate the short side vector (0, 1 side)
  Eigen::Vector2d short_edge = slot.pt_2 - slot.pt_0; //debug shuaili26
  double short_edge_length = short_edge.norm();

  // 4. Calculate the direction vector and normal vector
  Eigen::Vector2d line_direction(1.0, k);
  line_direction.normalize();

  Eigen::Vector2d line_normal(-k, 1.0);
  line_normal.normalize();

  const double dot = long_edge.dot(line_direction);
  if (dot < 0.0) {
    line_direction = -line_direction;
    line_normal = -line_normal;
  }

  ILOG_INFO << "line_normal = " << line_normal.x() << ", " << line_normal.y();
  // 5. Calculate the signed distance from the current center point to the
  // straight line
  double current_distance =
      std::fabs((k * center.x() - center.y() + b) / std::sqrt(k * k + 1));
  ILOG_INFO << "current_distance = " << current_distance;
  ILOG_INFO << "center = " << center.x() << ", " << center.y();

  current_distance = current_distance - 0.5 * slot_width;

  Eigen::Vector2d ego_head_normal(-ego_head.y(), ego_head.x());
  Eigen::Vector2d front_out_wheel = ego_pos +
                                    (apa_param.GetParam().wheel_base +
                                     apa_param.GetParam().front_overhanging) *
                                        ego_head +
                                    0.5 * apa_param.GetParam().car_width *
                                        t_lane_.slot_side_sgn *
                                        (-ego_head_normal);
  Eigen::Vector2d rear_out_wheel =
      ego_pos - apa_param.GetParam().rear_overhanging * ego_head +
      0.5 * apa_param.GetParam().car_width * t_lane_.slot_side_sgn *
          (-ego_head_normal);
  const double front_distance =
      std::fabs((k * front_out_wheel.x() - front_out_wheel.y() + b) /
                std::sqrt(k * k + 1));
  const double rear_distance = std::fabs(
      (k * rear_out_wheel.x() - rear_out_wheel.y() + b) / std::sqrt(k * k + 1));
  ILOG_INFO << "front_out_wheel: " << front_out_wheel.x() << " "
            << front_out_wheel.y() << " rear_out_wheel: " << rear_out_wheel.x()
            << " " << rear_out_wheel.y()
            << " front_distance = " << front_distance
            << " rear_distance = " << rear_distance;

  current_distance = std::min(rear_distance, front_distance);
  t_lane_.car_to_curb_dis = current_distance;

  // 6. Calculate the distance that needs to be moved
  // (only approaching the straight line, not crossing to the other side)
  double move_distance = 0.0;
  if (std::abs(current_distance) > distance) {
    // The current distance is greater than the target distance,
    // and it is necessary to approach in a straight line
    move_distance = current_distance - distance;
    move_distance =
        std::min(move_distance, apa_param.GetParam().pa_slot_move_distance);
  }

  const double ego_distance =
      std::fabs((k * ego_pos.x() - ego_pos.y() + b) / std::sqrt(k * k + 1));
  const double slot_distance =
      std::fabs((k * center.x() - center.y() + b) / std::sqrt(k * k + 1));

  const double ego_to_slot = std::fabs(ego_distance - slot_distance);

  if (std::fabs(ego_pos.y() - center.y()) > 0.01 &&
      ego_to_slot > apa_param.GetParam().pa_slot_move_distance) {
    line_normal *= -1.0;
    move_distance = ego_to_slot - apa_param.GetParam().pa_slot_move_distance;
    ILOG_INFO << "reverse line_normal = " << line_normal.x() << ", "
              << line_normal.y();
  } else if (move_distance < 0.05) {
    ILOG_INFO << "move_distance < 0.05, distance is too small";
    return aligned_slot;
  }
  // If the current distance is already less than or equal to
  // the target distance, there is no need to move (move_distance remains at 0)
  ILOG_INFO << "current_distance = " << current_distance
            << " move_distance = " << move_distance;

  // 7. Calculate the position of the target center point
  Eigen::Vector2d target_center =
      center - move_distance * line_normal * t_lane_.slot_side_sgn;
  ILOG_INFO << "target_center = " << target_center.x() << ", "
            << target_center.y();

  // 8. Calculate the new long side direction (parallel to the straight line)
  Eigen::Vector2d new_long_edge = line_direction * long_edge_length;

  // 9. Calculate the new short side direction (perpendicular to the long side)
  Eigen::Vector2d new_short_edge(-line_direction.y(), line_direction.x());
  new_short_edge *= short_edge_length * t_lane_.slot_side_sgn;

  // 10. Calculate the new four vertices based
  // on the new edge vectors and center points
  Eigen::Vector2d half_long = 0.5 * new_long_edge;
  Eigen::Vector2d half_short = 0.5 * new_short_edge;

  aligned_slot.pt_0 = target_center + half_long + half_short;
  aligned_slot.pt_1 = target_center - half_long + half_short;
  aligned_slot.pt_2 = target_center - half_long - half_short;
  aligned_slot.pt_3 = target_center + half_long - half_short;

  return aligned_slot;
}

const bool ParallelParkInScenario::CheckPAFinished() {
  ILOG_INFO << "Enter CheckPAFinished!";

  const EgoInfoUnderSlot& ego_slot_info =
      apa_world_ptr_->GetSlotManagerPtr()->GetEgoInfoUnderSlot();

  pnc::geometry_lib::PathPoint terminal_err(
      first_plan_cur_pos.pos - ego_slot_info.cur_pose.pos,
      pnc::geometry_lib::NormalizeAngle(
          first_plan_cur_pos.heading -
          ego_slot_info.cur_pose.heading));

  // ego_slot_info.terminal_err.pos;
  const bool lon_condition =
      std::fabs(ego_slot_info.terminal_err.pos.x()) <
      apa_param.GetParam().finish_parallel_pa_lon_err;

  ILOG_INFO << "terminal x error = " << ego_slot_info.terminal_err.pos.x();
  ILOG_INFO << "lon_condition = " << lon_condition;

  const bool heading_condition =
      std::fabs(ego_slot_info.terminal_err.heading) <
      apa_param.GetParam().finish_parallel_pa_heading_err * kDeg2Rad;

  ILOG_INFO << "terminal heading error = "
            << ego_slot_info.terminal_err.heading * kRad2Deg;
  ILOG_INFO << "heading_condition = " << heading_condition;

  ILOG_INFO << "lat error = " << terminal_err.pos.y();
  const bool lat_move_condition =
      std::fabs(terminal_err.pos.y()) >
      apa_param.GetParam().finish_parallel_pa_lat_err;
  const bool lat_err_y_condition =
      std::fabs(ego_slot_info.terminal_err.pos.y()) <= 0.05;
  const bool lat_condition = lat_move_condition || lat_err_y_condition;

  const bool static_condition =
      apa_world_ptr_->GetMeasureDataManagerPtr()->GetStaticFlag();

  ILOG_INFO << "static_condition = " << static_condition;

  if (frame_.total_plan_count > kMaxReplanTimes) {
    ILOG_INFO << "replan times too much, only check heading and x axis";
    return lon_condition && static_condition && heading_condition;
  }

  return lon_condition && lat_condition && heading_condition &&
         static_condition;
}

void ParallelParkInScenario::ExtractLongestLineSegmentPointsPCA(
    const std::vector<Eigen::Vector2d>& obs_pos,
    std::vector<Eigen::Vector2d>& pa_curb_obs, Eigen::Vector2d& line_coeffs,
    double distance_threshold) {
  ILOG_INFO << "Enter ExtractLongestLineSegmentPointsPCA!";
  if (obs_pos.size() < 3) {
    return;
  }

  // 1. Calculate the centroid of the point set
  Eigen::Vector2d centroid(0, 0);
  for (const auto& point : obs_pos) {
    centroid += point;
  }
  centroid /= obs_pos.size();

  // 2. Construct the covariance matrix and perform PCA
  Eigen::Matrix2d covariance = Eigen::Matrix2d::Zero();
  for (const auto& point : obs_pos) {
    Eigen::Vector2d diff = point - centroid;
    covariance += diff * diff.transpose();
  }
  covariance /= obs_pos.size();

  // 3. Calculate the eigenvalues and eigenvectors
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> eigensolver(covariance);
  Eigen::Vector2d eigenvalues = eigensolver.eigenvalues();
  Eigen::Matrix2d eigenvectors = eigensolver.eigenvectors();

  // 4. Obtain the main direction
  // (the eigenvector corresponding to the maximum eigenvalue)
  Eigen::Vector2d main_direction = eigenvectors.col(1).normalized();

  // 4.1 cal line_coeffs
  line_coeffs.x() = main_direction.y() / main_direction.x();
  line_coeffs.y() = centroid.y() - line_coeffs.x() * centroid.x();

  // 5. Project all the points onto the main direction
  std::vector<std::pair<double, int>> projections;
  for (size_t i = 0; i < obs_pos.size(); ++i) {
    double projection = (obs_pos[i] - centroid).dot(main_direction);
    projections.push_back({projection, static_cast<int>(i)});
  }

  // 6. Sort by projected value
  std::sort(projections.begin(), projections.end());

  // 7. Find consecutive line segment points
  std::vector<bool> is_linear_point(obs_pos.size(), false);
  std::vector<std::vector<int>> segments;
  std::vector<int> current_segment;

  for (size_t i = 0; i < projections.size(); ++i) {
    int idx = projections[i].second;
    const Eigen::Vector2d& point = obs_pos[idx];

    // Calculate the distance from the point to the straight line
    // in the main direction
    double distance = std::abs(
        (point - centroid)
            .dot(Eigen::Vector2d(-main_direction.y(), main_direction.x())));

    if (distance < distance_threshold) {
      is_linear_point[idx] = true;
      current_segment.push_back(idx);
    } else {
      if (!current_segment.empty()) {
        segments.push_back(current_segment);
        current_segment.clear();
      }
    }
  }

  if (!current_segment.empty()) {
    segments.push_back(current_segment);
  }

  // 8. Find the longest continuous line segment
  size_t max_segment_size = 0;
  int max_segment_index = -1;

  for (size_t i = 0; i < segments.size(); ++i) {
    if (segments[i].size() > max_segment_size) {
      max_segment_size = segments[i].size();
      max_segment_index = static_cast<int>(i);
    }
  }

  // 9. Return the point on the longest line segment
  if (max_segment_index >= 0) {
    for (int idx : segments[max_segment_index]) {
      pa_curb_obs.push_back(obs_pos[idx]);
    }
  }

  return;
}

const bool ParallelParkInScenario::GeneralPASlot() {
  EgoInfoUnderSlot& ego_info_under_slot =
      apa_world_ptr_->GetSlotManagerPtr()->GetMutableEgoInfoUnderSlot();

  auto& select_slot_global =
      ego_info_under_slot.slot.origin_corner_coord_global_;
  ILOG_INFO << " select_slot_global pt 0 = " << select_slot_global.pt_0.x()
            << ", " << select_slot_global.pt_0.y();
  ILOG_INFO << " select_slot_global pt 1 = " << select_slot_global.pt_1.x()
            << ", " << select_slot_global.pt_1.y();
  ILOG_INFO << " select_slot_global pt 2 = " << select_slot_global.pt_2.x()
            << ", " << select_slot_global.pt_2.y();
  ILOG_INFO << " select_slot_global pt 3 = " << select_slot_global.pt_3.x()
            << ", " << select_slot_global.pt_3.y();

  if (select_slot_global.pt_0 == select_slot_global.pt_1 ||
      select_slot_global.pt_0 == select_slot_global.pt_2 ||
      select_slot_global.pt_0 == select_slot_global.pt_3 ||
      select_slot_global.pt_1 == select_slot_global.pt_2 ||
      select_slot_global.pt_1 == select_slot_global.pt_3 ||
      select_slot_global.pt_2 == select_slot_global.pt_3) {
    ILOG_ERROR << "slot corner points exist same pt!";
    return false;
  }

  const auto measures_ptr = apa_world_ptr_->GetMeasureDataManagerPtr();

  Eigen::Vector2d v_10 =
      (select_slot_global.pt_0 - select_slot_global.pt_1).normalized();

  if (v_10.dot(measures_ptr->GetHeadingVec()) < 1e-9) {
    v_10 *= -1.0;
    select_slot_global.pt_0.swap(select_slot_global.pt_1);
    select_slot_global.pt_2.swap(select_slot_global.pt_3);
  }
  select_slot_global.CalExtraCoord();
  const double heading_10 = std::atan2(v_10.y(), v_10.x());

  const pnc::geometry_lib::LineSegment line_01(
      select_slot_global.pt_1, select_slot_global.pt_0, heading_10);

  const double dist_01_2 =
      pnc::geometry_lib::CalPoint2LineDist(select_slot_global.pt_2, line_01);

  const double dist_01_3 =
      pnc::geometry_lib::CalPoint2LineDist(select_slot_global.pt_3, line_01);

  ego_info_under_slot.slot.slot_width_ = std::min(dist_01_2, dist_01_3);
  ego_info_under_slot.slot.slot_length_ =
      (select_slot_global.pt_0 - select_slot_global.pt_1).norm();

  ILOG_INFO << "slot_length_ = " << ego_info_under_slot.slot.slot_length_;
  ILOG_INFO << "slot_width = " << ego_info_under_slot.slot.slot_width_;
  if (ego_info_under_slot.slot.slot_length_ <
      ego_info_under_slot.slot.slot_width_) {
    ILOG_ERROR << " slot_length_ > slot_width_ in parallel slot";
    return false;
  }

  if (frame_.is_replan_first) {
    const auto v_23mid_to_01 =
        ego_info_under_slot.slot.origin_corner_coord_global_.pt_23mid_01mid_vec
            .normalized();

    const double cross_ego_to_slot_heading =
        pnc::geometry_lib::GetCrossFromTwoVec2d(measures_ptr->GetHeadingVec(),
                                                v_23mid_to_01);
    ILOG_INFO << "v_23mid_to_01 " << v_23mid_to_01.x() << " "
              << v_23mid_to_01.y();

    frame_.current_gear = pnc::geometry_lib::SEG_GEAR_REVERSE;
    if (cross_ego_to_slot_heading > 1e-3) {
      t_lane_.slot_side_sgn = 1.0;
      t_lane_.slot_side = pnc::geometry_lib::SLOT_SIDE_RIGHT;
      ego_info_under_slot.slot_side = pnc::geometry_lib::SLOT_SIDE_RIGHT;

    } else if (cross_ego_to_slot_heading < -1e-3) {
      t_lane_.slot_side_sgn = -1.0;
      t_lane_.slot_side = geometry_lib::SLOT_SIDE_LEFT;
      ego_info_under_slot.slot_side = pnc::geometry_lib::SLOT_SIDE_LEFT;

    } else {
      t_lane_.slot_side = pnc::geometry_lib::SLOT_SIDE_INVALID;
      ego_info_under_slot.slot_side = pnc::geometry_lib::SLOT_SIDE_INVALID;
      ILOG_INFO << "calculate parallel slot side error ";
      return false;
    }

    const Eigen::Vector2d& mid01 = select_slot_global.pt_01_mid;
    const Eigen::Vector2d& mid23 = select_slot_global.pt_23_mid;
    Eigen::Vector2d rect_center =
        mid23 + (mid23 - mid01).normalized() * 0.5;  // 延伸x米
    double original_heading =
        std::atan2((mid23 - mid01).y(), (mid23 - mid01).x());
    double rect_heading = pnc::geometry_lib::NormalizeAngle(
        original_heading + M_PI / 2.0);  // 逆时针旋转90度
    double rect_length = ego_info_under_slot.slot.slot_length_;  // 长
    double rect_width = 1.2;                                     // 宽
    ILOG_INFO << "rect_center = " << rect_center.x() << " " << rect_center.y();
    ILOG_INFO << "rect_heading = " << rect_heading;
    // Eigen::Vector2d pt_01to23_vec =
    bool is_rigid = false;
    std::unordered_map<size_t, std::vector<Eigen::Vector2d>> pa_curb_obs_map;
    for (const auto& pair :
         apa_world_ptr_->GetObstacleManagerPtr()->GetObstacles()) {
      if (pair.second.GetObsMovementType() != ApaObsMovementType::STATIC) {
        continue;
      }
      const auto obs_scement = pair.second.GetObsScemanticType();

      is_rigid = (obs_scement == ApaObsScemanticType::WALL ||
                  obs_scement == ApaObsScemanticType::COLUMN ||
                  obs_scement == ApaObsScemanticType::CAR);
      if (obs_scement == ApaObsScemanticType::UNKNOWN ||
          obs_scement == ApaObsScemanticType::WALL) {
        for (const auto& obs_pt_local : pair.second.GetPtClout2dGlobal()) {
          if (IsPointInOrientedRectangle(obs_pt_local, rect_center,
                                         rect_heading, rect_length,
                                         rect_width)) {
            // pa_curb_obs.emplace_back(obs_pt_local);
            pa_curb_obs_map[pair.first].emplace_back(obs_pt_local);
          }
        }
      }
    }
    Eigen::Vector2d line_coeffs_out;
    std::vector<Eigen::Vector2d> all_pa_curb_obs;
    size_t maximum_obs_id = 0;
    size_t maximum_obs_size = 0;
    for (const auto& cur_obs : pa_curb_obs_map) {
      if (cur_obs.second.size() > maximum_obs_size) {
        maximum_obs_id = cur_obs.first;
        maximum_obs_size = cur_obs.second.size();
      }
      all_pa_curb_obs.insert(all_pa_curb_obs.end(), cur_obs.second.begin(),
                             cur_obs.second.end());
    }
    std::vector<Eigen::Vector2d> pa_curb_obs;
    Eigen::Vector2d ego_pose = measures_ptr->GetPos();
    const double eps = 0.01;
    const double c2s_dis = pnc::geometry_lib::CalTwoPointDistSquare(
        select_slot_global.pt_center,
        ego_pose + v_10 * apa_param.GetParam().wheel_base * 0.5);
    ILOG_INFO << "debug: " << ego_pose.y() << " "
              << select_slot_global.pt_center.y() << " "
              << measures_ptr->GetHeading() << " " << heading_10
              << " c2s_dis = " << c2s_dis;
    const bool is_line_slot =
        std::fabs(measures_ptr->GetHeading() - heading_10) > eps ||
        c2s_dis > eps;
    t_lane_.use_sturn_plan = false;
    if (pa_curb_obs_map[maximum_obs_id].size() < 3) {
      t_lane_.use_sturn_plan = true;
      ILOG_INFO << "pa curb obs size too very little";
      if (is_line_slot) {
        line_coeffs_out.x() = v_10.y() / v_10.x();
        line_coeffs_out.y() = select_slot_global.pt_2.y() -
                              line_coeffs_out.x() * select_slot_global.pt_3.x();
      } else {
        line_coeffs_out.x() = v_10.y() / v_10.x();
        Eigen::Vector2d v_10_normal(-v_10.y(), v_10.x());
        Eigen::Vector2d move_normal =
            v_10_normal.normalized() * t_lane_.slot_side_sgn *
            apa_param.GetParam().pa_slot_move_distance;
        move_normal = select_slot_global.pt_2 + move_normal;
        line_coeffs_out.y() =
            move_normal.y() - line_coeffs_out.x() * move_normal.x();
      }
      // return false;
    } else {
      if (is_line_slot) {
        // line slot to pa
        line_coeffs_out.x() = v_10.y() / v_10.x();
        line_coeffs_out.y() = select_slot_global.pt_2.y() -
                              line_coeffs_out.x() * select_slot_global.pt_2.x();
      } else {
        // space slot to pa
        Eigen::Vector2d line_coeffs;
        auto apa_obs_map =
            apa_world_ptr_->GetObstacleManagerPtr()->GetObstacles();
        if (apa_obs_map.find(maximum_obs_id) != apa_obs_map.end()) {
          ExtractLongestLineSegmentPointsPCA(pa_curb_obs_map[maximum_obs_id],
                                             pa_curb_obs, line_coeffs);
        }
        if (pa_curb_obs.size() < 3) {
          ILOG_INFO << "pa_curb_obs size < 3";
          return false;
        }
        ILOG_INFO << "pa_curb_obs size = " << pa_curb_obs.size();

        ILOG_INFO << "line_coeffs = " << line_coeffs.x() << " "
                  << line_coeffs.y();
        std::vector<bool> inliers_mask;
        bool res = GetOffsetLineCoeffsWithEgoPose(line_coeffs_out, line_coeffs,
                                                  all_pa_curb_obs, inliers_mask,
                                                  ego_pose);
        if (!res) {
          ILOG_ERROR << "GetOffsetLineCoeffsWithEgoPose failed!";
          return false;
        }
      }
    }
    ILOG_INFO << "line_coeffs_out = " << line_coeffs_out.x() << " "
              << line_coeffs_out.y();
    first_line_coeffs_ = line_coeffs_out;
    slot2curb_dist_ = is_rigid ? 0.45 : 0.15;  // TODO debug
    // calc slot side once at first
    slot_new_ = AlignAndMoveSlotToLine(
        select_slot_global, measures_ptr->GetPos(),
        measures_ptr->GetHeadingVec(), first_line_coeffs_.x(),
        first_line_coeffs_.y(), ego_info_under_slot.slot.slot_width_,
        slot2curb_dist_);
    if ((slot_new_.pt_0 - slot_new_.pt_1).norm() <=
        apa_param.GetParam().static_pos_eps) {
      ILOG_INFO << "first plan move slot failed";
      return false;
    }
    line_coeffs_out_ = first_line_coeffs_;
    RecordDebugPaInfo(first_line_coeffs_, slot_new_, pa_curb_obs);
  } else {
    if ((slot_new_.pt_0 - slot_new_.pt_1).norm() <=
        apa_param.GetParam().static_pos_eps) {
      ILOG_INFO << "other plan move slot failed";
      return false;
    }
  }
  ILOG_INFO << "slot_side = " << static_cast<int>(t_lane_.slot_side);

  ILOG_INFO << "slot_new pt 0 = " << slot_new_.pt_0.x() << " "
            << slot_new_.pt_0.y();
  ILOG_INFO << "slot_new pt 1 = " << slot_new_.pt_1.x() << " "
            << slot_new_.pt_1.y();
  ILOG_INFO << "slot_new pt 2 = " << slot_new_.pt_2.x() << " "
            << slot_new_.pt_2.y();
  ILOG_INFO << "slot_new pt 3 = " << slot_new_.pt_3.x() << " "
            << slot_new_.pt_3.y();
  slot_new_.CalExtraCoord();
  // return true;

  const Eigen::Vector2d n = (slot_new_.pt_0 - slot_new_.pt_1).normalized();
  ILOG_INFO << "n = " << n.x() << " " << n.y();
  const Eigen::Vector2d t(-n.y(), n.x());
  ILOG_INFO << "t = " << t.x() << " " << t.y();

  ego_info_under_slot.origin_pose_global.heading = std::atan2(n.y(), n.x());
  ILOG_INFO << " ego_info_under_slot.origin_pose_global.heading = "
            << ego_info_under_slot.origin_pose_global.heading * kRad2Deg;
  ego_info_under_slot.origin_pose_global.heading_vec = n;

  const double target_y_sgn =
      (ego_info_under_slot.slot_side == pnc::geometry_lib::SLOT_SIDE_LEFT
           ? 1.0
           : -1.0);
  const double target_y = 0.5 * ego_info_under_slot.slot.GetWidth();

  ego_info_under_slot.origin_pose_global.pos =
      slot_new_.pt_1 + target_y * target_y_sgn * t;

  ILOG_INFO << "ego_info_under_slot.origin_pose_global.pos = "
            << ego_info_under_slot.origin_pose_global.pos.x() << " "
            << ego_info_under_slot.origin_pose_global.pos.y();

  ego_info_under_slot.g2l_tf = pnc::geometry_lib::GlobalToLocalTf(
      ego_info_under_slot.origin_pose_global.pos,
      ego_info_under_slot.origin_pose_global.heading);

  ego_info_under_slot.l2g_tf = pnc::geometry_lib::LocalToGlobalTf(
      ego_info_under_slot.origin_pose_global.pos,
      ego_info_under_slot.origin_pose_global.heading);

  ego_info_under_slot.origin_pose_local.pos << 0.0, 0.0;
  ego_info_under_slot.origin_pose_local.heading = 0.0;
  ego_info_under_slot.origin_pose_local.heading_vec << 1.0, 0.0;
  ego_info_under_slot.slot.TransformCoordFromGlobalToLocal(
      ego_info_under_slot.g2l_tf);

  ego_info_under_slot.cur_pose.pos =
      ego_info_under_slot.g2l_tf.GetPos(measures_ptr->GetPos());
  ego_info_under_slot.cur_pose.heading =
      ego_info_under_slot.g2l_tf.GetHeading(measures_ptr->GetHeading());
  ego_info_under_slot.cur_pose.heading_vec =
      pnc::geometry_lib::GenHeadingVec(ego_info_under_slot.cur_pose.heading);

  ILOG_INFO << "ego_pos_slot = " << ego_info_under_slot.cur_pose.pos.x() << " "
            << ego_info_under_slot.cur_pose.pos.y();
  ILOG_INFO << "ego_heading_slot (deg)= "
            << ego_info_under_slot.cur_pose.heading * kRad2Deg;

  ILOG_INFO << "slot width =" << ego_info_under_slot.slot.slot_width_;
  ILOG_INFO << "t_lane_.slot_side = " << static_cast<int>(t_lane_.slot_side);
  ILOG_INFO << "frame_.current_arc_steer = "
            << static_cast<int>(frame_.current_arc_steer);

  // firstly locate in middle of slot
  double target_x = 0.5 * (ego_info_under_slot.slot.slot_length_ -
                           apa_param.GetParam().car_length) +
                    apa_param.GetParam().rear_overhanging;

  ILOG_INFO << "target x in middle = " << target_x;

  const auto& limiter = ego_info_under_slot.slot.GetLimiter();
  ILOG_INFO << "limiter.valid = " << limiter.valid;
  if (limiter.valid) {
    t_lane_.limiter.valid = true;
    // transfer limiter in slot coordination
    t_lane_.limiter.start_pt =
        ego_info_under_slot.g2l_tf.GetPos(limiter.start_pt);

    t_lane_.limiter.end_pt = ego_info_under_slot.g2l_tf.GetPos(limiter.end_pt);

    const double max_limiter_x =
        std::max(t_lane_.limiter.start_pt.x(), t_lane_.limiter.end_pt.x());

    const double min_limiter_x =
        std::min(t_lane_.limiter.start_pt.x(), t_lane_.limiter.end_pt.x());

    if (max_limiter_x < 0.5 * ego_info_under_slot.slot.slot_length_) {
      ILOG_INFO << "limiter behind!";
      target_x = std::max(
          target_x,
          max_limiter_x +
              apa_param.GetParam().parallel_ego_ac_x_offset_with_limiter);
    } else {
      ILOG_INFO << "limiter front!";
      target_x = std::min(
          target_x,
          min_limiter_x - apa_param.GetParam().wheel_base -
              apa_param.GetParam().parallel_ego_ac_x_offset_with_limiter);
    }
  }
  ILOG_INFO << "target x consider limiter = " << target_x;
  ego_info_under_slot.target_pose.pos << target_x, 0.0;
  ego_info_under_slot.target_pose.heading = 0.0;
  ego_info_under_slot.target_pose.heading_vec << 1.0, 0.0;

  // calc terminal error once
  ego_info_under_slot.terminal_err.Set(
      ego_info_under_slot.cur_pose.pos - ego_info_under_slot.target_pose.pos,
      pnc::geometry_lib::NormalizeAngle(
          ego_info_under_slot.cur_pose.heading -
          ego_info_under_slot.target_pose.heading));

  // calc slot occupied ratio
  double slot_occupied_ratio = 0.0;
  const double y_err_ratio = ego_info_under_slot.terminal_err.pos.y() /
                               (0.5 * ego_info_under_slot.slot.slot_width_);
  if (t_lane_.slot_side == pnc::geometry_lib::SLOT_SIDE_RIGHT) {
    slot_occupied_ratio = pnc::mathlib::Clamp(1 - y_err_ratio, 0.0, 1.0);
  } else if (t_lane_.slot_side == pnc::geometry_lib::SLOT_SIDE_LEFT) {
    slot_occupied_ratio = pnc::mathlib::Clamp(1.0 + y_err_ratio, 0.0, 1.0);
  }
  if (pnc::mathlib::IsInBound(ego_info_under_slot.terminal_err.pos.x(), -4.0,
                              4.0)) {
    ego_info_under_slot.slot_occupied_ratio = slot_occupied_ratio;
  }
  if (pnc::mathlib::IsInBound(ego_info_under_slot.terminal_err.pos.x(), -5.0,
                              4.0) &&
      slot_occupied_ratio > 0.75) {
    ego_info_under_slot.slot_occupied_ratio = slot_occupied_ratio;
  }
  if (frame_.is_replan_first && enable_pa_park_) {
    first_plan_cur_pos = ego_info_under_slot.cur_pose;
    t_lane_.first_occupied_ratio = slot_occupied_ratio;
  }

  ILOG_INFO << "ego_slot_info.slot_occupied_ratio = "
            << ego_info_under_slot.slot_occupied_ratio;

  return true;
}

void ParallelParkInScenario::RecordDebugPaInfo(
    const Eigen::Vector2d& line_coeffs, const SlotCoord& slot,
    const std::vector<Eigen::Vector2d>& pa_curb_obs) {
  auto& debug = DebugInfoManager::GetInstance().GetDebugInfoPb();
  debug->mutable_apa_path_debug()->clear_pa_objects();
  // add line: y = kx + b
  planning::common::PaObject* line_object =
      debug->mutable_apa_path_debug()->add_pa_objects();
  line_object->set_category(planning::common::PaCategory::LINE);
  line_object->set_object_id("line_1");

  // add: k, b
  planning::common::TrajectoryPoint* start_point = line_object->add_points();
  start_point->set_x(line_coeffs.x());
  start_point->set_y(line_coeffs.y());

  // add slot points
  planning::common::PaObject* slot_object =
      debug->mutable_apa_path_debug()->add_pa_objects();
  slot_object->set_category(planning::common::PaCategory::SLOT);
  slot_object->set_object_id("slot_1");
  planning::common::TrajectoryPoint* point_0 = slot_object->add_points();
  point_0->set_x(slot.pt_0.x());
  point_0->set_y(slot.pt_0.y());

  planning::common::TrajectoryPoint* point_1 = slot_object->add_points();
  point_1->set_x(slot.pt_1.x());
  point_1->set_y(slot.pt_1.y());

  planning::common::TrajectoryPoint* point_2 = slot_object->add_points();
  point_2->set_x(slot.pt_2.x());
  point_2->set_y(slot.pt_2.y());

  planning::common::TrajectoryPoint* point_3 = slot_object->add_points();
  point_3->set_x(slot.pt_3.x());
  point_3->set_y(slot.pt_3.y());

  // add obs points for fitting line
  planning::common::PaObject* obs_object =
      debug->mutable_apa_path_debug()->add_pa_objects();
  obs_object->set_category(planning::common::PaCategory::OBSTACLE);
  obs_object->set_object_id("obstacle_1");

  for (const auto& obs_point : pa_curb_obs) {
    planning::common::TrajectoryPoint* point = obs_object->add_points();
    point->set_x(obs_point.x());
    point->set_y(obs_point.y());
  }

  return;
}

const bool ParallelParkInScenario::CheckEgoToSlotRelation() {
  EgoInfoUnderSlot& ego_info_under_slot =
      apa_world_ptr_->GetSlotManagerPtr()->GetMutableEgoInfoUnderSlot();

  auto& select_slot_global =
      ego_info_under_slot.slot.origin_corner_coord_global_;

  const auto measures_ptr = apa_world_ptr_->GetMeasureDataManagerPtr();

  // 获取自车方向向量
  Eigen::Vector2d ego_heading_vec = measures_ptr->GetHeadingVec();

  // 计算车位01边的向量
  Eigen::Vector2d slot_edge_01 =
      (select_slot_global.pt_0 - select_slot_global.pt_1).normalized();

  // 计算两个向量之间的夹角(弧度)
  double angle_rad =
      std::acos(std::clamp(ego_heading_vec.dot(slot_edge_01), -1.0, 1.0));
  angle_rad = std::fabs(angle_rad);

  // 将弧度转换为角度
  double angle_deg = angle_rad * kRad2Deg;

  ILOG_INFO << "Angle between ego heading and slot edge 01: " << angle_deg
            << " degrees";

  // 判断夹角是否在指定范围之间
  const double vertical_deg = 90.0;
  const double threshold_deg = 20.0;
  bool is_angle_in_range = (angle_deg > (vertical_deg - threshold_deg) &&
                            angle_deg < (vertical_deg + threshold_deg));
  return is_angle_in_range;
}

const bool ParallelParkInScenario::UpdateEgoSlotInfo() {
  EgoInfoUnderSlot& ego_info_under_slot =
      apa_world_ptr_->GetSlotManagerPtr()->GetMutableEgoInfoUnderSlot();

  auto& select_slot_global =
      ego_info_under_slot.slot.origin_corner_coord_global_;
  ILOG_INFO << " select_slot_global pt 0 = " << select_slot_global.pt_0.x()
            << " " << select_slot_global.pt_0.y();
  ILOG_INFO << " select_slot_global pt 1 = " << select_slot_global.pt_1.x()
            << " " << select_slot_global.pt_1.y();
  ILOG_INFO << " select_slot_global pt 2 = " << select_slot_global.pt_2.x()
            << " " << select_slot_global.pt_2.y();
  ILOG_INFO << " select_slot_global pt 3 = " << select_slot_global.pt_3.x()
            << " " << select_slot_global.pt_3.y();

  if (select_slot_global.pt_0 == select_slot_global.pt_1 ||
      select_slot_global.pt_0 == select_slot_global.pt_2 ||
      select_slot_global.pt_0 == select_slot_global.pt_3 ||
      select_slot_global.pt_1 == select_slot_global.pt_2 ||
      select_slot_global.pt_1 == select_slot_global.pt_3 ||
      select_slot_global.pt_2 == select_slot_global.pt_3) {
    ILOG_ERROR << "slot corner points exist same pt!";
    return false;
  }

  const auto measures_ptr = apa_world_ptr_->GetMeasureDataManagerPtr();

  Eigen::Vector2d v_10 =
      (select_slot_global.pt_0 - select_slot_global.pt_1).normalized();

  if (v_10.dot(measures_ptr->GetHeadingVec()) < 1e-9) {
    v_10 *= -1.0;
    select_slot_global.pt_0.swap(select_slot_global.pt_1);
    select_slot_global.pt_2.swap(select_slot_global.pt_3);
  }
  select_slot_global.CalExtraCoord();

  if (apa_world_ptr_->GetStateMachineManagerPtr()->IsSeachingStatus()) {
    if (CheckEgoToSlotRelation()) {
      ILOG_ERROR << "too big angle from ego to slot 01 edge!";
      return false;
    }
  }

  const double heading_10 = std::atan2(v_10.y(), v_10.x());

  const pnc::geometry_lib::LineSegment line_01(
      select_slot_global.pt_1, select_slot_global.pt_0, heading_10);

  const double dist_01_2 =
      pnc::geometry_lib::CalPoint2LineDist(select_slot_global.pt_2, line_01);

  const double dist_01_3 =
      pnc::geometry_lib::CalPoint2LineDist(select_slot_global.pt_3, line_01);

  ego_info_under_slot.slot.slot_width_ = std::min(dist_01_2, dist_01_3);
  ego_info_under_slot.slot.slot_length_ =
      (select_slot_global.pt_0 - select_slot_global.pt_1).norm();

  ILOG_INFO << "slot_length_ = " << ego_info_under_slot.slot.slot_length_;
  ILOG_INFO << "slot_width = " << ego_info_under_slot.slot.slot_width_;
  if (ego_info_under_slot.slot.slot_length_ <
      ego_info_under_slot.slot.slot_width_) {
    ILOG_ERROR << " slot_length_ > slot_width_ in parallel slot";
    return false;
  }

  // calc slot side once at first
  if (frame_.is_replan_first) {
    const auto v_23mid_to_01 =
        ego_info_under_slot.slot.origin_corner_coord_global_.pt_23mid_01mid_vec
            .normalized();

    const double cross_ego_to_slot_heading =
        pnc::geometry_lib::GetCrossFromTwoVec2d(measures_ptr->GetHeadingVec(),
                                                v_23mid_to_01);
    ILOG_INFO << "v_23mid_to_01 " << v_23mid_to_01.x();

    frame_.current_gear = pnc::geometry_lib::SEG_GEAR_REVERSE;
    if (cross_ego_to_slot_heading > 1e-3) {
      t_lane_.slot_side_sgn = 1.0;
      t_lane_.slot_side = pnc::geometry_lib::SLOT_SIDE_RIGHT;
      ego_info_under_slot.slot_side = pnc::geometry_lib::SLOT_SIDE_RIGHT;

    } else if (cross_ego_to_slot_heading < -1e-3) {
      t_lane_.slot_side_sgn = -1.0;
      t_lane_.slot_side = geometry_lib::SLOT_SIDE_LEFT;
      ego_info_under_slot.slot_side = pnc::geometry_lib::SLOT_SIDE_LEFT;

    } else {
      t_lane_.slot_side = pnc::geometry_lib::SLOT_SIDE_INVALID;
      ego_info_under_slot.slot_side = pnc::geometry_lib::SLOT_SIDE_INVALID;
      ILOG_INFO << "calculate parallel slot side error ";
      return false;
    }
  }
  ILOG_INFO << "slot_side = " << static_cast<int>(t_lane_.slot_side);

  const Eigen::Vector2d n =
      (select_slot_global.pt_0 - select_slot_global.pt_1).normalized();
  // ILOG_INFO << "n = " << n.transpose();
  const Eigen::Vector2d t(-n.y(), n.x());
  // ILOG_INFO << "t = " << t.transpose();

  ego_info_under_slot.origin_pose_global.heading = std::atan2(n.y(), n.x());
  ILOG_INFO << " ego_info_under_slot.origin_pose_global.heading = "
            << ego_info_under_slot.origin_pose_global.heading * kRad2Deg;
  ego_info_under_slot.origin_pose_global.heading_vec = n;

  const double target_y_sgn =
      (ego_info_under_slot.slot_side == pnc::geometry_lib::SLOT_SIDE_LEFT
           ? 1.0
           : -1.0);
  const double target_y = 0.5 * ego_info_under_slot.slot.GetWidth();

  ego_info_under_slot.origin_pose_global.pos =
      select_slot_global.pt_1 + target_y * target_y_sgn * t;

  // ILOG_INFO << "ego_info_under_slot.origin_pose_global.pos = "
  //           << ego_info_under_slot.origin_pose_global.pos.transpose();

  ego_info_under_slot.g2l_tf = pnc::geometry_lib::GlobalToLocalTf(
      ego_info_under_slot.origin_pose_global.pos,
      ego_info_under_slot.origin_pose_global.heading);

  ego_info_under_slot.l2g_tf = pnc::geometry_lib::LocalToGlobalTf(
      ego_info_under_slot.origin_pose_global.pos,
      ego_info_under_slot.origin_pose_global.heading);

  ego_info_under_slot.origin_pose_local.pos << 0.0, 0.0;
  ego_info_under_slot.origin_pose_local.heading = 0.0;
  ego_info_under_slot.origin_pose_local.heading_vec << 1.0, 0.0;
  ego_info_under_slot.slot.TransformCoordFromGlobalToLocal(
      ego_info_under_slot.g2l_tf);

  ego_info_under_slot.cur_pose.pos =
      ego_info_under_slot.g2l_tf.GetPos(measures_ptr->GetPos());
  ego_info_under_slot.cur_pose.heading =
      ego_info_under_slot.g2l_tf.GetHeading(measures_ptr->GetHeading());
  ego_info_under_slot.cur_pose.heading_vec =
      pnc::geometry_lib::GenHeadingVec(ego_info_under_slot.cur_pose.heading);

  // ILOG_INFO << "ego_pos_slot = "
  //           << ego_info_under_slot.cur_pose.pos.transpose();
  ILOG_INFO << "ego_heading_slot (deg)= "
            << ego_info_under_slot.cur_pose.heading * kRad2Deg;

  ILOG_INFO << "slot width =" << ego_info_under_slot.slot.slot_width_;
  ILOG_INFO << "t_lane_.slot_side = " << static_cast<int>(t_lane_.slot_side);
  ILOG_INFO << "frame_.current_arc_steer = "
            << static_cast<int>(frame_.current_arc_steer);

  // firstly locate in middle of slot
  double target_x = 0.5 * (ego_info_under_slot.slot.slot_length_ -
                           apa_param.GetParam().car_length) +
                    apa_param.GetParam().rear_overhanging;

  ILOG_INFO << "target x in middle = " << target_x;

  const auto& limiter = ego_info_under_slot.slot.GetLimiter();
  ILOG_INFO << "limiter.valid = " << limiter.valid;
  // encourage to out again if limiter is valid aftering in running state
  // out_again_path_better_ = false;
  if (apa_world_ptr_->GetStateMachineManagerPtr()->GetStateMachine() ==
          ApaStateMachine::ACTIVE_IN_CAR_FRONT ||
      apa_world_ptr_->GetStateMachineManagerPtr()->GetStateMachine() ==
          ApaStateMachine::ACTIVE_IN_CAR_REAR) {
    if (!last_frame_limiter_valid_ && limiter.valid) {
      out_again_path_better_ = true;
    }
  }
  last_frame_limiter_valid_ = limiter.valid;
  if (limiter.valid) {
    t_lane_.limiter.valid = true;
    // transfer limiter in slot coordination
    t_lane_.limiter.start_pt =
        ego_info_under_slot.g2l_tf.GetPos(limiter.start_pt);

    t_lane_.limiter.end_pt = ego_info_under_slot.g2l_tf.GetPos(limiter.end_pt);

    ILOG_INFO << "limiter start pos = " << t_lane_.limiter.start_pt.x() << " "
              << t_lane_.limiter.start_pt.y();
    ILOG_INFO << "limiter end pos = " << t_lane_.limiter.end_pt.x() << " "
              << t_lane_.limiter.end_pt.y();

    const double max_limiter_x =
        std::max(t_lane_.limiter.start_pt.x(), t_lane_.limiter.end_pt.x());

    const double min_limiter_x =
        std::min(t_lane_.limiter.start_pt.x(), t_lane_.limiter.end_pt.x());

    if (max_limiter_x < 0.5 * ego_info_under_slot.slot.slot_length_) {
      ILOG_INFO << "limiter behind!";
      target_x = std::max(
          target_x,
          max_limiter_x +
              apa_param.GetParam().parallel_ego_ac_x_offset_with_limiter);
    } else {
      ILOG_INFO << "limiter front!";
      target_x = std::min(
          target_x,
          min_limiter_x - apa_param.GetParam().wheel_base -
              apa_param.GetParam().parallel_ego_ac_x_offset_with_limiter);
    }
  }
  ILOG_INFO << "target x consider limiter = " << target_x;
  ego_info_under_slot.target_pose.pos << target_x, 0.0;
  ego_info_under_slot.target_pose.heading = 0.0;
  ego_info_under_slot.target_pose.heading_vec << 1.0, 0.0;


  const auto& front_slot_limiter = ego_info_under_slot.slot.GetFrontLimiter();
  ILOG_INFO << "front_slot_limiter.valid = " << front_slot_limiter.valid;
  if (front_slot_limiter.valid) {
    t_lane_.front_slot_limiter.valid = true;
    // transfer limiter in slot coordination
    t_lane_.front_slot_limiter.start_pt =
        ego_info_under_slot.g2l_tf.GetPos(front_slot_limiter.start_pt);
    if (front_slot_limiter.valid) {
      t_lane_.front_slot_limiter.valid = true;
      // transfer limiter in slot coordination
      t_lane_.front_slot_limiter.start_pt =
          ego_info_under_slot.g2l_tf.GetPos(front_slot_limiter.start_pt);

      t_lane_.front_slot_limiter.end_pt =
          ego_info_under_slot.g2l_tf.GetPos(front_slot_limiter.end_pt);

      ILOG_INFO << "front_slot_limiter start pos = "
                << t_lane_.front_slot_limiter.start_pt.x() << " "
                << t_lane_.front_slot_limiter.start_pt.y();
      ILOG_INFO << "front_slot_limiter end pos = "
                << t_lane_.front_slot_limiter.end_pt.x() << " "
                << t_lane_.front_slot_limiter.end_pt.y();
    }
  }

  // calc terminal error once
  ego_info_under_slot.terminal_err.Set(
      ego_info_under_slot.cur_pose.pos - ego_info_under_slot.target_pose.pos,
      pnc::geometry_lib::NormalizeAngle(
          ego_info_under_slot.cur_pose.heading -
          ego_info_under_slot.target_pose.heading));

  // calc slot occupied ratio
  double slot_occupied_ratio = 0.0;
  if (pnc::mathlib::IsInBound(ego_info_under_slot.terminal_err.pos.x(), -3.0,
                              4.0)) {
    const double y_err_ratio = ego_info_under_slot.terminal_err.pos.y() /
                               (0.5 * ego_info_under_slot.slot.slot_width_);

    if (t_lane_.slot_side == pnc::geometry_lib::SLOT_SIDE_RIGHT) {
      slot_occupied_ratio = pnc::mathlib::Clamp(1 - y_err_ratio, 0.0, 1.0);
    } else if (t_lane_.slot_side == pnc::geometry_lib::SLOT_SIDE_LEFT) {
      slot_occupied_ratio = pnc::mathlib::Clamp(1.0 + y_err_ratio, 0.0, 1.0);
    }
  }
  ego_info_under_slot.slot_occupied_ratio = slot_occupied_ratio;
  ILOG_INFO << "ego_slot_info.slot_occupied_ratio = "
            << ego_info_under_slot.slot_occupied_ratio;
  return true;
}

const bool ParallelParkInScenario::GenTlane() {
  ILOG_INFO << "GenTlane ------------";

  // y
  // ^_______________    left side
  // |               |
  // |->x            |
  // |               |
  //
  // y                  ego car------->>
  // ^
  // |               |
  // |->x            |
  // |_______________|   right side

  EgoInfoUnderSlot& ego_info_under_slot =
      apa_world_ptr_->GetSlotManagerPtr()->GetMutableEgoInfoUnderSlot();

  const double slot_length = ego_info_under_slot.slot.GetLength();
  const double slot_width = ego_info_under_slot.slot.GetWidth();
  const double half_slot_width = 0.5 * slot_width;
  const double quarter_slot_width = 0.5 * half_slot_width;
  const double side_sgn = t_lane_.slot_side_sgn;
  const Eigen::Vector2d slot_center(0.5 * slot_length, 0.0);
  const bool is_cur_pose_in_slot =
      CalcSlotOccupiedRatio(ego_info_under_slot.cur_pose) >= 0.1;
  obs_pt_local_vec_.clear();
  parent_total_count.clear();
  parent_height_count.clear();
  std::unordered_map<size_t, double> parent_min_y_abs;//<parent_id, min_y_abs>

  apa_world_ptr_->GetObstacleManagerPtr()->TransformCoordFromGlobalToLocal(
      ego_info_under_slot.g2l_tf);
  apa_world_ptr_->GetCollisionDetectorPtr()->SetParam(
      CollisionDetector::Paramters(kDeletedObsDistOutSlot, true));


  for (const auto& pair :
       apa_world_ptr_->GetObstacleManagerPtr()->GetObstacles()) {
    if (pair.second.GetObsMovementType() != ApaObsMovementType::STATIC) {
      continue;
    }
    bool is_limiter = false;
    if ( pair.second.GetObsScemanticType() == ApaObsScemanticType::LIMITER) {
      is_limiter = true;
    }
    const auto obs_scement = pair.second.GetObsScemanticType();
    const auto obs_height = pair.second.GetObsHeightType();

    const bool is_rigid = (obs_scement == ApaObsScemanticType::WALL ||
                           obs_scement == ApaObsScemanticType::COLUMN ||
                           obs_scement == ApaObsScemanticType::CAR);

    const auto obs_heightType = pair.second.GetObsHeightType();
    // const auto obs_id = pair.second.GetId();
    const auto obs_parent_id = pair.second.GetParentId();
    const auto obs_attribute_type = pair.second.GetObsAttributeType();

    for (const auto& obs_pt_local : pair.second.GetPtClout2dLocal()) {
      if ((obs_pt_local - slot_center).norm() > 25.0) {
        continue;
      }

      // outof total box range
      if (!pnc::mathlib::IsInBound(
              obs_pt_local.x(), -5.0,
              apa_param.GetParam().parallel_channel_x_mag)) {
        // total_box_x_fail_cnt++;
        continue;
      }

      if (obs_pt_local.y() * side_sgn >
              apa_param.GetParam().parallel_channel_y_mag ||
          obs_pt_local.y() * side_sgn <
              (-0.5 * slot_width - apa_param.GetParam().curb_offset)) {
        // total_box_y_fail_cnt++;
        continue;
      }

      // remote front T-boundary obs
      if (obs_pt_local.x() > slot_length + kFrontDetaXMagWhenFrontVacant &&
          obs_pt_local.y() * side_sgn < -0.3 * side_sgn) {
        // front_box_fail_cnt++;
        continue;
      }

      // remote rear T-boundary obs
      if (obs_pt_local.x() < -5.0 &&
          obs_pt_local.y() * side_sgn < -0.3 * side_sgn) {
        // rear_box_fail_cnt++;
        continue;
      }

      const bool curb_condition =
          pnc::mathlib::IsInBound(obs_pt_local.x(), 0.0,
                                  slot_length ) &&
          (obs_pt_local.y() * side_sgn <=
           -kCurbYMagIdentification);  // kCurbInitialOffset

      if ( curb_condition && obs_attribute_type != ApaObsAttributeType::USS_POINT_CLOUD ) {
         double current_y_abs = std::abs(obs_pt_local.y());
         if (parent_min_y_abs.find(obs_parent_id) == parent_min_y_abs.end() ||
          current_y_abs < parent_min_y_abs[obs_parent_id]) {
        parent_min_y_abs[obs_parent_id] = current_y_abs;
      }

        if (parent_total_count.find(obs_parent_id) == parent_total_count.end()) {
          parent_total_count[obs_parent_id] = 0;
        }

        parent_total_count[obs_parent_id]+=1;
        const bool is_high_obs = (obs_heightType == ApaObsHeightType::HIGH || obs_scement == ApaObsScemanticType::WALL ||
                           obs_scement == ApaObsScemanticType::COLUMN ||
                           obs_scement == ApaObsScemanticType::CAR) ;
        // ILOG_INFO << "is_HIGH_obs = " << is_HIGH_obs;
        // ILOG_INFO<<"obs_heightType = "<< static_cast<int>(obs_heightType);
        // ILOG_INFO<<"obs_scement = "<< int(obs_scement);
        // ILOG_INFO<<"parent_total_count["<<obs_parent_id<<"] = "<<parent_total_count[obs_parent_id];
        if(is_high_obs){

          if(parent_height_count.find(obs_parent_id) == parent_height_count.end()){
            parent_height_count[obs_parent_id] = 0;
          }
          parent_height_count[obs_parent_id]+=1;
          ILOG_INFO<<"parent_height_count["<<obs_parent_id<<"] = "<<parent_height_count[obs_parent_id];
          ILOG_INFO<<"obs_pt_local = "<<obs_pt_local.x()<<" , "<<obs_pt_local.y();
        }
      }
      if (apa_world_ptr_->GetCollisionDetectorPtr()->IsObstacleInCar(
              obs_pt_local, ego_info_under_slot.cur_pose,
              0.0168)) {
        // in_ego_cnt++;
        continue;
      }
      if (is_rigid &&
          mathlib::IsInBound(obs_pt_local.x(), 0.8, slot_length - 0.8) &&
          mathlib::IsInBound(
              obs_pt_local.y(), -0.25 * slot_width * side_sgn,
              -(0.5 * slot_width + apa_param.GetParam().curb_offset) *
                  side_sgn)) {
        t_lane_.is_inside_rigid = true;

      }
      if (is_limiter && is_cur_pose_in_slot) {

        auto obs_pt_local_cp = obs_pt_local;
        ILOG_INFO << "befor limiter obs = " << obs_pt_local_cp.x()  ;
        obs_pt_local_cp.x() = obs_pt_local_cp.x()+ 0.9;
        ILOG_INFO << "limiter obs = " << obs_pt_local_cp.x()  ;
        obs_pt_local_vec_[static_cast<size_t>(obs_scement)].emplace_back(
          std::move(obs_pt_local_cp));
        continue;
      }
      if (obs_height == ApaObsHeightType::HIGH) {
        obs_pt_local_vec_[static_cast<size_t>(ApaObsScemanticType::WALL)]
            .emplace_back(std::move(obs_pt_local));
        continue;
      }
      obs_pt_local_vec_[static_cast<size_t>(obs_scement)].emplace_back(
          std::move(obs_pt_local));

    }

  }
  // 高度点云占比统计
  bool height_condition = false;
  size_t selected_parent_id = 0;
  double min_y_abs = std::numeric_limits<double>::max();
  bool found_valid_obstacle = false;

  // 首先找到点云数量最多的父ID
  size_t max_count_parent_id = 0;
  size_t max_point_count = 0;
  for (const auto& pair : parent_total_count) {
      if (pair.second > max_point_count) {
          max_point_count = pair.second;
          max_count_parent_id = pair.first;
      }
    }
  ILOG_INFO << "Parent ID with max point count: " << max_count_parent_id << ", count: " << max_point_count;

  // 获取点云数量最多的父ID的最小y绝对值
  double max_count_parent_min_y = std::numeric_limits<double>::max();
  auto max_count_y_it = parent_min_y_abs.find(max_count_parent_id);
  if (max_count_y_it != parent_min_y_abs.end()) {
      max_count_parent_min_y = max_count_y_it->second;
  }

  for (const auto& pair : parent_min_y_abs) {
    size_t parent_id = pair.first;
    double y_abs = pair.second;
    // 检查该parent_id是否有统计数据
    auto total_it = parent_total_count.find(parent_id);
    if (total_it == parent_total_count.end() || total_it->second <= 0) {
      continue; // 跳过没有有效点数的障碍物
    }
    ILOG_INFO << "parent_id: " << parent_id << " min_y_abs: " << y_abs;

    // 判断是否与点云数量最多的父ID距离小于10cm
    bool is_near_max_count_parent = false;
    if (max_count_parent_min_y < std::numeric_limits<double>::max()) {
        double distance = std::abs(y_abs - max_count_parent_min_y);
        is_near_max_count_parent = (y_abs < max_count_parent_min_y + kEps)&&(distance < 0.1); // 10cm
        ILOG_INFO << "Distance to max count parent: " << distance << "m, is_near: " << is_near_max_count_parent;
    }

    // 选择条件：y绝对值最小，并且与点云数量最多的父ID距离小于10cm
    bool is_better_candidate = false;
    if (is_near_max_count_parent) {
        // 如果距离小于10cm，优先选择y绝对值更小的
        if (y_abs+ kEps < min_y_abs) {
            is_better_candidate = true;
        }
    } else {
        // 如果距离大于等于10cm，只有当当前候选还没有找到合适的时才考虑
        // if (!found_valid_obstacle && y_abs < min_y_abs) {
        //     is_better_candidate = true;
        // }
        continue;
    }

    if (is_better_candidate) {
        min_y_abs = y_abs;
        selected_parent_id = parent_id;
        found_valid_obstacle = true;
        ILOG_INFO << "Updated selected parent_id: " << selected_parent_id << " with y_abs: " << min_y_abs;
    }
  }

  // 如果没有找到距离 < 10cm 的，则选择点云数量最多的障碍物
  if (!found_valid_obstacle && max_count_parent_min_y < std::numeric_limits<double>::max()) {
      selected_parent_id = max_count_parent_id;
      min_y_abs = max_count_parent_min_y;
      found_valid_obstacle = true;
      ILOG_INFO << "No nearby obstacles found, using max count parent_id: " << selected_parent_id;
  }

  ILOG_INFO << "Final selected parent_id: " << selected_parent_id << ", y_abs: " << min_y_abs;

  if (found_valid_obstacle) {
    auto total_it = parent_total_count.find(selected_parent_id);
    auto height_it = parent_height_count.find(selected_parent_id);

    if (total_it != parent_total_count.end() && total_it->second > 0) {

      int total_count = total_it->second;
      int height_count = (height_it != parent_height_count.end()) ? height_it->second : 0;


      // 计算比例
      double height_ratio = static_cast<double>(height_count) / total_count;

      // 计算条件
      if(height_count>= kWallPtNumThred){
        height_condition = ( height_ratio >= kSmallWallHeightRatioThred);
      }else{
        height_condition  =  ( height_ratio >= kBigWallHeightRatioThred);
      }


      // 日志输出
      ILOG_INFO << "Selected parent_id: " << selected_parent_id
                << " total_count: " << total_count
                << " height_count: " << height_count
                << " height_ratio: " << height_ratio
                << " min_y_abs: " << min_y_abs;

      ILOG_INFO << "height_condition: " << height_condition;

      // // 最终判断
      // if (height_condition) {
      //   t_lane_.is_inside_rigid = true;
      //   ILOG_INFO << "t_lane_.is_inside_rigid = true based on selected parent_id: " << selected_parent_id;
      // }
    }
  }

  // set initial x coordination for front and rear tlane obs
  double front_min_x = slot_length + kFrontDetaXMagWhenFrontVacant;
  double rear_max_x = -kRearDetaXMagWhenFrontOccupiedRearVacant;

  // set initial y coordination for front and rear y tlane obs
  double front_parallel_line_y_limit = side_sgn * quarter_slot_width;
  double rear_parallel_line_y_limit = front_parallel_line_y_limit;

  // channel
  double channel_x_limit = apa_param.GetParam().parallel_channel_x_mag;
  double channel_y_limit =
      side_sgn * apa_param.GetParam().parallel_channel_y_mag;

  // curb
  size_t curb_count = 0;
  double curb_y_limit = -side_sgn * (half_slot_width + kCurbInitialOffset);

  const double pa_x_limit_min = ego_info_under_slot.cur_pose.GetX() -
                                apa_param.GetParam().rear_overhanging - 0.2;
  const double pa_x_limit_max = ego_info_under_slot.cur_pose.GetX() +
                                apa_param.GetParam().front_overhanging +
                                apa_param.GetParam().wheel_base + 0.2;

  for (const auto& obstacle_point_set : obs_pt_local_vec_) {
    for (const auto& obstacle_point_slot : obstacle_point_set.second) {
      bool front_obs_y_condition = false;
      if (obstacle_point_set.first ==
              static_cast<size_t>(ApaObsScemanticType::WALL) ||
          obstacle_point_set.first ==
              static_cast<size_t>(ApaObsScemanticType::COLUMN)) {
          front_obs_y_condition = pnc::mathlib::IsInBound(
              obstacle_point_slot.y(), -1.0 * side_sgn,
              (half_slot_width + kFrontObsLineYMagIdentification) * side_sgn);
      } else {
          front_obs_y_condition = pnc::mathlib::IsInBound(
              obstacle_point_slot.y(), -0.6 * side_sgn,
              (half_slot_width + kFrontObsLineYMagIdentification) * side_sgn);
      }

      bool front_obs_condition =
          pnc::mathlib::IsInBound(
              obstacle_point_slot.x(),
              slot_length - kFrontMaxDetaXMagWhenFrontOccupied,
              slot_length + kFrontDetaXMagWhenFrontVacant) &&
          front_obs_y_condition;

      if (front_obs_condition) {
        front_min_x = std::min(front_min_x, obstacle_point_slot.x());

        // ILOG_INFO<<"front_obs_condition!");
      }

      const bool rear_obs_condition =
          ((pnc::mathlib::IsInBound(obstacle_point_slot.x(),
                                    -kRearDetaXMagWhenFrontOccupiedRearVacant,
                                    kRearMaxDetaXMagWhenRearOccupied)) &&
           (pnc::mathlib::IsInBound(obstacle_point_slot.y(),
                                    -side_sgn * kRearObsLineYMagIdentification,
                                    (half_slot_width + 0.1) * side_sgn)));

      if (rear_obs_condition) {
        rear_max_x = std::max(rear_max_x, obstacle_point_slot.x());
        // ILOG_INFO<<"rear_obs_condition!");
      }
      double curb_lower = 0.5;
      double curb_upper = slot_length - 0.5;
      if(t_lane_.is_inside_rigid == true){
        curb_lower = 0.0;
        curb_upper = slot_length;
      }
      const bool curb_condition =
          pnc::mathlib::IsInBound(obstacle_point_slot.x(), curb_lower,
                                  curb_upper ) &&
          (obstacle_point_slot.y() * side_sgn <=
           -kCurbYMagIdentification);  // kCurbInitialOffset

      if (enable_pa_park_) {
        const bool cat_to_curb_condition =
            pnc::mathlib::IsInBound(obstacle_point_slot.x(), pa_x_limit_min,
                                    pa_x_limit_max) &&
            pnc::mathlib::IsInBound(obstacle_point_slot.y(),
                                    kCurbYMagIdentification,
                                    -half_slot_width * side_sgn);
        if (curb_condition &&
            cat_to_curb_condition) {
          if (side_sgn > 0.0) {
            curb_y_limit = std::max(curb_y_limit, obstacle_point_slot.y());
          } else {
            curb_y_limit = std::min(curb_y_limit, obstacle_point_slot.y());
          }
        }
      } else {
        const bool cat_to_curb_condition =
            pnc::mathlib::IsInBound(obstacle_point_slot.x(), curb_lower,
                                    curb_upper) &&
            pnc::mathlib::IsInBound(obstacle_point_slot.y(),
                                    kCurbYMagIdentification,
                                    -half_slot_width * side_sgn);
        if (curb_condition && !(cat_to_curb_condition &&
            obstacle_point_set.first ==
                static_cast<size_t>(ApaObsScemanticType::CAR))) {
          curb_count++;
          if (side_sgn > 0.0) {
            curb_y_limit = std::max(curb_y_limit, obstacle_point_slot.y());
          } else {
            curb_y_limit = std::min(curb_y_limit, obstacle_point_slot.y());
          }

          // ILOG_INFO << "curb curb_y_limit = " << curb_y_limit;
        }
      }

      const bool front_parallel_line_condition =
          pnc::mathlib::IsInBound(
              obstacle_point_slot.x(), slot_length - 0.2,
              slot_length + kFrontDetaXMagWhenFrontVacant) &&
          pnc::mathlib::IsInBound(obstacle_point_slot.y(), 0.0, 2.5 * side_sgn);

      if (front_parallel_line_condition) {
        front_parallel_line_y_limit =
            side_sgn > 0.0
                ? std::max(front_parallel_line_y_limit, obstacle_point_slot.y())
                : std::min(front_parallel_line_y_limit,
                           obstacle_point_slot.y());
        // ILOG_INFO<<"front_parallel_line_y_limit condition!");
      }

      const bool rear_parallel_line_condition =
          pnc::mathlib::IsInBound(obstacle_point_slot.x(), -2.5, 0.2) &&
          pnc::mathlib::IsInBound(obstacle_point_slot.y() * side_sgn, 0.0, 2.3);

      if (rear_parallel_line_condition) {
        rear_parallel_line_y_limit =
            side_sgn > 0.0
                ? std::max(rear_parallel_line_y_limit, obstacle_point_slot.y())
                : std::min(rear_parallel_line_y_limit, obstacle_point_slot.y());
        // ILOG_INFO<<"rear_parallel_line_y_limit condition!");
      }
    }
  }

  bool front_vacant = false;
  bool rear_vacant = false;

  if (front_min_x >= slot_length + kFrontDetaXMagWhenFrontVacant - kEps) {
    front_vacant = true;
    ILOG_INFO << "front space empty!";
  }

  if (rear_max_x <= -kRearDetaXMagWhenFrontOccupiedRearVacant + kEps) {
    rear_vacant = true;
    ILOG_INFO << "rear space empty!";
  }
  if (front_vacant && rear_vacant) {
    front_min_x = slot_length + kFrontDetaXMagWhenFrontVacant;
    rear_max_x = -kRearDetaXMagWhenBothSidesVacant;

  } else if (front_vacant && !rear_vacant) {
    front_min_x = slot_length + kFrontDetaXMagWhenFrontVacant;
    // protection for rear x due to the low accuracy of rear uss obstacle
    // points
    rear_max_x =
        std::max(rear_max_x, -kRearDetaXMagWhenFrontVacantRearOccupied);

  } else if (!front_vacant && rear_vacant) {
    rear_max_x = -kRearDetaXMagWhenFrontOccupiedRearVacant;
  } else {
  }

  ILOG_INFO << "front_vacant = " << front_vacant;
  ILOG_INFO << "rear_vacant = " << rear_vacant;

  ILOG_INFO << "front_min_x before clamp = " << front_min_x;
  front_min_x = pnc::mathlib::Clamp(
      front_min_x, slot_length - kFrontMaxDetaXMagWhenFrontOccupied,
      slot_length + kFrontDetaXMagWhenFrontVacant);
  ILOG_INFO << "front_min_x after clamp =" << front_min_x;

  const double front_y_limit = front_parallel_line_y_limit;
  ILOG_INFO << "front parallel line y =" << front_y_limit;

  ILOG_INFO << "rear_max_x before clamp = " << rear_max_x;
  rear_max_x =
      pnc::mathlib::Clamp(rear_max_x, -kRearDetaXMagWhenFrontOccupiedRearVacant,
                          kRearMaxDetaXMagWhenRearOccupied);
  if (rear_vacant &&
      ego_info_under_slot.slot_occupied_ratio > kEnterMultiPlanSlotRatio) {
    rear_max_x =
        std::min(rear_max_x, ego_info_under_slot.cur_pose.pos.x() - 1.6);
  }

  ILOG_INFO << "rear_max_x after clamp = " << rear_max_x;

  ILOG_INFO << "rear parallel line y =" << rear_parallel_line_y_limit;

  t_lane_.obs_pt_inside << front_min_x, front_y_limit;
  t_lane_.obs_pt_outside << rear_max_x, rear_parallel_line_y_limit;

  curb_y_limit = pnc::mathlib::Clamp(
      curb_y_limit, -side_sgn * (half_slot_width + kCurbInitialOffset),
      -side_sgn * (half_slot_width - kCurbInitialOffset));

  if (ego_info_under_slot.slot_occupied_ratio < 0.01) {
    curb_y_limit +=
        side_sgn * apa_param.GetParam().curb_offset_when_ego_outside_slot;
  }

  t_lane_.corner_inside_slot << slot_length, half_slot_width * side_sgn;
  t_lane_.corner_outside_slot << 0.0, half_slot_width * side_sgn;

  t_lane_.curb_y = curb_y_limit;
  t_lane_.channel_y = channel_y_limit;
  t_lane_.channel_x_limit = channel_x_limit;

  t_lane_.pt_outside = t_lane_.obs_pt_outside;
  t_lane_.pt_inside = t_lane_.obs_pt_inside;

  t_lane_.slot_length = slot_length;
  t_lane_.slot_width = slot_width;

  // ego target x is already set according to middle of slot and limiter in
  // gentlane, it's time to set target x and y considering obstacles
  double ori_upper_bound = std::min(
      t_lane_.obs_pt_inside.x(),
      t_lane_.slot_length +
          apa_param.GetParam().parallel_max_ego_x_offset_with_invasion);

  ILOG_INFO << "debug for target x ---------------------------";
  ILOG_INFO << "obs_pt_inside x = " << t_lane_.obs_pt_inside.x() << " "
            << t_lane_.obs_pt_inside.y();
  ILOG_INFO << "slot length +  parallel_max_ego_x_offset_with_invasion = "
            << t_lane_.slot_length +
                   apa_param.GetParam().parallel_max_ego_x_offset_with_invasion;
  ILOG_INFO << "upper_bound min of them = " << ori_upper_bound;

  ori_upper_bound -= (apa_param.GetParam().front_overhanging +
                      apa_param.GetParam().wheel_base);
  double upper_bound = ori_upper_bound;
  if (!front_vacant) {
    ILOG_INFO << "FRONT occupied! need to reduce extra buffer";
    upper_bound -= apa_param.GetParam().parallel_terminal_x_offset_with_obs;
  }
  ILOG_INFO << "final upper_bound = " << upper_bound;

  double ori_lower_bound =
      std::max(t_lane_.obs_pt_outside.x(),
               -apa_param.GetParam().parallel_max_ego_x_offset_with_invasion);
  ILOG_INFO << "debug for target y ---------";
  ILOG_INFO << "obs_pt_outside x = " << t_lane_.obs_pt_outside.x() << " "
            << t_lane_.obs_pt_outside.y();
  ILOG_INFO
      << "--apa_param.GetParam().parallel_max_ego_x_offset_with_invasion = "
      << -apa_param.GetParam().parallel_max_ego_x_offset_with_invasion;

  ori_lower_bound += apa_param.GetParam().rear_overhanging;
  double lower_bound = ori_lower_bound;

  if (!rear_vacant) {
    ILOG_INFO << "rear occupied, need extra buffer!";
    lower_bound += apa_param.GetParam().parallel_terminal_x_offset_with_obs;
  }
  ILOG_INFO << "lower_bound max of them = " << lower_bound;
  if (!(apa_world_ptr_->GetStateMachineManagerPtr()->IsSAPAMode())) {
    const size_t need_size = 8;
    if (ego_info_under_slot.slot_occupied_ratio < 0.1) {
      double ref_angle = relative_loc_observer_manager_.CalCameraOberserveAngel(
          apa_world_ptr_->GetMeasureDataManagerPtr(),
          ego_info_under_slot.slot.origin_corner_coord_global_.pt_01_mid);
      // try_bound_map_[ego_info_under_slot.id].emplace_back(ref_angle);
      try_bound_map_[ego_info_under_slot.id].insert(
          AngleResult(ref_angle, (upper_bound > lower_bound), curb_y_limit));
      while (try_bound_map_[ego_info_under_slot.id].size() > need_size) {
        auto it = try_bound_map_[ego_info_under_slot.id].end();
        --it;
        try_bound_map_[ego_info_under_slot.id].erase(it);
      }
      for (const auto& ar : try_bound_map_[ego_info_under_slot.id]) {
        ILOG_INFO << "calc debug ang: " << ar.ang << ", res: " << ar.res
                  << ", curb_y: " << ar.curb_y
                  << " id: " << ego_info_under_slot.id;
      }

      // ILOG_INFO<<"height_condition for insert: "<<height_condition;
      auto& height_obs_vec = multi_frame_height_obs_map_[ego_info_under_slot.id];
       height_obs_vec.insert(height_obs_vec.begin(),
                         AngleResultHeightObs(ref_angle, height_condition));
       //
      // ILOG_INFO << "Inserted new entry - angle: " << ref_angle
      //           << ", height_condition: " << height_condition;
      // ILOG_INFO << "After insertion, container size: " << height_obs_vec.size();
      if (height_obs_vec.size() > need_size) {
        const auto& oldest_data = height_obs_vec.back();
        // ILOG_INFO << "Removing oldest entry - angle: " << oldest_data.ang
        //           << ", height_condition: " << oldest_data.res;
        height_obs_vec.pop_back();
      }

      // ILOG_INFO << "Current height observations (newest first):";
      // for (size_t i = 0; i < height_obs_vec.size(); ++i) {
      //     const auto& ar = height_obs_vec[i];
      //     ILOG_INFO << "  [" << i << "] angle: " << ar.ang
      //               << ", height_condition: " << ar.res;
      // }
    }
    int count_valid = -1;
    int count_height_cond_valid = -1;
    if (apa_world_ptr_->GetStateMachineManagerPtr()->IsParkingStatus()) {
      if (try_bound_map_.find(ego_info_under_slot.id) != try_bound_map_.end()) {
        if (try_bound_map_[ego_info_under_slot.id].size() >= 4) {
          count_valid = 0;
          for (const auto& it : try_bound_map_[ego_info_under_slot.id]) {
            double ang = it.ang;
            bool res = it.res;
            ILOG_INFO << "park debug ang: " << ang << ", res: " << res
                      << ", curb_y: " << it.curb_y
                      << " id: " << ego_info_under_slot.id;
            count_valid += res ? 1 : 0;
          }
        }
        if (ego_info_under_slot.slot_occupied_ratio < 0.1) {
          curb_y_limit = try_bound_map_[ego_info_under_slot.id].begin()->curb_y;
          t_lane_.curb_y = curb_y_limit;
        }
      }

      if(multi_frame_height_obs_map_.find(ego_info_under_slot.id) != multi_frame_height_obs_map_.end()){
        const auto& height_obs_vec = multi_frame_height_obs_map_[ego_info_under_slot.id];
        if (height_obs_vec.size() >= 1) {
            count_height_cond_valid = 0;
            for (const auto& it : height_obs_vec) {
                double ang = it.ang;
                bool res = it.res;
                // ILOG_INFO << "park debug height condition: " << ang
                //           << ", height condition: " << res
                //           << ", slot id: " << ego_info_under_slot.id;
                count_height_cond_valid += res ? 1 : 0;
            }
        }
      }
    }
    if (count_height_cond_valid >= 1) {
      t_lane_.is_inside_rigid = true;
    }
    if (lower_bound > upper_bound) {
      const double small_obs_buffer = 0.2;
      lower_bound = ori_lower_bound + (rear_vacant ? 0.0 : small_obs_buffer);
      upper_bound = ori_upper_bound - (front_vacant ? 0.0 : small_obs_buffer);
      ILOG_INFO << "new upper bound = " << upper_bound;
      ILOG_INFO << "new lowwer bound = " << lower_bound;

      if (lower_bound > upper_bound) {
        if (count_valid >= 0 && count_valid > (need_size / 2)) {
          ILOG_INFO << "based on the results of multiple frames, go!";
        } else {
          ILOG_ERROR << "lower_bound > upper_bound, too much failed!";
          return false;
        }
      }
      ego_info_under_slot.target_pose.pos.x() = pnc::mathlib::Clamp(
          ego_info_under_slot.target_pose.pos.x(), lower_bound, upper_bound);

    } else {
      ego_info_under_slot.target_pose.pos.x() = pnc::mathlib::Clamp(
          ego_info_under_slot.target_pose.pos.x(), lower_bound, upper_bound);
    }
    ILOG_INFO << "ego_info_under_slot.target_pose.pos.x() before = "
              << ego_info_under_slot.target_pose.pos.x() << " "
              << ego_info_under_slot.target_pose.pos.y();
    ILOG_INFO << "bound = [ " << lower_bound << ", " << upper_bound << " ]";
  } else {
    ego_info_under_slot.target_pose.pos.x() =
        0.5 * (ego_info_under_slot.slot.slot_length_ -
               apa_param.GetParam().car_length) +
        apa_param.GetParam().rear_overhanging;
  }

  ILOG_INFO << "t_lane_.is_inside_rigid = " << t_lane_.is_inside_rigid;
  // set target y with curb
  const double y_offset_with_obs_type =
      t_lane_.is_inside_rigid
          ? apa_param.GetParam().terminal_parallel_y_offset_with_wall
          : apa_param.GetParam().terminal_parallel_y_offset_with_curb;

  const double target_y_with_curb =
      curb_y_limit + side_sgn * (y_offset_with_obs_type +
                                 0.5 * apa_param.GetParam().car_width);
  ILOG_INFO << "curb_y_limit = " << curb_y_limit
            << " target_y_with_curb = " << target_y_with_curb;

  if (enable_pa_park_ ||
      apa_world_ptr_->GetStateMachineManagerPtr()->IsSAPAMode()) {
    ego_info_under_slot.target_pose.pos.y() = 0.0;
  } else {
    const bool is_width_curb =
        -side_sgn * curb_y_limit > (half_slot_width + kWidthCurbOffset) ? true
                                                                        : false;
    const bool is_width_slot = slot_width > kWidthSlot ? true : false;
    const double terminal_parallel_y_offset =
        (is_width_curb || is_width_slot)
            ? 0.0
            : apa_param.GetParam().terminal_parallel_y_offset;
    ego_info_under_slot.target_pose.pos.y() =
        (side_sgn > 0.0
             ? std::max(-terminal_parallel_y_offset, target_y_with_curb)
             : std::min(terminal_parallel_y_offset, target_y_with_curb));
  }

  ILOG_INFO << "ego pose = " << ego_info_under_slot.cur_pose.pos.x() << " "
            << ego_info_under_slot.cur_pose.pos.y();

  ILOG_INFO << "Final terminal pose = "
            << ego_info_under_slot.target_pose.pos.x() << " "
            << ego_info_under_slot.target_pose.pos.y();
  t_lane_.pt_terminal_pos = ego_info_under_slot.target_pose.pos;

  ego_info_under_slot.terminal_err.Set(
      ego_info_under_slot.cur_pose.pos - ego_info_under_slot.target_pose.pos,
      pnc::geometry_lib::NormalizeAngle(
          ego_info_under_slot.cur_pose.heading -
          ego_info_under_slot.target_pose.heading));

  ILOG_INFO << "-- t_lane --------";
  if (pnc::mathlib::IsDoubleEqual(side_sgn, 1.0)) {
    ILOG_INFO << "right slot";
  } else if (pnc::mathlib::IsDoubleEqual(side_sgn, -1.0)) {
    ILOG_INFO << "left slot!";
  }
  ILOG_INFO << "obs_pt_inside = " << t_lane_.obs_pt_inside.x() << " "
            << t_lane_.obs_pt_inside.y();
  ILOG_INFO << "obs_pt_outside = " << t_lane_.obs_pt_outside.x() << " "
            << t_lane_.obs_pt_outside.y();
  ILOG_INFO << "corner_inside_slot = " << t_lane_.corner_inside_slot.x() << " "
            << t_lane_.corner_inside_slot.y();
  ILOG_INFO << "corner_outside_slot = " << t_lane_.corner_outside_slot.x()
            << " " << t_lane_.corner_outside_slot.y();

  ILOG_INFO << "pt_outside = " << t_lane_.pt_outside.x() << " "
            << t_lane_.pt_outside.y();
  ILOG_INFO << "pt_inside = " << t_lane_.pt_inside.x() << " "
            << t_lane_.pt_inside.y();
  ILOG_INFO << "pt_terminal_pos = " << t_lane_.pt_terminal_pos.x() << " "
            << t_lane_.pt_terminal_pos.y();
  ILOG_INFO << "slot length =" << t_lane_.slot_length;
  ILOG_INFO << "half slot width =" << 0.5 * t_lane_.slot_width;
  ILOG_INFO << "curb y =" << t_lane_.curb_y;
  ILOG_INFO << "channel x =" << t_lane_.channel_x_limit;
  ILOG_INFO << "channel y =" << t_lane_.channel_y;
  ILOG_INFO << "------------------";

  JSON_DEBUG_VALUE("para_tlane_obs_in_x", t_lane_.obs_pt_inside.x())
  JSON_DEBUG_VALUE("para_tlane_obs_in_y", t_lane_.obs_pt_inside.y())
  JSON_DEBUG_VALUE("para_tlane_obs_out_x", t_lane_.obs_pt_outside.x())
  JSON_DEBUG_VALUE("para_tlane_obs_out_x", t_lane_.obs_pt_outside.y())

  return true;
}

const bool ParallelParkInScenario::GenObstacles() { return true; }

void ParallelParkInScenario::GenTBoundaryObstacles() {
  //                                                  |
  //                         c-------------D
  //                         |             |  pin
  //            A -----------B pout        E---------F
  //                         |       ego-------->     |
  //  channel pt1 ----------------------------------- channel pt2
  //                                                  |
  //                         |       ego-------->     |
  //            A -----------B pout        E---------F
  //                         |             |  pin
  //                         c-------------D

  apa_world_ptr_->GetCollisionDetectorPtr()->Reset();
  const EgoInfoUnderSlot& ego_info_under_slot =
      apa_world_ptr_->GetSlotManagerPtr()->GetEgoInfoUnderSlot();

  const double slot_side_sgn = t_lane_.slot_side_sgn;

  // set T-Boundary obstacles
  const Eigen::Vector2d B(t_lane_.obs_pt_outside.x() - kTBoundaryXdiff,
                          0.3 * slot_side_sgn);

  const Eigen::Vector2d A(B.x() - 3.2, B.y());

  const Eigen::Vector2d E(t_lane_.obs_pt_inside.x() + kTBoundaryXdiff,
                          t_lane_.obs_pt_inside.y() - slot_side_sgn * 0.8);

  const Eigen::Vector2d C(
      B.x(), (-0.5 * t_lane_.slot_width - apa_param.GetParam().curb_offset) *
                 slot_side_sgn);

  const Eigen::Vector2d D(E.x(), C.y());

  const Eigen::Vector2d F(t_lane_.channel_x_limit, E.y());

  const Eigen::Vector2d channel_point_1(
      A.x(), apa_param.GetParam().parallel_channel_y_mag * slot_side_sgn);

  const Eigen::Vector2d channel_point_2(
      F.x(), apa_param.GetParam().parallel_channel_y_mag * slot_side_sgn);

  const pnc::geometry_lib::LineSegment channel_line(channel_point_1,
                                                    channel_point_2);

  t_lane_.tlane_corner =
      TlaneCorner(A, B, C, D, E, F, channel_point_1, channel_point_2);

  // sample channel boundary line
  std::vector<Eigen::Vector2d> channel_line_obs_vec;
  std::vector<Eigen::Vector2d> filtered_channel_obs_vec;
  pnc::geometry_lib::SamplePointSetInLineSeg(channel_line_obs_vec, channel_line,
                                             kChannelSampleDist);

  for (const auto& obs : channel_line_obs_vec) {
    if (!apa_world_ptr_->GetCollisionDetectorPtr()->IsObstacleInCar(
            obs, ego_info_under_slot.cur_pose, kDeletedObsDistOutSlot)) {
      filtered_channel_obs_vec.emplace_back(obs);
    }
  }

  const double slot_side_y =
      slot_side_sgn > 0.0
          ? std::max(t_lane_.obs_pt_outside.y(), t_lane_.obs_pt_inside.y())
          : std::min(t_lane_.obs_pt_outside.y(), t_lane_.obs_pt_inside.y());
  ILOG_INFO << "1 channel_y =" << t_lane_.channel_y;
  for (const auto& obstacle_point_set : obs_pt_local_vec_) {
    for (const auto& obstacle_point_slot : obstacle_point_set.second) {
      // add obs near channel
      const bool channel_x_condition =
          pnc::mathlib::IsInBound(obstacle_point_slot.x(),
                                  t_lane_.slot_length + kFrontShortChannelMin,
                                  E.x() + kFrontShortChannelMax) &&
          pnc::mathlib::IsInBound(
              obstacle_point_slot.y(), E.y(),
              kMinChannelYMagIdentification * slot_side_sgn);

      const bool channel_y_condition =
          pnc::mathlib::IsInBound(obstacle_point_slot.x(), channel_point_1.x(),
                                  channel_point_2.x()) &&
          pnc::mathlib::IsInBound(obstacle_point_slot.y(),
                                  kMinChannelYMagIdentification * slot_side_sgn,
                                  channel_point_1.y());
      // add channel obs beside slot
      const bool channel_slot_condition =
          pnc::mathlib::IsInBound(obstacle_point_slot.x(), B.x(), E.x()) &&
          pnc::mathlib::IsInBound(
              obstacle_point_slot.y(), slot_side_y,
              kMinChannelYMagIdentification * slot_side_sgn);

      if (channel_y_condition || channel_x_condition ||
          channel_slot_condition) {
        filtered_channel_obs_vec.emplace_back(obstacle_point_slot);
      }
      if (channel_x_condition) {
        t_lane_.is_short_channel = true;
        continue;
      }
      if (channel_y_condition) {
        if (pnc::mathlib::IsInBound(obstacle_point_slot.x(),
                                    t_lane_.slot_length,
                                    t_lane_.slot_length + 5.0)) {
          if (slot_side_sgn > 0.0) {
            t_lane_.channel_y =
                std::min(obstacle_point_slot.y(), t_lane_.channel_y);
          } else {
            t_lane_.channel_y =
                std::max(obstacle_point_slot.y(), t_lane_.channel_y);
          }
        }
      }
    }
  }
  ILOG_INFO << "is_short_channel =" << t_lane_.is_short_channel;
  ILOG_INFO << "2 channel_y =" << t_lane_.channel_y;
  apa_world_ptr_->GetCollisionDetectorPtr()->SetObstacles(
      filtered_channel_obs_vec, CollisionDetector::CHANNEL_OBS);

  // set tlane obs
  pnc::geometry_lib::LineSegment tlane_line;
  std::vector<pnc::geometry_lib::LineSegment> tlane_line_vec;
  // set tlane parallel line obs
  tlane_line.SetPoints(A, B);
  tlane_line_vec.emplace_back(tlane_line);

  tlane_line.SetPoints(E, F);
  tlane_line_vec.emplace_back(tlane_line);

  std::vector<Eigen::Vector2d> point_set;
  std::vector<Eigen::Vector2d> tlane_obstacle_vec;

  for (const auto& line : tlane_line_vec) {
    pnc::geometry_lib::SamplePointSetInLineSeg(point_set, line,
                                               kTBoundarySampleDist);

    for (const auto& obs : point_set) {
      if (!apa_world_ptr_->GetCollisionDetectorPtr()->IsObstacleInCar(
              obs, ego_info_under_slot.cur_pose, kDeletedObsDistOutSlot)) {
        tlane_obstacle_vec.emplace_back(obs);
      }
    }
  }

  // tlane vertical line
  for (const auto& obstacle_point_set : obs_pt_local_vec_) {
    for (const auto& obstacle_point_slot : obstacle_point_set.second) {
      const bool is_rear_tlane_line =
          pnc::mathlib::IsInBound(obstacle_point_slot.x(), A.x(),
                                  t_lane_.obs_pt_outside.x()) &&
          pnc::mathlib::IsInBound(
              obstacle_point_slot.y(), B.y(),
              kMinChannelYMagIdentification * slot_side_sgn);

      if (is_rear_tlane_line) {
        tlane_obstacle_vec.emplace_back(obstacle_point_slot);
        continue;
      }

      const bool is_front_tlane_line =
          pnc::mathlib::IsInBound(obstacle_point_slot.x(),
                                  t_lane_.obs_pt_inside.x() - 0.3, F.x()) &&
          pnc::mathlib::IsInBound(
              obstacle_point_slot.y(), E.y(),
              kMinChannelYMagIdentification * slot_side_sgn);

      if (is_front_tlane_line) {
        tlane_obstacle_vec.emplace_back(obstacle_point_slot);
      }
    }
  }

  // tlane
  tlane_line_vec.clear();

  const Eigen::Vector2d C_curb(C.x(), t_lane_.curb_y);
  const Eigen::Vector2d D_curb(D.x(), t_lane_.curb_y);

  tlane_line.SetPoints(B, C_curb);
  tlane_line_vec.emplace_back(tlane_line);

  tlane_line.SetPoints(D_curb, E);
  tlane_line_vec.emplace_back(tlane_line);

  for (const auto& line : tlane_line_vec) {
    pnc::geometry_lib::SamplePointSetInLineSeg(point_set, line,
                                               kTBoundarySampleDist);

    for (const auto& obs : point_set) {
      if (!apa_world_ptr_->GetCollisionDetectorPtr()->IsObstacleInCar(
              obs, ego_info_under_slot.cur_pose, kDeletedObsDistInSlot)) {
        tlane_obstacle_vec.emplace_back(obs);
      }
    }
  }

  std::vector<Eigen::Vector2d> in_tlane_obstacle_vec;
  for (const auto& obstacle_point_set : obs_pt_local_vec_) {
    for (const auto& obs_pos : obstacle_point_set.second) {
      const bool is_tlane_obs =
          pnc::mathlib::IsInBound(obs_pos.x(), B.x(), E.x()) &&
          pnc::mathlib::IsInBound(obs_pos.y(), t_lane_.obs_pt_inside.y(),
                                  C_curb.y());
      if (!is_tlane_obs) {
        continue;
      }

      const bool is_front2rear_obs =
          pnc::mathlib::IsInBound(obs_pos.x(), t_lane_.obs_pt_inside.x() - kEps,
                                  t_lane_.obs_pt_outside.x() + kEps) &&
          pnc::mathlib::IsInBound(obs_pos.y(), t_lane_.obs_pt_inside.y() - kEps,
                                  C_curb.y() + kEps);

      if (is_front2rear_obs) {
        in_tlane_obstacle_vec.emplace_back(obs_pos);
      }

      if (pnc::mathlib::IsInBound(obs_pos.x(), 0.5,
                                  t_lane_.slot_length - 0.5) &&
          pnc::mathlib::IsInBound(obs_pos.y(), -0.4 * slot_side_sgn,
                                  1.2 * slot_side_sgn)) {
        // obs noise in slot
        continue;
      }

      const bool is_front_tlane_obs =
          pnc::mathlib::IsInBound(obs_pos.x(), t_lane_.obs_pt_inside.x(),
                                  E.x()) &&
          pnc::mathlib::IsInBound(obs_pos.y(), t_lane_.obs_pt_inside.y(),
                                  D_curb.y());
      const bool is_rear_tlane_obs =
          pnc::mathlib::IsInBound(obs_pos.x(), B.x(),
                                  t_lane_.obs_pt_outside.x()) &&
          pnc::mathlib::IsInBound(obs_pos.y(), 1.5 * slot_side_sgn, C_curb.y());

      if (is_front_tlane_obs || is_rear_tlane_obs) {
        tlane_obstacle_vec.emplace_back(obs_pos);
      }
    }
  }
  apa_world_ptr_->GetCollisionDetectorPtr()->SetObstacles(
      tlane_obstacle_vec, CollisionDetector::TLANE_BOUNDARY_OBS);
  apa_world_ptr_->GetCollisionDetectorPtr()->SetObstacles(
      in_tlane_obstacle_vec, CollisionDetector::CURB_OBS);

  point_set.clear();
  tlane_obstacle_vec.clear();
  tlane_line.SetPoints(C_curb, D_curb);
  pnc::geometry_lib::SamplePointSetInLineSeg(point_set, tlane_line,
                                             kTBoundarySampleDist);

  for (const auto& obs : point_set) {
    if (!apa_world_ptr_->GetCollisionDetectorPtr()->IsObstacleInCar(
            obs, ego_info_under_slot.cur_pose, kDeletedObsDistInSlot)) {
      tlane_obstacle_vec.emplace_back(obs);
    }
  }

  apa_world_ptr_->GetCollisionDetectorPtr()->SetObstacles(
      tlane_obstacle_vec, CollisionDetector::CURB_OBS);

  if (t_lane_.limiter.valid) {
    double ori_limiter_obs_x = 0.0;
    double limiter_obs_x = 0.0;
    if (t_lane_.limiter.start_pt.x() < 0.5 * t_lane_.slot_length) {
      ILOG_INFO << "rear limiter";
      ori_limiter_obs_x =
          std::max(t_lane_.limiter.start_pt.x(), t_lane_.limiter.end_pt.x());
      limiter_obs_x =
          ori_limiter_obs_x +
          (apa_param.GetParam().parallel_ego_ac_x_offset_with_limiter -
           apa_param.GetParam().rear_overhanging - 0.12);
      ILOG_INFO << "limiter_obs_x = " << limiter_obs_x;
    } else {
      ILOG_INFO << "front limiter";
      ori_limiter_obs_x =
          std::min(t_lane_.limiter.start_pt.x(), t_lane_.limiter.end_pt.x());
      limiter_obs_x =
          ori_limiter_obs_x +
          (-apa_param.GetParam().parallel_ego_ac_x_offset_with_limiter -
           apa_param.GetParam().front_overhanging + 0.12);
    }

    const Eigen::Vector2d limiter_obs_start(limiter_obs_x,
                                            t_lane_.limiter.start_pt.y());
    const Eigen::Vector2d limiter_obs_end(limiter_obs_x,
                                          t_lane_.limiter.end_pt.y());

    const pnc::geometry_lib::LineSegment limiter_line(limiter_obs_start,
                                                      limiter_obs_end);
    point_set.clear();
    tlane_obstacle_vec.clear();
    pnc::geometry_lib::SamplePointSetInLineSeg(point_set, limiter_line,
                                               0.1);
    for (const auto& obs : point_set) {
      if (!apa_world_ptr_->GetCollisionDetectorPtr()->IsObstacleInCar(
              obs, ego_info_under_slot.cur_pose, kDeletedObsDistInSlot)) {
        tlane_obstacle_vec.emplace_back(obs);
      }
    }
    apa_world_ptr_->GetCollisionDetectorPtr()->SetObstacles(
        tlane_obstacle_vec, CollisionDetector::LIMITER_OBS);
    // front virtual limit obs
    // tlane_obstacle_vec.clear();
    // const double parallel_virtual_limit_x_offset =
    //     apa_param.GetParam().parallel_virtual_limit_x_offset;
    // if (parallel_virtual_limit_x_offset > 0.0) {
    //   for (const auto& obs : point_set) {
    //     Eigen::Vector2d virtual_front_limiter_obs = Eigen::Vector2d(
    //         ori_limiter_obs_x + parallel_virtual_limit_x_offset, obs.y());
    //     tlane_obstacle_vec.emplace_back(std::move(virtual_front_limiter_obs));
    //   }
    // }
    // apa_world_ptr_->GetCollisionDetectorPtr()->SetObstacles(
    //     tlane_obstacle_vec, CollisionDetector::SIDE_CROSS_LIMITER_OBS);

    const Eigen::Vector2d side_cross_limiter_obs_start(ori_limiter_obs_x,
                                            t_lane_.limiter.start_pt.y());
    const Eigen::Vector2d side_cross_limiter_obs_end(ori_limiter_obs_x,
                                          t_lane_.limiter.end_pt.y());

    const pnc::geometry_lib::LineSegment side_cross_limiter_line(side_cross_limiter_obs_start,
                                                      side_cross_limiter_obs_end);

    point_set.clear();
    tlane_obstacle_vec.clear();
    pnc::geometry_lib::SamplePointSetInLineSeg(point_set, side_cross_limiter_line,
                                               0.1);
    for (const auto& obs : point_set) {
      if (!apa_world_ptr_->GetCollisionDetectorPtr()->IsObstacleInCar(
              obs, ego_info_under_slot.cur_pose, kDeletedObsDistInSlot)) {
        tlane_obstacle_vec.emplace_back(obs);
      }
    }

    apa_world_ptr_->GetCollisionDetectorPtr()->SetObstacles(
        tlane_obstacle_vec, CollisionDetector::SIDE_CROSS_LIMITER_OBS);
  }



  if (t_lane_.front_slot_limiter.valid) {
    double limiter_obs_x = 0.0;

    limiter_obs_x =
          std::max(t_lane_.front_slot_limiter.start_pt.x(), t_lane_.front_slot_limiter.end_pt.x());

    const Eigen::Vector2d limiter_obs_start(limiter_obs_x,
                                            t_lane_.front_slot_limiter.start_pt.y());
    const Eigen::Vector2d limiter_obs_end(limiter_obs_x,
                                          t_lane_.front_slot_limiter.end_pt.y());

    const pnc::geometry_lib::LineSegment limiter_line(limiter_obs_start,
                                                      limiter_obs_end);
    point_set.clear();
    tlane_obstacle_vec.clear();
    pnc::geometry_lib::SamplePointSetInLineSeg(point_set, limiter_line,
                                               0.1);
    for (const auto& obs : point_set) {
      if (!apa_world_ptr_->GetCollisionDetectorPtr()->IsObstacleInCar(
              obs, ego_info_under_slot.cur_pose, kDeletedObsDistInSlot)) {
        tlane_obstacle_vec.emplace_back(obs);
      }
    }
    apa_world_ptr_->GetCollisionDetectorPtr()->SetObstacles(
        tlane_obstacle_vec, CollisionDetector::SIDE_CROSS_LIMITER_OBS);


  }
}

const bool ParallelParkInScenario::CheckLastPathCollided() {
  const EgoInfoUnderSlot& ego_info =
      apa_world_ptr_->GetSlotManagerPtr()->GetEgoInfoUnderSlot();

  std::vector<pnc::geometry_lib::PathSegment> need_check_path;
  const auto& pre_output = previous_parallel_path_planner_.GetOutput();
  const auto& pre_ego_info =
      previous_parallel_path_planner_.GetInput().ego_info_under_slot;
  for (size_t i = pre_output.path_seg_index.first;
       i <= pre_output.path_seg_index.second; i++) {
    pnc::geometry_lib::PathSegment cur_path = pre_output.path_segment_vec[i];
    cur_path.LocalToGlobal(pre_ego_info.l2g_tf);
    cur_path.GlobalToLocal(ego_info.g2l_tf);
    need_check_path.emplace_back(cur_path);
  }
  if (parallel_path_planner_.CheckPreviousPathSegVecCollided(need_check_path,
                                                             -0.3)) {
    ILOG_INFO << "last path is collided";
    return true;
  }
  return false;
}

// To use or not to use, that is the question.
const ParallelPathGenerator& ParallelParkInScenario::UseOrNotUseLastPath() {
  ILOG_INFO << "Enter  UseOrNotUseLastPath";
  if (frame_.is_replan_first) {
    ILOG_INFO << "first replan, not using last path";
    frame_.pathplan_result = PathPlannerResult::PLAN_UPDATE;
    return parallel_path_planner_;
  }
  const EgoInfoUnderSlot& ego_info_under_slot =
      apa_world_ptr_->GetSlotManagerPtr()->GetEgoInfoUnderSlot();

  const pnc::geometry_lib::PathPoint& cur_pose = ego_info_under_slot.cur_pose;

  pnc::geometry_lib::PathPlanSource pre_path_source =
      previous_parallel_path_planner_.GetOutput()
          .path_segment_vec.front()
          .path_source;
  pnc::geometry_lib::PathPlanSource cur_path_source =
      parallel_path_planner_.GetOutput().path_segment_vec.front().path_source;

  pnc::geometry_lib::PathPoint end_pt =
      previous_parallel_path_planner_.GetOutput()
          .path_segment_vec[previous_parallel_path_planner_.GetOutput()
                                .path_seg_index.second]
          .GetEndPose();

  pnc::geometry_lib::PathPoint global_end_pt(
      previous_parallel_path_planner_.GetInput()
          .ego_info_under_slot.l2g_tf.GetPos(end_pt.pos),
      previous_parallel_path_planner_.GetInput()
          .ego_info_under_slot.l2g_tf.GetHeading(end_pt.heading));

  // need to set is_first_path to true
  previous_parallel_path_planner_.GetOutputPtr()->is_first_path = true;

  previous_parallel_path_planner_.DeleteFirstSegPath();

  // insert line ego -> path start pt
  pnc::geometry_lib::PathPoint line_pa_pose = cur_pose;
  line_pa_pose.LocalToGlobal(ego_info_under_slot.l2g_tf);
  line_pa_pose.GlobalToLocal(
      previous_parallel_path_planner_.GetInput().ego_info_under_slot.g2l_tf);
  previous_parallel_path_planner_.InsertLineSegToEgo2Path(line_pa_pose);

  const bool res = previous_parallel_path_planner_.SetCurrentPathSegIndex();
  if (!res) {
    ILOG_INFO << "set current path seg index failed, use replan path";
    frame_.pathplan_result = PathPlannerResult::PLAN_UPDATE;
    return parallel_path_planner_;
  }

  ILOG_INFO << "first seg idx = "
            << previous_parallel_path_planner_.GetOutput().path_seg_index.first;
  ILOG_INFO
      << "last seg idx = "
      << previous_parallel_path_planner_.GetOutput().path_seg_index.second;

  previous_parallel_path_planner_.SampleCurrentPathSeg();
  previous_parallel_path_planner_.JudgeNeedOptimize();

  pnc::geometry_lib::PathPlanSource pre_next_path_source =
      previous_parallel_path_planner_.GetOutput()
          .path_segment_vec.front()
          .path_source;

  bool path_source_condition = false;
  if (pre_path_source ==
      pnc::geometry_lib::PathPlanSource::EGO_TO_PREPARELINE) {
    if (cur_path_source ==
            pnc::geometry_lib::PathPlanSource::MULTI_ALIGN_INSLOT ||
        cur_path_source ==
            pnc::geometry_lib::PathPlanSource::MULTI_PLAN_INSLOT ||
        cur_path_source ==
            pnc::geometry_lib::PathPlanSource::ADJUST_PLAN_INSLOT) {
      path_source_condition = true;
    }
  }

  pnc::geometry_lib::PathPoint global_cur_pose(
      ego_info_under_slot.l2g_tf.GetPos(cur_pose.pos),
      ego_info_under_slot.l2g_tf.GetHeading(cur_pose.heading));

  Eigen::Vector2d end_to_cur = global_cur_pose.pos - global_end_pt.pos;
  Eigen::Vector2d end_pt_unit_dir(std::cos(global_end_pt.heading),
                                  std::sin(global_end_pt.heading));
  // Eigen::Vector2d end_pt_unit_dir = global_end_pt.pos.normalized();
  Eigen::Vector2d end_pt_normal_unit_dir(-end_pt_unit_dir.y(),
                                         end_pt_unit_dir.x());
  double lon_dir_length = end_to_cur.dot(end_pt_unit_dir);
  double lat_dir_length = end_to_cur.dot(end_pt_normal_unit_dir);
  ILOG_INFO << "lat_dir_length: " << lat_dir_length
            << " lon_dir_length: " << lon_dir_length;

  // ILOG_INFO << "cur_pose: " << cur_pose.pos.x() << " " << cur_pose.pos.y()
  //           << " heading: " << cur_pose.GetTheta();
  ILOG_INFO << "global_cur_pose: " << global_cur_pose.pos.x() << " "
            << global_cur_pose.pos.y()
            << " heading: " << global_cur_pose.GetTheta();
  // ILOG_INFO << "end_pt: " << end_pt.pos.x() << " " << end_pt.pos.y()
  //           << " heading: " << end_pt.GetTheta();
  ILOG_INFO << "global_end_pt: " << global_end_pt.pos.x() << " "
            << global_end_pt.pos.y()
            << " heading: " << global_end_pt.GetTheta();
  bool follow_pos_condition = false;
  if (std::fabs(pnc::geometry_lib::NormalizeAngle(global_cur_pose.heading -
                                                  global_end_pt.heading)) <
      (1.0 * kDeg2Rad)) {
    if (std::fabs(lat_dir_length) < 0.04 && std::fabs(lon_dir_length) < 0.1) {
      follow_pos_condition = true;
    }
  }

  Eigen::Vector2d previous_global_target_pos =
      previous_parallel_path_planner_.GetInput()
          .ego_info_under_slot.l2g_tf.GetPos(
              previous_parallel_path_planner_.GetInput()
                  .ego_info_under_slot.target_pose.pos);
  Eigen::Vector2d cur_global_target_pos = ego_info_under_slot.l2g_tf.GetPos(
      parallel_path_planner_.GetInput().ego_info_under_slot.target_pose.pos);

  double cur_global_target_head = ego_info_under_slot.l2g_tf.GetHeading(
      parallel_path_planner_.GetInput()
          .ego_info_under_slot.target_pose.heading);

  Eigen::Vector2d pos_vector =
      previous_global_target_pos - cur_global_target_pos;
  Eigen::Vector2d unit_dir(std::cos(cur_global_target_head),
                           std::sin(cur_global_target_head));
  // Eigen::Vector2d unit_dir = cur_global_target_pos.normalized();
  Eigen::Vector2d normal_unit_dir(-unit_dir.y(), unit_dir.x());
  // double projection_length = pos_vector.dot(unit_dir);
  double normal_length = pos_vector.dot(normal_unit_dir);

  ILOG_INFO << "previous_global_target_pos: " << previous_global_target_pos.x()
            << " " << previous_global_target_pos.y();
  ILOG_INFO << "cur_global_target_pos: " << cur_global_target_pos.x() << " "
            << cur_global_target_pos.y();
  ILOG_INFO << "normal_length: " << normal_length;
  bool target_y_condition = true;
  const bool y_dev =
      t_lane_.slot_side_sgn > 0
          ? (previous_global_target_pos.y() > cur_global_target_pos.y())
          : (previous_global_target_pos.y() < cur_global_target_pos.y());
  if (!y_dev) {
    target_y_condition = std::fabs(normal_length) < 0.1;
  }

  bool gear_condition =
      (previous_parallel_path_planner_.GetOutput().gear_cmd_vec.front() !=
       parallel_path_planner_.GetOutput().gear_cmd_vec.front());

  ILOG_INFO << "follow_pos_condition: " << int(follow_pos_condition)
            << " path_source_condition: " << int(path_source_condition)
            << " target_y_condition: " << int(target_y_condition)
            << " gear_condition: " << int(gear_condition);
  if (follow_pos_condition && target_y_condition &&
      (path_source_condition || gear_condition)) {
    if (!CheckLastPathCollided()) {
      ILOG_INFO << "use previous path";
      frame_.pathplan_result = PathPlannerResult::PLAN_HOLD;
      return previous_parallel_path_planner_;
    }
  }

  ILOG_INFO << "use replan path";
  frame_.pathplan_result = PathPlannerResult::PLAN_UPDATE;
  return parallel_path_planner_;
}

const ParallelPathGenerator& ParallelParkInScenario::SuitablePathReplan() {
  if (parallel_replan_again_ != 1) {
    return UseOrNotUseLastPath();
  }
  uint8_t cur_replan_gear =
      parallel_path_planner_.GetOutput().gear_cmd_vec.front();
  pnc::geometry_lib::PathSegGear cur_car_gear =
      apa_world_ptr_->GetMeasureDataManagerPtr()->GetGear();
  int cur_gear_size = parallel_path_planner_.GetOutput().gear_change_count;
  int pre_gear_size =
      previous_parallel_path_planner_.GetOutput().gear_change_count;
  ILOG_INFO << "cur_car_gear: " << int(cur_car_gear)
            << " cur_replan_gear: " << int(cur_replan_gear)
            << " cur_gear_size : " << cur_gear_size
            << " pre_gear_size : " << pre_gear_size;
  parallel_replan_again_ = 2;
  if (cur_car_gear != cur_replan_gear || cur_gear_size > pre_gear_size) {
    ILOG_INFO << "use previous path";
    parallel_path_planner_ = previous_parallel_path_planner_;
    frame_.pathplan_result = PathPlannerResult::PLAN_HOLD;
    return previous_parallel_path_planner_;
  }
  ILOG_INFO << "use replan path";
  frame_.pathplan_result = PathPlannerResult::PLAN_UPDATE;
  return parallel_path_planner_;
}


const uint8_t ParallelParkInScenario::PathPlanOnce() {
  ILOG_INFO << "start PathPlanOnce -------------";
  // construct input
  const EgoInfoUnderSlot& ego_info_under_slot =
      apa_world_ptr_->GetSlotManagerPtr()->GetEgoInfoUnderSlot();
  GeometryPathInput path_planner_input;
  path_planner_input.tlane = t_lane_;
  path_planner_input.sample_ds = apa_world_ptr_->GetSimuParam().sample_ds;
  path_planner_input.is_replan_first = frame_.is_replan_first;
  path_planner_input.is_complete_path =
      apa_world_ptr_->GetSimuParam().is_complete_path;
  path_planner_input.ego_info_under_slot = ego_info_under_slot;
  path_planner_input.is_searching_stage =
      apa_world_ptr_->GetStateMachineManagerPtr()->IsSeachingStatus();
  path_planner_input.out_again_path_better = out_again_path_better_;


  if (frame_.is_replan_first) {
    // temprarily give driving gear
    frame_.current_gear = pnc::geometry_lib::SEG_GEAR_DRIVE;

    frame_.current_arc_steer =
        t_lane_.slot_side == pnc::geometry_lib::SLOT_SIDE_RIGHT
            ? pnc::geometry_lib::SEG_STEER_LEFT
            : pnc::geometry_lib::SEG_STEER_RIGHT;

    path_planner_input.path_planner_state =
        ParallelPathGenerator::PrepareStepPlan;
  } else if (path_planner_input.ego_info_under_slot.slot_occupied_ratio >
                 kEnterMultiPlanSlotRatio &&
             frame_.in_slot_plan_count == 0) {
    frame_.current_gear = pnc::geometry_lib::SEG_GEAR_DRIVE;
    frame_.current_arc_steer =
        t_lane_.slot_side == pnc::geometry_lib::SLOT_SIDE_RIGHT
            ? pnc::geometry_lib::SEG_STEER_RIGHT
            : pnc::geometry_lib::SEG_STEER_LEFT;
  }

  ILOG_INFO << "slot_occupied_ratio = "
            << ego_info_under_slot.slot_occupied_ratio;
  ILOG_INFO << "frame_.in_slot_plan_count = "
            << static_cast<int>(frame_.in_slot_plan_count);

  path_planner_input.ref_gear = frame_.current_gear;
  path_planner_input.ref_arc_steer = frame_.current_arc_steer;

  ILOG_INFO << "ref gear to path planner input ="
            << static_cast<int>(path_planner_input.ref_gear);
  ILOG_INFO << "ref steer to path planner input ="
            << static_cast<int>(path_planner_input.ref_arc_steer);

  path_planner_input.parallel_replan_again_ = parallel_replan_again_;
  if (enable_pa_park_) {
    parallel_path_planner_.EnablePAPark();
    ILOG_INFO << "total_plan_count = " << int(frame_.total_plan_count)
              << " y diff = "
              << std::fabs(first_plan_cur_pos.pos.y() -
                           ego_info_under_slot.cur_pose.pos.y());
    if ((frame_.total_plan_count >
             apa_param.GetParam().pa_max_invalid_replan_times &&
         std::fabs(first_plan_cur_pos.pos.y() -
                   ego_info_under_slot.cur_pose.pos.y()) < 0.01) ||
        frame_.total_plan_count > kMaxReplanTimes) {
      path_planner_input.invalid_replan = true;
    }
  }
  parallel_path_planner_.SetInput(path_planner_input);

  const double path_plan_start_time = IflyTime::Now_ms();

  const bool path_plan_success =
      parallel_path_planner_.Update(apa_world_ptr_->GetCollisionDetectorPtr());

  ILOG_INFO << "path planner cost time(ms) = "
            << IflyTime::Now_ms() - path_plan_start_time;
  // const auto& path_planner_output = parallel_path_planner_.GetOutput();

  if (parallel_replan_again_ == 1 && !path_plan_success) {
    ILOG_INFO << "path planner replan failed, using last path";
    parallel_replan_again_ = 2;
    parallel_path_planner_ = previous_parallel_path_planner_;
    const auto& planner_output = parallel_path_planner_.GetOutput();
    frame_.plan_fail_reason = ParkingFailReason::NOT_FAILED;
    current_path_point_global_vec_.clear();
    current_path_point_global_vec_ = previous_current_path_point_global_vec_;
    if (!apa_world_ptr_->GetStateMachineManagerPtr()->IsSeachingStatus()) {
      std::vector<geometry_lib::PathPoint> tmp_path_point_vec;
      tmp_path_point_vec = current_path_point_global_vec_;
      parallel_path_planner_.TrimPathByLimiterPathPoint(tmp_path_point_vec,
                                                        true);
      if (tmp_path_point_vec.size() < current_path_point_global_vec_.size()) {
        current_path_point_global_vec_ = tmp_path_point_vec;
      }
    }

    pnc::geometry_lib::PathPoint global_point;
    complete_path_point_global_vec_.clear();
    complete_path_point_global_vec_.reserve(
        planner_output.all_gear_path_point_vec.size());
    for (const auto& path_point : planner_output.all_gear_path_point_vec) {
      global_point.Set(
          ego_info_under_slot.l2g_tf.GetPos(path_point.pos),
          ego_info_under_slot.l2g_tf.GetHeading(path_point.heading));
      global_point.lat_buffer = path_point.lat_buffer;
      global_point.s = path_point.s;
      global_point.kappa = path_point.kappa;
      global_point.gear = path_point.gear;
      complete_path_point_global_vec_.emplace_back(global_point);
    }
    frame_.pathplan_result = PathPlannerResult::PLAN_HOLD;
    return PathPlannerResult::PLAN_HOLD;
  }

  uint8_t plan_result = 0;
  if (!path_plan_success) {
    ILOG_INFO << "path plan fail!";
    complete_path_point_global_vec_.clear();
    plan_result = PathPlannerResult::PLAN_FAILED;
    frame_.plan_fail_reason = ParkingFailReason::PATH_PLAN_FAILED;
    frame_.pathplan_result = PathPlannerResult::PLAN_FAILED;
    return PathPlannerResult::PLAN_FAILED;
  }

  ILOG_INFO << "path plan success!";
  plan_result = PathPlannerResult::PLAN_UPDATE;
  frame_.plan_fail_reason = ParkingFailReason::NOT_FAILED;

  parallel_path_planner_.SetCurrentPathSegIndex();

  const auto path_planner_output = parallel_path_planner_.GetOutput();
  ILOG_INFO << "first seg idx = " << path_planner_output.path_seg_index.first;
  ILOG_INFO << "last seg idx = " << path_planner_output.path_seg_index.second;

  const auto& cur_path_end_pose =
      path_planner_output
          .path_segment_vec[path_planner_output.path_seg_index.second]
          .GetEndPose();

  const auto pos_error =
      (cur_path_end_pose.pos - t_lane_.pt_terminal_pos).cwiseAbs();

  const double heading_deg_diff_mag =
      std::fabs(cur_path_end_pose.heading * kRad2Deg);

  double current_path_length = 0.0;
  for (size_t i = path_planner_output.path_seg_index.first;
       i <= path_planner_output.path_seg_index.second; i++) {
    current_path_length += path_planner_output.path_segment_vec[i].GetLength();
  }

  // enter slot
  if (ego_info_under_slot.slot_occupied_ratio > kEnterMultiPlanSlotRatio) {
    double extend_lenth = 0.0;

    const bool is_final_path =
        pos_error.x() < apa_param.GetParam().finish_parallel_lon_err &&
        pos_error.y() < 0.4 &&
        heading_deg_diff_mag <
            apa_param.GetParam().finish_parallel_heading_err * kRad2Deg;

    if (is_final_path) {
      if (current_path_length < 0.15) {
        extend_lenth = 0.3;
      } else {
        extend_lenth = 0.0;
      }
    } else {
      if (current_path_length < 0.5) {
        extend_lenth = std::max(0.5 - current_path_length, 0.1);
      } else {
        if (heading_deg_diff_mag > 10.0 &&
            frame_.current_gear == pnc::geometry_lib::SEG_GEAR_REVERSE) {
          extend_lenth = 0.2;
        } else if (frame_.current_gear == pnc::geometry_lib::SEG_GEAR_DRIVE) {
          extend_lenth = 0.0;
        }
      }
    }

    parallel_path_planner_.InsertLineSegAfterCurrentFollowLastPath(
        extend_lenth, kInsertLineLonBuffer);
  } else {
    // outside slot normal extended length and buffer
    double lon_buffer = kInsertLineLonBuffer;
    double extend_lenth = kExtendLengthOutsideSlot;

    // ego is outof slot, current path is backward to slot
    if (path_planner_output.current_gear ==
            pnc::geometry_lib::SEG_GEAR_REVERSE &&
        pos_error.y() < t_lane_.slot_width * 0.5 &&
        mathlib::IsInBound(cur_path_end_pose.pos.x(), -1.5,
                           0.75 * t_lane_.slot_length) &&
        heading_deg_diff_mag > 5.0) {
      extend_lenth = 0.5;
      lon_buffer = 0.15;
    }

    parallel_path_planner_.InsertLineSegAfterCurrentFollowLastPath(extend_lenth,
                                                                   lon_buffer);
  }
  if (apa_param.GetParam().is_trim_limter_parallel_enable &&
      !path_planner_input.is_searching_stage) {
    // parallel_path_planner_.TrimPathByLimiterLastPathVec(true);
    parallel_path_planner_.TrimPathByLimiterPathPoint(
        current_path_point_global_vec_);
  }

  parallel_path_planner_.SampleCurrentPathSeg();
  parallel_path_planner_.JudgeNeedOptimize();
  parallel_path_planner_.SetLastPathSeg(
      parallel_path_planner_.GetOutput().path_point_vec);

  frame_.total_plan_count++;
  if (ego_info_under_slot.slot_occupied_ratio > kEnterMultiPlanSlotRatio) {
    frame_.in_slot_plan_count++;
  }

  // print segment info
  // pnc::geometry_lib::PrintSegmentsVecInfo(
  //     parallel_path_planner_.GetOutput().path_segment_vec);

  const ParallelPathGenerator& update_path_planner = SuitablePathReplan();
  const auto& planner_output = update_path_planner.GetOutput();//SuitablePathReplan();
  const EgoInfoUnderSlot& update_ego_info = update_path_planner.GetInput().ego_info_under_slot;
  plan_result = frame_.pathplan_result;

  // reverse info for next plan
  if (frame_.is_replan_first) {
    frame_.current_gear =
        pnc::geometry_lib::ReverseGear(planner_output.gear_cmd_vec.front());

    ILOG_INFO << "next gear =" << static_cast<int>(frame_.current_gear);
  } else {
    // set current gear
    frame_.current_gear = pnc::geometry_lib::ReverseGear(frame_.current_gear);
    ILOG_INFO << "next gear =" << static_cast<int>(frame_.current_gear);

    if (!pnc::geometry_lib::IsValidGear(frame_.current_gear)) {
      ILOG_INFO << "frame_.current_gear == invalid gear!";
      complete_path_point_global_vec_.clear();
      frame_.pathplan_result = PathPlannerResult::PLAN_FAILED;
      return PathPlannerResult::PLAN_FAILED;
    }

    if (ego_info_under_slot.slot_occupied_ratio > kEnterMultiPlanSlotRatio) {
      // set current arc steer
      frame_.current_arc_steer =
          pnc::geometry_lib::ReverseSteer(frame_.current_arc_steer);

      if (!pnc::geometry_lib::IsValidArcSteer(frame_.current_arc_steer)) {
        ILOG_INFO << "frame_.current_arc == invalid arc!";
        complete_path_point_global_vec_.clear();
        frame_.pathplan_result = PathPlannerResult::PLAN_FAILED;
        return PathPlannerResult::PLAN_FAILED;
      }
    }
  }

  frame_.gear_command = planner_output.current_gear;

  frame_.is_replan_first = false;

  ILOG_INFO << "start lat optimizer!";

  // lateral path optimization
  bool is_use_optimizer = planner_output.is_need_optimizer;
  ILOG_INFO  << "is_use_optimizer: " << is_use_optimizer;

  // refuse optimizer
  if (planner_output.path_point_vec.size() < 3) {
    ILOG_INFO << " input size is too small";
    is_use_optimizer = false;
  } else {
    const auto path_length = (planner_output.path_point_vec.front().pos -
                              planner_output.path_point_vec.back().pos)
                                 .norm();
    const double compensate_distance = 0.6;
    if (path_length <
        apa_param.GetParam().min_opt_path_length + compensate_distance) {
      ILOG_INFO << "path length is too short, optimizer is closed ";

      is_use_optimizer = false;
    }
  }

  bool cilqr_optimization_enable = true;
  bool parallel_optimization_enable = true;
  if (!apa_world_ptr_->GetSimuParam().is_simulation) {
    parallel_optimization_enable = apa_param.GetParam().parallel_lat_opt_enable;
    cilqr_optimization_enable =
        apa_param.GetParam().cilqr_path_optimization_enable;

  } else {
    parallel_optimization_enable =
        apa_world_ptr_->GetSimuParam().is_path_optimization;
    cilqr_optimization_enable =
        apa_world_ptr_->GetSimuParam().is_cilqr_optimization;
  }

  double lat_path_opt_cost_time_ms = 0.0;
  if (parallel_optimization_enable && is_use_optimizer) {
    ILOG_INFO << "------------------------ lateral path optimization "
                 "------------------------";

    ILOG_INFO << "frame_.gear_command_= "
              << static_cast<int>(frame_.gear_command);

    ILOG_INFO << "origin path size= " << planner_output.path_point_vec.size();

    LateralPathOptimizer::Parameter param;
    param.sample_ds = apa_world_ptr_->GetSimuParam().sample_ds;
    param.q_ref_xy = apa_world_ptr_->GetSimuParam().q_ref_xy;
    param.q_ref_theta = apa_world_ptr_->GetSimuParam().q_ref_theta;
    param.q_terminal_xy = apa_world_ptr_->GetSimuParam().q_terminal_xy;
    param.q_terminal_theta = apa_world_ptr_->GetSimuParam().q_terminal_theta;
    param.q_k = apa_world_ptr_->GetSimuParam().q_k;
    param.q_u = apa_world_ptr_->GetSimuParam().q_u;
    param.q_k_bound = apa_world_ptr_->GetSimuParam().q_k_bound;
    param.q_u_bound = apa_world_ptr_->GetSimuParam().q_u_bound;

    apa_world_ptr_->GetLateralPathOptimizerPtr()->Init(
        cilqr_optimization_enable);

    apa_world_ptr_->GetLateralPathOptimizerPtr()->SetParam(param);

    auto time_start = IflyTime::Now_ms();
    apa_world_ptr_->GetLateralPathOptimizerPtr()->Update(
        planner_output.path_point_vec, frame_.gear_command);

    auto time_end = IflyTime::Now_ms();
    lat_path_opt_cost_time_ms = time_end - time_start;

    const auto& optimized_path_vec =
        apa_world_ptr_->GetLateralPathOptimizerPtr()->GetOutputPathVec();

    current_path_point_global_vec_.clear();
    current_path_point_global_vec_.reserve(optimized_path_vec.size());

    pnc::geometry_lib::PathPoint global_point;
    for (const auto& path_point : optimized_path_vec) {
      global_point.Set(
          update_ego_info.l2g_tf.GetPos(path_point.pos),
          update_ego_info.l2g_tf.GetHeading(path_point.heading));

      global_point.s = path_point.s;
      global_point.kappa = path_point.kappa;
      global_point.gear = frame_.gear_command;

      current_path_point_global_vec_.emplace_back(global_point);
    }
    previous_current_path_point_global_vec_.clear();
    previous_current_path_point_global_vec_ = current_path_point_global_vec_;
    if (!apa_world_ptr_->GetStateMachineManagerPtr()->IsSeachingStatus()) {
      std::vector<geometry_lib::PathPoint> tmp_path_point_vec;
      tmp_path_point_vec = current_path_point_global_vec_;
      parallel_path_planner_.TrimPathByLimiterPathPoint(tmp_path_point_vec,
                                                        true);
      if (tmp_path_point_vec.size() < current_path_point_global_vec_.size()) {
        current_path_point_global_vec_ = tmp_path_point_vec;
      }
    }
    complete_path_point_global_vec_.clear();
    complete_path_point_global_vec_.reserve(
        planner_output.all_gear_path_point_vec.size());
    for (const auto& path_point : planner_output.all_gear_path_point_vec) {
      global_point.Set(
          update_ego_info.l2g_tf.GetPos(path_point.pos),
          update_ego_info.l2g_tf.GetHeading(path_point.heading));
      global_point.lat_buffer = path_point.lat_buffer;
      global_point.s = path_point.s;
      global_point.kappa = path_point.kappa;
      global_point.gear = path_point.gear;
      complete_path_point_global_vec_.emplace_back(global_point);
    }

    const auto plan_debug_info =
        apa_world_ptr_->GetLateralPathOptimizerPtr()->GetOutputDebugInfo();

    ILOG_INFO << "lat_path_opt_cost_time_ms = " << lat_path_opt_cost_time_ms;

    ILOG_INFO << "terminal point error = "
              << plan_debug_info.terminal_pos_error();

    ILOG_INFO << "terminal heading error = "
              << plan_debug_info.terminal_heading_error();
  } else {
    current_path_point_global_vec_.clear();
    current_path_point_global_vec_.reserve(
        planner_output.path_point_vec.size());

    pnc::geometry_lib::PathPoint global_point;
    for (const auto& path_point : planner_output.path_point_vec) {
      global_point.Set(
          update_ego_info.l2g_tf.GetPos(path_point.pos),
          update_ego_info.l2g_tf.GetHeading(path_point.heading));
      global_point.s = path_point.s;
      global_point.kappa = path_point.kappa;
      global_point.gear = frame_.gear_command;

      current_path_point_global_vec_.emplace_back(global_point);
    }
    previous_current_path_point_global_vec_.clear();
    previous_current_path_point_global_vec_ = current_path_point_global_vec_;
    if (!apa_world_ptr_->GetStateMachineManagerPtr()->IsSeachingStatus()) {
      std::vector<geometry_lib::PathPoint> tmp_path_point_vec;
      tmp_path_point_vec = current_path_point_global_vec_;
      parallel_path_planner_.TrimPathByLimiterPathPoint(tmp_path_point_vec,
                                                        true);
      if (tmp_path_point_vec.size() < current_path_point_global_vec_.size()) {
        current_path_point_global_vec_ = tmp_path_point_vec;
      }
    }
    complete_path_point_global_vec_.clear();
    complete_path_point_global_vec_.reserve(
        planner_output.all_gear_path_point_vec.size());
    for (const auto& path_point : planner_output.all_gear_path_point_vec) {
      global_point.Set(
          update_ego_info.l2g_tf.GetPos(path_point.pos),
          update_ego_info.l2g_tf.GetHeading(path_point.heading));
      global_point.lat_buffer = path_point.lat_buffer;
      global_point.s = path_point.s;
      global_point.kappa = path_point.kappa;
      global_point.gear = path_point.gear;
      complete_path_point_global_vec_.emplace_back(global_point);
    }
  }
  ILOG_INFO << "path_point_vec size= " << planner_output.path_point_vec.size();
  ILOG_INFO << "all_gear_path_point_vec size= "
            << planner_output.all_gear_path_point_vec.size();
  ILOG_INFO << "complete_path_point_global_vec_.size() = "
            << complete_path_point_global_vec_.size();

  JSON_DEBUG_VALUE("cilqr_optimization_enable", cilqr_optimization_enable);
  JSON_DEBUG_VALUE("lat_path_opt_cost_time_ms", lat_path_opt_cost_time_ms);

  ILOG_INFO << "current_path_point_global_vec_.size() = "
            << current_path_point_global_vec_.size();

  return plan_result;
}

const bool ParallelParkInScenario::CheckOneReverseToSlot() {
  const GeometryPathOutput& output = parallel_path_planner_.GetOutput();
  if (output.path_seg_index.second >= output.path_segment_vec.size()) {
    ILOG_INFO << "path_seg_index.second >= output.path_segment_vec.size";
    return false;
  }

  const auto& start_pose =
      output.path_segment_vec[output.path_seg_index.first].GetStartPose();

  const auto& end_pose =
      output.path_segment_vec[output.path_seg_index.second].GetEndPose();

  const bool is_start_pose_in_slot =
      CalcSlotOccupiedRatio(start_pose) >= kEnterMultiPlanSlotRatio;

  const bool is_end_pose_in_slot =
      CalcSlotOccupiedRatio(end_pose) >= kEnterMultiPlanSlotRatio;

  const bool is_reverse = output.gear_cmd_vec[output.path_seg_index.first] ==
                          geometry_lib::SEG_GEAR_REVERSE;

  const bool is_last_path =
      (output.path_seg_index.second == (output.gear_cmd_vec.size() - 1));

  if (is_reverse && !is_start_pose_in_slot && is_end_pose_in_slot &&
      is_last_path) {
    return true;
  }
  return false;
}

const bool ParallelParkInScenario::CheckFinished() {
  ILOG_INFO << "start CheckFinished!";
  if (frame_.is_replan_first) {
    ILOG_INFO << "before first finish, not check finish";
    return false;
  }

  const EgoInfoUnderSlot& ego_slot_info =
      apa_world_ptr_->GetSlotManagerPtr()->GetEgoInfoUnderSlot();

  // const Eigen::Vector2d rear_bumper_center =
  //     ego_slot_info.ego_pos_slot - apa_param.GetParam().rear_overhanging *
  //                                      ego_slot_info.ego_heading_slot_vec;

  // const Eigen::Vector2d front_bumper_center =
  //     ego_slot_info.ego_pos_slot + (apa_param.GetParam().wheel_base +
  //                                   apa_param.GetParam().front_overhanging)
  //                                   *
  //                                      ego_slot_info.ego_heading_slot_vec;

  const bool static_condition =
      apa_world_ptr_->GetMeasureDataManagerPtr()->GetStaticFlag();

  ILOG_INFO << "static_condition = " << static_condition;
  if (!static_condition) {
    delay_check_finish_ = false;
  }
  if (delay_check_finish_) {
    ILOG_INFO << "delay_check_finish_ = " << delay_check_finish_;
    return false;
  }

  double adjust_lon_err = apa_param.GetParam().finish_parallel_lon_err;
  if (CheckOneReverseToSlot()) {
    adjust_lon_err += kAdjustLonErr;
    ILOG_INFO << "adjust_lon_err = " << adjust_lon_err;
  }
  const bool lon_condition =
      std::fabs(ego_slot_info.terminal_err.pos.x()) < adjust_lon_err;

  ILOG_INFO << "terminal x error = " << ego_slot_info.terminal_err.pos.x();
  ILOG_INFO << "lon_condition = " << lon_condition;

  const bool heading_condition =
      std::fabs(ego_slot_info.terminal_err.heading) <=
      apa_param.GetParam().finish_parallel_heading_err / 57.3;

  ILOG_INFO << "terminal heading error = "
            << ego_slot_info.terminal_err.heading * 57.3;
  ILOG_INFO << "heading_condition = " << heading_condition;

  ILOG_INFO << "lat error = " << ego_slot_info.terminal_err.pos.y();
  const bool lat_condition_1 = std::fabs(ego_slot_info.terminal_err.pos.y()) <=
                               apa_param.GetParam().finish_parallel_lat_rac_err;

  // lat condition 2, keep both outer wheels in slot
  const double side_sgn =
      t_lane_.slot_side == pnc::geometry_lib::SLOT_SIDE_RIGHT ? 1.0 : -1.0;

  pnc::geometry_lib::LocalToGlobalTf ego2slot;
  ego2slot.Init(ego_slot_info.cur_pose.pos, ego_slot_info.cur_pose.heading);

  const auto& front_out_wheel = ego2slot.GetPos(
      Eigen::Vector2d(apa_param.GetParam().wheel_base,
                      0.5 * apa_param.GetParam().car_width * side_sgn));

  const auto& rear_out_wheel = ego2slot.GetPos(
      Eigen::Vector2d(0.0, 0.5 * apa_param.GetParam().car_width * side_sgn));

  const double slot_outer_pt_y =
      0.5 * ego_slot_info.slot.slot_width_ * side_sgn;
  bool lat_condition_2 = false;
  if (side_sgn > 0.0) {
    const double wheel_limit_y =
        slot_outer_pt_y - apa_param.GetParam().finish_parallel_lat_err;
    lat_condition_2 = (rear_out_wheel.y() <= wheel_limit_y) &&
                      (front_out_wheel.y() <= wheel_limit_y);
  } else {
    const double wheel_limit_y =
        slot_outer_pt_y + apa_param.GetParam().finish_parallel_lat_err;
    lat_condition_2 = (rear_out_wheel.y() >= wheel_limit_y) &&
                      (front_out_wheel.y() >= wheel_limit_y);
  }
  const bool lat_condition = lat_condition_1 || lat_condition_2;

  ILOG_INFO << "terminal y error = " << ego_slot_info.terminal_err.pos.y();
  ILOG_INFO << "lat condition  = " << lat_condition;
  if (lat_condition) {
    if (lat_condition_1) {
      ILOG_INFO << "lat y err = " << ego_slot_info.terminal_err.pos.y() << " < "
                << apa_param.GetParam().finish_parallel_lat_rac_err;
    } else {
      ILOG_INFO << "ego outer wheel are both in slot!";
    }
  }

  return lon_condition && lat_condition && heading_condition &&
         static_condition;
}

const double ParallelParkInScenario::CalcSlotOccupiedRatio(
    const Eigen::Vector2d& terminal_err, const double slot_width,
    const bool is_right_side) const {
  double slot_occupied_ratio = 0.0;
  if (pnc::mathlib::IsInBound(terminal_err.x(), -3.0, 4.0)) {
    const double y_err_ratio = terminal_err.y() / (0.5 * slot_width);

    if (is_right_side) {
      slot_occupied_ratio = pnc::mathlib::Clamp(1 - y_err_ratio, 0.0, 1.0);
    } else {
      slot_occupied_ratio = pnc::mathlib::Clamp(1.0 + y_err_ratio, 0.0, 1.0);
    }
  }
  return slot_occupied_ratio;
}

const double ParallelParkInScenario::CalcSlotOccupiedRatio(
    const pnc::geometry_lib::PathPoint start_pose) const {
  double slot_occupied_ratio = 0.0;

  if (pnc::mathlib::IsInBound(start_pose.pos.x(), t_lane_.pt_outside.x(),
                              t_lane_.pt_inside.x())) {
    const double y_err_ratio = start_pose.pos.y() / (0.5 * t_lane_.slot_width);

    if (t_lane_.slot_side_sgn) {
      slot_occupied_ratio = pnc::mathlib::Clamp(1 - y_err_ratio, 0.0, 1.0);
    } else {
      slot_occupied_ratio = pnc::mathlib::Clamp(1.0 + y_err_ratio, 0.0, 1.0);
    }
  }
  return slot_occupied_ratio;
}

void ParallelParkInScenario::Log() const {
  const auto& ego_info_under_slot =
      apa_world_ptr_->GetSlotManagerPtr()->GetEgoInfoUnderSlot();

  const auto& l2g_tf = ego_info_under_slot.l2g_tf;

  const auto p0_g = l2g_tf.GetPos(t_lane_.obs_pt_outside);
  const auto p1_g = l2g_tf.GetPos(t_lane_.obs_pt_inside);
  ILOG_INFO << "obs p out = " << p0_g.x();
  ILOG_INFO << "obs p in = " << p1_g.x();

  size_t real_obs_size = 0;
  for (const auto& obs_pair :
       apa_world_ptr_->GetCollisionDetectorPtr()->GetObstaclesMap()) {
    if (obs_pair.first == CollisionDetector::VIRTUAL_OBS) {
      continue;
    }
    real_obs_size += obs_pair.second.size();
  }

  ILOG_INFO << "obs_size = " << real_obs_size;
  const int count_unit = std::ceil(real_obs_size / 400.0);
  const size_t simplify_obs_num =
      std::ceil(real_obs_size / std::max(1, count_unit));

  std::vector<double> obstaclesX;
  std::vector<double> obstaclesY;
  obstaclesX.reserve(simplify_obs_num);
  obstaclesY.reserve(simplify_obs_num);
  for (const auto& obs_pair :
       apa_world_ptr_->GetCollisionDetectorPtr()->GetObstaclesMap()) {
    if (obs_pair.first == CollisionDetector::VIRTUAL_OBS) {
      continue;
    }
    const auto& obs_vec = obs_pair.second;

    for (size_t i = 0; i < obs_vec.size(); i += count_unit) {
      const auto obs_g = l2g_tf.GetPos(obs_vec[i]);
      obstaclesX.emplace_back(obs_g.x());
      obstaclesY.emplace_back(obs_g.y());
    }
  }
  if (obstaclesX.empty()) {
    obstaclesX = {0.0};
    obstaclesY = {0.0};
  }

  const size_t max_count = 798;
  if (obstaclesX.size() > max_count) {
    obstaclesX.resize(max_count);
    obstaclesY.resize(max_count);
  }
  RecordDebugObstacle(obstaclesX, obstaclesY);

  std::vector<double> limiter_corner_X = {-2.0, -2.0};
  std::vector<double> limiter_corner_Y = {1.2, -1.2};
  JSON_DEBUG_VECTOR("limiter_corner_X", limiter_corner_X, 2)
  JSON_DEBUG_VECTOR("limiter_corner_Y", limiter_corner_Y, 2)

  const auto& pts = ego_info_under_slot.slot.origin_corner_coord_global_;
  std::vector<double> slot_corner_X = {pts.pt_0.x(), pts.pt_1.x(), pts.pt_2.x(),
                                       pts.pt_3.x()};

  std::vector<double> slot_corner_Y = {pts.pt_0.y(), pts.pt_1.y(), pts.pt_2.y(),
                                       pts.pt_3.y()};
  JSON_DEBUG_VECTOR("slot_corner_X", slot_corner_X, 2)
  JSON_DEBUG_VECTOR("slot_corner_Y", slot_corner_Y, 2)

  JSON_DEBUG_VALUE("terminal_error_x", ego_info_under_slot.terminal_err.pos.x())
  JSON_DEBUG_VALUE("terminal_error_y", ego_info_under_slot.terminal_err.pos.y())
  JSON_DEBUG_VALUE("terminal_error_heading",
                   ego_info_under_slot.terminal_err.heading)

  JSON_DEBUG_VALUE("replan_flag", frame_.replan_flag)
  JSON_DEBUG_VALUE("is_replan_first", frame_.is_replan_first)
  JSON_DEBUG_VALUE("current_path_length", frame_.current_path_length)
  JSON_DEBUG_VALUE("path_plan_success", frame_.plan_stm.path_plan_success)
  JSON_DEBUG_VALUE("planning_status", frame_.plan_stm.planning_status)
  JSON_DEBUG_VALUE("spline_success", frame_.spline_success)
  JSON_DEBUG_VALUE("remain_dist", frame_.remain_dist_path)
  JSON_DEBUG_VALUE("remain_dist_obs", frame_.remain_dist_obs)
  JSON_DEBUG_VALUE("stuck_time", frame_.stuck_time)
  JSON_DEBUG_VALUE("replan_reason", frame_.replan_reason)
  JSON_DEBUG_VALUE("plan_fail_reason", frame_.plan_fail_reason)
  JSON_DEBUG_VALUE("ego_heading_slot", ego_info_under_slot.cur_pose.heading)

  JSON_DEBUG_VALUE("selected_slot_id", ego_info_under_slot.slot.id_)
  JSON_DEBUG_VALUE("slot_length", ego_info_under_slot.slot.GetLength())
  JSON_DEBUG_VALUE("slot_width", ego_info_under_slot.slot.GetWidth())

  JSON_DEBUG_VALUE("slot_origin_pos_x",
                   ego_info_under_slot.origin_pose_global.pos.x())

  JSON_DEBUG_VALUE("slot_origin_pos_y",
                   ego_info_under_slot.origin_pose_global.pos.y())

  JSON_DEBUG_VALUE("slot_origin_heading",
                   ego_info_under_slot.origin_pose_global.heading)

  JSON_DEBUG_VALUE("slot_occupied_ratio",
                   ego_info_under_slot.slot_occupied_ratio)

  const std::vector<double> target_ego_pos_slot = {
      ego_info_under_slot.target_pose.pos.x(),
      ego_info_under_slot.target_pose.pos.y()};

  JSON_DEBUG_VALUE("pathplan_result", frame_.pathplan_result)
  JSON_DEBUG_VECTOR("target_ego_pos_slot", target_ego_pos_slot, 2)

  const auto& path_plan_output = parallel_path_planner_.GetOutput();
  JSON_DEBUG_VALUE("path_start_seg_index",
                   path_plan_output.path_seg_index.first)
  JSON_DEBUG_VALUE("path_end_seg_index", path_plan_output.path_seg_index.second)
  JSON_DEBUG_VALUE("path_length", path_plan_output.length)

  // lateral optimization
  const auto plan_debug_info =
      apa_world_ptr_->GetLateralPathOptimizerPtr()->GetOutputDebugInfo();

  if (plan_debug_info.has_terminal_pos_error()) {
    JSON_DEBUG_VALUE("optimization_terminal_pose_error",
                     plan_debug_info.terminal_pos_error())
    JSON_DEBUG_VALUE("optimization_terminal_heading_error",
                     plan_debug_info.terminal_heading_error())
  } else {
    JSON_DEBUG_VALUE("optimization_terminal_pose_error", 0.0)
    JSON_DEBUG_VALUE("optimization_terminal_heading_error", 0.0)
  }
}

void ParallelParkInScenario::CalStaticBufferInDiffSteps(
    double& lat_buffer, double& safe_uss_remain_dist) const {
  const auto slot_mgr = apa_world_ptr_->GetSlotManagerPtr();
  const auto& ego_info = slot_mgr->GetEgoInfoUnderSlot();

  const auto& output = parallel_path_planner_.GetOutput();
  if (output.path_segment_vec.size() == 0) {
    lat_buffer = 0.0;
    safe_uss_remain_dist = 0.3;
    return;
  }

  const auto& start_pose =
      output.path_segment_vec[output.path_seg_index.first].GetStartPose();

  const auto& end_pose =
      output.path_segment_vec[output.path_seg_index.second].GetEndPose();

  const bool is_start_pose_in_slot =
      CalcSlotOccupiedRatio(start_pose) >= kEnterMultiPlanSlotRatio;

  const bool is_end_pose_in_slot =
      CalcSlotOccupiedRatio(end_pose) >= kEnterMultiPlanSlotRatio;

  const bool is_ego_in_slot =
      std::max(ego_info.slot_occupied_ratio,
               CalcSlotOccupiedRatio(apa_world_ptr_->GetSlotManagerPtr()
                                         ->GetMutableEgoInfoUnderSlot()
                                         .cur_pose)) >=
      kEnterMultiPlanSlotRatio;

  ILOG_INFO << "is_start_pose_in_slot = " << is_start_pose_in_slot;
  ILOG_INFO << "is_end_pose_in_slot = " << is_end_pose_in_slot;
  ILOG_INFO << "is_ego_in_slot = " << is_ego_in_slot;

  // totally in slot
  if (is_ego_in_slot && is_start_pose_in_slot && is_end_pose_in_slot) {
    ILOG_INFO << " totally in slot!";
    safe_uss_remain_dist =
        enable_pa_park_
            ? apa_param.GetParam().safe_uss_remain_dist_in_parallel_slot_pa
            : apa_param.GetParam().safe_uss_remain_dist_in_parallel_slot;

    lat_buffer =
        t_lane_.is_inside_rigid
            ? apa_param.GetParam().safe_lat_buffer_with_wall_in_parallel_slot
            : apa_param.GetParam().safe_lat_buffer_in_parallel_slot;
    return;
  }

  // out slot
  lat_buffer = apa_param.GetParam().safe_lat_buffer_outside_parallel_slot;
  safe_uss_remain_dist = apa_param.GetParam().safe_uss_remain_dist_out_slot;

  const bool is_reverse = output.gear_cmd_vec[output.path_seg_index.first] ==
                          geometry_lib::SEG_GEAR_REVERSE;
  // in 1r step
  if (is_reverse && !is_start_pose_in_slot && is_end_pose_in_slot) {
    ILOG_INFO << "in 1r step!";
    lat_buffer =
        t_lane_.is_inside_rigid
            ? apa_param.GetParam().safe_lat_buffer_with_wall_in_parallel_slot
            : apa_param.GetParam().safe_lat_buffer_in_1r_parallel_slot;

    safe_uss_remain_dist =
        t_lane_.is_inside_rigid
            ? apa_param.GetParam()
                  .safe_remain_dist_in_1r_with_wall_parallel_slot
            : apa_param.GetParam().safe_remain_dist_in_1r_parallel_slot;

  } else {
    ILOG_INFO << "outside slot not 1r step!";
  }
}

void ParallelParkInScenario::CalDynamicBufferInDiffSteps(
    double& dynaminc_lat_buffer, double& dynamic_lon_buffer) const {
  if (apa_world_ptr_->GetSlotManagerPtr()
          ->GetEgoInfoUnderSlot()
          .slot_occupied_ratio < kEnterMultiPlanSlotRatio) {
    dynaminc_lat_buffer = apa_param.GetParam().parallel_dynamic_lat_buffer;
    dynamic_lon_buffer = apa_param.GetParam().parallel_dynamic_lon_buffer;
  } else {
    dynaminc_lat_buffer =
        apa_param.GetParam().parallel_dynamic_lat_buffer_in_slot;
    dynamic_lon_buffer =
        apa_param.GetParam().parallel_dynamic_lon_buffer_in_slot;
  }
}

const bool ParallelParkInScenario::PostProcessPathPara() {
  const size_t origin_trajectory_size = current_path_point_global_vec_.size();
  if (origin_trajectory_size < 3) {
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
  frame_.current_path_length = s;
  frame_.x_s_spline.set_points(s_vec, x_vec);
  frame_.y_s_spline.set_points(s_vec, y_vec);

  frame_.spline_success = true;
  return true;

}

/*
 *@brief fitting line with ransac
 *@param line_coeffs: line coefficients
 *@param points: input points
 *@param inliers: output inliers mask bool vector with size of points
 *@return
 */
const bool ParallelParkInScenario::LineFittingWithRansac(
    Eigen::Vector2d& line_coeffs, const std::vector<Eigen::Vector2d>& points,
    std::vector<bool>& inliers) {
  if (points.size() < 2) {
    ILOG_ERROR << "error, points size: " << points.size();
    return false;
  }
  const double start_time_ms = IflyTime::Now_ms();

  int iterations = 300;
  double threshold = 0.02;
  int n = points.size();
  int bestInliersCount = 0;
  inliers.assign(n, false);

  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_int_distribution<int> dist(0, n - 1);


  for (int iter = 0; iter < iterations; ++iter) {
    int idx1 = dist(gen);
    int idx2 = 0;
    if (idx1 > n/2) {
      idx2 = idx1 - n/2;
    }else{
      idx2 = idx1 + n/2;
    }
    // int idx2 = dist(gen);

    while (idx1 == idx2) {
      idx2 = dist(gen);
    }

    const Point& p1 = points[idx1];
    const Point& p2 = points[idx2];

    if (std::abs(p2.x() - p1.x()) < 1e-8) {
      continue;
    }

    double currentA = (p2.y() - p1.y()) / (p2.x() - p1.x());
    double currentB = p1.y() - currentA * p1.x();

    std::vector<bool> currentInliers(n, false);
    int currentInliersCount = 0;

    for (int i = 0; i < n; ++i) {
      double distance =
          std::abs(currentA * points[i].x() - points[i].y() + currentB) /
          std::sqrt(currentA * currentA + 1);

      if (distance < threshold) {
        currentInliers[i] = true;
        currentInliersCount++;
      }
    }

    if (currentInliersCount > bestInliersCount) {
      bestInliersCount = currentInliersCount;
      inliers = currentInliers;

      int count = bestInliersCount;
      if (count >= 2) {
        ILOG_INFO << "A count = " << count;
        // 构建矩阵 A 和向量 b，求解 Ax = b

        // int last_size = 0;

        // for (int i = 0; i < n; ++i) {
        //   if (i % 3 == 0 && inliers[i]) {

        //     last_size++;
        //   }
        // }
        Eigen::MatrixXd A(2, 2);
        Eigen::VectorXd y(2);
        int idx = 0;
        for (int i = 0; i < n; ++i) {
          if ( inliers[i]) {
            A.row(idx) << points[i].x(), 1.0;
            y(idx) = points[i].y();
            idx++;
            break;
          }

        }
        for (int i = n-1; i < n; ++i) {
          if ( inliers[i]) {
            A.row(idx) << points[i].x(), 1.0;
            y(idx) = points[i].y();
            idx++;
            break;
          }

        }

        // 求解最小二乘问题: A * [a; b] = y
        line_coeffs = A.colPivHouseholderQr().solve(y);
      }
    }
  }

  // 结束计时并输出耗时
  const double end_time_ms = IflyTime::Now_ms();
  const double elapsed_time_ms = end_time_ms - start_time_ms;
  ILOG_INFO << "LineFittingWithRansac time: " << elapsed_time_ms << " ms";
  ILOG_INFO << "bestInliersCount: " << bestInliersCount << "/" << n;
  ILOG_INFO << "line param: y = " << line_coeffs(0) << "x + " <<
  line_coeffs(1);

  return true;
}

/*
 *@brief get the offset line coeffs with ego pose
 *@param line_coeffs_out: the offset line coeffs
 *@param line_coeffs_in: the input line coeffs
 *@param points: the points of the line
 *@param inliers_mask: the mask of the inliers
 *@param ego_pose: the ego pose
 *@return
 *
 */
const bool ParallelParkInScenario::GetOffsetLineCoeffsWithEgoPose(
    Eigen::Vector2d& line_coeffs_out, const Eigen::Vector2d& line_coeffs_in,
    const std::vector<Eigen::Vector2d>& points,
    const std::vector<bool>& inliers_mask, const Eigen::Vector2d& ego_pose) {
  if (points.size() < 2) { //|| inliers_mask.size() != points.size()
    ILOG_ERROR << "error, points size: " << points.size()
               << " or inliers_mask size != points size ";
    return false;
  }
  double max_dist_postive = 0.0;
  double max_dist_negtive = 0.0;
  int max_dist_postive_index = 0;
  int max_dist_negtive_index = 0;
  double tem_dis_ego =
      (line_coeffs_in(0) * ego_pose(0) + line_coeffs_in(1) - ego_pose(1)) /
      std::sqrt(line_coeffs_in(0) * line_coeffs_in(0) + 1);
  // cal the max distances of inlie points beside the line
  for (size_t i = 0; i < points.size(); ++i) {
    if (1) { // inliers_mask[i]
      double tem_dis = (line_coeffs_in(0) * points[i](0) + line_coeffs_in(1) -
                        points[i](1)) /
                       std::sqrt(line_coeffs_in(0) * line_coeffs_in(0) + 1);
      if (tem_dis > 0) {
        if (tem_dis > max_dist_postive) {
          max_dist_postive = tem_dis;
          max_dist_postive_index = i;
        }
      } else {
        if (tem_dis <= max_dist_negtive) {
          max_dist_negtive = tem_dis;
          max_dist_negtive_index = i;
        }
      }
    }
  }
  if (tem_dis_ego > 0) {  // ego pose  above the line
    line_coeffs_out(0) = line_coeffs_in(0);
    line_coeffs_out(1) =
        points[max_dist_postive_index](1) -
        line_coeffs_in(0) * points[max_dist_postive_index](0);  // b = y - ax
  } else {  // ego pose  below the line
    line_coeffs_out(0) = line_coeffs_in(0);
    line_coeffs_out(1) =
        points[max_dist_negtive_index](1) -
        line_coeffs_in(0) * points[max_dist_negtive_index](0);  // b = y - ax
  }
  return true;
}

}  // namespace apa_planner
}  // namespace planning
