#include "lane_change_state_machine_manager.h"
#include <Eigen/src/Core/Matrix.h>

#include <algorithm>
#include <climits>
#include <cmath>
#include <complex>
#include <cstddef>
#include <vector>

#include "config/basic_type.h"
#include "debug_info_log.h"
#include "define/geometry.h"
#include "ifly_time.h"
#include "log_glog.h"
#include "planning_context.h"
#include "spline_projection.h"
#include "utils/kd_path.h"
#include "virtual_lane.h"
namespace planning {
namespace {
constexpr double kEps = 1e-6;
constexpr double kLaneChangeWaitTimeoutThreshold = 15.0;
}  // namespace

LaneChangeStateMachineManager::LaneChangeStateMachineManager(
    const EgoPlanningConfigBuilder *config_builder, framework::Session *session,
    std::shared_ptr<LaneChangeRequestManager> lane_change_req_mgr,
    std::shared_ptr<LaneChangeLaneManager> lane_change_lane_mgr)
    : session_(session),
      lc_req_mgr_(lane_change_req_mgr),
      lc_lane_mgr_(lane_change_lane_mgr) {
  config_ = config_builder->cast<ScenarioStateMachineConfig>();
}

void LaneChangeStateMachineManager::Update() {
  RunStateMachine();
  GenerateStateMachineOutput();
  UpdateCoarsePlanningInfo();
  UpdateStateMachineDebugInfo();
}

void LaneChangeStateMachineManager::RunStateMachine() {
  switch (transition_info_.lane_change_status) {
    case StateMachineLaneChangeStatus::kLaneKeeping: {
      if (transition_info_.lane_change_status ==
          StateMachineLaneChangeStatus::kLaneKeeping) {
        // clear_lc_stage_info();
        RequestType lane_change_direction = NO_CHANGE;
        RequestSource lane_change_type = NO_REQUEST;
        bool is_lanekeeping_to_propose =
            CheckIfProposeLaneChange(&lane_change_direction, &lane_change_type);
        if (is_lanekeeping_to_propose) {
          transition_info_.lane_change_status =
              StateMachineLaneChangeStatus::kLaneChangePropose;
          transition_info_.lane_change_direction = lane_change_direction;
          transition_info_.lane_change_type = lane_change_type;
          lc_lane_mgr_->assign_lc_lanes(lc_req_mgr_->target_lane_virtual_id());
        } else {
          //在没有变道，过路口时，当前车道的virtual_id可能会发生跳变的现象
          //在这重新维护lc_lane的值，可以保证fix lane不会跳变
          lc_lane_mgr_->reset_lc_lanes(transition_info_.lane_change_status);
        }
      }
      break;
    }
    case StateMachineLaneChangeStatus::kLaneChangePropose: {
      if (transition_info_.lane_change_status ==
          StateMachineLaneChangeStatus::kLaneChangePropose) {
        bool is_propose_to_execution =
            CheckIfProposeToExecution(transition_info_.lane_change_direction,
                                      transition_info_.lane_change_type);
        bool is_propose_to_cancel =
            CheckIfProposeToCancel(transition_info_.lane_change_direction,
                                   transition_info_.lane_change_type);
        if (is_propose_to_execution) {
          transition_info_.lane_change_status =
              StateMachineLaneChangeStatus::kLaneChangeExecution;
          lc_lane_mgr_->set_fix_lane_to_target();
          lc_timer_.Reset();
        } else if (is_propose_to_cancel) {
          transition_info_.lane_change_status =
              StateMachineLaneChangeStatus::kLaneKeeping;
          ResetStateMachine();
        }
      }
      break;
    }
    case StateMachineLaneChangeStatus::kLaneChangeExecution: {
      if (transition_info_.lane_change_status ==
          StateMachineLaneChangeStatus::kLaneChangeExecution) {
        // Execution -> Complete
        bool is_lane_change_complete =
            CheckIfLaneChangeComplete(transition_info_.lane_change_direction,
                                      transition_info_.lane_change_type);
        // execution -> cancel
        bool is_execution_to_cancel =
            CheckIfExecutionToCancel(transition_info_.lane_change_direction,
                                     transition_info_.lane_change_type);
        // execution -> hold
        bool is_execution_to_hold =
            CheckIfExecutionToHold(transition_info_.lane_change_direction,
                                   transition_info_.lane_change_type);
        if (is_lane_change_complete) {
          transition_info_.lane_change_status =
              StateMachineLaneChangeStatus::kLaneChangeComplete;
        } else if (is_execution_to_cancel) {
          transition_info_.lane_change_status =
              StateMachineLaneChangeStatus::kLaneChangeCancel;
          lc_lane_mgr_->reset_lc_lanes(transition_info_.lane_change_status);
        } else if (is_execution_to_hold) {
          transition_info_.lane_change_status =
              StateMachineLaneChangeStatus::kLaneChangeHold;
          lc_lane_mgr_->set_fix_lane_to_origin();
        }
      }
      break;
    }
    case StateMachineLaneChangeStatus::kLaneChangeHold: {
      if (transition_info_.lane_change_status ==
          StateMachineLaneChangeStatus::kLaneChangeHold) {
        bool is_hold_to_cancel =
            CheckIfHoldToCancel(transition_info_.lane_change_direction,
                                transition_info_.lane_change_type);
        bool is_hold_to_execution =
            CheckIfHoldToExecution(transition_info_.lane_change_direction,
                                   transition_info_.lane_change_type);
        bool is_hold_to_complete =
            CheckIfLaneChangeComplete(transition_info_.lane_change_direction,
                                      transition_info_.lane_change_type);
        // is_hold_to_execution = false; //<hack> todo: update

        if (is_hold_to_complete) {
          transition_info_.lane_change_status =
              StateMachineLaneChangeStatus::kLaneChangeComplete;
          lc_timer_.Reset();
          lc_lane_mgr_->set_fix_lane_to_target();
          lane_change_stage_info_.Reset();
        } else if (is_hold_to_cancel) {
          transition_info_.lane_change_status =
              StateMachineLaneChangeStatus::kLaneChangeCancel;
          lc_lane_mgr_->reset_lc_lanes(transition_info_.lane_change_status);
          lane_change_stage_info_.Reset();
        } else if (is_hold_to_execution) {
          transition_info_.lane_change_status =
              StateMachineLaneChangeStatus::kLaneChangeExecution;
          lc_lane_mgr_->set_fix_lane_to_target();
        }
      }
      break;
    }
    case StateMachineLaneChangeStatus::kLaneChangeComplete: {
      if (transition_info_.lane_change_status ==
          StateMachineLaneChangeStatus::kLaneChangeComplete) {
        bool is_complete_to_lane_keeping = CheckIfCompleteToLaneKeeping();
        bool is_complete_to_cancel = CheckIfCompleteToCancel();
        if (is_complete_to_lane_keeping) {
          transition_info_.lane_change_status =
              StateMachineLaneChangeStatus::kLaneKeeping;
          ResetStateMachine();
          pre_lane_change_finish_time_ = IflyTime::Now_ms();
        } else if (is_complete_to_cancel) {
          transition_info_.lane_change_status =
              StateMachineLaneChangeStatus::kLaneChangeCancel;
          lc_lane_mgr_->reset_lc_lanes(transition_info_.lane_change_status);
        }
      }
      break;
    }
    case StateMachineLaneChangeStatus::kLaneChangeCancel: {
      if (transition_info_.lane_change_status ==
          StateMachineLaneChangeStatus::kLaneChangeCancel) {
        bool is_cancel_to_lane_keeping = CheckIfCancelToLaneKeeping();
        if (is_cancel_to_lane_keeping) {
          transition_info_.lane_change_status =
              StateMachineLaneChangeStatus::kLaneKeeping;
          ResetStateMachine();
          pre_lane_change_finish_time_ = IflyTime::Now_ms();
        }
      }
      break;
    }
    default:
      // should never enter here, add some debug info here
      break;
  }
  const int cur_state = transition_info_.lane_change_status;
  JSON_DEBUG_VALUE("cur_state", cur_state)
}

bool LaneChangeStateMachineManager::CheckIfProposeLaneChange(
    RequestType *const lane_change_direction,
    RequestSource *const lane_change_type) const {
  *lane_change_direction = lc_req_mgr_->request();
  *lane_change_type = lc_req_mgr_->request_source();
  if ((*lane_change_direction) != NO_CHANGE &&
      *lane_change_type != NO_REQUEST) {
    bool is_ego_in_perfect_pose = CheckIfInPerfectLaneKeeping();
    JSON_DEBUG_VALUE("is_ego_in_perfect_pose", is_ego_in_perfect_pose)
    if (*lane_change_type == INT_REQUEST) {
      return true;
    } else {
      return is_ego_in_perfect_pose;
    }
  }
  return false;
}

bool LaneChangeStateMachineManager::CheckIfProposeToExecution(
    const RequestType &lane_change_direction,
    const RequestSource &lane_change_type) {
  const auto &virtual_lane_manager =
      session_->environmental_model().get_virtual_lane_manager();
  const bool has_target_lane =
      virtual_lane_manager->has_lane(lc_req_mgr_->target_lane_virtual_id());
  // check lc gap if feasible
  CheckLaneChangeValid(lane_change_direction);
  return has_target_lane && lane_change_stage_info_.gap_insertable;
}

bool LaneChangeStateMachineManager::CheckIfProposeToCancel(
    const RequestType &lane_change_direction,
    const RequestSource &lane_change_type) {
  const double propose_time_threshold = 15;
  const bool is_no_lc_request = (lc_req_mgr_->request() == NO_CHANGE);
  bool propose_time_out =
      lc_req_mgr_->GetReqStartTime(lc_req_mgr_->request_source()) -
          IflyTime::Now_s() >
      propose_time_threshold;
  if (is_no_lc_request || propose_time_out) {
    return true;
  }
  return false;
}

bool LaneChangeStateMachineManager::CheckIfLaneChangeComplete(
    const RequestType &lane_change_direction,
    const RequestSource &lane_change_type) {
  const auto &ego_lane = session_->environmental_model()
                             .get_virtual_lane_manager()
                             ->get_current_lane();
  if (ego_lane == nullptr) {
    return false;
  }
  const auto &current_reference_path =
      session_->environmental_model()
          .get_reference_path_manager()
          ->get_reference_path_by_current_lane();
  if (current_reference_path == nullptr) {
    return false;
  }

  const double target_lane_virtual_id = lc_lane_mgr_->target_lane_virtual_id();
  const auto reference_path_manager =
      session_->environmental_model().get_reference_path_manager();
  const auto virtual_lane_manager =
      session_->environmental_model().get_virtual_lane_manager();
  const auto target_reference_path =
      reference_path_manager->get_reference_path_by_lane(
          target_lane_virtual_id);
  const double ego_l_target = target_reference_path->get_frenet_ego_state().l();
  const double ego_s_target = target_reference_path->get_frenet_ego_state().s();
  const double width_by_target =
      virtual_lane_manager->get_lane_with_virtual_id(target_lane_virtual_id)
          ->width();
  if (std::abs(ego_l_target) < std::abs(width_by_target) / 2) {
    return true;
  }

  const auto &reference_path_ego_state =
      current_reference_path->get_frenet_ego_state();
  const double ego_s = reference_path_ego_state.s();
  const double ego_l = reference_path_ego_state.l();
  double width = ego_lane->width(ego_s);
  double lane_change_complete_l_threshold =
      std::min(std::max(0.5, width / 4.0), 1.0);

  // check if ego lane jump to right direction
  bool jump_to_target_lane = false;
  if (ego_l * pre_ego_l_ < 0) {
    if (lane_change_direction == LEFT_CHANGE) {
      if (ego_l < 0 && pre_ego_l_ > 0) {
        jump_to_target_lane = true;
      }
    } else if (lane_change_direction == RIGHT_CHANGE) {
      if (ego_l > 0 && pre_ego_l_ < 0) {
        jump_to_target_lane = true;
      }
    }
  }
  if (jump_to_target_lane &&
      (std::fabs(ego_l) > lane_change_complete_l_threshold ||
       std::fabs(pre_ego_l_) > lane_change_complete_l_threshold)) {
    pre_ego_l_ = ego_l;
    return true;
  }
  pre_ego_l_ = ego_l;
  return false;
}

bool LaneChangeStateMachineManager::CheckIfExecutionToCancel(
    const RequestType &lane_change_direction,
    const RequestSource &lane_change_type) {
  // check if execution time out
  const double time_out_threshold_execute_time_s = 8.0;
  bool execution_time_out =
      TimeOut(true, &lc_timer_.execution_time_count_,
              &lc_timer_.execution_at_time_, time_out_threshold_execute_time_s);
  //(fengwang31)TODO:temp debug
  execution_time_out = false;
  if (execution_time_out) {
    return true;
  }
  // check if gap is dangerous
  CheckLaneChangeBackValid(lane_change_direction);
  // check if driver cancel
  const bool is_no_lc_request = (lc_req_mgr_->request() == NO_CHANGE);
  // NOTICE: some cancel needs to check whether lane change can be cancelled
  if ((lane_change_stage_info_.lc_should_back || is_no_lc_request)) {
    return true;
  }
  return false;
}

bool LaneChangeStateMachineManager::CheckIfExecutionToHold(
    const RequestType &lane_change_direction,
    const RequestSource &lane_change_type) {
  // check if execution time out
  const double time_out_threshold_execute_time_s = 8.0;
  bool execution_time_out =
      TimeOut(true, &lc_timer_.execution_time_count_,
              &lc_timer_.execution_at_time_, time_out_threshold_execute_time_s);
  //(fengwang31)TODO:temp debug
  execution_time_out = false;
  if (execution_time_out) {
    return true;
  }
  CheckLaneChangeBackValid(lane_change_direction);
  const double time_out_threshold_gap_not_available_time_s = 0.15;
  const bool gap_not_available_time_out =
      TimeOut(lane_change_stage_info_.lc_should_back,
              &lc_timer_.gap_not_available_time_count_,
              &lc_timer_.gap_not_available_at_time_,
              time_out_threshold_gap_not_available_time_s);

  const bool is_no_lc_request = (lc_req_mgr_->request() == NO_CHANGE);

  if ((lane_change_stage_info_.lc_should_back || is_no_lc_request) &&
      lane_change_type == MAP_REQUEST) {
    return true;
  }
  return false;
}

bool LaneChangeStateMachineManager::CheckIfHoldToCancel(
    const RequestType &lane_change_direction,
    const RequestSource &lane_change_type) {
  // check if hold time out
  const double time_out_threshold_hold_time_s = 1;
  bool hold_time_out =
      TimeOut(true, &lc_timer_.hold_time_count_, &lc_timer_.hold_at_time_,
              time_out_threshold_hold_time_s);
  if (hold_time_out) {
    return true;
  }

  return false;
}

bool LaneChangeStateMachineManager::CheckIfHoldToExecution(
    const RequestType &lane_change_direction,
    const RequestSource &lane_change_type) {
  const auto &virtual_lane_manager =
      session_->environmental_model().get_virtual_lane_manager();
  const bool has_target_lane =
      virtual_lane_manager->has_lane(lc_req_mgr_->target_lane_virtual_id());
  CheckLaneChangeValid(lane_change_direction);
  const double hold_to_execution_is_safe_time_out_threshold = 0.2;
  const bool is_safe_time_out =
      TimeOut(lane_change_stage_info_.gap_insertable,
              &lc_timer_.is_safe_hold_to_execution_count_,
              &lc_timer_.is_safe_hold_to_execution_at_time_,
              hold_to_execution_is_safe_time_out_threshold);
  return lane_change_stage_info_.gap_insertable && has_target_lane;
}

bool LaneChangeStateMachineManager::CheckIfCompleteToLaneKeeping() const {
  const bool perfect_in_lane = CheckIfInPerfectLaneKeeping();
  if (perfect_in_lane) {
    std::cout << "perfect_in_lane!!!" << std::endl;
  }
  return perfect_in_lane;
}

bool LaneChangeStateMachineManager::CheckIfInPerfectLaneKeeping() const {
  const auto &virtual_lane_mgr =
      session_->mutable_environmental_model()->get_virtual_lane_manager();
  const auto &clane = virtual_lane_mgr->get_current_lane();
  const auto &clane_virtual_id = virtual_lane_mgr->current_lane_virtual_id();
  const int origin_relative_id_zero_nums =
      virtual_lane_mgr->origin_relative_id_zero_nums();
  bool perfect_in_diversion_lane = false;

  const double v_ego =
      session_->mutable_environmental_model()->get_ego_state_manager()->ego_v();
  std::vector<double> xp_v_ego{10.0, 15.0, 20.0, 25.0};
  double dist_threshold = interp(v_ego, xp_v_ego, config_.lc_finished_dist_thr);
  if (road_to_ramp_turn_signal_ != RAMP_NONE) {
    //匝道汇主路打灯时，关灯阈值可以增大一点
    dist_threshold = 0.3;
  }

  std::vector<double> angle_thre_v{0.72, 0.48, 0.12};
  std::vector<double> angle_thre_bp{1.0, 3.0, 5.0};
  double angle_threshold = interp(v_ego, angle_thre_bp, angle_thre_v);

  std::shared_ptr<ReferencePathManager> reference_path_mgr =
      session_->mutable_environmental_model()->get_reference_path_manager();
  const auto &current_reference_path =
      reference_path_mgr->get_reference_path_by_lane(clane_virtual_id);
  const auto &frenet_ego_state = current_reference_path->get_frenet_ego_state();

  bool perfect_in_lane = false;
  perfect_in_lane =
      ((std::fabs(frenet_ego_state.l()) < dist_threshold) &&
       (std::fabs(frenet_ego_state.heading_angle()) < angle_threshold));

  if (origin_relative_id_zero_nums > 1) {
    perfect_in_diversion_lane =
        (std::fabs(frenet_ego_state.l()) < dist_threshold);
  }
  perfect_in_lane = perfect_in_lane || perfect_in_diversion_lane;

  return perfect_in_lane;
}

bool LaneChangeStateMachineManager::CheckIfCompleteToCancel() {
  const double compete_time_threshold = 8.0;
  bool complete_time_out =
      TimeOut(true, &lc_timer_.complete_time_count_,
              &lc_timer_.complete_at_time_, compete_time_threshold);
  //(fengwang31)TODO:temp debug
  complete_time_out = false;
  return complete_time_out;
}

bool LaneChangeStateMachineManager::CheckIfCancelToLaneKeeping() const {
  const bool perfect_in_lane = CheckIfInPerfectLaneKeeping();
  return perfect_in_lane;
}

bool LaneChangeStateMachineManager::CheckIfCancelTimeOut() {
  const double cancel_time_out_threshold = 6.0;
  bool cancel_time_out =
      TimeOut(true, &lc_timer_.cancel_time_count_, &lc_timer_.cancel_at_time_,
              cancel_time_out_threshold);

  if (cancel_time_out) {
    // debug_string_ += "CheckIfCancelTimeOut cancel_time_out!";
    // cancel_reason_ += "CheckIfCancelTimeOut cancel_time_out!";
  }
  return cancel_time_out;
}

void LaneChangeStateMachineManager::LaneChangeInfoReset() {
  transition_info_.Rest();
  lc_timer_.Reset();
}

void LaneChangeStateMachineManager::CheckLaneChangeValid(
    RequestType direction) {
  // check single frame lc gap if feasible
  lane_change_stage_info_ = CheckLCGapFeasible(direction);
  int lc_valid_thre = 4;
  const auto &virtual_lane_manager =
      session_->mutable_environmental_model()->get_virtual_lane_manager();
  const double dis_to_ramp = virtual_lane_manager->dis_to_ramp();
  if (dis_to_ramp < 1000.) lc_valid_thre = 1;
  // can lc if more than continue 4 frame gap_insertable
  if (lane_change_stage_info_.gap_insertable) {
    lc_valid_cnt_ += 1;
    LOG_DEBUG("decide_lc_valid_info lc_valid_cnt : %d \n", lc_valid_cnt_);
    if (lc_valid_cnt_ > lc_valid_thre) {
      lane_change_stage_info_.lc_valid = true;
    } else {
      lane_change_stage_info_.gap_insertable = false;
      lane_change_stage_info_.lc_invalid_reason = "valid cnt below threshold";
    }
  } else {
    LOG_DEBUG("arbitrator lc invalid reason %s ",
              lane_change_stage_info_.lc_invalid_reason.c_str());
    lc_valid_cnt_ = 0;
  }
}

LaneChangeStageInfo LaneChangeStateMachineManager::CheckLCGapFeasible(
    RequestType direction) {
  assert(direction == LEFT_CHANGE || direction == RIGHT_CHANGE);
  LaneChangeStageInfo lc_state_info;
  const auto &lateral_obstacle =
      session_->environmental_model().get_lateral_obstacle();
  const auto &virtual_lane_manager =
      session_->mutable_environmental_model()->get_virtual_lane_manager();
  const auto &target_lane = virtual_lane_manager->get_lane_with_virtual_id(
      lc_req_mgr_->target_lane_virtual_id());
  if (target_lane == nullptr) {
    return lc_state_info;
  }
  lc_state_info.gap_insertable = true;
  lc_state_info.gap_approached = true;
  lc_state_info.gap_valid = true;
  lc_state_info.side_approach = false;
  lc_state_info.gap = {-10, -10};
  lc_invalid_track_.reset();

  if (!lateral_obstacle->sensors_okay()) {
    if (lateral_obstacle->fvf_dead()) {
      lc_state_info.lc_invalid_reason = "no front view";
    } else if (lateral_obstacle->svf_dead()) {
      lc_state_info.lc_invalid_reason = "no side view";
    }
    lc_state_info.gap_valid = false;
    lc_state_info.gap_approached = false;
    lc_state_info.gap_insertable = false;
    return lc_state_info;
  }
  near_cars_target_.clear();
  std::vector<TrackInfo> near_cars_target;
  std::vector<TrackedObject> side_target_tracks;
  std::vector<TrackedObject> front_target_tracks;
  auto target_lane_reference_path = target_lane->get_reference_path();
  if (target_lane_reference_path == nullptr) {
    return lc_state_info;
  }
  auto tlane_obstacles = target_lane_reference_path->get_lane_obstacles_ids();
  //处理目标车车道后方障碍物
  for (auto &obstacle : lateral_obstacle->side_tracks()) {
    if (std::count(tlane_obstacles.begin(), tlane_obstacles.end(),
                   obstacle.track_id) > 0) {
      if (!(obstacle.fusion_source & OBSTACLE_SOURCE_CAMERA)) {
        continue;
      }
      side_target_tracks.push_back(obstacle);
    }
  }
  for (auto &tr : side_target_tracks) {
    TrackInfo side_track(tr.track_id, tr.d_rel, tr.v_rel);
    near_cars_target.push_back(side_track);
  }
  if (!side_target_tracks.empty()) {
    std::sort(side_target_tracks.begin(), side_target_tracks.end(),
              [](const TrackedObject &obs1, const TrackedObject &obs2) {
                return std::abs(obs1.d_rel) < std::abs(obs2.d_rel);
              });
    CalculateSideGapFeasible(side_target_tracks, &lc_state_info);
    if (!lc_state_info.gap_insertable) {
      return lc_state_info;
    }
  }
  // 处理目标车车道前方障碍物，整体逻辑同上处理后方障碍物
  for (auto &obstacle : lateral_obstacle->front_tracks()) {
    if (std::count(tlane_obstacles.begin(), tlane_obstacles.end(),
                   obstacle.track_id) > 0) {
      if (!(obstacle.fusion_source & OBSTACLE_SOURCE_CAMERA)) {
        continue;
      }
      front_target_tracks.push_back(obstacle);
    }
  }
  for (auto &tr : front_target_tracks) {
    TrackInfo front_track(tr.track_id, tr.d_rel, tr.v_rel);
    near_cars_target_.push_back(front_track);
  }
  if (!front_target_tracks.empty()) {
    std::sort(front_target_tracks.begin(), front_target_tracks.end(),
              [](const TrackedObject &obs1, const TrackedObject &obs2) {
                return std::abs(obs1.d_rel) < std::abs(obs2.d_rel);
              });
    CalculateFrontGapFeasible(front_target_tracks, &lc_state_info);
    if (!lc_state_info.gap_insertable) {
      return lc_state_info;
    }
  }
  return lc_state_info;
}

void LaneChangeStateMachineManager::CheckLaneChangeBackValid(
    RequestType direction) {
  lane_change_stage_info_ = CheckIfNeedLCBack(direction);
  int lc_back_thre = 4;
  if (lane_change_stage_info_.lc_should_back) {
    lc_back_cnt_ += 1;
    if (lc_back_cnt_ <= lc_back_thre) {
      lane_change_stage_info_.lc_should_back = false;
      lane_change_stage_info_.lc_back_reason = "but back cnt below threshold";
    }
  } else {
    lc_back_cnt_ = 0;
  }
}

LaneChangeStageInfo LaneChangeStateMachineManager::CheckIfNeedLCBack(
    RequestType direction) {
  LaneChangeStageInfo lc_state_info;
  const auto &lateral_obstacle =
      session_->environmental_model().get_lateral_obstacle();
  const auto &fix_lane =
      session_->environmental_model()
          .get_virtual_lane_manager()
          ->get_lane_with_virtual_id(lc_lane_mgr_->fix_lane_virtual_id());
  if (fix_lane == nullptr) {
    return lc_state_info;
  }
  if (!lateral_obstacle->sensors_okay()) {
    if (lateral_obstacle->fvf_dead()) {
      lc_state_info.lc_back_reason = "no front view";
    } else if (lateral_obstacle->svf_dead()) {
      lc_state_info.lc_back_reason = "no side view";
    }
    lc_state_info.lc_should_back = true;
    return lc_state_info;
  }
  auto fix_lane_reference_path = fix_lane->get_reference_path();
  if (fix_lane_reference_path == nullptr) {
    return lc_state_info;
  }
  const auto &tlane_obstacles =
      fix_lane_reference_path->get_lane_obstacles_ids();

  near_cars_target_.clear();
  std::vector<TrackInfo> near_cars_target;
  std::vector<TrackedObject> side_target_tracks;
  std::vector<TrackedObject> front_target_tracks;

  for (auto &obstacle : lateral_obstacle->side_tracks()) {
    if (std::count(tlane_obstacles.begin(), tlane_obstacles.end(),
                   obstacle.track_id) > 0) {
      if (!(obstacle.fusion_source & OBSTACLE_SOURCE_CAMERA)) {
        continue;
      }
      side_target_tracks.push_back(obstacle);
    }
  }

  for (auto &obstacle : lateral_obstacle->front_tracks()) {
    if (std::count(tlane_obstacles.begin(), tlane_obstacles.end(),
                   obstacle.track_id) > 0) {
      if (!(obstacle.fusion_source & OBSTACLE_SOURCE_CAMERA)) {
        continue;
      }
      front_target_tracks.push_back(obstacle);
    }
  }

  if (side_target_tracks.size() > 0) {
    std::sort(side_target_tracks.begin(), side_target_tracks.end(),
              [](const TrackedObject &obs1, const TrackedObject &obs2) {
                return std::abs(obs1.d_rel) < std::abs(obs2.d_rel);
              });
    for (auto &tr : side_target_tracks) {
      TrackInfo side_track(tr.track_id, tr.d_rel, tr.v_rel);
      near_cars_target.push_back(side_track);
    }

    CalculateSideAreaIfNeedBack(side_target_tracks, direction, &lc_state_info);
  }

  bool is_do_not_lc_back_due_to_over_lat_threshold =
      !lc_state_info.lc_should_back &&
      (lc_state_info.lc_back_reason == "exceed move_thre, do not back");
  if (lc_state_info.lc_should_back ||
      is_do_not_lc_back_due_to_over_lat_threshold) {
    return lc_state_info;
  }

  if (front_target_tracks.size() > 0) {
    std::sort(front_target_tracks.begin(), front_target_tracks.end(),
              [](const TrackedObject &obs1, const TrackedObject &obs2) {
                return std::abs(obs1.d_rel) < std::abs(obs2.d_rel);
              });

    for (auto &tr : front_target_tracks) {
      TrackInfo front_track(tr.track_id, tr.d_rel, tr.v_rel);
      near_cars_target_.push_back(front_track);
    }

    CalculateFrontAreaIfNeedBack(front_target_tracks, direction,
                                 &lc_state_info);
  }
  if (lc_state_info.close_to_accident && lc_state_info.lc_should_back) {
    lc_state_info.lc_should_back = false;
    lc_state_info.accident_back = true;
  }
  return lc_state_info;
}

void LaneChangeStateMachineManager::UpdateCoarsePlanningInfo() {
  // Step 1) 计算state
  auto &coarse_planning_info = session_->mutable_planning_context()
                                   ->mutable_lane_change_decider_output()
                                   .coarse_planning_info;
  coarse_planning_info.source_state = coarse_planning_info.target_state;
  const int current_state =
      session_->planning_context().lane_change_decider_output().curr_state;
  coarse_planning_info.target_state =
      static_cast<StateMachineLaneChangeStatus>(current_state);
  coarse_planning_info.lane_change_request_source =
      lc_req_mgr_->request_source();
  coarse_planning_info.source_lane_id = lc_lane_mgr_->origin_lane_virtual_id();
  coarse_planning_info.target_lane_id = lc_lane_mgr_->fix_lane_virtual_id();
  coarse_planning_info.reference_path =
      session_->mutable_environmental_model()
          ->get_reference_path_manager()
          ->make_map_lane_reference_path(coarse_planning_info.target_lane_id);

  const auto &planning_init_point =
      coarse_planning_info.reference_path->get_frenet_ego_state()
          .planning_init_point();
  // Step 3) calculate trajectory points
  // generate reference path
  static const double min_ego_v_cruise = 2.0;
  const auto &v_ref_cruise = std::fmax(
      session_->environmental_model().get_ego_state_manager()->ego_v_cruise(),
      min_ego_v_cruise);

  static const size_t &N = config_.num_point;
  const auto &delta_time = config_.delta_t;

  auto &cart_ref_info = coarse_planning_info.cart_ref_info;
  const auto &frenet_coord =
      coarse_planning_info.reference_path->get_frenet_coord();

  double s = 0.0;
  Point2D frenet_pt{s, 0.0};
  Point2D cart_pt(0.0, 0.0);
  const auto &ref_point = coarse_planning_info.reference_path->get_points();
  auto point_size = ref_point.size();
  cart_ref_info.x_vec.resize(point_size);
  cart_ref_info.y_vec.resize(point_size);
  cart_ref_info.s_vec.resize(point_size);
  std::vector<double> kappa_radius_vec;
  kappa_radius_vec.resize(point_size);
  float normal_care_spline_length = 50.;
  const float preview_time = 20.;
  const double min_preview_spline_length = 20.;

  if (session_->is_hpp_scene()) {
    const double kHppMaxRearDistance = 30.0;
    double distance_to_first_point = kHppMaxRearDistance;
    if (point_size > 0) {
      double diff_x =
          ref_point.at(0).path_point.x - planning_init_point.lat_init_state.x();
      double diff_y =
          ref_point.at(0).path_point.y - planning_init_point.lat_init_state.y();
      distance_to_first_point = std::sqrt(diff_x * diff_x + diff_y * diff_y);
    }

    normal_care_spline_length =
        std::min(kHppMaxRearDistance, distance_to_first_point);
  }

  for (size_t i = 0; i < point_size; ++i) {
    cart_ref_info.x_vec[i] = ref_point.at(i).path_point.x;
    cart_ref_info.y_vec[i] = ref_point.at(i).path_point.y;
    cart_ref_info.s_vec[i] =
        i > 0 ? cart_ref_info.s_vec[i - 1] +
                    std::hypot(ref_point.at(i).path_point.x -
                                   ref_point.at(i - 1).path_point.x,
                               ref_point.at(i).path_point.y -
                                   ref_point.at(i - 1).path_point.y)
              : 0.;
    kappa_radius_vec[i] = std::min(
        std::max(1.0 / (ref_point.at(i).path_point.kappa + 1e-6), -10000.0),
        10000.0);
    if (cart_ref_info.s_vec[i] >
        normal_care_spline_length +
            std::max(v_ref_cruise * preview_time, min_preview_spline_length)) {
      cart_ref_info.x_vec.resize(i);
      cart_ref_info.y_vec.resize(i);
      cart_ref_info.s_vec.resize(i);
      kappa_radius_vec.resize(i);
      break;
    }
  }

  cart_ref_info.x_s_spline.set_points(cart_ref_info.s_vec, cart_ref_info.x_vec);
  cart_ref_info.y_s_spline.set_points(cart_ref_info.s_vec, cart_ref_info.y_vec);

  JSON_DEBUG_VECTOR("raw_refline_x_vec", cart_ref_info.x_vec, 2)
  JSON_DEBUG_VECTOR("raw_refline_y_vec", cart_ref_info.y_vec, 2)
  JSON_DEBUG_VECTOR("raw_refline_s_vec", cart_ref_info.s_vec, 2)
  JSON_DEBUG_VECTOR("raw_refline_k_vec", kappa_radius_vec, 2)

  Eigen::Vector2d init_pos(planning_init_point.lat_init_state.x(),
                           planning_init_point.lat_init_state.y());
  const auto &ego_state =
      session_->environmental_model().get_ego_state_manager();

  JSON_DEBUG_VALUE("ego_pos_x", ego_state->ego_pose_raw().x)
  JSON_DEBUG_VALUE("ego_pos_y", ego_state->ego_pose_raw().y)
  JSON_DEBUG_VALUE("ego_pos_yaw", ego_state->ego_pose_raw().theta)
  JSON_DEBUG_VALUE("init_pos_x", init_pos.x())
  JSON_DEBUG_VALUE("init_pos_y", init_pos.y())

  // start from project s
  pnc::spline::Projection projection_spline;
  projection_spline.CalProjectionPoint(
      cart_ref_info.x_s_spline, cart_ref_info.y_s_spline,
      cart_ref_info.s_vec.front(), cart_ref_info.s_vec.back(), init_pos);

  double s_ref = projection_spline.GetOutput().s_proj;

  const auto &frenet_length = frenet_coord->Length();

  s_ref = std::min(s_ref, frenet_length * 0.95);
  const double delta_s = frenet_length - s_ref;
  const double v_cruise_scale = std::min(delta_s / (v_ref_cruise * 5.0), 1.0);
  session_->mutable_planning_context()->set_v_ref_cruise(v_cruise_scale *
                                                         v_ref_cruise);

  coarse_planning_info.trajectory_points.clear();
  TrajectoryPoint point;
  for (size_t i = 0; i < N; ++i) {
    // cart info
    if (s_ref < cart_ref_info.s_vec.back() + kEps) {
      point.x = cart_ref_info.x_s_spline(s_ref);
      point.y = cart_ref_info.y_s_spline(s_ref);
      point.heading_angle =
          std::atan2(cart_ref_info.y_s_spline.deriv(1, s_ref),
                     cart_ref_info.x_s_spline.deriv(1, s_ref));
    }

    // frenet info
    Point2D frenet_pt{0.0, 0.0};
    Point2D cart_pt(point.x, point.y);
    frenet_coord->XYToSL(cart_pt, frenet_pt);
    point.s = frenet_pt.x;
    point.l = frenet_pt.y;
    point.t = static_cast<double>(i) * delta_time;

    s_ref += v_cruise_scale * v_ref_cruise * delta_time;
    coarse_planning_info.trajectory_points.emplace_back(point);
  }
}

void LaneChangeStateMachineManager::GenerateStateMachineOutput() {
  auto &lane_change_decider_output = session_->mutable_planning_context()
                                         ->mutable_lane_change_decider_output();

  lane_change_decider_output.curr_state = transition_info_.lane_change_status;
  lane_change_decider_output.fix_lane_virtual_id =
      lc_lane_mgr_->fix_lane_virtual_id();
  lane_change_decider_output.origin_lane_virtual_id =
      lc_lane_mgr_->origin_lane_virtual_id();
  lane_change_decider_output.target_lane_virtual_id =
      lc_lane_mgr_->target_lane_virtual_id();
  lane_change_decider_output.has_target_lane = lc_lane_mgr_->has_target_lane();

  lane_change_decider_output.scenario = scenario_;
  lane_change_decider_output.lc_back_reason =
      lane_change_stage_info_.lc_back_reason;
  lane_change_decider_output.lc_invalid_reason =
      lane_change_stage_info_.lc_invalid_reason;
  lane_change_decider_output.behavior_suspend = behavior_suspend_;
  lane_change_decider_output.suspend_obs.assign(suspend_obs_.begin(),
                                                suspend_obs_.end());
  lane_change_decider_output.must_change_lane = must_change_lane_;

  if (map_turn_signal_ != NO_CHANGE) {
    lane_change_decider_output.turn_light = map_turn_signal_;
  } else if (lc_req_mgr_->turn_signal() > 0) {
    lane_change_decider_output.turn_light = lc_req_mgr_->turn_signal();
  } else {
    lane_change_decider_output.turn_light = 0;
  }
  lane_change_decider_output.map_turn_light = map_turn_signal_;

  lane_change_decider_output.is_lc_valid = lane_change_stage_info_.lc_valid;
  lane_change_decider_output.accident_back =
      lane_change_stage_info_.accident_back;
  lane_change_decider_output.accident_ahead =
      lane_change_stage_info_.accident_ahead;
  lane_change_decider_output.close_to_accident =
      lane_change_stage_info_.close_to_accident;
  lane_change_decider_output.should_premove =
      lane_change_stage_info_.should_premove;
  lane_change_decider_output.should_suspend =
      lane_change_stage_info_.should_suspend;
  lane_change_decider_output.lc_pause = lane_change_stage_info_.lc_pause;
  lane_change_decider_output.lc_pause_id = lane_change_stage_info_.lc_pause_id;
  lane_change_decider_output.tr_pause_l = lane_change_stage_info_.tr_pause_l;
  lane_change_decider_output.tr_pause_s = lane_change_stage_info_.tr_pause_s;
  lane_change_decider_output.lc_request = lc_req_mgr_->request();
  lane_change_decider_output.lc_request_source = lc_req_mgr_->request_source();
  lane_change_decider_output.act_request_source =
      lc_req_mgr_->act_request_source();
  lane_change_decider_output.lc_turn_light = lc_req_mgr_->turn_signal();
  lane_change_decider_output.near_cars_target = near_cars_target_;
  lane_change_decider_output.near_cars_origin = near_cars_origin_;
  lane_change_decider_output.lc_invalid_track = lc_invalid_track_;
  lane_change_decider_output.lc_back_track = lc_back_track_;
  lane_change_decider_output.lc_valid_cnt = lc_valid_cnt_;
  lane_change_decider_output.lc_back_cnt = lc_back_cnt_;
  lane_change_decider_output.start_move_dist_lane = start_move_dist_lane_;
  int merge_lane_virtual_id = lane_change_decider_output.fix_lane_virtual_id;
  lane_change_decider_output.is_merge_region =
      IsMergeRegion(&merge_lane_virtual_id);
  lane_change_decider_output.merge_lane_virtual_id = merge_lane_virtual_id;
  JSON_DEBUG_VALUE("is_merge_region",
                   lane_change_decider_output.is_merge_region);
  JSON_DEBUG_VALUE("merge_lane_virtual_id", merge_lane_virtual_id);
  if (lane_change_decider_output.is_merge_region) {
    std::vector <Point2D> merge_point_list;
    merge_point_list.resize(2);
    CalculateMergePoint(&merge_point_list,lane_change_decider_output.merge_lane_virtual_id);
    lane_change_decider_output.merge_point = merge_point_list[0];
    lane_change_decider_output.boundary_merge_point = merge_point_list[1];
  } else {
    const auto& ego_stete = session_->environmental_model().get_ego_state_manager();
    Point2D ego_point = {ego_stete->planning_init_point().x, ego_stete->planning_init_point().y};
    lane_change_decider_output.merge_point = ego_point;
    lane_change_decider_output.boundary_merge_point = ego_point;
  }
  JSON_DEBUG_VALUE(
      "macroeconomic_decider_merge_point_x",
      lane_change_decider_output.merge_point.x);
  JSON_DEBUG_VALUE(
      "macroeconomic_decider_merge_point_y",
      lane_change_decider_output.merge_point.y);
  JSON_DEBUG_VALUE(
      "boundary_line_merge_point_x",
      lane_change_decider_output.boundary_merge_point.x);
  JSON_DEBUG_VALUE(
      "boundary_line_merge_point_y",
      lane_change_decider_output.boundary_merge_point.y);
  GenerateTurnSignalForSplitRegion();
  lane_change_decider_output.dir_turn_signal_road_to_ramp =
      road_to_ramp_turn_signal_;
  lane_change_decider_output.int_request_cancel_reason =
      lc_req_mgr_->int_request_cancel_reason();

  lane_change_decider_output.ilc_virtual_req =
      lc_req_mgr_->get_ilc_virtual_request();
}
void LaneChangeStateMachineManager::CalculateSideGapFeasible(
    const std::vector<TrackedObject> &vec_side_obstacles,
    LaneChangeStageInfo *const lc_state_info) {
  const double v_ego =
      session_->environmental_model().get_ego_state_manager()->ego_v();
  const auto &vel_sequence = session_->planning_context()
                                 .lateral_behavior_planner_output()
                                 .vel_sequence;
  const auto &reference_path_manager =
      session_->mutable_environmental_model()->get_reference_path_manager();
  const auto &target_reference_path =
      reference_path_manager->get_reference_path_by_lane(
          lc_req_mgr_->target_lane_virtual_id());
  const auto &target_lane_frenet_ego_state =
      target_reference_path->get_frenet_ego_state();
  const auto &target_lane =
      session_->environmental_model()
          .get_virtual_lane_manager()
          ->get_lane_with_virtual_id(lc_req_mgr_->target_lane_virtual_id());
  double dist_mline = std::abs(std::fabs(target_lane_frenet_ego_state.l()) -
                               target_lane->width() / 2);
  double t_reaction = (dist_mline == DBL_MAX) ? 0.5 : 0.5 * dist_mline / 1.8;
  double mss = 0.0;
  double mss_t = 0.0;
  double safety_dist = v_ego * v_ego * 0.02 + 2.0;

  for (auto &tr : vec_side_obstacles) {
    if (tr.type == 2 || tr.type == 3 || tr.type == 4) {
      safety_dist = v_ego * v_ego * 0.015 + 2.0;
    }
    //(1)相对速度大于100
    if (tr.d_rel < safety_dist && tr.d_rel > -5.0 - safety_dist &&
        tr.v_rel > 100.0) {
      lc_invalid_track_.set_value(tr.track_id, tr.d_rel, tr.v_rel);
      lc_state_info->gap_insertable = false;
      lc_state_info->lc_invalid_reason = "side view invalid";
      return;
    }
    // （2）相对速度小于100，且纵向上在自车后方5米以内（相当于一个车的长度）
    if (tr.v_rel < 100.0 && tr.d_rel > -5.0) {
      // 根据相对速度大于0，或者小于0计算mss
      if (tr.v_rel < 0) {
        mss = tr.v_rel * tr.v_rel / 2 + safety_dist;
        mss_t = mss;

        if (v_ego + tr.v_rel < 1) {
          mss_t = -3.5;
        }
        if (tr.d_rel < 0. && v_ego < 3) {
          mss = -3.5 + tr.v_rel * tr.v_rel / 2;
        }
      } else {
        mss = -tr.v_rel * tr.v_rel / 2 + safety_dist;
        if (tr.d_rel < 0. && v_ego < 3) {
          mss = -3.5 - tr.v_rel * tr.v_rel / 2;
        }
      }
      std::array<double, 2> xp{0, 3.5};
      std::array<double, 2> fp{1, 0.7};
      if (((tr.d_rel < interp(tr.v_rel, xp, fp) * safety_dist ||
            tr.d_rel < mss || (mss_t != mss && mss_t > -3.5)) &&
           (tr.d_rel >= 0 || v_ego >= 3)) ||
          (tr.d_rel < 0 && tr.d_rel >= mss && v_ego < 3)) {
        lc_invalid_track_.set_value(tr.track_id, tr.d_rel, tr.v_rel);
        lc_state_info->gap_insertable = false;
        lc_state_info->lc_invalid_reason = "side view invalid";
        return;
      }
    }
    //(3) 相对速度小于100，纵向上在自车后方5米以外
    if (tr.v_rel < 100.0 && tr.d_rel <= -5.0) {
      // 根据自车车速小于17m/s，自车速度大于17m/s分别计算mss
      if (v_ego < 17 && vel_sequence.size() > 1) {
        double temp1 = tr.v_rel;
        double temp2 = tr.v_rel - (vel_sequence[5] - v_ego);
        std::array<double, 5> xp1{-10., -2.5, 0., 3., 5.};
        std::array<double, 5> fp1{5., 2.5, 1.5, 0.5, 0.3};
        double a = interp(temp2, xp1, fp1);
        int sign = temp2 < 0 ? -1 : 1;
        mss =
            std::min(std::max(temp1 * t_reaction +
                                  sign * temp2 * temp2 / (2. * a) + safety_dist,
                              3.0),
                     120.0);
        if (tr.type == 2 || tr.type == 3 || tr.type == 4) {
          mss = std::min(
              std::max(temp1 * (t_reaction + 1) +
                           sign * temp2 * temp2 / (2 * a) + safety_dist,
                       3.0),
              120.0);
        }
        for (size_t i = 0; i + 5 < vel_sequence.size(); i++) {
          mss_t = tr.d_rel +
                  std::max(tr.v_rel - (vel_sequence[i] - v_ego) / 2.0, 0.0) *
                      0.1 * i;
          if (((mss_t > -7.0 || tr.d_rel > -5.0 - mss) && tr.v_lead > 1) ||
              (tr.v_lead <= 1 && tr.d_rel > -5.0)) {
            lc_invalid_track_.set_value(tr.track_id, tr.d_rel, tr.v_rel);
            lc_state_info->gap_insertable = false;
            lc_state_info->lc_invalid_reason = "side view invalid";
            return;
          }
        }
      } else {
        double temp = tr.v_rel;
        std::array<double, 5> xp1{-10., -2.5, 0., 3., 5.};
        std::array<double, 5> fp1{5., 2.5, 1.5, 0.5, 0.3};
        double a = interp(temp, xp1, fp1);
        int sign = temp < 0 ? -1 : 1;
        mss = std::min(std::max(temp * t_reaction +
                                    sign * temp * temp / (2 * a) + safety_dist,
                                3.0),
                       120.0);
        if (tr.type == 2 || tr.type == 3 || tr.type == 4) {
          mss =
              std::min(std::max(temp * (t_reaction + 1) +
                                    sign * temp * temp / (2 * a) + safety_dist,
                                3.0),
                       120.0);
        }
      }
      if (tr.v_rel > 0.0 && tr.d_rel > -2 * (5.0 + mss)) {
        lc_state_info->side_approach = true;
      }
      if (tr.v_rel > 0.0 && tr.d_rel > -4 * (5.0 + mss)) {
        lc_state_info->should_premove = true;
      }
      if ((tr.d_rel > -5.0 - mss && tr.v_lead > 1) ||
          (tr.v_lead <= 1 && tr.d_rel > -5.0)) {
        lc_invalid_track_.set_value(tr.track_id, tr.d_rel, tr.v_rel);
        lc_state_info->gap_insertable = false;
        lc_state_info->lc_invalid_reason = "side view invalid";
        return;
      }
    }
  }
}
void LaneChangeStateMachineManager::CalculateFrontGapFeasible(
    const std::vector<TrackedObject> &vec_front_obstacles,
    LaneChangeStageInfo *const lc_state_info) {
  const double v_ego =
      session_->environmental_model().get_ego_state_manager()->ego_v();
  const auto &vel_sequence = session_->planning_context()
                                 .lateral_behavior_planner_output()
                                 .vel_sequence;
  double mss = 0.0;
  double mss_t = 0.0;
  double safety_dist = v_ego * v_ego * 0.02 + 2.0;
  for (auto &tr : vec_front_obstacles) {
    // (1)如果相对距离在自车5米后，则跳过
    if (tr.d_rel <= -5.0) {
      continue;
    }
    std::array<double, 5> a_dec_v{2.0, 1.5, 1.3, 1.2, 1.0};
    std::array<double, 5> a_ace_v{0.3, 0.6, 0.8, 1.0, 1.5};
    std::array<double, 5> v_ego_bp{0, 10, 15, 20, 30};
    // 插值计算当前速度下的减速度a_dflc、加速度a_aflc
    double a_dflc = interp(v_ego, v_ego_bp, a_dec_v);
    double a_aflc = interp(v_ego, v_ego_bp, a_ace_v);
    // (2)如果相对速度小于0，即前方的障碍物车辆比自车慢
    if (tr.v_rel < 0.0) {
      if (v_ego < 17 && vel_sequence.size() > 6) {
        double temp = std::min(tr.v_rel - vel_sequence[5] + v_ego, 0.0);
        mss = temp * temp / (2 * a_dflc) + safety_dist;
        for (size_t i = 0; i + 5 < vel_sequence.size(); i++) {
          mss_t =
              tr.d_rel +
              std::min(tr.v_rel - (vel_sequence[i] - v_ego) / 2, 0.0) * 0.1 * i;
          if (mss_t < 2.0 || tr.d_rel < mss) {
            lc_invalid_track_.set_value(tr.track_id, tr.d_rel, tr.v_rel);
            lc_state_info->gap_insertable = false;
            lc_state_info->lc_invalid_reason = "front view invalid";
            return;
          }
        }
      } else {
        mss = tr.v_rel * tr.v_rel / (2 * a_dflc) + safety_dist;
      }
    }
    // (3)相对速度大于等于0，即前方的障碍物车辆速度比自车要快或者相等
    if (tr.v_rel >= 0.0) {
      if (v_ego < 17 && vel_sequence.size() > 1) {
        double temp = tr.v_rel - vel_sequence[5] + v_ego;
        mss = -temp * temp / (2 * a_aflc) + safety_dist;

        for (size_t i = 0; i + 5 < vel_sequence.size(); i++) {
          mss_t =
              tr.d_rel + (tr.v_rel - (vel_sequence[i] - v_ego) / 2) * 0.1 * i;

          if (mss_t < 2.0 || tr.d_rel < mss) {
            lc_invalid_track_.set_value(tr.track_id, tr.d_rel, tr.v_rel);
            lc_state_info->gap_insertable = false;
            lc_state_info->lc_invalid_reason = "front view invalid";
            return;
          }
        }
      } else {
        mss = std::pow(std::max(2 - tr.v_rel, 0.0), 2) / (2 * a_aflc) +
              safety_dist;
      }
    }

    std::array<double, 2> xp{0, 3.5};
    std::array<double, 2> fp{1, 0.7};
    // 该值表示前车速度越大，那么变道安全距离可以在原来基础上更小一点
    const double real_safety_dist = interp(tr.v_rel, xp, fp) * safety_dist;
    if (tr.d_rel < real_safety_dist || tr.d_rel < mss) {
      lc_invalid_track_.set_value(tr.track_id, tr.d_rel, tr.v_rel);
      lc_state_info->gap_insertable = false;
      lc_state_info->lc_invalid_reason = "front view invalid";
      return;
    }
  }
}
void LaneChangeStateMachineManager::CalculateSideAreaIfNeedBack(
    const std::vector<TrackedObject> &vec_side_obstacles,
    const RequestType &direction, LaneChangeStageInfo *const lc_state_info) {
  const auto &ego_state =
      session_->environmental_model().get_ego_state_manager();
  const double car_width = 2.2;  // TODO(fengwang31):load config
  const double lane_width =
      3.8;  // TODO(fengwang31):use fix_lane->get_lane_width()
  const double v_ego = ego_state->ego_v();
  const auto &virtual_lane_manager =
      session_->mutable_environmental_model()->get_virtual_lane_manager();
  const auto &fix_lane = virtual_lane_manager->get_lane_with_virtual_id(
      lc_req_mgr_->target_lane_virtual_id());
  const auto &reference_path_manager =
      session_->mutable_environmental_model()->get_reference_path_manager();
  const auto &fix_reference_path =
      reference_path_manager->get_reference_path_by_lane(
          lc_lane_mgr_->fix_lane_virtual_id());
  const auto &target_reference_path =
      reference_path_manager->get_reference_path_by_lane(
          lc_lane_mgr_->target_lane_virtual_id());
  const auto &frenet_ego_state = fix_reference_path->get_frenet_ego_state();
  const auto &target_lane_frenet_ego_state =
      target_reference_path->get_frenet_ego_state();
  const double dist_mline = std::fabs(target_lane_frenet_ego_state.l()) - 1.8;
  const double t_reaction =
      (dist_mline == DBL_MAX) ? 1.0 : 1.0 * dist_mline / 1.8;
  const double l_ego = frenet_ego_state.l();
  // 根据车道边界的曲率插值计算横向移动阈值 move_thre，曲率越大，move越大
  std::array<double, 3> xp_heading{0, 0.5, 1};
  std::array<double, 3> fp_l{0, 0.4, 0.6};
  double c1 = 0;
  if (direction == LEFT_CHANGE) {
    c1 = fix_lane->get_right_lane_boundary().poly_coefficient[1];
  } else if (direction == RIGHT_CHANGE) {
    c1 = fix_lane->get_left_lane_boundary().poly_coefficient[1];
  }
  double move_thre =
      std::max(interp(std::fabs(c1), xp_heading, fp_l) +
                   ego_state->ego_v() * config_.lc_t_actuator_delay,
               config_.lc_back_available_thr);

  for (auto &tr : vec_side_obstacles) {
    auto &x = tr.trajectory.x;
    auto &y = tr.trajectory.y;
    std::vector<double> ego_x;
    std::vector<double> ego_y;
    std::vector<double> ego_speed;
    std::vector<double> ego_yaw;
    double ego_fx = std::cos(ego_state->heading_angle());
    double ego_fy = std::sin(ego_state->heading_angle());
    double ego_lx = -ego_fy;
    double ego_ly = ego_fx;
    double theta = 0.0;
    double send = 0.0;
    double lend = 0.0;
    int end_idx = 0;
    if (tr.trajectory.intersection == 0) {
      for (int i = 0; i < x.size(); i++) {
        double dx = x[i] - ego_state->ego_pose().x;
        double dy = y[i] - ego_state->ego_pose().y;

        double rel_x = dx * ego_fx + dy * ego_fy;
        double rel_y = dx * ego_lx + dy * ego_ly;
        ego_x.push_back(rel_x);
        ego_y.push_back(rel_y);
      }
      int end_range = min((int)ego_x.size() - 1, 25);
      for (int i = end_range; i >= 0; i--) {
        if (ego_x[i] < 0 || ego_x[i] > 80) {
          continue;
        }

        end_idx = i;
        break;
      }
    }
    double safety_dist = v_ego * 0.2 + 2;
    double mss = 0.0;
    double mss_t = 0.0;
    if (lc_state_info->lc_should_back == false &&
        ((direction == LEFT_CHANGE &&
          lend < car_width + 0.3 - lane_width / 2 + tr.width / 2) ||
         (direction == RIGHT_CHANGE &&
          lend > -(car_width + 0.3) + lane_width / 2 - tr.width / 2) ||
         tr.trajectory.intersection != 0)) {
      if (tr.type == 2 || tr.type == 3 || tr.type == 4) {
        safety_dist = v_ego * 0.1 + 1;
      }
      //(1)后车相对车速大于100m/s
      if (tr.d_rel < safety_dist && tr.d_rel > -5.0 - safety_dist &&
          tr.v_rel > 100.0) {
        lc_state_info->lc_should_back = true;
        lc_state_info->lc_back_reason = "side view back";
        lc_back_track_.set_value(tr.track_id, tr.d_rel, tr.v_rel);
        lc_state_info->lc_pause_id = tr.track_id;
      }
      //(2) 后车相对车速小于100m/s
      if (tr.v_rel < 100.0) {
        // (2.1) 相对距离在自车后方一个车长范围内
        if (tr.d_rel > -5.0) {
          if (tr.v_rel < 0) {
            mss = tr.v_rel * tr.v_rel / 4 + safety_dist;
            mss_t = mss;

            if (v_ego + tr.v_rel > 1) {
              mss_t = -2;
            }
            if (tr.d_rel < 0. && tr.v_lead < 1) {
              mss = -2 + tr.v_rel * tr.v_rel / 2;
            }
          } else {
            mss = -tr.v_rel * tr.v_rel / 4 + safety_dist;
          }

          std::array<double, 2> xp{0, 3.5};
          std::array<double, 2> fp{1, 0.7};

          if (((tr.d_rel < 0.5 * interp(tr.v_rel, xp, fp) * safety_dist ||
                tr.d_rel < mss || (mss_t != mss && mss_t > -2)) &&
               tr.v_lead >= 1) ||
              (tr.d_rel > mss && tr.v_lead < 1)) {
            lc_state_info->lc_should_back = true;
            lc_state_info->lc_back_reason = "side view back";
            lc_back_track_.set_value(tr.track_id, tr.d_rel, tr.v_rel);
            if (tr.d_rel < 0 && tr.d_rel > lc_state_info->tr_pause_s) {
              lc_state_info->lc_pause_id = tr.track_id;
              lc_state_info->tr_pause_s = tr.d_rel;
              lc_state_info->tr_pause_l = tr.d_center_cpath;
              lc_state_info->tr_pause_dv = tr.v_rel;
            }
          }
        }
        // (2.2)
        // 相对距离在自车后方一个车长范围外,而且后车车速比自车快，若是比自车慢，那么可以不用处理
        if (tr.d_rel <= -5.0 && tr.v_rel >= 0) {
          double temp = std::max(tr.v_rel, 0.0);
          double lat_thre = car_width + 0.6 - lane_width / 2;
          std::array<double, 3> xp1{0., 3., 5.};
          std::array<double, 3> fp1{2, 1.5, 1.};
          double a = interp(temp, xp1, fp1);
          if (std::fabs(virtual_lane_manager->lc_map_decision(fix_lane)) == 1) {
            mss = tr.v_rel * t_reaction / 2 * 0.7 +
                  temp * temp / (2 * a) * 0.7 + safety_dist -
                  2 / (tr.v_rel + 1);

            if (tr.type == 2 || tr.type == 3 || tr.type == 4) {
              mss = tr.v_rel * (t_reaction + 2) / 2 * 0.7 +
                    temp * temp / 2 * 0.7 + safety_dist * 1.1 -
                    2 / (tr.v_rel + 1);
            }
          } else {
            mss = tr.v_rel * t_reaction / 2 + temp * temp / (2 * a) +
                  safety_dist - 2 / (tr.v_rel + 1);
            if (tr.type == 2 || tr.type == 3 || tr.type == 4) {
              mss = tr.v_rel * (t_reaction + 2) / 2 + temp * temp / 2 +
                    safety_dist - 2 / (tr.v_rel + 1);
            }
          }
          //(fengwang31):由于目前障碍物的横向速度不准确，所以这部分涉及横向速度的计算先暂时注掉
          // if (tr.d_rel >
          //         -5.0 -
          //             (mss -
          //               std::max(
          //                   std::max(
          //                       (std::max(l_ego - tr.d_max_cpath - 1.6,
          //                                 tr.d_min_cpath - l_ego - 1.6)),
          //                       0.0) /
          //                       std::max((-tr.v_lat + 0.7), 0.1) * tr.v_rel,
          //                   0.0)) &&
          //     ((tr.d_max_cpath >= -lat_thre &&
          //       tr.d_min_cpath <= lat_thre) ||
          //       (tr.d_max_cpath -
          //           tr.v_lat * std::min(tr.d_rel + 5 / tr.v_rel, 4.) >=
          //       -lat_thre) ||
          //       (tr.d_min_cpath +
          //           tr.v_lat * std::min(tr.d_rel + 5 / tr.v_rel, 4.) <=
          //       lat_thre)))
          if (tr.d_rel > -5 - mss) {
            lc_state_info->lc_should_back = true;
            lc_state_info->lc_back_reason = "side view back";
            lc_back_track_.set_value(tr.track_id, tr.d_rel, tr.v_rel);
            if (tr.d_rel < 0 && tr.d_rel > lc_state_info->tr_pause_s) {
              lc_state_info->lc_pause_id = tr.track_id;
              lc_state_info->tr_pause_s = tr.d_rel;
              lc_state_info->tr_pause_l = tr.d_center_cpath;
              lc_state_info->tr_pause_dv = tr.v_rel;
            }
          }
        }
      }
    } else {
      break;
    }
  }
  // 确定车道的宽度
  double left_lane_width = 3.8;
  double right_lane_width = 3.8;
  auto left_lane = virtual_lane_manager->get_left_lane();
  if (left_lane != nullptr) {
    for (auto &p : left_lane->lane_points()) {
      if (p.car_point.x >= 0 && p.car_point.x <= 20 && !p.is_in_intersection) {
        left_lane_width = p.lane_width;
        if (left_lane_width < 100) {
          break;
        }
      }
    }
  }

  auto right_lane = virtual_lane_manager->get_right_lane();
  if (right_lane != nullptr) {
    for (auto &p : right_lane->lane_points()) {
      if (p.car_point.x >= 0 && p.car_point.x <= 20 && !p.is_in_intersection) {
        right_lane_width = p.lane_width;
        if (right_lane_width < 100) {
          break;
        }
      }
    }
  }
  const double distance = frenet_ego_state.l();
  const double current_lane_virtual_id =
      virtual_lane_manager->current_lane_virtual_id();
  const double pause_ttc = 2.0;
  const double pause_v_rel = 2.0;
  if ((direction == LEFT_CHANGE &&
       distance > -left_lane_width / 2 - move_thre) ||
      (direction == RIGHT_CHANGE &&
       distance < right_lane_width / 2 + move_thre)) {
    if (lc_state_info->lc_should_back &&
        lc_state_info->tr_pause_dv > pause_v_rel &&
        -lc_state_info->tr_pause_s / lc_state_info->tr_pause_dv < pause_ttc &&
        ((direction == LEFT_CHANGE &&
          lc_state_info->tr_pause_l - distance > 0.5) ||
         (direction == RIGHT_CHANGE &&
          distance - lc_state_info->tr_pause_l > 0.5))) {
      lc_state_info->lc_pause = true;
    }
    if (lc_state_info->lc_should_back) {
      behavior_suspend_ = true;
      suspend_obs_.push_back(lc_back_track_.track_id);
    }
    lc_state_info->lc_should_back = false;
    lc_state_info->lc_back_reason = "exceed move_thre, do not back";
  }
}
void LaneChangeStateMachineManager::CalculateFrontAreaIfNeedBack(
    const std::vector<TrackedObject> &vec_front_obstacles,
    const RequestType &direction, LaneChangeStageInfo *const lc_state_info) {
  const auto &virtual_lane_manager =
      session_->mutable_environmental_model()->get_virtual_lane_manager();
  const auto &fix_lane = virtual_lane_manager->get_lane_with_virtual_id(
      lc_req_mgr_->target_lane_virtual_id());
  const auto &reference_path_manager =
      session_->mutable_environmental_model()->get_reference_path_manager();
  const auto &fix_reference_path =
      reference_path_manager->get_reference_path_by_lane(
          lc_lane_mgr_->fix_lane_virtual_id());
  const auto &target_reference_path =
      reference_path_manager->get_reference_path_by_lane(
          lc_lane_mgr_->target_lane_virtual_id());
  const auto &frenet_ego_state = fix_reference_path->get_frenet_ego_state();
  const double l_ego = frenet_ego_state.l();
  const double v_ego =
      session_->environmental_model().get_ego_state_manager()->ego_v();
  std::array<double, 2> a_dec_v{3.0, 2.0};
  std::array<double, 2> v_ego_bp{6, 20};
  double a_dflc = interp(v_ego, v_ego_bp, a_dec_v);
  for (auto &tr : vec_front_obstacles) {
    double safety_dist = v_ego * 0.2 + 2;
    double mss = 0.0;
    if (tr.type == 2 || tr.type == 3 || tr.type == 4) {
      safety_dist = v_ego * 0.1 + 1;
    }
    if (tr.v_rel < 0) {
      mss = tr.v_rel * tr.v_rel / (2 * a_dflc) + safety_dist;
    } else {
      mss = std::max(-tr.v_rel * tr.v_rel / 4 + safety_dist, 2.0);
    }
    double lat_condi = std::max((std::max(l_ego - tr.d_max_cpath - 1.6,
                                          tr.d_min_cpath - l_ego - 1.6)),
                                0.0) /
                       (std::max(-tr.v_lat + 0.7, 0.3));
    if ((lat_condi < 1.5 && tr.d_rel < 1.0) ||
        (lat_condi <= 1.5 &&
         tr.d_rel < 0.8 * (mss - std::max(lat_condi * tr.v_rel, 0.0)))) {
      lc_state_info->lc_should_back = true;
      lc_state_info->lc_back_reason = "front view back";
      lc_back_track_.set_value(tr.track_id, tr.d_rel, tr.v_rel);
    }
  }
}

void LaneChangeStateMachineManager::ResetStateMachine() {
  transition_info_.Rest();
  lc_lane_mgr_->reset_lc_lanes(transition_info_.lane_change_status);
  lc_req_mgr_->FinishRequest();
  lane_change_stage_info_.Reset();
  lc_timer_.Reset();

  pre_ego_l_ = 0;
  lc_valid_cnt_ = 0;
  lc_back_cnt_ = 0;
  lc_invalid_track_.reset();
  lc_back_track_.reset();
  near_cars_target_.clear();
  near_cars_origin_.clear();
  must_change_lane_ = false;
  start_move_dist_lane_ = 0;
  behavior_suspend_ = false;
  suspend_obs_.clear();
}

bool LaneChangeStateMachineManager::TimeOut(const bool &trigger,
                                            bool *is_start_count,
                                            double *time_count,
                                            const double &threshold) {
  if (trigger) {
    if (!(*is_start_count)) {
      *is_start_count = true;
      *time_count = IflyTime::Now_ms();
    }

    if (IflyTime::Now_ms() - *time_count > threshold) {
      return true;
    }
  } else {
    *is_start_count = false;
    *time_count = IflyTime::Now_ms();
  }
  return false;
}

void LaneChangeStateMachineManager::UpdateStateMachineDebugInfo() {
  const auto &lane_change_decider_output =
      session_->planning_context().lane_change_decider_output();
  auto &debug_info_manager = DebugInfoManager::GetInstance();
  auto &planning_debug_data = debug_info_manager.GetDebugInfoPb();
  auto lat_behavior_common = planning_debug_data->mutable_lat_behavior_common();
  lat_behavior_common->set_current_state(lane_change_decider_output.curr_state);
  lat_behavior_common->set_lc_invalid_obj_id(
      lane_change_decider_output.lc_invalid_track.track_id);
  lat_behavior_common->set_lc_back_obj_id(
      lane_change_decider_output.lc_back_track.track_id);
  lat_behavior_common->set_lc_invalid_reason(
      lane_change_decider_output.lc_invalid_reason);
  lat_behavior_common->set_lc_back_reason(
      lane_change_decider_output.lc_back_reason);
  lat_behavior_common->set_lc_back_invalid_reason(
      lane_change_decider_output.lc_back_invalid_reason);
  lat_behavior_common->mutable_near_car_ids_origin()->Clear();
  for (auto &near_car_origin : lane_change_decider_output.near_cars_origin) {
    lat_behavior_common->add_near_car_ids_origin(near_car_origin.track_id);
  }
  lat_behavior_common->mutable_near_car_ids_target()->Clear();
  for (auto &near_car_target : lane_change_decider_output.near_cars_target) {
    lat_behavior_common->add_near_car_ids_target(near_car_target.track_id);
  }
  lat_behavior_common->set_is_faster_left_lane(
      lane_change_decider_output.left_is_faster);
  lat_behavior_common->set_is_faster_right_lane(
      lane_change_decider_output.right_is_faster);
  lat_behavior_common->mutable_left_alc_car_ids()->Clear();
  for (auto id : lane_change_decider_output.left_alc_car) {
    lat_behavior_common->add_left_alc_car_ids(id);
  }
  lat_behavior_common->mutable_right_alc_car_ids()->Clear();
  for (auto id : lane_change_decider_output.right_alc_car) {
    lat_behavior_common->add_right_alc_car_ids(id);
  }
  lat_behavior_common->set_is_forbid_left_alc_car(
      lane_change_decider_output.neg_left_alc_car);
  lat_behavior_common->set_is_forbid_right_alc_car(
      lane_change_decider_output.neg_right_alc_car);
  lat_behavior_common->set_fix_lane_virtual_id(
      lane_change_decider_output.fix_lane_virtual_id);
  lat_behavior_common->set_origin_lane_virtual_id(
      lane_change_decider_output.origin_lane_virtual_id);
  lat_behavior_common->set_target_lane_virtual_id(
      lane_change_decider_output.target_lane_virtual_id);

  lat_behavior_common->set_turn_light(lane_change_decider_output.turn_light);
  lat_behavior_common->set_map_turn_light(
      lane_change_decider_output.map_turn_light);
  lat_behavior_common->set_lc_request(lane_change_decider_output.lc_request);
  lat_behavior_common->set_lc_request_source(
      lane_change_decider_output.lc_request_source);
  lat_behavior_common->set_lc_turn_light(
      lane_change_decider_output.lc_turn_light);
  lat_behavior_common->set_act_request_source(
      lane_change_decider_output.act_request_source);

  lat_behavior_common->set_is_lc_valid(lane_change_decider_output.is_lc_valid);
  lat_behavior_common->set_lc_valid_cnt(
      lane_change_decider_output.lc_valid_cnt);
  lat_behavior_common->set_lc_back_cnt(lane_change_decider_output.lc_back_cnt);
}

void LaneChangeStateMachineManager::GenerateTurnSignalForSplitRegion() {
  // fengwang31:hack temp split lane nums is two
  //生成转向灯信号的条件：同时满足以下5个条件:
  // 1、存在两条lane的relative_id为0；
  // 2、ramp的方向即转向灯信号的方向；
  // 3、当前自车还在高速路上(NOA模式下);
  // 4、当前没有生成变道请求。
  // 5、当前还在主路上，即不在匝道上。
  const auto virtual_lane_manager =
      session_->environmental_model().get_virtual_lane_manager();
  if (virtual_lane_manager == nullptr) {
    road_to_ramp_turn_signal_ = RAMP_NONE;
    return;
  }
  const auto reference_path_manager =
      session_->environmental_model().get_reference_path_manager();
  if (reference_path_manager == nullptr) {
    road_to_ramp_turn_signal_ = RAMP_NONE;
    return;
  }
  int origin_relative_id_zero_nums =
      virtual_lane_manager->origin_relative_id_zero_nums();
  bool is_on_highway = virtual_lane_manager->is_on_highway();
  JSON_DEBUG_VALUE("origin_relative_id_zero_nums",
                   origin_relative_id_zero_nums);
  // overlap_lane_virtual_id_ = virtual_lane_manager->current_lane_virtual_id();
  bool is_off_turn_signal = false;
  if (origin_relative_id_zero_nums > 1) {
    RampDirection ramp_direction = RAMP_NONE;
    if (IsSplitRegion(&ramp_direction)) {
      if (is_on_highway &&
          transition_info_.lane_change_status == kLaneKeeping) {
        if (ramp_direction == RAMP_ON_RIGHT) {
          road_to_ramp_turn_signal_ = RAMP_ON_RIGHT;
        } else if (ramp_direction == RAMP_ON_LEFT) {
          road_to_ramp_turn_signal_ = RAMP_ON_LEFT;
        } else {
          road_to_ramp_turn_signal_ = RAMP_NONE;
        }
      }
    }
  } else {
    if (road_to_ramp_turn_signal_ != RAMP_NONE &&
        IsOffTurnLight(road_to_ramp_turn_signal_)) {
      road_to_ramp_turn_signal_ = RAMP_NONE;
      is_off_turn_signal = true;
    }
  }
  const auto distance_to_toll_station = virtual_lane_manager->get_distance_to_toll_station();
  const auto distance_to_route_end = virtual_lane_manager->get_distance_to_route_end();
  //接近收费站或者终点时，抑制分流点的判断
  if (distance_to_toll_station < 400 ||
      distance_to_route_end < 400) {
    road_to_ramp_turn_signal_ = RAMP_NONE;
  }
  JSON_DEBUG_VALUE("is_off_turn_signal", static_cast<int>(is_off_turn_signal));
  JSON_DEBUG_VALUE("road_to_ramp_turn_signal",
                   static_cast<int>(road_to_ramp_turn_signal_));
}

bool LaneChangeStateMachineManager::IsSplitRegion(
    RampDirection *ramp_direction) {
  const auto virtual_lane_manager =
      session_->environmental_model().get_virtual_lane_manager();
  const auto reference_path_manager =
      session_->environmental_model().get_reference_path_manager();
  const auto left_lane = virtual_lane_manager->get_left_lane();
  const auto right_lane = virtual_lane_manager->get_right_lane();
  const auto current_lane = virtual_lane_manager->get_current_lane();
  const auto current_reference_path =
      reference_path_manager->get_reference_path_by_current_lane();
  const double lane_lat_diff_threshold = 3.8 - 0.8;
  std::shared_ptr<ReferencePath> left_reference_path;
  std::shared_ptr<ReferencePath> right_reference_path;
  double left_ego_l = NL_NMAX;
  double right_ego_l = NL_NMAX;
  double ego_l = current_reference_path->get_frenet_ego_state().l();
  double left_lane_width = 0;
  double right_lane_width = 0;

  if (left_lane != nullptr) {
    left_reference_path = reference_path_manager->get_reference_path_by_lane(
        left_lane->get_virtual_id());
    left_lane_width = left_lane->width();
    if (left_reference_path != nullptr) {
      const auto left_frenet_ego_state =
          left_reference_path->get_frenet_ego_state();
      left_ego_l = left_frenet_ego_state.l();
    }
  }
  if (right_lane != nullptr) {
    right_lane_width = right_lane->width();
    right_reference_path = reference_path_manager->get_reference_path_by_lane(
        right_lane->get_virtual_id());
    if (right_reference_path != nullptr) {
      const auto right_frenet_ego_state =
          right_reference_path->get_frenet_ego_state();
      right_ego_l = right_frenet_ego_state.l();
    }
  }
  // 目前只有一分二的场景，后续一分二以上的场景需要进一步处理  TODO(fengwang31)
  JSON_DEBUG_VALUE("left_lat_err:", std::abs(left_ego_l - ego_l));
  JSON_DEBUG_VALUE("right_lat_err:", std::abs(right_ego_l - ego_l));
  bool is_overlap = false;
  if (std::abs(left_ego_l - ego_l) < left_lane_width / 2) {
    overlap_lane_virtual_id_ = left_lane->get_virtual_id();
    is_overlap = true;
  } else if (std::abs(right_ego_l - ego_l) < right_lane_width / 2) {
    overlap_lane_virtual_id_ = right_lane->get_virtual_id();
    is_overlap = true;
  }
  if (is_overlap) {
    double lat_diff = 0;
    const auto &overlap_reference_path =
        reference_path_manager->get_reference_path_by_lane(
            overlap_lane_virtual_id_);
    if (!overlap_reference_path) {
      return false;
    }
    CalculateLatOffsetOfOverlappedLanes(&lat_diff, overlap_reference_path);
    JSON_DEBUG_VALUE("lat_diff", lat_diff);
    if (std::abs(lat_diff) > lane_lat_diff_threshold) {
      if (lat_diff > 0) {
        *ramp_direction = RAMP_ON_LEFT;
      } else {
        *ramp_direction = RAMP_ON_RIGHT;
      }
      return true;
    }
  }
  return false;
}

void LaneChangeStateMachineManager::CalculateLatOffsetOfOverlappedLanes(
    double *lat_diff, const std::shared_ptr<ReferencePath> reference_path) {
  const auto current_reference_path =
      session_->environmental_model()
          .get_reference_path_manager()
          ->get_reference_path_by_current_lane();
  const double current_lane_front_length =
      current_reference_path->get_frenet_coord()->Length() -
      current_reference_path->get_frenet_ego_state().s();
  const double reference_lane_front_length =
      reference_path->get_frenet_coord()->Length() -
      reference_path->get_frenet_ego_state().s();
  const double length_diff_cur_lane_with_overlap_lane =
      current_lane_front_length - reference_lane_front_length;
  const double length_diff_threshold = 10.0;  // 两条车道线的前方长度差值阈值
  bool is_cur_path_project_to_ref_path = true;
  const auto cur_ref_path_finally_point =
      current_reference_path->get_points().back().path_point;
  std::shared_ptr<KDPath> reference_path_frenet_coordinate =
      reference_path->get_frenet_coord();
  Point2D projection_point = {cur_ref_path_finally_point.x,
                              cur_ref_path_finally_point.y};
  bool is_on_ramp = session_->environmental_model().get_virtual_lane_manager()->is_on_ramp();
  if (is_on_ramp) {
    ReferencePathPoint refpoint = {};
    if (current_reference_path->get_reference_point_by_lon(
        current_reference_path->get_frenet_ego_state().s() + 50,
        refpoint)) {
      projection_point = {refpoint.path_point.x, refpoint.path_point.y};
    }
  } else {
    if (std::abs(length_diff_cur_lane_with_overlap_lane) >
        length_diff_threshold) {
      if (length_diff_cur_lane_with_overlap_lane > length_diff_threshold) {
        is_cur_path_project_to_ref_path = false;
        const auto ref_path_finally_point =
            current_reference_path->get_points().back().path_point;
        projection_point = {ref_path_finally_point.x, ref_path_finally_point.y};
        reference_path_frenet_coordinate =
            current_reference_path->get_frenet_coord();
      }
    } else {
      ReferencePathPoint refpoint = {};
      if (current_reference_path->get_reference_point_by_lon(
          current_reference_path->get_frenet_coord()->Length() -
              length_diff_threshold,
          refpoint)) {
        projection_point = {refpoint.path_point.x, refpoint.path_point.y};
      }
    }
  }

  Point2D frenet_point;
  if (reference_path_frenet_coordinate->XYToSL(projection_point,
                                               frenet_point)) {
    if (is_cur_path_project_to_ref_path) {
      *lat_diff = frenet_point.y;
    } else {
      *lat_diff = -frenet_point.y;
    }
  }
}

bool LaneChangeStateMachineManager::IsOffTurnLight(
    const RampDirection ramp_direction) {
  const auto virtual_lane_manager =
      session_->environmental_model().get_virtual_lane_manager();
  const auto reference_path_manager =
      session_->environmental_model().get_reference_path_manager();
  std::shared_ptr<ReferencePath> overlap_reference_path;
  std::shared_ptr<VirtualLane> overlap_lane;
  if (ramp_direction == RAMP_ON_LEFT) {
    const auto right_lane = virtual_lane_manager->get_right_lane();
    if (!right_lane) {
      return true;
    }
    overlap_lane = right_lane;
    const auto &right_path = reference_path_manager->get_reference_path_by_lane(
        right_lane->get_virtual_id());
    if (!right_path) {
      return true;
    }
    overlap_reference_path = right_path;
  } else if (ramp_direction == RAMP_ON_RIGHT) {
    const auto left_lane = virtual_lane_manager->get_left_lane();
    if (!left_lane) {
      return true;
    }
    overlap_lane = left_lane;
    const auto &left_path = reference_path_manager->get_reference_path_by_lane(
        left_lane->get_virtual_id());
    if (!left_path) {
      return true;
    }
    overlap_reference_path = left_path;
  } else {
    return true;
  }
  if (!overlap_reference_path) {
    return true;
  }
  if (!overlap_lane) {
    return true;
  }
  double overlap_lane_width = overlap_lane->width();
  std::shared_ptr<KDPath> overlap_path_frenet_coordinate =
      overlap_reference_path->get_frenet_coord();

  const auto &ego_vertices_points = session_->environmental_model()
                                        .get_ego_state_manager()
                                        ->polygon()
                                        .points();
  double ego_dis_to_ref_lane = NL_NMAX;
  for (auto ego_vertices_point : ego_vertices_points) {
    Point2D frenet_point;
    Point2D ego_vertices_point_tem = {ego_vertices_point.x(),
                                      ego_vertices_point.y()};
    if (overlap_path_frenet_coordinate->XYToSL(ego_vertices_point_tem,
                                               frenet_point)) {
      if (std::abs(frenet_point.y) < ego_dis_to_ref_lane) {
        ego_dis_to_ref_lane = std::abs(frenet_point.y);
      }
    }
  }
  if (ego_dis_to_ref_lane < overlap_lane_width / 2 ||
      !CheckIfInPerfectLaneKeeping()) {
    return false;
  }
  return true;
}

bool LaneChangeStateMachineManager::IsMergeRegion(int *merge_lane_virtual_id) {
  const auto virtual_lane_manager =
      session_->environmental_model().get_virtual_lane_manager();
  const auto reference_path_manager =
      session_->environmental_model().get_reference_path_manager();
  const auto left_lane = virtual_lane_manager->get_left_lane();
  const auto right_lane = virtual_lane_manager->get_right_lane();
  const auto cur_ref_path =
      reference_path_manager->get_reference_path_by_current_lane();
  std::shared_ptr<ReferencePath> left_reference_path;
  std::shared_ptr<ReferencePath> right_reference_path;
  //目前仅考虑二合一的场景，后续城区城区可能还会有二以上合一的场景
  //判断与左车道是否merge
  if (left_lane != nullptr) {
    left_reference_path =
        reference_path_manager->get_reference_path_by_lane(
            left_lane->get_virtual_id());
  }
  if (left_reference_path != nullptr) {
    if (IsOverlapWithOtherLaneOnEndRegion(left_reference_path, LEFT_DIRECTION)) {
      *merge_lane_virtual_id = left_lane->get_virtual_id();
      return true;
    }
  }

  //判断与右车道是否merge
  if (right_lane != nullptr) {
    right_reference_path =
        reference_path_manager->get_reference_path_by_lane(
            right_lane->get_virtual_id());
  }
  if (right_reference_path != nullptr) {
    if (IsOverlapWithOtherLaneOnEndRegion(right_reference_path,
                                          RIGHT_DIRECTION)) {
      *merge_lane_virtual_id = right_lane->get_virtual_id();
      return true;
    }
  }
  return false;
}

bool LaneChangeStateMachineManager::IsOverlapWithOtherLaneOnEndRegion(
    const std::shared_ptr<ReferencePath> reference_path,
    const RelativeDirection rel_dir) {
  bool lane_end_satisfied_merge_condition = false;
  const double standard_lane_width = 3.8;
  const double cur_lane_lat_diff_threshold = standard_lane_width - 0.8;
  const double end_lane_lat_diff_threshold = standard_lane_width / 4;
  double cur_to_other_lane_end_lat_diff = NL_NMAX;
  double ref_ego_l = NL_NMAX;
  double ref_ego_s = NL_NMAX;
  double cur_ego_l = NL_NMAX;
  double cur_ego_s = NL_NMAX;
  double abs_lat_diff = 0.0;
  const auto reference_path_manager =
      session_->environmental_model().get_reference_path_manager();
  const auto &virtual_lane_manager = session_->environmental_model().get_virtual_lane_manager();
  const auto ref_frenet_ego_state = reference_path->get_frenet_ego_state();
  ref_ego_l = ref_frenet_ego_state.l();
  ref_ego_s = ref_frenet_ego_state.s();
  const auto ref_lane_coord = reference_path->get_frenet_coord();
  const auto cur_reference_path =
      reference_path_manager->get_reference_path_by_current_lane();
  if (cur_reference_path == nullptr) {
    return false;
  }
  //计算当前位置的abs_lat_diff
  cur_ego_l = cur_reference_path->get_frenet_ego_state().l();
  cur_ego_s = cur_reference_path->get_frenet_ego_state().s();
  ReferencePathPoint cur_ref_path_point{};
  cur_reference_path->get_reference_point_by_lon(cur_ego_s, cur_ref_path_point);
  Point2D cur_ref_point = {cur_ref_path_point.path_point.x,
                           cur_ref_path_point.path_point.y};
  Point2D frenet_ref_point;
  if (ref_lane_coord->XYToSL(cur_ref_point, frenet_ref_point)) {
    abs_lat_diff = std::abs(frenet_ref_point.y);
  } else {
    abs_lat_diff = std::abs(ref_ego_l - cur_ego_l);
  }
  const auto cur_lane_coord = cur_reference_path->get_frenet_coord();
  const auto cur_ref_path_end_point = cur_reference_path->get_points().back();
  
  //计算自车前方车道线的长度
  double ego_front_length = CalculateEgoFrontLineLength();
  if (ego_front_length < kEps) {
    return false;
  }
  //计算两条车道起始点的lat_diff
  double start_abs_lat_diff = 0.0;
  Point2D frenet_point_start;
  Point2D cur_ref_path_start_point_temp = {
      cur_reference_path->get_points().begin()->path_point.x,
      cur_reference_path->get_points().begin()->path_point.y};
  Point2D ref_path_start_point_temp = {
      reference_path->get_points().begin()->path_point.x,
      reference_path->get_points().begin()->path_point.y};
  if (ref_lane_coord->XYToSL(cur_ref_path_start_point_temp,
                             frenet_point_start)) {
    start_abs_lat_diff = std::abs(frenet_point_start.y);
  } else {
    if (cur_lane_coord->XYToSL(ref_path_start_point_temp, frenet_point_start)) {
      start_abs_lat_diff = std::abs(frenet_point_start.y);
    }
  }
  //计算两条车道终点的lat_diff
  Point2D cur_ref_path_end_point_temp;
  ego_front_length = std::min(ego_front_length,cur_reference_path->get_frenet_coord()->Length() - cur_ego_s);
  ReferencePathPoint cur_ref_path_point_temp{};
  if (cur_reference_path->get_reference_point_by_lon(cur_ego_s + ego_front_length - 1.0, cur_ref_path_point_temp)) {
    cur_ref_path_end_point_temp = {cur_ref_path_point_temp.path_point.x, cur_ref_path_point_temp.path_point.y};
  } else {
    cur_ref_path_end_point_temp = {cur_ref_path_end_point.path_point.x, cur_ref_path_end_point.path_point.y};
  }
  Point2D ref_path_end_point_temp;
  ReferencePathPoint ref_path_point_temp{};
  if (reference_path->get_reference_point_by_lon(ref_ego_s + ego_front_length - 1.0, ref_path_point_temp)) {
    ref_path_end_point_temp = {ref_path_point_temp.path_point.x, ref_path_point_temp.path_point.y};
  } else {
    ref_path_end_point_temp = {
      reference_path->get_points().back().path_point.x,
      reference_path->get_points().back().path_point.y};
  }

  Point2D frenet_point;
  if (ref_lane_coord->XYToSL(cur_ref_path_end_point_temp, frenet_point)) {
    cur_to_other_lane_end_lat_diff = frenet_point.y;
    if (std::abs(cur_to_other_lane_end_lat_diff) <
               end_lane_lat_diff_threshold) {
      lane_end_satisfied_merge_condition = true;
    } 
  } else {
    if (cur_lane_coord->XYToSL(ref_path_end_point_temp, frenet_point)) {
      cur_to_other_lane_end_lat_diff = frenet_point.y;
      if (std::abs(cur_to_other_lane_end_lat_diff) <
                  end_lane_lat_diff_threshold) {
        lane_end_satisfied_merge_condition = true;
      } 
    }
  }
  if ((abs_lat_diff > cur_lane_lat_diff_threshold ||
       start_abs_lat_diff > cur_lane_lat_diff_threshold) &&
      lane_end_satisfied_merge_condition) {
    std::cout << "is merge region!!!" << std::endl;
    return true;
  }
  //遍历自车向前的点，是否有overlap情况
  const double cur_lane_length = cur_lane_coord->Length();
  const double remaining_length = cur_lane_length - cur_ego_s;
  const double step_length = 5.0;
  const double buffer = 1.0;
  const int calculate_nums = (int)(ego_front_length / step_length - buffer);
  for (int i = 1; i <= calculate_nums; i++) {
    ReferencePathPoint cur_ref_path_point_temp{};
    Point2D frenet_point_temp;
    if (!cur_reference_path->get_reference_point_by_lon(cur_ego_s + i * 5, cur_ref_path_point_temp)) {
      continue;
    }
    Point2D cur_point_temp = {cur_ref_path_point_temp.path_point.x, cur_ref_path_point_temp.path_point.y};
    if (ref_lane_coord->XYToSL(cur_point_temp, frenet_point_temp)) {
      double cur_point_to_other_lane_lat_diff = frenet_point_temp.y;
      if (std::abs(cur_point_to_other_lane_lat_diff) <
                end_lane_lat_diff_threshold) {
        lane_end_satisfied_merge_condition = true;
        break;
      }
    }
  }
  if ((abs_lat_diff > cur_lane_lat_diff_threshold ||
       start_abs_lat_diff > cur_lane_lat_diff_threshold) &&
      lane_end_satisfied_merge_condition) {
    std::cout << "is merge region!!!" << std::endl;
    return true;
  }
  return false;
}

void LaneChangeStateMachineManager::CalculateMergePoint(std::vector<Point2D>* merge_point_list, const int merge_lane_virtual_id) {
  const auto& ego_stete = session_->environmental_model().get_ego_state_manager();
  Point2D merge_point = {ego_stete->planning_init_point().x, ego_stete->planning_init_point().y};
  Point2D boundary_line_merge_point = {ego_stete->planning_init_point().x, ego_stete->planning_init_point().y};
  (*merge_point_list)[0] = merge_point;
  (*merge_point_list)[1] = boundary_line_merge_point;
  const auto& virtual_lane_manager = session_->environmental_model().get_virtual_lane_manager();
  const auto& reference_path_manager = session_->environmental_model().get_reference_path_manager();
  const auto& overlap_lane = virtual_lane_manager->get_lane_with_virtual_id(merge_lane_virtual_id);
  if (!overlap_lane) {
    return;
  }
  const auto& cur_path = reference_path_manager->get_reference_path_by_current_lane();
  if (!cur_path) {
    return;
  }
  const auto& overlap_path = reference_path_manager->get_reference_path_by_lane(merge_lane_virtual_id);
  if (!overlap_path) {
    return;
  }
  const auto& overlap_path_coordinate = overlap_path->get_frenet_coord();
  const double ego_front_line_length = CalculateEgoFrontLineLength();
  if (ego_front_line_length < 0) {
    return;
  }
  const double cur_ego_s = cur_path->get_frenet_ego_state().s();
  const double ego_front_center_line_length = cur_path->get_frenet_coord()->Length() - cur_ego_s;
  const double buffer = 1.0;
  const double need_judgement_length = std::max(0.0, std::min(ego_front_line_length, ego_front_center_line_length) - buffer);
  const double step_length = 1.0;
  const double num = std::max(need_judgement_length / step_length, 0.0);
  bool is_find_merge_point = false;
  bool is_find_boundary_merge_point = false;
  const double lat_err = 0.3;
  for (double i = 0; i < num; i++) {
    ReferencePathPoint cur_ref_path_point_temp{};
    if (!cur_path->get_reference_point_by_lon(cur_ego_s + i * step_length, cur_ref_path_point_temp)) {
      continue;
    }
    Point2D frenet_point;
    Point2D projection_point = {cur_ref_path_point_temp.path_point.x, cur_ref_path_point_temp.path_point.y};
    if (!overlap_path_coordinate->XYToSL(projection_point,frenet_point)) {
      continue;
    }
    double lat_diff = std::abs(frenet_point.y);
    const double overlap_lane_width = overlap_lane->width_by_s(cur_ego_s + i * step_length);
    if (lat_diff < overlap_lane_width / 2 &&
        !is_find_boundary_merge_point) {
      boundary_line_merge_point = projection_point;
      is_find_boundary_merge_point = true;
    }
    if (lat_diff < lat_err) {
      // last_point_lat_diff = lat_diff;
      merge_point = projection_point;
      is_find_merge_point = true;
      break;
    }
    //处理遍历完，没有找到merge point的情况
    if (i + 1 > num &&
        !is_find_merge_point) {
      ReferencePathPoint cur_ref_path_point_temp{};
      if (cur_path->get_reference_point_by_lon(cur_ego_s + need_judgement_length, cur_ref_path_point_temp)) {
        merge_point.x = cur_ref_path_point_temp.path_point.x;
        merge_point.y = cur_ref_path_point_temp.path_point.y;
      }
    }
  }

  (*merge_point_list)[0] = merge_point;
  (*merge_point_list)[1] = boundary_line_merge_point;
}

const double LaneChangeStateMachineManager::CalculateEgoFrontLineLength(){
  const auto& virtual_lane_manager = session_->environmental_model().get_virtual_lane_manager();
  const auto& cur_lane = virtual_lane_manager->get_current_lane();
  const auto &ego_state =
      session_->environmental_model().get_ego_state_manager();
  const auto &plannig_init_point = ego_state->planning_init_point();
  double ego_x = plannig_init_point.lat_init_state.x();
  double ego_y = plannig_init_point.lat_init_state.y();
  double ego_s = 0.0, ego_l = 0.0;
  const auto &cur_lane_left_boundary= cur_lane->get_left_lane_boundary();
  std::shared_ptr<planning_math::KDPath> target_boundary_path;
    target_boundary_path =
        virtual_lane_manager->MakeBoundaryPath(cur_lane_left_boundary);
  if (target_boundary_path != nullptr) {
    if (!target_boundary_path->XYToSL(ego_x, ego_y, &ego_s, &ego_l)) {
      return -1;
    }
  } else {
    return -1;
  }
  const double ego_front_length = target_boundary_path->Length() - ego_s;
  return ego_front_length;
}
}  // namespace planning