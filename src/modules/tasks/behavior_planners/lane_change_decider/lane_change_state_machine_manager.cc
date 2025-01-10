#include "lane_change_state_machine_manager.h"

#include <Eigen/src/Core/Matrix.h>

#include <algorithm>
#include <climits>
#include <cmath>
#include <complex>
#include <cstddef>
#include <cstdint>
#include <vector>

#include "config/basic_type.h"
#include "debug_info_log.h"
#include "define/geometry.h"
#include "dynamic_world/dynamic_agent_node.h"
#include "ifly_time.h"
#include "log_glog.h"
#include "planning_context.h"
#include "spline_projection.h"
#include "task_interface/lane_change_common_helper.h"
#include "utils/kd_path.h"
#include "virtual_lane.h"

namespace planning {
namespace {
constexpr double kEps = 1e-6;
constexpr double kEgoReachBoundaryTime = 4.0;
constexpr double kLargeAgentLengthM = 8.0;
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
  PreProcess();
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
          // 在没有变道，过路口时，当前车道的virtual_id可能会发生跳变的现象
          // 在这重新维护lc_lane的值，可以保证fix lane不会跳变
          lc_lane_mgr_->reset_lc_lanes(transition_info_.lane_change_status);
        }
      }
      break;
    }
    case StateMachineLaneChangeStatus::kLaneChangePropose: {
      if (transition_info_.lane_change_status ==
          StateMachineLaneChangeStatus::kLaneChangePropose) {
        const auto &virtual_lane_mgr =
            session_->environmental_model().get_virtual_lane_manager();
        propose_state_frame_nums_++;

        bool is_propose_to_execution =
            CheckIfProposeToExecution(transition_info_.lane_change_direction,
                                      transition_info_.lane_change_type);

        iflyauto::LaneBoundaryType boundary_type =
            MakesureCurrentBoundaryType(transition_info_.lane_change_direction);
        const bool is_dashed_line =
            (boundary_type ==
                 iflyauto::LaneBoundaryType::LaneBoundaryType_MARKING_DASHED ||
             boundary_type ==
                 iflyauto::LaneBoundaryType::LaneBoundaryType_MARKING_VIRTUAL);

        bool is_propose_to_cancel =
            CheckIfProposeToCancel(transition_info_.lane_change_direction,
                                   transition_info_.lane_change_type);

        // 在propose阶段计算靠近车道线的横向偏移量
        //  CalculateLatCloseValue();

        if (is_propose_to_execution && is_dashed_line) {
          transition_info_.lane_change_status =
              StateMachineLaneChangeStatus::kLaneChangeExecution;
          lc_lane_mgr_->set_fix_lane_to_target();
          lc_timer_.Reset();
          propose_state_frame_nums_ = 0;
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
        execution_state_frame_nums_++;
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
          execution_state_frame_nums_ = 0;
        } else if (is_execution_to_cancel) {
          transition_info_.lane_change_status =
              StateMachineLaneChangeStatus::kLaneChangeCancel;
          lc_lane_mgr_->reset_lc_lanes(transition_info_.lane_change_status);
          execution_state_frame_nums_ = 0;
        } else if (is_execution_to_hold) {
          transition_info_.lane_change_status =
              StateMachineLaneChangeStatus::kLaneChangeHold;
          lc_lane_mgr_->set_fix_lane_to_origin();
          execution_state_frame_nums_ = 0;
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
    iflyauto::LaneBoundaryType boundary_type =
        MakesureCurrentBoundaryType(*lane_change_direction);
    const bool is_dashed_line =
        (boundary_type ==
             iflyauto::LaneBoundaryType::LaneBoundaryType_MARKING_DASHED ||
         boundary_type ==
             iflyauto::LaneBoundaryType::LaneBoundaryType_MARKING_VIRTUAL);
    bool is_ego_in_perfect_pose =
        CheckIfInPerfectLaneKeeping() && is_dashed_line;
    JSON_DEBUG_VALUE("is_ego_in_perfect_pose", is_ego_in_perfect_pose)
    if (*lane_change_type == INT_REQUEST && is_dashed_line) {
      return true;
    } else if (*lane_change_type == EMERGENCE_AVOID_REQUEST ||
               *lane_change_type == CONE_REQUEST) {
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
  const auto target_lane =
      virtual_lane_manager->get_lane_with_virtual_id(target_lane_virtual_id);
  if (target_lane != nullptr) {
    const double width_by_target = target_lane->width();
    if (std::abs(ego_l_target) < std::abs(width_by_target) / 2) {
      return true;
    }
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
  (void)TimeOut(lane_change_stage_info_.lc_should_back,
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
  (void)TimeOut(lane_change_stage_info_.gap_insertable,
                &lc_timer_.is_safe_hold_to_execution_count_,
                &lc_timer_.is_safe_hold_to_execution_at_time_,
                hold_to_execution_is_safe_time_out_threshold);
  return lane_change_stage_info_.gap_insertable && has_target_lane;
}

bool LaneChangeStateMachineManager::CheckIfCompleteToLaneKeeping() {
  const bool perfect_in_lane = CheckIfInPerfectLaneKeeping();
  if (target_lane_front_node_) {
    lane_change_stage_info_.lc_gap_info.front_agent_id =
        target_lane_front_node_->node_agent_id();
  }
  if (target_lane_rear_node_) {
    lane_change_stage_info_.lc_gap_info.rear_agent_id =
        target_lane_rear_node_->node_agent_id();
  }
  if (perfect_in_lane) {
    std::cout << "perfect_in_lane!!!" << std::endl;
  }
  return perfect_in_lane;
}

bool LaneChangeStateMachineManager::CheckIfInPerfectLaneKeeping() const {
  const auto &virtual_lane_mgr =
      session_->mutable_environmental_model()->get_virtual_lane_manager();
  const auto &clane_virtual_id = virtual_lane_mgr->current_lane_virtual_id();
  const double v_ego =
      session_->mutable_environmental_model()->get_ego_state_manager()->ego_v();
  std::vector<double> xp_v_ego{10.0, 15.0, 20.0, 25.0};
  double dist_threshold = interp(v_ego, xp_v_ego, config_.lc_finished_dist_thr);

  // 大曲率阈值增大一点
  std::shared_ptr<ReferencePathManager> reference_path_mgr =
      session_->mutable_environmental_model()->get_reference_path_manager();
  const auto &flane_virtual_id = lc_lane_mgr_->fix_lane_virtual_id();
  const auto &target_reference_path =
      reference_path_mgr->make_map_lane_reference_path(flane_virtual_id);
  const auto &ego_s = target_reference_path->get_frenet_ego_state().s();
  const double preview_length = 10.0;
  const double preview_step = 1.0;
  double sum_far_kappa = 0.0;
  double preview_x = 3.0 * v_ego - 5.0;
  for (double preview_distance = 0.0; preview_distance < preview_length;
       preview_distance += preview_step) {
    ReferencePathPoint preview_point;
    if (target_reference_path->get_reference_point_by_lon(
            ego_s + preview_x + preview_distance, preview_point)) {
      sum_far_kappa += std::fabs(preview_point.path_point.kappa());
    }
  }
  double aver_far_kappa =
      sum_far_kappa / std::max((preview_length / preview_step), 1.0);
  double road_radius = 1.0 / std::max(aver_far_kappa, 0.0001);
  if (road_radius < 750.0) {
    dist_threshold = 0.3;
  }

  if (road_to_ramp_turn_signal_ != RAMP_NONE) {
    // 匝道汇主路打灯时，关灯阈值可以增大一点
    dist_threshold = 0.3;
  }
  // 考虑向避让的情况
  const double lateral_offset = session_->planning_context()
                                    .lateral_offset_decider_output()
                                    .lateral_offset;
  dist_threshold = dist_threshold + std::abs(lateral_offset);

  std::vector<double> angle_thre_v{0.72, 0.48, 0.12};
  std::vector<double> angle_thre_bp{1.0, 3.0, 5.0};
  double angle_threshold = interp(v_ego, angle_thre_bp, angle_thre_v);

  // std::shared_ptr<ReferencePathManager> reference_path_mgr =
  //     session_->mutable_environmental_model()->get_reference_path_manager();
  const auto &current_reference_path =
      reference_path_mgr->get_reference_path_by_lane(clane_virtual_id);
  const auto &frenet_ego_state = current_reference_path->get_frenet_ego_state();

  bool perfect_in_lane = false;
  perfect_in_lane =
      ((std::fabs(frenet_ego_state.l()) < dist_threshold) &&
       (std::fabs(frenet_ego_state.heading_angle()) < angle_threshold));
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
  if (transition_info_.lane_change_type == EMERGENCE_AVOID_REQUEST ||
      transition_info_.lane_change_type == CONE_REQUEST ||
      transition_info_.lane_change_type == MERGE_REQUEST) {
    lc_valid_thre = 1;
  }
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
  const auto &virtual_lane_manager =
      session_->mutable_environmental_model()->get_virtual_lane_manager();
  const auto target_lane = virtual_lane_manager->get_lane_with_virtual_id(
      lc_req_mgr_->target_lane_virtual_id());
  if (target_lane == nullptr) {
    return lc_state_info;
  }
  lc_state_info.gap_insertable = true;
  lc_invalid_track_.reset();

  // 判断与各障碍物之间的gap是否安全
  if (target_lane_middle_node_) {
    lc_invalid_track_.set_value(
        target_lane_middle_node_->node_agent_id(),
        target_lane_middle_node_->node_back_edge_to_ego_front_edge_distance(),
        target_lane_middle_node_->node_speed());
    lc_state_info.gap_insertable = false;
    lc_state_info.lc_invalid_reason = "front view invalid";
    lc_state_info.lc_gap_info.front_agent_id =
        target_lane_middle_node_->node_agent_id();
    return lc_state_info;
  }

  if (target_lane_front_node_) {
    lc_state_info.lc_gap_info.front_agent_id =
        target_lane_front_node_->node_agent_id();
    CalculateFrontGapFeasible(&lc_state_info);
    if (!lc_state_info.gap_insertable) {
      return lc_state_info;
    }
  }

  if (target_lane_rear_node_) {
    lc_state_info.lc_gap_info.rear_agent_id =
        target_lane_rear_node_->node_agent_id();
    CalculateSideGapFeasible(&lc_state_info);
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
  assert(direction == LEFT_CHANGE || direction == RIGHT_CHANGE);
  LaneChangeStageInfo lc_state_info;
  const auto &virtual_lane_manager =
      session_->mutable_environmental_model()->get_virtual_lane_manager();
  const auto target_lane = virtual_lane_manager->get_lane_with_virtual_id(
      lc_req_mgr_->target_lane_virtual_id());
  if (target_lane == nullptr) {
    return lc_state_info;
  }
  lc_state_info.gap_insertable = true;
  lc_invalid_track_.reset();

  // 判断与各障碍物之间的gap是否安全
  if (target_lane_middle_node_) {
    lc_state_info.lc_should_back = true;
    lc_state_info.lc_back_reason = "front view back";
    lc_back_track_.set_value(
        target_lane_middle_node_->node_agent_id(),
        target_lane_middle_node_->node_back_edge_to_ego_front_edge_distance(),
        target_lane_middle_node_->node_speed());
    return lc_state_info;
  }

  if (target_lane_front_node_) {
    lc_state_info.lc_gap_info.front_agent_id =
        target_lane_front_node_->node_agent_id();
    CalculateFrontAreaIfNeedBack(&lc_state_info);
    if (lc_state_info.lc_should_back) {
      return lc_state_info;
    }
  }

  if (target_lane_rear_node_) {
    lc_state_info.lc_gap_info.rear_agent_id =
        target_lane_rear_node_->node_agent_id();
    CalculateSideAreaIfNeedBack(&lc_state_info);
    if (lc_state_info.lc_should_back) {
      return lc_state_info;
    }
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
  // static const double min_ego_v_cruise = 2.0;
  const auto &v_ref_cruise = std::fmax(
      session_->environmental_model().get_ego_state_manager()->ego_v_cruise(),
      config_.min_ego_v_cruise);

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
      double diff_x = ref_point.at(0).path_point.x() -
                      planning_init_point.lat_init_state.x();
      double diff_y = ref_point.at(0).path_point.y() -
                      planning_init_point.lat_init_state.y();
      distance_to_first_point = std::sqrt(diff_x * diff_x + diff_y * diff_y);
    }

    normal_care_spline_length =
        std::min(kHppMaxRearDistance, distance_to_first_point);
  }

  for (size_t i = 0; i < point_size; ++i) {
    cart_ref_info.x_vec[i] = ref_point.at(i).path_point.x();
    cart_ref_info.y_vec[i] = ref_point.at(i).path_point.y();
    cart_ref_info.s_vec[i] =
        i > 0 ? cart_ref_info.s_vec[i - 1] +
                    std::hypot(ref_point.at(i).path_point.x() -
                                   ref_point.at(i - 1).path_point.x(),
                               ref_point.at(i).path_point.y() -
                                   ref_point.at(i - 1).path_point.y())
              : 0.;
    kappa_radius_vec[i] = std::min(
        std::max(1.0 / (ref_point.at(i).path_point.kappa() + 1e-6), -10000.0),
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

  Point2D cart_init_pt(init_pos.x(), init_pos.y());
  Point2D frenet_init_pt{0.0, 0.0};
  if (frenet_coord->XYToSL(cart_init_pt, frenet_init_pt)) {
    s_ref = frenet_init_pt.x;
  } else {
    coarse_planning_info.reference_path.reset();
    LOG_DEBUG("kd_path coordinate conversion init_pos failed");
  }

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
  lane_change_decider_output.should_premove =
      lane_change_stage_info_.should_premove;
  lane_change_decider_output.lc_request = lc_req_mgr_->request();
  lane_change_decider_output.lc_request_source = lc_req_mgr_->request_source();
  lane_change_decider_output.act_request_source =
      lc_req_mgr_->act_request_source();
  lane_change_decider_output.lc_turn_light = lc_req_mgr_->turn_signal();

  lane_change_decider_output.lc_invalid_track = lc_invalid_track_;
  lane_change_decider_output.lc_back_track = lc_back_track_;
  lane_change_decider_output.lc_valid_cnt = lc_valid_cnt_;
  lane_change_decider_output.lc_back_cnt = lc_back_cnt_;

  lane_change_decider_output.is_ego_on_leftmost_lane = is_ego_on_leftmost_lane_;
  lane_change_decider_output.is_ego_on_rightmost_lane =
      is_ego_on_rightmost_lane_;

  lane_change_decider_output.lc_gap_info.front_agent_id =
      lane_change_stage_info_.lc_gap_info.front_agent_id;
  lane_change_decider_output.lc_gap_info.rear_agent_id =
      lane_change_stage_info_.lc_gap_info.rear_agent_id;

  GenerateTurnSignalForSplitRegion();
  lane_change_decider_output.dir_turn_signal_road_to_ramp =
      road_to_ramp_turn_signal_;
  lane_change_decider_output.int_request_cancel_reason =
      lc_req_mgr_->int_request_cancel_reason();
  lane_change_decider_output.ilc_virtual_req =
      lc_req_mgr_->get_ilc_virtual_request();
  if (lane_change_decider_output.curr_state == kLaneChangePropose) {
    lane_change_decider_output.lateral_close_boundary_offset =
        lat_close_boundary_offset_;
  } else {
    lane_change_decider_output.lateral_close_boundary_offset = 0;
  }
  JSON_DEBUG_VALUE("lat_close_bound_offset",
                   lane_change_decider_output.lateral_close_boundary_offset)
}
void LaneChangeStateMachineManager::CalculateSideGapFeasible(
    LaneChangeStageInfo *const lc_state_info) {
  const double v_ego =
      session_->environmental_model().get_ego_state_manager()->ego_v();
  const double node_v = target_lane_rear_node_->node_speed();
  const double node_a = target_lane_rear_node_->node_accel();
  const double distance_rel =
      target_lane_rear_node_->node_front_edge_to_ego_back_edge_distance();

  const double need_safety_dist = planning::CalcGapObjSafeDistance(
      v_ego, distance_rel, node_v, node_a, is_large_car_in_side_, false);

  if (distance_rel < need_safety_dist) {
    lc_invalid_track_.set_value(target_lane_rear_node_->node_agent_id(),
                                -distance_rel, node_v);
    lc_state_info->gap_insertable = false;
    lc_state_info->lc_invalid_reason = "side view invalid";
  }
}
void LaneChangeStateMachineManager::CalculateFrontGapFeasible(
    LaneChangeStageInfo *const lc_state_info) {
  const double v_ego =
      session_->environmental_model().get_ego_state_manager()->ego_v();
  double safety_dist = v_ego * v_ego * 0.02 + 2.0;
  const double distance_rel =
      target_lane_front_node_->node_back_edge_to_ego_front_edge_distance();
  const double node_v = target_lane_front_node_->node_speed();
  const double node_a = target_lane_front_node_->node_accel();

  const double target_lane_need_safety_dist = planning::CalcGapObjSafeDistance(
      v_ego, distance_rel, node_v, node_a, false, true);

  if (distance_rel < target_lane_need_safety_dist) {
    lc_invalid_track_.set_value(target_lane_front_node_->node_agent_id(),
                                distance_rel, node_v);
    lc_state_info->gap_insertable = false;
    lc_state_info->lc_invalid_reason = "front view invalid";
    return;
  }

  if (ego_lane_front_node_) {
    const double ego_lane_distance_rel =
        ego_lane_front_node_->node_back_edge_to_ego_front_edge_distance();
    const double ego_front_node_v = ego_lane_front_node_->node_speed();
    const double ego_front_node_a = ego_lane_front_node_->node_accel();

    const double ego_lane_need_safety_dist = planning::CalcGapObjSafeDistance(
        v_ego, ego_lane_distance_rel, ego_front_node_v, ego_front_node_a, false,
        true);
    if (ego_lane_distance_rel < ego_lane_need_safety_dist) {
      lc_invalid_track_.set_value(ego_lane_front_node_->node_agent_id(),
                                  ego_lane_distance_rel, ego_front_node_v);
      lc_state_info->gap_insertable = false;
      lc_state_info->lc_invalid_reason = "front view invalid";
    }
  }
}
void LaneChangeStateMachineManager::CalculateSideAreaIfNeedBack(
    LaneChangeStageInfo *const lc_state_info) {
  const auto &ego_state =
      session_->environmental_model().get_ego_state_manager();
  const double v_ego = ego_state->ego_v();
  const double v_node = target_lane_rear_node_->node_speed();
  const double a_node = target_lane_rear_node_->node_accel();
  const double distance_rel =
      target_lane_rear_node_->node_front_edge_to_ego_back_edge_distance();
  // 在安全性gap的判断中是预计自车中心3s可到车道边界线。
  const int total_frames_in_execution_state = kEgoReachBoundaryTime * 10;

  // 如果以当前的加速度为基准，预测剩余时间后是否会发生碰撞
  const double t_remain_lc =
      (total_frames_in_execution_state - execution_state_frame_nums_) * 0.1;
  if (t_remain_lc > 0) {
    const double obstacle_dist_remain =
        0.5 * a_node * t_remain_lc * t_remain_lc + v_node * t_remain_lc;
    const double ego_dist_remain = v_ego * t_remain_lc;
    std::array<double, 3> xp{40.0 / 3.6, 80.0 / 3.6, 120.0 / 3.6};
    std::array<double, 3> buffer{1.0, 3.0, 5.0};
    const double buffer_dist = interp(v_ego, xp, buffer);
    const double need_rel_dis =
        obstacle_dist_remain - ego_dist_remain + buffer_dist;
    if (need_rel_dis > distance_rel) {
      lc_state_info->lc_should_back = true;
      lc_state_info->lc_back_reason = "side view back";
      lc_back_track_.set_value(target_lane_rear_node_->node_agent_id(),
                               -distance_rel, v_node);
      return;
    }
  }
}
void LaneChangeStateMachineManager::CalculateFrontAreaIfNeedBack(
    LaneChangeStageInfo *const lc_state_info) {
  const auto &ego_state =
      session_->environmental_model().get_ego_state_manager();
  const double v_ego = ego_state->ego_v();
  const double v_node = target_lane_front_node_->node_speed();
  const double a_node = target_lane_front_node_->node_accel();
  const double distance_rel =
      target_lane_front_node_->node_back_edge_to_ego_front_edge_distance();
  // 在安全性gap的判断中是预计自车中心3s可到车道边界线。
  const int total_frames_in_execution_state = kEgoReachBoundaryTime * 10;

  // 如果以当前的加速度为基准，预测剩余时间后是否会发生碰撞
  const double t_remain_lc =
      (total_frames_in_execution_state - execution_state_frame_nums_) * 0.1;
  if (t_remain_lc > 0) {
    const double obstacle_dist_remain =
        0.5 * a_node * t_remain_lc * t_remain_lc + v_node * t_remain_lc;
    const double ego_dist_remain = v_ego * t_remain_lc;
    std::array<double, 3> xp{40.0 / 3.6, 80.0 / 3.6, 120.0 / 3.6};
    std::array<double, 3> buffer{1.0, 3.0, 5.0};
    const double buffer_dist = interp(v_ego, xp, buffer);
    const double need_rel_dis =
        ego_dist_remain - obstacle_dist_remain + buffer_dist;
    if (need_rel_dis > distance_rel) {
      lc_state_info->lc_should_back = true;
      lc_state_info->lc_back_reason = "front view back";
      lc_back_track_.set_value(target_lane_front_node_->node_agent_id(),
                               distance_rel, v_node);
      return;
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
  must_change_lane_ = false;
  propose_state_frame_nums_ = 0;
  execution_state_frame_nums_ = 0;
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
  lat_behavior_common->mutable_near_car_ids_target()->Clear();
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
  // 生成转向灯信号的条件：同时满足以下5个条件:
  // 1、存在两条lane的relative_id为0；
  // 2、ramp的方向即转向灯信号的方向；
  // 3、当前自车还在高速路上(NOA模式下);
  // 4、当前没有生成变道请求。
  // 5、当前还在主路上，即不在匝道上。
  const auto virtual_lane_manager =
      session_->environmental_model().get_virtual_lane_manager();
  const auto &route_info_output =
      session_->environmental_model().get_route_info()->get_route_info_output();
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
  bool is_ego_on_expressway = route_info_output.is_ego_on_expressway;
  JSON_DEBUG_VALUE("origin_relative_id_zero_nums",
                   origin_relative_id_zero_nums);
  // overlap_lane_virtual_id_ = virtual_lane_manager->current_lane_virtual_id();
  bool is_off_turn_signal = false;
  if (origin_relative_id_zero_nums > 1) {
    RampDirection ramp_direction = RAMP_NONE;
    if (IsSplitRegion(&ramp_direction)) {
      if (is_ego_on_expressway &&
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
  const auto distance_to_toll_station =
      route_info_output.distance_to_toll_station;
  const auto distance_to_route_end = route_info_output.distance_to_route_end;
  // 接近收费站或者终点时，抑制分流点的判断
  if (distance_to_toll_station < 400 || distance_to_route_end < 400) {
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
  const auto &route_info_output =
      session_->environmental_model().get_route_info()->get_route_info_output();
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
  std::shared_ptr<planning_math::KDPath> reference_path_frenet_coordinate =
      reference_path->get_frenet_coord();
  Point2D projection_point = {cur_ref_path_finally_point.x(),
                              cur_ref_path_finally_point.y()};
  const double front_line_distance = CalculateEgoFrontLineLength();
  bool is_on_ramp = route_info_output.is_on_ramp;
  if (is_on_ramp) {
    ReferencePathPoint refpoint = {};
    if (current_reference_path->get_reference_point_by_lon(
            current_reference_path->get_frenet_ego_state().s() + 50,
            refpoint)) {
      projection_point = {refpoint.path_point.x(), refpoint.path_point.y()};
    }
  } else {
    if (std::abs(length_diff_cur_lane_with_overlap_lane) >
        length_diff_threshold) {
      if (length_diff_cur_lane_with_overlap_lane > length_diff_threshold) {
        is_cur_path_project_to_ref_path = false;
        const auto ref_path_finally_point =
            current_reference_path->get_points().back().path_point;
        projection_point = {ref_path_finally_point.x(),
                            ref_path_finally_point.y()};
        reference_path_frenet_coordinate =
            current_reference_path->get_frenet_coord();
      }
    } else {
      ReferencePathPoint refpoint = {};
      if (current_reference_path->get_reference_point_by_lon(
              current_reference_path->get_frenet_ego_state().s() +
                  front_line_distance,
              refpoint)) {
        projection_point = {refpoint.path_point.x(), refpoint.path_point.y()};
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
  std::shared_ptr<planning_math::KDPath> overlap_path_frenet_coordinate =
      overlap_reference_path->get_frenet_coord();

  const auto &ego_vertices_points = session_->environmental_model()
                                        .get_ego_state_manager()
                                        ->polygon()
                                        .points();
  double ego_dis_to_ref_lane = NL_NMAX;
  for (const auto &ego_vertices_point : ego_vertices_points) {
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

const double LaneChangeStateMachineManager::CalculateEgoFrontLineLength() {
  const double default_lane_line_length = -1.0;
  const auto virtual_lane_manager =
      session_->environmental_model().get_virtual_lane_manager();
  const auto &cur_lane = virtual_lane_manager->get_current_lane();
  const auto &ego_state =
      session_->environmental_model().get_ego_state_manager();
  const auto &plannig_init_point = ego_state->planning_init_point();
  double ego_x = plannig_init_point.lat_init_state.x();
  double ego_y = plannig_init_point.lat_init_state.y();
  double ego_s = 0.0, ego_l = 0.0;
  std::shared_ptr<planning_math::KDPath> target_boundary_path;
  const auto &cur_lane_left_boundary = cur_lane->get_left_lane_boundary();
  target_boundary_path =
      virtual_lane_manager->MakeBoundaryPath(cur_lane_left_boundary);

  if (target_boundary_path != nullptr) {
    if (!target_boundary_path->XYToSL(ego_x, ego_y, &ego_s, &ego_l)) {
      return default_lane_line_length;
    }
  } else {
    return default_lane_line_length;
  }
  const double ego_front_length = target_boundary_path->Length() - ego_s;
  return ego_front_length;
}

// TODO(fengwang31):后面需要把这个函数和lane change
// request中的MakesureCurrentBoundaryType合在一起
iflyauto::LaneBoundaryType
LaneChangeStateMachineManager::MakesureCurrentBoundaryType(
    const RequestType lc_request) const {
  const auto &virtual_lane_mgr =
      session_->environmental_model().get_virtual_lane_manager();
  const auto &ego_state =
      session_->environmental_model().get_ego_state_manager();
  double lane_line_length = 0.0;
  std::shared_ptr<planning_math::KDPath> target_boundary_path;
  const auto &plannig_init_point = ego_state->planning_init_point();
  double ego_x = plannig_init_point.lat_init_state.x();
  double ego_y = plannig_init_point.lat_init_state.y();
  double ego_s = 0.0, ego_l = 0.0;
  const std::shared_ptr<VirtualLane> current_lane =
      virtual_lane_mgr->get_current_lane();

  if (lc_request == LEFT_CHANGE) {
    const auto &left_lane_boundarys = current_lane->get_left_lane_boundary();
    target_boundary_path =
        virtual_lane_mgr->MakeBoundaryPath(left_lane_boundarys);
    if (target_boundary_path != nullptr) {
      if (!target_boundary_path->XYToSL(ego_x, ego_y, &ego_s, &ego_l)) {
        return iflyauto::LaneBoundaryType_MARKING_SOLID;
      }
    } else {
      return iflyauto::LaneBoundaryType_MARKING_SOLID;
    }
    for (int i = 0; i < left_lane_boundarys.type_segments_size; i++) {
      lane_line_length += left_lane_boundarys.type_segments[i].length;
      if (lane_line_length > ego_s) {
        return left_lane_boundarys.type_segments[i].type;
      }
    }
  } else if (lc_request == RIGHT_CHANGE) {
    const auto &right_lane_boundarys = current_lane->get_right_lane_boundary();
    target_boundary_path =
        virtual_lane_mgr->MakeBoundaryPath(right_lane_boundarys);
    if (target_boundary_path != nullptr) {
      if (!target_boundary_path->XYToSL(ego_x, ego_y, &ego_s, &ego_l)) {
        return iflyauto::LaneBoundaryType_MARKING_SOLID;
      }
    } else {
      return iflyauto::LaneBoundaryType_MARKING_SOLID;
    }
    for (int i = 0; i < right_lane_boundarys.type_segments_size; i++) {
      lane_line_length += right_lane_boundarys.type_segments[i].length;
      if (lane_line_length > ego_s) {
        return right_lane_boundarys.type_segments[i].type;
      }
    }
  }
  return iflyauto::LaneBoundaryType_MARKING_SOLID;
}

void LaneChangeStateMachineManager::PreProcess() {
  IsEgoOnSideLane();
  lane_change_stage_info_.Reset();
  target_lane_front_node_ = nullptr;
  target_lane_middle_node_ = nullptr;
  target_lane_rear_node_ = nullptr;
  ego_lane_front_node_ = nullptr;

  RequestType direction = lc_req_mgr_->request();
  const auto &current_lc_state = transition_info_.lane_change_status;
  if (direction == RequestType::NO_CHANGE) {
    return;
  }
  // 获取目标车道的障碍物
  const auto &dynamic_world =
      session_->environmental_model().get_dynamic_world();
  const int64_t ego_front_node_id = dynamic_world->ego_front_node_id();
  int64_t target_lane_front_node_id = planning_data::kInvalidId;
  int64_t target_lane_middle_node_id = planning_data::kInvalidId;
  int64_t target_lane_rear_node_id = planning_data::kInvalidId;
  int64_t ego_lane_front_node_id = planning_data::kInvalidId;
  if (direction == LEFT_CHANGE) {
    if (current_lc_state == kLaneChangeExecution ||
        current_lc_state == kLaneChangeComplete) {
      target_lane_front_node_id = dynamic_world->ego_front_node_id();
      target_lane_rear_node_id = dynamic_world->ego_rear_node_id();
    } else {
      target_lane_front_node_id = dynamic_world->ego_left_front_node_id();
      target_lane_middle_node_id = dynamic_world->ego_left_node_id();
      target_lane_rear_node_id = dynamic_world->ego_left_rear_node_id();
      ego_lane_front_node_id = dynamic_world->ego_front_node_id();
    }
  } else {
    if (current_lc_state == kLaneChangeExecution ||
        current_lc_state == kLaneChangeComplete) {
      target_lane_front_node_id = dynamic_world->ego_front_node_id();
      target_lane_rear_node_id = dynamic_world->ego_rear_node_id();
    } else {
      target_lane_front_node_id = dynamic_world->ego_right_front_node_id();
      target_lane_middle_node_id = dynamic_world->ego_right_node_id();
      target_lane_rear_node_id = dynamic_world->ego_right_rear_node_id();
      ego_lane_front_node_id = dynamic_world->ego_front_node_id();
    }
  }
  target_lane_front_node_ = dynamic_world->GetNode(target_lane_front_node_id);
  target_lane_middle_node_ = dynamic_world->GetNode(target_lane_middle_node_id);
  target_lane_rear_node_ = dynamic_world->GetNode(target_lane_rear_node_id);
  ego_lane_front_node_ = dynamic_world->GetNode(ego_lane_front_node_id);

  if (target_lane_rear_node_) {
    is_large_car_in_side_ = IsLargeAgent(target_lane_rear_node_);
  } else {
    is_large_car_in_side_ = false;
  }
}

bool LaneChangeStateMachineManager::IsLargeAgent(
    const planning_data::DynamicAgentNode *agent) {
  return agent::AgentType::BUS == agent->type() ||
         agent::AgentType::TRUCK == agent->type() ||
         agent::AgentType::TRAILER == agent->type() ||
         agent->node_length() > kLargeAgentLengthM;
}

void LaneChangeStateMachineManager::CalculateLatCloseValue() {
  // 当前plannning的规划周期为10hz，安全性检查连续5帧通过则视为安全。
  // 若在10帧内未通过则采取自车向目标车道靠近的策略
  const auto &virtual_lane_mgr =
      session_->environmental_model().get_virtual_lane_manager();
  const double ego_half_width =
      VehicleConfigurationContext::Instance()->get_vehicle_param().width / 2;
  const double cur_lane_half_width =
      virtual_lane_mgr->get_current_lane()->width() / 2;
  const double buffer = 0.35;
  const double lat_offset_value = cur_lane_half_width - ego_half_width - buffer;
  const double v_ego =
      session_->environmental_model().get_ego_state_manager()->ego_v();
  double obj_a = 0.0;
  if (target_lane_rear_node_) {
    obj_a = target_lane_rear_node_->node_accel();
  }
  // 假设后车加速度大于0.7m/s2,则认为后车没有减速的意图
  bool is_no_brake_attention_side_car = obj_a > 0.7;
  bool is_lat_offset_when_neirghbor_car =
      !target_lane_middle_node_ ||
      (target_lane_middle_node_ && lat_close_boundary_offset_ > kEps);
  // lat_close_boundary_offset_ > kEps表示已经在靠近车道边界线的过程中了

  if (lat_offset_value > 0 && propose_state_frame_nums_ > 10 &&
      is_lat_offset_when_neirghbor_car && !is_large_car_in_side_ &&
      !is_no_brake_attention_side_car) {
    lat_close_boundary_offset_ =
        transition_info_.lane_change_direction == LEFT_CHANGE
            ? lat_offset_value
            : -lat_offset_value;
  } else {
    lat_close_boundary_offset_ = 0;
  }
}

void LaneChangeStateMachineManager::IsEgoOnSideLane() {
  const auto &virtual_lane_manager =
      session_->environmental_model().get_virtual_lane_manager();
  auto &lane_change_decider_output = session_->mutable_planning_context()
                                         ->mutable_lane_change_decider_output();
  const auto &left_lane = virtual_lane_manager->get_left_lane();
  const auto &right_lane = virtual_lane_manager->get_right_lane();
  if (!left_lane) {
    is_ego_on_leftmost_lane_ = true;
  } else {
    is_ego_on_leftmost_lane_ = false;
  }
  if (!right_lane) {
    is_ego_on_rightmost_lane_ = true;
  } else {
    is_ego_on_rightmost_lane_ = false;
  }
}
}  // namespace planning