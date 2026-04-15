#include "lane_change_state_machine_manager.h"

#include <Eigen/src/Core/Matrix.h>

#include <algorithm>
#include <cassert>
#include <climits>
#include <cmath>
#include <complex>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <unordered_set>
#include <vector>

#include "basic_types.pb.h"
#include "behavior_planners/lane_change_decider/lane_change_joint_decision_generator/lat_lon_joint_decision_generator.h"
#include "behavior_planners/lane_change_decider/lane_change_joint_decision_generator/lat_lon_joint_decision_output.h"
#include "config/basic_type.h"
#include "debug_info_log.h"
#include "define/geometry.h"
#include "dynamic_world/dynamic_agent_node.h"
#include "geometry_math.h"
#include "ifly_time.h"
#include "log_glog.h"
#include "planning_context.h"
#include "session.h"
#include "spline_projection.h"
#include "src/modules/adas_function/display_state_types.h"
#include "task_interface/lane_change_utils.h"
#include "trajectory1d/constant_jerk_trajectory1d.h"
#include "trajectory1d/piecewise_jerk_acceleration_trajectory1d.h"
#include "utils/cartesian_coordinate_system.h"
#include "utils/kd_path.h"
#include "virtual_lane.h"

namespace planning {
using namespace planning_math;  // fix
namespace {
constexpr double kEps = 1e-6;
constexpr double kEgoReachBoundaryTime = 3.0;
constexpr double kStandardLaneWidth = 3.7;
constexpr double kLargeAgentLengthM = 8.0;
constexpr double kPreTriggleHighPriorityMLCTime = 10.0;
constexpr std::array<double, 3> xp{11.111, 22.222, 33.333};
constexpr std::array<double, 3> fp{3.0, 8.0, 20.0};
constexpr std::array<double, 3> buffer{1.0, 3.0, 10.0};
constexpr std::array<double, 3> fp_for_large_car{6.0, 12.0, 30.0};
constexpr std::array<double, 3> buffer_for_large_car{3.0, 6, 20.0};
constexpr double kMaxReachedTime = 10.0;

const std::unordered_set<agent::AgentType> FacilityTypes = {
    agent::AgentType::TRAFFIC_CONE, agent::AgentType::TRAFFIC_BARREL,
    agent::AgentType::WATER_SAFETY_BARRIER, agent::AgentType::CTASH_BARREL};
}  // namespace

LaneChangeStateMachineManager::LaneChangeStateMachineManager(
    const EgoPlanningConfigBuilder* config_builder, framework::Session* session,
    std::shared_ptr<LaneChangeRequestManager> lane_change_req_mgr,
    std::shared_ptr<LaneChangeLaneManager> lane_change_lane_mgr)
    : session_(session),
      lc_req_mgr_(lane_change_req_mgr),
      lc_lane_mgr_(lane_change_lane_mgr),
      config_builder_(config_builder),
      lc_joint_decision_generator_(
          std::make_shared<LatLonJointDecision>(config_builder, session)),
      lc_request_(session,
                  session->environmental_model().get_virtual_lane_manager(),
                  lane_change_lane_mgr) {
  config_ = config_builder->cast<ScenarioStateMachineConfig>();
  speed_planning_config_ = config_builder->cast<SpeedPlannerConfig>();
  congestion_detection_config_ =
      config_builder->cast<CongestionDetectionConfig>();
  lc_safety_check_config_ = config_builder->cast<LanChangeSafetyCheckConfig>();
}

void LaneChangeStateMachineManager::Update() {
  PreProcess();
  JointLaneChangeDecisionGeneration();
  RunStateMachine();
  GenerateStateMachineOutput();
  UpdateCoarsePlanningInfo();
  // UpdateLCCoarsePlanningInfo();
  UpdateStateMachineDebugInfo();
  // UpdateHMIInfo();
}

void LaneChangeStateMachineManager::RunStateMachine() {
  SplitSelectingStateMachine();
  //进入到选道状态机跳转状态则重置和跳过变道状态机
  if(split_selecting_info_.split_selecting_status != kNonSelecting){
    OnlyResetLCStateMachine();// 只清空变道状态机，不清空拨杆请求
    return;
  }
  const auto& pilot_req = session_->environmental_model()
                              .get_local_view()
                              .function_state_machine_info.pilot_req;
  switch (transition_info_.lane_change_status) {
    case StateMachineLaneChangeStatus::kLaneKeeping: {
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
        is_pre_move_ = false;
        lat_close_boundary_offset_ = 0.0;
      } else {
        // 在没有变道，过路口时，当前车道的virtual_id可能会发生跳变的现象
        // 在这重新维护lc_lane的值，可以保证fix lane不会跳变
        lc_lane_mgr_->reset_lc_lanes(transition_info_.lane_change_status);
      }
      break;
    }
    case StateMachineLaneChangeStatus::kLaneChangePropose: {
      if (transition_info_.lane_change_status ==
          StateMachineLaneChangeStatus::kLaneChangePropose) {
        const auto& virtual_lane_mgr =
            session_->environmental_model().get_virtual_lane_manager();
        // const bool is_exist_interactive_select_split =
        //   virtual_lane_mgr->get_is_exist_interactive_select_split();
        // const bool split_lane_on_left_side_before_interactive =
        //     virtual_lane_mgr->get_split_lane_on_left_side_before_interactive();
        // const bool split_lane_on_right_side_before_interactive =
        //     virtual_lane_mgr->get_split_lane_on_right_side_before_interactive();
        // const bool other_split_lane_left_side = virtual_lane_mgr->get_other_split_lane_left_side();
        // const bool other_split_lane_right_side = virtual_lane_mgr->get_other_split_lane_right_side();
        // bool enable_interactive_select_split = false;
        // if(((other_split_lane_left_side && transition_info_.lane_change_direction == LEFT_CHANGE) ||
        //     (other_split_lane_right_side && transition_info_.lane_change_direction == RIGHT_CHANGE) ||
        //     (split_lane_on_left_side_before_interactive && transition_info_.lane_change_direction == LEFT_CHANGE) ||
        //     (split_lane_on_right_side_before_interactive && transition_info_.lane_change_direction == RIGHT_CHANGE) ||
        //     is_exist_interactive_select_split) && transition_info_.lane_change_type == INT_REQUEST) {
        //   enable_interactive_select_split = true;
        // }
        propose_state_frame_nums_++;
        if (config_.enable_overtake_lane_change_confirmation){
          if (!overtake_lane_change_confirmed_ &&
              pilot_req.is_overtake_lane_change_confirmed) {
            overtake_lane_change_confirmed_ = true;
          }
          if (lc_req_mgr_->request_source() ==
                  RequestSource::OVERTAKE_REQUEST &&
              !overtake_lane_change_confirmed_) {
            break;
          }
        }
        bool is_propose_to_execution =
            CheckIfProposeToExecution(transition_info_.lane_change_direction,
                                      transition_info_.lane_change_type);

        // const bool is_dashed_line =
        //     IsDashLineCurBoundary(transition_info_.lane_change_direction);

        bool is_propose_to_cancel =
            CheckIfProposeToCancel(transition_info_.lane_change_direction,
                                   transition_info_.lane_change_type);

        // 在propose阶段计算靠近车道线的横向偏移量
        // CalculateLatCloseValue();
        CalculateCongestionLatOffsetValue();
        // lat_close_boundary_offset_ = 0.0;

        if (is_propose_to_execution && !is_propose_to_cancel) {
          transition_info_.lane_change_status =
              StateMachineLaneChangeStatus::kLaneChangeExecution;
          lc_lane_mgr_->set_fix_lane_to_target();
          lc_timer_.Reset();
          propose_state_frame_nums_ = 0;
          is_pre_move_ = false;
          lat_close_boundary_offset_ = 0.0;
        } else if (is_propose_to_cancel) {
          transition_info_.lane_change_status =
              StateMachineLaneChangeStatus::kLaneKeeping;
          // if (enable_interactive_select_split) {
          //   last_state_ = kLaneKeeping;
          // }
          // ResetStateMachine();
          is_pre_move_ = false;
          lat_close_boundary_offset_ = 0.0;
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

        bool is_dash_enough = !IsLCPathCollisionWithSolidLine(
            lc_lane_mgr_->origin_lane_virtual_id(),
            transition_info_.lane_change_status,
            transition_info_.lane_change_type,
            transition_info_.lane_change_direction);
        if (!is_dash_enough) {
          execution_state_dash_cnt++;
        } else {
          execution_state_dash_cnt = 0;
        }
        if (execution_state_dash_cnt >= 2) {
          is_dash_not_enough_for_lc_ = true;
        }
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
          lc_hold_state_lat_offset_ = CalculateLCHoldStateLatOffset();
          execution_state_frame_nums_ = 0;
        }
      }
      break;
    }
    case StateMachineLaneChangeStatus::kLaneChangeHold: {
      if (transition_info_.lane_change_status ==
          StateMachineLaneChangeStatus::kLaneChangeHold) {
        hold_state_frame_nums_++;

        // 在hold状态下最多维持8s，不满足变道条件那么就返回原车道
        bool is_dash_enough = !IsLCPathCollisionWithSolidLine(
                                  lc_lane_mgr_->origin_lane_virtual_id(),
                                  transition_info_.lane_change_status,
                                  transition_info_.lane_change_type,
                                  transition_info_.lane_change_direction) &&
                              CheckTargetLaneValid() &&
                              !IsLCPathCollisionWithRoadEdge(
                                  lc_lane_mgr_->origin_lane_virtual_id(),
                                  lc_lane_mgr_->target_lane_virtual_id(),
                                  transition_info_.lane_change_status);
        hold_state_dash_cnt = is_dash_enough ? 0 : hold_state_dash_cnt + 1;

        bool is_hold_to_cancel =
            hold_state_frame_nums_ > 80
            || (hold_state_dash_cnt >= 2)
            || lc_req_mgr_->request() == NO_CHANGE;

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
          hold_state_frame_nums_ = 0;
          is_high_priority_back_ = false;
        } else if (is_hold_to_cancel) {
          transition_info_.lane_change_status =
              StateMachineLaneChangeStatus::kLaneChangeCancel;
          lc_lane_mgr_->reset_lc_lanes(transition_info_.lane_change_status);
          lane_change_stage_info_.Reset();
          hold_state_frame_nums_ = 0;
          is_high_priority_back_ = false;
        } else if (is_hold_to_execution) {
          transition_info_.lane_change_status =
              StateMachineLaneChangeStatus::kLaneChangeExecution;
          lc_lane_mgr_->set_fix_lane_to_target();
          hold_state_frame_nums_ = 0;
          is_high_priority_back_ = false;
        }
      }
      break;
    }
    case StateMachineLaneChangeStatus::kLaneChangeComplete: {
      if (transition_info_.lane_change_status ==
          StateMachineLaneChangeStatus::kLaneChangeComplete) {
        complete_state_frame_nums_++;

        bool is_complete_to_lane_keeping = CheckIfCompleteToLaneKeeping();

        bool is_complete_to_cancel = CheckIfCompleteToCancel();

        if (is_complete_to_lane_keeping) {
          transition_info_.lane_change_status =
              StateMachineLaneChangeStatus::kLaneKeeping;
          // ResetStateMachine();
          pre_lane_change_finish_time_ = IflyTime::Now_ms();
        } else if (is_complete_to_cancel) {
          transition_info_.lane_change_status =
              StateMachineLaneChangeStatus::kLaneChangeCancel;
          lc_lane_mgr_->reset_lc_lanes(transition_info_.lane_change_status);
          complete_state_frame_nums_ = 0;
        }
      }
      break;
    }
    case StateMachineLaneChangeStatus::kLaneChangeCancel: {
      if (transition_info_.lane_change_status ==
          StateMachineLaneChangeStatus::kLaneChangeCancel) {
        bool is_cancel_to_lane_keeping = CheckIfCancelToLaneKeeping();

        bool is_cancel_to_complete = CheckIfCancelToComplete();

        if (is_cancel_to_lane_keeping) {
          transition_info_.lane_change_status =
              StateMachineLaneChangeStatus::kLaneKeeping;
          // ResetStateMachine();
          pre_lane_change_finish_time_ = IflyTime::Now_ms();
        } else if (is_cancel_to_complete) {
          transition_info_.lane_change_status =
              StateMachineLaneChangeStatus::kLaneChangeComplete;
          lc_lane_mgr_->reset_lc_lanes(transition_info_.lane_change_status);
        }
      }
      break;
    }
    default:
      // should never enter here, add some debug info here
      break;
  }
  JSON_DEBUG_VALUE("overtake_lane_change_confirmed", overtake_lane_change_confirmed_)
  JSON_DEBUG_VALUE("pilot_overtake_comfirm_siginal", pilot_req.is_overtake_lane_change_confirmed)
  JSON_DEBUG_VALUE("enable_overtake_confirm",
                   config_.enable_overtake_lane_change_confirmation)
  // update history
  if (target_lane_rear_node_) {
    last_target_rear_agent_id_ = target_lane_rear_node_->node_id();
  } else {
    last_target_rear_agent_id_ = -1;
  }
}

bool LaneChangeStateMachineManager::CheckIfProposeLaneChange(
    RequestType* const lane_change_direction,
    RequestSource* const lane_change_type) const {
  *lane_change_direction = lc_req_mgr_->request();
  *lane_change_type = lc_req_mgr_->request_source();

  if ((*lane_change_direction) != NO_CHANGE &&
      *lane_change_type != NO_REQUEST) {
    // const bool is_dashed_line =
    // IsDashLineCurBoundary(*lane_change_direction);

    // bool is_ego_in_perfect_pose = IsLatOffsetValid();
    // JSON_DEBUG_VALUE("is_ego_in_perfect_pose", is_ego_in_perfect_pose)

    bool is_ego_in_perfect_pose = true;

    if (*lane_change_type == EMERGENCE_AVOID_REQUEST ||
        *lane_change_type == CONE_REQUEST) {
      return true;
    } else {
      return is_ego_in_perfect_pose;
    }
  }
  return false;
}

bool LaneChangeStateMachineManager::CheckIfProposeToExecution(
    const RequestType& lane_change_direction,
    const RequestSource& lane_change_type) {
  // if (ego_trajs_future_.empty()) {
  //   return false;
  // }
  const auto& virtual_lane_manager =
      session_->environmental_model().get_virtual_lane_manager();
  const bool has_target_lane =
      virtual_lane_manager->has_lane(lc_req_mgr_->target_lane_virtual_id());
  // check lc gap if feasible
  if (!has_target_lane) {
    lane_change_stage_info_.lc_invalid_reason = "no target lane";
    return false;
  }
  // check target lane curve (execpt emergency lc request)
  bool is_suppress_large_curve = false;
  if(lane_change_type != EMERGENCE_AVOID_REQUEST && lane_change_type != CONE_REQUEST) {
    is_suppress_large_curve = lc_request_.IsCurveSurpressLaneChange(lc_req_mgr_->target_lane_virtual_id());
    if(is_suppress_large_curve) {
      lane_change_stage_info_.lc_invalid_reason = "target lane too large curve";
      return false;
    }
  }
  CheckLaneChangeValid(lane_change_direction);
  const bool is_suppress_LC_short_dis = IsSuppressLCShortDis();
  const bool is_allowed_lc_in_cone_scene =
      lc_req_mgr_->get_int_request_is_allowed_lc_in_cone_scene();
  return has_target_lane && lane_change_stage_info_.gap_insertable &&
         !ego_trajs_future_.empty() && !is_suppress_LC_short_dis && !is_suppress_large_curve &&
         (is_allowed_lc_in_cone_scene || lane_change_type != INT_REQUEST);
}

bool LaneChangeStateMachineManager::CheckIfProposeToCancel(
    const RequestType& lane_change_direction,
    const RequestSource& lane_change_type) {
  const bool is_no_lc_request = (lc_req_mgr_->request() == NO_CHANGE);

  // 当前规划为10hz，则每帧时间大约为0.1s，拨杆变道在等待状态满15s后，还未变道，则取消当前次的拨杆变道
  const double propose_frame_threshold = 150;
  // 针对超车变道，变道等待过程持续超30s则取消当前得超车变道意图
  const double overtake_propose_frame_threshold = 300;

  bool propose_time_out = (transition_info_.lane_change_type == INT_REQUEST &&
                           propose_state_frame_nums_ > propose_frame_threshold) ||
                           (transition_info_.lane_change_type == OVERTAKE_REQUEST &&
                           propose_state_frame_nums_ > overtake_propose_frame_threshold);

  // bool propose_time_out =
  //     lc_req_mgr_->GetReqStartTime(lc_req_mgr_->request_source()) -
  //         IflyTime::Now_s() >
  //     propose_time_threshold;

  bool is_target_lane_merge_to_origin_lane =
      IsNeedCancelLCTargetLaneMergeToOriginLane();
  bool is_no_care_mrege =
      transition_info_.lane_change_type == INT_REQUEST ||
      transition_info_.lane_change_type == EMERGENCE_AVOID_REQUEST ||
      transition_info_.lane_change_type == CONE_REQUEST;
  if (propose_time_out) {
    lane_change_stage_info_.lc_invalid_reason = "propose time out";
  }

  if (is_no_lc_request || propose_time_out ||
      (is_target_lane_merge_to_origin_lane && !is_no_care_mrege)) {
    return true;
  }
  return false;
}

bool LaneChangeStateMachineManager::CheckIfLaneChangeComplete(
    const RequestType& lane_change_direction,
    const RequestSource& lane_change_type) const {
  // const double ego_press_line_ratio = lc_request_.CalculatePressLineRatio(
  //     lc_lane_mgr_->origin_lane_virtual_id(),
  //     lane_change_direction);
  const double ego_press_line_ratio =
      lc_request_.CalculatePressLineRatioByTwoLanes(
          lc_lane_mgr_->origin_lane_virtual_id(),
          lc_lane_mgr_->target_lane_virtual_id(), lane_change_direction);

  // 当前box的一半宽超过车道边界线，认为变道进入complete状态
  if (ego_press_line_ratio > 0.5 && is_side_clear_ &&
  (execution_state_frame_nums_ > 5 || hold_state_frame_nums_ > 5)) {
    return true;
  }

  return false;
}

bool LaneChangeStateMachineManager::CheckIfExecutionToCancel(
    const RequestType& lane_change_direction,
    const RequestSource& lane_change_type) {
  // check if execution time out
  const double time_out_threshold_execute_time_s = 8.0;
  bool execution_time_out =
      TimeOut(true, &lc_timer_.execution_time_count_,
              &lc_timer_.execution_at_time_, time_out_threshold_execute_time_s);

  execution_time_out = false;
  if (execution_time_out) {
    lane_change_stage_info_.lc_back_reason = "time out";
    return true;
  }
  bool is_target_lane_merge_to_origin_lane =
      IsNeedCancelLCTargetLaneMergeToOriginLane();
  bool is_no_care_mrege =
      transition_info_.lane_change_type == INT_REQUEST ||
      transition_info_.lane_change_type == EMERGENCE_AVOID_REQUEST ||
      transition_info_.lane_change_type == CONE_REQUEST;
  if (is_target_lane_merge_to_origin_lane && !is_no_care_mrege) {
    lane_change_stage_info_.lc_back_reason = "target lane is merge region";
    is_high_priority_back_ = true;
    return true;
  }
  // 增加路沿检查,连续3帧才触发cancel
  if (IsLCPathCollisionWithRoadEdge(lc_lane_mgr_->origin_lane_virtual_id(),
                                    lc_lane_mgr_->target_lane_virtual_id(),
                                    transition_info_.lane_change_status)) {
    road_edge_collision_cnt_++;
    if (road_edge_collision_cnt_ >= 3) {
      road_edge_collision_cnt_ = 0;
      lane_change_stage_info_.lc_invalid_reason = "no target lane";
      return true;
    }
  } else {
    road_edge_collision_cnt_ = 0;
  }
  CheckLaneChangeBackValid(lane_change_direction);

  // check if driver cancel
  const bool is_no_lc_request = (lc_req_mgr_->request() == NO_CHANGE);
  if (is_no_lc_request) {
    lane_change_stage_info_.lc_back_reason = "no lc request";
    return true;
  }

  // NOTICE: some cancel needs to check whether lane change can be cancelled
  if (lane_change_stage_info_.lc_should_back) {
    lane_change_stage_info_.is_cancel_to_hold = true;
    is_high_priority_back_ = true;
    return false;
  }

  return false;
}

bool LaneChangeStateMachineManager::CheckIfExecutionToHold(
    const RequestType& lane_change_direction,
    const RequestSource& lane_change_type) {
  if (lane_change_stage_info_.lc_should_back &&
      lane_change_stage_info_.is_cancel_to_hold) {
    return true;
  }

  return false;
}

bool LaneChangeStateMachineManager::CheckIfHoldToCancel(
    const RequestType& lane_change_direction,
    const RequestSource& lane_change_type) {
  // check if hold time out
  const double time_out_threshold_hold_time_s = 1;
  bool hold_time_out =
      TimeOut(true, &lc_timer_.hold_time_count_, &lc_timer_.hold_at_time_,
              time_out_threshold_hold_time_s);
  if (hold_time_out) {
    return true;
  }
  // 增加路沿检查，连续3帧才触发cancel
  if (IsLCPathCollisionWithRoadEdge(lc_lane_mgr_->origin_lane_virtual_id(),
                                    lc_lane_mgr_->target_lane_virtual_id(),
                                    transition_info_.lane_change_status)) {
    road_edge_collision_cnt_++;
    if (road_edge_collision_cnt_ >= 3) {
      road_edge_collision_cnt_ = 0;
      lane_change_stage_info_.lc_invalid_reason = "no target lane";
      return true;
    }
  } else {
    road_edge_collision_cnt_ = 0;
  }
  return false;
}

bool LaneChangeStateMachineManager::CheckIfHoldToExecution(
    const RequestType& lane_change_direction,
    const RequestSource& lane_change_type) {
  if (ego_trajs_future_.empty()) {
    return false;
  }
  const auto& virtual_lane_manager =
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

  bool is_high_priority_complete_mlc = IsHighPriorityCompleteMLC();

  // 防止自车因为横向不居中等情况，导致一直卡在complete状态。所以在complete状态下超过3s后，结束当前状态
  bool is_time_out = complete_state_frame_nums_ > 30;

  if (target_lane_front_node_) {
    lane_change_stage_info_.lc_gap_info.front_node_id =
        target_lane_front_node_->node_id();
  }

  if (target_lane_rear_node_) {
    lane_change_stage_info_.lc_gap_info.rear_node_id =
        target_lane_rear_node_->node_id();
  }

  if (perfect_in_lane || is_high_priority_complete_mlc || is_time_out) {
    ILOG_DEBUG << "perfect_in_lane!!!";
    return true;
  }

  return false;
}

bool LaneChangeStateMachineManager::CheckIfInPerfectLaneKeeping() const {
  const auto& virtual_lane_mgr =
      session_->mutable_environmental_model()->get_virtual_lane_manager();
  const auto& clane_virtual_id = virtual_lane_mgr->current_lane_virtual_id();
  const double v_ego =
      session_->mutable_environmental_model()->get_ego_state_manager()->ego_v();
  std::vector<double> xp_v_ego{10.0, 15.0, 20.0, 25.0};
  double dist_threshold = interp(v_ego, xp_v_ego, config_.lc_finished_dist_thr);

  // 1、大曲率阈值增大一点
  std::shared_ptr<ReferencePathManager> reference_path_mgr =
      session_->mutable_environmental_model()->get_reference_path_manager();
  const auto& flane_virtual_id = lc_lane_mgr_->fix_lane_virtual_id();
  const auto& target_reference_path =
      reference_path_mgr->make_map_lane_reference_path(flane_virtual_id);
  const auto& ego_s = target_reference_path->get_frenet_ego_state().s();
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

  // 2、匝道汇主路打灯时，关灯阈值可以增大一点
  if (road_to_ramp_turn_signal_ != RAMP_NONE) {
    dist_threshold = 0.3;
  }

  // 3、考虑向避让的情况
  const double lateral_offset = session_->planning_context()
                                    .lateral_offset_decider_output()
                                    .lateral_offset;
  dist_threshold = dist_threshold + std::abs(lateral_offset);

  const auto& ego_boundary =
      target_reference_path->get_frenet_ego_state().boundary();
  double lateral_dist_threshold = 1.5;
  ReferencePathPoint cur_pos_path_point;
  if (target_reference_path->get_reference_point_by_lon(ego_s,
                                                        cur_pos_path_point)) {
    lateral_dist_threshold =
        std::max(0.5 * cur_pos_path_point.lane_width, dist_threshold);
  }

  std::vector<double> angle_thre_v{0.1047, 0.1047, 0.0698, 0.0524, 0.0349};
  std::vector<double> angle_thre_bp{1.0, 4.167, 8.333, 12.5, 16.667};
  double angle_threshold = interp(v_ego, angle_thre_bp, angle_thre_v);

  const auto& current_reference_path =
      reference_path_mgr->get_reference_path_by_lane(clane_virtual_id);
  const auto& frenet_coord = current_reference_path->get_frenet_coord();
  const auto& frenet_ego_state = current_reference_path->get_frenet_ego_state();
  const auto& planning_init_point = frenet_ego_state.planning_init_point();
  double diff_heading_angle = planning_math::NormalizeAngle(
      planning_init_point.heading_angle -
      frenet_coord->GetPathCurveHeading(planning_init_point.frenet_state.s));
  const auto& general_lateral_decider_output =
      session_->planning_context().general_lateral_decider_output();
  bool perfect_in_lane = false;
  // perfect_in_lane =
  //     ((std::fabs(frenet_ego_state.l()) < dist_threshold) &&
  //      (std::fabs(frenet_ego_state.heading_angle()) < angle_threshold));
  perfect_in_lane =
      (((std::fabs(planning_init_point.frenet_state.r) < dist_threshold) ||
        (std::fabs(ego_boundary.l_start) < lateral_dist_threshold &&
         std::fabs(ego_boundary.l_end) < lateral_dist_threshold)) &&
       (std::fabs(diff_heading_angle) < angle_threshold));
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

void LaneChangeStateMachineManager::LaneChangeDecisionInfoReset() {
  transition_info_.Rest();
  lc_timer_.Reset();
}

void LaneChangeStateMachineManager::CheckLaneChangeValid(
    RequestType direction) {
  // check single frame lc gap if feasible
  lane_change_stage_info_ = CheckLCGapFeasible(direction);
  const bool is_dash_line_safe = !IsLCPathCollisionWithSolidLine(
      lc_lane_mgr_->origin_lane_virtual_id(),
      transition_info_.lane_change_status, transition_info_.lane_change_type,
      transition_info_.lane_change_direction);
  const bool is_roadedge_safe =
      !IsLCPathCollisionWithRoadEdge(lc_lane_mgr_->origin_lane_virtual_id(),
                                     lc_lane_mgr_->target_lane_virtual_id(),
                                     transition_info_.lane_change_status);
  const bool is_targetlane_valid = CheckTargetLaneValid();

  bool is_lc_condition_satisfied = false;
  int lc_valid_thre = 4;
  if (transition_info_.lane_change_type == EMERGENCE_AVOID_REQUEST ||
      transition_info_.lane_change_type == CONE_REQUEST) {
    lc_valid_thre = 1;
    is_lc_condition_satisfied = is_roadedge_safe;  // 避让请求和锥桶请求不需要dash足够
  } else if (transition_info_.lane_change_type == MERGE_REQUEST ||
    transition_info_.lane_change_type == DYNAMIC_AGENT_EMERGENCE_AVOID_REQUEST) {
    lc_valid_thre = 1;
    is_lc_condition_satisfied =
        is_dash_line_safe && is_targetlane_valid && is_roadedge_safe;
  } else {
    is_lc_condition_satisfied =
        is_dash_line_safe && is_targetlane_valid && is_roadedge_safe;
  }

  // can lc if more than continue 4 frame gap_insertable
  if (lane_change_stage_info_.gap_insertable && is_lc_condition_satisfied) {
    lc_valid_cnt_ += 1;
    ILOG_DEBUG << "decide_lc_valid_info lc_valid_cnt :" << lc_valid_cnt_;
    if (lc_valid_cnt_ > lc_valid_thre) {
      lc_valid_cnt_ = 0;
      lane_change_stage_info_.lc_valid = true;
    } else {
      lane_change_stage_info_.gap_insertable = false;
      lane_change_stage_info_.lc_invalid_reason = "valid cnt below threshold";
    }
  } else {
    ILOG_DEBUG << "arbitrator lc invalid reason "
               << lane_change_stage_info_.lc_invalid_reason.c_str();
    lane_change_stage_info_.gap_insertable = false;
    lc_valid_cnt_ = 0;
    if (!is_lc_condition_satisfied) {
      if (!is_roadedge_safe || !is_targetlane_valid) {
        lane_change_stage_info_.lc_invalid_reason = "no target lane";
      } else {
        lane_change_stage_info_.lc_invalid_reason = "dash not enough";
      }
    }
  }
}

LaneChangeStageInfo LaneChangeStateMachineManager::CheckLCGapFeasible(
    RequestType direction) {
  assert(direction == LEFT_CHANGE || direction == RIGHT_CHANGE);

  LaneChangeStageInfo lc_state_info;

  const auto& virtual_lane_manager =
      session_->mutable_environmental_model()->get_virtual_lane_manager();

  const auto target_lane = virtual_lane_manager->get_lane_with_virtual_id(
      lc_req_mgr_->target_lane_virtual_id());

  if (target_lane == nullptr || ego_trajs_future_.empty()) {
    return lc_state_info;
  }

  lc_state_info.gap_insertable = true;
  lc_invalid_track_.reset();

  // 计算安全性判断的时间
  lc_safety_check_time_ = CalculateLCSafetyCheckTime();

  // 在安全性判断的时候留一个余量，保证在complete状态之前能一直做安全性检查
  lc_safety_check_num_ = static_cast<int>(lc_safety_check_time_ / 0.2) + 1;

  // get ego future trajectorys
  // ego_trajs_future_ = CalculateEgoFutureTrajs();
  // get ego future trajs by pp idm
  // ego_trajs_future_ = CalculateEgoPPIDMTrajs();
  for (int i = 0; i <= lc_safety_check_num_; ++i) {  // 当前只进行了s检查
    lc_time_vec_.emplace_back(i * 0.2);
    lc_egos_vec_.emplace_back(ego_trajs_future_[i].s - ego_trajs_future_[0].s);
  }

  // 判断与各障碍物之间的gap是否安全
  // if (target_lane_middle_node_) {
  //   lc_invalid_track_.set_value(
  //       target_lane_middle_node_->node_agent_id(),
  //       target_lane_middle_node_->node_back_edge_to_ego_front_edge_distance(),
  //       target_lane_middle_node_->node_speed());
  //   lc_state_info.gap_insertable = false;
  //   lc_state_info.lc_invalid_reason = "front view invalid";
  //   lc_state_info.lc_gap_info.front_node_id =
  //       target_lane_middle_node_->node_id();
  //   return lc_state_info;
  // }

  if (target_lane_front_node_) {
    if (FacilityTypes.count(target_lane_front_node_->type()) > 0) {
      const int target_lane_virtual_id = lc_req_mgr_->target_lane_virtual_id();
      if (!IsLCFeasibleForTrafficConeInTargetLane(target_lane_front_node_,
                                                  target_lane_virtual_id)) {
        lc_invalid_track_.set_value(
            target_lane_front_node_->node_agent_id(),
            target_lane_front_node_
                ->node_back_edge_to_ego_front_edge_distance(),
            target_lane_front_node_->node_speed());
        lc_state_info.gap_insertable = false;
        lc_state_info.lc_invalid_reason = "front view invalid";
      }
    } else {
      CalculateLCGapFeasibleWithPredictionInfo(
          &lc_state_info, target_lane_front_node_, true, false);
    }

    if (!lc_state_info.gap_insertable) {
      return lc_state_info;
    }
  }

  if (target_lane_rear_node_) {
    CalculateLCGapFeasibleWithPredictionInfo(
        &lc_state_info, target_lane_rear_node_, false, false);
    if (!lc_state_info.gap_insertable) {
      return lc_state_info;
    }
  }

  if(!risk_side_agents_nodes_.empty()) {
    for (const auto& side_obs : risk_side_agents_nodes_) {
      if (side_obs == nullptr) {
            continue;
      }
      // bool is_large = IsLargeAgent(side_obs);// 判断是否大车
      bool is_front_car = side_obs->node_to_ego_distance() > 0.0; // node s 在自车前还是后
      CalculateLCGapFeasibleWithPredictionInfo(&lc_state_info, side_obs, is_front_car, false, true);
      if (!lc_state_info.gap_insertable) {
        return lc_state_info;
      }
    }
  }
  if (ego_lane_front_node_) {
    if (FacilityTypes.count(ego_lane_front_node_->type()) > 0) {
      if (!IsLCFeasibleForTrafficCone(ego_lane_front_node_)) {
        lc_invalid_track_.set_value(
            ego_lane_front_node_->node_agent_id(),
            ego_lane_front_node_->node_back_edge_to_ego_front_edge_distance(),
            ego_lane_front_node_->node_speed());
        lc_state_info.gap_insertable = false;
        lc_state_info.lc_invalid_reason = "front view invalid";
      }
    }
    //  else {
    //   CalculateLCGapFeasibleWithPredictionInfo(&lc_state_info,
    //                                          ego_lane_front_node_, true,
    //                                          true);
    // }
    if (!lc_state_info.gap_insertable) {
      return lc_state_info;
    }
  }
  return lc_state_info;
}
void LaneChangeStateMachineManager::CheckOtherAgents(
  LaneChangeStageInfo* const lc_state_info) {
// int risk_front_agent_id = -1;
// int risk_side_agent_id = -1;
// bool lc_safety = true;
// bool is_risk_node = false;
// bool is_side_closing = false;
// // === 前方检查 ===
// for (const auto* risk_node : risk_agents_nodes_) {
//   if (risk_node == nullptr) {
//     continue;
//   }
//   bool is_large = IsLargeAgent(risk_node);
//   bool is_risk = !CheckFrontRiskAgentTrajs(risk_node, is_large);
//   if (is_risk) {
//     lc_safety = false;
//     is_risk_node = true;
//     risk_front_agent_id = risk_node->node_agent_id();
//     break;
//   }
// }

// const int target_lane_virtual_id = lc_req_mgr_->target_lane_virtual_id();
// const auto reference_path_manager =
//     session_->environmental_model().get_reference_path_manager();
// const auto virtual_lane_manager =
//     session_->environmental_model().get_virtual_lane_manager();

// if (virtual_lane_manager == nullptr || reference_path_manager == nullptr) {
//   return;
// }

// const auto target_lane =
//     virtual_lane_manager->get_lane_with_virtual_id(target_lane_virtual_id);
// if (target_lane == nullptr) {
//   return;
// }
// const double half_width_by_target = target_lane->width() * 0.5;

// const auto& vehicle_param =
//     VehicleConfigurationContext::Instance()->get_vehicle_param();
// const double half_width = vehicle_param.max_width * 0.5;
// const double single_risk_buff = 1.0;

// const auto target_reference_path =
//     reference_path_manager->get_reference_path_by_lane(
//         target_lane_virtual_id);

// const auto& ego_sl_state = target_reference_path->get_frenet_ego_state();
// const auto& ego_sl_bd = target_reference_path->get_ego_frenet_boundary();

// std::pair<double, double> ego_lat{ego_sl_bd.l_start - single_risk_buff,
//                                   ego_sl_bd.l_end + single_risk_buff};
// std::pair<double, double> ego_lon{ego_sl_bd.s_start - single_risk_buff,
//                                   ego_sl_bd.s_end + single_risk_buff};

// double ego_lat_vel = ego_sl_state.velocity_l();
// double ego_lon_vel = ego_sl_state.velocity_s();
// // === 侧边车辆安全检查 ===
// for (const auto& side_obs : risk_side_agents_nodes_) {
//   const auto& obstacle_sl = side_obs->frenet_obstacle_boundary();
//   std::pair<double, double> obs_lat{obstacle_sl.l_start, obstacle_sl.l_end};
//   std::pair<double, double> obs_lon{obstacle_sl.s_start, obstacle_sl.s_end};
//   double obs_lat_vel = side_obs->frenet_velocity_l();
//   double obs_lon_vel = side_obs->frenet_velocity_s();
//   // frenet_velocity_lateral 横向是否在明显靠近：
//   //   - 符号 > 0 代表远离
//   //   - 符号 < 0 代表靠近
//   //   - 正常直行允许轮胎压线
//   // double tolerance_buff =
//   //     (side_obs->frenet_velocity_lateral() < -0.1) ? 0.0 : 0.20;

//   // std::pair<double, double> center_lat{-half_width_by_target +
//   // tolerance_buff,
//   //                                      half_width_by_target -
//   //                                      tolerance_buff};

//   // bool lateral_collision = IfFrenetCollision(
//   //     center_lat, 0.0, obs_lat, obs_lat_vel, 4.0, 1.0);  // 目标车 4s 横向
//   // bool ego_collision = IfFrenetCollision(
//   //     ego_lat, ego_lat_vel, obs_lat, obs_lat_vel, 4.0, 1.0);  // 两车 4s
//   //     横向
//   bool sl_collision =
//       IfFrenetCollision2D(ego_lon, ego_lon_vel, obs_lon, obs_lon_vel, ego_lat,
//                           ego_lat_vel, obs_lat, obs_lat_vel, 4.0, 0.5);
//   if (sl_collision) {
//     lc_safety = false;
//     is_side_closing = true;
//     risk_side_agent_id = side_obs->obstacle()->id();
//     break;
//   }
// }

// // 输出失败id
// if (!lc_safety) {
//   if (transition_info_.lane_change_status == kLaneChangeExecution) {
//     lc_state_info->lc_should_back = true;

//     if (is_risk_node) {
//       lc_state_info->lc_back_reason = "front risk node";
//       lc_back_track_.set_value(risk_front_agent_id, 0.0, 0.0);
//     }
//     if (is_side_closing) {
//       lc_state_info->lc_back_reason = "side closing";
//       lc_back_track_.set_value(risk_side_agent_id, 0.0, 0.0);
//     }

//   } else if (transition_info_.lane_change_status == kLaneChangePropose ||
//              transition_info_.lane_change_status == kLaneChangeHold) {
//     lc_state_info->gap_insertable = false;

//     if (is_risk_node) {
//       lc_state_info->lc_invalid_reason = "front risk node";
//       lc_invalid_track_.set_value(risk_front_agent_id, 0.0, 0.0);
//     }
//     if (is_side_closing) {
//       lc_state_info->lc_invalid_reason = "side closing";
//       lc_invalid_track_.set_value(risk_side_agent_id, 0.0, 0.0);
//     }
//   }
// }
}
void LaneChangeStateMachineManager::CheckMergingRearAgent(
    LaneChangeStageInfo* const lc_state_info) {
  bool lc_safety = true;
  lc_safety = CheckMergingRearAgentTraj(merging_rear_agent_id_);
  // 输出失败id
  if (!lc_safety) {
    if (transition_info_.lane_change_status == kLaneChangeExecution) {
      lc_state_info->lc_should_back = true;
      lc_state_info->lc_back_reason = "merging rear agent";
      lc_back_track_.set_value(merging_rear_agent_id_, 0.0, 0.0);
    } else if (transition_info_.lane_change_status == kLaneChangePropose ||
               transition_info_.lane_change_status == kLaneChangeHold) {
      lc_state_info->gap_insertable = false;
      lc_state_info->lc_invalid_reason = "merging rear agent";
      lc_invalid_track_.set_value(merging_rear_agent_id_, 0.0, 0.0);
    }
  }
}
void LaneChangeStateMachineManager::CheckLaneChangeBackValid(
    RequestType direction) {
  lane_change_stage_info_ = CheckIfNeedLCBack(direction);
  int lc_back_thre = 2;
  // 对于明显over take的 后车，减少计数次数
  if(lane_change_stage_info_.lc_back_reason == "side view back"
    && rear_agent_overtaking_) {
    lc_back_thre = 1;
  }
  if (lane_change_stage_info_.lc_should_back) {
    lc_back_cnt_ += 1;
    if (lc_back_cnt_ <= lc_back_thre) {
      lane_change_stage_info_.lc_should_back = false;
      lane_change_stage_info_.lc_back_reason = "but back cnt below threshold";
    } else {
      lc_back_cnt_ = 0;
    }
  } else {
    lc_back_cnt_ = 0;
  }
}

LaneChangeStageInfo LaneChangeStateMachineManager::CheckIfNeedLCBack(
    RequestType direction) {
  assert(direction == LEFT_CHANGE || direction == RIGHT_CHANGE);

  LaneChangeStageInfo lc_state_info;

  const auto& virtual_lane_manager =
      session_->mutable_environmental_model()->get_virtual_lane_manager();

  const auto target_lane = virtual_lane_manager->get_lane_with_virtual_id(
      lc_req_mgr_->target_lane_virtual_id());

  if (target_lane == nullptr) {
    lc_state_info.lc_should_back = true;
    lc_state_info.lc_back_reason = "dash not enough";
    return lc_state_info;
  }

  lc_state_info.lc_should_back = false;
  lc_invalid_track_.reset();

  // 计算安全性判断的时间
  lc_safety_check_time_ = CalculateLCSafetyCheckTime();
  // lc_safety_check_time_ = 4.5;

  // 在安全性判断的时候留一个余量，保证在complete状态之前能一直做安全性检查
  lc_safety_check_num_ = static_cast<int>(lc_safety_check_time_ / 0.2) + 1;

  // get ego future trajectorys
  // ego_trajs_future_ = CalculateEgoFutureTrajs();
  // ego_trajs_future_ = CalculateEgoPPIDMTrajs();

  // store debug_info
  const auto& ego_v =
      session_->environmental_model().get_ego_state_manager()->ego_v();

  for (int i = 0; i <= lc_safety_check_num_; ++i) {
    lc_time_vec_.emplace_back(i * 0.2);
    lc_egos_vec_.emplace_back(ego_trajs_future_[i].s - ego_trajs_future_[0].s);
  }

  // 判断与各障碍物之间的gap是否安全
  // if (target_lane_middle_node_) {
  //   lc_state_info.lc_should_back = true;
  //   lc_state_info.lc_back_reason = "front view back";
  //   lc_back_track_.set_value(
  //       target_lane_middle_node_->node_agent_id(),
  //       target_lane_middle_node_->node_back_edge_to_ego_front_edge_distance(),
  //       target_lane_middle_node_->node_speed());
  //   return lc_state_info;
  // }

  if (target_lane_front_node_) {
    CalculateLCGapFeasibleWithPredictionInfo(
        &lc_state_info, target_lane_front_node_, true, false);
    if (lc_state_info.lc_should_back) {
      return lc_state_info;
    }
  }

  if (target_lane_rear_node_) {
    CalculateLCGapFeasibleWithPredictionInfo(  // 第2个 false， 对应
                                               // is_ego_lane_agent
        &lc_state_info, target_lane_rear_node_, false, false);
    if (lc_state_info.lc_should_back) {
      return lc_state_info;
    }
  }
  if(!risk_side_agents_nodes_.empty()) {
    for (const auto& side_obs : risk_side_agents_nodes_) {
      if (side_obs == nullptr) {
            continue;
      }
      // bool is_large = IsLargeAgent(side_obs);// 判断是否大车
      bool is_front_car = side_obs->node_to_ego_distance() > 0.0; // node s 在自车前还是后
      CalculateLCGapFeasibleWithPredictionInfo(&lc_state_info, side_obs, is_front_car, false, true);
      if (lc_state_info.lc_should_back) {
        return lc_state_info;
      }
    }
  }
  return lc_state_info;
}

void LaneChangeStateMachineManager::UpdateCoarsePlanningInfo() {
  // ego_trajs_future_ = CalculateEgoPPIDMTrajs();//tmp output traj in complete
  // status

  // Step 1) 计算state
  auto& coarse_planning_info = session_->mutable_planning_context()
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
  coarse_planning_info.int_lane_change_cmd = lc_req_mgr_->lane_change_cmd();
  coarse_planning_info.reference_path =
      session_->mutable_environmental_model()
          ->get_reference_path_manager()
          ->make_map_lane_reference_path(coarse_planning_info.target_lane_id);
  if (coarse_planning_info.reference_path == nullptr) {
    return;
  }
  const auto& planning_init_point =
      coarse_planning_info.reference_path->get_frenet_ego_state()
          .planning_init_point();
  // Step 3) calculate trajectory points
  // generate reference path
  // static const double min_ego_v_cruise = 2.0;
  const auto& v_ref_cruise = std::fmax(
      session_->environmental_model().get_ego_state_manager()->ego_v_cruise(),
      config_.min_ego_v_cruise);

  static const size_t& N = config_.num_point;
  const auto& delta_time = config_.delta_t;

  auto& cart_ref_info = coarse_planning_info.cart_ref_info;
  const auto& frenet_coord =
      coarse_planning_info.reference_path->get_frenet_coord();

  // double s = 0.0;
  // Point2D frenet_pt{s, 0.0};
  // Point2D cart_pt(0.0, 0.0);
  const auto& ref_point = coarse_planning_info.reference_path->get_points();
  auto point_size = ref_point.size();
  cart_ref_info.x_vec.resize(point_size);
  cart_ref_info.y_vec.resize(point_size);
  cart_ref_info.k_vec.resize(point_size);
  cart_ref_info.s_vec.resize(point_size);
  std::vector<double> kappa_radius_vec;
  kappa_radius_vec.resize(point_size);
  double normal_care_spline_length = 50.;
  const float preview_time = 20.;
  const double min_preview_spline_length = 20.;

  if (session_->is_hpp_scene()) {
    const double kHppMaxRearDistance = 25.0;
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
  normal_care_spline_length = std::max(planning_init_point.frenet_state.s, normal_care_spline_length);
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
    cart_ref_info.k_vec[i] = ref_point.at(i).path_point.kappa();
    kappa_radius_vec[i] = std::min(
        std::max(1.0 / (cart_ref_info.k_vec[i] + 1e-6), -10000.0), 10000.0);
    if (!session_->is_rads_scene() &&
        cart_ref_info.s_vec[i] >
            normal_care_spline_length + std::max(v_ref_cruise * preview_time,
                                                 min_preview_spline_length)) {
      cart_ref_info.x_vec.resize(i);
      cart_ref_info.y_vec.resize(i);
      cart_ref_info.k_vec.resize(i);
      cart_ref_info.s_vec.resize(i);
      kappa_radius_vec.resize(i);
      break;
    }
  }

  cart_ref_info.x_s_spline.set_points(cart_ref_info.s_vec, cart_ref_info.x_vec);
  cart_ref_info.y_s_spline.set_points(cart_ref_info.s_vec, cart_ref_info.y_vec);
  if (cart_ref_info.k_vec.size() > 2) {
    cart_ref_info.k_vec[0] = cart_ref_info.k_vec[1];
    cart_ref_info.k_vec[cart_ref_info.k_vec.size() - 1] =
        cart_ref_info.k_vec[cart_ref_info.k_vec.size() - 2];
  }
  cart_ref_info.k_s_spline.set_points(cart_ref_info.s_vec, cart_ref_info.k_vec,
                                      pnc::mathlib::spline::linear);

  JSON_DEBUG_VECTOR("raw_refline_x_vec", cart_ref_info.x_vec, 2)
  JSON_DEBUG_VECTOR("raw_refline_y_vec", cart_ref_info.y_vec, 2)
  JSON_DEBUG_VECTOR("raw_refline_s_vec", cart_ref_info.s_vec, 2)
  JSON_DEBUG_VECTOR("raw_refline_k_vec", kappa_radius_vec, 2)

  Eigen::Vector2d init_pos(planning_init_point.lat_init_state.x(),
                           planning_init_point.lat_init_state.y());
  const auto& ego_state =
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
  const double end_extension_length = session_->is_hpp_scene() ? 10.0 : 0.0;
  if (frenet_coord->XYToSL(cart_init_pt, frenet_init_pt, end_extension_length)) {
    s_ref = frenet_init_pt.x;
  } else {
    coarse_planning_info.reference_path.reset();
    ILOG_DEBUG << "kd_path coordinate conversion init_pos failed";
  }

  const auto& frenet_length = frenet_coord->Length();

  s_ref = std::min(s_ref, frenet_length * 0.95);
  const double delta_s = frenet_length - s_ref;
  const double v_cruise_scale = std::min(delta_s / (v_ref_cruise * 5.0), 1.0);
  session_->mutable_planning_context()->set_v_ref_cruise(v_cruise_scale *
                                                         v_ref_cruise);

  coarse_planning_info.trajectory_points.clear();
  TrajectoryPoint point;
  double last_valid_s = 0.0;
  double last_valid_l = 0.0;
  bool first_point = true;

  for (size_t i = 0; i < N; ++i) {
    if (s_ref < cart_ref_info.s_vec.back() + kEps) {
      point.x = cart_ref_info.x_s_spline(s_ref);
      point.y = cart_ref_info.y_s_spline(s_ref);
      point.heading_angle =
          std::atan2(cart_ref_info.y_s_spline.deriv(1, s_ref),
                     cart_ref_info.x_s_spline.deriv(1, s_ref));
    } else {
      point.x = cart_ref_info.x_s_spline(cart_ref_info.s_vec.back());
      point.y = cart_ref_info.y_s_spline(cart_ref_info.s_vec.back());
      point.heading_angle = std::atan2(
          cart_ref_info.y_s_spline.deriv(1, cart_ref_info.s_vec.back()),
          cart_ref_info.x_s_spline.deriv(1, cart_ref_info.s_vec.back()));
    }

    Point2D frenet_pt{0.0, 0.0};
    Point2D cart_pt(point.x, point.y);
    bool projection_success = frenet_coord->XYToSL(cart_pt, frenet_pt);

    if (projection_success) {
      point.s = frenet_pt.x;
      point.l = frenet_pt.y;

      if (!first_point && point.s <= last_valid_s) {
        point.s = last_valid_s + 0.01;
      }

      last_valid_s = point.s;
      last_valid_l = point.l;
    } else {
      if (first_point) {
        point.s = 0.0;
        point.l = 0.0;
      } else {
        point.s = last_valid_s + v_cruise_scale * v_ref_cruise * delta_time;
        point.l = last_valid_l;
      }

      last_valid_s = point.s;
      last_valid_l = point.l;
    }

    point.t = static_cast<double>(i) * delta_time;
    coarse_planning_info.trajectory_points.emplace_back(point);

    s_ref += v_cruise_scale * v_ref_cruise * delta_time;
    first_point = false;
  }
}

void LaneChangeStateMachineManager::GenerateStateMachineOutput() {
  auto& lane_change_decider_output = session_->mutable_planning_context()
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
  // lane_change_decider_output.act_request_source =
  //     lc_req_mgr_->act_request_source();
  lane_change_decider_output.lc_turn_light = lc_req_mgr_->turn_signal();

  lane_change_decider_output.lc_invalid_track = lc_invalid_track_;
  lane_change_decider_output.lc_back_track = lc_back_track_;
  lane_change_decider_output.lc_valid_cnt = lc_valid_cnt_;
  lane_change_decider_output.lc_back_cnt = lc_back_cnt_;
  lane_change_decider_output.is_dash_not_enough_for_lc =
      is_dash_not_enough_for_lc_;

  lane_change_decider_output.is_ego_on_leftmost_lane = is_ego_on_leftmost_lane_;
  lane_change_decider_output.is_ego_on_rightmost_lane =
      is_ego_on_rightmost_lane_;
  if (target_lane_front_node_) {
    lane_change_stage_info_.lc_gap_info.front_node_id =
            target_lane_front_node_->node_id();
  } else {
    lane_change_stage_info_.lc_gap_info.front_node_id = -1;
  }
  if (target_lane_rear_node_) {
    lane_change_stage_info_.lc_gap_info.rear_node_id =
        target_lane_rear_node_->node_id();
  } else {
    lane_change_stage_info_.lc_gap_info.rear_node_id = -1;
  }
  if (ego_lane_front_node_) {
    lane_change_decider_output.origin_agent_id =
        ego_lane_front_node_->node_agent_id();
  } else {
    lane_change_decider_output.origin_agent_id = -1;
  }
  lane_change_decider_output.lc_gap_info.front_node_id =
      lane_change_stage_info_.lc_gap_info.front_node_id;
  lane_change_decider_output.lc_gap_info.rear_node_id =
      lane_change_stage_info_.lc_gap_info.rear_node_id;
  lane_change_decider_output.origin_agent_id =
      lane_change_decider_output.origin_agent_id;

  GenerateTurnSignalForSplitRegion();
  JSON_DEBUG_VALUE("road_to_ramp_turn_signal",
                   static_cast<int>(road_to_ramp_turn_signal_));
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

  if (lc_req_mgr_->request_source() == MAP_REQUEST) {
    const auto& virtual_lane_mgr =
        session_->environmental_model().get_virtual_lane_manager();
    lane_change_decider_output.is_nearing_ramp =
        virtual_lane_mgr->get_current_lane()->is_nearing_ramp_mlc_task();
  }

  if (session_->is_hpp_scene()) {
    lane_change_decider_output.hpp_turn_signal = CalculaTurnSignalForHPP();
  } else {
    lane_change_decider_output.hpp_turn_signal = NO_CHANGE;
  }
  JSON_DEBUG_VALUE("HPP turn signal",
                   int(lane_change_decider_output.hpp_turn_signal))

  lane_change_decider_output.lc_hold_state_lat_offset =
      lc_hold_state_lat_offset_;

  lane_change_decider_output.is_high_priority_back = is_high_priority_back_;
  lane_change_decider_output.is_aggressive_scence = is_aggressive_scence_;
  // idm
  if (!ego_trajs_future_.empty()) {
    lane_change_decider_output.ego_trajs_future = ego_trajs_future_;
  } else {
    lane_change_decider_output.ego_trajs_future.clear();
  }
  if (lane_change_decider_output.curr_state == kLaneKeeping &&
      last_state_ != kLaneKeeping &&
      split_selecting_info_.split_selecting_status == kNonSelecting) {
    ResetStateMachine();
  }
  const double ego_press_line_ratio =
    lc_request_.CalculatePressLineRatioByTwoLanes(
        lc_lane_mgr_->origin_lane_virtual_id(),
        lc_lane_mgr_->target_lane_virtual_id(),
        transition_info_.lane_change_direction);
  lane_change_decider_output.ego_press_line_ratio = ego_press_line_ratio;
  JSON_DEBUG_VALUE("lc_ego_press_line_ratio", ego_press_line_ratio);  // 更新压线率

  lane_change_decider_output.split_selecting_status = split_selecting_info_.split_selecting_status;
  lane_change_decider_output.split_select_direction = split_selecting_info_.split_select_direction;
  JSON_DEBUG_VALUE("split_selecting_status", static_cast<int>(split_selecting_info_.split_selecting_status));
  JSON_DEBUG_VALUE("selecting_origin_order_id", split_selecting_info_.origin_lane_order_id);
  JSON_DEBUG_VALUE("selecting_selected_order_id", split_selecting_info_.selected_lane_order_id);
  JSON_DEBUG_VALUE("selecting_unfinished_reason", static_cast<int>(split_selecting_info_.unfinished_reason));
  // bool is_warning_collision_risk =
  // transition_info_.lane_change_status == kLaneChangeHold ||
  // transition_info_.lane_change_status == kLaneChangeCancel||
  // (transition_info_.lane_change_status == kLaneChangeExecution && lc_back_cnt_ > 0);
  // if(is_warning_collision_risk) {
  //   lane_change_decider_output.is_collision_risk = true;
  // } else {
  //   lane_change_decider_output.is_collision_risk = false;
  // }
  const int cur_state = transition_info_.lane_change_status;
  JSON_DEBUG_VALUE("cur_state", cur_state)
}
bool LaneChangeStateMachineManager::CalculateSideGapFeasible(
    const planning_data::DynamicAgentNode* const agent) {
  const double v_ego =
      session_->environmental_model().get_ego_state_manager()->ego_v();
  const double node_v = agent->node_speed();
  const double node_a = agent->node_accel();
  const double distance_rel =
      agent->node_front_edge_to_ego_back_edge_distance();

  const double need_safety_dist = planning::CalcGapObjSafeDistance(
      v_ego, node_v, node_a, is_large_car_in_side_, false);

  lc_rear_objs_vec_ =
      GetObjsDebugInfo(node_v, node_a, kEgoReachBoundaryTime, -distance_rel);
  if (distance_rel < need_safety_dist) {
    return false;
  }
  // if (distance_rel < need_safety_dist) {
  //   lc_invalid_track_.set_value(target_lane_rear_node_->node_agent_id(),
  //                               -distance_rel, node_v);
  //   lc_state_info->gap_insertable = false;
  //   lc_state_info->lc_invalid_reason = "side view invalid";
  // }
  return true;
}
void LaneChangeStateMachineManager::CalculateFrontGapFeasible(
    LaneChangeStageInfo* const lc_state_info) {
  const double v_ego =
      session_->environmental_model().get_ego_state_manager()->ego_v();
  const double distance_rel =
      target_lane_front_node_->node_back_edge_to_ego_front_edge_distance();
  const double node_v = target_lane_front_node_->node_speed();
  const double node_a = target_lane_front_node_->node_accel();

  const double target_lane_need_safety_dist =
      planning::CalcGapObjSafeDistance(v_ego, node_v, node_a, false, true);
  lc_front_objs_tar_lane_vec_ =
      GetObjsDebugInfo(node_v, node_a, kEgoReachBoundaryTime, distance_rel);
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

    if (FacilityTypes.count(ego_lane_front_node_->type()) > 0) {
      if (!IsLCFeasibleForTrafficCone(ego_lane_front_node_)) {
        lc_invalid_track_.set_value(ego_lane_front_node_->node_agent_id(),
                                    ego_lane_distance_rel, ego_front_node_v);
        lc_state_info->gap_insertable = false;
        lc_state_info->lc_invalid_reason = "front view invalid";
      }
    } else {
      const double ego_lane_need_safety_dist = planning::CalcGapObjSafeDistance(
          v_ego, ego_front_node_v, ego_front_node_a, false, true);
      if (ego_lane_distance_rel < ego_lane_need_safety_dist) {
        lc_invalid_track_.set_value(ego_lane_front_node_->node_agent_id(),
                                    ego_lane_distance_rel, ego_front_node_v);
        lc_state_info->gap_insertable = false;
        lc_state_info->lc_invalid_reason = "front view invalid";
      }
    }
    lc_front_objs_ego_lane_vec_ =
        GetObjsDebugInfo(ego_front_node_v, ego_front_node_a,
                         kEgoReachBoundaryTime, ego_lane_distance_rel);
  }
}
bool LaneChangeStateMachineManager::CalculateSideAreaIsSafetyExecution(
    const planning_data::DynamicAgentNode* const agent) {
  const auto& ego_state =
      session_->environmental_model().get_ego_state_manager();
  const double v_ego = ego_state->ego_v();
  const double v_node = agent->node_speed();
  const double a_node = agent->node_accel();
  const double distance_rel =
      agent->node_front_edge_to_ego_back_edge_distance();
  // 在安全性gap的判断中是预计自车中心3s可到车道边界线。
  const int total_frames_in_execution_state = kEgoReachBoundaryTime * 10;

  // 如果以当前的加速度为基准，预测剩余时间后是否会发生碰撞
  const double t_remain_lc =
      (total_frames_in_execution_state - execution_state_frame_nums_) * 0.1;
  if (t_remain_lc > 0) {
    const double obstacle_dist_remain =
        0.5 * a_node * t_remain_lc * t_remain_lc + v_node * t_remain_lc;
    const double ego_dist_remain = v_ego * t_remain_lc;
    std::array<double, 3> xp{11.111, 22.222, 33.333};
    std::array<double, 3> buffer{1.0, 3.0, 5.0};
    const double buffer_dist = interp(v_ego, xp, buffer);
    const double need_rel_dis =
        obstacle_dist_remain - ego_dist_remain + buffer_dist;

    // store debug_info
    lc_rear_objs_vec_ =
        GetObjsDebugInfo(v_node, a_node, t_remain_lc, -distance_rel);
    if (need_rel_dis > distance_rel) {
      return false;
    }
    // if (need_rel_dis > distance_rel) {
    //   lc_state_info->lc_should_back = true;
    //   lc_state_info->lc_back_reason = "side view back";
    //   lc_back_track_.set_value(target_lane_rear_node_->node_agent_id(),
    //                            -distance_rel, v_node);
    //   return;
    // }
  }
  return true;
}
void LaneChangeStateMachineManager::CalculateFrontAreaIfNeedBack(
    LaneChangeStageInfo* const lc_state_info) {
  const auto& ego_state =
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
    std::array<double, 3> xp{11.111, 22.222, 33.333};
    std::array<double, 3> buffer{1.0, 3.0, 5.0};
    const double buffer_dist = interp(v_ego, xp, buffer);
    const double need_rel_dis =
        ego_dist_remain - obstacle_dist_remain + buffer_dist;

    // store debug_info
    lc_front_objs_tar_lane_vec_ =
        GetObjsDebugInfo(v_node, a_node, t_remain_lc, distance_rel);
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
  lc_target_lane_merge_to_origin_lane_cnt_ = 0;
  lc_invalid_track_.reset();
  lc_back_track_.reset();
  must_change_lane_ = false;
  propose_state_frame_nums_ = 0;
  execution_state_frame_nums_ = 0;
  hold_state_frame_nums_ = 0;
  complete_state_frame_nums_ = 0;
  is_high_priority_back_ = false;
  ego_trajs_future_.clear();
  lc_path_generate_.reset();
  is_dash_not_enough_for_lc_ = false;
  execution_state_dash_cnt = 0;
  hold_state_dash_cnt = 0;
  road_edge_collision_cnt_ = 0;
  overtake_lane_change_confirmed_ = false;
}
void LaneChangeStateMachineManager::WeaklyResetStateMachine() {
  if (transition_info_.lane_change_status != kLaneChangePropose &&
      transition_info_.lane_change_status != kLaneKeeping &&
      transition_info_.lane_change_status != kLaneChangeHold) {
    return;
  }
  transition_info_.Rest();
  lc_lane_mgr_->reset_lc_lanes(transition_info_.lane_change_status);
  lane_change_stage_info_.Reset();
  lc_timer_.Reset();

  pre_ego_l_ = 0;
  lc_valid_cnt_ = 0;
  lc_back_cnt_ = 0;
  lc_target_lane_merge_to_origin_lane_cnt_ = 0;
  lc_invalid_track_.reset();
  lc_back_track_.reset();
  must_change_lane_ = false;
  propose_state_frame_nums_ = 0;
  execution_state_frame_nums_ = 0;
  hold_state_frame_nums_ = 0;
  complete_state_frame_nums_ = 0;
  is_high_priority_back_ = false;
  ego_trajs_future_.clear();
  lc_path_generate_.reset();
  is_dash_not_enough_for_lc_ = false;
  execution_state_dash_cnt = 0;
  hold_state_dash_cnt = 0;
  road_edge_collision_cnt_ = 0;
}
void LaneChangeStateMachineManager::OnlyResetLCStateMachine() {
  //重置 变道状态机 变道请求 但是保留lane change cmd
  if (transition_info_.lane_change_status != kLaneChangePropose &&
      transition_info_.lane_change_status != kLaneKeeping &&
      transition_info_.lane_change_status != kLaneChangeHold) {
    return;
  }
  transition_info_.Rest();
  lc_lane_mgr_->reset_lc_lanes(transition_info_.lane_change_status);
  lane_change_stage_info_.Reset();
  lc_timer_.Reset();
  pre_ego_l_ = 0;
  lc_valid_cnt_ = 0;
  lc_back_cnt_ = 0;
  lc_target_lane_merge_to_origin_lane_cnt_ = 0;
  lc_invalid_track_.reset();
  lc_back_track_.reset();
  must_change_lane_ = false;
  propose_state_frame_nums_ = 0;
  execution_state_frame_nums_ = 0;
  hold_state_frame_nums_ = 0;
  complete_state_frame_nums_ = 0;
  is_high_priority_back_ = false;
  ego_trajs_future_.clear();
  lc_path_generate_.reset();
  is_dash_not_enough_for_lc_ = false;
  execution_state_dash_cnt = 0;
  hold_state_dash_cnt = 0;
}
bool LaneChangeStateMachineManager::TimeOut(const bool trigger,
                                            bool* is_start_count,
                                            double* time_count,
                                            const double threshold) {
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
  const auto& lane_change_decider_output =
      session_->planning_context().lane_change_decider_output();

#ifdef ENABLE_PROTO_LOG
  auto& debug_info_manager = DebugInfoManager::GetInstance();
  auto& planning_debug_data = debug_info_manager.GetDebugInfoPb();
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
#endif

  JSON_DEBUG_VECTOR("front_obj_s_vec", lc_front_objs_ego_lane_vec_, 2);
  JSON_DEBUG_VECTOR("front_obj_s_tar_lane_vec", lc_front_objs_tar_lane_vec_, 2);
  JSON_DEBUG_VECTOR("rear_obj_s_vec", lc_rear_objs_vec_, 2);
  JSON_DEBUG_VECTOR("ego_s_vec", lc_egos_vec_, 2);
  JSON_DEBUG_VECTOR("t_vec", lc_time_vec_, 2);
  JSON_DEBUG_VECTOR("front_obj_need_dis_vec", lc_front_obj_need_dis_vec_, 2);
  JSON_DEBUG_VECTOR("rear_obj_need_dis_vec", lc_rear_obj_need_dis_vec_, 2);

  JSON_DEBUG_VECTOR("front_obj_future_v_vec", front_obj_future_v_, 2);
  JSON_DEBUG_VECTOR("rear_obj_future_v_vec", rear_obj_future_v_, 2);
  JSON_DEBUG_VECTOR("ego_future_v_vec", ego_future_v_, 2);

  JSON_DEBUG_VALUE("lc_safety_check_time", lc_safety_check_time_);
  JSON_DEBUG_VALUE("target_lane_congestion_level",
                   int(fix_lane_congestion_level_.level));
  JSON_DEBUG_VALUE("lat_offset_propose",
                   lane_change_decider_output.lateral_close_boundary_offset);
  JSON_DEBUG_VALUE("lat_offset_lc_hold",
                   lane_change_decider_output.lc_hold_state_lat_offset);
  auto& pp_path = lc_path_generate_->get_lc_path_result();
  if (lc_path_generate_ != nullptr && !pp_path.x.empty()) {
    JSON_DEBUG_VECTOR("lat_path_x", pp_path.x, 2);
    JSON_DEBUG_VECTOR("lat_path_y", pp_path.y, 2);
    JSON_DEBUG_VECTOR("lat_path_v", pp_path.v, 2);
    JSON_DEBUG_VECTOR("lat_path_t", pp_path.t, 2);
  } else {
    JSON_DEBUG_VECTOR("lat_path_x", std::vector<double>{0.0}, 2);
    JSON_DEBUG_VECTOR("lat_path_y", std::vector<double>{0.0}, 2);
    JSON_DEBUG_VECTOR("lat_path_v", std::vector<double>{0.0}, 2);
    JSON_DEBUG_VECTOR("lat_path_t", std::vector<double>{0.0}, 2);
  }

  // JSON_DEBUG_VALUE("front_agent_id",lane_change_decider_output.lc_gap_info.front_node_id);
  // JSON_DEBUG_VALUE("rear_agent_id",lane_change_decider_output.lc_gap_info.rear_node_id);
  int32_t rear_id = -1;
  int32_t front_id = -1;
  int32_t front_other_id = -1;
  int32_t side_id = -1;
  if (target_lane_rear_node_) {
    rear_id = target_lane_rear_node_->node_agent_id();
  }
  if (target_lane_front_node_) {
    front_id = target_lane_front_node_->node_agent_id();
  }
  if (!risk_agents_nodes_.empty()) {
    front_other_id = risk_agents_nodes_[0]->node_agent_id();
  }
  if (!risk_side_agents_nodes_.empty()) {
    side_id = risk_side_agents_nodes_[0]->node_agent_id();
  }
  JSON_DEBUG_VALUE("front_agent_id", front_id);
  JSON_DEBUG_VALUE("front_other_id", front_other_id);
  JSON_DEBUG_VALUE("rear_agent_id", rear_id);
  JSON_DEBUG_VALUE("side_id", side_id);
  JSON_DEBUG_VALUE("merging_rear_id", merging_rear_agent_id_);
  if (agent_box_corners_x_.empty() || ego_box_corners_x_.empty()) {
    JSON_DEBUG_VECTOR("agent_box_corners_x", std::vector<double>{0.0}, 2);
    JSON_DEBUG_VECTOR("agent_box_corners_y", std::vector<double>{0.0}, 2);
    JSON_DEBUG_VECTOR("ego_box_corners_x", std::vector<double>{0.0}, 2);
    JSON_DEBUG_VECTOR("ego_box_corners_y", std::vector<double>{0.0}, 2);
  } else {
    JSON_DEBUG_VECTOR("agent_box_corners_x", agent_box_corners_x_, 2);
    JSON_DEBUG_VECTOR("agent_box_corners_y", agent_box_corners_y_, 2);
    JSON_DEBUG_VECTOR("ego_box_corners_x", ego_box_corners_x_, 2);
    JSON_DEBUG_VECTOR("ego_box_corners_y", ego_box_corners_y_, 2);
  }

  // JSON_DEBUG_VECTOR("ego_sim_s", ego_sim_s, 2);
}

void LaneChangeStateMachineManager::GenerateTurnSignalForSplitRegion() {
  const auto& function_mode =
      session_->environmental_model().function_info().function_mode();
  const auto& route_info_output =
      session_->environmental_model().get_route_info()->get_route_info_output();

  if (function_mode == common::DrivingFunctionInfo::NOA &&
      route_info_output.map_vendor ==
          iflymapdata::sdpro::MAP_VENDOR_TENCENT_SD_PRO) {
    road_to_ramp_turn_signal_ = CalcTurnSignalForTencentSplitRegion();
  } else if (function_mode == common::DrivingFunctionInfo::NOA &&
             route_info_output.map_vendor ==
                 iflymapdata::sdpro::MAP_VENDOR_BAIDU_LD) {
    road_to_ramp_turn_signal_ = CalcTurnSignalForBaiduSplitRegion();
  } else {
    road_to_ramp_turn_signal_ = RAMP_NONE;
    return;
  }
}

bool LaneChangeStateMachineManager::IsSplitRegion(
    RampDirection* ramp_direction) {
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
    const auto& overlap_reference_path =
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
    double* lat_diff, const std::shared_ptr<ReferencePath> reference_path) {
  const auto& route_info_output =
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

  if (reference_path_frenet_coordinate == nullptr){
    return;
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
    const auto& right_path = reference_path_manager->get_reference_path_by_lane(
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
    const auto& left_path = reference_path_manager->get_reference_path_by_lane(
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
  if (overlap_path_frenet_coordinate == nullptr) {
    return false;
  }
  const auto& ego_vertices_points = session_->environmental_model()
                                        .get_ego_state_manager()
                                        ->polygon()
                                        .points();
  double ego_dis_to_ref_lane = NL_NMAX;
  for (const auto& ego_vertices_point : ego_vertices_points) {
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
  const auto& cur_lane = virtual_lane_manager->get_current_lane();
  const auto& ego_state =
      session_->environmental_model().get_ego_state_manager();
  const auto& plannig_init_point = ego_state->planning_init_point();
  double ego_x = plannig_init_point.lat_init_state.x();
  double ego_y = plannig_init_point.lat_init_state.y();
  double ego_s = 0.0, ego_l = 0.0;
  std::shared_ptr<planning_math::KDPath> target_boundary_path;
  const auto& cur_lane_left_boundary = cur_lane->get_left_lane_boundary();
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
  const auto& virtual_lane_mgr =
      session_->environmental_model().get_virtual_lane_manager();
  const auto& ego_state =
      session_->environmental_model().get_ego_state_manager();
  double lane_line_length = 0.0;
  std::shared_ptr<planning_math::KDPath> target_boundary_path;
  const auto& plannig_init_point = ego_state->planning_init_point();
  double ego_x = plannig_init_point.lat_init_state.x();
  double ego_y = plannig_init_point.lat_init_state.y();
  double ego_s = 0.0, ego_l = 0.0;
  const std::shared_ptr<VirtualLane> current_lane =
      virtual_lane_mgr->get_current_lane();

  if (lc_request == LEFT_CHANGE) {
    const auto& left_lane_boundarys = current_lane->get_left_lane_boundary();
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
    const auto& right_lane_boundarys = current_lane->get_right_lane_boundary();
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
  last_state_ =
      session_->planning_context().lane_change_decider_output().curr_state;
  IsEgoOnSideLane();
  lane_change_stage_info_.Reset();
  target_lane_front_node_ = nullptr;
  target_lane_middle_node_ = nullptr;
  target_lane_rear_node_ = nullptr;
  ego_lane_front_node_ = nullptr;
  merging_rear_agent_id_ = -1;
  // 初始化 is_aggressive_scence_
  is_aggressive_scence_ = false;
  // init debug info
  lc_front_objs_ego_lane_vec_.clear();
  lc_front_objs_tar_lane_vec_.clear();
  lc_rear_objs_vec_.clear();
  lc_egos_vec_.clear();
  lc_time_vec_.clear();
  lc_front_obj_need_dis_vec_.clear();
  lc_rear_obj_need_dis_vec_.clear();
  front_obj_future_v_.clear();
  rear_obj_future_v_.clear();
  ego_future_v_.clear();

  // init ego future trajactorys
  ego_trajs_future_.clear();
  ego_trajs_future_copy_.clear();
  agent_box_corners_x_.clear();
  agent_box_corners_y_.clear();
  ego_box_corners_x_.clear();
  ego_box_corners_y_.clear();
  risk_agents_nodes_.clear();
  risk_side_agents_nodes_.clear();
  RequestType direction = lc_req_mgr_->request();
  const auto& current_lc_state = transition_info_.lane_change_status;
  if (direction == RequestType::NO_CHANGE) {
    return;
  }
  // 获取目标车道的障碍物
  const auto& dynamic_world =
      session_->environmental_model().get_dynamic_world();
  const int64_t ego_front_node_id = dynamic_world->ego_front_node_id();
  int64_t target_lane_front_node_id = planning_data::kInvalidId;
  int64_t target_lane_middle_node_id = planning_data::kInvalidId;
  int64_t target_lane_rear_node_id = planning_data::kInvalidId;
  int64_t ego_lane_front_node_id = planning_data::kInvalidId;
  if (direction == LEFT_CHANGE) {
    if (current_lc_state == kLaneChangeExecution ||
        current_lc_state == kLaneChangeComplete) {  // 目标车道已经切换
      target_lane_front_node_id = dynamic_world->ego_front_node_id();
      target_lane_rear_node_id = dynamic_world->ego_rear_node_id();
    } else {  // 目标车道没切换
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
  // target_lane_front_node_ =
  // dynamic_world->GetNode(target_lane_front_node_id);
  target_lane_middle_node_ = dynamic_world->GetNode(target_lane_middle_node_id);
  target_lane_rear_node_ = dynamic_world->GetNode(target_lane_rear_node_id);
  ego_lane_front_node_ = dynamic_world->GetNode(ego_lane_front_node_id);
  // add second check for target node
  CheckTargetFrontNode(target_lane_front_node_id);
  CheckTargetRearNode(target_lane_rear_node_id);
  // GetFrontRiskAgentTrajs();
  GetSideRiskAgents();
  // AddRearAgentMerging();

  if (target_lane_rear_node_) {
    is_large_car_in_side_ = IsLargeAgent(target_lane_rear_node_);
  } else {
    is_large_car_in_side_ = false;
  }

  const int fix_lane_virtual_id = lc_lane_mgr_->target_lane_virtual_id();
  const auto& ref_path = session_->environmental_model()
                             .get_reference_path_manager()
                             ->get_reference_path_by_lane(fix_lane_virtual_id);
  const auto& virtual_lane_mgr =
      session_->environmental_model().get_virtual_lane_manager();
  // 初始化拥堵检测器并进行检测 检测目标车道
  CongestionDetector congestion_detector(&congestion_detection_config_,
                                         session_, fix_lane_virtual_id);
  fix_lane_congestion_level_ = congestion_detector.DetectLaneCongestion();
  // 变道条件下，生成初始轨迹
  is_aggressive_scence_ = IsEmergencyScene();
  JSON_DEBUG_VALUE("is_aggressive_scence", is_aggressive_scence_);
  JSON_DEBUG_VALUE("is_default_aggressive_scence", lc_safety_check_config_.is_default_aggressive_scence);
}
void LaneChangeStateMachineManager::JointLaneChangeDecisionGeneration() {
  JSON_DEBUG_VALUE("joint_lane_change_state",
                   static_cast<int>(transition_info_.lane_change_status));
  if (transition_info_.lane_change_status ==
      StateMachineLaneChangeStatus::kLaneKeeping) {
    return;
  }
  ego_trajs_future_ =
      CalculateEgoPPIDMTrajs();  // 生成初始轨迹，确保点数 26 [原在用IDM'PP]
  ego_trajs_future_copy_ = ego_trajs_future_;

  if(ego_trajs_future_.empty()){
    return;
  }
  lc_joint_decision_generator_->ClearLaneChangeDecisionInfo();
  int gap_front_agent_id =
      target_lane_front_node_ ? target_lane_front_node_->node_agent_id() : -1;
  int gap_rear_agent_id =
      target_lane_rear_node_ ? target_lane_rear_node_->node_agent_id() : -1;
  //原车道前车(可能切换)
  // const auto lead_one =
  //     session_->environmental_model().get_lateral_obstacle()->leadone();
  const auto cipv_decider_output =
      session_->planning_context().cipv_decider_output();
  // 自车压线目标车道情况
  const double ego_press_line_ratio =
      lc_request_.CalculatePressLineRatioByTwoLanes(
          lc_lane_mgr_->origin_lane_virtual_id(),
          lc_lane_mgr_->target_lane_virtual_id(),
          transition_info_.lane_change_direction);
  // ptr 非空， 后轴中心没过线，与targt front 不同，压线较少，则原来取前车id
  bool is_lead_one_valid = false;
  if (cipv_decider_output.cipv_id() != -1) {
    is_lead_one_valid =
        cipv_decider_output.cipv_id() != gap_front_agent_id &&
        transition_info_.lane_change_status != kLaneChangeComplete &&
        ego_press_line_ratio <
            lc_safety_check_config_.press_line_ratio_threshold;
  }
  int origin_agent_id = is_lead_one_valid ? cipv_decider_output.cipv_id() : -1;
  LaneChangeDecisionInfo lc_info;
  lc_info.gap_front_agent_id = gap_front_agent_id;
  lc_info.gap_rear_agent_id = gap_rear_agent_id;
  lc_info.origin_agent_id = origin_agent_id;
  lc_info.ego_ref_traj = ego_trajs_future_;
  lc_info.target_lane_virtual_id = lc_lane_mgr_->target_lane_virtual_id();
  lc_info.origin_lane_virtual_id = lc_lane_mgr_->origin_lane_virtual_id();
  is_side_clear_ = IsSideClear(gap_front_agent_id, gap_rear_agent_id);
  joint_decision_obstacle_labels_.clear();
  rear_agent_overtaking_ = false;
  if (transition_info_.lane_change_status ==
    StateMachineLaneChangeStatus::kLaneKeeping) {
      return;
  }
  lc_joint_decision_generator_->SetLaneChangeDecisionInfo(lc_info);
  lc_joint_decision_generator_
      ->Execute();  // 生成联合优化轨迹(内部修饰障碍物轨迹)
  JSON_DEBUG_VALUE("target_lane_virtual_id", lc_info.target_lane_virtual_id);
  JSON_DEBUG_VALUE("origin_lane_virtual_id", lc_info.origin_lane_virtual_id);
  JSON_DEBUG_VALUE("lc_gap_front_agent_id", lc_info.gap_front_agent_id);
  JSON_DEBUG_VALUE("lc_gap_rear_agent_id", lc_info.gap_rear_agent_id);

  // 输出检查
  const auto joint_decision_output =
      session_->planning_context().lat_lon_joint_decision_output();
  joint_decision_success_ = joint_decision_output.IsPlanningSuccess();
  const auto& obstacle_ids = joint_decision_output.GetSelectedObstacleIds();
  const auto& obstacle_labels =
      joint_decision_output.GetSelectedObstacleLabels();
  const size_t obstacle_num = std::min(obstacle_ids.size(), obstacle_labels.size());
  for (size_t i = 0; i < obstacle_num; ++i) {
    joint_decision_obstacle_labels_[obstacle_ids[i]] = obstacle_labels[i];
  }
  if (!joint_decision_success_) {
    ego_trajs_future_.clear();
    return;
  }
  const auto& ego_opt_traj =
      joint_decision_output.GetLaneChangeEgoTrajectory();
  ego_trajs_future_.clear();
  ego_trajs_future_.resize(ego_opt_traj.size());
  for (size_t i = 0; i < ego_opt_traj.size(); i++) {
    ego_trajs_future_[i].x = ego_opt_traj[i].x;
    ego_trajs_future_[i].y = ego_opt_traj[i].y;
    ego_trajs_future_[i].heading_angle = ego_opt_traj[i].theta;
    ego_trajs_future_[i].delta = ego_opt_traj[i].delta;
    ego_trajs_future_[i].v = ego_opt_traj[i].vel;
    ego_trajs_future_[i].a = ego_opt_traj[i].acc;
    ego_trajs_future_[i].jerk = ego_opt_traj[i].jerk;
    ego_trajs_future_[i].s = ego_opt_traj[i].s;
    ego_trajs_future_[i].t = ego_opt_traj[i].t;
  }
  // 记录更新后的后车标签，未命中则保持默认值。
  // 优化器 EGO_OVERTAKE 表示实际轨迹与理想减速轨迹不匹配，进一步确定是否会在短时间内超越自车
  const auto rear_it =
      joint_decision_obstacle_labels_.find(lc_info.gap_rear_agent_id);
  if (rear_it == joint_decision_obstacle_labels_.end()) {
    return;
  }
  if (rear_it->second != lane_change_joint_decision::LongitudinalLabel::EGO_OVERTAKE) {
    return;
  }
  if (ego_trajs_future_.empty()) {
    return;
  }
  const auto& dynamic_world = session_->environmental_model().get_dynamic_world();
  const auto rear_agent = dynamic_world->agent_manager()->GetAgent(lc_info.gap_rear_agent_id);
  if (rear_agent == nullptr) {
    return;
  }
  const auto& agent_trajs = rear_agent->trajectories_used_by_st_graph();
  if (agent_trajs.empty() || agent_trajs[0].empty()) {
    return;
  }
  const int target_lane_virtual_id = lc_req_mgr_->target_lane_virtual_id();
  const auto& target_ref_path = session_->environmental_model()
                                    .get_reference_path_manager()
                                    ->get_reference_path_by_lane(target_lane_virtual_id);
  if (target_ref_path == nullptr) {
    return;
  }
  const auto& target_lane_coord = target_ref_path->get_frenet_coord();
  if (target_lane_coord == nullptr) {
    return;
  }
  const auto& traj = agent_trajs[0];
  const auto& vehicle_param = VehicleConfigurationContext::Instance()->get_vehicle_param();
  const double two_car_length =
      vehicle_param.rear_edge_to_rear_axle + rear_agent->length() * 0.5;
  double agent_s0 = 0.0, agent_l0 = 0.0;
  if (!target_lane_coord->XYToSL(traj[0].x(), traj[0].y(), &agent_s0, &agent_l0)) {
    return;
  }
  const double rel_vel = std::max(0.0, traj[0].vel() - ego_trajs_future_[0].v);
  const double predict_t = lc_safety_check_time_ + 1.5;
  const double rear_acc = rear_agent->accel_fusion();
  double dis_diff_vel = 0.0;
  if (rear_acc > 0.3) {
    dis_diff_vel = rel_vel * predict_t + 0.5 * rear_acc * predict_t * predict_t;
  } else if (rear_acc < -0.3) {
    dis_diff_vel = rel_vel * rel_vel / (2.0 * (-rear_acc));
  } else {
    dis_diff_vel = rel_vel * predict_t;
  }
  const double rear_gap = ego_trajs_future_[0].s - agent_s0 - two_car_length;
  rear_agent_overtaking_ = !(rear_gap > dis_diff_vel && rear_gap > 0.0);
}
void LaneChangeStateMachineManager::CheckTargetFrontNode(
    int64_t target_lane_front_node_id) {
  const int target_lane_virtual_id = lc_lane_mgr_->target_lane_virtual_id();
  const auto& virtual_lane_manager =
      session_->environmental_model().get_virtual_lane_manager();
  const auto& target_lane =
      virtual_lane_manager->get_lane_with_virtual_id(target_lane_virtual_id);
  if (target_lane == nullptr) {
    return;
  }
  const auto& ref_path =
      session_->environmental_model()
          .get_reference_path_manager()
          ->get_reference_path_by_lane(target_lane_virtual_id);
  const auto& ego_sl_bd = ref_path->get_ego_frenet_boundary();
  const auto& ego_sl_state = ref_path->get_frenet_ego_state();
  const auto& dynamic_world =
      session_->environmental_model().get_dynamic_world();
  const auto& agent_mgr = dynamic_world->agent_manager();
  const auto origin_target_node =
      dynamic_world->GetNode(target_lane_front_node_id);
  RequestType direction = lc_req_mgr_->request();
  double car_width =
      VehicleConfigurationContext::Instance()->get_vehicle_param().width;
  // const auto agent =
  // agent_mgr->GetAgent(origin_target_node->node_agent_id());

  double target_front_s = 150.0;
  int64_t target_front_node_id = planning_data::kInvalidId;
  // target_lane_front_node_ 不满足 继续向前根据目标车道选择cipv
  const auto& target_lane_nodes =
      session_->environmental_model().get_dynamic_world()->GetNodesByLaneId(
          target_lane_virtual_id);
  const auto& obstacles_map = ref_path->get_obstacles_map();
  const auto& ego_boundary = ref_path->get_ego_frenet_boundary();
  for (const auto* target_lane_node : target_lane_nodes) {
    if(target_lane_node == nullptr) {
      continue;
    }
    double x = target_lane_node->node_x();
    double y = target_lane_node->node_y();
    Point2D node_cart(x, y);
    double s = 0;
    double l = 0;
    const auto& nearest_lane =
        virtual_lane_manager->GetNearestLane(node_cart, &s, &l);
    if (nearest_lane == nullptr) {
      continue;
    }
    // if (target_lane_virtual_id != nearest_lane->get_virtual_id()) {
    //   continue;
    // } banned: 因为可变车道分叉
    double agent_s = target_lane_node->node_s();
    double agent_s_start = agent_s - target_lane_node->node_length() * 0.5;
    if (agent_s + target_lane_node->node_length() * 0.5 < ego_sl_bd.s_end) {
      continue;  // 车头落后自车就是后车
    }
    // 逆向车
    const auto& agent_trajs =
        target_lane_node->node_trajectories_used_by_st_graph();
    if (agent_trajs.empty()) {
      continue;
    }
    const auto& traj =
        target_lane_node->node_trajectories_used_by_st_graph()[0];
    int mid_index = std::max(static_cast<int>(traj.size() / 2), 0);
    const auto& target_lane_coord = ref_path->get_frenet_coord();
    if (traj.empty() || target_lane_coord == nullptr) {
      continue;
    }
    double s_start = 0.0;
    double l_start = 0.0;
    if (!target_lane_coord->XYToSL(traj.front().x(), traj.front().y(), &s_start,
                                   &l_start)) {
      continue;
    }
    double s_mid = 0.0;
    double l_mid = 0.0;
    if (!target_lane_coord->XYToSL(traj[mid_index].x(), traj[mid_index].y(),
                                   &s_mid, &l_mid)) {
      continue;
    }
    double s_end = 0.0;
    double l_end = 0.0;
    if (!target_lane_coord->XYToSL(traj.back().x(), traj.back().y(), &s_end,
                                   &l_end)) {
      continue;
    }
    bool is_reverse = false;
    if (s_end < s_start && target_lane_node->node_speed() > 2.0) {
      is_reverse = true;
    }
    // 静态-动态： cut in - no cut in
    // 创建agent 3个 box 对应的 sl_bd
    const auto agent = agent_mgr->GetAgent(target_lane_node->node_agent_id());
    // Vec2d agent_mid_rac(traj[mid_index].x(), traj[mid_index].y());
    // double agent_mid_theta = traj[mid_index].theta();
    // planning_math::Box2d agent_mid_box(agent_mid_rac, agent_mid_theta,
    // target_lane_node->node_length(),
    //                                target_lane_node->node_width());

    // Vec2d agent_end_rac(traj.back().x(), traj.back().y());
    // double agent_end_theta = traj.back().theta();
    // planning_math::Box2d agent_end_box(agent_end_rac, agent_end_theta,
    // target_lane_node->node_length(),
    //                               target_lane_node->node_width());

    const auto& agent_start_bd = GetSLboundaryFromAgent(ref_path, agent->box());
    // const auto &agent_mid_bd = GetSLboundaryFromAgent(ref_path,
    // agent_mid_box); const auto &agent_end_bd =
    // GetSLboundaryFromAgent(ref_path, agent_end_box);
    // 横向距离变小，预测是保守的，这里用或  ; 能绑定说明预测轨迹进来了。
    bool is_lat_closing = (std::fabs(l_end) - std::fabs(l_mid) < -0.5 ||
                           std::fabs(l_mid) - std::fabs(l_start) < -0.5);
    bool is_side = !(agent_start_bd.s_start > ego_boundary.s_end ||
                        agent_start_bd.s_end < ego_boundary.s_start);
    const double target_lane_width = target_lane->width_by_s(agent_s);
    double safety_buff = 0.8;
    if(is_reverse){
      safety_buff = 1.4;//对向车道压线 约剩余空间 3.4 m则忽略
      bool pass_in_lane = PassInLane(target_lane_width, agent_start_bd,
                                     car_width, safety_buff, direction);
      if (pass_in_lane) {
        continue;
      }
    } else if (target_lane_node->is_static_type()) {  // 静止
      safety_buff = 0.7;
      bool pass_in_lane = PassInLane(target_lane_width, agent_start_bd,
                                     car_width, safety_buff, direction);
      if (pass_in_lane) {
        continue;
      }
    } else if (!is_lat_closing) {  // 正常直行, 不在侧边
      safety_buff = 1.0;
      bool pass_in_lane = PassInLane(target_lane_width, agent_start_bd,
                                     car_width, safety_buff, direction);
      if (pass_in_lane && !target_lane_node->is_agent_most_within_lane() && !is_side) {
        continue;  // 前者针对大型车压线，后者针对小vru靠边， 不是侧方车才可以过滤
      }
    } else {  // cutin 趋势
      if (!target_lane_node->is_agent_within_lane()) { // 当前不在，短时间内也不会在，过滤。
      //   auto it = obstacles_map.find(target_lane_node->node_agent_id());
      //   const auto agent = agent_mgr->GetAgent(target_lane_node->node_agent_id());
      //   if (it != obstacles_map.end() && agent != nullptr) {
      //     const auto& agent_bd = GetSLboundaryFromAgent(ref_path, agent->box());
      //     const auto& front_side_obs = it->second;
      //     std::pair<double, double> target_center_lat{- target_lane_width * 0.5, target_lane_width * 0.5};
      //     std::pair<double, double> obs_lat{agent_bd.l_start, agent_bd.l_end};
      //     double obs_lat_vel = front_side_obs->frenet_velocity_l();
      //     bool is_target_lane_cuting_in =
      //         IfFrenetCollision(target_center_lat, 0.0, obs_lat, obs_lat_vel,
      //                         lc_safety_check_config_.target_lane_front_cut_in_check_time, 0.5);
      //     if (!is_target_lane_cuting_in) {
      //       continue;  // 1.5 s 不进入目标车道过滤
      //     }
      //   }
      // }
        continue;
      }
    }
    if (agent_s_start < target_front_s) { // 对比前车尾部
      target_front_node_id = target_lane_node->node_id();
      target_front_s = agent_s_start;
    }
  }
  target_lane_front_node_ = dynamic_world->GetNode(target_front_node_id);
}
void LaneChangeStateMachineManager::CheckTargetRearNode(
    int64_t target_lane_rear_node_id) {
  const int target_lane_virtual_id = lc_lane_mgr_->target_lane_virtual_id();
  const auto& virtual_lane_manager =
      session_->environmental_model().get_virtual_lane_manager();
  const auto& target_lane =
      virtual_lane_manager->get_lane_with_virtual_id(target_lane_virtual_id);
  if (target_lane == nullptr) {
    return;
  }
  const auto& ref_path =
      session_->environmental_model()
          .get_reference_path_manager()
          ->get_reference_path_by_lane(target_lane_virtual_id);
  const auto& ego_sl_bd = ref_path->get_ego_frenet_boundary();
  const auto& ego_sl_state = ref_path->get_frenet_ego_state();
  const auto& dynamic_world =
      session_->environmental_model().get_dynamic_world();
  const auto& agent_mgr = dynamic_world->agent_manager();
  const auto origin_target_rear_node =
      dynamic_world->GetNode(target_lane_rear_node_id);
  RequestType direction = lc_req_mgr_->request();

  const auto& target_lane_nodes =
      session_->environmental_model().get_dynamic_world()->GetNodesByLaneId(
          target_lane_virtual_id);
  double target_rear_s = - std::numeric_limits<double>::infinity(); // 后车车头位置
  int64_t target_rear_node_id = planning_data::kInvalidId;
      //横向运动信息
  const auto& obstacles_map = ref_path->get_obstacles_map();
  double urgent_rear_gap = std::numeric_limits<double>::infinity();
  double urgent_reached_time = kMaxReachedTime;
  bool has_target = false;
  bool has_near_rear = false;
  for (const auto* target_lane_node : target_lane_nodes) {
    if(target_lane_node == nullptr) {
      continue;
    }
    int idx = target_lane_node->node_agent_id();
    if(idx < 0) {
      continue;
    }
    double x = target_lane_node->node_x();
    double y = target_lane_node->node_y();
    Point2D node_cart(x, y);
    double s = 0;
    double l = 0;
    const auto& nearest_lane =
        virtual_lane_manager->GetNearestLane(node_cart, &s, &l);
    if (nearest_lane == nullptr) {
      continue;
    }
    double agent_s = target_lane_node->node_s();
    double agent_s_end = agent_s + target_lane_node->node_length() * 0.5;
    if (agent_s + target_lane_node->node_length() * 0.5 >= ego_sl_bd.s_end) {
      continue;  // 车头落后自车就是后车
    }
    // if (target_lane_virtual_id != nearest_lane->get_virtual_id() &&
    //     agent_s + target_lane_node->node_length() * 0.5 >
    //         ego_sl_bd.s_start - 5.0) {
    //   continue;  //
    //   过滤：预测轨迹进入目标车道，但是只是近距离跟随的后车。banned:
    //   可变车道会分叉
    // }
    const double target_lane_width = target_lane->width_by_s(agent_s);
    const auto agent = agent_mgr->GetAgent(target_lane_node->node_agent_id());
    const auto& agent_bd = GetSLboundaryFromAgent(ref_path, agent->box());
    bool is_out_target_lane = agent_bd.l_start > target_lane_width * 0.5 ||
                              agent_bd.l_end < - target_lane_width * 0.5;
    if (is_out_target_lane) {
      continue;
    }
    // auto it = obstacles_map.find(target_lane_node->node_agent_id());
    // if (it != obstacles_map.end()) {
    //   const auto& rear_obs = it->second;
    //   std::pair<double, double> target_center_lat{- target_lane_width * 0.5, target_lane_width * 0.5};
    //   std::pair<double, double> obs_lat{agent_bd.l_start, agent_bd.l_end};
    //   double obs_lat_vel = rear_obs->frenet_velocity_l();
    //   bool is_target_lane_cuting_in =
    //       IfFrenetCollision(target_center_lat, 0.0, obs_lat, obs_lat_vel,
    //                        lc_safety_check_config_.target_lane_rear_cut_in_check_time, 0.5);
    //   if (!is_target_lane_cuting_in) {
    //     continue;  // 1.5 s 不进入目标车道才过滤
    //   }
    // }
    const auto& agent_trajs =
        target_lane_node->node_trajectories_used_by_st_graph();
    if (agent_trajs.empty()) {
      continue;
    }
    const auto& traj =
        target_lane_node->node_trajectories_used_by_st_graph()[0];
    const auto& target_lane_coord = ref_path->get_frenet_coord();
    if (traj.empty() || target_lane_coord == nullptr) {
      continue;
    }
    double s_start = 0.0;
    double l_start = 0.0;
    if (!target_lane_coord->XYToSL(traj.front().x(), traj.front().y(), &s_start,
                                   &l_start)) {
      continue;
    }
    double s_end = 0.0;
    double l_end = 0.0;
    if (!target_lane_coord->XYToSL(traj.back().x(), traj.back().y(), &s_end,
                                   &l_end)) {
      continue;
    }
    int mid_index = std::max(static_cast<int>(traj.size() / 2), 0);
    double s_mid = 0.0;
    double l_mid = 0.0;
    if (!target_lane_coord->XYToSL(traj[mid_index].x(), traj[mid_index].y(),
                                   &s_mid, &l_mid)) {
      continue;
    }
    if (s_end < s_start) {
      continue;
    }  // 小心太后方的车可能 frenet转化后堆在一起
    // if (agent_s_end > target_rear_s) {
    //   target_rear_node_id = target_lane_node->node_id();
    //   target_rear_s = agent_s;  // 选择最靠前的
    // }
    if (target_lane_node->is_static_type() || target_lane_node->node_speed() <0.3 ) {
      continue;
    }
    double rear_risk_dis = target_lane_node->node_speed() * 0.5 + 2.0;
    double rear_gap = std::max(0.0, ego_sl_bd.s_start - agent_s_end);
    bool is_rear_risk = rear_gap < rear_risk_dis;
    double rear_faster_vel =
        target_lane_node->node_speed() - ego_sl_state.velocity_s();
    double valid_rel_vel = std::max(rear_faster_vel, kEps);
    double rear_reached_time =
        rear_faster_vel > 0.0 ? rear_gap / valid_rel_vel : kMaxReachedTime;
    rear_reached_time = rear_gap < kEps ? 0.0 : rear_reached_time;
    rear_reached_time = std::min(rear_reached_time, kMaxReachedTime);
    if (is_rear_risk) {
      // ===== 近距离区：绝对最高优先级 =====
      if (!has_near_rear || agent_s_end > target_rear_s) {
        has_near_rear = true;
        has_target = true;
        target_rear_node_id = target_lane_node->node_id();
        target_rear_s = agent_s_end;
        urgent_rear_gap = rear_gap;
        urgent_reached_time = rear_reached_time;
      }
    } else if (!has_near_rear) {
      // ===== 非近距离区，只在尚未命中近距离时参与 =====
      if (!has_target) {
        // 第一个非近距离目标
        has_target = true;
        target_rear_node_id = target_lane_node->node_id();
        target_rear_s = agent_s_end;
        urgent_rear_gap = rear_gap;
        urgent_reached_time = rear_reached_time;
      } else if (rear_reached_time < kMaxReachedTime &&
                 urgent_reached_time < kMaxReachedTime) {
        // 追击时间比较
        if (rear_reached_time < urgent_reached_time) {
          target_rear_node_id = target_lane_node->node_id();
          target_rear_s = agent_s_end;
          urgent_rear_gap = rear_gap;
          urgent_reached_time = rear_reached_time;
        }
      } else if (rear_reached_time < kMaxReachedTime &&
                 urgent_reached_time >= kMaxReachedTime) {
        // 有效追击时间优先
        target_rear_node_id = target_lane_node->node_id();
        target_rear_s = agent_s_end;
        urgent_rear_gap = rear_gap;
        urgent_reached_time = rear_reached_time;
      } else if (rear_reached_time >= kMaxReachedTime &&
                 urgent_reached_time >= kMaxReachedTime) {
        // 都无追击意义，回退距离
        if (agent_s_end > target_rear_s) {
          target_rear_node_id = target_lane_node->node_id();
          target_rear_s = agent_s_end;
          urgent_rear_gap = rear_gap;
          urgent_reached_time = rear_reached_time;
        }
      }
    }
  }
  target_lane_rear_node_ = dynamic_world->GetNode(target_rear_node_id);
}
void LaneChangeStateMachineManager::GetFrontRiskAgentTrajs() {
  const int target_lane_virtual_id = lc_req_mgr_->target_lane_virtual_id();
  const auto& virtual_lane_manager =
      session_->environmental_model().get_virtual_lane_manager();
  const auto& target_lane =
      virtual_lane_manager->get_lane_with_virtual_id(target_lane_virtual_id);
  if (target_lane == nullptr) {
    return;
  }
  double target_lane_half_width = target_lane->width() * 0.5;
  const auto& ref_path =
      session_->environmental_model()
          .get_reference_path_manager()
          ->get_reference_path_by_lane(target_lane_virtual_id);
  if (ref_path == nullptr) {
    return;
  }
  const auto& obstacles_map = ref_path->get_obstacles_map();
  const auto& frenet_ego_state = ref_path->get_frenet_ego_state();
  const auto& ego_state_mgr =
      session_->environmental_model().get_ego_state_manager();
  const auto& dynamic_world =
      session_->environmental_model().get_dynamic_world();
  const auto& agent_mgr = dynamic_world->agent_manager();

  RequestType direction = lc_req_mgr_->request();
  double car_width =
      VehicleConfigurationContext::Instance()->get_vehicle_param().width;
  // const auto agent =
  // agent_mgr->GetAgent(origin_target_node->node_agent_id());

  double target_front_s =
      frenet_ego_state.s() +
      planning_math::Clamp(frenet_ego_state.velocity_s() * 5.0, 10.0,
                           150.0);  // 默认关注距离
  if (!ego_trajs_future_.empty()) {
    target_front_s = ego_trajs_future_.back().s;
  }
  const auto& ego_sl_bd = ref_path->get_ego_frenet_boundary();
  int64_t target_front_node_id = planning_data::kInvalidId;
  // target_lane_front_node_ 不满足 继续向前根据目标车道选择cipv
  // 根据target front 设置关注的前视距离

  if (target_lane_front_node_ != nullptr) {
    target_front_s = target_lane_front_node_->node_s() -
                     target_lane_front_node_->node_length() * 0.5;
  }
  // 遍历寻找
  const auto& target_lane_nodes =
      session_->environmental_model().get_dynamic_world()->GetNodesByLaneId(
          target_lane_virtual_id);
  risk_agents_nodes_.clear();
  for (const auto* target_lane_node : target_lane_nodes) {
    if (target_lane_node == nullptr) {
      continue;
    }
    double x = target_lane_node->node_x();
    double y = target_lane_node->node_y();
    Point2D node_cart(x, y);
    double s = 0;
    double l = 0;
    const auto& nearest_lane =
        virtual_lane_manager->GetNearestLane(node_cart, &s, &l);
    if (nearest_lane == nullptr) {
      continue;
    }
    if (target_lane_virtual_id != nearest_lane->get_virtual_id()) {
      continue;
    }
    double agent_s = target_lane_node->node_s();
    if (target_lane_front_node_ != nullptr &&
        target_lane_node->node_id() == target_lane_front_node_->node_id()) {
      continue;
    }
    if (agent_s - target_lane_node->node_length() * 0.5 > target_front_s) {
      continue;
    }
    if (agent_s - target_lane_node->node_length() * 0.5 < ego_sl_bd.s_end) {
      continue;
    }
    if (FacilityTypes.count(target_lane_node->type()) > 0) {
      continue;
    }

    int id = target_lane_node->node_agent_id();
    auto it = obstacles_map.find(id);
    if (it != obstacles_map.end()) {
      const auto& front_obs = it->second;
      const auto& front_obs_sl = front_obs->frenet_obstacle_boundary();
      if (front_obs_sl.l_end < -target_lane_half_width ||
          front_obs_sl.l_start > target_lane_half_width) {
        continue;
      }
      bool pass_in_lane = PassInLane(target_lane_half_width * 2, front_obs_sl,
                                     car_width, 0.8, direction);
      if (pass_in_lane) {
        continue;
      }
    }
    risk_agents_nodes_.push_back(target_lane_node);
  }
}
// 关注自车左右侧快速靠近的障碍物
void LaneChangeStateMachineManager::GetSideRiskAgents() {
  risk_side_agents_nodes_.clear();
  const int target_lane_virtual_id = lc_req_mgr_->target_lane_virtual_id();
  const auto virtual_lane_manager =
      session_->environmental_model().get_virtual_lane_manager();
  const auto target_lane =
      virtual_lane_manager->get_lane_with_virtual_id(target_lane_virtual_id);
  if (target_lane == nullptr) {
    return;
  }
  const double half_width_by_target = target_lane->width() * 0.5;
  const auto reference_path_manager =
      session_->environmental_model().get_reference_path_manager();
  const auto target_reference_path =
      reference_path_manager->get_reference_path_by_lane(
          target_lane_virtual_id);
  const auto& ego_sl_bd = target_reference_path->get_ego_frenet_boundary();
  const auto& ego_sl_state = target_reference_path->get_frenet_ego_state();
  const auto& obstacles = target_reference_path->get_obstacles();
  if(target_reference_path == nullptr) {
    return;
  }
  const auto& target_lane_coord = target_reference_path->get_frenet_coord();
  if(target_lane_coord == nullptr) {
    return;
  }
  double ego_vel = ego_sl_state.velocity_s();
  RequestType direction = lc_req_mgr_->request();
  const auto& agent_mgr = session_->environmental_model().get_dynamic_world()->agent_manager();
  double side_s_end = ego_vel * 2.0 + ego_sl_state.s();
  double side_s_start = - 50.0; //自车后100m以外不考虑
  if (target_lane_front_node_ != nullptr) {
    side_s_end = target_lane_front_node_->node_s() -
                target_lane_front_node_->node_length() * 0.5;
  }
  if (target_lane_rear_node_ != nullptr) {
    side_s_start = target_lane_rear_node_->node_s() +
              target_lane_rear_node_->node_length() * 0.5;
  }
  const auto& target_lane_nodes =
      session_->environmental_model().get_dynamic_world()->GetNodesByLaneId(
          target_lane_virtual_id);
  int64_t target_rear_node_id = planning_data::kInvalidId;
      //横向运动信息
  const auto& obstacles_map = target_reference_path->get_obstacles_map();
  for (const auto* target_lane_node : target_lane_nodes) {
    if(target_lane_node == nullptr) {
      continue;
    }
    const auto agent = agent_mgr->GetAgent(target_lane_node->node_agent_id());
    if (agent == nullptr) {
      continue;
    }
    int id = agent->agent_id();
    const auto& obstacle_sl = GetSLboundaryFromAgent(target_reference_path, agent->box());
    //过滤障碍物
    if (direction == RIGHT_CHANGE && obstacle_sl.l_start > ego_sl_bd.l_end) {
      continue;
    }
    if (direction == LEFT_CHANGE && obstacle_sl.l_end < ego_sl_bd.l_start) {
      continue;
    }
    if (target_lane_front_node_ != nullptr &&
        id == target_lane_front_node_->node_agent_id()) {
      continue;
    }
    if (target_lane_rear_node_ != nullptr &&
        id == target_lane_rear_node_->node_agent_id()) {
      continue;
    }
    if (agent->is_static()) {
      continue;
    }
    // gap 范围过滤
    if(obstacle_sl.s_start > side_s_end || obstacle_sl.s_end < side_s_start) {
      continue;
    }
    // 侵入趋势检查
    if (FacilityTypes.count(agent->type()) > 0) {
      continue;
    }
    const auto& trajs = agent->trajectories_used_by_st_graph();
    if(trajs.empty()) {
      continue;
    }
    const auto& traj = trajs[0];
    if(traj.empty()) {
      continue;
    }
    double s_start = 0.0;
    double l_start = 0.0;
    if (!target_lane_coord->XYToSL(traj.front().x(), traj.front().y(), &s_start,
                                  &l_start)) {
      continue;
    }
    double s_end = 0.0;
    double l_end = 0.0;
    if (!target_lane_coord->XYToSL(traj.back().x(), traj.back().y(), &s_end,
                                  &l_end)) {
      continue;
    }
    int mid_index = std::max(static_cast<int>(traj.size() / 2), 0);
    double s_mid = 0.0;
    double l_mid = 0.0;
    if (!target_lane_coord->XYToSL(traj[mid_index].x(), traj[mid_index].y(),
                                  &s_mid, &l_mid)) {
      continue;
    }
    //2.5s 预测轨迹没有侵入，过滤
    bool is_mid_out_lane = l_mid - agent->width() * 0.5 > half_width_by_target ||
                        l_mid + agent->width() * 0.5 < - half_width_by_target ;
    if (is_mid_out_lane) {
      continue;
    }
    //横向速度不侵入，过滤
    const auto& it = obstacles_map.find(id);
    if (it != obstacles_map.end()) {
      const auto& side_obs = it->second;
      double obs_lat_vel = side_obs->frenet_velocity_l();
      std::pair<double, double> target_center_lat{- half_width_by_target, half_width_by_target};
      std::pair<double, double> obs_lat{obstacle_sl.l_start, obstacle_sl.l_end};
      bool is_target_lane_cuting_in =
        IfFrenetCollision(target_center_lat, 0.0, obs_lat, obs_lat_vel,
                        lc_safety_check_config_.target_lane_side_cut_in_check_time, 0.5);
      if (!is_target_lane_cuting_in) {
        continue;  // 1.5 s 不进入目标车道过滤
      }
    }else{
      continue;
    }
    risk_side_agents_nodes_.push_back(target_lane_node);
  }
  return;
}
void LaneChangeStateMachineManager::AddRearAgentMerging() {
}
bool LaneChangeStateMachineManager::IfFrenetCollision(
    std::pair<double, double> l1, double v1, std::pair<double, double> l2,
    double v2, double max_time, double dt) {
  for (double t = 0.0; t <= max_time; t += dt) {
    double l1_s = l1.first + v1 * t;
    double l1_e = l1.second + v1 * t;
    double l2_s = l2.first + v2 * t;
    double l2_e = l2.second + v2 * t;

    if (!(l1_e <= l2_s || l2_e <= l1_s)) {
      return true;  // 碰撞
    }
  }
  return false;
}
bool LaneChangeStateMachineManager::IfFrenetCollision2D(
    std::pair<double, double> s1, double vs1, std::pair<double, double> s2,
    double vs2, std::pair<double, double> l1, double vl1,
    std::pair<double, double> l2, double vl2, double max_time, double dt) {
  for (double t = 0.0; t <= max_time; t += dt) {
    // s 方向位置
    double s1_s = s1.first + vs1 * t;
    double s1_e = s1.second + vs1 * t;
    double s2_s = s2.first + vs2 * t;
    double s2_e = s2.second + vs2 * t;

    // l 方向位置
    double l1_s = l1.first + vl1 * t;
    double l1_e = l1.second + vl1 * t;
    double l2_s = l2.first + vl2 * t;
    double l2_e = l2.second + vl2 * t;

    bool s_overlap = !(s1_e <= s2_s || s2_e <= s1_s);
    bool l_overlap = !(l1_e <= l2_s || l2_e <= l1_s);

    if (s_overlap && l_overlap) {
      return true;  // 同时重叠 -> 碰撞
    }
  }
  return false;  // 全程都没有同时重叠 -> 安全
}

bool LaneChangeStateMachineManager::CheckFrontRiskAgentTrajs(
    const planning_data::DynamicAgentNode* agent_node, bool is_large_car) {
  // 车辆参数
  const auto& vehicle_param =
      VehicleConfigurationContext::Instance()->get_vehicle_param();
  const double kEgoWidth = vehicle_param.width;
  const double kEgoLength = vehicle_param.length;
  const double kEgoFrontEdgeToRearAxleDistance =
      vehicle_param.front_edge_to_rear_axle;
  const double kEgoBackEdgeToRearAxleDistance =
      vehicle_param.rear_edge_to_rear_axle;
  // 障碍物参数
  const double agent_length = agent_node->node_length();
  const double agent_width = agent_node->node_width();

  const double prediction_default_rear_dis = 50.0;
  const double prediction_default_front_dis = 150.0;
  const double rear_dis_err = 5.0;
  const double front_dis_err = 145.0;
  const auto& agent_trajs = agent_node->node_trajectories_used_by_st_graph();
  if (agent_trajs.empty()) {
    return false;
  }
  const auto& agent_traj = agent_node->node_trajectories_used_by_st_graph()[0];
  // 安全阈值前置条件
  // 增加在接近ramp或者split时更激进的变道判断
  const auto& cur_lane = session_->environmental_model()
                             .get_virtual_lane_manager()
                             ->get_current_lane();
  if (!cur_lane) {
    return false;
  }
  const auto& route_info_output =
      session_->environmental_model().get_route_info()->get_route_info_output();

  // 根据当前速度计算触发激进导航变道的阈值
  const double kAggresiveMLCThreshold = ego_trajs_future_[0].v * 15;

  const bool is_mlc = transition_info_.lane_change_type == MAP_REQUEST;
  const bool is_aggresive_mlc =
      is_mlc && ((cur_lane->is_nearing_ramp_mlc_task() &&
                  route_info_output.dis_to_ramp < kAggresiveMLCThreshold) ||
                 (cur_lane->is_nearing_split_mlc_task() &&
                  route_info_output.distance_to_first_road_split <
                      kAggresiveMLCThreshold));

  double solid_safety_dist = 0;
  // 障碍物没有预测轨迹
  if (agent_traj.empty()) {
    return false;
  }
  // 确认判断的时间
  double max_lat_buff = 3.5; // unused
  double lat_buff = 3.5;
  const int iter_count = std::min(agent_traj.size(), ego_trajs_future_.size());
  double distance = 100.0;
  for (int i = 0; i < iter_count; i++) {
    if (i < 8) {
      lat_buff = max_lat_buff;
    } else if (i < 16) {
      lat_buff = max_lat_buff * 0.6;
    } else if (i < 24) {
      lat_buff = max_lat_buff * 0.3;
    } else {
      lat_buff = 0;
    }
    // 安全性判断 ： 原版安全检查
    //  if (std::abs(agent_traj[i].s - ego_trajs_future_[i].s) - two_car_length
    //  <
    //      safety_dist) {
    //    ILOG_DEBUG << "not safety !!!" << std::endl;
    //    return false;
    //  }
    const double focus_v = ego_trajs_future_[i].v;
    // 考虑目前自车的执行器响应时间为0.5s，在换道时纵向的稳态跟车距离为1*v；
    // 为了提高变道变道成功率，纵向又不至于减速太猛，因此设自车前方的最小安全距离为0.7*v。
    // 在距离匝道或者split距离小于200m时，变道优先级较高，那么最小安全距离设为0.5*v。
    double safety_dist_factor = is_aggresive_mlc ? 0.3 : 0.4;
    double safety_dist = safety_dist_factor * focus_v;
    // 如果两车速度差大于2m/s，那么安全距离可以降低至当前的0.8倍
    double rel_v = ego_trajs_future_[i].v - agent_traj[i].vel();
    bool is_large_v_diff = rel_v < -2;
    safety_dist =
        (is_large_v_diff && !is_large_car) ? safety_dist * 0.7 : safety_dist;

    // 新安全检查：
    // 自车box
    double ego_theta = ego_trajs_future_[i].heading_angle;
    Vec2d rac(ego_trajs_future_[i].x, ego_trajs_future_[i].y);
    Vec2d ego_center = rac + Vec2d::CreateUnitVec2d(ego_theta) *
                                 kEgoBackEdgeToRearAxleDistance;
    double ego_lateral_buff =
        2.0;  // 根据agent 类型选择 或者根据agent 尺寸选择  根据不同时刻选择
    planning_math::Box2d ego_box(ego_center, ego_theta, kEgoLength,
                                 kEgoWidth + ego_lateral_buff);
    // agent box
    Vec2d agent_rac(agent_traj[i].x(), agent_traj[i].y());
    double agent_theta = agent_traj[i].theta();
    // agent  risk buff
    double agent_lateral_buff =
        1.5;  // 根据agent 类型选择 或者根据agent 尺寸选择  根据不同时刻选择
    agent_lateral_buff = agent_node->is_VRU_type() ? 3.5 : 1.5;
    double agent_longitudinal_buff = 2.0 * safety_dist;
    planning_math::Box2d agent_box(agent_rac, agent_theta, agent_length,
                                   agent_width + agent_lateral_buff);
    double distance_i = ego_box.DistanceTo(agent_box);
    distance = std::min(distance, distance_i);
    // 记录box
    const auto& agent_corners = agent_box.GetAllCorners();
    for (const auto& corner_point : agent_corners) {
      agent_box_corners_x_.push_back(corner_point.x());
      agent_box_corners_y_.push_back(corner_point.y());
    }
    const auto& ego_corners = ego_box.GetAllCorners();
    for (const auto& corner_point : ego_corners) {
      ego_box_corners_x_.push_back(corner_point.x());
      ego_box_corners_y_.push_back(corner_point.y());
    }
  }
  if (distance < 0.01) {
    ILOG_DEBUG << "box-box not safety !!!";
    return false;
  }
  return true;
}
bool LaneChangeStateMachineManager::CheckMergingRearAgentTraj(
    const int merging_rear_agent_id) {
  // 目标车道信息
  int target_lane_virtual_id = lc_req_mgr_->target_lane_virtual_id();
  const auto& ref_path_manager =
      session_->environmental_model().get_reference_path_manager();
  if (!ref_path_manager) {
    return false;
  }
  const auto& target_ref_path =
      ref_path_manager->get_reference_path_by_lane(target_lane_virtual_id);
  const auto& target_lane_coord = target_ref_path->get_frenet_coord();
  if (!target_lane_coord) {
    return false;
  }
  // 获取自车参数
  const auto& vehicle_param =
      VehicleConfigurationContext::Instance()->get_vehicle_param();
  const double kEgoWidth = vehicle_param.width;
  const double kEgoLength = vehicle_param.length;
  const double kEgoFrontEdgeToRearAxleDistance =
      vehicle_param.front_edge_to_rear_axle;
  const double kEgoBackEdgeToRearAxleDistance =
      vehicle_param.rear_edge_to_rear_axle;
  // 获取后方车辆预测轨迹
  if (merging_rear_agent_id < 0) {
    return true;  // 无车，安全
  }
  const auto& agent_mgr =
      session_->environmental_model().get_dynamic_world()->agent_manager();
  const auto agent = agent_mgr->GetAgent(merging_rear_agent_id);
  if (agent == nullptr) {
    return true;
  }
  const auto agent_trajs = agent->trajectories_used_by_st_graph();
  if (agent_trajs.empty()) {
    return false;
  }
  const auto& trajectory_used_by_st_graph =
      agent->trajectories_used_by_st_graph()[0];
  // 预测轨迹赋值
  TrajectoryPoints agent_traj;
  for (int i = 0; i < trajectory_used_by_st_graph.size(); i++) {
    const auto& point = trajectory_used_by_st_graph[i];
    if (i > lc_safety_check_num_) {
      break;
    }
    TrajectoryPoint agent_point;
    agent_point.x = point.x();
    agent_point.y = point.y();
    double s = 0.0;
    double l = 0.0;
    if (!target_lane_coord->XYToSL(point.x(), point.y(), &s, &l)) {
      continue;  //
    }
    Point2D frenet_point(s, l);
    agent_point.s = frenet_point.x;
    agent_point.l = frenet_point.y;
    agent_point.t = point.absolute_time();
    agent_point.v = point.vel();
    agent_point.heading_angle = point.theta();  // add store heading angle
    agent_traj.emplace_back(agent_point);
  }
  // 障碍物没有预测轨迹(正常不会的)
  if (agent_traj.empty() || ego_trajs_future_.empty()) {
    return false;
  }
  // 确认判断的时间
  double max_lat_buff = 3.5; //Unused
  double lat_buff = 3.5;
  const int iter_count = std::min(agent_traj.size(), ego_trajs_future_.size());
  double distance = 100.0;
  double box_longitudinal_buff = 0.0;
  double box_ttc = lc_safety_check_time_;  // time 的计算已经有非零保护了
  double std_beyond_lane_time = 2.0;       // 实际在2.5 左右
  // 根据目标车速度 调整速度阈值
  std::array<double, 6> xp{10., 40., 60., 80.0, 100., 120.};  // 后车速度kph
  std::array<double, 6> fp{2.0, 6.0, 8.0,
                           12., 15., 20.};  // 触发变道需要预留最小空间
  bool is_executing =
      transition_info_.lane_change_status == kLaneChangeExecution;
  // 考虑假设侵入的轨迹
  TrajectoryPoints agent_switch_traj;
  double deceleration = -1.0;  // m/s^2, 后车减速
  double jerk = -2.0;
  bool is_large_agent = (agent::AgentType::BUS == agent->type() ||
                         agent::AgentType::TRUCK == agent->type() ||
                         agent::AgentType::TRAILER == agent->type() ||
                         agent->length() > kLargeAgentLengthM);
  if (is_large_agent) {
    deceleration = -0.7;  // 大车减速度小很多
  }
  if (is_executing && agent_traj.size() > 3 && !agent->is_static()) {
    GetDecelerationTraj(agent->accel_fusion(), agent_traj, agent_switch_traj,
                        deceleration, jerk, false);
  } else {
    agent_switch_traj = agent_traj;
  }
  for (int i = 0; i < iter_count; i++) {
    double agent_kph = agent_switch_traj[i].v * 3.6;
    const double dis_buff = interp(agent_kph, xp, fp);  // 距离/ 时间
    // 一定是后车
    box_ttc = lc_safety_check_time_ - i * 0.2;
    box_ttc = std::max(box_ttc, 0.0);
    double beyond_lane_time = std_beyond_lane_time - i * 0.2;
    beyond_lane_time = std::max(beyond_lane_time, 0.0);
    if (is_executing) {  // execution 阶段
      double agent_vel_i = agent_switch_traj[i].v;
      double back_agent_tts_s =
          agent_vel_i * box_ttc + 0.5 * deceleration * box_ttc * box_ttc;
      double ego_ttc_s = ego_trajs_future_[i].v * box_ttc;
      box_longitudinal_buff = std::max(back_agent_tts_s - ego_ttc_s, 0.);
    } else {  // proposal, hold 阶段
      // 根据速度差获得ttc 速度阈值
      double rel_vel = agent_switch_traj[i].v - ego_trajs_future_[i].v;
      // double dist_rel_vel = std::max(rel_vel * box_ttc, 0.0);
      // box_longitudinal_buff = std::max(dist_rel_vel, dis_buff);
      double dist_rel_vel =
          (rel_vel > 0) ? rel_vel * box_ttc : -rel_vel * beyond_lane_time;
      box_longitudinal_buff =
          (rel_vel > 0)
              ? std::max(dist_rel_vel, dis_buff)
              : dis_buff - dist_rel_vel;  // 1.5s 到达边界(实际观察约为2.5s)
      box_longitudinal_buff = std::max(box_longitudinal_buff, 2.0);
    }
    // 如果在变道返回状态下，将安全距离再度减小
    if (transition_info_.lane_change_status == kLaneChangeExecution) {
      box_longitudinal_buff = box_longitudinal_buff * 0.7;  // 滞回
    }
    //  横向新安全距离：
    if (i < 8) {
      lat_buff = max_lat_buff;
    } else if (i < 16) {
      lat_buff = max_lat_buff * 0.6;
    } else if (i < 24) {
      lat_buff = max_lat_buff * 0.3;
    } else {
      lat_buff = 0;
    }
    // 自车box
    double ego_theta = ego_trajs_future_[i].heading_angle;
    Vec2d rac(ego_trajs_future_[i].x, ego_trajs_future_[i].y);
    Vec2d ego_center = rac + Vec2d::CreateUnitVec2d(ego_theta) *
                                 kEgoBackEdgeToRearAxleDistance;
    planning_math::Box2d ego_box(ego_center, ego_theta,
                                 kEgoLength + box_longitudinal_buff,
                                 kEgoWidth + lat_buff);
    // agent box
    double agent_length = agent->length();
    double agent_width = agent->width();
    Vec2d agent_rac(agent_switch_traj[i].x, agent_switch_traj[i].y);
    double agent_theta = agent_switch_traj[i].heading_angle;
    planning_math::Box2d agent_box(agent_rac, agent_theta,
                                   agent_length + box_longitudinal_buff,
                                   agent_width + lat_buff);
    double distance_i = ego_box.DistanceTo(agent_box);
    distance = std::min(distance, distance_i);
    // 记录box
    const auto& agent_corners = agent_box.GetAllCorners();
    for (const auto& corner_point : agent_corners) {
      agent_box_corners_x_.push_back(corner_point.x());
      agent_box_corners_y_.push_back(corner_point.y());
    }
    const auto& ego_corners = ego_box.GetAllCorners();
    for (const auto& corner_point : ego_corners) {
      ego_box_corners_x_.push_back(corner_point.x());
      ego_box_corners_y_.push_back(corner_point.y());
    }
  }
  if (distance < 0.01) {
    ILOG_DEBUG << "box-box not safety !!!";
    return false;
  }
  return true;
}
bool LaneChangeStateMachineManager::IsLargeAgent(
    const planning_data::DynamicAgentNode* agent) {
  if (agent == nullptr) {
    return false;
  }
  return agent::AgentType::BUS == agent->type() ||
         agent::AgentType::TRUCK == agent->type() ||
         agent::AgentType::TRAILER == agent->type() ||
         agent->node_length() > kLargeAgentLengthM;
}

void LaneChangeStateMachineManager::CalculateLatCloseValue() {
  // 当前planning的规划周期为10Hz，安全性检查连续5帧通过则视为安全。
  // 若在10帧内未通过则采取自车向目标车道靠近的策略

  // 缓存环境模型引用，避免重复调用
  const auto& env_model = session_->environmental_model();
  const auto& virtual_lane_mgr = env_model.get_virtual_lane_manager();

  // 获取车辆宽度的一半
  const double ego_half_width =
      VehicleConfigurationContext::Instance()->get_vehicle_param().width / 2.0;

  // 获取当前车道并检查其有效性
  const auto& current_lane = virtual_lane_mgr->get_current_lane();
  if (!current_lane) {
    lat_close_boundary_offset_ = 0.0;  // 如果当前车道不存在，设置偏移为0
    return;
  }
  //前车或者后车不存在
  if(target_lane_front_node_ == nullptr || target_lane_rear_node_ == nullptr) {
    lat_close_boundary_offset_ = 0.0;
    return ;
  }
  // 自车 速度较快
  double ego_v = env_model.get_ego_state_manager()->ego_v();
  if(ego_v > 8.33) {
    lat_close_boundary_offset_ = 0.0;
    return ;
  }
  // 计算当前车道半宽
  const double cur_lane_half_width = current_lane->width() / 2.0;

  // 定义缓冲区和最小横向偏移值
  const double lat_offset_value =
      cur_lane_half_width - ego_half_width - lc_safety_check_config_.lat_offset_buffer;

  // 获取自车速度
  const double v_ego = env_model.get_ego_state_manager()->ego_v();

  // 定义连续通过的安全帧数阈值
  constexpr int SAFETY_FRAME_THRESHOLD = 6;

  // 判断是否满足安全条件并设置横向偏移值
  if (fix_lane_congestion_level_.level == CongestionLevel::CONGESTION &&
      lat_offset_value > 0.0 &&
      propose_state_frame_nums_ > SAFETY_FRAME_THRESHOLD) {
    lat_close_boundary_offset_ =
        (transition_info_.lane_change_direction == LEFT_CHANGE)
            ? lat_offset_value
            : -lat_offset_value;
  } else {
    lat_close_boundary_offset_ = 0.0;  // 不满足安全条件时，设置偏移为0
  }
}
void LaneChangeStateMachineManager::IsEgoOnSideLane() {
  const auto& virtual_lane_manager =
      session_->environmental_model().get_virtual_lane_manager();
  auto& lane_change_decider_output = session_->mutable_planning_context()
                                         ->mutable_lane_change_decider_output();
  const auto& left_lane = virtual_lane_manager->get_left_lane();
  const auto& right_lane = virtual_lane_manager->get_right_lane();
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
void LaneChangeStateMachineManager::CalculateCongestionLatOffsetValue() {
  if(is_pre_move_){ // 已经判定提前移动则保持 lat_close_boundary_offset_ ，直到变道执行,避免蛇形
    return;
  }
  // is_pre_move_ = false:
  lat_close_boundary_offset_ = 0.0;
  const auto& virtual_lane_mgr = session_->environmental_model().get_virtual_lane_manager();
  int target_lane_virtual_id = lc_req_mgr_->target_lane_virtual_id();
  const auto& current_lane = virtual_lane_mgr->get_current_lane();
  const auto& target_lane = virtual_lane_mgr->get_lane_with_virtual_id(target_lane_virtual_id);
  //紧急变道类型不premove
  if(transition_info_.lane_change_type ==  DYNAMIC_AGENT_EMERGENCE_AVOID_REQUEST){
    return;
  }
  if (!target_lane) {
    return;
  }
  // 排除不应该触发贴变行为的case：
  //自车速度较快
  double ego_vel = session_->environmental_model().get_ego_state_manager()->ego_v();
  if(ego_vel > 8.33){
    return;
  }
  // 前方或者后方无车，可调速
  if(target_lane_front_node_ == nullptr || target_lane_rear_node_ == nullptr){
    return;
  }
  // proposed state 帧数不多，可以变道只是正常计数：正常5 帧安全候触发
  if(propose_state_frame_nums_ < 5){
    return;
  }
  // 大曲率抑制premove
  if(lane_change_stage_info_.lc_invalid_reason == "target lane too large curve") {
    return;
  }
  //目标车道附近平均速度
  const auto& target_lane_nodes =
  session_->environmental_model().get_dynamic_world()->GetNodesByLaneId(
      target_lane_virtual_id);
  std::vector<const planning_data::DynamicAgentNode *> target_lane_nodes_vec;
  for(const auto& node : target_lane_nodes){
      target_lane_nodes_vec.push_back(node);
  }
  if(target_lane_nodes_vec.size() < 3){
    return;
  }
  const auto& ref_path = target_lane->get_reference_path();
  if (!ref_path) {
    return;
  }
  const double ego_s = ref_path->get_frenet_ego_state().s();
  std::sort(target_lane_nodes_vec.begin(), target_lane_nodes_vec.end(),
            [ego_s](const auto* a, const auto* b) {
              return std::fabs(a->node_s() - ego_s) <
                     std::fabs(b->node_s() - ego_s);
            });
  double target_lane_avg_speed = 0.0;
  double sum_speed = 0.0;
  int valid_cnt = 0;
  for (const auto* node : target_lane_nodes_vec) {
    if(node == nullptr){
      continue;
    }
    sum_speed += node->node_speed();
    ++valid_cnt;
    if (valid_cnt >= 4) {
      break;
    }
  }
  valid_cnt = std::min(valid_cnt, 4);
  target_lane_avg_speed = valid_cnt > 1 ? sum_speed / valid_cnt : 0.0;
  if(target_lane_avg_speed > 7.0 || target_lane_avg_speed < 1.0){
    return; // 快速或者停车场景
  }
  if (!current_lane) {
    return;
  }
  const double cur_lane_half_width = current_lane->width() / 2.0;
  const double ego_half_width =
      VehicleConfigurationContext::Instance()->get_vehicle_param().width / 2.0;
  const double lat_offset_value =
      cur_lane_half_width - ego_half_width - lc_safety_check_config_.lat_offset_buffer;
  if (lat_offset_value <= 0.0) {
    return;
  }

  if (transition_info_.lane_change_direction == LEFT_CHANGE) {
    lat_close_boundary_offset_ = lat_offset_value;
    is_pre_move_ = true;
  } else if (transition_info_.lane_change_direction == RIGHT_CHANGE) {
    lat_close_boundary_offset_ = -lat_offset_value;
    is_pre_move_ = true;
  } else {
    return;
  }
}

bool LaneChangeStateMachineManager::IsLatOffsetValid() const {
  const auto& cur_path = session_->environmental_model()
                             .get_reference_path_manager()
                             ->get_reference_path_by_current_lane();
  const auto& frenet_ego_state = cur_path->get_frenet_ego_state();
  const double ego_l = frenet_ego_state.l();
  const double lat_offset_threshold = 0.5;
  const double lat_offset = std::abs(ego_l);

  bool is_high_priority_complete_mlc = IsHighPriorityCompleteMLC();

  if (lat_offset < lat_offset_threshold || is_high_priority_complete_mlc) {
    return true;
  }
  return false;
}
bool LaneChangeStateMachineManager::IsLCFeasibleForTrafficCone(
    const planning_data::DynamicAgentNode* traffic_cone) const {
  const auto& current_lane = session_->environmental_model()
                                 .get_reference_path_manager()
                                 ->get_reference_path_by_current_lane();
  const auto& current_lane_coord = current_lane->get_frenet_coord();
  if (!current_lane_coord) {
    return false;
  }
  //当前车道半宽度
  double half_origin_lane = 1.75;
  const auto origin_lane = session_->environmental_model().get_virtual_lane_manager()
                          ->get_lane_with_virtual_id(lc_lane_mgr_->origin_lane_virtual_id());
  if (origin_lane != nullptr) {
    half_origin_lane = origin_lane->width() * 0.5;
  }
  const auto& ego_state =
      session_->environmental_model().get_ego_state_manager();
  const double ego_need_dis = ego_state->ego_v() * kEgoReachBoundaryTime;
  // Point2D frenet_point;
  // Point2D traffic_cone_pt{traffic_cone->node_x(), traffic_cone->node_y()};
  // if (!current_lane_coord->XYToSL(traffic_cone_pt, frenet_point)) {
  //   return false;
  // }
  double s = 0.0;
  double l = 0.0;
  if (!current_lane_coord->XYToSL(traffic_cone->node_x(),
                                  traffic_cone->node_y(), &s, &l)) {
    return false;  // 更换xytosl
  }
  Point2D frenet_point(s, l);
  double ego_to_cone_distacne = frenet_point.x - current_lane->get_frenet_ego_state().s();
  if (ego_to_cone_distacne > ego_need_dis || ego_to_cone_distacne < 0.0) {
    // 在变道所需的纵向距离之外，可以不用考虑，可以直接变道
    return true;
  } else {
    // TODO(fengwang31):暂时把traffic_cone的横向距离限制距离中心线在1m以外则认为安全
    // 后续根据自车的轨迹点是否与锥桶有overlap来判断是否安全
    double approx_lat_offset = std::min(ego_to_cone_distacne / ego_need_dis, 1.0) * half_origin_lane;
    //原方案假设 3V 纵向距离对应自车横向运动半个车道
    if (transition_info_.lane_change_direction == RIGHT_CHANGE &&
        frenet_point.y > 1.0 - approx_lat_offset) {
      return true;
    } else if (transition_info_.lane_change_direction == LEFT_CHANGE &&
               frenet_point.y < -1.0 + approx_lat_offset) {
      return true;
    }
  }
  return false;
}

bool LaneChangeStateMachineManager::IsLCFeasibleForTrafficConeInTargetLane(
    const planning_data::DynamicAgentNode* traffic_cone,
    const int target_lane_virtual_id) const {
  const auto& reference_path_manager =
      session_->environmental_model().get_reference_path_manager();
  const auto& target_ref_path =
      reference_path_manager->get_reference_path_by_lane(
          target_lane_virtual_id);
  if (!target_ref_path) {
    return false;
  }

  const auto& target_lane_coord = target_ref_path->get_frenet_coord();
  if (!target_lane_coord) {
    return false;
  }

  const auto& ego_state =
      session_->environmental_model().get_ego_state_manager();
  if (!ego_state) {
    return false;
  }

  const auto& vehicle_param =
      VehicleConfigurationContext::Instance()->get_vehicle_param();
  const double kEgoWidth = vehicle_param.width;

  const double ego_need_dis = ego_state->ego_v() * kEgoReachBoundaryTime;

  // Point2D traffic_cone_frenet_pt;
  // Point2D traffic_cone_pt{traffic_cone->node_x(), traffic_cone->node_y()};
  // if (!target_lane_coord->XYToSL(traffic_cone_pt, traffic_cone_frenet_pt)) {
  //   return false;
  // }

  // Point2D ego_frenet_point;
  // Point2D ego_pt{ego_state->ego_pose().x, ego_state->ego_pose().y};
  // if (!target_lane_coord->XYToSL(ego_pt, ego_frenet_point)) {
  //   return false;
  // }
  // traffic_cone
  double cone_s = 0.0;
  double cone_l = 0.0;
  if (!target_lane_coord->XYToSL(traffic_cone->node_x(), traffic_cone->node_y(),
                                 &cone_s, &cone_l)) {
    return false;  // 更换xytosl
  }
  Point2D traffic_cone_frenet_pt(cone_s, cone_l);
  // ego
  double ego_s = 0.0;
  double ego_l = 0.0;
  if (!target_lane_coord->XYToSL(ego_state->ego_pose().x,
                                 ego_state->ego_pose().y, &ego_s, &ego_l)) {
    return false;  // 更换xytosl
  }
  Point2D ego_frenet_point(ego_s, ego_l);
  if (traffic_cone_frenet_pt.x - ego_frenet_point.x > ego_need_dis) {
    // 在变道所需的纵向距离之外，可以不用考虑，可以直接变道
    return true;
  } else {
    // TODO(fengwang31):暂时把traffic_cone的横向距离限制距离半车宽以外则认为安全
    // 后续根据自车的轨迹点是否与锥桶有overlap来判断是否安全
    if (transition_info_.lane_change_direction == RIGHT_CHANGE &&
        traffic_cone_frenet_pt.y < -(kEgoWidth / 2 + 0.1)) {
      return true;
    } else if (transition_info_.lane_change_direction == LEFT_CHANGE &&
               traffic_cone_frenet_pt.y > (kEgoWidth / 2 + 0.1)) {
      return true;
    }
  }
  return false;
}

bool LaneChangeStateMachineManager::IsNotNeedLCBackForTrafficConeInTargetLane(
    const planning_data::DynamicAgentNode* traffic_cone,
    const int target_lane_virtual_id, const double t_remain_lc) const {
  const auto& reference_path_manager =
      session_->environmental_model().get_reference_path_manager();
  const auto& target_ref_path =
      reference_path_manager->get_reference_path_by_lane(
          target_lane_virtual_id);
  if (!target_ref_path) {
    return false;
  }

  const auto& target_lane_coord = target_ref_path->get_frenet_coord();
  if (!target_lane_coord) {
    return false;
  }

  const auto& ego_state =
      session_->environmental_model().get_ego_state_manager();
  if (!ego_state) {
    return false;
  }

  const auto& vehicle_param =
      VehicleConfigurationContext::Instance()->get_vehicle_param();
  const double kEgoWidth = vehicle_param.width;
  const double kEgoLength = vehicle_param.length;
  // 目前是判断t_remain_lc时间到达跨线点
  const double ego_need_dis = ego_state->ego_v() * t_remain_lc;

  // Point2D traffic_cone_frenet_pt;
  // Point2D traffic_cone_pt{traffic_cone->node_x(), traffic_cone->node_y()};
  // if (!target_lane_coord->XYToSL(traffic_cone_pt, traffic_cone_frenet_pt)) {
  //   return false;
  // }

  // Point2D ego_frenet_point;
  // Point2D ego_pt{ego_state->ego_pose().x, ego_state->ego_pose().y};
  // if (!target_lane_coord->XYToSL(ego_pt, ego_frenet_point)) {
  //   return false;
  // }
  double cone_s = 0.0;
  double cone_l = 0.0;
  if (!target_lane_coord->XYToSL(traffic_cone->node_x(), traffic_cone->node_y(),
                                 &cone_s, &cone_l)) {
    return false;  // 更换xytosl
  }
  Point2D traffic_cone_frenet_pt(cone_s, cone_l);
  double ego_s = 0.0;
  double ego_l = 0.0;
  if (!target_lane_coord->XYToSL(ego_state->ego_pose().x,
                                 ego_state->ego_pose().y, &ego_s, &ego_l)) {
    return false;  // 更换xytosl
  }
  Point2D ego_frenet_point(ego_s, ego_l);
  if (traffic_cone_frenet_pt.x - ego_frenet_point.x <
      ego_need_dis - kEgoLength) {
    // 在跨线点之前，可以不用考虑，可以直接变道
    return true;
  } else {
    // TODO(fengwang31):暂时把traffic_cone的横向距离限制距离半车宽以外则认为安全
    // 后续根据自车的轨迹点是否与锥桶有overlap来判断是否安全

    if (traffic_cone_frenet_pt.y > -(kEgoWidth / 2 + 0.1) &&
        traffic_cone_frenet_pt.y < (kEgoWidth / 2 + 0.1)) {
      // 在车道中间，需要返回
      return false;
    } else {
      // 在车道边上
      if ((traffic_cone_frenet_pt.x - ego_frenet_point.x >=
           ego_need_dis - kEgoLength) &&
          (traffic_cone_frenet_pt.x - ego_frenet_point.x <=
           ego_need_dis + kEgoLength)) {
        if (transition_info_.lane_change_direction == RIGHT_CHANGE &&
            traffic_cone_frenet_pt.y > -(kEgoWidth / 2 + 0.1)) {
          return false;
        } else if (transition_info_.lane_change_direction == LEFT_CHANGE &&
                   traffic_cone_frenet_pt.y < (kEgoWidth / 2 + 0.1)) {
          return false;
        }
      }
    }
  }

  return true;
}

const std::vector<double> LaneChangeStateMachineManager::GetObjsDebugInfo(
    const double obj_v, const double obj_a, const double obj_t,
    const double obj_s) const {
  // 暂时是预测4s后障碍物的运动轨迹
  const int iter = obj_t * 10;
  std::vector<double> obj_s_vec;
  for (int i = 0; i < iter; i++) {
    double s = obj_v * (i * 0.1) + 0.5 * obj_a * (i * 0.1) * (i * 0.1) + obj_s;
    obj_s_vec.push_back(s);
  }
  return obj_s_vec;
}
bool LaneChangeStateMachineManager::IsDashLineCurBoundary(
    const RequestType lc_direction) const {
  const auto& cur_lane = session_->environmental_model()
                             .get_virtual_lane_manager()
                             ->get_current_lane();
  const auto& route_info_output =
      session_->environmental_model().get_route_info()->get_route_info_output();
  if ((cur_lane->is_nearing_ramp_mlc_task() &&
       route_info_output.dis_to_ramp < 100) ||
      (cur_lane->is_nearing_split_mlc_task() &&
       route_info_output.distance_to_first_road_split < 100)) {
    // 当接近匝道或split区域距离小于100m时，实线也可以变道
    return true;
  }
  iflyauto::LaneBoundaryType boundary_type =
      MakesureCurrentBoundaryType(lc_direction);
  const bool is_dashed_line =
      (boundary_type ==
           iflyauto::LaneBoundaryType::LaneBoundaryType_MARKING_DASHED ||
       boundary_type == iflyauto::LaneBoundaryType::
                            LaneBoundaryType_MARKING_DECELERATION_DASHED ||
       boundary_type ==
           iflyauto::LaneBoundaryType::LaneBoundaryType_MARKING_VIRTUAL);
  return is_dashed_line;
}

RequestType LaneChangeStateMachineManager::CalculaTurnSignalForHPP() {
  const auto& cur_reference_path = session_->environmental_model()
                                       .get_reference_path_manager()
                                       ->get_reference_path_by_current_lane();
  const auto& ego_state =
      session_->environmental_model().get_ego_state_manager();
  const double ego_v = ego_state->ego_v();
  const double t_defuault_pre_light = 1.0;
  const double defuault_road_radius = 20.0;
  const double pre_dis = t_defuault_pre_light * ego_v;
  const double ego_s = cur_reference_path->get_frenet_ego_state().s();

  ReferencePathPoint front_reference_path_point;
  if (!cur_reference_path->get_reference_point_by_lon(
          ego_s + pre_dis, front_reference_path_point)) {
    ILOG_DEBUG << "get front_reference_path_point failed!!!";
    return NO_CHANGE;
  }
  // 以0.5m为间隔，判断前方2.5m的距离内，是否车道线半径小于defuault_road_radius
  const double s_interval = 0.5;
  const double s_start = front_reference_path_point.path_point.s();
  const double s_end = s_start + 5 * s_interval;
  for (double s = s_start; s <= s_end + 1e-6; s += s_interval) {
    ReferencePathPoint temp_ref_path_point;
    if (!cur_reference_path->get_reference_point_by_lon(s,
                                                        temp_ref_path_point)) {
      ILOG_DEBUG << "get front_reference_path_point failed!!!";
      return NO_CHANGE;
    }

    if (std::abs(temp_ref_path_point.path_point.kappa()) <
        1 / defuault_road_radius) {
      ILOG_DEBUG << "front no turn";
      return NO_CHANGE;
    }
  }

  if (front_reference_path_point.path_point.kappa() > 0) {
    return LEFT_CHANGE;
  } else {
    return RIGHT_CHANGE;
  }
}
void LaneChangeStateMachineManager::CalculateLCGapFeasibleWithPredictionInfo(
    LaneChangeStageInfo* const lc_state_info,
    const planning_data::DynamicAgentNode* agent_node,
    const bool is_front_agent, const bool is_ego_lane_agent,
    const bool is_side_obs) {
  // get agent prediction trajs
  bool is_invalid_fiter_agent = false;
  const planning_data::DynamicAgentNode* after_filter_agent = agent_node;
  const auto agent_prediction_trajs = CalculateAgentPredictionTrajs(
      agent_node, is_front_agent, is_ego_lane_agent, &after_filter_agent,
      is_side_obs);

  // 过滤掉旁车道障碍物后，没有车的case，则是安全的，直接退出
  if (after_filter_agent == nullptr && agent_prediction_trajs.empty()) {
    return;
  }

  // check safety for prediction trajs
  if (after_filter_agent == nullptr) {
    after_filter_agent = agent_node;
  }

  if (is_front_agent) {
    lc_state_info->lc_gap_info.front_node_id = after_filter_agent->node_id();
  } else {
    lc_state_info->lc_gap_info.rear_node_id = after_filter_agent->node_id();
  }

  bool lc_safety = true;
  if (FacilityTypes.count(after_filter_agent->type()) > 0 &&
      is_front_agent) {
    const int target_lane_virtual_id = lc_req_mgr_->target_lane_virtual_id();
    if (transition_info_.lane_change_status == kLaneChangePropose) {
      lc_safety = IsLCFeasibleForTrafficConeInTargetLane(
          after_filter_agent, target_lane_virtual_id);
    } else if (transition_info_.lane_change_status == kLaneChangeExecution) {
      lc_safety = IsNotNeedLCBackForTrafficConeInTargetLane(
          after_filter_agent, target_lane_virtual_id, lc_safety_check_time_);
    }
  } else {
    // 如果前方障碍物有减速趋势，假设自车减速趋势来判定是否变道
    int solid_index = 0;

    if (!agent_prediction_trajs.empty()) {
      const double agent_future_v = agent_prediction_trajs.rbegin()->v;
      const double agent_current_v = agent_prediction_trajs.begin()->v;
      const auto& ego_state_manager =
          session_->environmental_model().get_ego_state_manager();

      const double ego_current_v = ego_trajs_future_[0].v;
      const double ego_finally_v = ego_trajs_future_.back().v;

      // 车辆参数
      const auto& vehicle_param =
          VehicleConfigurationContext::Instance()->get_vehicle_param();
      const double kEgoFrontEdgeToRearAxleDistance =
          vehicle_param.front_edge_to_rear_axle;
      double two_car_length = kEgoFrontEdgeToRearAxleDistance +
                              after_filter_agent->node_length() * 0.5;

      const int last_point_index =
          std::min(agent_prediction_trajs.size(), ego_trajs_future_.size()) - 1;
      const double last_point_rel_dis =
          agent_prediction_trajs[last_point_index].s -
          ego_trajs_future_[last_point_index].s - two_car_length;
      const double safety_dis_threshold = std::max(0.1 * ego_finally_v, 1.0);
      // 正在修改idm推出的速度和s 我理解未来这里可以不需要了
      // 前方障碍物车辆的未来车速比自车小，且在安全距离之内的，那么需要考虑纵向在减速至目标车速过程中，与障碍物是否在安全距离之外

      // if (is_front_agent && agent_future_v < ego_finally_v &&
      //     last_point_rel_dis < safety_dis_threshold) {
      //   const double tager_v = agent_future_v;

      //   int itear_num = ego_trajs_future_.size();

      //   //
      //   要考虑自车的执行器0.5s的响应时间，因此假设在0.5s后，自车才开始以最大减速度减速
      //   for (int i = 0; i < itear_num; i++) {
      //     if (ego_trajs_future_[i].t > 0.5) {
      //       solid_index = i;
      //       break;
      //     }
      //   }

      //   const double ego_virtual_v = ego_trajs_future_[solid_index].v;

      //   const auto ego_max_deceleration_curve =
      //       GenerateEgoMaxDecelerationCurve(ego_virtual_v, tager_v);

      //   bool is_reach_target_v = false;
      //   int index_reach_target_v = 0;
      //   for (int i = solid_index; i < itear_num; i++) {
      //     const double t =
      //         ego_trajs_future_[i].t - ego_trajs_future_[solid_index].t;
      //     const double solid_s = ego_trajs_future_[solid_index].s;
      //     //自车减速到目标车速后，那么假定自车匀速行驶
      //     if (!is_reach_target_v) {
      //       //在减速中
      //       ego_trajs_future_[i].v = ego_max_deceleration_curve.Evaluate(1,
      //       t); ego_trajs_future_[i].s =
      //           solid_s + ego_max_deceleration_curve.Evaluate(0, t);
      //       lc_egos_vec_[i] = ego_trajs_future_[i].s -
      //       ego_trajs_future_[0].s; if
      //       (ego_max_deceleration_curve.Evaluate(1, t) < tager_v) {
      //         is_reach_target_v = true;
      //         index_reach_target_v = i;
      //       }
      //     } else {
      //       //减速到目标车速后
      //       ego_trajs_future_[i].v =
      //       ego_trajs_future_[index_reach_target_v].v; const double delta_t =
      //       ego_trajs_future_[i].t -
      //                              ego_trajs_future_[index_reach_target_v].t;
      //       ego_trajs_future_[i].s =
      //           ego_trajs_future_[index_reach_target_v].s +
      //           ego_trajs_future_[index_reach_target_v].v * delta_t;
      //       lc_egos_vec_[i] = ego_trajs_future_[i].s -
      //       ego_trajs_future_[0].s;
      //     }
      //   }
      // }
    }

    // 保存调试信息，
    for (int i = 0; i < agent_prediction_trajs.size(); i++) {
      if (is_front_agent) {
        front_obj_future_v_.emplace_back(agent_prediction_trajs[i].v);
      } else {
        rear_obj_future_v_.emplace_back(agent_prediction_trajs[i].v);
      }
    }

    for (int i = 0; i < ego_trajs_future_.size(); i++) {
      ego_future_v_.emplace_back(ego_trajs_future_[i].v);
    }

    const bool is_large_car_in_side =
        is_front_agent ? false : IsLargeAgent(after_filter_agent);

    lc_safety = CheckIfSafetyForPredictionTrajs(  // 安全检查函数 target  node
        agent_prediction_trajs, after_filter_agent, is_large_car_in_side,
        is_front_agent);
    // lc_safety = CheckIfSafetyForOptimizedTrajs(agent_prediction_trajs,
    //   after_filter_agent, is_large_car_in_side, is_front_agent);
  }

  if (!lc_safety) {
    const double node_v = after_filter_agent->node_speed();
    const double distance_rel =
        is_front_agent
            ? after_filter_agent->node_back_edge_to_ego_front_edge_distance()
            : -after_filter_agent->node_front_edge_to_ego_back_edge_distance();

    if (transition_info_.lane_change_status == kLaneChangeExecution) {
      lc_state_info->lc_should_back = true;
      if (is_front_agent) {
        lc_state_info->lc_back_reason = "front view back";
      } else {
        lc_state_info->lc_back_reason = "side view back";
      }
      lc_back_track_.set_value(after_filter_agent->node_agent_id(),
                               distance_rel, node_v);
    } else if (transition_info_.lane_change_status == kLaneChangePropose ||
               transition_info_.lane_change_status == kLaneChangeHold) {
      lc_invalid_track_.set_value(after_filter_agent->node_agent_id(),
                                  distance_rel, node_v);
      lc_state_info->gap_insertable = false;
      if (is_front_agent) {
        lc_state_info->lc_invalid_reason = "front view invalid";
      } else {
        lc_state_info->lc_invalid_reason = "side view invalid";
      }
    }
  }
}
TrajectoryPoints LaneChangeStateMachineManager::CalculateAgentPredictionTrajs(
    const planning_data::DynamicAgentNode* agent_node,
    const bool is_front_agent, const bool is_ego_lane_agent,
    const planning_data::DynamicAgentNode** after_filter_agent,
    const bool is_side_obs) {
  TrajectoryPoints agent_prediction_trajs;
  const auto& virtual_lane_manager =
      session_->environmental_model().get_virtual_lane_manager();
  const auto& lane_change_decider_output =
      session_->planning_context().lane_change_decider_output();
  const int target_lane_virtual_id = lc_req_mgr_->target_lane_virtual_id();
  const auto& target_lane =
      virtual_lane_manager->get_lane_with_virtual_id(target_lane_virtual_id);
  if (target_lane == nullptr) {
    return agent_prediction_trajs;
  }

  const auto& reference_path_manager =
      session_->environmental_model().get_reference_path_manager();
  const auto& target_reference_path =
      reference_path_manager->get_reference_path_by_lane(
          target_lane_virtual_id);
  if (target_reference_path == nullptr) {
    return agent_prediction_trajs;
  }

  const auto& target_lane_coor = target_reference_path->get_frenet_coord();
  if (!target_lane_coor) {
    return agent_prediction_trajs;
  }

  const auto& dynamic_world =
      session_->environmental_model().get_dynamic_world();
  const planning_data::DynamicAgentNode* temp_agent_node = agent_node;
  // while (IsFilterAgent(temp_agent_node, target_lane_coor,
  //                      &agent_prediction_trajs, is_ego_lane_agent,
  //                      is_front_agent)) {
  //   if (!agent_prediction_trajs.empty()) {
  //     agent_prediction_trajs.clear();
  //   }

  //   int64_t target_node_id = is_front_agent ? temp_agent_node->front_node_id()
  //                                           : temp_agent_node->rear_node_id();
  //   temp_agent_node = dynamic_world->GetNode(target_node_id);

  //   *after_filter_agent = temp_agent_node;

  //   if (!temp_agent_node) {
  //     return agent_prediction_trajs;
  //   }
  // }
  // 不在内部沿while过滤障碍物，造成id不一致
  if (after_filter_agent) {
    *after_filter_agent = agent_node;
  }
  BuildAgentPredictionTrajsInTargetLane(agent_node, target_lane_coor, is_front_agent, &agent_prediction_trajs, is_side_obs);
  // StoreObjDebugPredictionInfo(agent_node, &agent_prediction_trajs,
  //                             is_front_agent, is_ego_lane_agent);
  return agent_prediction_trajs;
}

TrajectoryPoints LaneChangeStateMachineManager::CalculateEgoFutureTrajs()
    const {
  TrajectoryPoints ego_trajs_future;
  const auto& virtual_lane_manager =
      session_->environmental_model().get_virtual_lane_manager();
  const auto& lane_change_decider_output =
      session_->planning_context().lane_change_decider_output();
  const int target_lane_virtual_id = lc_req_mgr_->target_lane_virtual_id();
  const auto& target_lane =
      virtual_lane_manager->get_lane_with_virtual_id(target_lane_virtual_id);
  if (target_lane == nullptr) {
    return ego_trajs_future;
  }
  const auto& reference_path_manager =
      session_->environmental_model().get_reference_path_manager();
  const auto& target_reference_path =
      reference_path_manager->get_reference_path_by_lane(
          target_lane_virtual_id);
  if (target_reference_path == nullptr) {
    return ego_trajs_future;
  }

  const auto& target_frenet_ego_state =
      target_reference_path->get_frenet_ego_state();
  const double ego_s = target_frenet_ego_state.s();
  const double ego_v = target_frenet_ego_state.velocity();
  const double ego_a = target_frenet_ego_state.acc();

  const auto& planning_init_point =
      target_frenet_ego_state.planning_init_point();

  std::array<double, 3> init_lon_state = {0, ego_v, ego_a};

  auto virtual_acc_curve = MakeVirtualZeroAccCurve(init_lon_state);
  // 因为现在预测是以0.2s的时间间隔发的预测轨迹点
  for (int i = 0; i <= lc_safety_check_num_; ++i) {
    TrajectoryPoint ego_traj_future;
    ego_traj_future.s = ego_s + virtual_acc_curve->Evaluate(0, i * 0.2);
    ego_traj_future.t = i * 0.2;
    ego_traj_future.v = virtual_acc_curve->Evaluate(1, i * 0.2);
    ego_trajs_future.emplace_back(ego_traj_future);
  }
  return ego_trajs_future;
}
TrajectoryPoints LaneChangeStateMachineManager::CalculateEgoPPIDMTrajs() {
  TrajectoryPoints ego_trajs_future;
  const auto& cur_state =
      session_->planning_context().lane_change_decider_output().curr_state;
  const auto& reference_path_manager =
      session_->environmental_model().get_reference_path_manager();
  const int fix_lane_virtual_id = lc_lane_mgr_->fix_lane_virtual_id();
  const int target_lane_virtual_id = lc_lane_mgr_->target_lane_virtual_id();
  // 防止在跳execution那一帧，cur_lane没有更新，所以这里使用fix lane
  const auto& target_lane_ref_path =
      reference_path_manager->get_reference_path_by_lane(
          target_lane_virtual_id);
  if (!target_lane_ref_path) {
    return ego_trajs_future;
  }
  lc_path_generate_ = std::make_shared<LaneChangePathGenerateManager>(
      target_lane_ref_path, session_, config_builder_);
  lc_path_generate_->get_lc_path_result().reset();
  // lat_close_boundary_offset_
  lc_path_generate_->GenerateEgoFutureTrajectory(
      0.0, target_lane_front_node_);  // 失败处理

  // const auto lc_path_result = lc_path_generate_->get_lc_path_result();

  // auto &traj_points = session_->mutable_planning_context()
  //                         ->mutable_lane_change_decider_output()
  //                         .coarse_planning_info.trajectory_points;

  // UpdateLCPath(traj_points, lc_path_result, fix_lane_ref_path);
  ego_trajs_future = lc_path_generate_->get_ego_future_trajectory();
  return ego_trajs_future;
}

TrajectoryPoints LaneChangeStateMachineManager::CalculateEgoPPIDMTrajs(
    const planning_data::DynamicAgentNode* front_agent_node) {
  TrajectoryPoints ego_trajs_future;
  const auto& cur_state =
      session_->planning_context().lane_change_decider_output().curr_state;
  const auto& reference_path_manager =
      session_->environmental_model().get_reference_path_manager();
  const int fix_lane_virtual_id = lc_lane_mgr_->fix_lane_virtual_id();
  const int target_lane_virtual_id = lc_lane_mgr_->target_lane_virtual_id();
  // 防止在跳execution那一帧，cur_lane没有更新，所以这里使用fix lane
  const auto& target_lane_ref_path =
      reference_path_manager->get_reference_path_by_lane(
          target_lane_virtual_id);
  if (!target_lane_ref_path) {
    return ego_trajs_future;
  }
  lc_path_generate_ = std::make_shared<LaneChangePathGenerateManager>(
      target_lane_ref_path, session_);
  lc_path_generate_->get_lc_path_result().reset();
  lc_path_generate_->GenerateEgoFutureTrajectory(lat_close_boundary_offset_,
                                                 front_agent_node);  // 失败处理

  // const auto lc_path_result = lc_path_generate_->get_lc_path_result();

  // auto &traj_points = session_->mutable_planning_context()
  //                         ->mutable_lane_change_decider_output()
  //                         .coarse_planning_info.trajectory_points;

  // UpdateLCPath(traj_points, lc_path_result, fix_lane_ref_path);
  ego_trajs_future = lc_path_generate_->get_ego_future_trajectory();
  return ego_trajs_future;
}
bool LaneChangeStateMachineManager::CheckIfSafetyForOptimizedTrajs(
    const TrajectoryPoints& agent_traj,
    const planning_data::DynamicAgentNode* agent_node, bool is_large_car,
    const bool is_front_agent) {  // 优化后的自车轨迹和障碍物轨迹检查
  // 如果是前车，则检查自车轨迹末端距离障碍物轨迹末端是否安全
  // 如果是后车，则检查障碍物轨迹末端距离自车轨迹末端是否安全
  // 安全距离由辆车长度加上一个和差速相关的距离即可
  const auto& vehicle_param =
      VehicleConfigurationContext::Instance()->get_vehicle_param();
  const double kEgoLength = vehicle_param.length;
  const double kEgoFrontEdgeToRearAxleDistance =
      vehicle_param.front_edge_to_rear_axle;
  const double kEgoBackEdgeToRearAxleDistance =
      vehicle_param.rear_edge_to_rear_axle;
  const double agent_length = agent_node->node_length();
  const double ego_front_edge =
      ego_trajs_future_.back().s + kEgoFrontEdgeToRearAxleDistance;
  const double ego_rear_edge =
      ego_trajs_future_.back().s - kEgoBackEdgeToRearAxleDistance;
  const double agent_front_edge = agent_traj.back().s + agent_length * 0.5;
  const double agent_rear_edge =
      agent_traj.back().s -
      agent_length * 0.5;  // 注意障碍物预测轨迹存储时候考虑车长没
  const double agent_end_vel = agent_traj.back().v;
  const double ego_end_vel = ego_trajs_future_.back().v;
  const double ego_brake = 4.0;
  const double agent_yeild = 2.0;
  const double a_max = 1.0;
  const double b_max = 1.5;
  //因为是预测轨迹末端计算了，阈值激进
  // double front_margin = (ego_end_vel - agent_end_vel) * ego_end_vel / (2.0 *
  // std::sqrt(a_max * b_max)); front_margin = std::max(front_margin, 0.0);
  // double rear_margin = (agent_end_vel - ego_end_vel) * agent_end_vel  / (2.0
  // * std::sqrt(a_max * b_max)); rear_margin = std::max(rear_margin, 0.0);
  double front_margin = ego_end_vel * 0.3 + (ego_end_vel - agent_end_vel) /
                                                (2 * std::sqrt(1.5 * 2.5));
  front_margin = std::max(front_margin, 2.0);
  double rear_margin = ego_end_vel * 0.3 + (agent_end_vel - ego_end_vel) /
                                               (2 * std::sqrt(1.5 * 2.5));
  rear_margin = std::max(rear_margin, 2.0);
  //加速度检查，遍历障碍物轨迹加速度数值，超过阈值则变道不合理
  const double max_agent_deacceleration = 2.0;
  bool is_agent_deacceleration_safe = true;
  // 预测轨迹置信时间为 3.0s, 应对前车慢速时的安全检查
  double confidence_time = 3.0;
  int confidence_time_index = std::min(static_cast<int>(confidence_time / 0.2),
                                       static_cast<int>(agent_traj.size()));
  for (int i = 0; i < confidence_time_index; i++) {
    if (agent_traj[i].a < -max_agent_deacceleration) {
      is_agent_deacceleration_safe = false;
      break;
    }
  }
  // 自车优化轨迹置信时间 4.0s, 应对前车慢速时的安全检查
  confidence_time = 4.0;
  double max_ego_jerk = 1.5;
  bool is_ego_jerk_safe = true;
  confidence_time_index = std::min(static_cast<int>(confidence_time / 0.2),
                                   static_cast<int>(ego_trajs_future_.size()));
  for (int i = 0; i < confidence_time_index; i++) {
    if (ego_trajs_future_[i].jerk < -max_ego_jerk) {
      is_ego_jerk_safe = false;
      break;
    }
  }

  JSON_DEBUG_VALUE("ego_front_edge", ego_front_edge);
  JSON_DEBUG_VALUE("ego_rear_edge", ego_rear_edge);
  bool lc_safety = false;
  if (is_front_agent) {
    lc_safety =
        ego_front_edge < agent_rear_edge - front_margin && is_ego_jerk_safe;
    JSON_DEBUG_VALUE("front_agent_front_edge", agent_front_edge);
    JSON_DEBUG_VALUE("front_agent_rear_edge", agent_rear_edge);
  } else {
    lc_safety = agent_front_edge < ego_rear_edge - rear_margin &&
                is_agent_deacceleration_safe;
    JSON_DEBUG_VALUE("rear_agent_front_edge", agent_front_edge);
    JSON_DEBUG_VALUE("rear_agent_rear_edge", agent_rear_edge);
  }

  return lc_safety;
}
bool LaneChangeStateMachineManager::
    CheckIfSafetyForPredictionTrajs(  // front target node
        const TrajectoryPoints& agent_traj,
        const planning_data::DynamicAgentNode* agent_node, bool is_large_car,
        const bool is_front_agent) {
  // 车辆参数
  const auto& vehicle_param =
      VehicleConfigurationContext::Instance()->get_vehicle_param();
  const double kEgoWidth = vehicle_param.width;
  const double kEgoLength = vehicle_param.length;
  const double kEgoFrontEdgeToRearAxleDistance =
      vehicle_param.front_edge_to_rear_axle;
  const double kEgoBackEdgeToRearAxleDistance =
      vehicle_param.rear_edge_to_rear_axle;
  // 障碍物参数
  const double agent_length = agent_node->node_length();
  const double agent_width = agent_node->node_width();

  const double prediction_default_rear_dis = 50.0;
  const double prediction_default_front_dis = 150.0;
  const double rear_dis_err = 5.0;
  const double front_dis_err = 145.0;
  // 障碍物车辆在-50至150m外，没有预测轨迹
  if (agent_traj.empty()) {
    if (agent_node->node_s() < rear_dis_err) {
      // 表示障碍物在自车后方50m外，此时需要判断安全性
      if (transition_info_.lane_change_status == kLaneChangeExecution) {
        return CalculateSideAreaIsSafetyExecution(agent_node);
      } else {
        return CalculateSideGapFeasible(agent_node);
      }
    } else if (agent_node->node_s() > front_dis_err) {
      // 表示障碍物在自车前方150m外，距离够远，不需要判断安全性
      return true;
    } else {
      return false;
    }
  }

  // 增加在接近ramp或者split时更激进的变道判断
  const auto& cur_lane = session_->environmental_model()
                             .get_virtual_lane_manager()
                             ->get_current_lane();
  if (!cur_lane) {
    return false;
  }

  const auto& route_info_output =
      session_->environmental_model().get_route_info()->get_route_info_output();

  // 根据当前速度计算触发激进导航变道的阈值
  const double kAggresiveMLCThreshold = ego_trajs_future_[0].v * 15;

  const bool is_mlc = transition_info_.lane_change_type == MAP_REQUEST;
  const bool is_aggresive_mlc =
      is_mlc && ((cur_lane->is_nearing_ramp_mlc_task() &&
                  route_info_output.dis_to_ramp < kAggresiveMLCThreshold) ||
                 (cur_lane->is_nearing_split_mlc_task() &&
                  route_info_output.distance_to_first_road_split <
                      kAggresiveMLCThreshold));

  double solid_safety_dist = 0;
  // 后方的逆行车，直接返回true
  if (!is_front_agent && !agent_traj.empty()) {
    double s_start = agent_traj.front().s;
    double s_end = agent_traj.back().s;
    if (s_end < s_start) {
      return true;  // 防止向后行驶的车辆与自车回车时因为 buff 误触发返回
    }
  }

    double max_lat_buff = 4.0;
    double lat_buff = 3.5;
  // 确认初始横向buff 与横向速度相关
    const int target_lane_virtual_id = lc_req_mgr_->target_lane_virtual_id();
    const auto reference_path_manager =
        session_->environmental_model().get_reference_path_manager();
    const auto target_reference_path =
        reference_path_manager->get_reference_path_by_lane(
            target_lane_virtual_id);

    if (target_reference_path != nullptr) {
      const auto& obstacles_map = target_reference_path->get_obstacles_map();
      auto it = obstacles_map.find(agent_node->node_agent_id());
      if(it != obstacles_map.end()) {
        const auto& rear_obs = it->second;
        double obs_lat_vel = rear_obs->frenet_velocity_lateral(); //负：靠近目标车道
        double lat_buff_scalor_by_vel = 1.1;
        double extra_lat_buff = obs_lat_vel < 0 ? - obs_lat_vel * lat_buff_scalor_by_vel : 0.0;
        max_lat_buff += extra_lat_buff;
      }
    }
    max_lat_buff = std::min(std::max(max_lat_buff, 3.5), 4.6); // 横向最大4.6m[参靠自车居中，到目标车道远边界]，最小3.5m
  // int iter_count = std::min(agent_traj.size(), ego_trajs_future_.size());
  // iter_count = std::max(16, lc_safety_check_num_);
  int iter_count = 16;
  iter_count = std::min(iter_count, static_cast<int>(ego_trajs_future_.size()));
  double distance = 100.0;
  double box_longitudinal_buff = 0.0;
  double box_ttc = lc_safety_check_time_;  // time 的计算已经有非零保护了
  // 根据与后车差速 调整起始ttc

  // std::array<double, 7> xpv{0.0, 5.0,  10., 20.,
  //                           25., 30.0, 40.};  // 后车 - 自车速度 kph
  // std::array<double, 7> fpv{3.0, 4.0, 5.0, 7.0, 8.0, 9.5, 10.};  // 起始ttc
  const auto& diff_speed_init_ttc_map =
      lc_safety_check_config_.diff_speed_init_ttc_map;
  const auto& xpv = diff_speed_init_ttc_map.diff_kph_table;
  const auto& ttc_table = diff_speed_init_ttc_map.ttc_table;
  const auto& aggressive_ttc_table = diff_speed_init_ttc_map.aggressive_ttc_table;
  double delta_kph =
      3.6 * std::max(0., agent_traj[0].v - ego_trajs_future_[0].v);
  double init_diff_speed_ttc = is_aggressive_scence_
                                ? interp(delta_kph, xpv, aggressive_ttc_table)
                                : interp(delta_kph, xpv, ttc_table);
  // init_diff_speed_ttc -> 递减
  // double max_box_ttc_rear =
  // CalculateLCSafetyCheckTimeByDiffSpeed(init_diff_speed_ttc);
  double check_time_ratio = CalculateCheckTimeRatio();
  double max_box_ttc_rear = init_diff_speed_ttc * check_time_ratio;
  max_box_ttc_rear = std::min(max_box_ttc_rear, 10.0);
  max_box_ttc_rear = std::max(max_box_ttc_rear, 0.1);
  // 自车压线目标车道情况
  const double ego_press_line_ratio =
      lc_request_.CalculatePressLineRatioByTwoLanes(
          lc_lane_mgr_->origin_lane_virtual_id(),
          lc_lane_mgr_->target_lane_virtual_id(),
          transition_info_.lane_change_direction);
  // 如果压线了， 重新映射起始ttc
  std::array<double, 3> x_press_ratio{0.0, 0.2};  // 后车 - 自车速度 kph
  std::array<double, 3> f_press_ttc{max_box_ttc_rear, 0.0};  // 起始ttc
  double press_ttc =
      interp(ego_press_line_ratio, x_press_ratio, f_press_ttc);  // 距离/ 时间
  bool is_press_boundary = ego_press_line_ratio > 0.01;
  if (is_press_boundary) {
    max_box_ttc_rear = press_ttc;  //不压线时 不影响返回能力
  }
  double std_beyond_lane_time = std::max(
      0.0, std::min(2.0, lc_safety_check_time_ - 1.0));  // 实际初始在2.5 左右
  // 根据目标车速度 调整速度阈值
  const auto& rear_vehicle_speed_min_space_map =
      lc_safety_check_config_.rear_vehicle_speed_min_space_map;
  const auto& xp = rear_vehicle_speed_min_space_map.rear_speed_kph_table;
  const auto& fp = rear_vehicle_speed_min_space_map.min_space_table;
  bool is_executing =
      transition_info_.lane_change_status == kLaneChangeExecution;
  // bool is_deceleration_check =
  //     is_executing && !is_front_agent && agent_traj.size() >= 3 &&
  //     !agent_node->is_static_type() &&
  //     (last_target_rear_agent_id_ == agent_node->node_agent_id() ||
  //      ego_press_line_ratio > 0.01);
  //记录当前时刻压线率及安全检查数据
  std::vector<double> box_longitudinal_buff_vec; // 公用： 安全buff
  std::vector<double> distance_vec; // 共用膨胀后 box 之间的距离(剩余距离)
  std::vector<double> box_ttc_vec; // 插值ttc
  std::vector<double> front_agent_vel_vec;      // 前速度
  std::vector<double> rear_agent_vel_vec;      // 后速度
  std::vector<double> ego_vel_vec;        // 自车速度
  std::vector<double> rear_actual_gap_vec;  // 后车到自车的实际纵向距离
  std::vector<double> front_actual_gap_vec;  // 前车到自车的实际纵向距离
  box_longitudinal_buff_vec.reserve(iter_count);
  box_ttc_vec.reserve(iter_count);
  distance_vec.reserve(iter_count);
  front_agent_vel_vec.reserve(iter_count);
  rear_agent_vel_vec.reserve(iter_count);
  ego_vel_vec.reserve(iter_count);
  rear_actual_gap_vec.reserve(iter_count);
  front_actual_gap_vec.reserve(iter_count);
  bool is_front_reverse = false;
  if (agent_traj.back().s < agent_traj.front().s && agent_node->node_speed() > 2.0) {
    is_front_reverse = true;
  }
  //计算当前时刻与前车的ttc
  bool front_risk = false;
  double ego_v = ego_trajs_future_[0].v;
  double front_v = agent_traj[0].v;
  double front_gap = agent_traj[0].s - ego_trajs_future_[0].s
                    - 0.5 * agent_node->node_length() - kEgoFrontEdgeToRearAxleDistance;
  // gap小于0 则不安全，gap大于0，有差速，ttc小于2 也不安全
  if(front_gap < 1.0){
    front_risk = true;
  }else{
    //自车相对前车慢速返回安全，自车相对前车快速则计算ttc
    double rel_vel0 = ego_v - front_v;
    if(rel_vel0 < 0.0){//前车快速 无ttc
      front_risk = false;
    }else{
      double front_ttc = front_gap / (rel_vel0 + 1e-6);
      if(front_ttc < 2.0){
        front_risk = true;
      }else{
        front_risk = false;
      }
    }
  }
    double two_car_length = 0;
    // 目前障碍物的前、后边到后轴的距离等于车身长一半
    if (is_front_agent) {
      two_car_length = kEgoFrontEdgeToRearAxleDistance + 0.5 * agent_length;
    } else {
      two_car_length = kEgoBackEdgeToRearAxleDistance + 0.5 * agent_length;
    }
    bool is_checking_rear_overtaking = false;
    if(target_lane_rear_node_){
      is_checking_rear_overtaking = target_lane_rear_node_->node_agent_id() == agent_node->node_agent_id()
                                 && rear_agent_overtaking_ && !is_front_agent;
    }
  for (int i = 0; i < iter_count; i++) {
    // //执行后缩短预测轨迹检查
    // if(is_executing && i > 10){
    //   break;
    // }
    double beyond_lane_time = std_beyond_lane_time - i * 0.2;
    beyond_lane_time = std::max(beyond_lane_time, 0.0);
    if (is_front_agent) {
      double rel_vel = agent_traj[i].v - ego_trajs_future_[i].v;
      //保护大差速
      double ego_brake = 2.0;
      double ego_faster_buff = (- rel_vel * ego_trajs_future_[i].v) / (2.0 * ego_brake);
      //保护高速恐慌感
      box_longitudinal_buff = std::max(ego_faster_buff,
                ego_trajs_future_[i].v * lc_safety_check_config_.faster_rear_delay_time);
      if (is_large_car) {
        box_longitudinal_buff += 5.0;  // 大车额外增加5m基础距离
      }
      if (ego_press_line_ratio > 0.01 && is_side_clear_  && is_executing && !front_risk) {//对侧方车保持返回能力
        break;  // 已经压线以后，不再检查前车安全性，压线后再变道返回对前车是危险的。
      }
      if(ego_press_line_ratio > 0.5 && is_side_clear_){
        break;  // 已经压线过多，侧方无车不返回
      }
      //未来衰减
      const double ttc_decay_factor = lc_safety_check_config_.ttc_decay_factor;
      const double ttc_decay = std::pow(ttc_decay_factor, i);
      box_longitudinal_buff = box_longitudinal_buff * ttc_decay;
      //状态折扣
      box_longitudinal_buff = is_executing ?
                            box_longitudinal_buff * lc_safety_check_config_.exe_ttc_ratio * (1.0 - ego_press_line_ratio)
                            : box_longitudinal_buff * (1.0 - ego_press_line_ratio);
      if(is_front_reverse){
        double reverse_check_time = 4.0; //默认基准变道时间
        // double reverse_check_time = lc_safety_check_config_.diff_speed_init_ttc_map.ttc_table.front();
        box_longitudinal_buff = reverse_check_time * (agent_traj[i].v  + ego_trajs_future_[i].v);
      }
    } else {
      // 1. 运动学距离：后车反应时间 + 差速刹停距离
      // double rear_relative_decel =
      //     is_aggressive_scence_
      //         ? lc_safety_check_config_.rear_comfort_decel +
      //               lc_safety_check_config_.aggressive_decel_part
      //         : lc_safety_check_config_.rear_comfort_decel;
      // if (is_large_car) {
      //   rear_relative_decel = 0.3;
      // }
      double rear_acc = agent_traj[i].a;
      double rel_vel = std::max(0.0, agent_traj[i].v - ego_trajs_future_[i].v);
      double dis_diff_vel = 0.0;
      double predict_t = lc_safety_check_time_ + 0.5;
      if (rear_acc > 0.3) {
          // rear accelerating
          dis_diff_vel = rel_vel * predict_t + 0.5 * rear_acc * predict_t * predict_t;
      } else if (rear_acc < -0.3) {
        // rear braking
        dis_diff_vel = rel_vel * rel_vel / (2.0 * (- rear_acc));
      } else {
        // rear constant speed
        dis_diff_vel = rel_vel * predict_t;
      }
      dis_diff_vel += lc_safety_check_config_.faster_rear_delay_time * agent_traj[i].v;
      if (is_large_car) {
        dis_diff_vel += 5.0;
      }
      // 2 主观感受 TTC 距离：由预测差速映射 ttc，并与剩余检查时间耦合
      const double diffspeed_kph = 3.6 * rel_vel;
      double pred_ttc = is_aggressive_scence_
                            ? interp(diffspeed_kph, xpv, aggressive_ttc_table)
                            : interp(diffspeed_kph, xpv, ttc_table);
      double dist_ttc_interp = (rel_vel > 0.0) ? rel_vel * pred_ttc : 0.0;
      double safety_buff = std::max(dist_ttc_interp, dis_diff_vel);
      box_ttc_vec.push_back(pred_ttc);
      // 3) 时间衰减：对合并后的 buff 沿预测时域衰减
      const double ttc_decay_factor = (is_checking_rear_overtaking && !is_aggressive_scence_)?
                                      0.99: lc_safety_check_config_.ttc_decay_factor;
      const double ttc_decay = std::pow(ttc_decay_factor, i);
      box_longitudinal_buff = safety_buff * ttc_decay;
      // 4) 状态折扣：执行态下叠乘 exe 折扣[后车不超车]和 (1 - 压线率)
      const double press_ratio = std::clamp(ego_press_line_ratio, 0.0, 1.0);
      const double exe_ratio = is_checking_rear_overtaking ? 0.9 : std::clamp(lc_safety_check_config_.exe_ttc_ratio, 0.3, 1.0);
      box_longitudinal_buff = is_executing ?
                            box_longitudinal_buff * exe_ratio * (1.0 - press_ratio)
                            : box_longitudinal_buff * (1.0 - press_ratio);
      // 最小值
      double agent_kph = agent_traj[i].v * 3.6;
      double min_space = interp(agent_kph, xp, fp);
      // 防止在最小值跳变
      min_space = is_executing ? min_space * 1.0 : min_space * 1.25;
      box_longitudinal_buff = std::max(box_longitudinal_buff, min_space);
      // 两类特殊pass
      if (ego_press_line_ratio > 0.3 && is_side_clear_ && is_executing) {
        break;  // 已经压线过多，侧方无车不返回
      }
      if(ego_press_line_ratio > 0.5 && is_side_clear_){
        break;  // 已经压线过多，侧方无车不返回
      }
    }

    const double focus_v =
        is_front_agent ? ego_trajs_future_[i].v : agent_traj[i].v;
    // 考虑目前自车的执行器响应时间为0.5s，在换道时纵向的稳态跟车距离为1*v；
    // 为了提高变道变道成功率，纵向又不至于减速太猛，因此设自车前方的最小安全距离为0.7*v。
    // 在距离匝道或者split距离小于200m时，变道优先级较高，那么最小安全距离设为0.5*v。
    double safety_dist_factor = is_aggresive_mlc ? 0.3 : 0.4;
    double safety_dist = safety_dist_factor * focus_v;

    // 根据当前的测试数据，后方也以0.7作为系数，导致变道有点保守，因此后方的距离减小至原来的0.5倍。
    if (!is_front_agent) {
      safety_dist = 0.7 * safety_dist;
    }

    // 如果两车速度差大于2m/s，那么安全距离可以降低至当前的0.8倍
    double rel_v = ego_trajs_future_[i].v - agent_traj[i].v;
    bool is_large_v_diff = is_front_agent ? rel_v < -2 : rel_v > 2;
    safety_dist =
        (is_large_v_diff && !is_large_car) ? safety_dist * 0.7 : safety_dist;
    // 未来几秒的减小体现在 -i * 0.2 中
    // 考虑与后车的未来2s内距离足够的情况下，未来1-4s距离不够的情况，为了提升变道成功率，可以在1-4s时的安全距离逐渐减小
    if (i > 5) {
      // const double agent_last_rel_dis =
      //     agent_traj[iter_count - 1].s - ego_trajs_future_[0].s;
      const double buffer = 1.0;
      // const double agent_last_dis = agent_last_rel_dis + 0.5 * agent_length +
      //                               kEgoBackEdgeToRearAxleDistance + buffer;
      std::array<double, 2> dynamic_safety_dis{solid_safety_dist, buffer};
      double iter_count_temp = iter_count;
      std::array<double, 2> index{0, iter_count_temp - 1 - 5};
      safety_dist = interp(i - 5, index, dynamic_safety_dis);
    }
    // 在安全性判断阶段，把1s处的安全距离记录下来
    if (i == 5) {
      solid_safety_dist = safety_dist;
    }

    // 保存前、后障碍物所需要的安全距离
    if (is_front_agent) {
      lc_front_obj_need_dis_vec_.push_back(ego_trajs_future_[i].s -
                                           ego_trajs_future_[0].s +
                                           box_longitudinal_buff);
    } else {
      const double rel_dis = agent_traj[i].s - ego_trajs_future_[0].s;
      const double dis =
          rel_dis + 0.5 * agent_length + kEgoBackEdgeToRearAxleDistance;
      lc_rear_obj_need_dis_vec_.push_back(dis + box_longitudinal_buff);
    }

    // 安全性判断 ： 原版安全检查
    //  if (std::abs(agent_traj[i].s - ego_trajs_future_[i].s) - two_car_length
    //  <
    //      safety_dist) {
    //    ILOG_DEBUG << "not safety !!!" << std::endl;
    //    return false;
    //  }
    //  横向新安全检查：
    if (i < 8) {
      lat_buff = max_lat_buff;
    } else if (i < 16) {
      lat_buff = max_lat_buff * 0.6;
    } else if (i < 24) {
      lat_buff = max_lat_buff * 0.3;
    } else {
      lat_buff = 0;
    }

    // 自车box
    double ego_theta = ego_trajs_future_[i].heading_angle;
    Vec2d rac(ego_trajs_future_[i].x, ego_trajs_future_[i].y);
    Vec2d ego_center = rac + Vec2d::CreateUnitVec2d(ego_theta) *
                                 kEgoBackEdgeToRearAxleDistance;
    // double ego_lateral_buff = 2.0;// 根据agent 类型选择 或者根据agent
    // 尺寸选择  根据不同时刻选择
    planning_math::Box2d ego_box(ego_center, ego_theta,
                                 kEgoLength + box_longitudinal_buff,
                                 kEgoWidth + lat_buff);
    // agent box
    Vec2d agent_rac(agent_traj[i].x, agent_traj[i].y);
    double agent_theta = agent_traj[i].heading_angle;
    // agent  risk buff
    double agent_lateral_buff =
        1.5;  // 根据agent 类型选择 或者根据agent 尺寸选择  根据不同时刻选择
    agent_lateral_buff = agent_node->is_VRU_type() ? 3.5 : 1.5;
    double agent_longitudinal_buff = 2.0 * safety_dist;
    planning_math::Box2d agent_box(agent_rac, agent_theta,
                                   agent_length + box_longitudinal_buff,
                                   agent_width + lat_buff);
    double distance_i = ego_box.DistanceTo(agent_box);

    box_longitudinal_buff_vec.push_back(box_longitudinal_buff);
    distance_vec.push_back(distance_i);
    ego_vel_vec.push_back(ego_trajs_future_[i].v);
    if (!is_front_agent) {
      rear_agent_vel_vec.push_back(agent_traj[i].v);
      rear_actual_gap_vec.push_back(ego_trajs_future_[i].s - agent_traj[i].s -
                                    two_car_length);
    } else {
      front_agent_vel_vec.push_back(agent_traj[i].v);
      front_actual_gap_vec.push_back(agent_traj[i].s - ego_trajs_future_[i].s -
                                     two_car_length);
    }
    distance = std::min(distance, distance_i);
    // 记录box
    if (distance < 0.01) {
      ILOG_DEBUG << i << "box-box not safety !!!";
      const auto& agent_corners = agent_box.GetAllCorners();
      for (const auto& corner_point : agent_corners) {
        agent_box_corners_x_.push_back(corner_point.x());
        agent_box_corners_y_.push_back(corner_point.y());
      }
      const auto& ego_corners = ego_box.GetAllCorners();
      for (const auto& corner_point : ego_corners) {
        ego_box_corners_x_.push_back(corner_point.x());
        ego_box_corners_y_.push_back(corner_point.y());
      }
    }
  }
  // 循环结束后统一输出 JSON debug 数据
  JSON_DEBUG_VECTOR("ego_vel_vec", ego_vel_vec, 2);
  JSON_DEBUG_VALUE("rear_agent_overtaking", rear_agent_overtaking_);
  if (!is_front_agent) {
    JSON_DEBUG_VECTOR("rear_box_longitudinal_buff_vec", box_longitudinal_buff_vec, 2);
    JSON_DEBUG_VECTOR("rear_box_ttc_vec", box_ttc_vec, 2);
    JSON_DEBUG_VECTOR("rear_distance_vec", distance_vec, 2);
    JSON_DEBUG_VECTOR("rear_agent_vel_vec", rear_agent_vel_vec, 2);
    JSON_DEBUG_VECTOR("rear_actual_gap_vec", rear_actual_gap_vec, 2);
  } else {
    JSON_DEBUG_VECTOR("front_box_longitudinal_buff_vec", box_longitudinal_buff_vec, 2);
    JSON_DEBUG_VECTOR("front_distance_vec", distance_vec, 2);
    JSON_DEBUG_VECTOR("front_agent_vel_vec", front_agent_vel_vec, 2);
    JSON_DEBUG_VECTOR("front_actual_gap_vec", front_actual_gap_vec, 2);
  }
  // 利用整个 distance_vec 的碰撞分布模式判断安全性
  // distance_vec[j] == 0 表示扩展 box 重叠（DistanceTo 最小返回 0）
  // 根据 0 值在时间轴上的分布分三类处理：
  //   尾部集中（先安全后碰撞）→ 未来恶化，直接不安全
  //   头部集中（先碰撞后安全）→ 当前裕度不足但未来改善，需 1/6 为 0 才不安全
  //   其他（散布/全程）→ 需 1/4 为 0 才不安全
  const int vec_size = static_cast<int>(distance_vec.size());
  if (vec_size > 0) {
    int zero_count = 0;
    int first_zero = -1;
    int last_zero = -1;
    for (int j = 0; j < vec_size; ++j) {
      if (distance_vec[j] < 0.01) {
        ++zero_count;
        if (first_zero < 0) first_zero = j;
        last_zero = j;
      }
    }
    if (zero_count > 0) {
      bool tail_heavy =
          last_zero == vec_size - 1 && distance_vec[0] > 0.01;
      bool head_heavy =
          first_zero == 0 && distance_vec[vec_size - 1] > 0.01;

      if (tail_heavy) {
        // 前安全后碰撞：未来持续恶化，直接判定不安全
        return false;
      } else if (head_heavy) {
        // 前碰撞后安全：当前空间是为未来预留的裕度，未来在改善
        // 需要六分之一及以上步为 0 才判定不安全
        if (zero_count * 6 >= vec_size) {
          return false;
        }
      } else {
        // 散布或全程碰撞：需要四分之一及以上步为 0 才判定不安全
        if (zero_count * 4 >= vec_size) {
          return false;
        }
      }
    }
  }
  if (!is_front_agent) {
    // 近距离尾随的后车，发起变道安全感需要距离持续拉开
    bool is_rear_close = false;
    double rear_close_distance_threshold =
        lc_safety_check_config_.rear_close_distance_threshold;  // exe 后放大
    double rear_close_speed_diff_threshold =
        lc_safety_check_config_.rear_close_speed_diff_threshold;
    if (is_executing) {
      rear_close_distance_threshold =
          rear_close_distance_threshold *
          lc_safety_check_config_.exe_rear_distance_ratio;
      ;
      rear_close_speed_diff_threshold =
          rear_close_speed_diff_threshold *
          lc_safety_check_config_.exe_rear_speed_ratio;
    }
    double rear_distance = ego_trajs_future_[0].s - agent_node->node_s() -
                           agent_node->node_length() * 0.5 -
                           kEgoBackEdgeToRearAxleDistance;
    is_rear_close = rear_distance < rear_close_distance_threshold;
    if (is_rear_close && ego_press_line_ratio < 0.001) {
      // 如果后方安全距离没有充分拉大的趋势，且后车速度大于自车速度减去阈值，不允许变道
      if (agent_traj[0].v >
          ego_trajs_future_[0].v + rear_close_speed_diff_threshold) {
        return false;
      }
    }
  }
  return true;
}

bool LaneChangeStateMachineManager::IsFilterAgent(
    const planning_data::DynamicAgentNode* agent_node,
    const std::shared_ptr<planning_math::KDPath> target_lane_coor,
    TrajectoryPoints* agent_prediction_trajs, const bool is_ego_lane_agent,
    const bool is_front_agent) {
  if (IsFilterStaticAgentLC(*agent_node)) {
    return true;
  }
  const auto& virtual_lane_manager =
      session_->environmental_model().get_virtual_lane_manager();
  const int target_lane_virtual_id = lc_req_mgr_->target_lane_virtual_id();
  const auto& tar_lane =
      virtual_lane_manager->get_lane_with_virtual_id(target_lane_virtual_id);
  if (tar_lane == nullptr) {
    return false;
  }

  const double ego_v =
      session_->environmental_model().get_ego_state_manager()->ego_v();
  const auto& trajectory_optimized = agent_node->node_trajectory_optimized();
  const auto& agent_trajs = agent_node->node_trajectories_used_by_st_graph();
  if (agent_trajs.size() < 1) {
    return false;
  }
  const auto& agent_traj_st = agent_trajs[0];
  // 可切换的轨迹：使用引用代替指针
  const trajectory::Trajectory& agent_traj =
      (joint_decision_success_ && trajectory_optimized.size() > 1)
          ? trajectory_optimized
          : agent_traj_st;

  for (int i = 0; i < agent_traj.size(); i++) {
    const auto& agent_traj_point = agent_traj[i];
    // if (i > lc_safety_check_num_) {
    //   break;
    // }
    TrajectoryPoint agent_prediction_traj;
    agent_prediction_traj.x = agent_traj_point.x();
    agent_prediction_traj.y = agent_traj_point.y();
    // Point2D cart_point(agent_traj_point.x(), agent_traj_point.y());
    // Point2D frenet_point;
    // if (!target_lane_coor->XYToSL(cart_point, frenet_point)) {
    //   continue;
    // }
    double s = 0.0;
    double l = 0.0;
    if (!target_lane_coor->XYToSL(agent_traj_point.x(), agent_traj_point.y(),
                                  &s, &l)) {
      continue;  //
    }
    Point2D frenet_point(s, l);
    agent_prediction_traj.s = frenet_point.x;
    agent_prediction_traj.l = frenet_point.y;
    agent_prediction_traj.t = agent_traj_point.absolute_time();
    agent_prediction_traj.v = agent_traj_point.vel();
    agent_prediction_traj.a = agent_traj_point.acc();
    agent_prediction_traj.jerk = agent_traj_point.jerk();
    agent_prediction_traj.heading_angle =
        agent_traj_point.theta();  // add store heading angle

    agent_prediction_trajs->emplace_back(agent_prediction_traj);
  }

  if (agent_prediction_trajs->empty()) {
    return false;
  }

  // 对于横向位置只有半个车宽在自车道内的，根据预测信息判断是否会向本车道行驶
  // 如果该障碍物只有半个车宽在自车道内，且不向本车道行驶则过滤该障碍物
  // 还需要考虑纵向的情况

  const double default_lon_fix_time_dis = 1.0;

  const double real_time_dis = (lc_safety_check_time_ / kEgoReachBoundaryTime) *
                               default_lon_fix_time_dis;

  const double half_agent_width = agent_node->node_width() * 0.5;
  const double half_agent_length = agent_node->node_length() * 0.5;

  const double relative_dis =
      is_front_agent ? agent_node->node_back_edge_to_ego_front_edge_distance()
                     : agent_node->node_front_edge_to_ego_back_edge_distance();

  const double relative_v = is_front_agent
                                ? ego_v - agent_prediction_trajs->at(0).v
                                : agent_prediction_trajs->at(0).v - ego_v;
  // 后方车
  if (!is_front_agent) {
    // 针对提取的后方车，尝试用横向速度判断是否侵入目标车道
    const int target_lane_virtual_id = lc_req_mgr_->target_lane_virtual_id();
    const auto reference_path_manager =
        session_->environmental_model().get_reference_path_manager();
    const auto target_reference_path =
        reference_path_manager->get_reference_path_by_lane(
            target_lane_virtual_id);
    if (target_reference_path == nullptr) {
      return false;
    }
    const auto& ego_sl_bd = target_reference_path->get_ego_frenet_boundary();
    const auto& ego_sl_state = target_reference_path->get_frenet_ego_state();
    const auto& vehicle_param =
        VehicleConfigurationContext::Instance()->get_vehicle_param();
    double half_width = vehicle_param.max_width * 0.5;
    const auto& obstacles_map = target_reference_path->get_obstacles_map();
    auto it = obstacles_map.find(agent_node->node_agent_id());
    const auto& dynamic_world =
        session_->environmental_model().get_dynamic_world();
    const auto& agent_mgr = dynamic_world->agent_manager();
    const auto agent = agent_mgr->GetAgent(agent_node->node_agent_id());
    // if (it != obstacles_map.end()) {
    if (agent != nullptr && it != obstacles_map.end()) {
      const auto& rear_obs = it->second;
      double single_risk_buff = 0.4;
      double concerned_s_start = ego_sl_bd.s_start;
      double concerned_s_end = ego_sl_bd.s_end;
      std::pair<double, double> ego_longi{concerned_s_start, concerned_s_end};
      double ego_vel = ego_sl_state.velocity_s();
      std::pair<double, double> center_lat{-half_width - single_risk_buff,
                                           half_width + single_risk_buff};
      double ego_lat_vel = ego_sl_state.velocity_l();

      // const auto& obstacle_sl = rear_obs->frenet_obstacle_boundary();
      const auto& obstacle_sl =
          GetSLboundaryFromAgent(target_reference_path, agent->box());
      std::pair<double, double> obs_lat{obstacle_sl.l_start, obstacle_sl.l_end};
      double obs_lat_vel = rear_obs->frenet_velocity_l();
      bool is_rear_cut_in =
          IfFrenetCollision(center_lat, 0.0, obs_lat, obs_lat_vel, 4.0, 0.5);
      return !is_rear_cut_in;
    }
  }

  if (is_ego_lane_agent) {
    const auto& cur_lane = virtual_lane_manager->get_current_lane();
    const auto& reference_path_manager =
        session_->environmental_model().get_reference_path_manager();
    const auto& cur_ref_path_manager =
        reference_path_manager->get_reference_path_by_current_lane();

    if (cur_ref_path_manager == nullptr) {
      return false;
    }
    const auto& cur_lane_coor = cur_ref_path_manager->get_frenet_coord();
    if (cur_lane_coor == nullptr) {
      return false;
    }

    // Point2D obj_first_point(agent_prediction_trajs->at(0).x,
    //                         agent_prediction_trajs->at(0).y);
    // Point2D obj_first_frenet_point;
    // Point2D obj_back_point(agent_prediction_trajs->back().x,
    //                        agent_prediction_trajs->back().y);
    // Point2D obj_back_frenet_point;

    // if (!cur_lane_coor->XYToSL(obj_first_point, obj_first_frenet_point)) {
    //   return false;
    // }
    // if (!cur_lane_coor->XYToSL(obj_back_point, obj_back_frenet_point)) {
    //   return false;
    // }
    // first point
    double first_s = 0.0;
    double first_l = 0.0;
    if (!cur_lane_coor->XYToSL(agent_prediction_trajs->at(0).x,
                               agent_prediction_trajs->at(0).y, &first_s,
                               &first_l)) {
      return false;  // 更换xytosl
    }
    Point2D obj_first_frenet_point(first_s, first_l);
    // back point
    double back_s = 0.0;
    double back_l = 0.0;
    if (!cur_lane_coor->XYToSL(agent_prediction_trajs->back().x,
                               agent_prediction_trajs->back().y, &back_s,
                               &back_l)) {
      return false;  // 更换xytosl
    }
    Point2D obj_back_frenet_point(back_s, back_l);

    const double cur_lane_width =
        cur_lane->width_by_s(obj_first_frenet_point.x);
    const double lat_consider_dis = cur_lane_width * 0.5;

    bool is_lat_safe = std::abs(obj_first_frenet_point.y) > lat_consider_dis &&
                       std::abs(obj_back_frenet_point.y) > lat_consider_dis &&
                       obj_first_frenet_point.y * obj_back_frenet_point.y > 0;

    if (is_lat_safe) {
      return true;
    }
  } else {
    const double tar_lane_width =
        tar_lane->width_by_s(agent_prediction_trajs->at(0).s);
    const double lat_consider_dis = tar_lane_width * 0.5;
    // 这里实际上是 只用了
    // 预测轨迹的起点和终点，但是考虑到预测轨迹的特性，并不合适
    bool is_lat_safe =
        std::abs(agent_prediction_trajs->at(0).l) > lat_consider_dis &&
        std::abs(agent_prediction_trajs->back().l) > lat_consider_dis &&
        agent_prediction_trajs->at(0).l * agent_prediction_trajs->back().l > 0;

    const double lon_safety_dis =
        is_front_agent ? real_time_dis * ego_v
                       : agent_prediction_trajs->at(0).v * real_time_dis;

    bool is_lon_safe = lon_safety_dis < relative_dis;

    if (is_lat_safe && is_lon_safe) {
      return true;
    }
  }
  return false;
}
void LaneChangeStateMachineManager::BuildAgentPredictionTrajsInTargetLane(
  const planning_data::DynamicAgentNode* agent_node,
  const std::shared_ptr<planning_math::KDPath> target_lane_coor,
  const bool is_front_agent, TrajectoryPoints* agent_prediction_trajs,
  const bool is_side_obs) {
agent_prediction_trajs->clear();
if (agent_node == nullptr || target_lane_coor == nullptr) {
  return;
}
const auto& virtual_lane_manager =
    session_->environmental_model().get_virtual_lane_manager();
const int target_lane_virtual_id = lc_req_mgr_->target_lane_virtual_id();
const auto& tar_lane =
    virtual_lane_manager->get_lane_with_virtual_id(target_lane_virtual_id);
if (tar_lane == nullptr) {
  return;
}

const auto& trajectory_optimized = agent_node->node_trajectory_optimized();
const auto& agent_trajs = agent_node->node_trajectories_used_by_st_graph();
if (agent_trajs.size() < 1) {
  return;
}
const auto& agent_traj_st = agent_trajs[0];
const trajectory::Trajectory& agent_traj =
    (!is_side_obs && joint_decision_success_ && trajectory_optimized.size() > 1)
        ? trajectory_optimized
        : agent_traj_st;
for (int i = 0; i < agent_traj.size(); i++) {
  const auto& agent_traj_point = agent_traj[i];
  TrajectoryPoint agent_prediction_traj;
  agent_prediction_traj.x = agent_traj_point.x();
  agent_prediction_traj.y = agent_traj_point.y();
  double s = 0.0;
  double l = 0.0;
  if (!target_lane_coor->XYToSL(agent_traj_point.x(), agent_traj_point.y(),
                                &s, &l)) {
    continue;
  }
  Point2D frenet_point(s, l);
  agent_prediction_traj.s = frenet_point.x;
  agent_prediction_traj.l = frenet_point.y;
  agent_prediction_traj.t = agent_traj_point.absolute_time();
  agent_prediction_traj.v = agent_traj_point.vel();
  agent_prediction_traj.a = agent_traj_point.acc();
  agent_prediction_traj.jerk = agent_traj_point.jerk();
  agent_prediction_traj.heading_angle = agent_traj_point.theta();
  agent_prediction_trajs->emplace_back(agent_prediction_traj);
}
}
void LaneChangeStateMachineManager::StoreObjDebugPredictionInfo(
    const planning_data::DynamicAgentNode* agent_node,
    const TrajectoryPoints* agent_prediction_trajs, const bool is_front_agent,
    const bool is_ego_lane_agent) {
  if(agent_node == nullptr || agent_prediction_trajs == nullptr) {
    return;
  }
  const double agent_length = agent_node->node_length();

  const auto& vehicle_param =
      VehicleConfigurationContext::Instance()->get_vehicle_param();
  const double kEgoWidth = vehicle_param.width;
  const double kEgoFrontEdgeToRearAxleDistance =
      vehicle_param.front_edge_to_rear_axle;
  const double kEgoBackEdgeToRearAxleDistance =
      vehicle_param.rear_edge_to_rear_axle;

  for (const auto& agent_prediction_traj : *agent_prediction_trajs) {
    double agent_prediction_rel_s = agent_prediction_traj.s;
    if (!ego_trajs_future_.empty()) {
      agent_prediction_rel_s = agent_prediction_traj.s - ego_trajs_future_[0].s;
    }

    if (is_front_agent && !is_ego_lane_agent) {
      agent_prediction_rel_s = agent_prediction_rel_s - 0.5 * agent_length -
                               kEgoFrontEdgeToRearAxleDistance;
      lc_front_objs_tar_lane_vec_.emplace_back(agent_prediction_rel_s);
    } else if (!is_front_agent) {
      agent_prediction_rel_s = agent_prediction_rel_s + 0.5 * agent_length +
                               kEgoBackEdgeToRearAxleDistance;
      lc_rear_objs_vec_.emplace_back(agent_prediction_rel_s);
    } else {
      agent_prediction_rel_s = agent_prediction_rel_s - 0.5 * agent_length -
                               kEgoFrontEdgeToRearAxleDistance;
      lc_front_objs_ego_lane_vec_.emplace_back(agent_prediction_rel_s);
    }
  }
}
SecondOrderTimeOptimalTrajectory
LaneChangeStateMachineManager::GenerateEgoMaxDecelerationCurve(
    const double ego_v, const double target_v) {
  LonState init_state;
  init_state.p = 0;
  init_state.v = ego_v;
  init_state.a = 0;
  StateLimit state_limit;
  const double delta_v = ego_v - target_v;
  std::array<double, 3> xp{2, 5, 8};
  std::array<double, 3> fp{-1, -2.0, -2.5};
  const double a_j_min = interp(delta_v, xp, fp);
  state_limit.v_end = target_v;
  state_limit.a_max = 0;
  state_limit.a_min = a_j_min;
  state_limit.j_max = 0;
  state_limit.j_min = a_j_min;
  return SecondOrderTimeOptimalTrajectory(init_state, state_limit);
}

double LaneChangeStateMachineManager::CalculateLCSafetyCheckTime() const {
  double reach_line_time = -1.0;

  const int target_lane_virtual_id = lc_req_mgr_->target_lane_virtual_id();

  const auto reference_path_manager =
      session_->environmental_model().get_reference_path_manager();
  const auto virtual_lane_manager =
      session_->environmental_model().get_virtual_lane_manager();
  const auto target_reference_path =
      reference_path_manager->get_reference_path_by_lane(
          target_lane_virtual_id);
  if (target_reference_path == nullptr) {
    return reach_line_time;
  }
  const double ego_l_target = target_reference_path->get_frenet_ego_state().l();
  const auto target_lane =
      virtual_lane_manager->get_lane_with_virtual_id(target_lane_virtual_id);

  if (target_lane == nullptr) {
    return reach_line_time;
  }

  const double width_by_target = target_lane->width();

  double ego_dis_to_line =
      std::abs(ego_l_target) - std::abs(width_by_target) / 2;

  // 为了保证安全，加一个0.1m的buffer
  ego_dis_to_line = ego_dis_to_line + 0.1;

  const double standard_half_lane_width = kStandardLaneWidth / 2.0;

  // 由于在propose阶段考虑的是未来4s的安全性
  // 在execution过程中，其实有双方的交互博弈，所以这里安全性判断的时间可以比4s短一些
  // const double lc_gap_valid_check_time_execution = kEgoReachBoundaryTime - 1;

  // double base_time =
  //     (transition_info_.lane_change_status == kLaneChangeExecution)
  //         ? lc_gap_valid_check_time_execution
  //         : kEgoReachBoundaryTime;
  double base_time = kEgoReachBoundaryTime;
  reach_line_time = (ego_dis_to_line / standard_half_lane_width) * base_time;
  // 注：自车在接近车道线的时候ego_dis_to_line的值很小，后轴中心越过车道线时ego_dis_to_line为负的，
  // 因此，需要保证自车在每个时刻对未来至少要判断一定时间是安全的才合理，
  // 考虑到驾驶员的反应时间约为0.2s和自车执行器的响应时间0.7s，目前该值取2.0s。
  double min_check_time = 2.0;
  const auto& ego_sl_state = target_reference_path->get_frenet_ego_state();
  double ego_lat_vel = ego_sl_state.velocity_l();
  if (transition_info_.lane_change_status == kLaneChangeExecution ||
      transition_info_.lane_change_status == kLaneChangeHold) {
    if (ego_lat_vel > 0.2 &&
        transition_info_.lane_change_direction == LEFT_CHANGE) {
      min_check_time = 1.0;
    } else if (ego_lat_vel < -0.2 &&
               transition_info_.lane_change_direction == RIGHT_CHANGE) {
      min_check_time = 1.0;
    } else {
      min_check_time = 1.5;
    }
  }
  reach_line_time = std::min(std::max(reach_line_time, min_check_time),
                             kEgoReachBoundaryTime);

  return reach_line_time;
}
double LaneChangeStateMachineManager::CalculateCheckTimeRatio() const {
  double check_time_ratio = 1.0;
  const int target_lane_virtual_id = lc_req_mgr_->target_lane_virtual_id();
  const auto reference_path_manager =
      session_->environmental_model().get_reference_path_manager();
  const auto virtual_lane_manager =
      session_->environmental_model().get_virtual_lane_manager();
  const auto target_reference_path =
      reference_path_manager->get_reference_path_by_lane(
          target_lane_virtual_id);
  if (target_reference_path == nullptr) {
    return check_time_ratio;
  }
  const double ego_l_target = target_reference_path->get_frenet_ego_state().l();
  const auto origin_lane = virtual_lane_manager->get_lane_with_virtual_id(
      lc_lane_mgr_->origin_lane_virtual_id());
  const auto target_lane =
      virtual_lane_manager->get_lane_with_virtual_id(target_lane_virtual_id);
  if (target_lane == nullptr || origin_lane == nullptr) {
    return check_time_ratio;
  }
  const double width_by_target = target_lane->width();
  const double half_width_by_origin = origin_lane->width() * 0.5;
  double ego_dis_to_line =
      std::abs(ego_l_target) - std::abs(width_by_target) / 2;
  check_time_ratio = ego_dis_to_line / (half_width_by_origin + 0.001);
  return check_time_ratio;
}
std::unique_ptr<Trajectory1d>
LaneChangeStateMachineManager::MakeVirtualZeroAccCurve(
    const std::array<double, 3>& init_lon_state) const {
  auto virtual_zero_acc_curve =
      std::make_unique<PiecewiseJerkAccelerationTrajectory1d>(
          init_lon_state[0], init_lon_state[1]);
  const double dt = speed_planning_config_.dt;
  virtual_zero_acc_curve->AppendSegment(init_lon_state[2], dt);

  const double zero_acc_jerk_max = speed_planning_config_.zero_acc_jerk_max;
  const double zero_acc_jerk_min = speed_planning_config_.zero_acc_jerk_min;
  const double itear_time = lc_safety_check_num_ * 0.2;
  for (double t = dt; t <= itear_time; t += dt) {
    const double acc = virtual_zero_acc_curve->Evaluate(2, t);
    const double vel = virtual_zero_acc_curve->Evaluate(1, t);

    double a_next = 0.0;
    // if init acc < 0.0, move a to zero with jerk max
    // if init acc >0.0,move a to zero with jerk min
    if (init_lon_state[2] < 0.0) {
      a_next = acc + dt * zero_acc_jerk_max;
    } else {
      a_next = acc + dt * zero_acc_jerk_min;
    }

    if (init_lon_state[2] * acc <= 0.0) {
      a_next = 0.0;
    }

    if (vel <= 0.0) {
      a_next = 0.0;  //??
    }
    virtual_zero_acc_curve->AppendSegment(a_next, dt);
  }
  return virtual_zero_acc_curve;
}

bool LaneChangeStateMachineManager::IsTargetLaneMergeToOriginLane() const {
  const auto& ego_lane_road_right_decider_output =
      session_->planning_context().ego_lane_road_right_decider_output();

  bool is_merge_region = ego_lane_road_right_decider_output.is_merge_region;
  bool cur_lane_is_continue =
      ego_lane_road_right_decider_output.cur_lane_is_continue;
  int merge_lane_virtual_id =
      ego_lane_road_right_decider_output.merge_lane_virtual_id;

  const int fix_lane_virtual_id = lc_lane_mgr_->fix_lane_virtual_id();
  const int origin_lane_virtual_id = lc_lane_mgr_->origin_lane_virtual_id();
  const int target_lane_virtual_id = lc_lane_mgr_->target_lane_virtual_id();

  const auto& current_lane_virtual_id = session_->environmental_model()
                                            .get_virtual_lane_manager()
                                            ->current_lane_virtual_id();

  bool is_same_lane = current_lane_virtual_id == fix_lane_virtual_id;
  // 如果地图变道请求，不看感知主路辅路，直接return false
  bool is_map_lc = transition_info_.lane_change_type == MAP_REQUEST ||
                   transition_info_.lane_change_type == MERGE_REQUEST;
  if (is_map_lc) {
    return false;
  }
  if (transition_info_.lane_change_status == kLaneChangeExecution) {
    if (is_merge_region && !cur_lane_is_continue &&
        origin_lane_virtual_id == merge_lane_virtual_id && is_same_lane) {
      return true;
    }
  } else if (transition_info_.lane_change_status == kLaneChangePropose) {
    if (is_merge_region && cur_lane_is_continue &&
        target_lane_virtual_id == merge_lane_virtual_id && is_same_lane) {
      return true;
    }
  }

  return false;
}

bool LaneChangeStateMachineManager::
    IsNeedCancelLCTargetLaneMergeToOriginLane() {
  bool is_target_lane_merge_to_origin_lane = IsTargetLaneMergeToOriginLane();

  if (is_target_lane_merge_to_origin_lane) {
    lc_target_lane_merge_to_origin_lane_cnt_++;
    if (lc_target_lane_merge_to_origin_lane_cnt_ >= 3) {
      lane_change_stage_info_.lc_back_reason =
          "target lane merge to origin lane";
      return true;
    }
  } else {
    lc_target_lane_merge_to_origin_lane_cnt_ = 0;
  }

  return false;
}
bool LaneChangeStateMachineManager::IsCancelToHold() {
  const auto& virtual_lane_manager =
      session_->environmental_model().get_virtual_lane_manager();
  const auto origin_lane = virtual_lane_manager->get_lane_with_virtual_id(
      lc_lane_mgr_->origin_lane_virtual_id());
  if (origin_lane == nullptr) {
    return false;
  }

  const auto& reference_path_manager =
      session_->environmental_model().get_reference_path_manager();
  const auto& origin_ref_path =
      reference_path_manager->get_reference_path_by_lane(
          lc_lane_mgr_->origin_lane_virtual_id());
  if (origin_ref_path == nullptr) {
    return false;
  }

  const auto& origin_ego_state = origin_ref_path->get_frenet_ego_state();

  const double ego_origin_l = std::abs(origin_ego_state.l());
  const double origin_lane_width = origin_lane->width();
  if (ego_origin_l > origin_lane_width / 4.0) {
    return true;
  }

  return false;
}

double LaneChangeStateMachineManager::CalculateLCHoldStateLatOffset() const {
  const auto& ref_path_manager =
      session_->environmental_model().get_reference_path_manager();
  const auto& virtual_lane_manager =
      session_->environmental_model().get_virtual_lane_manager();
  const int fix_lane_virtual_id = lc_lane_mgr_->fix_lane_virtual_id();
  const auto fix_lane =
      virtual_lane_manager->get_lane_with_virtual_id(fix_lane_virtual_id);

  if (fix_lane == nullptr) {
    return 0.0;
  }
  const int target_lane_virtual_id = lc_lane_mgr_->target_lane_virtual_id();
  const int origin_lane_virtual_id = lc_lane_mgr_->origin_lane_virtual_id();
  const auto& target_lane =
      virtual_lane_manager->get_lane_with_virtual_id(target_lane_virtual_id);
  const auto& origin_lane =
      virtual_lane_manager->get_lane_with_virtual_id(origin_lane_virtual_id);
  if (target_lane == nullptr || origin_lane == nullptr) {
    return 0.0;
  }
  const double half_origin_lane_width = origin_lane->width() / 2.0;
  const double half_target_lane_width = target_lane->width() / 2.0;
  const auto& fix_ref_path =
      ref_path_manager->get_reference_path_by_lane(fix_lane_virtual_id);
  if (fix_ref_path == nullptr) {
    return 0.0;
  }
  //jerk约束低速基于steer rate
  double vel = fix_ref_path->get_frenet_ego_state().velocity();
  const auto &vehicle_param =
  VehicleConfigurationContext::Instance()->get_vehicle_param();
  double curv_factor = 1 / std::max(vehicle_param.wheel_base, 1e-6);
  const double kv2 = curv_factor * vel * vel;
  double steer_ratio = vehicle_param.steer_ratio;
  double max_steer_angle = vehicle_param.max_steer_angle;  // rad
  double max_steer_angle_rate_lc =
    std::min(vehicle_param.max_steer_angle_rate,
             lc_safety_check_config_.hold_steer_angle_rate_limit_deg / 57.3);
  double max_wheel_angle_rate_lc = max_steer_angle_rate_lc / steer_ratio;
  double steer_limit_jerk = max_wheel_angle_rate_lc * kv2;
  // 正常行驶基于横向差值
  double jerk_bound = planning::interp(vel,
      lc_safety_check_config_.hold_state_vel_jerk_map.vel_table,
      lc_safety_check_config_.hold_state_vel_jerk_map.jerk_table);
  double limit_jerk_hold = std::min(steer_limit_jerk, jerk_bound);
  //初始状态
  double vy = fix_ref_path->get_frenet_ego_state().velocity_l();
  const auto& ego_state_manager = session_->environmental_model().get_ego_state_manager();
  if (ego_state_manager == nullptr) {
    return 0.0;
  }
  double ego_steer_angle = ego_state_manager->ego_steer_angle();
  double ego_s = fix_ref_path->get_frenet_ego_state().s();
  double lat_acc = ego_steer_angle * kv2 / steer_ratio;
  double inertial_distance = ComputeInertialLatOffset(vy, lat_acc, limit_jerk_hold);
  //限幅
  inertial_distance = std::max(std::min(inertial_distance, half_origin_lane_width), 0.0);
  double inertial_lateral_offset = 0.0;
  // 惯性偏移后位置
  if (transition_info_.lane_change_direction == LEFT_CHANGE) {
    inertial_lateral_offset = fix_ref_path->get_frenet_ego_state().l() + inertial_distance;
  } else {
    inertial_lateral_offset = fix_ref_path->get_frenet_ego_state().l() - inertial_distance;
  }
  // 考虑后车僵持or超车
  // 保持在原车道内的偏移位置-避让后车位置
  double origin_lane_offset = half_origin_lane_width - vehicle_param.width / 2.0 - 0.1;
  double in_origin_lane_offset = 0.0;
  if(transition_info_.lane_change_direction == LEFT_CHANGE) {
    in_origin_lane_offset = origin_lane_offset;
  } else if(transition_info_.lane_change_direction == RIGHT_CHANGE) {
    in_origin_lane_offset = -origin_lane_offset;
  }
  double large_lateral_offset =
    (std::abs(inertial_lateral_offset) >= std::abs(in_origin_lane_offset))
      ? inertial_lateral_offset
      : in_origin_lane_offset; //大偏移
  double small_lateral_offset =
   (std::abs(inertial_lateral_offset) < std::abs(in_origin_lane_offset))
      ? inertial_lateral_offset
      : in_origin_lane_offset; //小偏移
  if(lane_change_stage_info_.lc_back_reason == "side view back"
    && rear_agent_overtaking_) {
    return small_lateral_offset; //小偏移
  }else{
    return large_lateral_offset; //大偏移
  }

}
ThirdOrderTimeOptimalTrajectory
LaneChangeStateMachineManager::GenerateLatMaxDecelerationCurve(
    const std::shared_ptr<ReferencePath> ref_path, const double p_end) {
  const auto frenet_ego_state = ref_path->get_frenet_ego_state();
  // 车辆参数
  const auto& vehicle_param =
      VehicleConfigurationContext::Instance()->get_vehicle_param();
  const double kEgoWidth = vehicle_param.width;

  const double k = 1 / vehicle_param.wheel_base;

  const double ego_steer_angle = session_->environmental_model()
                                     .get_ego_state_manager()
                                     ->ego_steer_angle();

  const double steer_ratio = vehicle_param.steer_ratio;
  LonState init_state;
  init_state.p = frenet_ego_state.planning_init_point().frenet_state.r;
  init_state.v = frenet_ego_state.planning_init_point().frenet_state.dr_ds;
  init_state.a = frenet_ego_state.planning_init_point().frenet_state.ddr_dsds;

  StateLimit state_limit;
  double v_max = 0.075;
  state_limit.p_end = p_end;
  state_limit.v_end = 0;
  state_limit.v_max = v_max;
  state_limit.v_min = -v_max;
  state_limit.a_max = v_max;
  state_limit.a_min = -v_max;
  state_limit.j_max = v_max;
  state_limit.j_min = -v_max;

  const double p_precision = 0.1;

  const auto lat_max_deceleration_curve =
      ThirdOrderTimeOptimalTrajectory(init_state, state_limit, p_precision);

  const auto ref_frenet_coord = ref_path->get_frenet_coord();
  if (ref_frenet_coord == nullptr) {
    return lat_max_deceleration_curve;
  }

  std::vector<double> x_vec;
  std::vector<double> y_vec;
  std::vector<double> l_vec;
  std::vector<double> s_vec;

  const double all_time = lat_max_deceleration_curve.ParamLength();
  const double dt = 1;

  for (double t = 0; t < all_time; t += dt) {
    double l = lat_max_deceleration_curve.Evaluate(0, t);
    double s = ref_path->get_frenet_ego_state().s() + t;
    l_vec.emplace_back(l);
    s_vec.emplace_back(s);
    Point2D sl_point{s, l};
    Point2D xy_point;
    if (!ref_frenet_coord->SLToXY(sl_point, xy_point)) {
      break;
    }
    x_vec.emplace_back(xy_point.x);
    y_vec.emplace_back(xy_point.y);
  }

  std::vector<double> ori_x_vec;
  std::vector<double> ori_y_vec;
  for (const auto& path_point : ref_frenet_coord->path_points()) {
    ori_x_vec.emplace_back(path_point.x());
    ori_y_vec.emplace_back(path_point.y());
  }

  return lat_max_deceleration_curve;
}

bool LaneChangeStateMachineManager::IsHighPriorityCompleteMLC() const {
  // 考虑自车是否在距离匝道较近的ramp变道
  const auto& ego_state_manager =
      session_->environmental_model().get_ego_state_manager();
  const auto& virtual_lane_manager =
      session_->environmental_model().get_virtual_lane_manager();

  const double triggle_dis =
      ego_state_manager->ego_v() * kPreTriggleHighPriorityMLCTime;

  const auto& cur_lane = virtual_lane_manager->get_current_lane();

  const auto& route_info_output =
      session_->environmental_model().get_route_info();

  bool is_high_priority_complete_mlc =
      !cur_lane->get_current_tasks().empty() &&
      route_info_output->get_route_info_output().distance_to_first_road_split <
          triggle_dis;

  return is_high_priority_complete_mlc;
}

bool LaneChangeStateMachineManager::CheckIfCancelToComplete() const {
  return CheckIfLaneChangeComplete(transition_info_.lane_change_direction,
                                   transition_info_.lane_change_type);
}

bool LaneChangeStateMachineManager::IsFilterStaticAgentLC(
    const planning_data::DynamicAgentNode& agent_node) const {
  if (!agent_node.is_static_type()) {
    return false;
  }

  const int target_lane_vir_id = lc_lane_mgr_->target_lane_virtual_id();
  const auto& reference_path_manager =
      session_->environmental_model().get_reference_path_manager();
  const auto& virtual_lane_manager =
      session_->environmental_model().get_virtual_lane_manager();

  const auto& targrt_ref_path =
      reference_path_manager->get_reference_path_by_lane(target_lane_vir_id);

  if (!targrt_ref_path) {
    return false;
  }

  const auto& target_coor = targrt_ref_path->get_frenet_coord();

  if (target_coor == nullptr) {
    return false;
  }

  const auto& target_lane =
      virtual_lane_manager->get_lane_with_virtual_id(target_lane_vir_id);

  if (target_lane == nullptr) {
    return false;
  }

  // planning::Point2D cart_point{agent_node.node_x(), agent_node.node_y()};
  // planning::Point2D frenet_point;
  // if (!target_coor->XYToSL(cart_point, frenet_point)) {
  //   return false;
  // }
  double s = 0.0;
  double l = 0.0;
  if (!target_coor->XYToSL(agent_node.node_x(), agent_node.node_y(), &s, &l)) {
    return false;  // 更换xytosl
  }
  planning::Point2D frenet_point(s, l);

  // 车辆参数
  const auto& vehicle_param =
      VehicleConfigurationContext::Instance()->get_vehicle_param();
  const double kEgoWidth = vehicle_param.width;

  const double buffer = 0.5;  // m

  const double need_lat_space = kEgoWidth + buffer;

  const double target_lane_width = target_lane->width_by_s(frenet_point.x);

  // 静态车辆侵入后，横向上的剩余空间能满足自车通行，那么可以把该静态障碍物过滤掉
  if (transition_info_.lane_change_direction == LEFT_CHANGE) {
    if (frenet_point.y > need_lat_space) {
      return true;
    }
  } else if (transition_info_.lane_change_direction == RIGHT_CHANGE) {
    if (frenet_point.y < -need_lat_space) {
      return true;
    }
  }
  return false;
}

void LaneChangeStateMachineManager::UpdateLCCoarsePlanningInfo() {
  const auto& cur_state =
      session_->planning_context().lane_change_decider_output().curr_state;

  if (cur_state == kLaneChangeExecution || cur_state == kLaneChangeCancel ||
      cur_state == kLaneChangeHold || cur_state == kLaneChangeComplete) {
    const auto& reference_path_manager =
        session_->environmental_model().get_reference_path_manager();
    const int fix_lane_virtual_id = lc_lane_mgr_->fix_lane_virtual_id();

    // 防止在跳execution那一帧，cur_lane没有更新，所以这里使用fix lane
    const auto& fix_lane_ref_path =
        reference_path_manager->get_reference_path_by_lane(fix_lane_virtual_id);
    if (!fix_lane_ref_path) {
      return;
    }

    lc_path_generate_ = std::make_shared<LaneChangePathGenerateManager>(
        fix_lane_ref_path, session_);

    if (!lc_path_generate_->GenerateLCPath(lc_hold_state_lat_offset_)) {
      return;
    }

    const auto lc_path_result = lc_path_generate_->get_lc_path_result();

    auto& traj_points = session_->mutable_planning_context()
                            ->mutable_lane_change_decider_output()
                            .coarse_planning_info.trajectory_points;

    UpdateLCPath(traj_points, lc_path_result, fix_lane_ref_path);
  }

  return;
}

void LaneChangeStateMachineManager::UpdateLCPath(
    TrajectoryPoints& traj_points,
    const LaneChangePathGenerateManager::LCPathResult& lc_path_result,
    const std::shared_ptr<ReferencePath> ref_path) {
  const auto& coarse_planning_info = session_->planning_context()
                                         .lane_change_decider_output()
                                         .coarse_planning_info;
  const auto& cur_ref = ref_path;
  const auto& cur_ref_path_coor = cur_ref->get_frenet_coord();

  const double ego_v = cur_ref->get_frenet_ego_state().velocity();
  // sample traj point on quintic path by t
  double lc_time = lc_path_result.t.back();
  TrajectoryPoint point;
  Eigen::Vector2d sample_point;
  Point2D frenet_point;
  size_t truncation_idx = 0;
  double s_truncation{0.0};
  for (size_t i = 0; i < traj_points.size(); i++) {
    if (traj_points[i].t < lc_time) {
      point.x = lc_path_result.x_t_spline(traj_points[i].t);
      point.y = lc_path_result.y_t_spline(traj_points[i].t);
      point.heading_angle = lc_path_result.theta_t_spline(traj_points[i].t);

      Point2D sl_point;
      Point2D xy_point(point.x, point.y);
      if (!cur_ref_path_coor->XYToSL(xy_point, sl_point)) {
        continue;
      }
      point.s = sl_point.x;
      point.l = sl_point.y;
      point.t = traj_points[i].t;

      traj_points[i] = point;
      truncation_idx = i;
    } else {
      if (i == truncation_idx + 1) {
        Eigen::Vector2d truncation_point(traj_points[truncation_idx].x,
                                         traj_points[truncation_idx].y);
        auto& cart_ref_info = coarse_planning_info.cart_ref_info;

        pnc::spline::Projection projection_truncation_point;
        projection_truncation_point.CalProjectionPoint(
            cart_ref_info.x_s_spline, cart_ref_info.y_s_spline,
            cart_ref_info.s_vec.front(), cart_ref_info.s_vec.back(),
            truncation_point);

        s_truncation = projection_truncation_point.GetOutput().s_proj;
      }
      point.s = std::fmin(s_truncation + ego_v * (i - truncation_idx) * 0.2,
                          cur_ref_path_coor->Length());
      point.l = 0;
      point.t = traj_points[i].t;

      frenet_point.x = point.s;
      frenet_point.y = point.l;
      Point2D cart_point;
      if (!cur_ref_path_coor->SLToXY(frenet_point, cart_point)) {
        LOG_ERROR("ERROR! Cart Point -> Frenet Point Failed!!!");
      }
      point.x = cart_point.x;
      point.y = cart_point.y;
      point.heading_angle = cur_ref_path_coor->GetPathCurveHeading(point.s);
      traj_points[i] = point;
    }
  }

  return;
}
FrenetObstacleBoundary LaneChangeStateMachineManager::GetSLboundaryFromAgent(
    const std::shared_ptr<ReferencePath> ref_path,
    const planning_math::Box2d& obs_box) {
  std::vector<planning_math::Vec2d> obs_corners;
  obs_corners = obs_box.GetAllCorners();
  std::vector<double> agent_sl_boundary(4);
  FrenetObstacleBoundary sl_bd;
  const auto& current_frenet_coord = ref_path->get_frenet_coord();
  for (size_t i = 0; i < obs_corners.size(); ++i) {
    double project_s = 0.0, project_l = 0.0;
    current_frenet_coord->XYToSL(obs_corners[i].x(), obs_corners[i].y(),
                                 &project_s,
                                 &project_l);  // 这是投影在路径上的 障碍物角点
    sl_bd.l_start = std::fmin(sl_bd.l_start, project_l);  // l right
    sl_bd.l_end = std::fmax(sl_bd.l_end, project_l);      // l left
    sl_bd.s_start = std::fmin(sl_bd.s_start, project_s);  // s start
    sl_bd.s_end = std::fmax(sl_bd.s_end, project_s);      // s end
  }
  return sl_bd;
}
bool LaneChangeStateMachineManager::PassInLane(
    double lane_width, const FrenetObstacleBoundary& obs_bd,
    const double car_width, const double safety_margin,
    const RequestType direction) {
  double half_lane_width = lane_width / 2.0;

  // 确保障碍物边界从小到大
  double obs_l_min = std::min(obs_bd.l_start, obs_bd.l_end);
  double obs_l_max = std::max(obs_bd.l_start, obs_bd.l_end);

  double lane_l_min = -half_lane_width;
  double lane_l_max = half_lane_width;
  if (direction == RequestType::LEFT_CHANGE) {
    // 左变道：看障碍物右侧到左车道边界的距离
    double left_free_width = obs_l_min - lane_l_min;
    return left_free_width >= (car_width + safety_margin);
  } else if (direction == RequestType::RIGHT_CHANGE) {
    // 右变道：看障碍物左侧到右车道边界的距离
    double right_free_width = lane_l_max - obs_l_max;
    return right_free_width >= (car_width + safety_margin);
  } else {
    return false;
  }
}
bool LaneChangeStateMachineManager::GetDecelerationTraj(
    double a0, const TrajectoryPoints& agent_traj,
    TrajectoryPoints& agent_deceleration_traj, const double deceleration,
    const double j, bool is_press_line) {
  is_press_line = false;  // 暂时不修饰-a轨迹
  const int point_size = agent_traj.size();
  if (point_size < 2) {
    return false;
  }
  if (agent_traj.back().s <= agent_traj.front().s) {
    agent_deceleration_traj = agent_traj;
    return false;
  }
  std::vector<double> s_vec(point_size);
  std::vector<double> x_vec(point_size);
  std::vector<double> y_vec(point_size);
  s_vec[0] = agent_traj[0].s;
  x_vec[0] = agent_traj[0].x;
  y_vec[0] = agent_traj[0].y;
  const double min_ds = 0.01;  // 最小增量
  for (int i = 1; i < point_size; ++i) {
    x_vec[i] = agent_traj[i].x;
    y_vec[i] = agent_traj[i].y;
    double dx = x_vec[i] - x_vec[i - 1];
    double dy = y_vec[i] - y_vec[i - 1];
    double ds = std::hypot(dx, dy);
    ds = std::max(ds, min_ds);
    s_vec[i] = s_vec[i - 1] + ds;
  }
  pnc::mathlib::spline x_s_spline;
  pnc::mathlib::spline y_s_spline;
  x_s_spline.set_points(s_vec, x_vec);
  y_s_spline.set_points(s_vec, y_vec);
  agent_deceleration_traj.clear();
  agent_deceleration_traj.resize(point_size);
  const double s0 = agent_traj.front().s;
  const double v0 = agent_traj.front().v;
  const double dt = 0.2;  // 时间步长
  double dec_a = 0.0;
  double dec_v = 0.0;
  double dec_s = 0.0;
  for (int i = 0; i < point_size; ++i) {
    double t = i * dt;
    if (is_press_line) {
      // 匀减速 s,v 公式
      dec_v = std::max(0.0, v0 + deceleration * t);
      dec_s = s0 + v0 * t + 0.5 * deceleration * t * t;
    } else {
      // 匀jerk
      //  double j = -2.0;  // 恒定 jerk
      dec_a = a0 + j * t;  // 当前加速度
      dec_v = std::max(0.1, v0 + a0 * t + 0.5 * j * t * t);
      dec_s = s0 + v0 * t + 0.5 * a0 * t * t + (1.0 / 6.0) * j * t * t * t;
    }
    // 确保位移不小于前一个点
    if (i > 0) {
      dec_s = std::max(dec_s,
                       agent_deceleration_traj[i - 1].s + 0.1);  // 最小前进距离
    }
    dec_s = std::clamp(dec_s, 0.0, s_vec.back());
    double dec_x = x_s_spline(dec_s);
    double dec_y = y_s_spline(dec_s);
    // heading 由 dx/ds, dy/ds 求，不插值
    double dx = x_s_spline.deriv(1, dec_s);
    double dy = y_s_spline.deriv(1, dec_s);
    double dec_theta = std::atan2(dy, dx);
    auto& pt = agent_deceleration_traj[i];
    pt.x = dec_x;
    pt.y = dec_y;
    pt.s = dec_s;
    pt.v = dec_v;
    pt.a = deceleration;
    pt.t = t;
    pt.heading_angle = dec_theta;
  }

  return true;
}

bool LaneChangeStateMachineManager::IsSuppressLCShortDis() const {
  // const double ego_v =
  //     session_->environmental_model().get_ego_state_manager()->ego_v();

  // if (ego_v > 1.0) {
  //   return false;
  // }

  // 获取自车道前方障碍物车辆
  // const auto& dynamic_world =
  //     session_->environmental_model().get_dynamic_world();
  // const int64_t ego_front_node_id = dynamic_world->ego_front_node_id();

  // const auto& ego_front_node = dynamic_world->GetNode(ego_front_node_id);

  // if (ego_front_node == nullptr) {
  //   return false;
  // }

  // bool is_suppress =
  //     ego_front_node->is_static_type() &&
  //     ego_front_node->node_back_edge_to_ego_front_edge_distance() < config_.lc_short_dis_thr;

  // if (!is_suppress) {
  //   return false;
  // }
  const auto& virtual_lane_manager =
      session_->environmental_model().get_virtual_lane_manager();
  const auto& cur_ref_path = session_->environmental_model()
                                 .get_reference_path_manager()
                                 ->get_reference_path_by_current_lane();

  if (cur_ref_path == nullptr) {
    return true;
  }
  const auto& ego_boundary = cur_ref_path->get_ego_frenet_boundary();
  const auto& cur_coord = cur_ref_path->get_frenet_coord();
  if (cur_coord == nullptr) {
    return true;
  }
  bool is_suppress = false;
  const double buffer = 0.5;
  const int fix_lane_virtual_id = lc_lane_mgr_->fix_lane_virtual_id();
  const auto& fix_lane_nodes =
      session_->environmental_model().get_dynamic_world()->GetNodesByLaneId(
          fix_lane_virtual_id);

  for (const auto* fix_lane_node : fix_lane_nodes) {
    if (fix_lane_node == nullptr) {
      continue;;
    }
    double x = fix_lane_node->node_x();
    double y = fix_lane_node->node_y();
    Point2D node_cart(x, y);
    double s = 0;
    double l = 0;
    const auto& nearest_lane =
        virtual_lane_manager->GetNearestLane(node_cart, &s, &l);
    if (nearest_lane == nullptr) {
      continue;
    }
    double agent_s = fix_lane_node->node_s();
    if (agent_s + fix_lane_node->node_length() * 0.5 < ego_boundary.s_start) {
      continue;
    }
    is_suppress =
        fix_lane_node->is_static_type() &&
        fix_lane_node->node_back_edge_to_ego_front_edge_distance() < config_.lc_short_dis_thr;
    if (!is_suppress) {
      continue;
    }
    Point2D frenet_point;
    if (!cur_coord->XYToSL(node_cart, frenet_point)) {
      continue;
    }
    double agent_half_width = fix_lane_node->node_width() * 0.5;
    if (lc_req_mgr_->request() == LEFT_CHANGE) {
      if (ego_boundary.l_start < frenet_point.y + agent_half_width - buffer) {
        return true;
      }
    } else if (lc_req_mgr_->request() == RIGHT_CHANGE) {
      if (ego_boundary.l_end > frenet_point.y - agent_half_width + buffer) {
        return true;
      }
    }
  }

  return false;
}

bool LaneChangeStateMachineManager::CheckTargetLaneValid() {
  bool is_valid = true;
  if (transition_info_.lane_change_type == INT_REQUEST) {
    const auto& origin_lane_virtual_id = lc_lane_mgr_->origin_lane_virtual_id();
    const auto& target_lane_virtual_id_tmp =
        transition_info_.lane_change_direction == LEFT_CHANGE
            ? origin_lane_virtual_id - 1
            : origin_lane_virtual_id + 1;
    const auto virtual_lane_manager =
        session_->environmental_model().get_virtual_lane_manager();
    if (virtual_lane_manager == nullptr) {
      is_valid = false;
    }
    auto tlane = virtual_lane_manager->get_lane_with_virtual_id(
        target_lane_virtual_id_tmp);
    if (tlane == nullptr ||
        tlane->get_lane_type() == iflyauto::LANETYPE_OPPOSITE) {
      is_valid = false;
    } else if ((transition_info_.lane_change_status == kLaneChangePropose ||
                transition_info_.lane_change_status == kLaneKeeping) &&
               lc_request_.IsRoadBorderSurpressLaneChange(
                   transition_info_.lane_change_direction,
                   origin_lane_virtual_id, target_lane_virtual_id_tmp)) {
      is_valid = false;
    }
  }
  return is_valid;
}

// 输入前后车node_id，判断是否当前满足gap enough 条件
bool LaneChangeStateMachineManager::IsSideClear(int front_agent_id,
                                                int rear_agent_id) {
  const auto& dynamic_world =
      session_->environmental_model().get_dynamic_world();
  const auto& agent_mgr = dynamic_world->agent_manager();
  const auto front_agent = agent_mgr->GetAgent(front_agent_id);
  const auto rear_agent = agent_mgr->GetAgent(rear_agent_id);
  const auto& ref_path = session_->environmental_model()
                             .get_reference_path_manager()
                             ->get_reference_path_by_current_lane();
  //自车boundary
  if (ego_trajs_future_.empty()) {
    return false;
  }
  if (front_agent != nullptr && ref_path != nullptr) {
    //前车sl boundary计算
    const auto& front_agent_bd =
        GetSLboundaryFromAgent(ref_path, front_agent->box());
    const auto& ego_boundary = ref_path->get_ego_frenet_boundary();
    if (front_agent_bd.s_start < ego_boundary.s_end) {
      return false;
    }
  }
  if (rear_agent != nullptr && ref_path != nullptr) {
    const auto& rear_agent_bd =
        GetSLboundaryFromAgent(ref_path, rear_agent->box());
    const auto& ego_boundary = ref_path->get_ego_frenet_boundary();
    if (rear_agent_bd.s_end > ego_boundary.s_start) {
      return false;
    }
  }
  return true;
}

bool LaneChangeStateMachineManager::IsExistExtendLane(
    const iflymapdata::sdpro::Lane* lane, bool is_rightest_lane) const {
  if (lane == nullptr) {
    return false;
  }
  const iflymapdata::sdpro::Lane* succesor_add_lane = nullptr;
  const iflymapdata::sdpro::Lane* iterator_lane = lane;
  double sum_dis = 0.0;

  const auto& route_info = session_->environmental_model().get_route_info();
  if (route_info == nullptr) {
    return false;
  }
  const auto& sd_promap = route_info->get_sdpro_map();
  const auto& route_info_output = route_info->get_route_info_output();

  while (iterator_lane) {
    if (iterator_lane->id() == lane->id()) {
      sum_dis = sum_dis + iterator_lane->length() * 0.01 -
                route_info_output.current_segment_passed_distance;
    } else {
      sum_dis = sum_dis + iterator_lane->length() * 0.01;
    }

    if (sum_dis > 100.0) {
      break;
    }

    if (iterator_lane->successor_lane_ids_size() > 1) {
      succesor_add_lane = iterator_lane;
      break;
    }

    if (iterator_lane->successor_lane_ids().empty()) {
      return false;
    }

    iterator_lane =
        sd_promap.GetLaneInfoByID(iterator_lane->successor_lane_ids()[0]);
  }

  if (succesor_add_lane == nullptr) {
    return false;
  }

  for (const auto& lane_id : succesor_add_lane->successor_lane_ids()) {
    const auto& lane_info = sd_promap.GetLaneInfoByID(lane_id);
    if (lane_info == nullptr) {
      continue;
    }

    if (is_rightest_lane) {
      if (lane_info->lane_transiton() ==
          iflymapdata::sdpro::LTS_LANE_ADD_RIGHT) {
        return true;
      }
    } else {
      if (lane_info->lane_transiton() ==
          iflymapdata::sdpro::LTS_LANE_ADD_LEFT) {
        return true;
      }
    }
  }
  return false;
}

std::pair<const iflymapdata::sdpro::Lane*, const iflymapdata::sdpro::Lane*>
LaneChangeStateMachineManager::CalculateLeftestRightestLane(
    const iflymapdata::sdpro::LinkInfo_Link* link) const {
  if (link == nullptr) {
    return {nullptr, nullptr};
  }

  const iflymapdata::sdpro::Lane* leftest_lane = nullptr;
  const iflymapdata::sdpro::Lane* rightest_lane = nullptr;
  const auto& route_info = session_->environmental_model().get_route_info();
  if (route_info == nullptr) {
    return {nullptr, nullptr};
  }
  const auto& sd_promap = route_info->get_sdpro_map();

  std::vector<std::pair<const iflymapdata::sdpro::Lane*, int>>
      valid_lane_sequence;
  for (const auto& lane_id : link->lane_ids()) {
    const iflymapdata::sdpro::Lane* lane_info =
        sd_promap.GetLaneInfoByID(lane_id);
    if (lane_info == nullptr) {
      continue;
    }

    bool is_exist_emergency_lane = false;
    for (const auto& lane_type : lane_info->lane_type()) {
      if (lane_type == iflymapdata::sdpro::LAT_EMERGENCY) {
        is_exist_emergency_lane = true;
        break;
      }
    }
    if (is_exist_emergency_lane) {
      continue;
    }

    bool is_exist_diversion_lane = false;
    for (const auto& lane_type : lane_info->lane_type()) {
      if (lane_type == iflymapdata::sdpro::LAT_DIVERSION) {
        is_exist_diversion_lane = true;
        break;
      }
    }
    if (is_exist_diversion_lane) {
      continue;
    }

    valid_lane_sequence.emplace_back(lane_info, lane_info->sequence());
  }

  if (valid_lane_sequence.empty()) {
    return {nullptr, nullptr};
  }

  // 按sequence排序
  std::sort(valid_lane_sequence.begin(), valid_lane_sequence.end(),
            [](const auto& a, const auto& b) { return a.second < b.second; });

  rightest_lane = valid_lane_sequence.front().first;
  leftest_lane = valid_lane_sequence.back().first;

  return {leftest_lane, rightest_lane};
}

RampDirection
LaneChangeStateMachineManager::CalcTurnSignalForTencentSplitRegion() const {
  const auto& function_mode =
      session_->environmental_model().function_info().function_mode();
  const auto& route_info = session_->environmental_model().get_route_info();
  if (route_info == nullptr) {
    return RAMP_NONE;
  }
  const auto& route_info_output = route_info->get_route_info_output();
  const auto& sdpro_map = route_info->get_sdpro_map();

  if (function_mode != common::DrivingFunctionInfo::NOA ||
      route_info_output.map_vendor !=
          iflymapdata::sdpro::MAP_VENDOR_TENCENT_SD_PRO ||
      !route_info_output.is_ego_on_expressway ||
      route_info_output.map_split_region_info_list.empty()) {
    return RAMP_NONE;
  }

  double split_start_position = 0.0;

  split_start_position = route_info_output.map_split_region_info_list.front()
                             .start_fp_point.fp_distance_to_split_point;
  const auto& start_fp_point =
      route_info_output.map_split_region_info_list.front().start_fp_point;
  const auto& end_fp_point =
      route_info_output.map_split_region_info_list.front().end_fp_point;

  if (route_info_output.map_split_region_info_list.front().distance_to_split_point +
          split_start_position >
      100.0) {
    return RAMP_NONE;
  }

  if (!route_info->get_sdmap_valid()) {
    return RAMP_NONE;
  }
  const auto& sd_promap =
      route_info->get_sdpro_map();
  const auto* current_link = sd_promap.GetLinkOnRoute(start_fp_point.link_id);
  if (current_link == nullptr) {
    return RAMP_NONE;
  }
  const auto* next_link = sd_promap.GetLinkOnRoute(end_fp_point.link_id);
  if (next_link == nullptr) {
    return RAMP_NONE;
  }

  bool current_link_is_ramp = sd_promap.isRamp(current_link->link_type()) ||
                              sd_promap.isSaPa(current_link->link_type());

  bool next_link_is_ramp = sd_promap.isRamp(next_link->link_type()) ||
                           sd_promap.isSaPa(next_link->link_type());

  for (const auto& lane_id : start_fp_point.lane_ids) {
    const auto* lane = sd_promap.GetLaneInfoByID(lane_id);
    if (lane == nullptr) {
      continue;
    }
    if ((lane->change_type() ==
             iflymapdata::sdpro::LaneChangeType::LeftTurnExpandingLane ||
         lane->change_type() ==
             iflymapdata::sdpro::LaneChangeType::BothDirectionExpandingLane) &&
        route_info_output.map_split_region_info_list.front().split_direction ==
            SPLIT_LEFT) {
      return RAMP_ON_LEFT;
    }
    if ((lane->change_type() ==
             iflymapdata::sdpro::LaneChangeType::RightTurnExpandingLane ||
         lane->change_type() ==
             iflymapdata::sdpro::LaneChangeType::BothDirectionExpandingLane) &&
        route_info_output.map_split_region_info_list.front().split_direction ==
            SPLIT_RIGHT) {
      return RAMP_ON_RIGHT;
    }
  }

  return RAMP_NONE;
}

RampDirection
LaneChangeStateMachineManager::CalcTurnSignalForBaiduSplitRegion() const {
  const auto& function_mode =
      session_->environmental_model().function_info().function_mode();
  const auto& route_info = session_->environmental_model().get_route_info();
  if (route_info == nullptr) {
    return RAMP_NONE;
  }
  const auto& route_info_output = route_info->get_route_info_output();

  if (function_mode != common::DrivingFunctionInfo::NOA ||
      route_info_output.map_vendor != iflymapdata::sdpro::MAP_VENDOR_BAIDU_LD ||
      !route_info_output.is_ego_on_expressway ||
      route_info_output.map_split_region_info_list.empty()) {
    return RAMP_NONE;
  }

  const auto& current_link = route_info->get_current_link();
  if (!current_link) {
    return RAMP_NONE;
  }

  const auto& [leftest_lane, rightest_lane] =
      CalculateLeftestRightestLane(current_link);

  // 判断前方100m内是否有1分2的extend车道
  bool is_rightest_extend_lane = IsExistExtendLane(rightest_lane, true);
  bool is_leftest_extend_lane = IsExistExtendLane(leftest_lane, false);
  constexpr double kNearingExchangeUpperThreshold = 100.0;
  constexpr double kNearingExchangeLowerThreshold = -20.0;

  if (!route_info_output.map_split_region_info_list.empty()) {
    double distance_to_exchange =
        route_info_output.map_split_region_info_list.front()
            .distance_to_split_point +
        route_info_output.map_split_region_info_list.front()
            .start_fp_point.fp_distance_to_split_point;
    bool is_nearing_exchange =
        distance_to_exchange < kNearingExchangeUpperThreshold &&
        distance_to_exchange > kNearingExchangeLowerThreshold;
    if (is_rightest_extend_lane &&
        route_info_output.map_split_region_info_list.front().split_direction ==
            SPLIT_RIGHT &&
        is_nearing_exchange) {
      return RAMP_ON_RIGHT;
    } else if (is_leftest_extend_lane &&
               route_info_output.map_split_region_info_list.front()
                       .split_direction == SPLIT_LEFT &&
               is_nearing_exchange) {
      return RAMP_ON_LEFT;
    }
  }
  return RAMP_NONE;
}
double LaneChangeStateMachineManager::ComputeInertialLatOffset(double v_y0, double a_y0, double j_max) const {
  v_y0 = std::fabs(v_y0);
  a_y0 = std::fabs(a_y0);
  j_max = std::fabs(j_max);

  if (j_max < kEps) {
      return 0.0;
  }

  v_y0 = std::max(0.0, v_y0);
  a_y0 = std::max(0.0, a_y0);

  if (v_y0 < kEps && a_y0 < kEps) {
      return 0.0;
  }

  return
      (v_y0 * a_y0) / j_max +
      (a_y0 * a_y0 * a_y0) / (3.0 * j_max * j_max);
}
bool LaneChangeStateMachineManager::IsWarningCollisionRisk() {
  const double ego_press_line_ratio =
    lc_request_.CalculatePressLineRatioByTwoLanes(
        lc_lane_mgr_->origin_lane_virtual_id(),
        lc_lane_mgr_->target_lane_virtual_id(),
        transition_info_.lane_change_direction);
  const auto& ego_state_manager = session_->environmental_model().get_ego_state_manager();
  double ego_v = ego_state_manager->ego_v();
  double risk_obs_vel = lc_back_track_.v_rel;
  double distance_to_risk_obs = lc_back_track_.d_rel;
  const int risk_obs_id = lc_back_track_.track_id;
  bool is_agent_risk = false;
  // 13变2 的直接播报
  if(!risk_side_agents_nodes_.empty()) {
    for (const auto& side_obs : risk_side_agents_nodes_) {
      if (side_obs == nullptr) {
            continue;
      }
      if (side_obs->node_agent_id() == risk_obs_id) {
        is_agent_risk = true;
        break;
      }
    }
  }
  if(target_lane_front_node_ != nullptr) {
    if(target_lane_front_node_->node_agent_id() == risk_obs_id) {
      //危险性判断：自车无法舒适减速
      double rel_distance = target_lane_front_node_->node_back_edge_to_ego_front_edge_distance();
      double agent_vel = target_lane_front_node_->node_speed();
      double ego_comfort_brake = 1.2;
      double driver_reaction_time = 0.3;
      double delta_v = std::max(0.0, ego_v - agent_vel);
      //自车减速到前车速度需要多走的距离 + 自车反应时距
      double ego_comfort_decel_distance = (delta_v * delta_v) / (2.0 * ego_comfort_brake);
      ego_comfort_decel_distance = std::max(ego_comfort_decel_distance, 0.0);
      if(rel_distance < ego_comfort_decel_distance +  ego_v * driver_reaction_time) {
        is_agent_risk = true;
      }
    }
  }
  if(target_lane_rear_node_ != nullptr) {
    //危险性判断: 后车距离过近或者无法舒适让行
    if(target_lane_rear_node_->node_agent_id() == risk_obs_id) {
      double rel_distance = target_lane_rear_node_->node_front_edge_to_ego_back_edge_distance();
      double agent_vel = target_lane_rear_node_->node_speed();
      double rear_comfort_decel = lc_safety_check_config_.rear_comfort_decel;
      if (IsLargeAgent(target_lane_rear_node_)) {
        rear_comfort_decel = 0.3;
      }
      double driver_reaction_time = 0.3;
      double delta_v = std::max(0.0, agent_vel - ego_v);
      //后车减速到自车速度需要多走的距离 + 后车反应时距
      double rear_comfort_decel_distance = (delta_v * delta_v) / (2.0 * rear_comfort_decel);
      rear_comfort_decel_distance = std::max(rear_comfort_decel_distance, 0.0);
      if(rel_distance < rear_comfort_decel_distance +  agent_vel * driver_reaction_time) {
        is_agent_risk = true;
      }
    }
  }
  return is_agent_risk;
}

bool LaneChangeStateMachineManager::IsEmergencyScene() const {
  // 调试用，如果配置文件为true，则直接返回true
  if (lc_safety_check_config_.is_default_aggressive_scence) {
    return true;
  }
  const auto& route_info_output =
      session_->environmental_model().get_route_info()->get_route_info_output();

  // 如果变道来源是 MERGE_REQUEST，返回 true
  if (lc_req_mgr_->request_source() == MERGE_REQUEST) {
    return true;
  }

  // 如果变道来源是 MAP_REQUEST
  if (lc_req_mgr_->request_source() == MAP_REQUEST) {
    // 场景类型是 MERGE_SCENE
    if (route_info_output.mlc_decider_scene_type_info.mlc_scene_type ==
        MERGE_SCENE) {
      return true;
    }

    constexpr double kMergeStopLineDistanceThreshold = 200.0;
    const auto& virtual_lane_mgr =
        session_->environmental_model().get_virtual_lane_manager();
    const auto& merge_point_info =
        route_info_output.map_merge_points_info.empty()
            ? virtual_lane_mgr->get_current_lane()->get_map_merge_point_info()
            : route_info_output.map_merge_points_info.front();

    if (route_info_output.mlc_decider_scene_type_info.mlc_scene_type ==
        SPLIT_SCENE) {
      double distance_to_split_point =
          route_info_output.mlc_decider_scene_type_info.dis_to_link_topo_change_point;
      double merge_stop_line_distance =
          distance_to_split_point - route_info_output.lsl_length;
      if (merge_stop_line_distance <= kMergeStopLineDistanceThreshold) {
        return true;
      }
    }
  }
  return false;
}
bool LaneChangeStateMachineManager::IsLCPathCollisionWithRoadEdge(
    int origin_lane_id, int target_lane_id,
    const StateMachineLaneChangeStatus& lc_status) {
  const auto& traj_points =
      session_->mutable_planning_context()->last_planning_result().traj_points;
  const auto& ego_trajs_future_points =
      lc_status == kLaneChangeExecution ? traj_points : ego_trajs_future_copy_;
  return lc_request_.IsPathCollisionWithRoadEdge(origin_lane_id,
                                     target_lane_id, ego_trajs_future_points);
}

bool LaneChangeStateMachineManager::IsLCPathCollisionWithSolidLine(
    int origin_lane_id, const StateMachineLaneChangeStatus& lc_status,
    const RequestSource& lc_request_source,
    const RequestType& lc_request_type) const {
  // 优先处理距离下匝道100米内忽略虚实线的逻辑
  if (lc_request_.IsMLCIgnoreSolidLaneCheck(lc_request_type,
                                            lc_request_source)) {
    return false;
  }

  const double road_line_buffer = 0.075;
  const auto& virtual_lane_mgr =
      session_->environmental_model().get_virtual_lane_manager();
  std::shared_ptr<ReferencePathManager> reference_path_mgr =
      session_->mutable_environmental_model()->get_reference_path_manager();
  const auto& lane_change_decider_output =
      session_->planning_context().lane_change_decider_output();
  const auto& traj_points =
      session_->mutable_planning_context()->last_planning_result().traj_points;

  std::shared_ptr<ReferencePath> origin_reference_path =
      reference_path_mgr->get_reference_path_by_lane(origin_lane_id, false);
  const std::shared_ptr<VirtualLane> origin_lane =
      virtual_lane_mgr->get_lane_with_virtual_id(origin_lane_id);
  if (!origin_reference_path) {
    return true;
  }
  if (!origin_lane) {
    return true;
  }

  const auto frenet_coord = origin_reference_path->get_frenet_coord();
  if (frenet_coord == nullptr || !frenet_coord->KdtreeValid()) {
    return true;
  }
  const auto& vehicle_param =
      VehicleConfigurationContext::Instance()->get_vehicle_param();
  const double half_vehicle_width = vehicle_param.width * 0.5;
  const double vehicle_length = vehicle_param.length;

  // 获取车道边界分段信息，用于精确判断实线/虚线类型
  const auto& lane_boundarys = lc_request_type == LEFT_CHANGE
                                   ? origin_lane->get_left_lane_boundary()
                                   : origin_lane->get_right_lane_boundary();
  const auto boundary_path =
      virtual_lane_mgr->MakeBoundaryPath(lane_boundarys);
  if (boundary_path == nullptr) {
    return true;
  }

  const auto& ego_trajs_future_points =
      lc_status == kLaneChangeExecution ? traj_points : ego_trajs_future_copy_;
  for (size_t i = 0; i < ego_trajs_future_points.size(); ++i) {
    double s = 0.0, l = 0.0;
    if (!frenet_coord->XYToSL(ego_trajs_future_points[i].x,
                              ego_trajs_future_points[i].y, &s, &l)) {
      continue;
    }
    ReferencePathPoint ref_pt{};
    if (!origin_reference_path->get_reference_point_by_lon(s, ref_pt)) {
      continue;
    }
    const double left_vehicle_edge = l + half_vehicle_width;
    const double right_vehicle_edge = l - half_vehicle_width;

    // 通过车道线分段精确获取当前轨迹点对应位置的车道线类型
    double boundary_s = 0.0, boundary_l = 0.0;
    if (!boundary_path->XYToSL(ego_trajs_future_points[i].x,
                               ego_trajs_future_points[i].y, &boundary_s,
                               &boundary_l)) {
      continue;
    }

    // 检查 [boundary_s, boundary_s + vehicle_length] 范围内是否存在实线段
    const double boundary_s_end = boundary_s + vehicle_length;
    bool is_solid_type = false;
    double acc_length = 0.0;
    for (int j = 0; j < lane_boundarys.type_segments_size; ++j) {
      const double seg_start = acc_length;
      acc_length += lane_boundarys.type_segments[j].length;
      if (acc_length <= boundary_s) {
        continue;
      }
      if (seg_start > boundary_s_end) {
        break;
      }
      const auto seg_type = lane_boundarys.type_segments[j].type;
      const bool seg_is_solid =
          lc_request_type == LEFT_CHANGE
              ? seg_type == iflyauto::LaneBoundaryType_MARKING_SOLID ||
                    seg_type ==
                        iflyauto::LaneBoundaryType_MARKING_DOUBLE_SOLID ||
                    seg_type ==
                        iflyauto::
                            LaneBoundaryType_MARKING_LEFT_DASHED_RIGHT_SOLID ||
                    seg_type ==
                        iflyauto::LaneBoundaryType_MARKING_DECELERATION_SOLID
              : seg_type == iflyauto::LaneBoundaryType_MARKING_SOLID ||
                    seg_type ==
                        iflyauto::LaneBoundaryType_MARKING_DOUBLE_SOLID ||
                    seg_type ==
                        iflyauto::
                            LaneBoundaryType_MARKING_LEFT_SOLID_RIGHT_DASHED ||
                    seg_type ==
                        iflyauto::LaneBoundaryType_MARKING_DECELERATION_SOLID;
      if (seg_is_solid) {
        is_solid_type = true;
        break;
      }
    }

    if (lc_request_type == LEFT_CHANGE && is_solid_type &&
        (left_vehicle_edge >
         ref_pt.distance_to_left_lane_border - road_line_buffer) &&
        (right_vehicle_edge <
         ref_pt.distance_to_left_lane_border + road_line_buffer)) {
      return true;
    }
    if (lc_request_type == RIGHT_CHANGE && is_solid_type &&
        (left_vehicle_edge >
         -ref_pt.distance_to_left_lane_border - road_line_buffer) &&
        (right_vehicle_edge <
         -ref_pt.distance_to_right_lane_border + road_line_buffer)) {
      return true;
    }
  }
  return false;
}

void LaneChangeStateMachineManager::SplitSelectingStateMachine() {
  switch (split_selecting_info_.split_selecting_status) {
    case kNonSelecting: {
      const auto& virtual_lane_manager = session_->environmental_model().get_virtual_lane_manager();
      const auto& current_lane = virtual_lane_manager->get_current_lane();
      if(current_lane == nullptr){
        break;
      }
      bool is_none_to_selecting = IsStartSplitSelecting();
      if (is_none_to_selecting) {
        split_selecting_info_.split_selecting_status = kSelectingExecuting;
        split_selecting_info_.selected_lane_order_id = current_lane->get_order_id();
        split_selecting_info_.selected_lane_virtual_id = current_lane->get_virtual_id();
        SetSelectingDirection(split_selecting_info_);
      } else {
        // 上游没有拨杆选道的时候记录orderid
        split_selecting_info_.origin_lane_order_id = current_lane->get_order_id();
        split_selecting_info_.origin_lane_virtual_id = current_lane->get_virtual_id();
        split_selecting_info_.split_selecting_status = kNonSelecting;
        split_selecting_info_.split_select_direction = SplitSelectingNone;
      }
      break;
    }
    case kSelectingExecuting: {
      bool update_fix_success = UpdateSelectingFixlane();  // 设置fix lane id
      bool is_selecting_to_cancel = !update_fix_success;   // 如果没有当前车道非法则取消
      bool is_selecting_to_complete = IsSelectingToComplete();
      if (is_selecting_to_cancel) {
        split_selecting_info_.split_selecting_status = kNonSelecting;
        split_selecting_info_.split_select_direction = SplitSelectingNone;
      } else if (is_selecting_to_complete) {
        split_selecting_info_.split_selecting_status = kSelectingComplete;
      }
      break;
    }
    case kSelectingComplete: {
      bool is_complete_to_none = true; // complete 下一帧恢复none
      if (is_complete_to_none) {
        split_selecting_info_.split_selecting_status = kNonSelecting;
        split_selecting_info_.split_select_direction = SplitSelectingNone;
        ResetStateMachine(); // 充值拨杆信号
        ResetSplitSelectingStateMachine();  // 重置
      }
      break;
    }
    default:
      break;
  }
}
bool LaneChangeStateMachineManager::IsStartSplitSelecting() {
  // 获取上游选道标志位和方向
  const auto& virtual_lane_manager = session_->environmental_model().get_virtual_lane_manager();
  //多种选道是否存在
  bool is_exist_intersection_split = virtual_lane_manager->get_is_exist_intersection_split();
  bool is_exist_ramp_on_road = virtual_lane_manager->get_is_exist_ramp_on_road();
  bool is_exist_split_on_ramp = virtual_lane_manager->get_is_exist_split_on_ramp();
  //是否拨杆
  bool is_exist_interactive_select_split = virtual_lane_manager->get_is_exist_interactive_select_split();
  if((is_exist_intersection_split || is_exist_ramp_on_road || is_exist_split_on_ramp) 
    && is_exist_interactive_select_split){
    split_selecting_info_.unfinished_reason = SplitSelectingInfo::NONE_REASON;
    return true;
  }else if(!is_exist_interactive_select_split){
    split_selecting_info_.unfinished_reason = SplitSelectingInfo::NO_INTERACTIVE_CMD;
    return false;
  }else{
    split_selecting_info_.unfinished_reason = SplitSelectingInfo::NO_SPLIT_REGION;
    return false;
  }
}
void LaneChangeStateMachineManager::SetSelectingDirection(SplitSelectingInfo& split_selecting_info){
  int int_lane_change_cmd = lc_req_mgr_->get_int_lane_change_cmd();
  if(int_lane_change_cmd == 1){
    split_selecting_info.split_select_direction = SplitSelectingLeft;
  }else if(int_lane_change_cmd == 2){
    split_selecting_info.split_select_direction = SplitSelectingRight;
  }else{
    split_selecting_info.split_select_direction = SplitSelectingNone;
  }
}
bool LaneChangeStateMachineManager::UpdateSelectingFixlane(){
    // 设置fix lane id
    const auto& virtual_lane_manager = session_->environmental_model().get_virtual_lane_manager();
    const auto& current_lane = virtual_lane_manager->get_current_lane();
    if(current_lane == nullptr){
      split_selecting_info_.unfinished_reason = SplitSelectingInfo::NO_FIX_LANE;
      return false;
    }else{
      split_selecting_info_.unfinished_reason = SplitSelectingInfo::NONE_REASON;
          // 持续更新目标车道id
      split_selecting_info_.selected_lane_virtual_id = current_lane->get_virtual_id();
      split_selecting_info_.selected_lane_order_id = current_lane->get_order_id();
      return true; // 设置fix lane id成功
    }
  }
bool LaneChangeStateMachineManager::IsSelectingToComplete(){
  //是否完成选道：自车离开分流区域， 距离当前车道足够近
  bool is_exist_split_region = session_->environmental_model().get_virtual_lane_manager()->get_is_ego_in_split_region();
  if(is_exist_split_region){
    split_selecting_info_.unfinished_reason = SplitSelectingInfo::STILL_IN_SPLIT_REGION;
    return false; //仍然在分流区域
  }
  const auto& virtual_lane_manager = session_->environmental_model().get_virtual_lane_manager();
  const auto& current_lane = virtual_lane_manager->get_current_lane();
  //选道过程fix current lane 
  if(current_lane == nullptr){
    split_selecting_info_.unfinished_reason = SplitSelectingInfo::NO_FIX_LANE;
    return false;
  }
  const auto& ref_path_manager = session_->environmental_model().get_reference_path_manager();
  const auto& ref_path = ref_path_manager->get_reference_path_by_current_lane();
  if(ref_path == nullptr){
    split_selecting_info_.unfinished_reason = SplitSelectingInfo::NO_FIX_LANE_REF;
    return false;
  }
  double distance_to_fix_lane = std::fabs(ref_path->get_frenet_ego_state().l());
  if(distance_to_fix_lane > 0.5){
    split_selecting_info_.unfinished_reason = SplitSelectingInfo::NOT_CLOSE_TO_FIX_LANE;
    return false; //距离当前车道足够近
  }else{
    split_selecting_info_.unfinished_reason = SplitSelectingInfo::NONE_REASON;
    return true;
  }
}
void LaneChangeStateMachineManager::ResetSplitSelectingStateMachine(){
  split_selecting_info_.split_selecting_status = kNonSelecting;
  split_selecting_info_.split_select_direction = SplitSelectingNone;
  split_selecting_info_.unfinished_reason = SplitSelectingInfo::NONE_REASON;
}
}