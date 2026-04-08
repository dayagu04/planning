#include "start_stop_decider.h"

#include <cmath>

#include "agent/agent.h"
#include "basic_types.pb.h"
#include "behavior_planners/start_stop_decider/start_stop_decider_output.h"
#include "debug_info_log.h"
#include "ego_planning_config.h"
#include "environmental_model.h"
#include "hpp_route_target_info_util.h"
#include "parking_slot_manager.h"
#include "planning_context.h"
#include "reference_path.h"
#include "route_info.h"

namespace planning {

StartStopDecider::StartStopDecider(
    const EgoPlanningConfigBuilder* config_builder, framework::Session* session)
    : Task(config_builder, session),
      config_(config_builder->cast<StartStopDeciderConfig>()),
      hpp_stop_config_(config_builder->cast<HppStopDeciderConfig>()),
      cipv_relative_s_prev_(0.0) {
  name_ = "StartStopDecider";
  cipv_relative_s_ = 0.0;
}

bool StartStopDecider::Execute() {
  if (!PreCheck()) {
    ILOG_DEBUG << "PreCheck failed";
    return false;
  }
  rads_scene_is_completed_ = false;
  UpdateInput();
  stop_distance_ = CalculateStopDistance();

  // HPP 预判断：提前计算本帧是否满足停车到位条件（含控制超调），供 CanTransitionToStop 使用
  hpp_stop_condition_met_this_frame_ = false;
  if (session_->is_hpp_scene()) {
    double dist_to_dest = 0.0;
    double dist_to_slot = 0.0;
    if (UpdateHppTargetInfo(&dist_to_dest, &dist_to_slot)) {
      const double ego_actual_vel =
          session_->environmental_model().get_ego_state_manager()->ego_v();
      hpp_stop_condition_met_this_frame_ =
          IsHppStopConditionMet(dist_to_dest, dist_to_slot, ego_actual_vel);
    }
  }

  UpdateStartStopStatus();

  const auto& agent_manager =
      session_->environmental_model().get_agent_manager();
  const auto& cipv = agent_manager->GetAgent(cipv_id_);

  // RADS（倒车）场景：通过CIPV为终点虚拟障碍物且足够近来判断完成
  const auto cipv_is_rads_destination_target =
      cipv && cipv->type() == agent::AgentType::VIRTUAL &&
      cipv->agent_id() ==
          agent::AgentDefaultInfo::kRadsStopDestinationVirtualAgentId;

  if (session_->get_scene_type() == common::SceneType::RADS &&
      ego_start_stop_info_.state() == common::StartStopInfo::STOP &&
      cipv_is_rads_destination_target &&
      cipv_relative_s_ < config_.stop_destination_to_ego_distance) {
    rads_scene_is_completed_ = true;
  }

  // HPP（前进记忆泊车）场景：使用dist_to_dest+dist_to_slot+velocity多条件判断
  if (session_->is_hpp_scene()) {
    ExecuteHppStopLogic();
  }

  SaveToSession();
  return true;
}

void StartStopDecider::UpdateInput() {
  const auto& environmental_model = session_->environmental_model();
  const auto& cipv_decider_output =
      session_->planning_context().cipv_decider_output();

  if (ego_start_stop_info_.state() != common::StartStopInfo::STOP) {
    cipv_relative_s_prev_ = cipv_relative_s_;
  }

  // cipv info
  cipv_id_ = cipv_decider_output.cipv_id();
  cipv_relative_s_ = cipv_decider_output.relative_s();
  cipv_vel_frenet_ = cipv_decider_output.v_frenet();
  cipv_is_large_ = cipv_decider_output.is_large();

  // ego state info
  planning_init_state_vel_ = environmental_model.get_ego_state_manager()
                                 ->ego_v();
  const auto& cur_start_stop_result =
      session_->planning_context().start_stop_result();
  ego_start_stop_info_.CopyFrom(cur_start_stop_result);
  // is ego reverse
  is_ego_reverse_ = session_->is_rads_scene();

  stand_wait_ =
      environmental_model.get_ego_state_manager()->has_stand_wait_request();
}

void StartStopDecider::UpdateStartStopStatus() {
  auto current_state = ego_start_stop_info_.state();

  switch (current_state) {
    case common::StartStopInfo::STOP:
      if (CanTransitionFromStopToStart()) {
        ego_start_stop_info_.set_state(common::StartStopInfo::START);
      }
      break;

    case common::StartStopInfo::START:
      if (CanTransitionFromStartToCruise()) {
        ego_start_stop_info_.set_state(common::StartStopInfo::CRUISE);
      } else if (CanTransitionToStop()) {
        ego_start_stop_info_.set_state(common::StartStopInfo::STOP);
        cipv_relative_s_prev_ = cipv_relative_s_;
      }
      break;

    case common::StartStopInfo::CRUISE:
      if (CanTransitionToStop()) {
        ego_start_stop_info_.set_state(common::StartStopInfo::STOP);
        cipv_relative_s_prev_ = cipv_relative_s_;
      }
      break;

    default:
      break;
  }

  JSON_DEBUG_VALUE("start_stop_status",
                   static_cast<int>(ego_start_stop_info_.state()))
  JSON_DEBUG_VALUE("cipv_relative_s", cipv_relative_s_)
  JSON_DEBUG_VALUE("cipv_relative_s_prev", cipv_relative_s_prev_)
  JSON_DEBUG_VALUE("cipv_vel_frenet", cipv_vel_frenet_)
  JSON_DEBUG_VALUE("cipv_stop_distance", stop_distance_)
  JSON_DEBUG_VALUE("stand_wait", stand_wait_)
}

double StartStopDecider::CalculateStopDistance() {
  double base_min_follow_distance = 3.5;
  if (cipv_id_ == -1) {
    return base_min_follow_distance;
  }

  const auto agent_manager =
      session_->environmental_model().get_agent_manager();
  const auto* agent = agent_manager->GetAgent(cipv_id_);
  if (!agent) {
    return base_min_follow_distance;
  }

  double low_speed_min_follow_distance_gap =
      cipv_is_large_ ? config_.lower_speed_large_vehicle_min_follow_distance_gap
                     : config_.lower_speed_min_follow_distance_gap;

  double high_speed_min_follow_distance_gap =
      config_.high_speed_min_follow_distance_gap;

  if (agent->type() == agent::AgentType::TRAFFIC_CONE) {
    high_speed_min_follow_distance_gap = config_.cone_min_follow_distance_gap;
  }

  if (agent->is_tfl_virtual_obs()) {
    high_speed_min_follow_distance_gap =
        config_.traffic_light_min_follow_distance_gap;
  }

  const double low_speed_threshold = config_.low_speed_threshold_kmph / 3.6;
  const double high_speed_threshold = config_.high_speed_threshold_kmph / 3.6;

  double final_distance = planning_math::LerpWithLimit(
      low_speed_min_follow_distance_gap, low_speed_threshold,
      high_speed_min_follow_distance_gap, high_speed_threshold,
      planning_init_state_vel_);

  return final_distance;
}

bool StartStopDecider::CanTransitionFromStopToStart() {
  // HPP 终点停车后不允许误起步：当 CIPV 为 HPP 终点虚拟障碍物时保持 STOP。
  if (session_->is_hpp_scene() &&
      IsCipvVirtualDestination(
          agent::AgentDefaultInfo::kHppStopDestinationVirtualAgentId)) {
    return false;
  }

  const double cipv_movement_distance =
      cipv_relative_s_ - cipv_relative_s_prev_;
  const double distance_start_between_ego_and_cipv_threshold =
      cipv_is_large_
          ? config_.distance_start_between_ego_and_large_cipv_threshold
          : config_.distance_start_between_ego_and_cipv_threshold;

  const bool cipv_moved_enough =
      cipv_movement_distance > distance_start_between_ego_and_cipv_threshold;

  const bool cipv_start_condition =
      cipv_vel_frenet_ > config_.cipv_vel_begin_start_threshold &&
      (cipv_relative_s_ >
       stop_distance_ + distance_start_between_ego_and_cipv_threshold) &&
      cipv_moved_enough;

  const bool cipv_is_static =
      std::fabs(cipv_vel_frenet_) < config_.cipv_static_vel_threshold;

  double distance_to_go_threshold =
      cipv_is_large_ ? config_.distance_to_go_threshold_behind_of_large_vehicle
                     : config_.distance_to_go_threshold;
  if (session_->get_scene_type() == common::SceneType::RADS) {
    const auto& agent_manager =
           session_->environmental_model().get_agent_manager();
    const auto& cipv = agent_manager->GetAgent(cipv_id_);
    const auto cipv_is_destination_target =
        cipv && cipv->type() == agent::AgentType::VIRTUAL &&
        cipv->agent_id() ==
            agent::AgentDefaultInfo::kRadsStopDestinationVirtualAgentId;
    if (cipv_is_destination_target) {
      distance_to_go_threshold = config_.rads_distance_stop_between_ego_and_destination_cipv_threshold;
    } else if (cipv && cipv->type() != agent::AgentType::VIRTUAL) {
      distance_to_go_threshold = config_.rads_distance_stop_between_ego_and_cipv_threshold;
    }
  }

  const bool is_distance_enough = cipv_relative_s_ > distance_to_go_threshold;

  const bool cipv_distance_condition = cipv_is_static && is_distance_enough;

  return (cipv_start_condition || cipv_distance_condition) && !stand_wait_;
}

bool StartStopDecider::CanTransitionFromStartToCruise() {
  return planning_init_state_vel_ > config_.start_to_cruise_vel_threshold;
}

bool StartStopDecider::IsCipvVirtualDestination(int expected_agent_id) const {
  const auto& agent_manager =
      session_->environmental_model().get_agent_manager();
  const auto* cipv = agent_manager->GetAgent(cipv_id_);
  return cipv && cipv->type() == agent::AgentType::VIRTUAL &&
         cipv->agent_id() == expected_agent_id;
}

bool StartStopDecider::CanTransitionToStop() {
  // HPP 场景下仅使用 HPP 终点停车条件，屏蔽普通 CIPV 停车条件
  if (session_->is_hpp_scene()) {
    return hpp_stop_condition_met_this_frame_;
  }

  bool ego_stop_condition =
      planning_init_state_vel_ < config_.ego_vel_begin_stop_threshold;
  const bool cipv_static_condition =
      std::fabs(cipv_vel_frenet_) < config_.cipv_static_vel_threshold;
  bool cipv_distance_condition =
      cipv_relative_s_ <
      stop_distance_ + config_.distance_stop_between_ego_and_cipv_threshold;

  if (session_->get_scene_type() == common::SceneType::RADS) {
    const auto &ego_state_mgr =
          session_->environmental_model().get_ego_state_manager();
    const double ego_real_v = ego_state_mgr->ego_v();
    ego_stop_condition = ego_stop_condition && (std::fabs(ego_real_v) < config_.ego_vel_begin_stop_threshold);
    const auto& agent_manager =
           session_->environmental_model().get_agent_manager();
    const auto& cipv = agent_manager->GetAgent(cipv_id_);
    const auto cipv_is_destination_target =
        cipv && cipv->type() == agent::AgentType::VIRTUAL &&
        cipv->agent_id() ==
            agent::AgentDefaultInfo::kRadsStopDestinationVirtualAgentId;
    if (cipv_is_destination_target && cipv_relative_s_ < config_.rads_distance_stop_between_ego_and_destination_cipv_threshold) {
      cipv_distance_condition = true;
    } else if (cipv && cipv->type() != agent::AgentType::VIRTUAL && cipv_relative_s_ <
               config_.rads_distance_stop_between_ego_and_cipv_threshold) {
      cipv_distance_condition = true;
    } else {
      cipv_distance_condition = false;
    }
    if (ego_stop_condition && cipv_static_condition && cipv_distance_condition) {
      return true;
    } else {
      const auto &virtual_lane_manager =
          session_->environmental_model().get_virtual_lane_manager();
      const auto ego_pose = ego_state_mgr->ego_pose();
      const auto& planned_kd_path =
              session_->planning_context().motion_planner_output().lateral_path_coord;
      const auto current_lane = virtual_lane_manager->get_current_lane();
      const auto &current_reference_path = current_lane->get_reference_path();
      const auto& current_raw_end_point =
          current_reference_path->GetRawEndRefPathPoint();
      double end_pnt_s = 0.0;
      double end_pnt_l = 0.0;
      planned_kd_path->XYToSL(current_raw_end_point.path_point.x(), current_raw_end_point.path_point.y(), &end_pnt_s,
                      &end_pnt_l);

      double ego_pose_s = 0.0;
      double ego_pose_l = 0.0;
      planned_kd_path->XYToSL(ego_pose.x, ego_pose.y, &ego_pose_s,
                      &ego_pose_l);

      double dis_ego_to_end = std::fabs(end_pnt_s - ego_pose_s);
      if (dis_ego_to_end < config_.rads_early_stop_distance_ego_to_end_threshold &&
          std::fabs(ego_real_v) < config_.rads_early_stop_vel_threshold &&
          planning_init_state_vel_ < config_.rads_early_stop_vel_threshold) {
        return true;
      } else {
        return false;
      }
    }

  }

  return ego_stop_condition && cipv_static_condition && cipv_distance_condition;
}

void StartStopDecider::SaveToSession() {
  auto& start_stop_decider_output =
      session_->mutable_planning_context()->mutable_start_stop_decider_output();
  start_stop_decider_output.mutable_ego_start_stop_info() =
      ego_start_stop_info_;
  start_stop_decider_output.mutable_rads_scene_is_completed() =
      rads_scene_is_completed_;

  if (session_->get_scene_type() == common::SceneType::RADS) {
    auto& mutable_rads_planning_completed =
        session_->mutable_planning_context()->mutable_rads_planning_completed();
    mutable_rads_planning_completed = rads_scene_is_completed_;
  }

  auto& start_stop_state_result =
      session_->mutable_planning_context()->mutable_start_stop_result();
  start_stop_state_result.CopyFrom(ego_start_stop_info_);
}

// ===================== HPP专用停车逻辑（整合自HppStopDecider）=====================

bool StartStopDecider::ValidateHppPreconditions() {
  const auto& current_reference_path =
      session_->environmental_model()
          .get_reference_path_manager()
          ->get_reference_path_by_current_lane();
  if (current_reference_path == nullptr || !current_reference_path->valid()) {
    ILOG_WARN << "StartStopDecider HPP: invalid reference path";
    return false;
  }
  return true;
}

StartStopDecider::HppStopConditions StartStopDecider::EvaluateHppStopConditions() {
  HppStopConditions conditions;

  if (!UpdateHppTargetInfo(&conditions.dist_to_dest, &conditions.dist_to_slot)) {
    ILOG_WARN << "StartStopDecider HPP: failed to update target info";
    return conditions;
  }

  const auto& env = session_->environmental_model();
  conditions.ego_velocity = env.get_ego_state_manager()->ego_v();

  // 判断是否到达目标停车位和目的地
  const auto& parking_slot_manager = env.get_parking_slot_manager();
  const bool is_exist_target_slot = parking_slot_manager->IsExistTargetSlot();
  conditions.is_reached_target_slot =
      is_exist_target_slot &&
      conditions.dist_to_slot < hpp_stop_config_.dist_to_target_slot_thr;
  conditions.is_reached_target_dest =
      conditions.dist_to_dest < hpp_stop_config_.dist_to_target_dest_thr;

  // 停车到位条件：当前为停车状态且 CIPV 为虚拟终点障碍物（多帧保持）
  const bool is_in_stop_state =
      (ego_start_stop_info_.state() == common::StartStopInfo::STOP);
  const bool cipv_is_hpp_destination =
      IsCipvVirtualDestination(
          agent::AgentDefaultInfo::kHppStopDestinationVirtualAgentId);
  conditions.is_stop_condition_met =
      is_in_stop_state && cipv_is_hpp_destination;

  return conditions;
}

bool StartStopDecider::UpdateHppStopFrameCount(bool is_stop_condition_met) {
  constexpr int kMaxStopFrameCount = 1000;
  bool is_stopped_at_destination = false;

  if (is_stop_condition_met) {
    if (last_frame_hpp_stop_condition_met_) {
      hpp_stop_frame_count_ = std::min(hpp_stop_frame_count_ + 1, kMaxStopFrameCount);
    } else {
      hpp_stop_frame_count_ = 1;
    }
    is_stopped_at_destination = true;
  } else {
    hpp_stop_frame_count_ = 0;
    is_stopped_at_destination = false;
  }
  last_frame_hpp_stop_condition_met_ = is_stop_condition_met;

  return is_stopped_at_destination;
}

void StartStopDecider::SaveHppStopOutput(const HppStopConditions& conditions,
                                         bool is_stopped_at_destination) {
  // 在判断进入 STOP 逻辑后，再更新 session 相关逻辑（避免未进入 STOP 时写下游）
  if (ego_start_stop_info_.state() != common::StartStopInfo::STOP) {
    session_->mutable_planning_context()->mutable_hpp_stop_decider_output().Clear();
    return;
  }

  session_->mutable_environmental_model()
      ->get_parking_slot_manager()
      ->SetIsReachedTarget(conditions.is_reached_target_slot,
                          conditions.is_reached_target_dest);

  auto& hpp_stop_output =
      session_->mutable_planning_context()->mutable_hpp_stop_decider_output();
  hpp_stop_output.is_stopped_at_destination = is_stopped_at_destination;
  hpp_stop_output.is_reached_target_slot = conditions.is_reached_target_slot;
  hpp_stop_output.is_reached_target_dest = conditions.is_reached_target_dest;
  hpp_stop_output.is_stop_condition_met = conditions.is_stop_condition_met;
  hpp_stop_output.stop_frame_count = hpp_stop_frame_count_;
}

void StartStopDecider::ExecuteHppStopLogic() {
  if (!ValidateHppPreconditions()) {
    return;
  }

  const auto conditions = EvaluateHppStopConditions();
  const bool is_stopped_at_destination =
      UpdateHppStopFrameCount(conditions.is_stop_condition_met);

  ILOG_DEBUG << "StartStopDecider HPP: dist_to_dest=" << conditions.dist_to_dest
             << ", dist_to_slot=" << conditions.dist_to_slot
             << ", ego_vel=" << conditions.ego_velocity
             << ", is_stop_condition_met=" << conditions.is_stop_condition_met
             << ", stop_frame_count=" << hpp_stop_frame_count_;

  JSON_DEBUG_VALUE("hpp_dist_to_target_dest", conditions.dist_to_dest)
  JSON_DEBUG_VALUE("hpp_dist_to_target_slot", conditions.dist_to_slot)
  JSON_DEBUG_VALUE("hpp_is_stop_condition_met", conditions.is_stop_condition_met)
  JSON_DEBUG_VALUE("hpp_stop_frame_count", hpp_stop_frame_count_)

  SaveHppStopOutput(conditions, is_stopped_at_destination);
}

bool StartStopDecider::IsHppStopConditionMet(double dist_to_dest,
                                              double dist_to_slot,
                                              double ego_velocity) {
  // 条件1：距离目标目的地的纵向距离 < 阈值
  const bool cond_dist_dest = dist_to_dest < hpp_stop_config_.dist_to_stop_dest_thr;
  // 条件2：距离目标停车位的纵向距离 < 阈值
  const bool cond_dist_slot = dist_to_slot < hpp_stop_config_.dist_to_stop_slot_thr;
  // 条件3：自车处于静止（速度 < 阈值）
  const bool cond_velocity = ego_velocity < hpp_stop_config_.ego_still_velocity_thr;
  return cond_dist_dest && cond_dist_slot && cond_velocity;
}

bool StartStopDecider::UpdateHppTargetInfo(double* dist_to_dest,
                                           double* dist_to_slot) {
  if (!dist_to_dest || !dist_to_slot) return false;

  // 使用与 StopDestinationDecider 统一的更新逻辑（已在 pipeline 前段执行一次），
  // 此处再执行一次保证本帧读取到的是基于当前 reference path 的数值
  const auto& current_reference_path =
      session_->environmental_model()
          .get_reference_path_manager()
          ->get_reference_path_by_current_lane();
  if (current_reference_path != nullptr) {
    UpdateHppRouteTargetInfoFromReferencePath(session_, current_reference_path);
  }

  const auto& route_info_output =
      session_->environmental_model().get_route_info()->get_route_info_output();
  *dist_to_dest = route_info_output.hpp_route_info_output.distance_to_target_dest;
  *dist_to_slot = route_info_output.hpp_route_info_output.distance_to_target_slot;
  return true;
}

}  // namespace planning