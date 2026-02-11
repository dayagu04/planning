#include "start_stop_decider.h"

#include <cmath>

#include "agent/agent.h"
#include "basic_types.pb.h"
#include "behavior_planners/start_stop_decider/start_stop_decider_output.h"
#include "debug_info_log.h"
#include "ego_planning_config.h"
#include "environmental_model.h"
#include "planning_context.h"

namespace planning {

StartStopDecider::StartStopDecider(
    const EgoPlanningConfigBuilder* config_builder, framework::Session* session)
    : Task(config_builder, session),
      config_(config_builder->cast<StartStopDeciderConfig>()),
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
  UpdateStartStopStatus();

  const auto& agent_manager =
      session_->environmental_model().get_agent_manager();
  const auto& cipv = agent_manager->GetAgent(cipv_id_);

  const auto cipv_is_destination_target =
      cipv && cipv->type() == agent::AgentType::VIRTUAL &&
      cipv->agent_id() ==
          agent::AgentDefaultInfo::kRadsStopDestinationVirtualAgentId;

  if (session_->get_scene_type() == common::SceneType::RADS &&
      ego_start_stop_info_.state() == common::StartStopInfo::STOP &&
      cipv_is_destination_target &&
      cipv_relative_s_ <
          config_.stop_destination_to_ego_distance) {
    rads_scene_is_completed_ = true;
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
                                 ->planning_init_point()
                                 .lon_init_state.v();
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

  const bool is_distance_enough = cipv_relative_s_ > distance_to_go_threshold;

  const bool cipv_distance_condition = cipv_is_static && is_distance_enough;

  return (cipv_start_condition || cipv_distance_condition) && !stand_wait_;
}

bool StartStopDecider::CanTransitionFromStartToCruise() {
  return planning_init_state_vel_ > config_.start_to_cruise_vel_threshold;
}

bool StartStopDecider::CanTransitionToStop() {
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
               config_.rads_distance_stop_between_ego_and_destination_cipv_threshold) {
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

}  // namespace planning