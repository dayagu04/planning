#include "stop_destination_decider.h"

#include <algorithm>
#include <math.h>

#include "debug_info_log.h"
#include "define/geometry.h"
#include "environmental_model.h"
#include "hpp_route_target_info_util.h"
#include "planning_context.h"
#include "reference_path.h"
#include "task.h"

namespace planning {
namespace {

bool CalculateHppDestinationPoint(framework::Session *session,
                                  const std::shared_ptr<ReferencePath> &reference_path,
                                  double hpp_stop_distance_to_destination,
                                  planning_math::PathPoint *destination_point) {
  if (session == nullptr || reference_path == nullptr || destination_point == nullptr) {
    return false;
  }
  const auto &route_info_output =
      session->environmental_model().get_route_info()->get_route_info_output();
  const auto &planning_init_point = reference_path->get_frenet_ego_state().planning_init_point();
  double distance_to_destination =
      route_info_output.hpp_route_info_output.distance_to_target_dest;
  const auto &parking_slot_manager =
      session->environmental_model().get_parking_slot_manager();
  const auto &frenet_coord = reference_path->get_frenet_coord();
  const auto &current_state = session->environmental_model()
                                  .get_local_view()
                                  .function_state_machine_info.current_state;
  // Keep old HPP destination logic for stop point.
  double stop_distance_to_destination = hpp_stop_distance_to_destination;

  if (current_state == iflyauto::FunctionalState_HPP_CRUISE_ROUTING) {
    if (parking_slot_manager->IsExistTargetSlot()) {
      const auto &target_slot_center = parking_slot_manager->GetTargetSlotCenter();
      Point2D frenet_point;
      if (frenet_coord != nullptr &&
          frenet_coord->XYToSL(
              Point2D(target_slot_center.x(), target_slot_center.y()),
              frenet_point)) {
        distance_to_destination =
            frenet_point.x - planning_init_point.frenet_state.s;
      }
    }
  } else if (current_state == iflyauto::FunctionalState_HPP_CRUISE_SEARCHING) {
    bool find_slot = parking_slot_manager->CalculateDistanceToNearestSlot(
        reference_path);
    if (find_slot) {
      stop_distance_to_destination = 0.0;
      distance_to_destination = parking_slot_manager->GetDistanceToNearestSlot();
    }
  }

  const double destination_s =
      planning_init_point.frenet_state.s +
      std::max((distance_to_destination - stop_distance_to_destination), 0.0);
  ReferencePathPoint destination_ref_point;
  if (!reference_path->get_reference_point_by_lon(destination_s, destination_ref_point)) {
    return false;
  }
  *destination_point = destination_ref_point.path_point;
  return true;
}

}  // namespace

StopDestinationDecider::StopDestinationDecider(
    const EgoPlanningConfigBuilder *config_builder, framework::Session *session)
    : Task(config_builder, session),
      config_(config_builder->cast<StopDestinationDeciderConfig>()),
      hpp_stop_distance_to_destination_(
          config_builder->cast<LongitudinalDeciderV3Config>()
              .stop_distance_to_destination),
      stop_destination_virtual_agent_time_headway_(
          config_.stop_destination_virtual_agent_time_headway) {
  name_ = "StopDestinationDecider";
}

bool StopDestinationDecider::Execute() {
  if (!PreCheck()) {
    ILOG_ERROR << "PreCheck failed";
    return false;
  }
  const auto function_mode =
      session_->environmental_model().function_info().function_mode();
  if (function_mode == common::DrivingFunctionInfo::RADS ||
      session_->is_hpp_scene()) {
    StopDestinationProcess();
  }
  return true;
}

void StopDestinationDecider::StopDestinationProcess() {
  const auto &virtual_lane_manager =
      session_->environmental_model().get_virtual_lane_manager();
  current_lane_ = virtual_lane_manager->get_current_lane();

  // HPP：先统一按当前 reference path 更新 distance_to_target_dest/slot，
  // 再计算虚拟终点位置，供本 decider 与后续 StartStopDecider 共用
  if (session_->is_hpp_scene() && current_lane_ != nullptr) {
    const auto &current_reference_path = current_lane_->get_reference_path();
    if (current_reference_path != nullptr) {
      UpdateHppRouteTargetInfoFromReferencePath(session_, current_reference_path);
    }
  }

  AddVirtualObstacle();
  SaveToSession();
}

// set virtual obstacle at the end of the lane
bool StopDestinationDecider::AddVirtualObstacle() {
  if (current_lane_ == nullptr) {
    stop_destination_virtual_agent_id_ = agent::AgentDefaultInfo::kNoAgentId;
    return false;
  }
  const auto &current_reference_path =
      current_lane_->get_reference_path();
  agent::Agent virtual_agent;
  if (current_reference_path == nullptr) {
    virtual_agent.set_agent_id(agent::AgentDefaultInfo::kNoAgentId);
    return false;
  }
  
  const auto& current_raw_end_point =
      current_reference_path->GetRawEndRefPathPoint();
  static double stop_destination_extended_s_buffer =
      config_.stop_destination_extended_s_buffer;
  // RADS（倒车）和HPP（前进）分别使用独立的虚拟障碍物ID，便于区分业务场景
  if (session_->is_hpp_scene()) {
    stop_destination_virtual_agent_id_ =
        agent::AgentDefaultInfo::kHppStopDestinationVirtualAgentId;
  } else {
    stop_destination_virtual_agent_id_ =
        agent::AgentDefaultInfo::kRadsStopDestinationVirtualAgentId;
  }
  virtual_agent.set_agent_id(stop_destination_virtual_agent_id_);
  virtual_agent.set_type(agent::AgentType::VIRTUAL);
  virtual_agent.set_is_tfl_virtual_obs(false);
  virtual_agent.set_is_stop_destination_virtual_obs(true);
  planning_math::PathPoint stop_point = current_raw_end_point.path_point;
  if (session_->is_hpp_scene()) {
    // HPP终点位置使用原HPP停车逻辑（参考general_longitudinal_decider的termination逻辑）
    if (!CalculateHppDestinationPoint(session_, current_reference_path,
                                      hpp_stop_distance_to_destination_,
                                      &stop_point)) {
      ILOG_WARN << "StopDestinationDecider: fallback to raw end point for HPP";
      stop_destination_virtual_agent_id_ = agent::AgentDefaultInfo::kNoAgentId;
      return false;
    }
  }
  // 终点虚拟障碍物沿路径航向延长（HPP/非HPP统一处理）
  stop_point.set_x(stop_point.x() + stop_destination_extended_s_buffer *
                                      cos(stop_point.theta()));
  stop_point.set_y(stop_point.y() + stop_destination_extended_s_buffer *
                                      sin(stop_point.theta()));
  virtual_agent.set_x(stop_point.x());
  virtual_agent.set_y(stop_point.y());
  virtual_agent.set_length(0.5);
  virtual_agent.set_width(2.0);
  virtual_agent.set_fusion_source(1);
  virtual_agent.set_is_static(true);
  virtual_agent.set_speed(0.0);
  virtual_agent.set_theta(stop_point.theta());
  virtual_agent.set_accel(0.0);
  virtual_agent.set_time_range({0.0, 5.0});

  planning::planning_math::Box2d box(
      planning::planning_math::Vec2d(virtual_agent.x(), virtual_agent.y()),
      virtual_agent.theta(), virtual_agent.length(), virtual_agent.width());
  virtual_agent.set_box(box);
  virtual_agent.set_timestamp_s(0.0);
  virtual_agent.set_timestamp_us(0.0);
  auto *agent_manager = session_->environmental_model()
                            .get_dynamic_world()
                            ->mutable_agent_manager();
  std::unordered_map<int32_t, planning::agent::Agent> agent_table;
  agent_table.insert({virtual_agent.agent_id(), virtual_agent});
  agent_manager->Append(agent_table);
  JSON_DEBUG_VALUE("stop_destination_virtual_agent_pos_x", virtual_agent.x())
  JSON_DEBUG_VALUE("stop_destination_virtual_agent_pos_y", virtual_agent.y())
  JSON_DEBUG_VALUE("stop_destination_virtual_agent_theta",
                   virtual_agent.theta())
  JSON_DEBUG_VALUE("stop_destination_virtual_agent_id",
                   virtual_agent.agent_id())
  JSON_DEBUG_VALUE("stop_destination_virtual_agent_width",
                   virtual_agent.width())
  JSON_DEBUG_VALUE("stop_destination_virtual_agent_length",
                   virtual_agent.length())
  return true;
}

void StopDestinationDecider::SaveToSession() {
  auto &mutable_stop_destination_decider_output =
      session_->mutable_planning_context()
          ->mutable_stop_destination_decider_output();
  mutable_stop_destination_decider_output
      .mutable_stop_destination_virtual_agent_id() =
      stop_destination_virtual_agent_id_;
  mutable_stop_destination_decider_output
      .mutable_stop_destination_virtual_agent_time_headway() =
      stop_destination_virtual_agent_time_headway_;
}

}  // namespace planning
