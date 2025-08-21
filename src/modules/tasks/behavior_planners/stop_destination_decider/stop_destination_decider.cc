#include "stop_destination_decider.h"

#include <math.h>

#include "debug_info_log.h"
#include "log.h"
#include "planning_context.h"
#include "task.h"

namespace planning {

StopDestinationDecider::StopDestinationDecider(
    const EgoPlanningConfigBuilder *config_builder, framework::Session *session)
    : Task(config_builder, session),
      config_(config_builder->cast<StopDestinationDeciderConfig>()),
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
  if (function_mode == common::DrivingFunctionInfo::RADS) {
    StopDestinationProcess();
  }
  return true;
}

void StopDestinationDecider::StopDestinationProcess() {
  const auto &virtual_lane_manager =
      session_->environmental_model().get_virtual_lane_manager();
  current_lane_ = virtual_lane_manager->get_current_lane();
  AddVirtualObstacle();
  SaveToSession();
}

// set virtual obstacle at the end of the lane
bool StopDestinationDecider::AddVirtualObstacle() {
  const auto &current_lane_kd_path =
      current_lane_->get_reference_path()->get_frenet_coord();
  agent::Agent virtual_agent;
  if (current_lane_kd_path == nullptr) {
    virtual_agent.set_agent_id(agent::AgentDefaultInfo::kNoAgentId);
    return false;
  }
  const auto current_lane_length = current_lane_kd_path->Length();
  const auto current_lane_end_point =
      current_lane_kd_path->GetPathPointByS(current_lane_length);
  static double stop_destination_extended_s_buffer =
      config_.stop_destination_extended_s_buffer;
  stop_destination_virtual_agent_id_ =
      agent::AgentDefaultInfo::kRadsStopDestinationVirtualAgentId;
  virtual_agent.set_agent_id(stop_destination_virtual_agent_id_);
  virtual_agent.set_type(agent::AgentType::VIRTUAL);
  virtual_agent.set_is_tfl_virtual_obs(false);
  virtual_agent.set_is_stop_destination_virtual_obs(true);
  virtual_agent.set_x(current_lane_end_point.x() +
                      stop_destination_extended_s_buffer *
                          cos(current_lane_end_point.theta()));
  virtual_agent.set_y(current_lane_end_point.y() +
                      stop_destination_extended_s_buffer *
                          sin(current_lane_end_point.theta()));
  virtual_agent.set_length(0.5);
  virtual_agent.set_width(2.0);
  virtual_agent.set_fusion_source(1);
  virtual_agent.set_is_static(true);
  virtual_agent.set_speed(0.0);
  virtual_agent.set_theta(current_lane_end_point.theta());
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
