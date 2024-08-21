#include "traffic_light_decider.h"
#include "lateral_obstacle.h"
#include "agent/agent_manager.h"
#include "environmental_model.h"

namespace planning {

TrafficLightDecider::TrafficLightDecider(
    const EgoPlanningConfigBuilder *config_builder, framework::Session *session)
    : Task(config_builder, session) {
  config_ = config_builder->cast<TrafficLightDeciderConfig>();
  name_ = "TrafficLightDecider";

}

bool TrafficLightDecider::Execute(){
  const auto &environmental_model = session_->environmental_model();
  double dis_to_stopline = environmental_model.get_virtual_lane_manager()->GetEgoDistanceToStopline();
  double dis_to_crosswalk = environmental_model.get_virtual_lane_manager()->GetEgoDistanceToCrosswalk();
  //此外认为已经进入路口
  if (dis_to_stopline > 0.5 && dis_to_crosswalk > 2.5) {
    const auto lateral_obstacles = environmental_model.get_lateral_obstacle();
    if (lateral_obstacles->leadone() != nullptr && lateral_obstacles->leadone()->d_rel + 4.0 < dis_to_stopline) {
      is_first_car_ = false;
    } else{
      is_first_car_ = true;
    }
    if (is_first_car_ && config_.enable_tfl_decider) {
      const auto tfl_manager = environmental_model.get_traffic_light_decision_manager();
      const auto traffic_status = tfl_manager->GetTrafficStatus();
      if (traffic_status.go_straight == 0) {
        AddVirtualObstacle();
      }
    }
  }
  return true;

}

bool TrafficLightDecider::AddVirtualObstacle(){
  planning::agent::Agent virtual_agent;
  virtual_agent.set_agent_id(100000);
  virtual_agent.set_type(iflyauto::OBJECT_TYPE_UNKNOWN_IMMOVABLE);
  virtual_agent.set_is_tfl_virtual_obs(true);

  //note stopline jump
  virtual_agent.set_x(10.0);//几何中心
  virtual_agent.set_y(10.0);
  virtual_agent.set_length(5.0);
  virtual_agent.set_width(2.0);

  virtual_agent.set_speed(0.0);
  virtual_agent.set_theta(0.0);
  virtual_agent.set_accel(0.0);
  virtual_agent.set_time_range({0.0, 5.0});

  planning::planning_math::Box2d box(planning::planning_math::Vec2d(virtual_agent.x(), virtual_agent.y()),
                             virtual_agent.theta(), virtual_agent.length(), virtual_agent.width());

  virtual_agent.set_box(box);
  virtual_agent.set_timestamp_s(0.0);
  virtual_agent.set_timestamp_us(0.0);
  auto *agent_manager =
        session_->environmental_model().get_dynamic_world()->mutable_agent_manager();
  std::unordered_map<int32_t, planning::agent::Agent> agent_table;
  agent_table.insert({virtual_agent.agent_id(), virtual_agent});
  agent_manager->Append(agent_table);

  return true;

}
}