#include "traffic_light_decider.h"
#include "agent/agent_decision.h"
#include "agent/agent_manager.h"
#include "environmental_model.h"
#include "lateral_obstacle.h"

namespace planning {

TrafficLightDecider::TrafficLightDecider(
    const EgoPlanningConfigBuilder *config_builder, framework::Session *session)
    : Task(config_builder, session) {
  config_ = config_builder->cast<TrafficLightDeciderConfig>();
  name_ = "TrafficLightDecider";
}

bool TrafficLightDecider::Execute() {
  const auto &environmental_model = session_->environmental_model();
  double dis_to_stopline = environmental_model.get_virtual_lane_manager()
                               ->GetEgoDistanceToStopline();
  double dis_to_crosswalk = environmental_model.get_virtual_lane_manager()
                                ->GetEgoDistanceToCrosswalk();

  const auto ego_state_mgr = environmental_model.get_ego_state_manager();
  double v_ego = ego_state_mgr->ego_v();
  is_small_front_intersection_ = false;

  planning::common::IntersectionState intersection_state =
      environmental_model.get_virtual_lane_manager()->GetIntersectionState();

  const auto lateral_obstacles = environmental_model.get_lateral_obstacle();
  if (lateral_obstacles->leadone() != nullptr &&
      lateral_obstacles->leadone()->d_rel + 4.0 < dis_to_stopline) {
    is_first_car_ = false;
  } else {
    is_first_car_ = true;
  }
  const auto tfl_manager =
      environmental_model.get_traffic_light_decision_manager();
  const auto traffic_status = tfl_manager->GetTrafficStatus();
  if (config_.enable_tfl_decider &&
      (dis_to_stopline > 0.5 && dis_to_crosswalk > 2) &&
      (intersection_state != planning::common::IN_INTERSECTION ||
       (intersection_state == planning::common::IN_INTERSECTION &&
        !can_pass_))) {
    if (traffic_status.go_straight == 1 || traffic_status.go_straight == 41 ||
        traffic_status.go_straight == 11 || traffic_status.go_straight == 10) {
      // red light or(==) red blink
      green_light_timer_ = 0.0;
      yellow_light_timer_ = 0.0;
      green_blink_timer_ = 0.0;
      if (can_pass_ && dis_to_stopline > 100.0) {
        can_pass_ = true;
      } else {
        if (dis_to_stopline < 100.0 && IsSmallFrontIntersection() &&
            !IsIntersectionMatchTFL()) {
          can_pass_ = true;
        } else {
          can_pass_ = false;
        }
      }

    } else if (traffic_status.go_straight == 3 ||
               traffic_status.go_straight == 43) {
      // green light
      green_light_timer_ += 0.1;
      yellow_light_timer_ = 0.0;
      green_blink_timer_ = 0.0;
      can_pass_ = true;

    } else if (traffic_status.go_straight == 2 ||
               traffic_status.go_straight == 42) {
      // yellow light
      if (dis_to_stopline > 100.0) {
        can_pass_ = true;
      } else {
        if (IsSmallFrontIntersection() && !IsIntersectionMatchTFL()) {
          can_pass_ = true;
        } else if (can_pass_ && (std::max(v_ego - 1.0, 0.0) *
                  std::max(2.0 - yellow_light_timer_, 0.0) > dis_to_stopline)) {
          can_pass_ = true;
        } else {
          can_pass_ = false;
        }
      }
      green_light_timer_ = 0.0;
      yellow_light_timer_ += 0.1;
      green_blink_timer_ = 0.0;

    } else if (traffic_status.go_straight == 30 ||
               traffic_status.go_straight == 32 ||
               traffic_status.go_straight == 33) {
      // green blink
      if (dis_to_stopline > 100.0) {
        can_pass_ = true;
      } else {
        if (can_pass_ && (std::max(v_ego - 1.0, 0.0) *
                              std::max(4.0 - green_blink_timer_, 0.0) >
                          dis_to_stopline)) {
          can_pass_ = true;
        } else {
          can_pass_ = false;
        }
      }

      green_light_timer_ = 0.0;
      yellow_light_timer_ = 0.0;
      green_blink_timer_ += 0.1;

    } else if (traffic_status.go_straight == 20 ||
               traffic_status.go_straight == 22) {
      // yellow blink in intersection, can pass
      green_light_timer_ = 0.0;
      yellow_light_timer_ = 0.0;
      green_blink_timer_ = 0.0;
      can_pass_ = true;

    } else {
      // others, can go
      green_light_timer_ = 0.0;
      yellow_light_timer_ = 0.0;
      green_blink_timer_ = 0.0;
      if (dis_to_stopline > 200.0 || !is_first_car_) {
        can_pass_ = true;
      }
      // can_pass_ = true;
    }
  } else if (config_.enable_tfl_decider &&
             (dis_to_stopline <= 0.5 || dis_to_crosswalk <= 2) &&
             (intersection_state == planning::common::IN_INTERSECTION &&
              !can_pass_)) {
    //一般是刹停在路口中，这是看到绿灯就设置can_pass_ = true
    if (traffic_status.go_straight == 3 || traffic_status.go_straight == 43) {
      // green light
      green_light_timer_ += 0.1;
      yellow_light_timer_ = 0.0;
      green_blink_timer_ = 0.0;
      can_pass_ = true;

    } else if (traffic_status.go_straight == 20 || traffic_status.go_straight == 22) {
      //wrong yellow light brake in intersection then yellow blink, can pass
      green_light_timer_ = 0.0;
      yellow_light_timer_ = 0.0;
      green_blink_timer_ = 0.0;
      can_pass_ = true;
    } else {
      can_pass_ = false;
    }

  } else {
    can_pass_ = true;
  }

  if (!can_pass_) {
    AddVirtualObstacle();
  }
  //此外认为已经进入路口
  /*
  if (dis_to_stopline > 0.5 && dis_to_crosswalk > 2.5) {
    const auto lateral_obstacles = environmental_model.get_lateral_obstacle();
    if (lateral_obstacles->leadone() != nullptr &&
        lateral_obstacles->leadone()->d_rel + 4.0 < dis_to_stopline) {
      is_first_car_ = false;
    } else {
      is_first_car_ = true;
    }
    if (is_first_car_ && config_.enable_tfl_decider) {
      const auto tfl_manager =
          environmental_model.get_traffic_light_decision_manager();
      const auto traffic_status = tfl_manager->GetTrafficStatus();
      if (traffic_status.go_straight == 0) {
        AddVirtualObstacle();
      }
    }
  }
  */
  auto cur_fsm_state = environmental_model.get_local_view()
                          .function_state_machine_info.current_state;
  if (cur_fsm_state == iflyauto::FunctionalState_ACC_STANDBY ||
      cur_fsm_state == iflyauto::FunctionalState_SCC_STANDBY ||
      cur_fsm_state == iflyauto::FunctionalState_NOA_STANDBY) {
    auto &tla_output_info =
        session_->mutable_planning_context()->mutable_planning_hmi_info()->tla_output_info;
    if (IsRunningRedTFL()) {
      tla_output_info.traffic_light_reminder = iflyauto::TrafficLightReminder::TRAFFIC_LIGHT_REMINDER_RED_LIGHT_STOP;
    }
    if (IsStayingStillGreenTFL()) {
      tla_output_info.traffic_light_reminder = iflyauto::TrafficLightReminder::TRAFFIC_LIGHT_REMINDER_GREEN_LIGHT_START;
    }

  }

  auto &tfl_decider = session_->mutable_planning_context()
                          ->mutable_traffic_light_decider_output();
  tfl_decider.can_pass = can_pass_;
  tfl_decider.is_small_front_intersection = is_small_front_intersection_;
  return true;
}

bool TrafficLightDecider::AddVirtualObstacle() {
  planning::agent::Agent virtual_agent;
  virtual_agent.set_agent_id(100000);
  virtual_agent.set_type(agent::AgentType::VIRTUAL);
  virtual_agent.set_is_tfl_virtual_obs(true);

  // note stopline jump
  double dis_to_stopline = session_->environmental_model()
                               .get_virtual_lane_manager()
                               ->GetEgoDistanceToStopline();
  double dis_to_crosswalk = session_->environmental_model()
                                .get_virtual_lane_manager()
                                ->GetEgoDistanceToCrosswalk();

  const auto &reference_path_ptr = session_->planning_context()
                                .lane_change_decider_output()
                                .coarse_planning_info.reference_path;
  if (reference_path_ptr == nullptr) {
    return false;
  }
  const auto &frenet_ego_state = reference_path_ptr->get_frenet_ego_state();
  double ego_start_s = frenet_ego_state.s();
  double virtual_obs_dis = std::min(dis_to_stopline + 3.5, dis_to_crosswalk + 1.5);
  ReferencePathPoint refpath_pt;
  if (reference_path_ptr->get_reference_point_by_lon(
    ego_start_s + virtual_obs_dis, refpath_pt)) {
    virtual_agent.set_x(refpath_pt.path_point.x());  //几何中心
    virtual_agent.set_y(refpath_pt.path_point.y());
  } else {
    return false;
  }

  /* auto &car2enu =
      session_->environmental_model().get_ego_state_manager()->get_car2enu();
  Eigen::Vector3d car_point, enu_point;
  car_point.x() = std::min(dis_to_stopline + 4.0, dis_to_crosswalk + 2.0);
  car_point.y() = 0.0;
  car_point.z() = 0.0;
  enu_point = car2enu * car_point;
  virtual_agent.set_x(enu_point.x());  //几何中心
  virtual_agent.set_y(enu_point.y()); */
  virtual_agent.set_length(5.0);
  virtual_agent.set_width(2.0);
  virtual_agent.set_fusion_source(1);
  virtual_agent.set_is_static(true);

  virtual_agent.set_speed(0.0);
  virtual_agent.set_theta(0.0);
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

  return true;
}

bool TrafficLightDecider::IsSmallFrontIntersection() {
  const auto &environmental_model = session_->environmental_model();
  const auto curr_lane =
      environmental_model.get_virtual_lane_manager()->get_current_lane();
  if (curr_lane == nullptr) {
    return false;
  }
  double ego_pos_x = 0.0;
  if (curr_lane->get_left_lane_boundary().car_points_size > 0 &&
      curr_lane->get_right_lane_boundary().car_points_size > 0) {
    double first_left_x = curr_lane->get_left_lane_boundary().car_points[0].x;
    double first_right_x = curr_lane->get_right_lane_boundary().car_points[0].x;
    ego_pos_x = std::max(0 - first_left_x, 0 - first_right_x);
  } else {
    return false;
  }
  double dis_to_stopline = environmental_model.get_virtual_lane_manager()
                               ->GetEgoDistanceToStopline();
  double judge_virtual_dis =
      ego_pos_x + dis_to_stopline + config_.virtual_dis_before_stopline;
  bool is_virtual_type =
      environmental_model.get_virtual_lane_manager()->IsPosXOnVirtualLaneType(
          judge_virtual_dis);
  is_small_front_intersection_ = !is_virtual_type;
  return !is_virtual_type;
}

bool TrafficLightDecider::IsIntersectionMatchTFL() {
  const auto &environmental_model = session_->environmental_model();
  const auto tfl_manager =
      environmental_model.get_traffic_light_decision_manager();
  const auto all_tfls = tfl_manager->GetTrafficLightsInfo();

  double dis_to_tfl = 10000.0;
  for (int i = 0; i < all_tfls.size(); i++) {
    if (all_tfls[i].traffic_light_x > 0 &&
        all_tfls[i].traffic_light_x < dis_to_tfl) {
      dis_to_tfl = all_tfls[i].traffic_light_x;
    }
  }

  double dis_to_stopline = environmental_model.get_virtual_lane_manager()
                               ->GetEgoDistanceToStopline();
  bool is_match = false;
  if (std::abs(dis_to_tfl - dis_to_stopline) < config_.stopline_tfl_dis_thred) {
    is_match = true;
  }
  return is_match;
}

bool TrafficLightDecider::IsRunningRedTFL() {
  const auto &environmental_model = session_->environmental_model();
  double dis_to_stopline = environmental_model.get_virtual_lane_manager()
                               ->GetEgoDistanceToStopline();
  double dis_to_crosswalk = environmental_model.get_virtual_lane_manager()
                                ->GetEgoDistanceToCrosswalk();

  const auto ego_state_mgr = environmental_model.get_ego_state_manager();
  double v_ego = ego_state_mgr->ego_v();

  double brake_stop_dis = std::max(0.1, std::min(dis_to_stopline - 4.5, dis_to_crosswalk - 6.5));
  double avg_decel = (0.0 - v_ego * v_ego) / (2.0 * brake_stop_dis);
  if (can_pass_ == false && avg_decel < -2.5) {
    return true;
  }
  return false;
}

bool TrafficLightDecider::IsStayingStillGreenTFL() {
  const auto &environmental_model = session_->environmental_model();
  const auto ego_state_mgr = environmental_model.get_ego_state_manager();
  double v_ego = ego_state_mgr->ego_v();
  if (v_ego < 0.1 && is_first_car_ && green_light_timer_ > 1.5) {
    return true;
  }
  return false;
}

}  // namespace planning