#include "modules/tasks/behavior_planners/potential_dangerous_agent_decider/potential_dangerous_agent_decider.h"

#include <algorithm>

#include "common/debug_info_log.h"
#include "common/pose2d.h"
#include "modules/context/environmental_model.h"
#include "modules/context/planning_context.h"
#include "modules/context/reference_path.h"
#include "modules/tasks/behavior_planners/general_lateral_decider/general_lateral_decider_utils.h"

namespace planning {

PotentialDangerousAgentDecider::PotentialDangerousAgentDecider(
    const EgoPlanningConfigBuilder* config_builder, framework::Session* session)
    : Task(config_builder, session), virtual_lane_manager_(nullptr) {
  config_ = config_builder->cast<PotentialDangerousAgentDeciderConfig>();
  name_ = "PotentialDangerousAgentDecider";
}

bool PotentialDangerousAgentDecider::Execute() {
  if (!config_.enable_potential_dangerous_agent_decider) {
    return true;
  }

  PotentialDangerousAgentDeciderOutput*
      potential_dangerous_agent_decider_output =
          session_->mutable_planning_context()
              ->mutable_potential_dangerous_agent_decider_output();
  if (potential_dangerous_agent_decider_output == nullptr) {
    ILOG_DEBUG << "potential_dangerous_agent_decider_output is nullptr";
    return false;
  }
  potential_dangerous_agent_decider_output->dangerous_agent_info.clear();
  auto ego_frenet_state = session_->environmental_model()
                              .get_reference_path_manager()
                              ->get_reference_path_by_current_lane()
                              ->get_frenet_ego_state();
  ego_state_ = {{ego_frenet_state.s(), ego_frenet_state.velocity_s(), 0.0},
                {ego_frenet_state.l(), 0.0, 0.0}};

  potential_dangerous_agent_decider_info_.Clear();

  virtual_lane_manager_ =
      session_->mutable_environmental_model()->mutable_virtual_lane_manager();
  ego_road_right_decision_ =
      session_->planning_context().ego_lane_road_right_decider_output();
  const auto current_lane = virtual_lane_manager_->get_current_lane();
  if (current_lane == nullptr) {
    ILOG_DEBUG << "current_lane is nullptr";
    return true;
  }
  ego_frenet_boundary_ =
      current_lane->get_reference_path()->get_ego_frenet_boundary();
  frenet_obstacles_ = current_lane->get_reference_path()->get_obstacles();

  std::for_each(frenet_obstacles_.begin(), frenet_obstacles_.end(),
                [potential_dangerous_agent_decider_output,
                 this](std::shared_ptr<FrenetObstacle> frenet_obstacle) {
                  return this->EstimateRiskLevel(
                      frenet_obstacle,
                      potential_dangerous_agent_decider_output);
                });

  std::sort(
      potential_dangerous_agent_decider_output->dangerous_agent_info.begin(),
      potential_dangerous_agent_decider_output->dangerous_agent_info.end(),
      [](const DangerousAgentInfo& lhs, const DangerousAgentInfo& rhs) -> bool {
        return lhs.risk_level > rhs.risk_level ||
               (lhs.risk_level == rhs.risk_level &&
                (lhs.lateral_distance * lhs.lateral_distance +
                 lhs.longitudinal_distance * lhs.longitudinal_distance) <
                    (rhs.lateral_distance * rhs.lateral_distance +
                     rhs.longitudinal_distance * rhs.longitudinal_distance));
      });

  LogDebugInfo();

  return true;
}

bool PotentialDangerousAgentDecider::EstimateRiskLevel(
    std::shared_ptr<FrenetObstacle> frenet_obstacle,
    PotentialDangerousAgentDeciderOutput*
        potential_dangerous_agent_decider_output) {
  if (frenet_obstacle == nullptr || frenet_obstacle->is_static() ||
      std::fabs(frenet_obstacle->obstacle()->velocity()) < 0.1) {
    return false;
  }
  if (potential_dangerous_agent_decider_output == nullptr) {
    ILOG_DEBUG << "risk_level is nullptr";
    return false;
  }
  AgentPosType agent_pos_type = AgentPosType::UNKNOWN;
  PotentialDangerousAgentDeciderConfig::RSSModelConfig rss_config;
  PotentialDangerousAgentDeciderConfig::RSSModelConfig reckless_config;
  double lateral_distance = 0.0;
  double longitudinal_distance = 0.0;
  EstimateAgentPosType(frenet_obstacle, &agent_pos_type, &lateral_distance,
                       &longitudinal_distance);

  if (agent_pos_type == AgentPosType::REAR ||
      agent_pos_type == AgentPosType::LEFT_REAR ||
      agent_pos_type == AgentPosType::RIGHT_REAR) {
    return true;
  }
  if (lateral_distance > config_.risk_free_lateral_distance ||
      longitudinal_distance > config_.risk_free_longitudinal_distance) {
    return true;
  }
  iflyauto::ObjectType type = frenet_obstacle->type();
  int config_type = 0;
  if (general_lateral_decider_utils::IsVRU(type)) {
    config_type = 1;
    rss_config = config_.vru_rss_params;
    reckless_config = config_.vru_rss_params_reckless;
  } else if (general_lateral_decider_utils::IsTruck(frenet_obstacle)) {
    rss_config = config_.oversize_vehicle_rss_params;
    reckless_config = config_.oversize_vehicle_rss_params_reckless;
    config_type = 2;
  } else {
    rss_config = config_.normal_size_vehicle_rss_params;
    reckless_config = config_.normal_size_vehicle_rss_params_reckless;
    config_type = 3;
  }
  RssModel::FrenetState obs_state = {
      {frenet_obstacle->frenet_s(), frenet_obstacle->frenet_velocity_s(), 0.0},
      {frenet_obstacle->frenet_l(), frenet_obstacle->frenet_velocity_l(), 0.0}};
  if (std::fabs(obs_state.second[1]) < 0.05) {
    return true;
  }
  double lateral_reckless_distance = std::numeric_limits<double>::infinity();
  double longitudinal_reckless_distance =
      std::numeric_limits<double>::infinity();
  double lateral_moderate_distance = std::numeric_limits<double>::infinity();
  double longitudinal_moderate_distance =
      std::numeric_limits<double>::infinity();
  RssModel::CalculateSafeDistance(obs_state, ego_state_, agent_pos_type,
                                  reckless_config, &lateral_reckless_distance,
                                  &longitudinal_reckless_distance);
  RssModel::CalculateSafeDistance(obs_state, ego_state_, agent_pos_type,
                                  rss_config, &lateral_moderate_distance,
                                  &longitudinal_moderate_distance);

  auto* debug = potential_dangerous_agent_decider_info_.add_potential_dangerous_agent();
  debug->set_s(obs_state.first[0]);
  debug->set_ds(obs_state.first[1]);
  debug->set_l(obs_state.second[0]);
  debug->set_dl(obs_state.second[1]);
  debug->set_agent_type(static_cast<int32>(config_type));
  debug->set_agent_pos_type(static_cast<int32>(agent_pos_type));

  bool ignore_lateral = agent_pos_type == AgentPosType::REAR ||
                        agent_pos_type == AgentPosType::FRONT;
  bool ignore_longitudinal = agent_pos_type == AgentPosType::LEFT_OVERLAP ||
                             agent_pos_type == AgentPosType::RIGHT_OVERLAP;
  bool ahead_obs = frenet_obstacle->frenet_obstacle_boundary().s_end >
                   ego_frenet_boundary_.s_end;
  RiskLevel risk_level = RiskLevel::NO_RISK;
  RecommendedManeuver maneuver;
  bool left_side_obs = agent_pos_type == AgentPosType::LEFT_FRONT ||
                       agent_pos_type == AgentPosType::LEFT_REAR ||
                       agent_pos_type == AgentPosType::LEFT_OVERLAP;
  bool right_side_obs = agent_pos_type == AgentPosType::RIGHT_FRONT ||
                        agent_pos_type == AgentPosType::RIGHT_REAR ||
                        agent_pos_type == AgentPosType::RIGHT_OVERLAP;
  if ((left_side_obs && obs_state.second[1] > 0.3) ||
      (right_side_obs && obs_state.second[1] < -0.3)) {
    return true;
  }
  if ((longitudinal_distance > longitudinal_moderate_distance ||
       (ignore_longitudinal && !ahead_obs)) ||
      (lateral_distance > lateral_moderate_distance || ignore_lateral)) {
    // auto* debug = DebugInfoManager::GetInstance()
    //                   .GetDebugInfoPb()
    //                   ->mutable_potential_dangerous_agent_decider_info()
    //                   ->add_potential_dangerous_agent();
    debug->set_id(frenet_obstacle->id());
    debug->set_lateral_distance(lateral_distance);
    debug->set_longitudinal_distance(longitudinal_distance);
    debug->set_obstacle_rel_pos_type(static_cast<int>(agent_pos_type));
    debug->set_lateral_vel(obs_state.second[1]);
    debug->set_longitudinal_vel(obs_state.first[1]);
    debug->set_risk_level(static_cast<int>(risk_level));
    debug->set_recommended_lateral_maneuver(
        static_cast<int>(maneuver.lateral_maneuver));
    debug->set_recommended_longitudinal_maneuver(
        static_cast<int>(maneuver.longitudinal_maneuver));
    debug->set_longitudinal_moderate_safe_distance(
        longitudinal_moderate_distance);
    debug->set_lateral_moderate_safe_distance(lateral_moderate_distance);
    debug->set_longitudinal_reckless_safe_distance(
        longitudinal_reckless_distance);
    debug->set_lateral_reckless_safe_distance(lateral_reckless_distance);
    return true;
  }
  if (!ignore_lateral && !ignore_longitudinal) {
    if (longitudinal_distance <= longitudinal_moderate_distance &&
        longitudinal_distance > longitudinal_reckless_distance) {
      risk_level = RiskLevel::LOW_RISK;
      maneuver.longitudinal_maneuver = LongitudinalManeuver::SLIGHTLY_BRAKE;
      maneuver.lateral_maneuver = LateralManeuver::IGNORE;
      potential_dangerous_agent_decider_output->dangerous_agent_info
          .emplace_back(frenet_obstacle->id(), risk_level, maneuver,
                        lateral_distance, longitudinal_distance);
    } else if (longitudinal_distance <= longitudinal_reckless_distance) {
      if (lateral_distance <= lateral_moderate_distance &&
          lateral_distance > lateral_reckless_distance) {
        risk_level = RiskLevel::LOW_RISK;
        maneuver.longitudinal_maneuver = LongitudinalManeuver::SLIGHTLY_BRAKE;
        if (agent_pos_type == AgentPosType::LEFT_FRONT ||
            agent_pos_type == AgentPosType::LEFT_OVERLAP ||
            agent_pos_type == AgentPosType::LEFT_REAR) {
          maneuver.lateral_maneuver = LateralManeuver::RIGHT_SLIGHTLY_NUDGE;
        } else if (agent_pos_type == AgentPosType::RIGHT_FRONT ||
                   agent_pos_type == AgentPosType::RIGHT_OVERLAP ||
                   agent_pos_type == AgentPosType::RIGHT_REAR) {
          maneuver.lateral_maneuver = LateralManeuver::LEFT_SLIGHTLY_NUDGE;
        } else {
          maneuver.lateral_maneuver = LateralManeuver::BOTH_SIDE_SLIGHTLY_NUDGE;
        }
        potential_dangerous_agent_decider_output->dangerous_agent_info
            .emplace_back(frenet_obstacle->id(), risk_level, maneuver,
                          lateral_distance, longitudinal_distance);
      } else {
        risk_level = RiskLevel::HIGH_RISK;
        maneuver.longitudinal_maneuver = LongitudinalManeuver::SLIGHTLY_BRAKE;
        if (agent_pos_type == AgentPosType::LEFT_FRONT ||
            agent_pos_type == AgentPosType::LEFT_OVERLAP ||
            agent_pos_type == AgentPosType::LEFT_REAR) {
          maneuver.lateral_maneuver = LateralManeuver::RIGHT_NUDGE;
        } else if (agent_pos_type == AgentPosType::RIGHT_FRONT ||
                   agent_pos_type == AgentPosType::RIGHT_OVERLAP ||
                   agent_pos_type == AgentPosType::RIGHT_REAR) {
          maneuver.lateral_maneuver = LateralManeuver::LEFT_NUDGE;
        } else {
          maneuver.lateral_maneuver = LateralManeuver::BOTH_SIDE_NUDGE;
        }
        potential_dangerous_agent_decider_output->dangerous_agent_info
            .emplace_back(frenet_obstacle->id(), risk_level, maneuver,
                          lateral_distance, longitudinal_distance);
      }
    }
  } else if (ignore_lateral) {
    if (longitudinal_distance <= longitudinal_moderate_distance &&
        longitudinal_distance > longitudinal_reckless_distance) {
      risk_level = RiskLevel::LOW_RISK;
      maneuver.longitudinal_maneuver = LongitudinalManeuver::SLIGHTLY_BRAKE;
      maneuver.lateral_maneuver = LateralManeuver::IGNORE;
      potential_dangerous_agent_decider_output->dangerous_agent_info
          .emplace_back(frenet_obstacle->id(), risk_level, maneuver,
                        lateral_distance, longitudinal_distance);

    } else if (longitudinal_distance <= longitudinal_reckless_distance) {
      risk_level = RiskLevel::HIGH_RISK;
      maneuver.longitudinal_maneuver = LongitudinalManeuver::SLIGHTLY_BRAKE;
      if (agent_pos_type == AgentPosType::LEFT_FRONT ||
          agent_pos_type == AgentPosType::LEFT_OVERLAP ||
          agent_pos_type == AgentPosType::LEFT_REAR) {
        maneuver.lateral_maneuver = LateralManeuver::RIGHT_NUDGE;
      } else if (agent_pos_type == AgentPosType::RIGHT_FRONT ||
                 agent_pos_type == AgentPosType::RIGHT_OVERLAP ||
                 agent_pos_type == AgentPosType::RIGHT_REAR) {
        maneuver.lateral_maneuver = LateralManeuver::LEFT_NUDGE;
      } else {
        maneuver.lateral_maneuver = LateralManeuver::BOTH_SIDE_NUDGE;
      }
      potential_dangerous_agent_decider_output->dangerous_agent_info
          .emplace_back(frenet_obstacle->id(), risk_level, maneuver,
                        lateral_distance, longitudinal_distance);
    }
  } else if (ignore_longitudinal) {
    if (lateral_distance <= lateral_moderate_distance &&
        lateral_distance > lateral_reckless_distance) {
      risk_level = RiskLevel::LOW_RISK;
      maneuver.longitudinal_maneuver = LongitudinalManeuver::IGNORE;
      if (agent_pos_type == AgentPosType::LEFT_FRONT ||
          agent_pos_type == AgentPosType::LEFT_OVERLAP ||
          agent_pos_type == AgentPosType::LEFT_REAR) {
        maneuver.lateral_maneuver = LateralManeuver::RIGHT_SLIGHTLY_NUDGE;
      } else if (agent_pos_type == AgentPosType::RIGHT_FRONT ||
                 agent_pos_type == AgentPosType::RIGHT_OVERLAP ||
                 agent_pos_type == AgentPosType::RIGHT_REAR) {
        maneuver.lateral_maneuver = LateralManeuver::LEFT_SLIGHTLY_NUDGE;
      } else {
        maneuver.lateral_maneuver = LateralManeuver::BOTH_SIDE_SLIGHTLY_NUDGE;
      }
      potential_dangerous_agent_decider_output->dangerous_agent_info
          .emplace_back(frenet_obstacle->id(), risk_level, maneuver,
                        lateral_distance, longitudinal_distance);

    } else if (lateral_distance <= lateral_reckless_distance) {
      risk_level = RiskLevel::HIGH_RISK;
      maneuver.longitudinal_maneuver = LongitudinalManeuver::IGNORE;
      if (agent_pos_type == AgentPosType::LEFT_FRONT ||
          agent_pos_type == AgentPosType::LEFT_OVERLAP ||
          agent_pos_type == AgentPosType::LEFT_REAR) {
        maneuver.lateral_maneuver = LateralManeuver::RIGHT_NUDGE;
      } else if (agent_pos_type == AgentPosType::RIGHT_FRONT ||
                 agent_pos_type == AgentPosType::RIGHT_OVERLAP ||
                 agent_pos_type == AgentPosType::RIGHT_REAR) {
        maneuver.lateral_maneuver = LateralManeuver::LEFT_NUDGE;
      } else {
        maneuver.lateral_maneuver = LateralManeuver::BOTH_SIDE_NUDGE;
      }
      // potential_dangerous_agent_decider_output->potential_risk_outputs.emplace(
      //     frenet_obstacle->id(), std::make_pair(risk_level, maneuver));
      potential_dangerous_agent_decider_output->dangerous_agent_info
          .emplace_back(frenet_obstacle->id(), risk_level, maneuver,
                        lateral_distance, longitudinal_distance);
    }
  }
  debug->set_id(frenet_obstacle->id());
  debug->set_lateral_distance(lateral_distance);
  debug->set_longitudinal_distance(longitudinal_distance);
  debug->set_obstacle_rel_pos_type(static_cast<int>(agent_pos_type));
  debug->set_lateral_vel(obs_state.second[1]);
  debug->set_longitudinal_vel(obs_state.first[1]);
  debug->set_risk_level(static_cast<int>(risk_level));
  debug->set_recommended_lateral_maneuver(
      static_cast<int>(maneuver.lateral_maneuver));
  debug->set_recommended_longitudinal_maneuver(
      static_cast<int>(maneuver.longitudinal_maneuver));
  debug->set_longitudinal_moderate_safe_distance(
      longitudinal_moderate_distance);
  debug->set_lateral_moderate_safe_distance(lateral_moderate_distance);
  debug->set_longitudinal_reckless_safe_distance(
      longitudinal_reckless_distance);
  debug->set_lateral_reckless_safe_distance(lateral_reckless_distance);
  return true;
}

void PotentialDangerousAgentDecider::EstimateAgentPosType(
    std::shared_ptr<FrenetObstacle> frenet_obstacle, AgentPosType* pos_type,
    double* lateral_distance, double* longitudinal_distance) {
  if (frenet_obstacle == nullptr) {
    return;
  }
  const auto current_lane = virtual_lane_manager_->get_current_lane();
  double obstacle_center_s = frenet_obstacle->frenet_s();

  double lane_width_at_obstacle_center =
      current_lane->width_by_s(obstacle_center_s);
  double lane_left_width = lane_width_at_obstacle_center / 2.0;
  double lane_right_width = lane_width_at_obstacle_center / 2.0;
  const auto& obstacle_frenet_boundary =
      frenet_obstacle->frenet_obstacle_boundary();
  double left_passable_width = lane_left_width - obstacle_frenet_boundary.l_end;
  double right_passable_width =
      lane_right_width + obstacle_frenet_boundary.l_start;

  double ego_width =
      VehicleConfigurationContext::Instance()->get_vehicle_param().width;
  bool left_passable =
      left_passable_width > 0.5 * lane_width_at_obstacle_center;
  bool right_passable =
      right_passable_width > 0.5 * lane_width_at_obstacle_center;
  // bool outlane_obs =
  //     obstacle_frenet_boundary.l_end - obstacle_frenet_boundary.l_start +
  //         lane_left_width + lane_right_width >
  //     std::fmax(obstacle_frenet_boundary.l_end, lane_left_width) -
  //         std::fmin(obstacle_frenet_boundary.l_start, -lane_right_width);

  bool inlane_obstacle =
      (left_passable_width > 0.0 && right_passable_width > 0.0) ||
      (!left_passable && !right_passable);
  // bool press_line_obstacle =
  // bool inlane_obstacle = lane_left_width > obstacle_frenet_boundary.l_end &&
  // (-lane_right_width < obstacle_frenet_boundary.l_start);
  bool has_longitudinal_overlap =
      obstacle_frenet_boundary.s_end - obstacle_frenet_boundary.s_start +
          ego_frenet_boundary_.s_end - ego_frenet_boundary_.s_start >
      (std::fmax(ego_frenet_boundary_.s_end, obstacle_frenet_boundary.s_end) -
       std::fmin(ego_frenet_boundary_.s_start,
                 obstacle_frenet_boundary.s_start));

  double headway_distance =
      obstacle_frenet_boundary.s_end - ego_frenet_boundary_.s_end;
  bool front = headway_distance > 0.0;
  if (inlane_obstacle) {
    *pos_type = front ? AgentPosType::FRONT : AgentPosType::REAR;
    *lateral_distance = 0.0;
    *longitudinal_distance =
        front ? obstacle_frenet_boundary.s_start - ego_frenet_boundary_.s_end
              : ego_frenet_boundary_.s_start - obstacle_frenet_boundary.s_end;
  } else {
    if (has_longitudinal_overlap) {
      *pos_type = left_passable ? AgentPosType::RIGHT_OVERLAP
                                : AgentPosType::LEFT_OVERLAP;
      *lateral_distance =
          left_passable
              ? ego_frenet_boundary_.l_start - obstacle_frenet_boundary.l_end
              : obstacle_frenet_boundary.l_start - ego_frenet_boundary_.l_end;
      *lateral_distance = std::fmax(0.0, *lateral_distance);
      *longitudinal_distance = 0.0;

    } else {
      *pos_type = front ? (left_passable ? AgentPosType::RIGHT_FRONT
                                         : AgentPosType::LEFT_FRONT)
                        : (left_passable ? AgentPosType::RIGHT_REAR
                                         : AgentPosType::LEFT_REAR);
      *longitudinal_distance =
          front ? obstacle_frenet_boundary.s_start - ego_frenet_boundary_.s_end
                : ego_frenet_boundary_.s_start - obstacle_frenet_boundary.s_end;
      *lateral_distance =
          left_passable
              ? ego_frenet_boundary_.l_start - obstacle_frenet_boundary.l_end
              : obstacle_frenet_boundary.l_start - ego_frenet_boundary_.l_end;
      *lateral_distance = std::fmax(0.0, *lateral_distance);
    }
  }

  return;
}


void PotentialDangerousAgentDecider::LogDebugInfo() {
  potential_dangerous_agent_decider_info_.set_risk_free_lateral_distance(
      config_.risk_free_lateral_distance);
  potential_dangerous_agent_decider_info_.set_risk_free_longitudinal_distance(
      config_.risk_free_longitudinal_distance);
  potential_dangerous_agent_decider_info_.set_ego_lateral_vel(ego_state_.second[1]);
  potential_dangerous_agent_decider_info_.set_ego_longitudinal_vel(ego_state_.first[1]);
  auto* ego_sl_info = potential_dangerous_agent_decider_info_.mutable_ego_sl_info();
  ego_sl_info->set_s(ego_state_.first[0]);
  ego_sl_info->set_ds(ego_state_.first[1]);
  ego_sl_info->set_l(ego_state_.second[0]);
  ego_sl_info->set_dl(ego_state_.second[1]);
  ego_sl_info->set_dds(ego_state_.first[2]);
  ego_sl_info->set_ddl(ego_state_.second[2]);

#ifdef ENABLE_PROTO_LOG
  DebugInfoManager::GetInstance()
      .GetDebugInfoPb()
      ->mutable_potential_dangerous_agent_decider_info()
      ->CopyFrom(potential_dangerous_agent_decider_info_);
#endif
}

}  // namespace planning