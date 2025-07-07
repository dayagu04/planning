#include "modules/common/rss_model/rss_model.h"

namespace planning {
bool RssModel::CalculateSafeDistance(
    const FrenetState& agent, const FrenetState& ego,
    const AgentPosType& pos_type,
    const PotentialDangerousAgentDeciderConfig::RSSModelConfig& rss_config,
    double* lateral_safe_distance, double* longitudinal_safe_distance) {
  if (lateral_safe_distance == nullptr ||
      longitudinal_safe_distance == nullptr) {
    return false;
  }
  *lateral_safe_distance = RssModel::CalculateLateralSafeDistance(
      agent.second, ego.second, pos_type, rss_config);
  *longitudinal_safe_distance = RssModel::CalculateLongitudinalSafeDistance(
      agent.first, ego.first, pos_type, rss_config);
  return true;
}

double RssModel::CalculateLateralSafeDistance(
    const State& agent, const State& ego, const AgentPosType& pos_type,
    const PotentialDangerousAgentDeciderConfig::RSSModelConfig& rss_config) {
  double safe_lateral_distance = 0.0;
  double ego_lat_vel_abs = std::fabs(ego[1]);
  double other_lat_vel_abs = std::fabs(agent[1]);
  double distance_correction = rss_config.lateral_miu;
  double ego_lat_vel_at_response_time =
      ego_lat_vel_abs + rss_config.response_time * rss_config.lateral_acc_max;
  double other_lat_vel_at_response_time =
      other_lat_vel_abs + rss_config.response_time * rss_config.lateral_acc_max;
  double ego_active_brake_distance =
      ego_lat_vel_abs * ego_lat_vel_abs / (2.0 * rss_config.lateral_brake_max);
  double ego_passive_brake_distance =
      (ego_lat_vel_abs + ego_lat_vel_at_response_time) / 2.0 *
          rss_config.response_time +
      ego_lat_vel_at_response_time * ego_lat_vel_at_response_time /
          (2.0 * rss_config.lateral_brake_min);
  double other_active_brake_distance = other_lat_vel_abs * other_lat_vel_abs /
                                       (2 * rss_config.lateral_brake_max);
  double other_passive_brake_distance =
      (other_lat_vel_abs + other_lat_vel_at_response_time) / 2.0 *
          rss_config.response_time +
      other_lat_vel_at_response_time * other_lat_vel_at_response_time /
          (2 * rss_config.lateral_brake_min);
  if (pos_type == AgentPosType::RIGHT_FRONT ||
      pos_type == AgentPosType::RIGHT_REAR ||
      pos_type == AgentPosType::RIGHT_OVERLAP) {
    if (ego[1] >= 0.0 && agent[1] >= 0.0) {
      safe_lateral_distance =
          other_passive_brake_distance - ego_active_brake_distance;
    } else if (ego[1] >= 0 && agent[1] < 0.0) {
      safe_lateral_distance = 0.0;
      distance_correction = 0.0;
    } else if (ego[1] < 0.0 && agent[1] < 0.0) {
      safe_lateral_distance =
          ego_passive_brake_distance - other_active_brake_distance;
    } else if (ego[1] < 0.0 && agent[1] >= 0.0) {
      safe_lateral_distance =
          ego_passive_brake_distance + other_passive_brake_distance;
    } else {
      safe_lateral_distance = 0.0;
    }
  } else if (pos_type == AgentPosType::LEFT_FRONT ||
             pos_type == AgentPosType::LEFT_REAR ||
             pos_type == AgentPosType::LEFT_OVERLAP) {
    if (ego[1] >= 0.0 && agent[1] >= 0.0) {
      safe_lateral_distance =
          ego_passive_brake_distance - other_active_brake_distance;
    } else if (ego[1] >= 0 && agent[1] < 0.0) {
      safe_lateral_distance =
          ego_passive_brake_distance + other_passive_brake_distance;

    } else if (ego[1] < 0.0 && agent[1] < 0.0) {
      safe_lateral_distance =
          other_passive_brake_distance - ego_active_brake_distance;
    } else if (ego[1] < 0.0 && agent[1] >= 0.0) {
      safe_lateral_distance = 0.0;
    } else {
      safe_lateral_distance = 0.0;
    }
  } else {
    safe_lateral_distance = 0.0;
  }
  safe_lateral_distance =
      safe_lateral_distance > 0.0 ? safe_lateral_distance : 0.0;
  safe_lateral_distance += distance_correction;
  return safe_lateral_distance;
}

double RssModel::CalculateLongitudinalSafeDistance(
    const State& agent, const State& ego, const AgentPosType& pos_type,
    const PotentialDangerousAgentDeciderConfig::RSSModelConfig& rss_config) {
  double ret = 0.0;
  double ego_vel = ego[1];
  double other_vel = agent[1];
  double ego_vel_abs = fabs(ego[1]);
  double other_vel_abs = fabs(agent[1]);
  double ego_vel_at_response_time =
      ego_vel_abs + rss_config.longitudinal_acc_max * rss_config.response_time;
  double other_vel_at_response_time =
      other_vel_abs +
      rss_config.longitudinal_acc_max * rss_config.response_time;

  double ego_distance_driven, other_distance_driven;
  if (pos_type == AgentPosType::LEFT_FRONT ||
      pos_type == AgentPosType::RIGHT_FRONT ||
      pos_type == AgentPosType::FRONT) {
    ego_distance_driven = (ego_vel_abs + ego_vel_at_response_time) / 2.0 *
                              rss_config.response_time +
                          ego_vel_at_response_time * ego_vel_at_response_time /
                              (2 * rss_config.longitudinal_brake_min);
    if (ego_vel >= 0.0 && other_vel >= 0.0) {
      // ego vehicle ==> other vehicle ->
      other_distance_driven = (other_vel_abs * other_vel_abs) /
                              (2 * rss_config.longitudinal_brake_max);
      ret = ego_distance_driven - other_distance_driven;
    } else if (ego_vel >= 0.0 && other_vel <= 0.0) {
      // ego vehicle ==> <-- other vehicle
      other_distance_driven = (other_vel_abs + other_vel_at_response_time) /
                                  2.0 * rss_config.response_time +
                              other_vel_at_response_time *
                                  other_vel_at_response_time /
                                  (2 * rss_config.longitudinal_brake_min);
      ret = ego_distance_driven + other_distance_driven;
    } else {
      ret = 0.0;
    }
  } else if (pos_type == AgentPosType::LEFT_REAR ||
             pos_type == AgentPosType::RIGHT_REAR ||
             pos_type == AgentPosType::REAR) {
    ego_distance_driven =
        ego_vel_abs * ego_vel_abs / (2 * rss_config.longitudinal_brake_max);
    if (ego_vel >= 0.0 && other_vel >= 0.0) {
      other_distance_driven = (other_vel_abs + other_vel_at_response_time) /
                                  2.0 * rss_config.response_time +
                              other_vel_at_response_time *
                                  other_vel_at_response_time /
                                  (2 * rss_config.longitudinal_brake_min);
      ret = other_distance_driven - ego_distance_driven;
    } else if (ego_vel >= 0.0 && other_vel <= 0.0) {
      ret = 0.0;
    } else {
      ret = 0.0;
    }
  } else {
    ret = 0.0;
  }
  return ret > 0.0 ? ret : 0.0;
}

}  // namespace planning