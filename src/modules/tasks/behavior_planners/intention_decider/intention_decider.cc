#include "intention_decider.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <vector>

#include "agent/agent.h"
#include "debug_info_log.h"
#include "environmental_model.h"
#include "log.h"
#include "math_utils.h"
#include "tracked_object.h"
#include "virtual_lane_manager.h"

namespace planning {
namespace {
// Fixed constants (not configurable)
constexpr double kEpsilon = 1.0e-4;
constexpr double kLonIntentMaxDistance = 100.0;
constexpr double kLonIntentMinDistance = -80.0;
constexpr double kLonIntentMaxLateralDistance = 8.0;
constexpr int kMaxHistorySize = 5;
constexpr int kMinHistoryForLonBayes = 5;
constexpr double kForgetFactor = 0.1;
}  // namespace
namespace longitudinal_intention {

IntentionDecider::IntentionDecider(
    const EgoPlanningConfigBuilder* config_builder,
    framework::Session* session)
    : Task(config_builder, session) {
  name_ = "IntentionDecider";
  config_ = config_builder->cast<LongitudinalIntentionConfig>();
}

bool IntentionDecider::Execute() {
  ILOG_DEBUG << "=======IntentionDecider=======";

  // if (!PreCheck()) {
  //   ILOG_DEBUG << "PreCheck failed";
  //   return false;
  // }

  // Get managers from session
  const auto& environmental_model = session_->environmental_model();
  virtual_lane_manager_ = environmental_model.get_virtual_lane_manager();
  ego_state_manager_ = environmental_model.get_ego_state_manager();
  const auto& agent_manager = environmental_model.get_agent_manager();

  if (agent_manager == nullptr || virtual_lane_manager_ == nullptr ||
      ego_state_manager_ == nullptr) {
    ILOG_DEBUG << "Required managers not available";
    return false;
  }

  const auto& current_lane = virtual_lane_manager_->get_current_lane();
  if (current_lane == nullptr) {
    return false;
  }

  const auto& ego_lane_coord = current_lane->get_lane_frenet_coord();
  if (ego_lane_coord == nullptr) {
    return false;
  }

  const auto& init_point = ego_state_manager_->planning_init_point();

  double ego_s = 0.0, ego_l = 0.0;
  ego_lane_coord->XYToSL(init_point.x, init_point.y, &ego_s, &ego_l);

  DecideLongitudinalIntent(*agent_manager, init_point, ego_lane_coord,
                            agent_manager.get());
  UpdateAgentTable(*agent_manager, ego_s);

  return true;
}

void IntentionDecider::DecideLongitudinalIntent(
    const agent::AgentManager& agent_manager,
    const PlanningInitPoint& init_point,
    const std::shared_ptr<planning_math::KDPath>& ego_lane_coord,
    agent::AgentManager* const mutable_agent_manager) {

  if (ego_lane_coord == nullptr || mutable_agent_manager == nullptr) {
    return;
  }

  current_agent_ids_.clear();
  processed_lon_intent_agent_ids_.clear();

  double ego_s = 0.0, ego_l = 0.0;
  if (!ego_lane_coord->XYToSL(init_point.x, init_point.y, &ego_s, &ego_l)) {
    return;
  }

  const auto& agents = agent_manager.GetAllCurrentAgents();
  for (const auto& agent_ptr : agents) {
    if (agent_ptr == nullptr) {
      continue;
    }
    const agent::Agent& agent = *agent_ptr;
    const int32_t agent_id = agent.agent_id();

    current_agent_ids_.insert(agent_id);

    // Filter by fusion source
    if (!(agent.fusion_source() & OBSTACLE_SOURCE_CAMERA)) {
      continue;
    }

    // Filter static agents
    if (agent.is_static()) {
      continue;
    }

    // Filter oncoming agents: prediction trajectory s decreasing
    const auto& trajs = agent.trajectories();
    if (!trajs.empty() && trajs.front().size() >= 2) {
      const auto& traj = trajs.front();
      double s_front = 0.0, l_front = 0.0;
      double s_back = 0.0, l_back = 0.0;
      if (ego_lane_coord->XYToSL(traj.front().x(), traj.front().y(),
                                  &s_front, &l_front) &&
          ego_lane_coord->XYToSL(traj.back().x(), traj.back().y(),
                                  &s_back, &l_back)) {
        if (s_back < s_front) {
          continue;
        }
      }
    }

    // Update agent history
    AgentHistoryState current_state;
    UpdateAndGetAgentState(agent, init_point, ego_lane_coord, current_state);

    // Get agent position in ego lane coordinate
    const double agent_s = current_state.s;
    const double agent_l = current_state.l;

    // Filter by distance
    const double longitudinal_distance = agent_s - ego_s;
    const double lateral_distance = std::abs(agent_l - ego_l);

    if (longitudinal_distance > kLonIntentMaxDistance ||
        longitudinal_distance < kLonIntentMinDistance ||
        lateral_distance > kLonIntentMaxLateralDistance) {
      continue;
    }

    // Filter by relative speed
    const double ego_speed = init_point.v;
    const double agent_speed = current_state.speed;
    if (longitudinal_distance < -30.0 && agent_speed < ego_speed) {
      continue;
    }
    if (longitudinal_distance > 40.0 && agent_speed > ego_speed) {
      continue;
    }

    // Extract features and calculate intent
    LongitudinalBayesFeatures lon_features =
        ExtractLongitudinalBayesFeatures(agent_id, agent, ego_lane_coord);

    if (!lon_features.valid) {
      continue;
    }

    LongitudinalIntentResult lon_intent_result =
        CalculateLongitudinalIntent(agent_id, agent, lon_features);

    processed_lon_intent_agent_ids_.insert(agent_id);

    // Update agent manager
    auto* mutable_agent = mutable_agent_manager->mutable_agent(agent_id);
    if (mutable_agent != nullptr) {
      mutable_agent->set_longitudinal_intent(
          static_cast<int>(lon_intent_result.intent));
      mutable_agent->set_lon_decel_prob(lon_intent_result.decel_prob);
      mutable_agent->set_lon_cruise_prob(lon_intent_result.cruise_prob);
      mutable_agent->set_lon_accel_prob(lon_intent_result.accel_prob);
    }
  }
}

void IntentionDecider::UpdateAgentTable(
    const agent::AgentManager& agent_manager, double ego_s) {

  std::set<int32_t> agents_to_remove;
  for (const auto& pair : agent_history_map_) {
    if (current_agent_ids_.find(pair.first) == current_agent_ids_.end()) {
      agents_to_remove.insert(pair.first);
    }
  }

  for (int32_t agent_id : agents_to_remove) {
    agent_history_map_.erase(agent_id);
  }

  for (auto& pair : lon_intent_map_) {
    if (current_agent_ids_.find(pair.first) == current_agent_ids_.end()) {
      continue;
    }
    if (processed_lon_intent_agent_ids_.find(pair.first) ==
        processed_lon_intent_agent_ids_.end()) {
      lon_decel_posterior_[pair.first] = config_.lon_bayes_decel_prior;
      lon_cruise_posterior_[pair.first] = config_.lon_bayes_cruise_prior;
      lon_accel_posterior_[pair.first] = config_.lon_bayes_accel_prior;
      pair.second = static_cast<int>(LongitudinalIntent::CRUISE);
    }//当前帧存在但本轮被过滤掉没算贝叶斯[超出范围、历史帧数还不够]
  }

  for (int32_t agent_id : agents_to_remove) {
    lon_decel_posterior_.erase(agent_id);
    lon_cruise_posterior_.erase(agent_id);
    lon_accel_posterior_.erase(agent_id);
    lon_intent_map_.erase(agent_id);
  }

  constexpr int kMaxDisplayPerIntent = 5;

  // Collect (distance, agent_id, score) tuples per intent, then sort by distance
  // Prioritize rear agents (negative distance), then front agents by ascending |distance|
  using IntentEntry = std::pair<double, std::pair<int32_t, double>>;  // (dist, (id, score))
  auto collect_and_sort = [&](LongitudinalIntent intent,
                               std::vector<double>& out_ids,
                               std::vector<double>& out_scores) {
    std::vector<IntentEntry> entries;
    for (const auto& kv : lon_intent_map_) {
      if (kv.second != static_cast<int>(intent)) continue;
      const int32_t id = kv.first;
      auto hist_it = agent_history_map_.find(id);
      if (hist_it == agent_history_map_.end() || hist_it->second.empty()) continue;
      const double dist = hist_it->second.back().s - ego_s;  // Keep sign: negative=rear, positive=front
      double score = 0.0;
      if (intent == LongitudinalIntent::DECEL && lon_decel_posterior_.count(id))
        score = lon_decel_posterior_.at(id);
      else if (intent == LongitudinalIntent::CRUISE && lon_cruise_posterior_.count(id))
        score = lon_cruise_posterior_.at(id);
      else if (intent == LongitudinalIntent::ACCEL && lon_accel_posterior_.count(id))
        score = lon_accel_posterior_.at(id);
      entries.emplace_back(dist, std::make_pair(id, score));
    }
    // Sort: rear agents first (by descending distance, i.e., closest rear first),
    // then front agents (by ascending distance, i.e., closest front first)
    std::sort(entries.begin(), entries.end(),
              [](const IntentEntry& a, const IntentEntry& b) {
                const bool a_rear = a.first < 0.0;
                const bool b_rear = b.first < 0.0;
                if (a_rear && b_rear) return a.first > b.first;  // Both rear: larger (closer to 0) first
                if (a_rear) return true;   // a rear, b front: a first
                if (b_rear) return false;  // a front, b rear: b first
                return a.first < b.first;  // Both front: smaller (closer to 0) first
              });
    const int n = std::min(static_cast<int>(entries.size()), kMaxDisplayPerIntent);
    for (int i = 0; i < n; ++i) {
      out_ids.emplace_back(entries[i].second.first);
      out_scores.emplace_back(entries[i].second.second);
    }
  };

  std::vector<double> lon_decel_agent_ids, lon_decel_scores;
  std::vector<double> lon_cruise_agent_ids, lon_cruise_scores;
  std::vector<double> lon_accel_agent_ids, lon_accel_scores;

  collect_and_sort(LongitudinalIntent::DECEL, lon_decel_agent_ids, lon_decel_scores);
  collect_and_sort(LongitudinalIntent::CRUISE, lon_cruise_agent_ids, lon_cruise_scores);
  collect_and_sort(LongitudinalIntent::ACCEL, lon_accel_agent_ids, lon_accel_scores);

  JSON_DEBUG_VECTOR("lon_decel_agent_ids", lon_decel_agent_ids, 0);
  JSON_DEBUG_VECTOR("lon_decel_scores", lon_decel_scores, 2);
  JSON_DEBUG_VECTOR("lon_cruise_agent_ids", lon_cruise_agent_ids, 0);
  JSON_DEBUG_VECTOR("lon_cruise_scores", lon_cruise_scores, 2);
  JSON_DEBUG_VECTOR("lon_accel_agent_ids", lon_accel_agent_ids, 0);
  JSON_DEBUG_VECTOR("lon_accel_scores", lon_accel_scores, 2);
}

void IntentionDecider::UpdateAndGetAgentState(
    const agent::Agent& agent,
    const PlanningInitPoint& init_point,
    const std::shared_ptr<planning_math::KDPath>& ego_lane_coord,
    AgentHistoryState& current_state) {

  const int32_t agent_id = agent.agent_id();

  current_state.timestamp = agent.timestamp_s();
  current_state.x = agent.x();
  current_state.y = agent.y();

  double current_s = 0.0, current_l = 0.0;
  if (ego_lane_coord->XYToSL(current_state.x, current_state.y, &current_s,
                             &current_l)) {
    current_state.l = current_l;
    current_state.s = current_s;

    // 1-frame features: speed, accel_fusion, vel_prediction, prediction_length
    current_state.speed = agent.speed();
    current_state.accel_fusion = agent.accel_fusion();

    const auto& trajs = agent.trajectories();
    if (!trajs.empty() && !trajs.front().empty() && trajs.front().size() >= 2) {
      const auto& traj = trajs.front();
      double traj_len = 0.0;
      for (size_t ti = 1; ti < traj.size(); ++ti) {
        const double dx = traj[ti].x() - traj[ti - 1].x();
        const double dy = traj[ti].y() - traj[ti - 1].y();
        traj_len += std::sqrt(dx * dx + dy * dy);
      }
      const double traj_time = 5.0;
      current_state.prediction_length = traj_len;
      current_state.vel_prediction =
          (traj_time > 1e-3) ? traj_len / traj_time : 0.0;
    }

    // 2-frame features: accel_sp, acc_prediction
    auto it = agent_history_map_.find(agent_id);
    if (it != agent_history_map_.end() && !it->second.empty()) {
      const auto& prev_state = it->second.back();
      double dt = 0.1;
      if (dt > 1e-3) {
        current_state.accel_sp = (current_state.speed - prev_state.speed) / dt;
        current_state.acc_prediction =
            (current_state.vel_prediction - prev_state.vel_prediction) / dt;
      } else {
        current_state.accel_sp = prev_state.accel_sp;
        current_state.acc_prediction = prev_state.acc_prediction;
      }
    } else {
      current_state.accel_sp = 0.0;
      current_state.acc_prediction = 0.0;
    }
  }

  agent_history_map_[agent_id].push_back(current_state);

  if (agent_history_map_[agent_id].size() > kMaxHistorySize) {
    agent_history_map_[agent_id].pop_front();
  }
}

LongitudinalBayesFeatures IntentionDecider::ExtractLongitudinalBayesFeatures(
    int32_t agent_id, const agent::Agent& agent,
    const std::shared_ptr<planning_math::KDPath>& ego_lane_coord) {
  LongitudinalBayesFeatures features;

  auto it = agent_history_map_.find(agent_id);
  if (it == agent_history_map_.end() ||
      static_cast<int>(it->second.size()) < kMinHistoryForLonBayes) {
    return features;
  }

  const auto& history = it->second;
  for (int i = 0; i < static_cast<int>(history.size()); ++i) {
    features.accel_fusion_vec.push_back(history[i].accel_fusion);
    features.accel_vel_vec.push_back(history[i].accel_sp);
    features.accel_pred_vec.push_back(history[i].acc_prediction);
  }

  features.valid = features.accel_fusion_vec.size() >= 2;
  return features;
}

LongitudinalIntentResult IntentionDecider::CalculateLongitudinalIntent(
    int32_t agent_id, const agent::Agent& agent,
    const LongitudinalBayesFeatures& features) {
  LongitudinalIntentResult result;

  if (!features.valid) {
    return result;
  }

  double log_L_decel = 0.0;
  double log_L_cruise = 0.0;
  double log_L_accel = 0.0;

  const int num_samples = static_cast<int>(features.accel_fusion_vec.size());
  for (int i = 0; i < num_samples; ++i) {
    const double accel_fusion = features.accel_fusion_vec[i];
    const double accel_vel = features.accel_vel_vec[i];
    const double accel_pred = features.accel_pred_vec[i];

    const double d_fusion_decel = accel_fusion - config_.accel_fusion_decel_mu;
    const double d_fusion_cruise = accel_fusion - config_.accel_fusion_cruise_mu;
    const double d_fusion_accel = accel_fusion - config_.accel_fusion_accel_mu;
    log_L_decel += config_.accel_fusion_weight *
                   (-0.5 * d_fusion_decel * d_fusion_decel /
                        (config_.accel_fusion_decel_sigma * config_.accel_fusion_decel_sigma) -
                    0.5 * std::log(2.0 * M_PI * config_.accel_fusion_decel_sigma * config_.accel_fusion_decel_sigma));
    log_L_cruise += config_.accel_fusion_weight *
                    (-0.5 * d_fusion_cruise * d_fusion_cruise /
                         (config_.accel_fusion_cruise_sigma * config_.accel_fusion_cruise_sigma) -
                     0.5 * std::log(2.0 * M_PI * config_.accel_fusion_cruise_sigma * config_.accel_fusion_cruise_sigma));
    log_L_accel += config_.accel_fusion_weight *
                   (-0.5 * d_fusion_accel * d_fusion_accel /
                        (config_.accel_fusion_accel_sigma * config_.accel_fusion_accel_sigma) -
                    0.5 * std::log(2.0 * M_PI * config_.accel_fusion_accel_sigma * config_.accel_fusion_accel_sigma));

    const double d_vel_decel = accel_vel - config_.accel_vel_decel_mu;
    const double d_vel_cruise = accel_vel - config_.accel_vel_cruise_mu;
    const double d_vel_accel = accel_vel - config_.accel_vel_accel_mu;
    log_L_decel += config_.accel_vel_weight *
                   (-0.5 * d_vel_decel * d_vel_decel /
                        (config_.accel_vel_decel_sigma * config_.accel_vel_decel_sigma) -
                    0.5 * std::log(2.0 * M_PI * config_.accel_vel_decel_sigma * config_.accel_vel_decel_sigma));
    log_L_cruise += config_.accel_vel_weight *
                    (-0.5 * d_vel_cruise * d_vel_cruise /
                         (config_.accel_vel_cruise_sigma * config_.accel_vel_cruise_sigma) -
                     0.5 * std::log(2.0 * M_PI * config_.accel_vel_cruise_sigma * config_.accel_vel_cruise_sigma));
    log_L_accel += config_.accel_vel_weight *
                   (-0.5 * d_vel_accel * d_vel_accel /
                        (config_.accel_vel_accel_sigma * config_.accel_vel_accel_sigma) -
                    0.5 * std::log(2.0 * M_PI * config_.accel_vel_accel_sigma * config_.accel_vel_accel_sigma));

    const double d_pred_decel = accel_pred - config_.accel_pred_decel_mu;
    const double d_pred_cruise = accel_pred - config_.accel_pred_cruise_mu;
    const double d_pred_accel = accel_pred - config_.accel_pred_accel_mu;
    log_L_decel += config_.accel_pred_weight *
                   (-0.5 * d_pred_decel * d_pred_decel /
                        (config_.accel_pred_decel_sigma * config_.accel_pred_decel_sigma) -
                    0.5 * std::log(2.0 * M_PI * config_.accel_pred_decel_sigma * config_.accel_pred_decel_sigma));
    log_L_cruise += config_.accel_pred_weight *
                    (-0.5 * d_pred_cruise * d_pred_cruise /
                         (config_.accel_pred_cruise_sigma * config_.accel_pred_cruise_sigma) -
                     0.5 * std::log(2.0 * M_PI * config_.accel_pred_cruise_sigma * config_.accel_pred_cruise_sigma));
    log_L_accel += config_.accel_pred_weight *
                   (-0.5 * d_pred_accel * d_pred_accel /
                        (config_.accel_pred_accel_sigma * config_.accel_pred_accel_sigma) -
                    0.5 * std::log(2.0 * M_PI * config_.accel_pred_accel_sigma * config_.accel_pred_accel_sigma));
  }

  // 归一化 log-likelihood：除以数据点数量，防止似然比过于极端
  const int total_points = std::max(1, num_samples);
  log_L_decel /= total_points;
  log_L_cruise /= total_points;
  log_L_accel /= total_points;

  const double max_log = std::max({log_L_decel, log_L_cruise, log_L_accel});
  const double L_decel = std::exp(log_L_decel - max_log);
  const double L_cruise = std::exp(log_L_cruise - max_log);
  const double L_accel = std::exp(log_L_accel - max_log);

  // --- 原方案：固定先验 + EMA 平滑 ---
  // const double denominator = L_decel * kLonBayesDecelPrior + L_cruise * kLonBayesCruisePrior +
  //                            L_accel * kLonBayesAccelPrior;
  //
  // double p_decel = (denominator < 1e-10) ? 0.0 : L_decel * kLonBayesDecelPrior / denominator;
  // double p_cruise = (denominator < 1e-10) ? 0.0 : L_cruise * kLonBayesCruisePrior / denominator;
  // double p_accel = (denominator < 1e-10) ? 0.0 : L_accel * kLonBayesAccelPrior / denominator;
  //
  // auto it_decel = lon_decel_posterior_.find(agent_id);
  // const double prev_decel = (it_decel != lon_decel_posterior_.end()) ? it_decel->second : kLonBayesDecelPrior;
  // p_decel = kLonBayesSmoothAlpha * p_decel + (1.0 - kLonBayesSmoothAlpha) * prev_decel;
  //
  // auto it_cruise = lon_cruise_posterior_.find(agent_id);
  // const double prev_cruise = (it_cruise != lon_cruise_posterior_.end()) ? it_cruise->second : kLonBayesCruisePrior;
  // p_cruise = kLonBayesSmoothAlpha * p_cruise + (1.0 - kLonBayesSmoothAlpha) * prev_cruise;
  //
  // auto it_accel = lon_accel_posterior_.find(agent_id);
  // const double prev_accel = (it_accel != lon_accel_posterior_.end()) ? it_accel->second : kLonBayesAccelPrior;
  // p_accel = kLonBayesSmoothAlpha * p_accel + (1.0 - kLonBayesSmoothAlpha) * prev_accel;

  // --- 新方案：上一帧后验作为本帧先验 + 遗忘因子防坍塌 ---
  auto it_decel = lon_decel_posterior_.find(agent_id);
  const double prev_decel = (it_decel != lon_decel_posterior_.end()) ? it_decel->second : config_.lon_bayes_decel_prior;
  auto it_cruise = lon_cruise_posterior_.find(agent_id);
  const double prev_cruise = (it_cruise != lon_cruise_posterior_.end()) ? it_cruise->second : config_.lon_bayes_cruise_prior;
  auto it_accel = lon_accel_posterior_.find(agent_id);
  const double prev_accel = (it_accel != lon_accel_posterior_.end()) ? it_accel->second : config_.lon_bayes_accel_prior;

  const double prior_decel = (1.0 - kForgetFactor) * prev_decel + kForgetFactor * config_.lon_bayes_decel_prior;
  const double prior_cruise = (1.0 - kForgetFactor) * prev_cruise + kForgetFactor * config_.lon_bayes_cruise_prior;
  const double prior_accel = (1.0 - kForgetFactor) * prev_accel + kForgetFactor * config_.lon_bayes_accel_prior;

  const double denominator = L_decel * prior_decel + L_cruise * prior_cruise +
                             L_accel * prior_accel;

  double p_decel = (denominator < 1e-10) ? 0.0 : L_decel * prior_decel / denominator;
  double p_cruise = (denominator < 1e-10) ? 0.0 : L_cruise * prior_cruise / denominator;
  double p_accel = (denominator < 1e-10) ? 0.0 : L_accel * prior_accel / denominator;

  const double sum = p_decel + p_cruise + p_accel;
  if (sum > 1e-10) {
    p_decel /= sum;
    p_cruise /= sum;
    p_accel /= sum;
  }

  // EMA 平滑：当前帧权重 0.7，历史权重 0.3
  auto it_decel_prev = lon_decel_posterior_.find(agent_id);
  auto it_cruise_prev = lon_cruise_posterior_.find(agent_id);
  auto it_accel_prev = lon_accel_posterior_.find(agent_id);

  if (it_decel_prev != lon_decel_posterior_.end() &&
      it_cruise_prev != lon_cruise_posterior_.end() &&
      it_accel_prev != lon_accel_posterior_.end()) {
    p_decel = config_.lon_bayes_smooth_alpha * p_decel + (1.0 - config_.lon_bayes_smooth_alpha) * it_decel_prev->second;
    p_cruise = config_.lon_bayes_smooth_alpha * p_cruise + (1.0 - config_.lon_bayes_smooth_alpha) * it_cruise_prev->second;
    p_accel = config_.lon_bayes_smooth_alpha * p_accel + (1.0 - config_.lon_bayes_smooth_alpha) * it_accel_prev->second;

    // 重新归一化
    const double sum_smoothed = p_decel + p_cruise + p_accel;
    if (sum_smoothed > 1e-10) {
      p_decel /= sum_smoothed;
      p_cruise /= sum_smoothed;
      p_accel /= sum_smoothed;
    }
  }

  lon_decel_posterior_[agent_id] = p_decel;
  lon_cruise_posterior_[agent_id] = p_cruise;
  lon_accel_posterior_[agent_id] = p_accel;

  const double max_prob = std::max({p_decel, p_cruise, p_accel});
  result.decel_prob = p_decel;
  result.cruise_prob = p_cruise;
  result.accel_prob = p_accel;
  result.confidence = max_prob;

  if (max_prob < config_.lon_intent_confidence_threshold) {
    result.intent = LongitudinalIntent::UNKNOWN;
    result.unknown_prob = 1.0 - max_prob;
  } else if (max_prob == p_decel) {
    result.intent = LongitudinalIntent::DECEL;
  } else if (max_prob == p_cruise) {
    result.intent = LongitudinalIntent::CRUISE;
  } else {
    result.intent = LongitudinalIntent::ACCEL;
  }

  lon_intent_map_[agent_id] = static_cast<int>(result.intent);

  return result;
}

}  // namespace longitudinal_intention
}  // namespace planning

