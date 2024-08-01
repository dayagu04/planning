#include "agent_manager.h"
#include <cstdint>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include "agent/agent.h"
#include "common_c.h"
#include "environmental_model.h"
#include "session.h"

namespace planning {
namespace agent {

AgentManager::AgentManager(const EgoPlanningConfigBuilder* config_builder,
                           planning::framework::Session* session)
    : session_(session) {
  // TODO: 添加AgentManager的config
  config_ = config_builder->cast<EgoPlanningObstacleManagerConfig>();
}

void AgentManager::Reset() {
  current_agents_.clear();
  current_agents_ids_.clear();
  historical_agents_.clear();
}

const std::vector<const Agent*>& AgentManager::GetAllCurrentAgents() const {
  return current_agents_;
}
const std::unordered_set<int32_t>& AgentManager::GetAgentSet() const {
  return current_agents_ids_;
}

void AgentManager::Update() {
  current_agents_.clear();
  current_agents_ids_.clear();
  const auto& ego_state =
      *session_->mutable_environmental_model()->get_ego_state_manager();
  double init_time = ego_state.planning_init_point().relative_time;
  const auto& prediction_objects =
      session_->environmental_model().get_prediction_info();
  for (int i = 0;
       i < session_->environmental_model().get_prediction_info().size(); i++) {
    auto prediction_object = prediction_objects[i];
    // TBD:后续前移至EnvironmentalModelManager::obstacle_prediction_update中
    if (prediction_object.type == 0 ||
        ((!(prediction_object.fusion_source & OBSTACLE_SOURCE_CAMERA)) &&
         (prediction_object.relative_position_x > 0 &&
          tan(25) > fabs(prediction_object.relative_position_y /
                         prediction_object.relative_position_x))) ||
        fabs(prediction_object.relative_position_y) > 10 ||
        prediction_object.length == 0 || prediction_object.width == 0) {
      LOG_DEBUG("[obstacle_prediction_update] ignore obstacle! : [%d] \n",
                prediction_object.id);
      continue;
    }

    double prediction_trajectory_length = 10.0;
    double prediction_duration = 0.0;
    if (prediction_object.trajectory_array.size() > 0) {
      const auto& trajectory_array = prediction_object.trajectory_array.at(0);
      if (trajectory_array.trajectory.size() > 0) {
        const auto& start_point = trajectory_array.trajectory.at(0);
        const auto& end_point = trajectory_array.trajectory.at(
            trajectory_array.trajectory.size() - 1);
        prediction_trajectory_length = std::sqrt(
            (start_point.x - end_point.x) * (start_point.x - end_point.x) +
            (start_point.y - end_point.y) * (start_point.y - end_point.y));
        prediction_duration = end_point.relative_time;
      }
    }

    const double kMaxStaticPredictionLength =
        config_.max_speed_static_obstacle * prediction_duration;
    bool is_static =
        prediction_object.speed < 0.1 ||
        prediction_object.trajectory_array.size() == 0 ||
        prediction_trajectory_length < kMaxStaticPredictionLength ||
        prediction_object.motion_pattern_current ==
            iflyauto::OBJECT_MOTION_TYPE_STATIC;
    double prediction_relative_time = prediction_object.delay_time - init_time;
    unordered_map<int32_t, Agent> agent_table;
    if (prediction_object.trajectory_array.size() == 0) {
      auto agent =
          Agent(prediction_object, is_static, prediction_relative_time);
      agent_table.insert({agent.agent_id(), agent});
      Append(agent_table);
      continue;
    }
    for (int i = 0; i < prediction_object.trajectory_array.size(); ++i) {
      auto agent =
          Agent(prediction_object, is_static, prediction_relative_time);
      agent_table.insert({agent.agent_id(), agent});
      Append(agent_table);
      break;
    }
  }
  DeleteOlderAgent();
}

void AgentManager::Update(
    const double start_timestamp_s,
    const std::unordered_map<int32_t, Agent>& agent_table) {
  current_agents_.clear();
  current_agents_ids_.clear();
  constexpr double kTooOldThreshold = 2.0;
  for (auto iter = historical_agents_.begin();
       iter != historical_agents_.end();) {
    bool is_too_old = start_timestamp_s - iter->second.back()->timestamp_s() >
                      kTooOldThreshold;
    bool is_del = is_too_old;
    if (is_del) {
      historical_agents_.erase(iter++);
    } else {
      ++iter;
    }
  }

  for (const auto& data : agent_table) {
    auto agent = std::make_unique<Agent>(data.second);
    current_agents_.emplace_back(agent.get());
    current_agents_ids_.insert(agent->agent_id());
    historical_agents_[agent->agent_id()].emplace_back(std::move(agent));
  }
  DeleteOlderAgent();
}

void AgentManager::Append(
    const std::unordered_map<int32_t, Agent>& agent_table) {
  for (const auto& data : agent_table) {
    if (current_agents_ids_.count(data.first) > 0) {
      continue;
    }
    auto agent = std::make_unique<Agent>(data.second);
    if (nullptr == agent) {
      continue;
    }
    current_agents_.emplace_back(agent.get());
    current_agents_ids_.insert(agent->agent_id());
    historical_agents_[agent->agent_id()].emplace_back(std::move(agent));
  }
}

void AgentManager::DeleteOlderAgent() {
  // pop the oldest obstacle while duration > obstacle_max_duration
  constexpr double kMaxDurationS = 2.0;
  constexpr int32_t kMaxNum = 20;
  for (auto& agent : historical_agents_) {
    if (agent.second.size() < 2) {
      continue;
    }
    while (agent.second.back()->timestamp_s() -
               agent.second.front()->timestamp_s() >
           kMaxDurationS) {
      agent.second.pop_front();
    }
    while (agent.second.back()->timestamp_s() <
           agent.second.front()->timestamp_s()) {
      agent.second.pop_front();
    }
    while (agent.second.size() > kMaxNum) {
      agent.second.pop_front();
    }
  }
}

Agent* AgentManager::mutable_agent(const int32_t id) {
  auto iter = historical_agents_.find(id);
  if (iter == historical_agents_.end() || iter->second.empty()) {
    return nullptr;
  }
  return iter->second.back().get();
}

const Agent* AgentManager::GetAgent(const int32_t id) const {
  auto iter = historical_agents_.find(id);
  if (iter == historical_agents_.end() || iter->second.empty()) {
    return nullptr;
  }
  return iter->second.back().get();
}

const Agent* AgentManager::GetHistoryAgentByIndex(
    const int32_t id, const int32_t prev_index) const {
  auto iter = historical_agents_.find(id);
  if (iter == historical_agents_.end() || iter->second.empty()) {
    return nullptr;
  }
  const auto& agent_list = iter->second;
  int32_t counter = 0;
  for (auto it = agent_list.rbegin(); it != agent_list.rend(); it++) {
    if (prev_index == counter) {
      return it->get();
      break;
    }
    counter++;
  }
  return nullptr;
}

}  // namespace agent
}  // namespace planning