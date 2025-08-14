#include "agent_manager.h"

#include <cstdint>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "agent/agent.h"
#include "common_c.h"
#include "environmental_model.h"
#include "ifly_time.h"
#include "session.h"

namespace planning {
namespace agent {

AgentManager::AgentManager(const EgoPlanningConfigBuilder* config_builder,
                           planning::framework::Session* session)
    : session_(session) {
  // TODO: 添加AgentManager的config
  SetConfig(config_builder);
}

void AgentManager::SetConfig(const EgoPlanningConfigBuilder* config_builder) {
  config_ = config_builder->cast<EgoPlanningObstacleManagerConfig>();
}

void AgentManager::Reset() {
  current_agents_.clear();
  current_agents_ids_.clear();
  historical_agents_.clear();
}

const std::vector<std::shared_ptr<Agent>>& AgentManager::GetAllCurrentAgents() const {
  return current_agents_;
}
const std::unordered_set<int32_t>& AgentManager::GetAgentSet() const {
  return current_agents_ids_;
}

void AgentManager::Update(const double start_timestamp_s) {
  current_agents_.clear();
  current_agents_ids_.clear();
  const auto& ego_state =
      *session_->mutable_environmental_model()->get_ego_state_manager();
  double init_time = ego_state.planning_init_point().relative_time;
  const auto& prediction_objects =
      session_->environmental_model().get_prediction_info();
  const double HALF_FOV = 25.0;

  constexpr double kTooOldThreshold = 0.3;
  for (auto iter = historical_agents_.begin();
       iter != historical_agents_.end();) {
    bool is_too_old = (start_timestamp_s - iter->second.back()->timestamp_s()) >
                      kTooOldThreshold;
    bool is_del = is_too_old;
    if (is_del) {
      historical_agents_.erase(iter++);
    } else {
      ++iter;
    }
  }

  for (int i = 0;
       i < session_->environmental_model().get_prediction_info().size(); i++) {
    auto prediction_object = prediction_objects[i];
    // TBD:后续前移至EnvironmentalModelManager::obstacle_prediction_update中
    // Ignore the agent which is within the FOV and is fail to fusion with the
    // camera，or too small, or unknown
    if (prediction_object.type == iflyauto::ObjectType::OBJECT_TYPE_UNKNOWN) {
      LOG_DEBUG("[AgentManager Update] ignore unknown obstacle : [%d] \n",
                prediction_object.id);
      continue;
    }
    bool is_in_fov =
        prediction_object.relative_position_x > 0 &&
        (tan(HALF_FOV) > fabs(prediction_object.relative_position_y /
                              prediction_object.relative_position_x));
    bool is_fusion_with_camera =
        prediction_object.fusion_source & OBSTACLE_SOURCE_CAMERA;
    bool is_ignore_by_fov = is_in_fov && (is_fusion_with_camera == false);
    bool is_ignore_by_size =
        prediction_object.length == 0 || prediction_object.width == 0;

    if (is_ignore_by_fov || is_ignore_by_size) {
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
    std::unordered_map<int32_t, Agent> agent_table;
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
  DeleteOlderAgentInfo();
}

// void AgentManager::Update(
//     const double start_timestamp_s,
//     const std::unordered_map<int32_t, Agent>& agent_table) {
//   current_agents_.clear();
//   current_agents_ids_.clear();
//   constexpr double kTooOldThreshold = 2.0;
//   for (auto iter = historical_agents_.begin();
//        iter != historical_agents_.end();) {
//     bool is_too_old = start_timestamp_s - iter->second.back()->timestamp_s()
//     >
//                       kTooOldThreshold;
//     bool is_del = is_too_old;
//     if (is_del) {
//       historical_agents_.erase(iter++);
//     } else {
//       ++iter;
//     }
//   }

//   for (const auto& data : agent_table) {
//     auto agent = std::make_unique<Agent>(data.second);
//     current_agents_.emplace_back(agent.get());
//     current_agents_ids_.insert(agent->agent_id());
//     historical_agents_[agent->agent_id()].emplace_back(std::move(agent));
//   }
//   DeleteOlderAgent();
// }

void AgentManager::Append(
    const std::unordered_map<int32_t, Agent>& agent_table) {
  for (const auto& data : agent_table) {
    if (current_agents_ids_.count(data.first) > 0) {
      continue;
    }
    auto agent = std::make_shared<Agent>(data.second);
    if (nullptr == agent) {
      continue;
    }
    current_agents_.emplace_back(agent);
    current_agents_ids_.insert(agent->agent_id());

    if (historical_agents_info_.find(agent->agent_id()) ==
        historical_agents_info_.end()) {
      historical_agents_info_[agent->agent_id()] = {};
    }
    historical_agents_info_[agent->agent_id()].emplace_back(agent);
    RecalculateDecelTrajectories(agent, historical_agents_info_);
    historical_agents_[agent->agent_id()].emplace_back(std::move(agent));
  }
}

void AgentManager::DeleteOlderAgent() {
  // pop the oldest obstacle while duration > obstacle_max_duration
  constexpr double kMaxDurationS = 0.3;
  constexpr int32_t kMaxNum = 3;
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

void AgentManager::DeleteOlderAgentInfo() {
  constexpr int32_t kMaxHistoryNum = 3;
  std::vector<int32_t> ids_to_remove;
  for (const auto& entry : historical_agents_info_) {
    int32_t agent_id = entry.first;

    if (current_agents_ids_.find(agent_id) == current_agents_ids_.end()) {
      ids_to_remove.push_back(agent_id);
    }
  }

  for (int32_t id : ids_to_remove) {
    historical_agents_info_.erase(id);
  }

  for (auto& entry : historical_agents_info_) {
    auto& history_list = entry.second;

    if (history_list.size() > kMaxHistoryNum) {
      size_t excess_count = history_list.size() - kMaxHistoryNum;

      auto it = history_list.begin();
      std::advance(it, excess_count);
      history_list.erase(history_list.begin(), it);
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

void AgentManager::RecalculateDecelTrajectories(
    const std::shared_ptr<Agent>& agent,
    const std::unordered_map<int32_t, std::list<std::shared_ptr<Agent>>>&
        historical_agents_info) {
  if (!agent || agent->trajectories().empty() ||
      agent->trajectories().front().empty()) {
    return;
  }

  if (!agent->is_vehicle_type()) {
    return;
  }

  constexpr int kPlanPoints = 26;
  constexpr double kTimeStep = 0.2;
  const double kAlpha = config_.processed_trajectory_filter_alpha;
  const double kDecelThreshold = config_.processed_trajectory_acc_thr;
  constexpr int kMinContinuousFrames = 3;

  const int32_t agent_id = agent->agent_id();
  auto hist_iter = historical_agents_info.find(agent_id);
  if (hist_iter == historical_agents_info.end() ||
      hist_iter->second.size() < kMinContinuousFrames) {
    return;
  }

  bool has_continuous_decel = true;
  int frame_count = 0;
  for (auto agent_iter = hist_iter->second.rbegin();
       agent_iter != hist_iter->second.rend() &&
       frame_count < kMinContinuousFrames;
       ++agent_iter, ++frame_count) {
    if ((*agent_iter)->accel_fusion() > kDecelThreshold) {
      has_continuous_decel = false;
      break;
    }
  }

  if (!has_continuous_decel) {
    return;
  }

  const auto& trajectory = agent->trajectories().front();
  const double init_accel = agent->accel_fusion();
  const double init_speed = std::max(agent->speed_fusion(), 0.0);

  std::vector<double> kin_positions, kin_speeds, kin_accels;
  kin_positions.reserve(kPlanPoints);
  kin_speeds.reserve(kPlanPoints);
  kin_accels.reserve(kPlanPoints);

  const double stop_time = (init_accel < 0)
                               ? -init_speed / init_accel
                               : std::numeric_limits<double>::max();

  for (int i = 0; i < kPlanPoints; ++i) {
    const double t = i * kTimeStep;

    if (init_accel < 0 && t >= stop_time) {
      kin_positions.push_back((init_speed * init_speed) /
                              (2 * std::fabs(init_accel)));
      kin_speeds.push_back(0.0);
      kin_accels.push_back(0.0);
    } else {
      kin_positions.push_back(init_speed * t + 0.5 * init_accel * t * t);
      kin_speeds.push_back(init_speed + init_accel * t);
      kin_accels.push_back(init_accel);
    }
  }

  std::vector<double> model_positions, model_speeds, model_accels;
  model_positions.reserve(kPlanPoints);
  model_speeds.reserve(kPlanPoints);
  model_accels.reserve(kPlanPoints);

  for (int i = 0; i < kPlanPoints; ++i) {
    const auto& point = trajectory.Evaluate(i * kTimeStep);
    model_positions.push_back(point.s());
    model_speeds.push_back(point.vel());
    model_accels.push_back(point.acc());
  }

  if (std::abs(model_positions.back() - kin_positions.back()) <
      0.2 * init_speed) {
    trajectory::Trajectory result;
    std::vector<trajectory::Trajectory> result_vec;
    result.reserve(kPlanPoints);

    for (int i = 0; i < kPlanPoints; ++i) {
      result.push_back(trajectory.Evaluate(i * kTimeStep));
    }
    result_vec.emplace_back(result);

    agent->set_trajectories_used_by_st_graph(std::move(result_vec));
    return;
  }

  std::vector<double> last_positions, last_speeds, last_accels;
  last_positions.reserve(kPlanPoints);
  last_speeds.reserve(kPlanPoints);
  last_accels.reserve(kPlanPoints);

  if (hist_iter->second.size() >= 2) {
    auto prev_frame_iter = hist_iter->second.end();
    std::advance(prev_frame_iter, -2);
    const auto& prev_frame_trajectories =
        (*prev_frame_iter)->trajectories_used_by_st_graph();
    if (!prev_frame_trajectories.empty()) {
      const auto& last_processed_trajectory = prev_frame_trajectories.front();
      for (int i = 0; i < kPlanPoints; ++i) {
        const auto& point = last_processed_trajectory.Evaluate(i * kTimeStep);
        last_positions.push_back(point.s());
        last_speeds.push_back(point.vel());
        last_accels.push_back(point.acc());
      }
    }
  }

  std::vector<double> fused_positions, fused_speeds, fused_accels;
  fused_positions.reserve(kPlanPoints);
  fused_speeds.reserve(kPlanPoints);
  fused_accels.reserve(kPlanPoints);

  for (int i = 0; i < kPlanPoints; ++i) {
    fused_positions.push_back(kAlpha * last_positions[i] +
                              (1 - kAlpha) * kin_positions[i]);
    fused_speeds.push_back(kAlpha * last_speeds[i] +
                           (1 - kAlpha) * kin_speeds[i]);
    fused_accels.push_back(kAlpha * last_accels[i] +
                           (1 - kAlpha) * kin_accels[i]);
  }

  trajectory::Trajectory result_trajectory;
  std::vector<trajectory::Trajectory> result_trajectory_vec;
  result_trajectory.reserve(kPlanPoints);

  std::map<double, TmpPathPoint> path_cache;
  for (const auto& point : trajectory) {
    path_cache[point.s()] = {point.x(), point.y(), point.theta()};
  }

  for (int i = 0; i < kPlanPoints; ++i) {
    const double s = fused_positions[i];
    auto it_upper = path_cache.upper_bound(s);

    if (it_upper == path_cache.begin()) {
      const auto& point = path_cache.begin()->second;
      result_trajectory.emplace_back(point.x, point.y, point.theta,
                                     fused_speeds[i], fused_accels[i],
                                     i * kTimeStep, 0.0, 0.0, s, 0.0);
      continue;
    }

    auto it_lower = std::prev(it_upper);
    if (it_upper == path_cache.end()) {
      const auto& point = it_lower->second;
      result_trajectory.emplace_back(point.x, point.y, point.theta,
                                     fused_speeds[i], fused_accels[i],
                                     i * kTimeStep, 0.0, 0.0, s, 0.0);
      continue;
    }

    const double s0 = it_lower->first;
    const double s1 = it_upper->first;
    const double ratio = (s - s0) / (s1 - s0);

    const auto& p0 = it_lower->second;
    const auto& p1 = it_upper->second;

    const double x = p0.x + ratio * (p1.x - p0.x);
    const double y = p0.y + ratio * (p1.y - p0.y);
    const double theta = p0.theta + ratio * (p1.theta - p0.theta);

    result_trajectory.emplace_back(x, y, theta, fused_speeds[i],
                                   fused_accels[i], i * kTimeStep, 0.0, 0.0, s,
                                   0.0);
  }
  result_trajectory_vec.emplace_back(result_trajectory);
  agent->set_trajectories_used_by_st_graph(std::move(result_trajectory_vec));
}

}  // namespace agent
}  // namespace planning