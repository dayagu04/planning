#include "agent_manager.h"

#include <cmath>
#include <cstdint>
#include <limits>
#include <map>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "agent/agent.h"
#include "common_c.h"
#include "environmental_model.h"
#include "ifly_time.h"
#include "obstacle_manager.h"
#include "session.h"
#include "trajectory/trajectory.h"

namespace planning {
namespace agent {

static constexpr int kPlanPoints = 26;
static constexpr double kTimeStep = 0.2;
static constexpr double kAlpha = 0.4;
static constexpr int kMinHistoryFrameCount = 3;
static constexpr double kTruckLengthThreshold = 8.0;

AgentManager::AgentManager(const EgoPlanningConfigBuilder* config_builder,
                           planning::framework::Session* session)
    : session_(session) {
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

const std::vector<std::shared_ptr<Agent>>& AgentManager::GetAllCurrentAgents()
    const {
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
    auto& prediction_object = prediction_objects[i];
    if (prediction_object.type == iflyauto::ObjectType::OBJECT_TYPE_UNKNOWN) {
      ILOG_DEBUG << "[AgentManager Update] ignore unknown obstacle :["
                 << prediction_object.id << "]";
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
      ILOG_DEBUG << "[obstacle_prediction_update] ignore obstacle :["
                 << prediction_object.id << "]";
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
    bool is_static = prediction_object.is_static;
    bool is_truck =
        (prediction_object.type == iflyauto::ObjectType::OBJECT_TYPE_BUS ||
         prediction_object.type == iflyauto::ObjectType::OBJECT_TYPE_TRUCK ||
         prediction_object.type == iflyauto::ObjectType::OBJECT_TYPE_TRAILER) &&
        prediction_object.length > kTruckLengthThreshold;

    double prediction_relative_time = prediction_object.delay_time - init_time;
    std::unordered_map<int32_t, Agent> agent_table;
    if (prediction_object.trajectory_array.size() == 0) {
      auto agent = Agent(prediction_object, is_static, prediction_relative_time,
                         is_truck);
      agent_table.insert({agent.agent_id(), agent});
      Append(agent_table);
      continue;
    }
    for (int i = 0; i < prediction_object.trajectory_array.size(); ++i) {
      auto agent = Agent(prediction_object, is_static, prediction_relative_time,
                         is_truck);
      agent_table.insert({agent.agent_id(), agent});
      Append(agent_table);
      break;
    }
  }

  const auto& obs_mgr = session_->environmental_model().get_obstacle_manager();
  const auto& occ_obs = obs_mgr->get_occupancy_obstacles().Items();
  std::unordered_map<int32_t, Agent> occ_agent_table;
  for (int i = 0; i < occ_obs.size(); i++) {
    planning::agent::Agent occ_agent;
    occ_agent.set_agent_id(occ_obs[i]->id());
    occ_agent.set_type(agent::AgentType(occ_obs[i]->type()));
    occ_agent.set_x(occ_obs[i]->x_center());
    occ_agent.set_y(occ_obs[i]->y_center());
    occ_agent.set_length(occ_obs[i]->length());
    occ_agent.set_width(occ_obs[i]->width());
    occ_agent.set_fusion_source(1);
    occ_agent.set_is_static(occ_obs[i]->is_static());

    occ_agent.set_speed(occ_obs[i]->velocity());
    occ_agent.set_theta(0.0);
    occ_agent.set_accel(0.0);
    occ_agent.set_time_range({0.0, 5.0});

    occ_agent.set_box(occ_obs[i]->perception_bounding_box());
    occ_agent.set_polygon(occ_obs[i]->perception_polygon());
    occ_agent.set_timestamp_s(0.0);
    occ_agent.set_timestamp_us(0.0);
    occ_agent_table.insert({occ_agent.agent_id(), occ_agent});
    Append(occ_agent_table);
  }

  //add uss obstacles
  const auto& uss_obs = obs_mgr->get_uss_obstacles().Items();
  std::unordered_map<int32_t, Agent> uss_agent_table;
  for (int i = 0; i < uss_obs.size(); i++) {
    planning::agent::Agent uss_agent;
    uss_agent.set_agent_id(uss_obs[i]->id());
    uss_agent.set_type(agent::AgentType(uss_obs[i]->type()));
    uss_agent.set_x(uss_obs[i]->x_center());  // 几何中心
    uss_agent.set_y(uss_obs[i]->y_center());
    uss_agent.set_length(uss_obs[i]->length());
    uss_agent.set_width(uss_obs[i]->width());
    uss_agent.set_fusion_source(1);
    uss_agent.set_is_static(uss_obs[i]->is_static());

    uss_agent.set_speed(uss_obs[i]->velocity());
    uss_agent.set_theta(0.0);
    uss_agent.set_accel(0.0);
    uss_agent.set_time_range({0.0, 5.0});

    uss_agent.set_box(uss_obs[i]->perception_bounding_box());
    uss_agent.set_polygon(uss_obs[i]->perception_polygon());
    uss_agent.set_timestamp_s(0.0);
    uss_agent.set_timestamp_us(0.0);
    uss_agent_table.insert({uss_agent.agent_id(), uss_agent});
    Append(uss_agent_table);
  }
  DeleteOlderAgent();
  DeleteOlderAgentInfo();
}

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
    if (historical_agents_info_.find(agent->agent_id()) !=
        historical_agents_info_.end()) {
      const auto& history_list = historical_agents_info_[agent->agent_id()];
      if (!history_list.empty()) {
        const auto& latest_history_agent = history_list.back();
        if (!latest_history_agent->trajectory_optimized().empty()) {
          agent->set_trajectory_optimized(
              latest_history_agent->trajectory_optimized());
        }
      }
    }
    current_agents_.emplace_back(agent);
    current_agents_ids_.insert(agent->agent_id());

    ProcessPredictionTrajectory(agent);

    if (historical_agents_info_.find(agent->agent_id()) ==
        historical_agents_info_.end()) {
      historical_agents_info_[agent->agent_id()] = {};
    }
    historical_agents_info_[agent->agent_id()].emplace_back(agent);
    historical_agents_[agent->agent_id()].emplace_back(std::move(agent));
  }
}

void AgentManager::DeleteOlderAgent() {
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

void AgentManager::ProcessPredictionTrajectory(std::shared_ptr<Agent>& agent) {
  const auto agent_type = agent->type();
  if (agent_type != AgentType::COUPE && agent_type != AgentType::MINIBUS &&
      agent_type != AgentType::VAN && agent_type != AgentType::BUS &&
      agent_type != AgentType::TRUCK && agent_type != AgentType::TRAILER &&
      agent_type != AgentType::MOTORCYCLE &&
      agent_type != AgentType::MOTORCYCLE_RIDING &&
      agent_type != AgentType::TRICYCLE &&
      agent_type != AgentType::TRICYCLE_RIDING) {
    return;
  }

  if (agent->trajectories().empty() || agent->trajectories()[0].empty()) {
    return;
  }

  auto hist_iter = historical_agents_info_.find(agent->agent_id());
  if (hist_iter == historical_agents_info_.end() ||
      hist_iter->second.size() < static_cast<size_t>(kMinHistoryFrameCount)) {
    return;
  }

  const auto& ego_state =
      *session_->mutable_environmental_model()->get_ego_state_manager();
  double heading_diff = std::abs(agent->theta_fusion() -
                                 ego_state.planning_init_point().heading_angle);
  if (heading_diff > M_PI) {
    heading_diff = 2 * M_PI - heading_diff;
  }
  if (heading_diff * 180.0 / M_PI > 60.0) {
    return;
  }

  const auto& hist_list = hist_iter->second;

  const auto& original_trajectory = agent->trajectories()[0];
  const double init_speed =
      (agent->speed() > 0.0) ? agent->speed() : 0.0;
  const double init_accel = agent->accel_fusion();

  std::map<double, TmpPathPoint> path_cache;
  if (!original_trajectory.empty()) {
    path_cache[0.0] = {original_trajectory[0].x(), original_trajectory[0].y(),
                       original_trajectory[0].theta()};
    double cumulative_s = 0.0;
    for (size_t i = 1; i < original_trajectory.size(); ++i) {
      const auto& prev = original_trajectory[i - 1];
      const auto& curr = original_trajectory[i];
      double dx = curr.x() - prev.x();
      double dy = curr.y() - prev.y();
      cumulative_s += std::sqrt(dx * dx + dy * dy);
      path_cache[cumulative_s] = {curr.x(), curr.y(), curr.theta()};
    }
  }

  auto interpolate_point = [&path_cache](double s) -> TmpPathPoint {
    auto it_upper = path_cache.upper_bound(s);
    if (it_upper == path_cache.begin()) {
      return path_cache.begin()->second;
    } else if (it_upper == path_cache.end()) {
      return path_cache.rbegin()->second;
    } else {
      auto it_lower = std::prev(it_upper);
      double s0 = it_lower->first;
      double s1 = it_upper->first;
      double ratio = (s1 - s0 > 1e-9) ? (s - s0) / (s1 - s0) : 0.0;
      const auto& p0 = it_lower->second;
      const auto& p1 = it_upper->second;
      double theta_diff = p1.theta - p0.theta;
      if (theta_diff > M_PI) {
        theta_diff -= 2.0 * M_PI;
      } else if (theta_diff < -M_PI) {
        theta_diff += 2.0 * M_PI;
      }
      return {p0.x + ratio * (p1.x - p0.x), p0.y + ratio * (p1.y - p0.y),
              p0.theta + ratio * theta_diff};
    }
  };

  std::vector<double> kin_s(kPlanPoints), kin_speeds(kPlanPoints),
      kin_accels(kPlanPoints), kin_x(kPlanPoints), kin_y(kPlanPoints),
      kin_theta(kPlanPoints);

  kin_s[0] = 0.0;
  kin_speeds[0] = init_speed;
  kin_accels[0] = init_accel;

  const double stop_s =
      (init_accel < 0.0 && init_speed > 0.0)
          ? (init_speed * init_speed) / (2.0 * std::fabs(init_accel))
          : std::numeric_limits<double>::max();

  for (size_t i = 1; i < kPlanPoints; ++i) {
    double t = i * kTimeStep;
    double predicted_speed = init_speed + init_accel * t;

    if (predicted_speed <= 0.0) {
      kin_s[i] = stop_s;
      kin_speeds[i] = 0.0;
      kin_accels[i] = 0.0;
    } else {
      kin_s[i] = init_speed * t + 0.5 * init_accel * t * t;
      kin_speeds[i] = predicted_speed;
      kin_accels[i] = init_accel;
    }
  }

  for (size_t i = 0; i < kPlanPoints; ++i) {
    auto pt = interpolate_point(kin_s[i]);
    kin_x[i] = pt.x;
    kin_y[i] = pt.y;
    kin_theta[i] = pt.theta;
  }

  const auto& last_hist_obj = hist_list.back();

  std::vector<double> last_s(kPlanPoints), last_speeds(kPlanPoints),
      last_accels(kPlanPoints), last_x(kPlanPoints), last_y(kPlanPoints),
      last_theta(kPlanPoints);

  if (!last_hist_obj->trajectories().empty() &&
      !last_hist_obj->trajectories()[0].empty()) {
    const auto& last_trajectory =
        last_hist_obj->trajectories_used_by_st_graph()[0];
    for (size_t i = 0; i < kPlanPoints && i < last_trajectory.size(); ++i) {
      last_x[i] = last_trajectory[i].x();
      last_y[i] = last_trajectory[i].y();
      last_theta[i] = last_trajectory[i].theta();
      last_speeds[i] = last_trajectory[i].vel();
      last_accels[i] = last_trajectory[i].acc();
    }
  }

  std::vector<double> fused_x(kPlanPoints), fused_y(kPlanPoints),
      fused_theta(kPlanPoints), fused_speeds(kPlanPoints),
      fused_acc(kPlanPoints);

  for (size_t i = 0; i < kPlanPoints; ++i) {
    fused_x[i] = kAlpha * last_x[i] + (1 - kAlpha) * kin_x[i];
    fused_y[i] = kAlpha * last_y[i] + (1 - kAlpha) * kin_y[i];

    double theta_diff = kin_theta[i] - last_theta[i];
    if (theta_diff > M_PI) {
      theta_diff -= 2.0 * M_PI;
    } else if (theta_diff < -M_PI) {
      theta_diff += 2.0 * M_PI;
    }

    fused_theta[i] = last_theta[i] + (1 - kAlpha) * theta_diff;
    fused_speeds[i] = kAlpha * last_speeds[i] + (1 - kAlpha) * kin_speeds[i];
    fused_acc[i] = kAlpha * last_accels[i] + (1 - kAlpha) * kin_accels[i];
  }

  trajectory::Trajectory processed_trajectory(original_trajectory);

  for (size_t i = 0; i < kPlanPoints && i < processed_trajectory.size(); ++i) {
    double absolute_time = static_cast<double>(i) * kTimeStep;
    processed_trajectory[i].set_x(fused_x[i]);
    processed_trajectory[i].set_y(fused_y[i]);
    processed_trajectory[i].set_theta(fused_theta[i]);
    processed_trajectory[i].set_vel(fused_speeds[i]);
    processed_trajectory[i].set_acc(fused_acc[i]);
    processed_trajectory[i].set_absolute_time(absolute_time);
  }

  std::vector<trajectory::Trajectory> processed_trajectories;
  processed_trajectories.emplace_back(processed_trajectory);
  agent->set_trajectories_used_by_st_graph(processed_trajectories);
}

}  // namespace agent
}  // namespace planning