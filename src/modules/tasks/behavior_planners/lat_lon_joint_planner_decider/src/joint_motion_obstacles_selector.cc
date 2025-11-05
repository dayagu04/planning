#include "joint_motion_obstacles_selector.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "agent/agent.h"
#include "environmental_model.h"
#include "lateral_obstacle.h"
#include "math/box2d.h"
#include "planning_context.h"
#include "utils/kd_path.h"

static const double pi_const = 3.141592654;

namespace planning {

namespace {
constexpr double kEpsilon = 1e-4;
constexpr double kFrontAgentNum = 1;
constexpr size_t kMaxSideAgentNum = 3;
// lane keep
constexpr double kConsiderFrontLonDistance = 150.0;
constexpr double kConsiderRearLonDistance = -20.0;
constexpr double kConsiderLeftLatDistance = 10.0;
constexpr double kConsiderRightLatDistance = -10.0;
// lane change
constexpr double kLCConsiderRearLonDistance = -50.0;
constexpr double kLCLConsiderLeftLatDistance = 15.0;
constexpr double kLCLConsiderRightLatDistance = -5.0;
constexpr double kLCRConsiderLeftLatDistance = 5.0;
constexpr double kLCRConsiderRightLatDistance = -15.0;
// large vehicle threshold
constexpr double kLargeAgentLengthM = 8.0;
// static obstacle filter
constexpr double kStaticSpeedThreshold = 0.5;
// static_agent_later_buffer
constexpr double kStaticAgentLaterBuffer = 0.8;
// rear_agent_long_buffer
constexpr double kRearAgentLonBuffer = 2.0;
// ego speed buffer
constexpr double kEgoSpeedBuffer = 2.0;
}  // namespace
JointMotionObstaclesSelector::JointMotionObstaclesSelector(
    framework::Session* session)
    : session_(session) {}

void JointMotionObstaclesSelector::SelectObstacles(
    const std::vector<JointPlannerTrajectoryPoint>& prior_trajectory,
    int32_t lead_one_id) {
  key_obstacles_.clear();
  surrounding_agents_.clear();

  const auto& agent_manager =
      session_->environmental_model().get_agent_manager();
  const auto& ego_state_manager =
      session_->environmental_model().get_ego_state_manager();
  const auto& virtual_lane_manager =
      session_->environmental_model().get_virtual_lane_manager();
  const auto& dynamic_world =
      session_->environmental_model().get_dynamic_world();
  if (agent_manager == nullptr || ego_state_manager == nullptr ||
      virtual_lane_manager == nullptr || dynamic_world == nullptr) {
    return;
  }

  const auto& agents = agent_manager->GetAllCurrentAgents();
  if (agents.empty()) {
    return;
  }

  const auto& planning_init_point = ego_state_manager->planning_init_point();
  const auto& ego_lane = virtual_lane_manager->get_current_lane();
  if (ego_lane == nullptr) {
    return;
  }

  const auto& ego_reference_path = ego_lane->get_reference_path();
  if (ego_reference_path == nullptr) {
    return;
  }

  const auto& ego_lane_coord = ego_reference_path->get_frenet_coord();
  if (ego_lane_coord == nullptr) {
    return;
  }

  const auto& vehicle_param =
      VehicleConfigurationContext::Instance()->get_vehicle_param();
  const double rear_axle_to_center = vehicle_param.rear_axle_to_center;

  double ego_s = 0.0;
  double ego_l = 0.0;
  if (!ego_lane_coord->XYToSL(planning_init_point.x, planning_init_point.y,
                              &ego_s, &ego_l)) {
    return;
  }

  const auto matched_point = ego_lane_coord->GetPathPointByS(ego_s);
  const double heading_diff = planning_math::NormalizeAngle(
      planning_init_point.heading_angle - matched_point.theta());

  const auto& lane_change_decider_output =
      session_->planning_context().lane_change_decider_output();
  const auto lc_request_direction = lane_change_decider_output.lc_request;
  const auto lane_change_state = lane_change_decider_output.curr_state;
  const bool is_in_lane_change_execution =
      lane_change_state == kLaneChangeExecution ||
      lane_change_state == kLaneChangeComplete ||
      lane_change_state == kLaneChangeHold;

  const int gap_front_node_id =
      lane_change_decider_output.lc_gap_info.front_node_id;
  const int gap_rear_node_id =
      lane_change_decider_output.lc_gap_info.rear_node_id;
  int32_t gap_front_agent_id = -1;
  if (dynamic_world->GetNode(gap_front_node_id)) {
    gap_front_agent_id =
        dynamic_world->GetNode(gap_front_node_id)->node_agent_id();
  }
  int32_t gap_rear_agent_id = -1;
  if (dynamic_world->GetNode(gap_rear_node_id)) {
    gap_rear_agent_id =
        dynamic_world->GetNode(gap_rear_node_id)->node_agent_id();
  }

  std::shared_ptr<agent::Agent> lead_one_agent_shared = nullptr;
  if (lead_one_id >= 0) {
    for (const auto& agent : agents) {
      if (agent != nullptr && agent->agent_id() == lead_one_id) {
        lead_one_agent_shared = agent;
        key_obstacles_.emplace_back(
            CreateKeyObstacle(lead_one_agent_shared, ego_lane_coord, YIELD));
        break;
      }
    }
  }

  const double lon_front = kConsiderFrontLonDistance;
  const double lon_rear = is_in_lane_change_execution
                              ? kLCConsiderRearLonDistance
                              : kConsiderRearLonDistance;

  const auto [lat_left, lat_right] = [&]() {
    if (!is_in_lane_change_execution) {
      return std::pair{kConsiderLeftLatDistance, kConsiderRightLatDistance};
    }
    return (lc_request_direction == LEFT_CHANGE)
               ? std::pair{kLCLConsiderLeftLatDistance,
                           kLCLConsiderRightLatDistance}
               : std::pair{kLCRConsiderLeftLatDistance,
                           kLCRConsiderRightLatDistance};
  }();

  const double ego_half_width = vehicle_param.width * 0.5;

  for (const auto& agent : agents) {
    if (agent == nullptr) {
      continue;
    }

    if (!(agent->fusion_source() & OBSTACLE_SOURCE_CAMERA)) {
      continue;
    }

    double agent_s = 0.0;
    double agent_l = 0.0;
    if (!ego_lane_coord->XYToSL(agent->x(), agent->y(), &agent_s, &agent_l)) {
      continue;
    }

    const double s_diff = agent_s - ego_s;
    const double l_diff = agent_l - ego_l;
    const double ego_speed = planning_init_point.v;

    const bool is_out_of_lon = (s_diff > lon_front) || (s_diff < lon_rear);
    const bool is_out_of_lat = (l_diff > lat_left) || (l_diff < lat_right);
    if (is_out_of_lon || is_out_of_lat) {
      continue;
    }

    const double lateral_distance = std::fabs(l_diff);
    const double lateral_threshold =
        ego_half_width + agent->width() * 0.5 + kStaticAgentLaterBuffer;
    const bool is_laterally_far = lateral_distance > lateral_threshold;
    if (is_laterally_far && agent->is_static()) {
      continue;
    }

    surrounding_agents_.emplace_back(agent);
  }

  if (is_in_lane_change_execution) {
    std::shared_ptr<agent::Agent> gap_front_agent_shared = nullptr;
    std::shared_ptr<agent::Agent> gap_rear_agent_shared = nullptr;

    for (const auto& agent : surrounding_agents_) {
      if (agent == nullptr) {
        continue;
      }
      const int32_t agent_id = agent->agent_id();

      if (gap_front_agent_id >= 0 && agent_id == gap_front_agent_id) {
        gap_front_agent_shared = agent;
      }

      if (gap_rear_agent_id >= 0 && agent_id == gap_rear_agent_id) {
        gap_rear_agent_shared = agent;
      }

      if ((gap_front_agent_id < 0 || gap_front_agent_shared != nullptr) &&
          (gap_rear_agent_id < 0 || gap_rear_agent_shared != nullptr)) {
        break;
      }
    }

    if (gap_front_agent_shared != nullptr) {
      bool already_exists = false;
      for (const auto& obs : key_obstacles_) {
        if (obs.agent_id == gap_front_agent_id) {
          already_exists = true;
          break;
        }
      }
      if (!already_exists) {
        key_obstacles_.emplace_back(
            CreateKeyObstacle(gap_front_agent_shared, ego_lane_coord, YIELD));
      }
    }

    if (gap_rear_agent_shared != nullptr) {
      key_obstacles_.emplace_back(
          CreateKeyObstacle(gap_rear_agent_shared, ego_lane_coord, OVERTAKE));
    }
  }

  std::vector<std::shared_ptr<agent::Agent>> front_agents;
  std::vector<std::shared_ptr<agent::Agent>> side_agents;
  for (const auto& agent : surrounding_agents_) {
    double agent_s = 0.0;
    double agent_l = 0.0;
    if (!ego_lane_coord->XYToSL(agent->x(), agent->y(), &agent_s, &agent_l)) {
      continue;
    }

    bool is_in_front = false;
    if (JudgeOverlapWithPriorTrajectory(agent, agent_l, planning_init_point,
                                        ego_lane_coord, prior_trajectory,
                                        &is_in_front)) {
      if (is_in_front) {
        front_agents.push_back(agent);
      } else {
        side_agents.push_back(agent);
      }
    }
  }

  if (front_agents.empty() && side_agents.empty()) {
    return;
  }

  struct AgentSL {
    std::shared_ptr<agent::Agent> agent;
    double s;
    double l;
  };

  auto compute_agent_sl =
      [&ego_lane_coord](
          const std::vector<std::shared_ptr<agent::Agent>>& agents) {
        std::vector<AgentSL> agents_with_sl;
        agents_with_sl.reserve(agents.size());
        for (const auto& agent : agents) {
          double s = 0.0, l = 0.0;
          if (ego_lane_coord->XYToSL(agent->x(), agent->y(), &s, &l)) {
            agents_with_sl.push_back({agent, s, l});
          }
        }
        return agents_with_sl;
      };

  std::vector<AgentSL> front_agents_sl = compute_agent_sl(front_agents);
  std::vector<AgentSL> side_agents_sl = compute_agent_sl(side_agents);

  if (!front_agents_sl.empty()) {
    size_t partial_size =
        std::min(size_t(kFrontAgentNum), front_agents_sl.size());
    std::partial_sort(
        front_agents_sl.begin(), front_agents_sl.begin() + partial_size,
        front_agents_sl.end(),
        [](const AgentSL& a, const AgentSL& b) { return a.s < b.s; });
  }

  if (!side_agents_sl.empty()) {
    size_t partial_size =
        std::min(size_t(kMaxSideAgentNum), side_agents_sl.size());
    std::partial_sort(
        side_agents_sl.begin(), side_agents_sl.begin() + partial_size,
        side_agents_sl.end(), [ego_l](const AgentSL& a, const AgentSL& b) {
          return std::abs(a.l - ego_l) < std::abs(b.l - ego_l);
        });
  }

  std::unordered_set<int32_t> selected_ids;

  auto select_obstacle = [&](const std::shared_ptr<agent::Agent>& agent) {
    if (agent == nullptr) return;
    int32_t agent_id = agent->agent_id();
    if (selected_ids.find(agent_id) == selected_ids.end()) {
      key_obstacles_.emplace_back(CreateKeyObstacle(agent, ego_lane_coord));
      selected_ids.insert(agent_id);
    }
  };

  for (const auto& obs : key_obstacles_) {
    selected_ids.insert(obs.agent_id);
  }

  for (size_t i = 0; i < front_agents_sl.size() && i < kFrontAgentNum; ++i) {
    select_obstacle(front_agents_sl[i].agent);
  }

  for (size_t i = 0; i < side_agents_sl.size() && i < kMaxSideAgentNum; ++i) {
    select_obstacle(side_agents_sl[i].agent);
  }

  if (key_obstacles_.empty()) {
    return;
  }

  key_obstacles_.erase(
      std::remove_if(key_obstacles_.begin(), key_obstacles_.end(),
                     [](const KeyObstacle& obs) {
                       return obs.init_s == std::numeric_limits<double>::max();
                     }),
      key_obstacles_.end());
}

void JointMotionObstaclesSelector::CalculateAgentSLBoundary(
    const std::shared_ptr<planning_math::KDPath>& planned_path,
    const planning_math::Box2d& agent_box, double* const ptr_min_s,
    double* const ptr_max_s, double* const ptr_min_l, double* const ptr_max_l) {
  if (ptr_min_s == nullptr || ptr_max_s == nullptr || ptr_min_l == nullptr ||
      ptr_max_l == nullptr) {
    return;
  }
  const auto& all_corners = agent_box.GetAllCorners();
  for (const auto& corner : all_corners) {
    double agent_s = 0.0;
    double agent_l = 0.0;
    planned_path->XYToSL(corner.x(), corner.y(), &agent_s, &agent_l);
    *ptr_min_s = std::fmin(*ptr_min_s, agent_s);
    *ptr_max_s = std::fmax(*ptr_max_s, agent_s);
    *ptr_min_l = std::fmin(*ptr_min_l, agent_l);
    *ptr_max_l = std::fmax(*ptr_max_l, agent_l);
  }
}

bool JointMotionObstaclesSelector::JudgeOverlapWithPriorTrajectory(
    const std::shared_ptr<agent::Agent>& agent, const double agent_l,
    const PlanningInitPoint init_point,
    const std::shared_ptr<planning_math::KDPath>& planned_path,
    const std::vector<JointPlannerTrajectoryPoint>& prior_trajectory,
    bool* is_in_front) {
  const auto& vehicle_param =
      VehicleConfigurationContext::Instance()->get_vehicle_param();
  const double ego_length = vehicle_param.length;
  const double ego_width = vehicle_param.width;
  const double ego_half_width = ego_width * 0.5;
  const double rear_axle_to_center = vehicle_param.rear_axle_to_center;
  const auto& trajectories = agent->trajectories_used_by_st_graph();
  if (trajectories.empty()) {
    return false;
  }
  const auto& trajectory = trajectories.front();
  if (trajectory.empty()) {
    return false;
  }

  const bool is_large_vehicle = agent->length() > kLargeAgentLengthM ||
                                agent->type() == agent::AgentType::BUS ||
                                agent->type() == agent::AgentType::TRUCK ||
                                agent->type() == agent::AgentType::TRAILER;

  constexpr double kLatConflictThreshold = 0.30;
  constexpr double kLargeVehicleLatConflictThreshold = 0.40;
  const double lateral_threshold = is_large_vehicle
                                       ? kLargeVehicleLatConflictThreshold
                                       : kLatConflictThreshold;

  constexpr double kLatFrontThreshold = 0.20;

  bool is_directly_in_front = false;

  constexpr double kTimeStep = 0.2;
  constexpr double kMaxTime = 3.0;

  for (double relative_time = 0.0; relative_time <= kMaxTime;
       relative_time += kTimeStep) {
    size_t index = static_cast<size_t>(relative_time / kTimeStep);
    if (index >= prior_trajectory.size()) {
      continue;
    }

    const auto& ego_point = prior_trajectory[index];
    double ego_x = ego_point.x;
    double ego_y = ego_point.y;
    double ego_theta = ego_point.theta;
    double ego_vel = ego_point.vel;

    const double center_x = ego_x + std::cos(ego_theta) * rear_axle_to_center;
    const double center_y = ego_y + std::sin(ego_theta) * rear_axle_to_center;

    planning_math::Box2d ego_box(planning_math::Vec2d(center_x, center_y),
                                 ego_theta, ego_length, ego_width);

    double ego_min_s = std::numeric_limits<double>::max();
    double ego_max_s = std::numeric_limits<double>::lowest();
    double ego_min_l = std::numeric_limits<double>::max();
    double ego_max_l = std::numeric_limits<double>::lowest();
    CalculateAgentSLBoundary(planned_path, ego_box, &ego_min_s, &ego_max_s,
                             &ego_min_l, &ego_max_l);

    const auto agent_point = trajectory.Evaluate(relative_time);
    double agent_vel = agent_point.vel();

    planning_math::Box2d obs_box(
        planning_math::Vec2d(agent_point.x(), agent_point.y()),
        agent_point.theta(), agent->length(), agent->width());

    double obs_min_s = std::numeric_limits<double>::max();
    double obs_max_s = std::numeric_limits<double>::lowest();
    double obs_min_l = std::numeric_limits<double>::max();
    double obs_max_l = std::numeric_limits<double>::lowest();
    CalculateAgentSLBoundary(planned_path, obs_box, &obs_min_s, &obs_max_s,
                             &obs_min_l, &obs_max_l);

    double lateral_dist = 0.0;
    if (obs_min_l >= ego_max_l) {
      lateral_dist = obs_min_l - ego_max_l;
    } else if (obs_max_l <= ego_min_l) {
      lateral_dist = ego_min_l - obs_max_l;
    } else {
      lateral_dist = 0.0;
    }

    double longitudinal_distance = obs_min_s - ego_max_s;
    bool is_rear_agent = obs_max_s < ego_min_s;

    if (relative_time < kEpsilon) {
      bool is_directly_behind =
          is_rear_agent && (lateral_dist <= ego_half_width);
      if (is_directly_behind) {
        return false;
      }

      if (!is_rear_agent && lateral_dist <= ego_half_width) {
        is_directly_in_front = true;
      }
    }

    constexpr double kBaseLongitudinalThreshold = 3.5;
    double longitudinal_threshold =
        kBaseLongitudinalThreshold + ego_vel * 0.3;

    if (lateral_dist <= lateral_threshold &&
        longitudinal_distance < longitudinal_threshold) {
      if (is_in_front != nullptr) {
        *is_in_front = is_directly_in_front;
      }
      return true;
    }
  }
  return false;
}

KeyObstacle JointMotionObstaclesSelector::CreateKeyObstacle(
    const std::shared_ptr<agent::Agent>& agent,
    const std::shared_ptr<planning_math::KDPath>& ego_lane_coord,
    LongitudinalLabel longitudinal_label) {
  KeyObstacle key_obstacle;

  key_obstacle.agent_id = agent->agent_id();
  key_obstacle.longitudinal_label = longitudinal_label;
  key_obstacle.length = agent->length();
  key_obstacle.width = agent->width();

  key_obstacle.init_x = agent->x();
  key_obstacle.init_y = agent->y();
  key_obstacle.init_theta = agent->theta();
  key_obstacle.init_vel = agent->speed();
  key_obstacle.init_acc = agent->accel_fusion();

  key_obstacle.init_s = 0.0;
  key_obstacle.init_l = 0.0;
  if (!ego_lane_coord->XYToSL(key_obstacle.init_x, key_obstacle.init_y,
                              &key_obstacle.init_s, &key_obstacle.init_l)) {
    key_obstacle.init_s = std::numeric_limits<double>::max();
    key_obstacle.init_l = std::numeric_limits<double>::max();
  }

  const auto& primary_trajectories = agent->trajectories_used_by_st_graph();
  const auto& fallback_trajectories = agent->trajectories();

  const auto& trajectories = primary_trajectories.empty()
                                 ? fallback_trajectories
                                 : primary_trajectories;

  if (trajectories.empty()) {
    key_obstacle.init_delta = 0.0;
    return key_obstacle;
  }
  const auto& trajectory = trajectories.front();

  const auto& ego_state_manager =
      session_->environmental_model().get_ego_state_manager();
  const auto& planning_init_point = ego_state_manager->planning_init_point();

  double ego_s = 0.0;
  double ego_l = 0.0;
  if (!ego_lane_coord->XYToSL(planning_init_point.x, planning_init_point.y,
                              &ego_s, &ego_l)) {
    return key_obstacle;
  }

  double init_s = 0.0;
  const auto& vehicle_param =
      VehicleConfigurationContext::Instance()->get_vehicle_param();
  init_s = key_obstacle.init_s - ego_s - key_obstacle.length * 0.5 -
           vehicle_param.front_edge_to_rear_axle;

  key_obstacle.init_s = init_s;
  const double kPlanningTimeStep = 0.2;

  for (size_t i = 0; i < trajectory.size(); ++i) {
    const double relative_time = i * kPlanningTimeStep;
    const auto point = trajectory.Evaluate(relative_time);
    key_obstacle.ref_x_vec.emplace_back(point.x());
    key_obstacle.ref_y_vec.emplace_back(point.y());
    key_obstacle.ref_vel_vec.emplace_back(point.vel());
    key_obstacle.ref_acc_vec.emplace_back(point.acc());
    key_obstacle.ref_s_vec.emplace_back(key_obstacle.init_s + point.s());
  }

  const double obs_wheel_base = key_obstacle.length * 0.75;
  const double max_steer_angle = 360 / 57.3 / 13.0;

  double theta_accumulated = 0.0;
  for (size_t i = 0; i < trajectory.size(); ++i) {
    const auto point = trajectory.Evaluate(i * kPlanningTimeStep);

    double theta_raw = 0.0;
    if (i == 0) {
      if (trajectory.size() > 1) {
        const auto next = trajectory.Evaluate(kPlanningTimeStep);
        double dx = next.x() - point.x();
        double dy = next.y() - point.y();
        if (std::hypot(dx, dy) > 1e-6) {
          theta_raw = std::atan2(dy, dx);
        } else {
          theta_raw = key_obstacle.init_theta;
        }
      } else {
        theta_raw = key_obstacle.init_theta;
      }
      theta_accumulated = theta_raw;
    } else {
      const auto prev = trajectory.Evaluate((i - 1) * kPlanningTimeStep);
      double dx = point.x() - prev.x();
      double dy = point.y() - prev.y();
      if (std::hypot(dx, dy) > 1e-6) {
        theta_raw = std::atan2(dy, dx);
        double theta_prev = key_obstacle.ref_theta_vec.back();
        double dtheta = planning_math::NormalizeAngle(theta_raw - theta_prev);
        theta_accumulated = theta_prev + dtheta;
      } else {
        theta_accumulated = key_obstacle.ref_theta_vec.back();
      }
    }

    double theta = theta_accumulated;

    double curvature = 0.0;
    if (i > 0 && i + 1 < trajectory.size()) {
      const auto prev = trajectory.Evaluate((i - 1) * kPlanningTimeStep);
      const auto next = trajectory.Evaluate((i + 1) * kPlanningTimeStep);
      double dx1 = point.x() - prev.x();
      double dy1 = point.y() - prev.y();
      double dx2 = next.x() - point.x();
      double dy2 = next.y() - point.y();
      double ds1 = std::hypot(dx1, dy1);
      double ds2 = std::hypot(dx2, dy2);
      if (ds1 > 1e-6 && ds2 > 1e-6) {
        double theta1 = std::atan2(dy1, dx1);
        double theta2 = std::atan2(dy2, dx2);
        double dtheta = planning_math::NormalizeAngle(theta2 - theta1);
        curvature = dtheta / ((ds1 + ds2) * 0.5);
      }
    }

    double delta = std::atan(obs_wheel_base * curvature);
    delta = std::clamp(delta, -max_steer_angle, max_steer_angle);

    key_obstacle.ref_theta_vec.emplace_back(theta);
    key_obstacle.ref_delta_vec.emplace_back(delta);
  }

  if (!key_obstacle.ref_delta_vec.empty()) {
    key_obstacle.init_delta = key_obstacle.ref_delta_vec[0];
  } else {
    key_obstacle.init_delta = 0.0;
  }

  if (!key_obstacle.ref_theta_vec.empty()) {
    key_obstacle.init_theta = key_obstacle.ref_theta_vec[0];
  }

  return key_obstacle;
}

std::vector<KeyObstacle> JointMotionObstaclesSelector::GetKeyObstacles() const {
  return key_obstacles_;
}

}  // namespace planning