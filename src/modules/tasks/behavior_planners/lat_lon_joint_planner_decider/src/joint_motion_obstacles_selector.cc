#include "joint_motion_obstacles_selector.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <unordered_map>
#include <vector>

#include "agent/agent.h"
#include "debug_info_log.h"
#include "dynamic_world/dynamic_agent_node.h"
#include "environmental_model.h"
#include "library/lc_pure_pursuit_lib/include/basic_pure_pursuit_model.h"
#include "math/box2d.h"
#include "planning_context.h"
#include "utils/kd_path.h"
#include "vehicle_model_simulation.h"

static const double pi_const = 3.141592654;

namespace planning {

namespace {
constexpr double kEpsilon = 1e-4;
constexpr double kConsiderFrontLonDistance = 150.0;
constexpr double kConsiderRearLonDistance = -20.0;
constexpr double kConsiderLeftLatDistance = 10.0;
constexpr double kConsiderRightLatDistance = -10.0;
constexpr double kLCConsiderRearLonDistance = -50.0;
constexpr double kLCLConsiderLeftLatDistance = 15.0;
constexpr double kLCLConsiderRightLatDistance = -5.0;
constexpr double kLCRConsiderLeftLatDistance = 5.0;
constexpr double kLCRConsiderRightLatDistance = -15.0;
constexpr double kLargeAgentLengthM = 8.0;
constexpr double kStaticSpeedThreshold = 0.5;
constexpr double kRearAgentLonBuffer = 2.0;
constexpr double kEgoSpeedBuffer = 2.0;
constexpr int kMaxFrontObstacles = 1;
constexpr int kMaxSideObstacles = 3;
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

  const auto& planning_init_point = ego_state_manager->planning_init_point();
  double ego_s = 0.0, ego_l = 0.0;
  if (!ego_lane_coord->XYToSL(planning_init_point.x, planning_init_point.y,
                              &ego_s, &ego_l)) {
    return;
  }

  const auto& route_info = session_->environmental_model().get_route_info();
  const auto& ego_lane_road_right_output =
      session_->planning_context().ego_lane_road_right_decider_output();
  bool is_confluence_area = false;

  if (route_info != nullptr) {
    const auto& ego_state =
        session_->environmental_model().get_ego_state_manager();
    const double ego_v = ego_state->ego_v();
    const double dis_threshold = ego_v * 10.0;
    const auto& route_info_output = route_info->get_route_info_output();
    bool is_closing_split =
        route_info_output.mlc_decider_scene_type_info.mlc_scene_type ==
            SPLIT_SCENE &&
        route_info_output.mlc_decider_scene_type_info.dis_to_link_topo_change_point <
            dis_threshold;
    bool is_closing_merge =
        route_info_output.mlc_decider_scene_type_info.mlc_scene_type ==
            MERGE_SCENE &&
        route_info_output.mlc_decider_scene_type_info.dis_to_link_topo_change_point <
            dis_threshold;
    if (is_closing_merge || is_closing_split) {
      is_confluence_area = true;
    }
  }
  if (ego_lane_road_right_output.is_merge_region ||
      ego_lane_road_right_output.is_split_region) {
    is_confluence_area = true;
  }

  std::unordered_map<int32_t, bool> neighbor_agents_map;

  if (is_confluence_area && dynamic_world != nullptr) {
    const int64_t ego_left_front_node_id =
        dynamic_world->ego_left_front_node_id();
    const int64_t ego_left_node_id = dynamic_world->ego_left_node_id();
    const int64_t ego_right_front_node_id =
        dynamic_world->ego_right_front_node_id();
    const int64_t ego_right_node_id = dynamic_world->ego_right_node_id();

    if (ego_left_front_node_id != planning_data::kInvalidId) {
      auto* node = dynamic_world->GetNode(ego_left_front_node_id);
      if (node != nullptr && node->node_agent_id() != -1) {
        neighbor_agents_map[node->node_agent_id()] = true;
      }
    }
    if (ego_left_node_id != planning_data::kInvalidId) {
      auto* node = dynamic_world->GetNode(ego_left_node_id);
      if (node != nullptr && node->node_agent_id() != -1) {
        neighbor_agents_map[node->node_agent_id()] = true;
      }
    }

    if (ego_right_front_node_id != planning_data::kInvalidId) {
      auto* node = dynamic_world->GetNode(ego_right_front_node_id);
      if (node != nullptr && node->node_agent_id() != -1) {
        neighbor_agents_map[node->node_agent_id()] = false;
      }
    }
    if (ego_right_node_id != planning_data::kInvalidId) {
      auto* node = dynamic_world->GetNode(ego_right_node_id);
      if (node != nullptr && node->node_agent_id() != -1) {
        neighbor_agents_map[node->node_agent_id()] = false;
      }
    }
  }

  std::unordered_map<int32_t, KeyObstacle> agent_obstacles_map;
  const double lon_front = kConsiderFrontLonDistance;
  const double lon_rear = kConsiderRearLonDistance;
  const double lat_left = kConsiderLeftLatDistance;
  const double lat_right = kConsiderRightLatDistance;

  for (const auto& agent : agents) {
    if (agent == nullptr ||
        !(agent->fusion_source() & OBSTACLE_SOURCE_CAMERA)) {
      continue;
    }

    double agent_s = 0.0, agent_l = 0.0;
    if (!ego_lane_coord->XYToSL(agent->x(), agent->y(), &agent_s, &agent_l)) {
      continue;
    }

    const double s_diff = agent_s - ego_s;
    const double l_diff = agent_l - ego_l;

    if ((s_diff > lon_front) || (s_diff < lon_rear) || (l_diff > lat_left) ||
        (l_diff < lat_right)) {
      continue;
    }

    surrounding_agents_.emplace_back(agent);

    int32_t agent_id = agent->agent_id();
    auto it = neighbor_agents_map.find(agent_id);
    if (it != neighbor_agents_map.end()) {
      KeyObstacle obstacle = CreateKeyObstacle(agent, ego_lane_coord, IGNORE);
      CorrectTrajectoryInConfluenceArea(obstacle, it->second);
      agent_obstacles_map[agent_id] = obstacle;
    } else {
      agent_obstacles_map[agent_id] =
          CreateKeyObstacle(agent, ego_lane_coord, IGNORE);
    }
  }

  std::vector<int32_t> front_agent_ids;
  std::vector<int32_t> side_agent_ids;

  for (const auto& agent : surrounding_agents_) {
    if (agent == nullptr) {
      continue;
    }

    int32_t agent_id = agent->agent_id();
    auto it = agent_obstacles_map.find(agent_id);
    if (it == agent_obstacles_map.end()) {
      continue;
    }

    double agent_s = 0.0, agent_l = 0.0;
    ego_lane_coord->XYToSL(agent->x(), agent->y(), &agent_s, &agent_l);

    bool is_in_front = false;
    if (JudgeOverlapWithPriorTrajectory(it->second, agent_l,
                                        planning_init_point, ego_lane_coord,
                                        prior_trajectory, &is_in_front)) {
      if (is_in_front) {
        front_agent_ids.push_back(agent_id);
      } else {
        side_agent_ids.push_back(agent_id);
      }
    }
  }

  if (lead_one_id != -1) {
    bool already_in_list =
        std::find(front_agent_ids.begin(), front_agent_ids.end(),
                  lead_one_id) != front_agent_ids.end() ||
        std::find(side_agent_ids.begin(), side_agent_ids.end(), lead_one_id) !=
            side_agent_ids.end();

    if (!already_in_list) {
      for (const auto& agent : agents) {
        if (agent != nullptr && agent->agent_id() == lead_one_id) {
          key_obstacles_.emplace_back(
              CreateKeyObstacle(agent, ego_lane_coord, IGNORE));
          break;
        }
      }
    }
  }

  if (front_agent_ids.empty() && side_agent_ids.empty()) {
    return;
  }

  std::sort(front_agent_ids.begin(), front_agent_ids.end(),
            [&agent_obstacles_map](int32_t a_id, int32_t b_id) {
              return agent_obstacles_map[a_id].init_s <
                     agent_obstacles_map[b_id].init_s;
            });

  std::sort(side_agent_ids.begin(), side_agent_ids.end(),
            [&agent_obstacles_map, ego_l](int32_t a_id, int32_t b_id) {
              double a_l = agent_obstacles_map[a_id].init_l;
              double b_l = agent_obstacles_map[b_id].init_l;
              return std::abs(a_l - ego_l) < std::abs(b_l - ego_l);
            });

  int front_count = 0;
  for (int32_t agent_id : front_agent_ids) {
    if (front_count >= kMaxFrontObstacles) {
      break;
    }
    bool already_exists = false;
    for (const auto& obs : key_obstacles_) {
      if (obs.agent_id == agent_id) {
        already_exists = true;
        break;
      }
    }
    if (!already_exists) {
      key_obstacles_.emplace_back(agent_obstacles_map[agent_id]);
      front_count++;
    }
  }

  int side_count = 0;
  for (int32_t agent_id : side_agent_ids) {
    if (side_count >= kMaxSideObstacles) {
      break;
    }
    bool already_exists = false;
    for (const auto& obs : key_obstacles_) {
      if (obs.agent_id == agent_id) {
        already_exists = true;
        break;
      }
    }
    if (!already_exists) {
      key_obstacles_.emplace_back(agent_obstacles_map[agent_id]);
      side_count++;
    }
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
      ptr_max_l == nullptr || planned_path == nullptr) {
    return;
  }
  const auto& all_corners = agent_box.GetAllCorners();
  for (const auto& corner : all_corners) {
    double agent_s = 0.0;
    double agent_l = 0.0;
    if (!planned_path->XYToSL(corner.x(), corner.y(), &agent_s, &agent_l)) {
      continue;
    }
    *ptr_min_s = std::fmin(*ptr_min_s, agent_s);
    *ptr_max_s = std::fmax(*ptr_max_s, agent_s);
    *ptr_min_l = std::fmin(*ptr_min_l, agent_l);
    *ptr_max_l = std::fmax(*ptr_max_l, agent_l);
  }
}

bool JointMotionObstaclesSelector::JudgeOverlapWithPriorTrajectory(
    const KeyObstacle& key_obstacle, const double agent_l,
    const PlanningInitPoint init_point,
    const std::shared_ptr<planning_math::KDPath>& planned_path,
    const std::vector<JointPlannerTrajectoryPoint>& prior_trajectory,
    bool* is_in_front) {
  const auto& vehicle_param =
      VehicleConfigurationContext::Instance()->get_vehicle_param();
  const double ego_length = vehicle_param.length;
  const double ego_width = vehicle_param.width;
  const double rear_axle_to_center = vehicle_param.rear_axle_to_center;

  if (key_obstacle.ref_x_vec.empty()) {
    return false;
  }

  const bool is_large_vehicle = key_obstacle.length > kLargeAgentLengthM ||
                                key_obstacle.type == agent::AgentType::BUS ||
                                key_obstacle.type == agent::AgentType::TRUCK ||
                                key_obstacle.type == agent::AgentType::TRAILER;

  constexpr double kLatConflictThreshold = 0.3;
  constexpr double kLargeVehicleLatConflictThreshold = 0.4;
  const double lateral_threshold = is_large_vehicle
                                       ? kLargeVehicleLatConflictThreshold
                                       : kLatConflictThreshold;

  bool is_directly_in_front = false;
  const double ego_half_width = ego_width * 0.5;

  constexpr double kTimeStep = 0.2;

  for (size_t index = 0; index < prior_trajectory.size(); ++index) {
    double relative_time = index * kTimeStep;

    size_t ego_index = index;
    if (ego_index >= prior_trajectory.size() - 1) {
      ego_index = prior_trajectory.size() - 1;
    }

    const auto& ego_point = prior_trajectory[ego_index];
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

    size_t obs_index = index;
    if (obs_index >= key_obstacle.ref_x_vec.size()) {
      obs_index = key_obstacle.ref_x_vec.size() - 1;
    }

    double obs_x = key_obstacle.ref_x_vec[obs_index];
    double obs_y = key_obstacle.ref_y_vec[obs_index];
    double obs_theta = key_obstacle.ref_theta_vec[obs_index];
    double obs_vel = key_obstacle.ref_vel_vec[obs_index];

    planning_math::Box2d obs_box(planning_math::Vec2d(obs_x, obs_y), obs_theta,
                                 key_obstacle.length, key_obstacle.width);

    double obs_min_s = std::numeric_limits<double>::max();
    double obs_max_s = std::numeric_limits<double>::lowest();
    double obs_min_l = std::numeric_limits<double>::max();
    double obs_max_l = std::numeric_limits<double>::lowest();
    CalculateAgentSLBoundary(planned_path, obs_box, &obs_min_s, &obs_max_s,
                             &obs_min_l, &obs_max_l);

    double lateral_distance = 0.0;
    if (obs_min_l >= ego_max_l) {
      lateral_distance = obs_min_l - ego_max_l;
    } else if (obs_max_l <= ego_min_l) {
      lateral_distance = ego_min_l - obs_max_l;
    } else {
      lateral_distance = 0.0;
    }

    double longitudinal_distance = obs_min_s - ego_max_s;

    if (index == 0) {
      bool is_rear_agent = obs_max_s < ego_min_s;
      double obs_center_l = (obs_min_l + obs_max_l) * 0.5;
      bool is_directly_behind =
          is_rear_agent && (std::abs(obs_center_l) < ego_half_width + 0.2);
      if (is_directly_behind) {
        return false;
      }

      if (!is_rear_agent && longitudinal_distance > 0) {
        if (std::abs(obs_center_l) < ego_half_width + 0.2) {
          is_directly_in_front = true;
        }
      }
    }

    constexpr double kBaseLongitudinalThreshold = 3.5;
    double dynamic_longitudinal_threshold =
        kBaseLongitudinalThreshold + ego_vel * 0.3;

    if (lateral_distance <= lateral_threshold &&
        longitudinal_distance < dynamic_longitudinal_threshold) {
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
  key_obstacle.type = agent->type();

  key_obstacle.init_x = agent->x();
  key_obstacle.init_y = agent->y();
  key_obstacle.init_theta = agent->theta();
  key_obstacle.init_vel = agent->speed();
  key_obstacle.init_acc = agent->accel_fusion();

  key_obstacle.init_s = 0.0;
  key_obstacle.init_l = 0.0;
  if (ego_lane_coord == nullptr ||
      !ego_lane_coord->XYToSL(key_obstacle.init_x, key_obstacle.init_y,
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
      const auto next = trajectory.Evaluate(kPlanningTimeStep);
      double dx = next.x() - point.x();
      double dy = next.y() - point.y();
      if (std::hypot(dx, dy) > 1e-6) {
        theta_raw = std::atan2(dy, dx);
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

void JointMotionObstaclesSelector::CorrectTrajectoryInConfluenceArea(
    KeyObstacle& key_obstacle, bool is_left_side) {
  if (key_obstacle.ref_x_vec.empty()) {
    return;
  }

  const auto& virtual_lane_manager =
      session_->environmental_model().get_virtual_lane_manager();
  if (virtual_lane_manager == nullptr) {
    return;
  }

  const auto& current_lane = virtual_lane_manager->get_current_lane();
  if (current_lane == nullptr) {
    return;
  }

  const auto& current_lane_coord = current_lane->get_lane_frenet_coord();
  if (current_lane_coord == nullptr) {
    return;
  }

  std::shared_ptr<VirtualLane> candidate_lane = nullptr;
  if (is_left_side) {
    candidate_lane = virtual_lane_manager->get_left_lane();
  } else {
    candidate_lane = virtual_lane_manager->get_right_lane();
  }

  if (candidate_lane == nullptr) {
    return;
  }

  const auto& candidate_lane_coord = candidate_lane->get_lane_frenet_coord();
  if (candidate_lane_coord == nullptr) {
    return;
  }

  constexpr double kTargetTime = 3.0;
  constexpr double kPlanningTimeStep = 0.2;
  const size_t target_time_index =
      static_cast<size_t>(kTargetTime / kPlanningTimeStep);

  if (target_time_index >= key_obstacle.ref_x_vec.size()) {
    return;
  }

  const double check_x = key_obstacle.ref_x_vec[target_time_index];
  const double check_y = key_obstacle.ref_y_vec[target_time_index];

  double check_s_current = 0.0, check_l_current = 0.0;

  std::shared_ptr<VirtualLane> target_lane = nullptr;
  if (current_lane_coord->XYToSL(check_x, check_y, &check_s_current,
                                 &check_l_current) &&
      std::fabs(check_l_current) < current_lane->width() * 0.5) {
    target_lane = current_lane;
  } else if (candidate_lane_coord->XYToSL(check_x, check_y, &check_s_current,
                                          &check_l_current) &&
             std::fabs(check_l_current) < candidate_lane->width() * 0.5) {
    target_lane = candidate_lane;
  } else {
    return;
  }

  if (target_lane == nullptr) {
    return;
  }

  const auto& reference_path_ptr = target_lane->get_reference_path();
  if (reference_path_ptr == nullptr) {
    return;
  }

  const auto& frenet_coord = reference_path_ptr->get_frenet_coord();
  if (frenet_coord == nullptr) {
    return;
  }

  const double obs_wheelbase = key_obstacle.length * 0.75;

  const size_t traj_size = key_obstacle.ref_x_vec.size();

  if (traj_size < 2) {
    return;
  }

  planning::BasicPurePursuitModel pp_model;
  if (pp_model.ProcessReferencePath(reference_path_ptr) !=
      ErrorType::kSuccess) {
    return;
  }

  planning::BasicPurePursuitModel::ModelState pp_state(
      key_obstacle.init_x, key_obstacle.init_y, key_obstacle.init_theta,
      key_obstacle.init_vel);

  for (size_t i = 1; i < traj_size; ++i) {
    double current_vel = pp_state.vel;
    double ld = std::max(3.0, current_vel * 1.2);
    planning::BasicPurePursuitModel::ModelParam pp_param(ld, obs_wheelbase);

    pp_model.set_model_state(pp_state);
    pp_model.set_model_param(pp_param);

    if (pp_model.CalculateDesiredDelta(0.0) != ErrorType::kSuccess) {
      break;
    }

    double desired_delta = pp_model.get_delta();
    constexpr double kMaxSteerAngle = 0.5;
    desired_delta = std::clamp(desired_delta, -kMaxSteerAngle, kMaxSteerAngle);

    pnc::steerModel::VehicleSimulation vehicle_simulate;
    pnc::steerModel::VehicleParameter vehicle_param;
    vehicle_param.c1_ = 1.0 / obs_wheelbase;

    pnc::steerModel::VehicleState vehicle_state{pp_state.x, pp_state.y,
                                                pp_state.theta};
    pnc::steerModel::VehicleControl vehicle_control{pp_state.vel,
                                                    desired_delta};

    vehicle_simulate.Init(vehicle_state);
    vehicle_simulate.Update(vehicle_control, vehicle_param);
    const auto new_state = vehicle_simulate.GetState();

    key_obstacle.ref_x_vec[i] = new_state.x_;
    key_obstacle.ref_y_vec[i] = new_state.y_;

    double theta_value = new_state.phi_;
    if (i > 0) {
      double last_theta = key_obstacle.ref_theta_vec[i - 1];
      double dtheta = planning_math::NormalizeAngle(theta_value - last_theta);
      theta_value = last_theta + dtheta;
    }
    key_obstacle.ref_theta_vec[i] = theta_value;
    key_obstacle.ref_delta_vec[i] = desired_delta;

    pp_state.x = new_state.x_;
    pp_state.y = new_state.y_;
    pp_state.theta = theta_value;
    pp_state.vel = key_obstacle.ref_vel_vec[i];
  }
}

std::vector<KeyObstacle> JointMotionObstaclesSelector::GetKeyObstacles() const {
  return key_obstacles_;
}

}  // namespace planning
