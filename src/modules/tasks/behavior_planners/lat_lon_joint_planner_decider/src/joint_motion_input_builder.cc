#include "joint_motion_input_builder.h"

#include <algorithm>
#include <cmath>
#include <memory>

#include "common/config/basic_type.h"
#include "environmental_model.h"
#include "lateral_obstacle.h"
#include "math/math_utils.h"
#include "planning_context.h"

namespace planning {

JointMotionInputBuilder::JointMotionInputBuilder(
    const EgoPlanningConfigBuilder* config_builder, framework::Session* session)
    : session_(session) {
  config_ = config_builder->cast<JointMotionPlannerConfig>();
  speed_planning_config_ = config_builder->cast<SpeedPlannerConfig>();
  obstacles_selector_ = std::make_shared<JointMotionObstaclesSelector>(session);
  speed_limit_calculator_ =
      std::make_unique<JointMotionSpeedLimit>(config_builder, session);

  joint_traj_params_.s0 = 3.5;
  joint_traj_params_.a = 1.5;
  joint_traj_params_.b_max = 2.0;
  joint_traj_params_.b = 1.0;
  joint_traj_params_.b_hard = 4.0;
  joint_traj_params_.max_accel_jerk = 3.0;
  joint_traj_params_.delta = 4.0;
  joint_traj_params_.cool_factor = 0.99;
  joint_traj_params_.T = 1.0;
  joint_traj_params_.delay_time_buffer = 0.3;
  joint_traj_params_.min_decel_jerk = 1.0;
  joint_traj_params_.max_decel_jerk = 2.0;
  joint_traj_params_.default_front_distance = 200.0;

  ref_trajectory_.clear();
  ref_trajectory_.reserve(kPlanningTimeSteps);
}

void JointMotionInputBuilder::SetObstaclesSelector(
    std::shared_ptr<JointMotionObstaclesSelector> obstacles_selector) {
  obstacles_selector_ = obstacles_selector;
}

void JointMotionInputBuilder::BuildInput(
    planning::common::JointMotionPlanningInput& planning_input,
    std::shared_ptr<pnc::joint_motion_planning::JointMotionPlanningProblem>
        planning_problem_ptr) {
  lead_one_id_ = -1;
  BuildEgoAndWeightInfo(planning_input);

  key_agent_ids_.clear();
  BuildObsInfo(planning_input);

  JSON_DEBUG_VECTOR("joint_key_agent_ids", key_agent_ids_, 0);

  BuildRoadInfo(planning_input, planning_problem_ptr);
}

void JointMotionInputBuilder::BuildEgoAndWeightInfo(
    planning::common::JointMotionPlanningInput& planning_input) {
  const auto& ego_state_manager =
      session_->environmental_model().get_ego_state_manager();
  const auto& planning_init_point = ego_state_manager->planning_init_point();

  planning_input.mutable_ego_init_state()->set_x(planning_init_point.x);
  planning_input.mutable_ego_init_state()->set_y(planning_init_point.y);
  planning_input.mutable_ego_init_state()->set_theta(
      planning_init_point.heading_angle);
  planning_input.mutable_ego_init_state()->set_delta(planning_init_point.delta);
  planning_input.mutable_ego_init_state()->set_vel(planning_init_point.v);
  planning_input.mutable_ego_init_state()->set_acc(planning_init_point.a);

  const auto& vehicle_param =
      planning::VehicleConfigurationContext::Instance()->get_vehicle_param();
  planning_input.set_ego_length(vehicle_param.length);
  planning_input.set_ego_width(vehicle_param.width);

  ref_trajectory_.clear();
  ref_trajectory_.reserve(kPlanningTimeSteps);
  ref_trajectory_.resize(kPlanningTimeSteps);

  GenerateReferenceTrajectory(planning_input);

  planning_input.mutable_ref_x_vec()->Clear();
  planning_input.mutable_ref_y_vec()->Clear();
  planning_input.mutable_ref_theta_vec()->Clear();
  planning_input.mutable_ref_delta_vec()->Clear();
  planning_input.mutable_ref_vel_vec()->Clear();
  planning_input.mutable_ref_acc_vec()->Clear();
  planning_input.mutable_ref_s_vec()->Clear();

  for (size_t i = 0; i < kPlanningTimeSteps; ++i) {
    planning_input.mutable_ref_x_vec()->Add(ref_trajectory_[i].x);
    planning_input.mutable_ref_y_vec()->Add(ref_trajectory_[i].y);
    planning_input.mutable_ref_theta_vec()->Add(ref_trajectory_[i].theta);
    planning_input.mutable_ref_delta_vec()->Add(ref_trajectory_[i].delta);
    planning_input.mutable_ref_vel_vec()->Add(ref_trajectory_[i].vel);
    planning_input.mutable_ref_acc_vec()->Add(ref_trajectory_[i].acc);
    planning_input.mutable_ref_s_vec()->Add(ref_trajectory_[i].s);
  }

  planning_input.set_q_ego_ref_x(config_.q_ego_ref_x);
  planning_input.set_q_ego_ref_y(config_.q_ego_ref_y);
  planning_input.set_q_ego_ref_theta(config_.q_ego_ref_theta);
  planning_input.set_q_ego_ref_delta(config_.q_ego_ref_delta);
  planning_input.set_q_ego_ref_vel(config_.q_ego_ref_vel);
  planning_input.set_q_ego_ref_acc(config_.q_ego_ref_acc);

  planning_input.mutable_curv_factor_vec()->Clear();
  for (size_t i = 0; i < kPlanningTimeSteps; ++i) {
    double c1 = 1 / std::max(vehicle_param.wheel_base, 1e-6);
    double c2 = 0.0;  // neutral
    double curv_factor =
        pnc::mathlib::GetCurvFactor(c1, c2, ref_trajectory_[i].vel);
    planning_input.mutable_curv_factor_vec()->Add(curv_factor);
  }

  planning_input.set_three_disc_safe_dist(config_.three_disc_safe_dist);
  planning_input.set_q_three_disc_safe_dist_weight(
      config_.q_three_disc_safe_dist_weight);

  planning_input.set_road_boundary_safe_dist(config_.road_boundary_safe_dist);
  planning_input.set_q_road_boundary_weight(config_.q_road_boundary_weight);
  planning_input.set_q_ego_acc_weight(config_.q_ego_acc_weight);
  planning_input.set_q_ego_jerk_weight(config_.q_ego_jerk_weight);
  planning_input.set_q_ego_omega_weight(config_.q_ego_omega_weight);
  planning_input.set_q_ego_delta_weight(config_.q_ego_delta_weight);

  planning_input.set_q_ego_acc_bound_weight(config_.q_ego_acc_bound_weight);
  planning_input.clear_ego_acc_max();
  planning_input.clear_ego_acc_min();
  for (size_t i = 0; i < kPlanningTimeSteps; ++i) {
    planning_input.add_ego_acc_max(config_.ego_acc_max);
    planning_input.add_ego_acc_min(config_.ego_acc_min);
  }

  planning_input.set_q_ego_jerk_bound_weight(config_.q_ego_jerk_bound_weight);
  planning_input.clear_ego_jerk_max();
  planning_input.clear_ego_jerk_min();
  for (size_t i = 0; i < kPlanningTimeSteps; ++i) {
    planning_input.add_ego_jerk_max(config_.ego_jerk_max);
    planning_input.add_ego_jerk_min(-joint_traj_params_.max_decel_jerk);
  }

  planning_input.set_q_hard_halfplane_weight(config_.q_hard_halfplane_weight);
  planning_input.set_hard_halfplane_dist(config_.hard_halfplane_dist);
  planning_input.set_halfplane_cost_allocation_ratio(
      config_.halfplane_cost_allocation_ratio);

  planning_input.set_q_soft_halfplane_weight(config_.q_soft_halfplane_weight);
  planning_input.set_soft_halfplane_s0(config_.soft_halfplane_s0);
  planning_input.set_soft_halfplane_tau(config_.soft_halfplane_tau);
  planning_input.set_soft_halfplane_cost_allocation_ratio(
      config_.soft_halfplane_cost_allocation_ratio);
}

void JointMotionInputBuilder::BuildObsInfo(
    planning::common::JointMotionPlanningInput& planning_input) {
  planning_input.clear_obs_init_state();
  planning_input.clear_obs_ref_trajectory();
  if (obstacles_selector_ != nullptr) {
    obstacles_selector_->SelectObstacles(ref_trajectory_, lead_one_id_);
    const auto& key_obstacles = obstacles_selector_->GetKeyObstacles();
    planning_input.set_obs_num(key_obstacles.size());

    for (const auto& obstacle : key_obstacles) {
      key_agent_ids_.push_back(obstacle.agent_id);
      auto* obs_state = planning_input.add_obs_init_state();
      obs_state->set_x(obstacle.init_x);
      obs_state->set_y(obstacle.init_y);
      obs_state->set_theta(obstacle.init_theta);
      obs_state->set_delta(obstacle.init_delta);
      obs_state->set_vel(obstacle.init_vel);
      obs_state->set_acc(obstacle.init_acc);

      auto* obs_ref_trajectory = planning_input.add_obs_ref_trajectory();
      obs_ref_trajectory->set_length(obstacle.length);
      obs_ref_trajectory->set_width(obstacle.width);
      obs_ref_trajectory->set_longitudinal_label(obstacle.longitudinal_label);
      obs_ref_trajectory->set_obs_id(obstacle.agent_id);
      obs_ref_trajectory->set_init_s(obstacle.init_s);

      const double obs_wheel_base = obstacle.length * 0.75;
      for (size_t i = 0; i < kPlanningTimeSteps; ++i) {
        obs_ref_trajectory->add_ref_x_vec(obstacle.ref_x_vec[i]);
        obs_ref_trajectory->add_ref_y_vec(obstacle.ref_y_vec[i]);
        obs_ref_trajectory->add_ref_theta_vec(obstacle.ref_theta_vec[i]);
        obs_ref_trajectory->add_ref_delta_vec(obstacle.ref_delta_vec[i]);
        obs_ref_trajectory->add_ref_vel_vec(obstacle.ref_vel_vec[i]);
        obs_ref_trajectory->add_ref_acc_vec(obstacle.ref_acc_vec[i]);
        obs_ref_trajectory->add_ref_s_vec(obstacle.ref_s_vec[i]);

        double c1 = 1 / std::max(obs_wheel_base, 1e-6);
        double c2 = 0.0;
        double obs_curv_factor =
            pnc::mathlib::GetCurvFactor(c1, c2, obstacle.ref_vel_vec[i]);
        obs_ref_trajectory->add_curv_factor_vec(obs_curv_factor);
      }
    }
  }
}

void JointMotionInputBuilder::BuildRoadInfo(
    planning::common::JointMotionPlanningInput& planning_input,
    std::shared_ptr<pnc::joint_motion_planning::JointMotionPlanningProblem>
        planning_problem_ptr) {
  const auto& reference_path_ptr = session_->planning_context()
                                       .lane_change_decider_output()
                                       .coarse_planning_info.reference_path;

  double ego_s = reference_path_ptr->get_frenet_ego_state().s();

  std::vector<planning::planning_math::PathPoint> left_road_points;
  std::vector<planning::planning_math::PathPoint> right_road_points;
  left_road_points.reserve(kPlanningTimeSteps);
  right_road_points.reserve(kPlanningTimeSteps);

  const std::shared_ptr<planning::planning_math::KDPath> frenet_coord =
      reference_path_ptr->get_frenet_coord();

  if (frenet_coord == nullptr) {
    return;
  }

  for (size_t i = 0; i < kPlanningTimeSteps; ++i) {
    double current_s = ego_s + ref_trajectory_[i].s;
    planning::ReferencePathPoint target_ref_point;
    if (!reference_path_ptr->get_reference_point_by_lon(current_s,
                                                        target_ref_point)) {
      continue;
    }

    double x, y;
    double left_road_distance =
        std::min(target_ref_point.distance_to_left_road_border, 20.0);
    if (frenet_coord->SLToXY(current_s, left_road_distance, &x, &y)) {
      left_road_points.emplace_back(planning::planning_math::PathPoint(x, y));
    }

    double right_road_distance =
        std::min(target_ref_point.distance_to_right_road_border, 20.0);
    if (frenet_coord->SLToXY(current_s, -right_road_distance, &x, &y)) {
      right_road_points.emplace_back(planning::planning_math::PathPoint(x, y));
    }
  }

  if (!left_road_points.empty() && !right_road_points.empty()) {
    planning_input.mutable_left_road_boundary()->clear_x_vec();
    planning_input.mutable_left_road_boundary()->clear_y_vec();
    planning_input.mutable_right_road_boundary()->clear_x_vec();
    planning_input.mutable_right_road_boundary()->clear_y_vec();

    for (const auto& point : left_road_points) {
      planning_input.mutable_left_road_boundary()->add_x_vec(point.x());
      planning_input.mutable_left_road_boundary()->add_y_vec(point.y());
    }

    for (const auto& point : right_road_points) {
      planning_input.mutable_right_road_boundary()->add_x_vec(point.x());
      planning_input.mutable_right_road_boundary()->add_y_vec(point.y());
    }

    auto left_boundary_path =
        left_road_points.size() < planning_math::KDPath::kKDPathMinPathPointSize
            ? nullptr
            : std::make_shared<planning::planning_math::KDPath>(
                  std::move(left_road_points));
    auto right_boundary_path =
        right_road_points.size() <
                planning_math::KDPath::kKDPathMinPathPointSize
            ? nullptr
            : std::make_shared<planning::planning_math::KDPath>(
                  std::move(right_road_points));

    planning_problem_ptr->SetBoundaryPaths(left_boundary_path,
                                           right_boundary_path);
  }
}

double JointMotionInputBuilder::CalculateIdmAcceleration(
    const double current_acc, const double current_vel, const double current_s,
    const double front_acc, const double front_vel, const double front_s,
    const double tau, const double v0, const double decel_jerk) const {
  constexpr double kEps = 1e-6;
  double s0 = joint_traj_params_.s0;
  double a = joint_traj_params_.a;
  double b_max = joint_traj_params_.b_max;
  double b = joint_traj_params_.b;
  double b_hard = joint_traj_params_.b_hard;
  double max_accel_jerk = joint_traj_params_.max_accel_jerk;
  double delta = joint_traj_params_.delta;
  double cool_factor = joint_traj_params_.cool_factor;

  double s_alpha = std::max(kEps, front_s - current_s);
  double delta_v = current_vel - front_vel;

  double s_star =
      s0 + std::max(0.0, current_vel * tau + (current_vel * delta_v) /
                                                 (2.0 * std::sqrt(a * b_max)));

  double target_v0 = std::max(kEps, v0);

  double a_free;
  if (current_vel <= target_v0) {
    a_free = a * (1.0 - std::pow(current_vel / target_v0, delta));
  } else {
    a_free = -b * (1.0 - std::pow(target_v0 / current_vel, a * delta / b));
  }

  double z = s_star / s_alpha;

  double a_idm;
  if (current_vel <= target_v0) {
    if (z < 1.0 && std::abs(a_free) > kEps) {
      a_idm = a_free * (1.0 - std::pow(z, 2.0 * a / a_free));
    } else {
      a_idm = a * (1.0 - std::pow(z, 2.0));
    }
  } else {
    if (z >= 1.0) {
      a_idm = a_free + a * (1.0 - std::pow(z, 2.0));
    } else {
      a_idm = a_free;
    }
  }

  a_idm = std::max(std::min(a, a_idm), -b_hard);

  double a_cah = -(b * current_vel * current_vel) /
                 (front_vel * front_vel + 2 * s_alpha * b);
  a_cah = std::max(std::min(a, a_cah), -b_hard);

  double final_acc;
  if (a_idm >= a_cah) {
    final_acc = a_idm;
  } else {
    final_acc = (1.0 - cool_factor) * a_idm +
                cool_factor * (a_cah - b * tanh((a_idm - a_cah) / (-b)));
  }

  double acc_change = final_acc - current_acc;
  if (acc_change > 0 && acc_change > max_accel_jerk * kPlanningTimeStep) {
    final_acc = current_acc + max_accel_jerk * kPlanningTimeStep;
  } else if (acc_change < 0 && acc_change < -decel_jerk * kPlanningTimeStep) {
    final_acc = current_acc - decel_jerk * kPlanningTimeStep;
  }

  final_acc = std::max(std::min(a, final_acc), -b_hard);

  return final_acc;
}

void JointMotionInputBuilder::GenerateReferenceTrajectory(
    planning::common::JointMotionPlanningInput& planning_input) {
  const auto& ego_state_manager =
      session_->environmental_model().get_ego_state_manager();
  const auto& planning_init_point = ego_state_manager->planning_init_point();

  const auto& reference_path_ptr = session_->planning_context()
                                       .lane_change_decider_output()
                                       .coarse_planning_info.reference_path;

  if (reference_path_ptr == nullptr) {
    return;
  }

  const auto& coarse_planning_info = session_->planning_context()
                                         .lane_change_decider_output()
                                         .coarse_planning_info;
  const auto& lane_change_decider_output =
      session_->planning_context().lane_change_decider_output();
  const auto lane_change_state = lane_change_decider_output.curr_state;

  const bool is_in_lane_change_condition =
      lane_change_state == kLaneChangeExecution ||
      lane_change_state == kLaneChangeComplete ||
      lane_change_state == kLaneChangeHold;

  bool is_use_spatio_planner_result = false;
  const auto& spatio_temporal_output =
      session_->planning_context().spatio_temporal_union_plan_output();

  const auto& virtual_lane_manager =
      session_->environmental_model().get_virtual_lane_manager();
  const auto intersection_state = virtual_lane_manager->GetIntersectionState();
  const double distance_to_stopline =
      virtual_lane_manager->GetEgoDistanceToStopline();

  const auto& lane_borrow_decider_output =
      session_->planning_context().lane_borrow_decider_output();

  pnc::mathlib::spline x_s_spline, y_s_spline;
  double s_min = 0.0, s_max = 0.0;

  double total_s = 0.0;
  if (!coarse_planning_info.trajectory_points.empty()) {
    total_s = coarse_planning_info.trajectory_points.back().s -
              coarse_planning_info.trajectory_points.front().s;
  }

  if (config_.enable_use_spatio_temporal_planning &&
      spatio_temporal_output.enable_using_st_plan &&
      !coarse_planning_info.trajectory_points.empty() && total_s > 1.0 &&
      !is_in_lane_change_condition &&
      !lane_borrow_decider_output.is_in_lane_borrow_status) {
    is_use_spatio_planner_result = true;
  }

  if ((is_use_spatio_planner_result || is_in_lane_change_condition) &&
      !coarse_planning_info.trajectory_points.empty()) {
    const auto& traj_points = coarse_planning_info.trajectory_points;
    std::vector<double> s_vec, x_vec, y_vec;
    s_vec.reserve(traj_points.size());
    x_vec.reserve(traj_points.size());
    y_vec.reserve(traj_points.size());

    for (size_t i = 0; i < traj_points.size(); ++i) {
      s_vec.push_back(traj_points[i].s);
      x_vec.push_back(traj_points[i].x);
      y_vec.push_back(traj_points[i].y);
    }

    x_s_spline.set_points(s_vec, x_vec);
    y_s_spline.set_points(s_vec, y_vec);
    s_min = s_vec.front();
    s_max = s_vec.back();
  } else {
    const auto& cart_ref_info = coarse_planning_info.cart_ref_info;

    if (cart_ref_info.s_vec.empty() ||
        cart_ref_info.x_s_spline.get_x().empty() ||
        cart_ref_info.y_s_spline.get_x().empty()) {
      return;
    }

    x_s_spline = cart_ref_info.x_s_spline;
    y_s_spline = cart_ref_info.y_s_spline;
    s_min = cart_ref_info.s_vec.front();
    s_max = cart_ref_info.s_vec.back();
  }

  const auto& ego_vehicle_param =
      VehicleConfigurationContext::Instance()->get_vehicle_param();
  const double wheel_base = ego_vehicle_param.wheel_base;
  const double max_steer_angle = ego_vehicle_param.max_steer_angle;
  const double front_edge_to_rear_axle =
      ego_vehicle_param.front_edge_to_rear_axle;
  const double dt = kPlanningTimeStep;

  const auto& ego_lane = session_->environmental_model()
                             .get_virtual_lane_manager()
                             ->get_current_lane();
  if (ego_lane == nullptr || ego_lane->get_lane_frenet_coord() == nullptr) {
    return;
  }
  const auto& ego_lane_coord = ego_lane->get_lane_frenet_coord();
  if (ego_lane_coord == nullptr) {
    return;
  }

  JSON_DEBUG_VALUE("joint_lane_change_state",
                   static_cast<int>(lane_change_state));
  JSON_DEBUG_VALUE("joint_use_spatio_result", is_use_spatio_planner_result);

  const agent::Agent* lead_one_agent = nullptr;
  std::vector<trajectory::TrajectoryPoint> lead_trajectory;

  if (is_in_lane_change_condition) {
    const auto& dynamic_world =
        session_->environmental_model().get_dynamic_world();
    const auto lc_request_direction = lane_change_decider_output.lc_request;

    int64_t target_lane_front_node_id = -1;

    if (lc_request_direction == LEFT_CHANGE) {
      if (lane_change_state == kLaneChangeExecution ||
          lane_change_state == kLaneChangeComplete) {
        target_lane_front_node_id = dynamic_world->ego_front_node_id();
      } else {
        target_lane_front_node_id = dynamic_world->ego_left_front_node_id();
      }
    } else if (lc_request_direction == RIGHT_CHANGE) {
      if (lane_change_state == kLaneChangeExecution ||
          lane_change_state == kLaneChangeComplete) {
        target_lane_front_node_id = dynamic_world->ego_front_node_id();
      } else {
        target_lane_front_node_id = dynamic_world->ego_right_front_node_id();
      }
    }

    if (target_lane_front_node_id != -1) {
      auto* target_lane_front_node =
          dynamic_world->GetNode(target_lane_front_node_id);
      if (target_lane_front_node != nullptr) {
        lead_one_agent =
            session_->environmental_model().get_agent_manager()->GetAgent(
                target_lane_front_node->node_agent_id());
        if (lead_one_agent != nullptr) {
          lead_one_id_ = target_lane_front_node->node_agent_id();
        }
      }
    }
  } else {
    const auto lead_one =
        session_->environmental_model().get_lateral_obstacle()->leadone();
    if (lead_one != nullptr) {
      lead_one_id_ = lead_one->id();
      lead_one_agent =
          session_->environmental_model().get_agent_manager()->GetAgent(
              lead_one_id_);
    }
  }

  double ego_s = 0.0, ego_l = 0.0;
  if (!ego_lane_coord->XYToSL(planning_init_point.x, planning_init_point.y,
                              &ego_s, &ego_l)) {
    return;
  }

  if (lead_one_agent != nullptr) {
    double lead_s = 0.0, lead_l = 0.0;
    if (ego_lane_coord->XYToSL(lead_one_agent->x(), lead_one_agent->y(),
                               &lead_s, &lead_l)) {
      if (lead_one_agent != nullptr) {
        double lead_s = 0.0, lead_l = 0.0;
        if (ego_lane_coord->XYToSL(lead_one_agent->x(), lead_one_agent->y(),
                                   &lead_s, &lead_l)) {
          if (lead_s > ego_s) {
            const auto& primary_trajectories =
                lead_one_agent->trajectories_used_by_st_graph();
            const auto& fallback_trajectories = lead_one_agent->trajectories();

            const auto& selected_trajectories = primary_trajectories.empty()
                                                    ? fallback_trajectories
                                                    : primary_trajectories;

            if (!selected_trajectories.empty() &&
                !selected_trajectories.front().empty()) {
              lead_trajectory = selected_trajectories.front();
            }
          } else {
            lead_one_agent = nullptr;
            lead_one_id_ = -1;
          }
        } else {
          lead_one_agent = nullptr;
          lead_one_id_ = -1;
        }
      } else {
        lead_one_agent = nullptr;
        lead_one_id_ = -1;
      }
    } else {
      lead_one_agent = nullptr;
      lead_one_id_ = -1;
    }
  }

  JSON_DEBUG_VALUE("joint_lead_one_id", lead_one_id_);

  const double cruise_speed = ego_state_manager->ego_v_cruise();
  JSON_DEBUG_VALUE("joint_cruise_speed", cruise_speed);

  auto speed_limit_result = speed_limit_calculator_->CalculateSpeedLimit();
  const double current_limit_speed = speed_limit_result.speed_limit;
  JSON_DEBUG_VALUE("joint_limit_speed", current_limit_speed);

  double current_s = 0.0;
  double current_v = planning_init_point.v;
  double current_a = planning_init_point.a;
  double current_theta = planning_init_point.heading_angle;
  double current_delta = planning_init_point.delta;

  ref_trajectory_[0].t = 0.0;
  ref_trajectory_[0].s = current_s;
  ref_trajectory_[0].vel = current_v;
  ref_trajectory_[0].acc = current_a;
  ref_trajectory_[0].x = planning_init_point.x;
  ref_trajectory_[0].y = planning_init_point.y;
  ref_trajectory_[0].theta = current_theta;
  ref_trajectory_[0].delta = current_delta;

  double v0 = current_limit_speed;
  double front_s = joint_traj_params_.default_front_distance;
  double front_vel = v0;
  double front_acc = 0.0;

  double ref_ego_s = reference_path_ptr->get_frenet_ego_state().s();

  double target_tau = interp(current_v, _EGO_VEL_TABLE, _TIME_HEADWAY_TABLE);

  JSON_DEBUG_VALUE("joint_target_tau", target_tau);

  for (int i = 1; i < kPlanningTimeSteps; ++i) {
    double t = i * kPlanningTimeStep;

    if (!lead_trajectory.empty() && lead_one_agent != nullptr) {
      size_t lead_idx = static_cast<size_t>(i);

      if (lead_idx >= lead_trajectory.size()) {
        lead_idx = lead_trajectory.size() - 1;
      }
      const auto& lead_point = lead_trajectory[lead_idx - 1];

      front_acc = lead_one_agent->accel_fusion();
      double lead_s = 0.0, lead_l = 0.0;
      if (ego_lane_coord->XYToSL(lead_point.x(), lead_point.y(), &lead_s,
                                 &lead_l)) {
        double lead_rear_s = lead_s - lead_one_agent->length() * 0.5;
        double ego_front_s = ego_s + front_edge_to_rear_axle;
        front_s = std::max(0.0, lead_rear_s - ego_front_s);
        auto matched_point = ego_lane_coord->GetPathPointByS(lead_s);
        double heading_diff = planning_math::NormalizeAngle(
            lead_point.theta() - matched_point.theta());
        front_vel = lead_point.vel() * std::cos(heading_diff);
      }
    }

    double decel_jerk = joint_traj_params_.min_decel_jerk;

    double next_acc = CalculateIdmAcceleration(
        current_a, current_v, current_s, front_acc, front_vel, front_s,
        target_tau, v0, decel_jerk);

    constexpr double kHalf = 0.5;
    const double dt = kPlanningTimeStep;
    const double dt2 = dt * dt;

    double next_v = std::max(0.0, current_v + next_acc * dt);

    double ds = std::max(0.0, current_v * dt + kHalf * next_acc * dt2);

    double next_s = current_s + ds;

    next_s = std::max(current_s, next_s);

    double ref_s = ref_ego_s + next_s;
    ref_s = std::clamp(ref_s, s_min, s_max);

    double ref_x = x_s_spline(ref_s);
    double ref_y = y_s_spline(ref_s);

    double dx_ds = x_s_spline.deriv(1, ref_s);
    double dy_ds = y_s_spline.deriv(1, ref_s);
    double d2x_ds2 = x_s_spline.deriv(2, ref_s);
    double d2y_ds2 = y_s_spline.deriv(2, ref_s);

    double ds_norm = std::sqrt(dx_ds * dx_ds + dy_ds * dy_ds);
    double curvature = 0.0;
    if (ds_norm > 1e-6) {
      curvature =
          (dx_ds * d2y_ds2 - dy_ds * d2x_ds2) / (ds_norm * ds_norm * ds_norm);
    }

    double next_delta = std::atan(wheel_base * curvature);
    next_delta = std::clamp(next_delta, -max_steer_angle, max_steer_angle);

    double next_theta =
        current_theta + (next_v * std::tan(current_delta) / wheel_base) * dt;

    ref_trajectory_[i].t = t;
    ref_trajectory_[i].s = next_s;
    ref_trajectory_[i].vel = next_v;
    ref_trajectory_[i].acc = next_acc;
    ref_trajectory_[i].x = ref_x;
    ref_trajectory_[i].y = ref_y;
    ref_trajectory_[i].theta = next_theta;
    ref_trajectory_[i].delta = next_delta;

    current_s = next_s;
    current_v = next_v;
    current_a = next_acc;
    current_theta = next_theta;
    current_delta = next_delta;
  }
}
}  // namespace planning