#include "lat_lon_joint_planner_decider.h"

#include <cmath>
#include <limits>

#include "debug_info_log.h"
#include "environmental_model.h"
#include "ifly_time.h"
#include "math/box2d.h"
#include "src/joint_motion_input_builder.h"
#include "src/joint_motion_obstacles_selector.h"
#include "src/joint_motion_planning_problem.h"

namespace planning {

LatLonJointPlannerDecider::LatLonJointPlannerDecider(
    const EgoPlanningConfigBuilder* config_builder, framework::Session* session)
    : Task(config_builder, session) {
  name_ = "LatLonJointPlannerDecider";
  input_builder_ =
      std::make_unique<JointMotionInputBuilder>(config_builder, session);
  obstacles_selector_ = std::make_shared<JointMotionObstaclesSelector>(session);

  input_builder_->SetObstaclesSelector(obstacles_selector_);

  Init();
}

void LatLonJointPlannerDecider::Init() {
  planning_problem_ptr_ = std::make_shared<
      pnc::joint_motion_planning::JointMotionPlanningProblem>();
  planning_problem_ptr_->Init();

  const size_t N = 26;
  planning_input_.mutable_ref_x_vec()->Reserve(N);
  planning_input_.mutable_ref_y_vec()->Reserve(N);
  planning_input_.mutable_ref_theta_vec()->Reserve(N);
  planning_input_.mutable_ref_delta_vec()->Reserve(N);
  planning_input_.mutable_ref_vel_vec()->Reserve(N);
  planning_input_.mutable_ref_acc_vec()->Reserve(N);
  planning_input_.mutable_ref_s_vec()->Reserve(N);

  planning_input_.mutable_ref_x_vec()->Resize(N, 0.0);
  planning_input_.mutable_ref_y_vec()->Resize(N, 0.0);
  planning_input_.mutable_ref_theta_vec()->Resize(N, 0.0);
  planning_input_.mutable_ref_delta_vec()->Resize(N, 0.0);
  planning_input_.mutable_ref_vel_vec()->Resize(N, 0.0);
  planning_input_.mutable_ref_acc_vec()->Resize(N, 0.0);
  planning_input_.mutable_ref_s_vec()->Resize(N, 0.0);
}

bool LatLonJointPlannerDecider::Execute() {
  LOG_DEBUG("=======LatLonJointPlannerDecider======= \n");

  if (!PreCheck()) {
    LOG_DEBUG("PreCheck failed\n");
    return false;
  }

  auto start_time = IflyTime::Now_ms();

  input_builder_->BuildInput(planning_input_, planning_problem_ptr_);

  Update();

  auto end_time = IflyTime::Now_ms();
  JSON_DEBUG_VALUE("LatLonJointPlannerDeciderTime", end_time - start_time);

  DebugInfoManager::GetInstance()
      .GetDebugInfoPb()
      ->mutable_joint_motion_planning_input()
      ->CopyFrom(planning_input_);

  DebugInfoManager::GetInstance()
      .GetDebugInfoPb()
      ->mutable_joint_motion_planning_output()
      ->CopyFrom(planning_problem_ptr_->GetOutput());

  const auto& danger_ids = lat_lon_planning_output_.GetDangerObstacleIds();
  std::vector<double> danger_ids_vec;
  for (int32_t id : danger_ids) {
    danger_ids_vec.push_back(static_cast<double>(id));
  }
  JSON_DEBUG_VECTOR("joint_danger_obstacle_ids", danger_ids_vec, 0);

  auto& context_output = session_->mutable_planning_context()
                             ->mutable_lat_lon_joint_planner_decider_output();
  context_output = lat_lon_planning_output_;

  return true;
}

void LatLonJointPlannerDecider::Update() {
  auto solver_condition = planning_problem_ptr_->Update(planning_input_);

  lat_lon_planning_output_.Clear();

  const size_t N =
      planning_problem_ptr_->GetiLqrCorePtr()->GetSolverConfigPtr()->horizon +
      1;
  const auto& dt =
      planning_problem_ptr_->GetiLqrCorePtr()->GetSolverConfigPtr()->model_dt;
  const auto& planning_output = planning_problem_ptr_->GetOutput();

  const bool motion_failed =
      solver_condition >= ilqr_solver::iLqr::BACKWARD_PASS_FAIL;

  lat_lon_planning_output_.SetPlanningSuccess(!motion_failed);

  const auto& key_agent_ids = input_builder_->GetKeyAgentIds();
  std::vector<int32_t> obstacle_ids;
  obstacle_ids.reserve(key_agent_ids.size());
  for (double id : key_agent_ids) {
    obstacle_ids.push_back(static_cast<int32_t>(id));
  }
  lat_lon_planning_output_.SetSelectedObstacleIds(obstacle_ids);

  if (!motion_failed) {
    auto& ego_trajectory = lat_lon_planning_output_.GetEgoTrajectory();
    ego_trajectory.resize(N);

    for (size_t i = 0; i < N; ++i) {
      auto& point = ego_trajectory[i];
      point.t = i * dt;
      point.x = planning_output.x_vec(i);
      point.y = planning_output.y_vec(i);
      point.theta = planning_output.theta_vec(i);
      point.vel = planning_output.vel_vec(i);
      point.acc = planning_output.acc_vec(i);
      point.delta = planning_output.delta_vec(i);
      point.omega = planning_output.omega_vec(i);
      point.jerk = planning_output.jerk_vec(i);
      point.s = planning_output.s_vec(i);
    }

    CheckCollisionWithObstacles(ego_trajectory, planning_input_, N, dt);
  }
}

void LatLonJointPlannerDecider::CheckCollisionWithObstacles(
    const EgoTrajectory& ego_trajectory,
    const planning::common::JointMotionPlanningInput& planning_input, size_t N,
    double dt) {
  std::vector<int32_t> danger_obstacles;

  if (ego_trajectory.empty()) {
    lat_lon_planning_output_.SetDangerObstacleIds(danger_obstacles);
    return;
  }

  const auto& vehicle_param =
      VehicleConfigurationContext::Instance()->get_vehicle_param();
  const double ego_length = vehicle_param.length;
  const double ego_width = vehicle_param.width;
  const double rear_axle_to_center = vehicle_param.rear_axle_to_center;
  constexpr double kStaticVehicleLatConflictThreshold = 0.2;
  constexpr double kLatConflictThreshold = 0.3;
  constexpr double kLargeVehicleLatConflictThreshold = 0.4;
  constexpr double kLargeVehicleLengthThreshold = 8.0;
  constexpr double kStaticVehicleVelocityThreshold = 0.3;
  constexpr double kBaseLongitudinalThreshold = 3.5;

  const auto& lane_change_output =
      session_->planning_context().lane_change_decider_output();
  const auto& reference_path_ptr =
      lane_change_output.coarse_planning_info.reference_path;
  if (reference_path_ptr == nullptr) {
    lat_lon_planning_output_.SetDangerObstacleIds(danger_obstacles);
    return;
  }

  const auto& frenet_coord = reference_path_ptr->get_frenet_coord();
  if (frenet_coord == nullptr) {
    lat_lon_planning_output_.SetDangerObstacleIds(danger_obstacles);
    return;
  }

  const size_t obs_num = planning_input.obs_ref_trajectory_size();

  const auto& route_info = session_->environmental_model().get_route_info();
  const auto& ego_lane_road_right_output =
      session_->planning_context().ego_lane_road_right_decider_output();
  bool is_confluence_area = false;
  if (route_info != nullptr) {
    const auto& route_info_output = route_info->get_route_info_output();
    if (route_info_output.is_closing_merge ||
        route_info_output.is_closing_split) {
      is_confluence_area = true;
    }
  }
  if (ego_lane_road_right_output.is_merge_region ||
      ego_lane_road_right_output.is_split_region) {
    is_confluence_area = true;
  }

  for (size_t obs_idx = 0; obs_idx < obs_num; ++obs_idx) {
    const auto& obs_ref_traj = planning_input.obs_ref_trajectory(obs_idx);
    const int32_t obs_id = obs_ref_traj.obs_id();
    const double obs_initial_vel = planning_input.obs_init_state(obs_idx).vel();
    const size_t ref_traj_size = obs_ref_traj.ref_x_vec_size();

    if (ref_traj_size == 0) {
      continue;
    }

    if (obs_id == -1) {
      continue;
    }

    auto* agent =
        session_->environmental_model().get_agent_manager()->GetAgent(obs_id);

    if (agent == nullptr) {
      continue;
    }

    bool has_collision = false;

    const bool is_large_vehicle =
        agent->type() == agent::AgentType::BUS ||
        agent->type() == agent::AgentType::TRUCK ||
        agent->type() == agent::AgentType::TRAILER ||
        obs_ref_traj.length() > kLargeVehicleLengthThreshold;

    const bool is_static_vehicle =
        std::fabs(obs_initial_vel) < kStaticVehicleVelocityThreshold ||
        agent->is_static();

    double lateral_threshold = is_large_vehicle
                                   ? kLargeVehicleLatConflictThreshold
                                   : kLatConflictThreshold;
    if (is_static_vehicle) {
      lateral_threshold = kStaticVehicleLatConflictThreshold;
    }

    bool is_directly_in_front_or_is_rear_agent = false;
    const double ego_half_width = ego_width * 0.5;

    for (size_t i = 0; i < N && i < ref_traj_size; ++i) {
      const auto& ego_point = ego_trajectory[i];

      double obs_x = obs_ref_traj.ref_x_vec(i);
      double obs_y = obs_ref_traj.ref_y_vec(i);
      double obs_theta = obs_ref_traj.ref_theta_vec(i);
      double obs_point_vel = obs_ref_traj.ref_vel_vec(i);

      const double center_x =
          ego_point.x + std::cos(ego_point.theta) * rear_axle_to_center;
      const double center_y =
          ego_point.y + std::sin(ego_point.theta) * rear_axle_to_center;

      planning_math::Box2d ego_box(planning_math::Vec2d(center_x, center_y),
                                   ego_point.theta, ego_length, ego_width);

      planning_math::Box2d obs_box(planning_math::Vec2d(obs_x, obs_y),
                                   obs_theta, obs_ref_traj.length(),
                                   obs_ref_traj.width());

      double ego_min_s = std::numeric_limits<double>::max();
      double ego_max_s = std::numeric_limits<double>::lowest();
      double ego_min_l = std::numeric_limits<double>::max();
      double ego_max_l = std::numeric_limits<double>::lowest();

      const auto& ego_corners = ego_box.GetAllCorners();
      for (const auto& corner : ego_corners) {
        double s = 0.0, l = 0.0;
        frenet_coord->XYToSL(corner.x(), corner.y(), &s, &l);
        ego_min_s = std::fmin(ego_min_s, s);
        ego_max_s = std::fmax(ego_max_s, s);
        ego_min_l = std::fmin(ego_min_l, l);
        ego_max_l = std::fmax(ego_max_l, l);
      }

      double obs_min_s = std::numeric_limits<double>::max();
      double obs_max_s = std::numeric_limits<double>::lowest();
      double obs_min_l = std::numeric_limits<double>::max();
      double obs_max_l = std::numeric_limits<double>::lowest();

      const auto& obs_corners = obs_box.GetAllCorners();
      for (const auto& corner : obs_corners) {
        double s = 0.0, l = 0.0;
        frenet_coord->XYToSL(corner.x(), corner.y(), &s, &l);
        obs_min_s = std::fmin(obs_min_s, s);
        obs_max_s = std::fmax(obs_max_s, s);
        obs_min_l = std::fmin(obs_min_l, l);
        obs_max_l = std::fmax(obs_max_l, l);
      }

      if (i == 0) {
        double obs_center_l = (obs_min_l + obs_max_l) * 0.5;
        double obs_center_s = (obs_min_s + obs_max_s) * 0.5;
        double ego_center_s = (ego_min_s + ego_max_s) * 0.5;
        if (std::fabs(obs_center_l) < ego_half_width) {
          is_directly_in_front_or_is_rear_agent = true;
        }
        if (obs_center_s < ego_center_s && !is_confluence_area) {
          is_directly_in_front_or_is_rear_agent = true;
        }
      }

      if (is_directly_in_front_or_is_rear_agent) {
        continue;
      }

      double lateral_dist = 0.0;
      if (obs_min_l >= ego_max_l) {
        lateral_dist = obs_min_l - ego_max_l;
      } else if (obs_max_l <= ego_min_l) {
        lateral_dist = ego_min_l - obs_max_l;
      } else {
        lateral_dist = 0.0;
      }

      double obs_longitudinal_distance = obs_min_s - ego_max_s;
      double ego_longitudinal_distance = ego_min_s - obs_max_s;

      double obs_longitudinal_threshold =
          kBaseLongitudinalThreshold + ego_point.vel * 0.3;

      bool is_overlap =
          (obs_longitudinal_distance < 0 && ego_longitudinal_distance < 0);

      bool long_collision = is_overlap || (obs_longitudinal_distance > 0 &&
                                           obs_longitudinal_distance <
                                               obs_longitudinal_threshold);

      if (lateral_dist <= lateral_threshold && long_collision) {
        has_collision = true;
        break;
      }
    }

    if (has_collision) {
      danger_obstacles.push_back(obs_id);
    }
  }

  lat_lon_planning_output_.SetDangerObstacleIds(danger_obstacles);
}

}  // namespace planning