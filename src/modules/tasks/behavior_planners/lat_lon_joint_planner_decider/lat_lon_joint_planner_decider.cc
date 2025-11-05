#include "lat_lon_joint_planner_decider.h"

#include "debug_info_log.h"
#include "ifly_time.h"
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

  auto build_input_end_time = IflyTime::Now_ms();

  Update();

  auto end_time = IflyTime::Now_ms();
  JSON_DEBUG_VALUE("LatLonJointPlannerDeciderTime", end_time - start_time);
  JSON_DEBUG_VALUE("JointPlannerOptimizationTime",
                   end_time - build_input_end_time);

  DebugInfoManager::GetInstance()
      .GetDebugInfoPb()
      ->mutable_joint_motion_planning_input()
      ->CopyFrom(planning_input_);

  DebugInfoManager::GetInstance()
      .GetDebugInfoPb()
      ->mutable_joint_motion_planning_output()
      ->CopyFrom(planning_problem_ptr_->GetOutput());

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

    lat_lon_planning_output_.BuildSplines();

    auto& motion_planner_output =
        session_->mutable_planning_context()->mutable_motion_planner_output();

    motion_planner_output = MotionPlannerOutput();

    motion_planner_output.v_t_spline = lat_lon_planning_output_.v_t_spline;
    motion_planner_output.a_t_spline = lat_lon_planning_output_.a_t_spline;
    motion_planner_output.j_t_spline = lat_lon_planning_output_.j_t_spline;

    motion_planner_output.x_s_spline = lat_lon_planning_output_.x_s_spline;
    motion_planner_output.y_s_spline = lat_lon_planning_output_.y_s_spline;
    motion_planner_output.theta_s_spline =
        lat_lon_planning_output_.theta_s_spline;
    motion_planner_output.delta_s_spline =
        lat_lon_planning_output_.delta_s_spline;
    motion_planner_output.omega_s_spline =
        lat_lon_planning_output_.omega_s_spline;

    motion_planner_output.curv_s_spline =
        lat_lon_planning_output_.curv_s_spline;
    motion_planner_output.d_curv_s_spline =
        lat_lon_planning_output_.d_curv_s_spline;

    motion_planner_output.s_lat_vec = lat_lon_planning_output_.s_lat_vec;

    motion_planner_output.path_backward_appended_length =
        lat_lon_planning_output_.path_backward_appended_length;

    motion_planner_output.lateral_s_t_spline =
        lat_lon_planning_output_.lateral_s_t_spline;
    motion_planner_output.lateral_t_s_spline =
        lat_lon_planning_output_.lateral_t_s_spline;

    motion_planner_output.lon_enable_flag = true;
    motion_planner_output.lat_enable_flag = true;

    const auto& reference_path_ptr = session_->planning_context()
                                         .lane_change_decider_output()
                                         .coarse_planning_info.reference_path;
    const auto& planning_init_point =
        reference_path_ptr->get_frenet_ego_state().planning_init_point();

    auto& traj_points = session_->mutable_planning_context()
                            ->mutable_planning_result()
                            .traj_points;
    traj_points.resize(N);

    std::vector<double> s_vec_lon(N);
    std::vector<double> v_vec(N);
    std::vector<double> a_vec(N);
    std::vector<double> j_vec(N);
    std::vector<double> t_vec(N);

    for (size_t i = 0; i < N; ++i) {
      s_vec_lon[i] = ego_trajectory[i].s;
      v_vec[i] = ego_trajectory[i].vel;
      a_vec[i] = ego_trajectory[i].acc;
      j_vec[i] = ego_trajectory[i].jerk;
      t_vec[i] = ego_trajectory[i].t;
    }

    const auto& s0 = s_vec_lon[0];
    const double init_point_s = planning_init_point.frenet_state.s;

    std::vector<double> assembled_x(N);
    std::vector<double> assembled_y(N);
    std::vector<double> assembled_theta(N);
    std::vector<double> assembled_delta(N);
    std::vector<double> assembled_omega(N);

    for (size_t i = 0; i < N; ++i) {
      traj_points[i].v = v_vec[i];
      traj_points[i].a = a_vec[i];
      traj_points[i].t = t_vec[i];
      traj_points[i].jerk = j_vec[i];

      auto s = std::max(s_vec_lon[i] - s0, 0.0);

      s = std::min(s, motion_planner_output.s_lat_vec.back());

      traj_points[i].x = motion_planner_output.x_s_spline(s);
      traj_points[i].y = motion_planner_output.y_s_spline(s);
      traj_points[i].heading_angle = motion_planner_output.theta_s_spline(s);
      traj_points[i].s = s + init_point_s;

      assembled_x[i] = traj_points[i].x;
      assembled_y[i] = traj_points[i].y;
      assembled_theta[i] = traj_points[i].heading_angle;
      assembled_delta[i] = motion_planner_output.delta_s_spline(s);
      assembled_omega[i] = motion_planner_output.omega_s_spline(s);
    }

    motion_planner_output.x_t_spline.set_points(t_vec, assembled_x);
    motion_planner_output.y_t_spline.set_points(t_vec, assembled_y);
    motion_planner_output.theta_t_spline.set_points(t_vec, assembled_theta);
    motion_planner_output.delta_t_spline.set_points(t_vec, assembled_delta);
    motion_planner_output.omega_t_spline.set_points(t_vec, assembled_omega);
  }
}

}  // namespace planning