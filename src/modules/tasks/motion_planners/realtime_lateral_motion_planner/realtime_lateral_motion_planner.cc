
#include "realtime_lateral_motion_planner.h"

#include <algorithm>
#include <cstddef>
#include <iostream>
#include <vector>

#include "debug_info_log.h"
#include "ilqr_define.h"
#include "lateral_motion_planner.pb.h"
#include "math_lib.h"
#include "spline.h"
#include "src/realtime_lateral_motion_planning_cost.h"

static const double planning_time = 5.0;
static const double planning_loop_dt = 0.1;
static const double ilqr_dt = 0.2;
static const double min_v_cruise = 0.5;
static const double c0_limit = 2.0;
namespace planning {
RealtimeLateralMotionPlanner::RealtimeLateralMotionPlanner(
    const EgoPlanningConfigBuilder *config_builder,
    const std::shared_ptr<TaskPipelineContext> &pipeline_context)
    : Task(config_builder, pipeline_context) {
  config_ = config_builder->cast<RealtimeLateralMotionPlannerConfig>();
  name_ = "LateralMotionPlanner";

  Init();
};

void RealtimeLateralMotionPlanner::Init() {
  // init planning problem
  planning_problem_ptr_ = std::make_shared<
      pnc::realtime_lateral_planning::RealtimeLateralMotionPlanningProblem>();
  planning_problem_ptr_->Init();

  // init planning input and ouput
  const auto N =
      planning_problem_ptr_->GetiLqrCorePtr()->GetSolverConfigPtr()->horizon +
      1;

  // init planning input
  planning_input_.mutable_ref_x_vec()->Resize(N, 0.0);
  planning_input_.mutable_ref_y_vec()->Resize(N, 0.0);
  planning_input_.mutable_ref_theta_vec()->Resize(N, 0.0);

  planning_input_.mutable_last_x_vec()->Resize(N, 0.0);
  planning_input_.mutable_last_y_vec()->Resize(N, 0.0);
  planning_input_.mutable_last_theta_vec()->Resize(N, 0.0);

  // bound never considered

  planning_input_.mutable_control_vec()->Resize(N, 0.0);
}

bool RealtimeLateralMotionPlanner::Execute(planning::framework::Frame *frame) {
  LOG_DEBUG("=======LateralMotionPlanner======= \n");
  frame_ = frame;
  if (Task::Execute(frame) == false) {
    return false;
  }

  // assemble input
  AssembleInput();

  // update
  Update();

  // record input and output
  DebugInfoManager::GetInstance()
      .GetDebugInfoPb()
      ->mutable_lateral_motion_planning_input()
      ->CopyFrom(planning_input_);
  DebugInfoManager::GetInstance()
      .GetDebugInfoPb()
      ->mutable_lateral_motion_planning_output()
      ->CopyFrom(planning_problem_ptr_->GetOutput());

  return true;
}

void RealtimeLateralMotionPlanner::AssembleInput() {
  auto &motion_planning_info = frame_->mutable_session()
                                   ->mutable_planning_context()
                                   ->mutable_planning_result()
                                   .motion_planning_info;
  // lateral_output derives from vision_only_lateral_motion_planner
  const auto &lateral_output = frame_->mutable_session()
                                   ->planning_context()
                                   .lateral_behavior_planner_output();

  const auto &ego_state =
      frame_->session()->environmental_model().get_ego_state_manager();

  // set ref_vel
  double ref_vel = std::max(
      frame_->session()->planning_context().v_ref_cruise(), ego_state->ego_v());
  ref_vel = max(ref_vel, min_v_cruise);
  planning_input_.set_ref_vel(ref_vel);

  // set curv_factor
  planning_input_.set_curv_factor(0.3);

  // set init state
  const auto &planning_init_point =
      reference_path_ptr_->get_frenet_ego_state().planning_init_point();

  planning_input_.mutable_init_state()->set_x(
      planning_init_point.lat_init_state.x());
  planning_input_.mutable_init_state()->set_y(
      planning_init_point.lat_init_state.y());
  planning_input_.mutable_init_state()->set_theta(
      planning_init_point.lat_init_state.theta());
  planning_input_.mutable_init_state()->set_delta(
      planning_init_point.lat_init_state.delta());

  // set reference
  const auto &d_polynomial = lateral_output.d_poly;

  const double &c3 = d_polynomial[0];
  const double &c2 = d_polynomial[1];
  const double &c1 = d_polynomial[2];

  const double c0 = pnc::mathlib::Limit(d_polynomial[3], c0_limit);

  // sampling by ref_vel and x
  std::vector<double> dx_vec;
  std::vector<double> dy_vec;
  std::vector<double> s_vec;

  size_t trajectory_size = 26;  // TODO: parameterized

  dx_vec.reserve(trajectory_size);
  dy_vec.reserve(trajectory_size);
  s_vec.reserve(trajectory_size);

  const double delta_x = ref_vel * ilqr_dt;

  double dx = 0.0;
  double s = 0.0;

  for (size_t i = 0; i < trajectory_size; ++i) {
    const double dy =
        c3 * std::pow(dx, 3) + c2 * std::pow(dx, 2) + c1 * dx + c0;
    dx_vec.emplace_back(dx);
    dy_vec.emplace_back(dy);

    if (i == 0) {
      s_vec.emplace_back(0.0);
    } else {
      const double ds = std::hypot(delta_x, dy_vec[i] - dy_vec[i - 1]);
      s += ds;
      s_vec.emplace_back(s);
    }

    dx += delta_x;
  }

  // lateral reference
  motion_planning_info.ref_x_s_spline.set_points(s_vec, dx_vec);
  motion_planning_info.ref_y_s_spline.set_points(s_vec, dy_vec);

  // resampling by ref vel and s
  double s_ref = 0;
  for (size_t i = 0; i < trajectory_size; ++i) {
    planning_input_.set_ref_x_vec(i,
                                  motion_planning_info.ref_x_s_spline(s_ref));
    planning_input_.set_ref_y_vec(i,
                                  motion_planning_info.ref_y_s_spline(s_ref));

    const double dx_s_derv_1st =
        motion_planning_info.ref_x_s_spline.deriv(1, s_ref);
    const double dy_s_derv_1st =
        motion_planning_info.ref_y_s_spline.deriv(1, s_ref);
    planning_input_.set_ref_theta_vec(i,
                                      std::atan2(dy_s_derv_1st, dx_s_derv_1st));

    s_ref += ref_vel * ilqr_dt;
  }

  // set last trajectory: temporarily same as reference: TODO
  double tmp_t = 0.0;
  if (motion_planning_info.lat_init_flag == true) {
    for (size_t i = 0; i < trajectory_size; ++i) {
      tmp_t = std::min(planning_loop_dt + i * ilqr_dt, planning_time);
      planning_input_.mutable_last_x_vec()->Set(
          i, motion_planning_info.lateral_x_t_spline(tmp_t));
      planning_input_.mutable_last_y_vec()->Set(
          i, motion_planning_info.lateral_y_t_spline(tmp_t));
      planning_input_.mutable_last_theta_vec()->Set(
          i, motion_planning_info.lateral_theta_t_spline(tmp_t));
    }
  } else {
    for (size_t i = 0; i < trajectory_size; ++i) {
      planning_input_.mutable_last_x_vec()->Set(i,
                                                planning_input_.ref_x_vec(i));
      planning_input_.mutable_last_y_vec()->Set(i,
                                                planning_input_.ref_y_vec(i));
      planning_input_.mutable_last_theta_vec()->Set(
          i, planning_input_.ref_theta_vec(i));
    }
  }

  // set weights
  planning_input_.set_acc_bound(config_.acc_bound);
  planning_input_.set_jerk_bound(config_.jerk_bound);
  planning_input_.set_q_ref_x(config_.q_ref_x);
  planning_input_.set_q_ref_y(config_.q_ref_y);
  planning_input_.set_q_ref_theta(config_.q_ref_theta);
  planning_input_.set_q_continuity(config_.q_continuity);
  planning_input_.set_q_acc(config_.q_acc);
  planning_input_.set_q_jerk(config_.q_jerk);
  planning_input_.set_q_acc_bound(config_.q_acc_bound);
  planning_input_.set_q_jerk_bound(config_.q_jerk_bound);
  planning_input_.set_q_soft_corridor(config_.q_soft_corridor);
  planning_input_.set_q_hard_corridor(config_.q_hard_corridor);

  // TODO: set control vec for warm start

  // set complete hold flag, concerned index
  const bool complete_follow = true;

  planning_input_.set_complete_follow(complete_follow);
  planning_input_.set_motion_plan_concerned_index(
      config_.motion_plan_concerned_index);
}

void RealtimeLateralMotionPlanner::Update() {
  auto solver_condition = planning_problem_ptr_->Update(planning_input_);
  LOG_DEBUG("realtime lateral motion: solver_condition:: %d\n",
            solver_condition);

  // update planning_output
  const auto &planning_output = planning_problem_ptr_->GetOutput();

  // assembling planning output proto
  const auto N =
      planning_problem_ptr_->GetiLqrCorePtr()->GetSolverConfigPtr()->horizon +
      1;

  std::vector<double> x_vec(N + 1);
  std::vector<double> y_vec(N + 1);
  std::vector<double> theta_vec(N + 1);
  std::vector<double> delta_vec(N + 1);
  std::vector<double> omega_vec(N + 1);
  std::vector<double> curv_vec(N + 1);
  std::vector<double> d_curv_vec(N + 1);
  std::vector<double> s_vec(N + 1);
  std::vector<double> t_vec(N + 1);

  double s = 0.0;
  double t = 0.0;
  for (size_t i = 0; i < N; ++i) {
    x_vec[i + 1] = planning_output.x_vec(i);
    y_vec[i + 1] = planning_output.y_vec(i);
    theta_vec[i + 1] =
        planning_output.theta_vec(i);  // note that theta cannot be limited with
                                       // [-pi, pi] to avoid incorrect spline
    delta_vec[i + 1] = planning_output.delta_vec(i);
    curv_vec[i + 1] =
        planning_input_.curv_factor() * planning_output.delta_vec(i);

    omega_vec[i + 1] = planning_output.omega_vec(i);
    d_curv_vec[i + 1] = planning_input_.curv_factor() * omega_vec[i + 1];

    if (i == 0) {
      s = 0.0;
      t = 0.0;
    } else {
      const double ds =
          std::hypot(x_vec[i + 1] - x_vec[i], y_vec[i + 1] - y_vec[i]);
      s += std::max(ds, 1e-3);
      t += ilqr_dt;
    }
    s_vec[i + 1] = s;
    t_vec[i + 1] = t;
  }

  // generate motion planning output into planning_context
  auto &motion_planning_info = frame_->mutable_session()
                                   ->mutable_planning_context()
                                   ->mutable_planning_result()
                                   .motion_planning_info;

  // append the planning traj anti-direction for decoupling lat & lon replan
  const static double appended_length = 1.5;
  Eigen::Vector2d unit_vector(x_vec[1] - x_vec[2], y_vec[1] - y_vec[2]);
  unit_vector.normalize();

  s_vec[0] = -appended_length;
  x_vec[0] = x_vec[1] + unit_vector.x() * appended_length;
  y_vec[0] = y_vec[1] + unit_vector.y() * appended_length;
  theta_vec[0] = theta_vec[1];
  delta_vec[0] = delta_vec[1];
  omega_vec[0] = omega_vec[1];
  theta_vec[0] = theta_vec[1];
  curv_vec[0] = curv_vec[1];
  d_curv_vec[0] = d_curv_vec[1];
  t_vec[0] = -ilqr_dt;

  // set state spline
  motion_planning_info.x_s_spline.set_points(s_vec, x_vec);
  motion_planning_info.y_s_spline.set_points(s_vec, y_vec);
  motion_planning_info.theta_s_spline.set_points(s_vec, theta_vec);
  motion_planning_info.delta_s_spline.set_points(s_vec, delta_vec);
  motion_planning_info.omega_s_spline.set_points(s_vec, omega_vec);
  motion_planning_info.curv_s_spline.set_points(s_vec, curv_vec);
  motion_planning_info.d_curv_s_spline.set_points(s_vec, d_curv_vec);
  motion_planning_info.lateral_x_t_spline.set_points(t_vec, x_vec);
  motion_planning_info.lateral_y_t_spline.set_points(t_vec, y_vec);
  motion_planning_info.lateral_theta_t_spline.set_points(t_vec, theta_vec);
  motion_planning_info.s_lat_vec = s_vec;
  motion_planning_info.lat_init_flag = true;

  ControlVec u_vec;
  u_vec.resize(N);

  // set u_vec to motion_planning_info for warm start
  for (size_t i = 0; i < N; ++i) {
    Control u;
    u.resize(1);
    u[0] = omega_vec[i];
    u_vec[i] = u;
  }
  motion_planning_info.u_vec = u_vec;

  auto &traj_points = pipeline_context_->planning_result.traj_points;
  for (size_t i = 0; i < N; i++) {
    traj_points[i].x = x_vec[i + 1];
    traj_points[i].y = y_vec[i + 1];
    traj_points[i].heading_angle = theta_vec[i + 1];
    traj_points[i].curvature = planning_input_.curv_factor() * delta_vec[i + 1];

    const auto ref_v_cruise =
        frame_->session()->planning_context().v_ref_cruise();

    traj_points[i].v = ref_v_cruise;
    traj_points[i].t = planning_output.time_vec(i);

    // realtime does not consider frenet state
    traj_points[i].s = 0.0;
    traj_points[i].l = 0.0;
  }

  motion_planning_info.lat_enable_flag = true;
}

}  // namespace planning