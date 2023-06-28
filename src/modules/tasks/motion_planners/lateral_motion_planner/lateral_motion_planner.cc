
#include "lateral_motion_planner.h"

#include <algorithm>
#include <cstddef>
#include <iostream>
#include <vector>

#include "debug_info_log.h"
#include "ilqr_define.h"
#include "lateral_motion_planner.pb.h"
#include "spline.h"
#include "src/lateral_motion_planning_cost.h"

static const double pi_const = 3.141592654;

namespace planning {
LateralMotionPlanner::LateralMotionPlanner(
    const EgoPlanningConfigBuilder *config_builder,
    const std::shared_ptr<TaskPipelineContext> &pipeline_context)
    : Task(config_builder, pipeline_context) {
  config_ = config_builder->cast<LateralMotionPlannerConfig>();
  name_ = "LateralMotionPlanner";

  Init();
};

void LateralMotionPlanner::Init() {
  // init planning problem
  planning_problem_ptr_ =
      std::make_shared<pnc::lateral_planning::LateralMotionPlanningProblem>();
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

  planning_input_.mutable_soft_upper_bound_x0_vec()->Resize(N, 0.0);
  planning_input_.mutable_soft_upper_bound_y0_vec()->Resize(N, 0.0);
  planning_input_.mutable_soft_upper_bound_x1_vec()->Resize(N, 0.0);
  planning_input_.mutable_soft_upper_bound_y1_vec()->Resize(N, 0.0);

  planning_input_.mutable_soft_lower_bound_x0_vec()->Resize(N, 0.0);
  planning_input_.mutable_soft_lower_bound_y0_vec()->Resize(N, 0.0);
  planning_input_.mutable_soft_lower_bound_x1_vec()->Resize(N, 0.0);
  planning_input_.mutable_soft_lower_bound_y1_vec()->Resize(N, 0.0);

  planning_input_.mutable_hard_upper_bound_x0_vec()->Resize(N, 0.0);
  planning_input_.mutable_hard_upper_bound_y0_vec()->Resize(N, 0.0);
  planning_input_.mutable_hard_upper_bound_x1_vec()->Resize(N, 0.0);
  planning_input_.mutable_hard_upper_bound_y1_vec()->Resize(N, 0.0);

  planning_input_.mutable_hard_lower_bound_x0_vec()->Resize(N, 0.0);
  planning_input_.mutable_hard_lower_bound_y0_vec()->Resize(N, 0.0);
  planning_input_.mutable_hard_lower_bound_x1_vec()->Resize(N, 0.0);
  planning_input_.mutable_hard_lower_bound_y1_vec()->Resize(N, 0.0);

  planning_input_.mutable_control_vec()->Resize(N, 0.0);
}

bool LateralMotionPlanner::Execute(planning::framework::Frame *frame) {
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

void LateralMotionPlanner::AssembleInput() {
  // set init state
  const auto &planning_init_point =
      reference_path_ptr_->get_frenet_ego_state().planning_init_point();

  JSON_DEBUG_VALUE("init_pos_x1", planning_init_point.lat_init_state.x())
  JSON_DEBUG_VALUE("init_pos_y1", planning_init_point.lat_init_state.y())

  planning_input_.mutable_init_state()->set_x(
      planning_init_point.lat_init_state.x());
  planning_input_.mutable_init_state()->set_y(
      planning_init_point.lat_init_state.y());
  planning_input_.mutable_init_state()->set_theta(
      planning_init_point.lat_init_state.theta());
  planning_input_.mutable_init_state()->set_delta(
      planning_init_point.lat_init_state.delta());

  // set reference
  const auto &lat_decider_output =  // result from lat decision
      frame_->session()->planning_context().lat_decider_output();

  const auto &enu_ref_path = lat_decider_output.enu_ref_path;
  const auto &enu_ref_theta = lat_decider_output.enu_ref_theta;
  assert(enu_ref_path.size() == enu_ref_theta.size());

  // set reference trajectory
  std::vector<double> ref_theta_vec(enu_ref_theta.size());

  for (size_t i = 0; i < enu_ref_path.size(); ++i) {
    planning_input_.mutable_ref_x_vec()->Set(i, enu_ref_path[i].first);
    planning_input_.mutable_ref_y_vec()->Set(i, enu_ref_path[i].second);
    ref_theta_vec[i] = enu_ref_theta[i];
  }

  // angle fix of difference between theta and ref_theta, such as [-179deg and
  // 179deg]
  double angle_compensate = 0.0;
  const auto d_theta =
      ref_theta_vec.front() - planning_init_point.lat_init_state.theta();
  if (d_theta > 1.5 * pi_const) {
    angle_compensate = -2.0 * pi_const;
  } else if (d_theta < -1.5 * pi_const) {
    angle_compensate = 2.0 * pi_const;
  } else {
    angle_compensate = 0.0;
  }

  // angle fix of ref_theta
  double angle_offset = 0.0;
  for (size_t i = 0; i < ref_theta_vec.size(); ++i) {
    if (i == 0) {
      planning_input_.mutable_ref_theta_vec()->Set(
          i, ref_theta_vec[i] + angle_compensate);
    } else {
      const auto delta_theta = ref_theta_vec[i] - ref_theta_vec[i - 1];
      if (delta_theta > 1.5 * pi_const) {
        angle_offset -= 2.0 * pi_const;
      } else if (delta_theta < -1.5 * pi_const) {
        angle_offset += 2.0 * pi_const;
      }
      planning_input_.mutable_ref_theta_vec()->Set(
          i, ref_theta_vec[i] + angle_offset + angle_compensate);
    }
  }

  // set init theta by ref_theta
  planning_input_.mutable_init_state()->set_theta(
      planning_input_.ref_theta_vec(0));

  // set last trajectory: temporarily same as reference: TODO
  for (size_t i = 0; i < enu_ref_path.size(); ++i) {
    planning_input_.mutable_last_x_vec()->Set(i, enu_ref_path[i].first);
    planning_input_.mutable_last_y_vec()->Set(i, enu_ref_path[i].second);
    planning_input_.mutable_last_theta_vec()->Set(i, enu_ref_theta[i]);
  }

  // set safe (hard) and path (soft) bound
  const auto &safe_bounds = lat_decider_output.safe_bounds;
  const auto &path_bounds = lat_decider_output.path_bounds;
  assert(safe_bounds.size() == path_bounds.size());

  for (size_t i = 0; i < safe_bounds.size(); ++i) {
    size_t index = i;
    size_t next_index = i + 1;

    if (i == safe_bounds.size() - 1) {
      index = i - 1;
      next_index = i;
    }

    const auto &safe_lower_bound = safe_bounds[index].first;
    const auto &safe_upper_bound = safe_bounds[index].second;
    const auto &next_safe_lower_bound = safe_bounds[next_index].first;
    const auto &next_safe_upper_bound = safe_bounds[next_index].second;

    planning_input_.mutable_soft_lower_bound_x0_vec()->Set(i,
                                                           safe_lower_bound.x);
    planning_input_.mutable_soft_lower_bound_y0_vec()->Set(i,
                                                           safe_lower_bound.y);
    planning_input_.mutable_soft_lower_bound_x1_vec()->Set(
        i, next_safe_lower_bound.x);
    planning_input_.mutable_soft_lower_bound_y1_vec()->Set(
        i, next_safe_lower_bound.y);

    planning_input_.mutable_soft_upper_bound_x0_vec()->Set(i,
                                                           safe_upper_bound.x);
    planning_input_.mutable_soft_upper_bound_y0_vec()->Set(i,
                                                           safe_upper_bound.y);
    planning_input_.mutable_soft_upper_bound_x1_vec()->Set(
        i, next_safe_upper_bound.x);
    planning_input_.mutable_soft_upper_bound_y1_vec()->Set(
        i, next_safe_upper_bound.y);

    const auto &path_lower_bound = path_bounds[index].first;
    const auto &path_upper_bound = path_bounds[index].second;
    const auto &next_path_lower_bound = path_bounds[next_index].first;
    const auto &next_path_upper_bound = path_bounds[next_index].second;

    planning_input_.mutable_hard_lower_bound_x0_vec()->Set(i,
                                                           path_lower_bound.x);
    planning_input_.mutable_hard_lower_bound_y0_vec()->Set(i,
                                                           path_lower_bound.y);
    planning_input_.mutable_hard_lower_bound_x1_vec()->Set(
        i, next_path_lower_bound.x);
    planning_input_.mutable_hard_lower_bound_y1_vec()->Set(
        i, next_path_lower_bound.y);

    planning_input_.mutable_hard_upper_bound_x0_vec()->Set(i,
                                                           path_upper_bound.x);
    planning_input_.mutable_hard_upper_bound_y0_vec()->Set(i,
                                                           path_upper_bound.y);
    planning_input_.mutable_hard_upper_bound_x1_vec()->Set(
        i, next_path_upper_bound.x);
    planning_input_.mutable_hard_upper_bound_y1_vec()->Set(
        i, next_path_upper_bound.y);

    static const double min_v_cruise = 0.5;
    planning_input_.set_ref_vel(
        std::max(lat_decider_output.v_cruise, min_v_cruise));

    planning_input_.set_curv_factor(0.3);
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
}

void LateralMotionPlanner::Update() {
  auto solver_condition = planning_problem_ptr_->Update(planning_input_);
  JSON_DEBUG_VALUE("solver_condition", solver_condition)

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

  double s = 0.0;
  for (size_t i = 0; i < N; ++i) {
    x_vec[i + 1] = planning_output.x_vec(i);
    y_vec[i + 1] = planning_output.y_vec(i);
    theta_vec[i + 1] = planning_output.theta_vec(
        i);  // note that theta cannot be limited with [-pi, pi]
             // to avoid incorrect spline
    delta_vec[i + 1] = planning_output.delta_vec(i);
    curv_vec[i + 1] =
        planning_input_.curv_factor() * planning_output.delta_vec(i);

    omega_vec[i + 1] = planning_output.omega_vec(i);
    d_curv_vec[i + 1] = planning_input_.curv_factor() * omega_vec[i + 1];

    if (i == 0) {
      s = 0.0;
    } else {
      const double ds =
          std::hypot(x_vec[i + 1] - x_vec[i], y_vec[i + 1] - y_vec[i]);
      s += std::max(ds, 1e-3);
    }
    s_vec[i + 1] = s;
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

  // set state spline
  motion_planning_info.x_s_spline.set_points(s_vec, x_vec);
  motion_planning_info.y_s_spline.set_points(s_vec, y_vec);
  motion_planning_info.theta_s_spline.set_points(s_vec, theta_vec);
  motion_planning_info.delta_s_spline.set_points(s_vec, delta_vec);
  motion_planning_info.omega_s_spline.set_points(s_vec, omega_vec);
  motion_planning_info.curv_s_spline.set_points(s_vec, curv_vec);
  motion_planning_info.d_curv_s_spline.set_points(s_vec, d_curv_vec);
  motion_planning_info.s_lat_vec = s_vec;

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

  // assemble results
  const auto &lat_decider_output =  // result from lat decision
      frame_->session()->planning_context().lat_decider_output();

  auto &traj_points = pipeline_context_->planning_result.traj_points;
  for (size_t i = 0; i < N; i++) {
    traj_points[i].x = x_vec[i + 1];
    traj_points[i].y = y_vec[i + 1];
    traj_points[i].heading_angle = theta_vec[i + 1];
    traj_points[i].curvature = planning_input_.curv_factor() * delta_vec[i + 1];

    traj_points[i].v = lat_decider_output.v_cruise;
    traj_points[i].t = planning_output.time_vec(i);

    // frenet state update
    Point2D cart_pt(traj_points[i].x, traj_points[i].y);
    Point2D frenet_pt;

    if (reference_path_ptr_->get_frenet_coord() != nullptr &&
        reference_path_ptr_->get_frenet_coord()->CartCoord2FrenetCoord(
            cart_pt, frenet_pt) == TRANSFORM_STATUS::TRANSFORM_SUCCESS) {
      traj_points[i].s = frenet_pt.x;
      traj_points[i].l = frenet_pt.y;
    } else {
      LOG_DEBUG(
          "CartCoord2FrenetCoord = FAILED !!!!!!!! index: %ld,  point.s : "
          "%f, point.l: %f ",
          i, traj_points[i].s, traj_points[i].l);
    }
  }
}

}  // namespace planning