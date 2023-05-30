
#include "general_lateral_motion_planner.h"

#include <algorithm>
#include <cstddef>

#include "debug_info_log.h"
#include "lateral_motion_planner.pb.h"
#include "lateral_motion_planning_cost.h"
#include "spline.h"

namespace planning {
LateralMotionPlanner::LateralMotionPlanner(
    const EgoPlanningConfigBuilder *config_builder,
    const std::shared_ptr<TaskPipelineContext> &pipeline_context)
    : Task(config_builder, pipeline_context) {
  config_ = config_builder->cast<GeneralLateralMotionPlannerConfig>();
  name_ = "LateralMotionPlanner";

  Init();
};

void LateralMotionPlanner::Init() {
  // init planning problem
  planning_problem_ptr_ =
      std::make_shared<pnc::lateral_planning::LateralMotionPlanningProblem>();
  planning_problem_ptr_->Init();

  // init planning input and ouput
  auto const N =
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

  // init planning output
  planning_output_.mutable_time_vec()->Resize(N, 0.0);
  planning_output_.mutable_x_vec()->Resize(N, 0.0);
  planning_output_.mutable_y_vec()->Resize(N, 0.0);
  planning_output_.mutable_theta_vec()->Resize(N, 0.0);
  planning_output_.mutable_delta_vec()->Resize(N, 0.0);
  planning_output_.mutable_omega_vec()->Resize(N, 0.0);
  planning_output_.mutable_omega_dot_vec()->Resize(N, 0.0);
  planning_output_.mutable_acc_vec()->Resize(N, 0.0);
  planning_output_.mutable_jerk_vec()->Resize(N, 0.0);

  // init state vec
  x_vec_.resize(N);
  y_vec_.resize(N);
  theta_vec_.resize(N);
  delta_vec_.resize(N);
  omega_vec_.resize(N);
  curv_vec.resize(N);
  d_curv_vec.resize(N);
  s_vec_.resize(N);
}

bool LateralMotionPlanner::Execute(planning::framework::Frame *frame) {
  LOG_DEBUG("=======LateralMotionPlanner======= \n");
  frame_ = frame;
  if (Task::Execute(frame) == false) {
    return false;
  }

  // generate input
  GeneratePlanningInput();

  // update
  planning_problem_ptr_->Update(planning_input_);

  // get output
  GeneratePlanningOutput();

  // record input
  DebugInfoManager::GetInstance()
      .GetDebugInfoPb()
      ->mutable_lateral_motion_planning_input()
      ->CopyFrom(planning_input_);

  DebugInfoManager::GetInstance()
      .GetDebugInfoPb()
      ->mutable_lateral_motion_planning_output()
      ->CopyFrom(planning_output_);

  return true;
}

void LateralMotionPlanner::GeneratePlanningInput() {
  const auto &lat_decider_output = // result from lat decision
      frame_->session()->planning_context().lat_decider_output();

  const auto &enu_ref_path = lat_decider_output.enu_ref_path;
  const auto &enu_ref_theta = lat_decider_output.enu_ref_theta;
  assert(enu_ref_path.size() == enu_ref_theta.size());

  // set reference trajectory
  for (size_t i = 0; i < enu_ref_path.size(); ++i) {
    planning_input_.mutable_ref_x_vec()->Set(i, enu_ref_path[i].first);
    planning_input_.mutable_ref_y_vec()->Set(i, enu_ref_path[i].second);
    planning_input_.mutable_ref_theta_vec()->Set(i, enu_ref_theta[i]);
  }

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

    planning_input_.set_ref_vel(lat_decider_output.v_cruise);

    planning_input_.set_curv_factor(0.3);
  }

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

  planning_input_.mutable_init_state()->set_omega(
      planning_init_point.lat_init_state.omega());

  // set weights
  planning_input_.set_acc_bound(config_.acc_bound);
  planning_input_.set_jerk_bound(config_.jerk_bound);
  planning_input_.set_q_ref_x(config_.q_ref_x);
  planning_input_.set_q_ref_y(config_.q_ref_y);
  planning_input_.set_q_ref_theta(config_.q_ref_theta);
  planning_input_.set_q_continuity(config_.q_continuity);
  planning_input_.set_q_acc(config_.q_acc);
  planning_input_.set_q_jerk(config_.q_jerk);
  planning_input_.set_q_snap(config_.q_snap);
  planning_input_.set_q_acc_bound(config_.q_acc_bound);
  planning_input_.set_q_jerk_bound(config_.q_jerk_bound);
  planning_input_.set_q_soft_corridor(config_.q_soft_corridor);
  planning_input_.set_q_hard_corridor(config_.q_hard_corridor);
}

void LateralMotionPlanner::GeneratePlanningOutput() {
  // assembling planning output proto
  const auto &state_result =
      planning_problem_ptr_->GetiLqrCorePtr()->GetStateResultPtr();
  const auto &control_result =
      planning_problem_ptr_->GetiLqrCorePtr()->GetControlResultPtr();
  const auto &N =
      planning_problem_ptr_->GetiLqrCorePtr()->GetSolverConfigPtr()->horizon +
      1;

  const auto &dt =
      planning_problem_ptr_->GetiLqrCorePtr()->GetSolverConfigPtr()->model_dt;

  double t = 0.0;
  double s = 0.0;
  const double kv2 = planning_input_.curv_factor() *
                     pnc::mathlib::Square(planning_input_.ref_vel());

  // assemble proto for pnc tools
  for (size_t i = 0; i < N; ++i) {
    planning_output_.mutable_time_vec()->Set(i, t);
    t += dt;

    x_vec_[i] = state_result->at(i)[pnc::lateral_planning::StateId::X];
    planning_output_.mutable_x_vec()->Set(i, x_vec_[i]);

    y_vec_[i] = state_result->at(i)[pnc::lateral_planning::StateId::Y];
    planning_output_.mutable_y_vec()->Set(i, y_vec_[i]);

    theta_vec_[i] = state_result->at(i)[pnc::lateral_planning::StateId::THETA];
    planning_output_.mutable_theta_vec()->Set(i, theta_vec_[i]);

    delta_vec_[i] = state_result->at(i)[pnc::lateral_planning::StateId::DELTA];
    planning_output_.mutable_delta_vec()->Set(i, delta_vec_[i]);

    omega_vec_[i] = state_result->at(i)[pnc::lateral_planning::StateId::OMEGA];
    planning_output_.mutable_omega_vec()->Set(i, omega_vec_[i]);

    curv_vec[i] = planning_input_.curv_factor() *
                  state_result->at(i)[pnc::lateral_planning::StateId::DELTA];

    planning_output_.mutable_acc_vec()->Set(
        i, kv2 * planning_output_.delta_vec(i));

    planning_output_.mutable_jerk_vec()->Set(
        i, kv2 * planning_output_.omega_vec(i));

    if (i < N - 1) {
      planning_output_.mutable_omega_dot_vec()->Set(
          i,
          control_result->at(i)[pnc::lateral_planning::ControlId::OMEGA_DOT]);

      d_curv_vec[i] =
          planning_input_.curv_factor() *
          control_result->at(i)[pnc::lateral_planning::ControlId::OMEGA_DOT];
    } else {
      planning_output_.mutable_omega_dot_vec()->Set(
          i, planning_output_.omega_dot_vec(i - 1));

      d_curv_vec[i] = d_curv_vec[i - 1];
    }

    if (i == 0) {
      s = 0.0;
    } else {
      const double ds =
          std::hypot(x_vec_[i] - x_vec_[i - 1], y_vec_[i] - y_vec_[i - 1]);
      s += std::max(ds, 1e-3);
    }
    s_vec_[i] = s;
  }

  // generate motion planning output into planning_context
  auto &traj_spline = frame_->mutable_session()
                          ->mutable_planning_context()
                          ->mutable_planning_result()
                          .traj_spline;

  // set state spline
  traj_spline.x_s_spline.set_points(s_vec_, x_vec_);
  traj_spline.y_s_spline.set_points(s_vec_, y_vec_);
  traj_spline.theta_s_spline.set_points(s_vec_, theta_vec_);
  traj_spline.delta_s_spline.set_points(s_vec_, delta_vec_);
  traj_spline.omega_s_spline.set_points(s_vec_, omega_vec_);
  traj_spline.curv_s_spline.set_points(s_vec_, curv_vec);
  traj_spline.d_curv_s_spline.set_points(s_vec_, d_curv_vec);

  traj_spline.enable_flag = true;

  traj_spline.x_vec = x_vec_;
  traj_spline.y_vec = y_vec_;
  traj_spline.s_vec = s_vec_;

  // assemble results
  const auto &v_cruise = frame_->session()
                             ->environmental_model()
                             .get_ego_state_manager()
                             ->ego_v_cruise();
  v_cruise_ = v_cruise;

  auto &traj_points = pipeline_context_->planning_result.traj_points;
  for (size_t i = 0; i < N; i++) {
    traj_points[i].x = state_result->at(i)[pnc::lateral_planning::StateId::X];
    traj_points[i].y = state_result->at(i)[pnc::lateral_planning::StateId::Y];
    traj_points[i].heading_angle =
        state_result->at(i)[pnc::lateral_planning::StateId::THETA];
    traj_points[i].curvature =
        planning_input_.curv_factor() *
        state_result->at(i)[pnc::lateral_planning::StateId::DELTA];

    traj_points[i].v = v_cruise;
    traj_points[i].t = planning_output_.time_vec(i);

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

  frame_->mutable_session()
      ->mutable_planning_context()
      ->mutable_planning_result()
              .init_flag = true;
}

void LateralMotionPlanner::UpdateOneStepTrajectory() {}

} // namespace planning