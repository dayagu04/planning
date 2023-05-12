
#include "general_lateral_motion_planner.h"

namespace planning {
GeneralLateralMotionPlanner::GeneralLateralMotionPlanner(
    const EgoPlanningConfigBuilder *config_builder,
    const std::shared_ptr<TaskPipelineContext> &pipeline_context)
    : Task(config_builder, pipeline_context) {
  config_ = config_builder->cast<GeneralLateralMotionPlannerConfig>();
  name_ = "GeneralLateralMotionPlanner";
  lat_motion_planning_problem_ptr_ =
      std::make_shared<pnc::lateral_planning::LateralMotionPlanningProblem>();
  lat_motion_planning_problem_ptr_->Init();
};

bool GeneralLateralMotionPlanner::Execute(planning::framework::Frame *frame) {
  generate_lat_motion_planner_input();

  lat_motion_planning_problem_ptr_->Update(planning_input_);

  generate_lat_motion_planner_output();
  return false;
}

bool GeneralLateralMotionPlanner::generate_lat_motion_planner_input() {
  const auto &lat_decider_output =  // result from lat decision
      frame_->session()->planning_context().lat_decider_output();

  const auto &enu_ref_path = lat_decider_output.enu_ref_path;
  const auto &enu_ref_theta = lat_decider_output.enu_ref_theta;
  assert(enu_ref_path.size() == enu_ref_theta.size());

  for (size_t i = 0; i < enu_ref_path.size(); ++i) {
    planning_input_.add_ref_x_vec(enu_ref_path[i].first);
    planning_input_.add_ref_y_vec(enu_ref_path[i].second);
    planning_input_.add_ref_theta_vec(
        enu_ref_theta[i]);  // TODO: default 0 theta currently;
  }

  const auto &safe_bounds = lat_decider_output.safe_bounds;
  const auto &path_bounds = lat_decider_output.path_bounds;

  assert(safe_bounds.size() == path_bounds.size());

  for (size_t i = 0; i < safe_bounds.size() - 1; ++i) {
    const auto &safe_lower_bound = safe_bounds[i].first;
    const auto &safe_upper_bound = safe_bounds[i].second;
    const auto &next_safe_lower_bound = safe_bounds[i + 1].first;
    const auto &next_safe_upper_bound = safe_bounds[i + 1].second;

    planning_input_.add_soft_lower_bound_x0_vec(safe_lower_bound.x);
    planning_input_.add_soft_lower_bound_y0_vec(safe_lower_bound.y);
    planning_input_.add_soft_lower_bound_x1_vec(next_safe_lower_bound.x);
    planning_input_.add_soft_lower_bound_y1_vec(next_safe_lower_bound.y);

    planning_input_.add_soft_upper_bound_x0_vec(safe_upper_bound.x);
    planning_input_.add_soft_upper_bound_y0_vec(safe_upper_bound.y);
    planning_input_.add_soft_upper_bound_x1_vec(next_safe_upper_bound.x);
    planning_input_.add_soft_upper_bound_y1_vec(next_safe_upper_bound.y);

    const auto &path_lower_bound = path_bounds[i].first;
    const auto &path_upper_bound = path_bounds[i].second;
    const auto &next_path_lower_bound = path_bounds[i + 1].first;
    const auto &next_path_upper_bound = path_bounds[i + 1].second;

    planning_input_.add_hard_lower_bound_x0_vec(path_lower_bound.x);
    planning_input_.add_hard_lower_bound_y0_vec(path_lower_bound.y);
    planning_input_.add_hard_lower_bound_x1_vec(next_path_lower_bound.x);
    planning_input_.add_hard_lower_bound_y1_vec(next_path_lower_bound.y);

    planning_input_.add_hard_upper_bound_x0_vec(path_upper_bound.x);
    planning_input_.add_hard_upper_bound_y0_vec(path_upper_bound.y);
    planning_input_.add_hard_upper_bound_x1_vec(next_path_upper_bound.x);
    planning_input_.add_hard_upper_bound_y1_vec(next_path_upper_bound.y);
  }

  const auto &last_safe_lower_bound = safe_bounds[safe_bounds.size() - 1].first;
  const auto &last_safe_upper_bound =
      safe_bounds[safe_bounds.size() - 1].second;
  const auto &last_second_safe_lower_bound =
      safe_bounds[safe_bounds.size() - 2].first;
  const auto &last_second_safe_upper_bound =
      safe_bounds[safe_bounds.size() - 2].second;

  // last point bound
  planning_input_.add_soft_lower_bound_x0_vec(last_second_safe_lower_bound.x);
  planning_input_.add_soft_lower_bound_y0_vec(last_second_safe_lower_bound.y);
  planning_input_.add_soft_lower_bound_x1_vec(last_safe_lower_bound.x);
  planning_input_.add_soft_lower_bound_y1_vec(last_safe_lower_bound.y);

  planning_input_.add_soft_upper_bound_x0_vec(last_second_safe_upper_bound.x);
  planning_input_.add_soft_upper_bound_y0_vec(last_second_safe_upper_bound.y);
  planning_input_.add_soft_upper_bound_x1_vec(last_safe_upper_bound.x);
  planning_input_.add_soft_upper_bound_y1_vec(last_safe_upper_bound.y);

  const auto &last_path_lower_bound = path_bounds[path_bounds.size() - 1].first;
  const auto &last_path_upper_bound =
      path_bounds[path_bounds.size() - 1].second;
  const auto &last_second_path_lower_bound =
      path_bounds[path_bounds.size() - 2].first;
  const auto &last_second_path_upper_bound =
      path_bounds[path_bounds.size() - 2].second;

  planning_input_.add_hard_lower_bound_x0_vec(last_second_path_lower_bound.x);
  planning_input_.add_hard_lower_bound_y0_vec(last_second_path_lower_bound.y);
  planning_input_.add_hard_lower_bound_x1_vec(last_path_lower_bound.x);
  planning_input_.add_hard_lower_bound_y1_vec(last_path_lower_bound.y);

  planning_input_.add_hard_upper_bound_x0_vec(last_second_path_upper_bound.x);
  planning_input_.add_hard_upper_bound_y0_vec(last_second_path_upper_bound.y);
  planning_input_.add_hard_upper_bound_x1_vec(last_path_upper_bound.x);
  planning_input_.add_hard_upper_bound_y1_vec(last_path_upper_bound.y);

  planning_input_.set_ref_vel(lat_decider_output.v_cruise);

  // hack: use current traj
  //  auto &last_enu_ref_path =
  //      frame_->session()->planning_context().mutable_last_lat_enu_ref_path();

  // auto &last_enu_ref_theta =
  //     frame_->session()->planning_context().mutable_last_enu_ref_theta();

  // if (last_enu_ref_path.size() == 0) {  // if the first frame;
  //   last_enu_ref_path = enu_ref_path;
  // }
  // if (last_enu_ref_theta.size() == 0) {  // if the first frame;
  //   last_enu_ref_theta = enu_ref_theta;
  // }

  for (size_t i = 0; i < enu_ref_path.size(); ++i) {
    planning_input_.add_last_x_vec(enu_ref_path[i].first);
    planning_input_.add_last_y_vec(enu_ref_path[i].second);
    planning_input_.add_last_theta_vec(enu_ref_theta[i]);
  }

  const auto &init_state = lat_decider_output.init_state;
  for (size_t i = 0; i < init_state.size(); ++i) {
    planning_input_.add_init_state(init_state[i]);
  }

  // set config parameters;
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

  // frame_->session()->planning_context().mutable_last_enu_ref_theta() =
  //     lat_decider_output.enu_ref_theta;

  // frame_->session()->planning_context().mutable_last_lat_enu_ref_path() =
  //     lat_decider_output.enu_ref_path;

  return true;
}

bool GeneralLateralMotionPlanner::generate_lat_motion_planner_output() {
  // assembling planning output proto
  auto const &state_result =
      lat_motion_planning_problem_ptr_->ilqr_core_ptr()->GetStateResultPtr();
  auto const &control_result =
      lat_motion_planning_problem_ptr_->ilqr_core_ptr()->GetControlResultPtr();
  const size_t N = lat_motion_planning_problem_ptr_->ilqr_core_ptr()
                       ->GetSolverConfigPtr()
                       ->horizion +
                   1;
  // assemble proto for pnc tools
  for (size_t i = 0; i < N; ++i) {
    planning_output_.add_x_vec(
        state_result->at(i)[pnc::lateral_planning::StateId::X]);
    planning_output_.add_y_vec(
        state_result->at(i)[pnc::lateral_planning::StateId::Y]);
    planning_output_.add_theta_vec(
        state_result->at(i)[pnc::lateral_planning::StateId::THETA]);
    planning_output_.add_delta_vec(
        state_result->at(i)[pnc::lateral_planning::StateId::DELTA]);
    planning_output_.add_omega_vec(
        state_result->at(i)[pnc::lateral_planning::StateId::OMEGA]);
    planning_output_.add_acc_vec(
        planning_input_.curv_factor() *
        pnc::mathlib::Square(planning_input_.ref_vel()) *
        (planning_output_.delta_vec(i)));
    planning_output_.add_jerk_vec(
        planning_input_.curv_factor() *
        pnc::mathlib::Square(planning_input_.ref_vel()) *
        (planning_output_.omega_vec(i)));

    if (i < N - 1) {
      planning_output_.add_omega_dot_vec(
          control_result->at(i)[pnc::lateral_planning::ControlId::OMEGA_DOT]);
    } else {
      planning_output_.add_omega_dot_vec(planning_output_.omega_dot_vec(N - 2));
    };
  }

  // generate motion planning output into planning_context

  auto &traj_points = pipeline_context_->planning_result.traj_points;
  assert(traj_points.size() == N);

  TrajectoryPoint point;
  const auto &planning_init_point = frame_->session()
                                        ->environmental_model()
                                        .get_ego_state_manager()
                                        ->planning_init_point();
  // const auto &frenet_coord = reference_path_ptr_->get_frenet_coord();
  const auto &v_cruise = frame_->session()
                             ->environmental_model()
                             .get_ego_state_manager()
                             ->ego_v_cruise();
  for (size_t i = 0; i < N; i++) {
    point.x = i == 0 ? planning_init_point.x
                     : state_result->at(i)[pnc::lateral_planning::StateId::X];
    point.y = i == 0 ? planning_init_point.y
                     : state_result->at(i)[pnc::lateral_planning::StateId::Y];

    Point2D frenet_pt;
    Point2D cart_pt{point.x, point.y};
    if (reference_path_ptr_->get_frenet_coord() != nullptr &&
        reference_path_ptr_->get_frenet_coord()->CartCoord2FrenetCoord(
            cart_pt, frenet_pt) == TRANSFORM_STATUS::TRANSFORM_SUCCESS) {
      point.s = frenet_pt.x;
      point.l = frenet_pt.y;
    } else {
      LOG_DEBUG(
          "CartCoord2FrenetCoord = FAILED !!!!!!!! index: %d,  point.s : "
          "%f, point.l: %f ",
          i, point.s, point.l);
    }

    point.v = i == 0 ? planning_init_point.v : v_cruise;
    point.t = i * config_.delta_t;
    traj_points.emplace_back(point);
  }
}

bool GeneralLateralMotionPlanner::update_one_step_traj() {}

}  // namespace planning