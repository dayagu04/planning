#include "longitudinal_motion_planner.h"
#include "debug_info_log.h"
#include "math_lib.h"

namespace planning {
LongitudinalMotionPlanner::LongitudinalMotionPlanner(
    const EgoPlanningConfigBuilder *config_builder,
    const std::shared_ptr<TaskPipelineContext> &pipeline_context)
    : Task(config_builder, pipeline_context) {
  config_ = config_builder->cast<LongitudinalMotionPlannerConfig>();
  config_start_stop_ = config_builder->cast<StartStopEnableConfig>();
  config_acc_ = config_builder->cast<AdaptiveCruiseControlConfig>();
  name_ = "LongitudinalMotionPlanner";

  Init();
};

void LongitudinalMotionPlanner::Init() {
  // init planning problem
  planning_problem_ptr_ = std::make_shared<
      pnc::longitudinal_planning::LongitudinalMotionPlanningProblem>();
  planning_problem_ptr_->Init();

  // init planning input and output
  auto const N =
      planning_problem_ptr_->GetiLqrCorePtr()->GetSolverConfigPtr()->horizon +
      1;

  // init planning input
  planning_input_.mutable_ref_pos_vec()->Resize(N, 0.0);
  planning_input_.mutable_ref_vel_vec()->Resize(N, 0.0);

  planning_input_.mutable_pos_max_vec()->Resize(N, 0.0);
  planning_input_.mutable_pos_min_vec()->Resize(N, 0.0);
  planning_input_.mutable_vel_max_vec()->Resize(N, 0.0);
  planning_input_.mutable_vel_min_vec()->Resize(N, 0.0);
  planning_input_.mutable_acc_max_vec()->Resize(N, 0.0);
  planning_input_.mutable_acc_min_vec()->Resize(N, 0.0);
  planning_input_.mutable_jerk_max_vec()->Resize(N, 0.0);
  planning_input_.mutable_jerk_min_vec()->Resize(N, 0.0);

  // init planning output
  planning_output_.mutable_time_vec()->Resize(N, 0.0);
  planning_output_.mutable_pos_vec()->Resize(N, 0.0);
  planning_output_.mutable_vel_vec()->Resize(N, 0.0);
  planning_output_.mutable_acc_vec()->Resize(N, 0.0);
  planning_output_.mutable_jerk_vec()->Resize(N, 0.0);
  planning_output_.mutable_snap_vec()->Resize(N, 0.0);

  // init state vector
  s_vec_.resize(N);
  v_vec_.resize(N);
  a_vec_.resize(N);
  j_vec_.resize(N);
  t_vec_.resize(N);
}

bool LongitudinalMotionPlanner::Execute(planning::framework::Frame *frame) {
  LOG_DEBUG("=======LongitudinalMotionPlanner======= \n");
  frame_ = frame;
  if (Task::Execute(frame) == false) {
    return false;
  }

  GeneratePlanningInput();

  planning_problem_ptr_->Update(planning_input_);

  GeneratePlanningOutput();

  // record input and output
  DebugInfoManager::GetInstance()
      .GetDebugInfoPb()
      ->mutable_longitudinal_motion_planning_input()
      ->CopyFrom(planning_input_);

  DebugInfoManager::GetInstance()
      .GetDebugInfoPb()
      ->mutable_longitudinal_motion_planning_output()
      ->CopyFrom(planning_output_);

  return true;
}

void LongitudinalMotionPlanner::GeneratePlanningInput() {
  const auto &lon_ref_path = // result from lon decision
      pipeline_context_->planning_info.lon_ref_path;

  const auto &s_refs = lon_ref_path.s_refs;
  const auto &v_refs = lon_ref_path.ds_refs;
  const auto &s_bounds = lon_ref_path.bounds;
  const auto &v_bounds = lon_ref_path.lon_bound_v;
  const auto &a_bounds = lon_ref_path.lon_bound_a;
  const auto &jerk_bounds = lon_ref_path.lon_bound_jerk;

  // const auto &enable_dx_ref = frame_->mutable_session()
  //                                 ->mutable_planning_context()
  //                                 ->mutable_adaptive_cruise_control_result()
  //                                 .navi_speed_control_info.enable_v_cost;

  // const auto &enable_stop_flag = frame_->mutable_session()
  //                                    ->mutable_planning_context()
  //                                    ->mutable_start_stop_result()
  //                                    .enable_stop;

  // const auto &mrc_brake_type = frame_->mutable_session()
  //                                  ->mutable_planning_context()
  //                                  ->mrc_condition()
  //                                  ->mrc_brake_type();

  // const bool mrc_condition_enable =
  //     mrc_brake_type == MrcBrakeType::SLOW_BRAKE ||
  //     mrc_brake_type == MrcBrakeType::HARD_BRAKE ||
  //     mrc_brake_type == MrcBrakeType::EMERGENCY_BRAKE;

  // set ref_pos and ref_vel
  for (size_t i = 0; i < s_refs.size(); ++i) {
    auto const &dt =
        planning_problem_ptr_->GetiLqrCorePtr()->GetSolverConfigPtr()->model_dt;

    auto const &v_ref = v_refs[i].first;
    auto s_ref = s_refs[i].first;

    // note that s_ref should be limited by v_ref
    if (i > 0) {
      auto const &last_s_ref = planning_input_.ref_pos_vec(i - 1);
      s_ref = pnc::mathlib::Limit(s_ref, last_s_ref + dt * v_ref);
    }

    planning_input_.mutable_ref_pos_vec()->Set(i, s_ref);
    planning_input_.mutable_ref_vel_vec()->Set(i, v_ref);
  }

  // TODO: s bound should know soft or hard
  // set s bounds
  for (size_t i = 0; i < s_bounds.size(); ++i) {
    Bound tmp_bound{-1.0e4, 1.0e4};
    for (auto &bound : s_bounds[i]) {
      tmp_bound.lower = std::fmax(bound.lower, tmp_bound.lower);
      tmp_bound.upper = std::fmin(bound.upper, tmp_bound.upper);
    }

    s_limit_.lower =
        tmp_bound.lower > s_limit_.lower ? tmp_bound.lower : s_limit_.lower;
    s_limit_.upper =
        tmp_bound.upper < s_limit_.upper ? tmp_bound.upper : s_limit_.upper;

    planning_input_.mutable_pos_max_vec()->Set(i, tmp_bound.upper);
    planning_input_.mutable_pos_min_vec()->Set(i, tmp_bound.lower);
  }

  // set vel bounds
  for (size_t i = 0; i < v_bounds.size(); ++i) {
    planning_input_.mutable_vel_max_vec()->Set(i, v_bounds[i].upper);
    planning_input_.mutable_vel_min_vec()->Set(i, v_bounds[i].lower);
  }

  // set acc bounds
  for (size_t i = 0; i < a_bounds.size(); ++i) {
    planning_input_.mutable_acc_max_vec()->Set(i, a_bounds[i].upper);
    planning_input_.mutable_acc_min_vec()->Set(i, a_bounds[i].lower);
  }

  // jerk bounds
  for (size_t i = 0; i < jerk_bounds.size(); ++i) {
    planning_input_.mutable_jerk_max_vec()->Set(i, jerk_bounds[i].upper);
    planning_input_.mutable_jerk_min_vec()->Set(i, jerk_bounds[i].lower);
  }

  // set init state
  const auto &planning_init_point =
      reference_path_ptr_->get_frenet_ego_state().planning_init_point();

  // init s uses frenet state
  planning_input_.mutable_init_state()->set_s(
      planning_init_point.frenet_state.s);

  planning_input_.mutable_init_state()->set_v(
      planning_init_point.lon_init_state.v());

  planning_input_.mutable_init_state()->set_a(
      planning_init_point.lon_init_state.a());

  planning_input_.mutable_init_state()->set_j(
      planning_init_point.lon_init_state.j());

  // set weights
  planning_input_.set_q_ref_pos(config_.q_ref_pos);
  planning_input_.set_q_ref_vel(config_.q_ref_vel);
  planning_input_.set_q_acc(config_.q_acc);
  planning_input_.set_q_jerk(config_.q_jerk);
  planning_input_.set_q_snap(config_.q_snap);
  planning_input_.set_q_stop_s(config_.q_stop_s);

  planning_input_.set_q_pos_bound(config_.q_pos_bound);
  planning_input_.set_q_vel_bound(config_.q_vel_bound);
  planning_input_.set_q_acc_bound(config_.q_acc_bound);
  planning_input_.set_q_jerk_bound(config_.q_jerk_bound);

  // what is s_stop?
  planning_input_.set_s_stop(1.0e4); // TBD: hack for input;
}

void LongitudinalMotionPlanner::GeneratePlanningOutput() {
  // assembling planning output proto
  const auto &state_result =
      planning_problem_ptr_->GetiLqrCorePtr()->GetStateResultPtr();
  const auto &control_result =
      planning_problem_ptr_->GetiLqrCorePtr()->GetControlResultPtr();
  const size_t N =
      planning_problem_ptr_->GetiLqrCorePtr()->GetSolverConfigPtr()->horizon +
      1;
  const auto &dt =
      planning_problem_ptr_->GetiLqrCorePtr()->GetSolverConfigPtr()->model_dt;

  double t = 0.;

  // assemble proto for pnc tools
  for (size_t i = 0; i < N; ++i) {
    planning_output_.mutable_time_vec()->Set(i, t);
    t_vec_[i] = t;

    t += dt;

    s_vec_[i] = state_result->at(i)[pnc::longitudinal_planning::StateId::POS];
    planning_output_.mutable_pos_vec()->Set(i, s_vec_[i]);

    v_vec_[i] = state_result->at(i)[pnc::longitudinal_planning::StateId::VEL];
    planning_output_.mutable_vel_vec()->Set(i, v_vec_[i]);

    a_vec_[i] = state_result->at(i)[pnc::longitudinal_planning::StateId::ACC];
    planning_output_.mutable_acc_vec()->Set(i, a_vec_[i]);

    j_vec_[i] = state_result->at(i)[pnc::longitudinal_planning::StateId::JERK];
    planning_output_.mutable_jerk_vec()->Set(i, j_vec_[i]);

    if (i < N - 1) {
      planning_output_.mutable_snap_vec()->Set(
          i,
          control_result->at(i)[pnc::longitudinal_planning::ControlId::SNAP]);
    } else {
      planning_output_.mutable_snap_vec()->Set(
          i, planning_output_.snap_vec(i - 1));
    };
  }

  // generate motion planning output into planning_context
  auto &traj_spline = frame_->mutable_session()
                          ->mutable_planning_context()
                          ->mutable_planning_result()
                          .traj_spline;

  traj_spline.s_t_spline.set_points(t_vec_, s_vec_);
  traj_spline.v_t_spline.set_points(t_vec_, v_vec_);
  traj_spline.a_t_spline.set_points(t_vec_, a_vec_);
  traj_spline.j_t_spline.set_points(t_vec_, j_vec_);

  traj_spline.lon_enable_flag = true;

  // assemble trajectory that combining lateral and longitudinal planning_result
  auto &traj_points = pipeline_context_->planning_result.traj_points;
  for (size_t i = 0; i < N; ++i) {
    traj_points[i].v = v_vec_[i];
    traj_points[i].a = a_vec_[i];
    traj_points[i].t = planning_output_.time_vec(i);

    // lateral path resampling
    // s is lateral path rather than longitudinal path (frenet)
    // considering an offset that equals to init s
    const auto &s = s_vec_[i] - s_vec_[0];
    traj_points[i].x = traj_spline.x_s_spline(s);
    traj_points[i].y = traj_spline.y_s_spline(s);
    traj_points[i].heading_angle =
        pnc::mathlib::DeltaAngleFix(traj_spline.theta_s_spline(s));

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

  // bool LongitudinalMotionPlanner::RecheckPlanningResult() {
  //   const auto &lon_ref_path = // result from lon decision
  //       pipeline_context_->planning_info.lon_ref_path;
  //   const auto &state_result =
  //       planning_problem_ptr_->GetiLqrCorePtr()->GetStateResultPtr();

  //   const auto &num_t = lon_ref_path.t_list.size();

  //   for (size_t i = 0; i < num_t; i++) {
  //     if (state_result->at(i)[pnc::longitudinal_planning::StateId::POS] <
  //             s_limit_.lower ||
  //         state_result->at(i)[pnc::longitudinal_planning::StateId::POS] >
  //             s_limit_.upper) {
  //       LOG_ERROR("[ILqrLonPlanning]: Error! s bound collide! %d %f [%f %f]
  //       \n",
  //                 i,
  //                 state_result->at(i)[pnc::longitudinal_planning::StateId::POS],
  //                 s_limit_.lower, s_limit_.upper);
  //     }

  //     if (state_result->at(i)[pnc::longitudinal_planning::StateId::ACC] >
  //             config_.kMaxAcc ||
  //         state_result->at(i)[pnc::longitudinal_planning::StateId::ACC] <
  //             config_.kMinDec) {
  //       LOG_ERROR("[ILqrLonPlanning]: Error! Invalid acc %f ! \n",
  //                 state_result->at(i)[pnc::longitudinal_planning::StateId::ACC]);
  //     }

  //     if (std::fabs(state_result->at(
  //             i)[pnc::longitudinal_planning::StateId::JERK]) >
  //             config_.kMaxJerk) {
  //       LOG_ERROR("[ILqrLonPlanning]: Error! Invalid jerk %f ! \n",
  //                 state_result->at(i)[pnc::longitudinal_planning::StateId::JERK]);
  //     }
  //   }
}
} // namespace planning
