#include "longitudinal_motion_planner.h"
#include "math_lib.h"

namespace planning {
LongitudinalMotionPlanner::LongitudinalMotionPlanner(
    const EgoPlanningConfigBuilder *config_builder,
    const std::shared_ptr<TaskPipelineContext> &pipeline_context)
    : Task(config_builder, pipeline_context) {
  config_ = config_builder->cast<ILqrLonMotionPlannerConfig>();
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

  // recheck the planning result
  // RecheckPlanningResult();

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

  assert(s_refs.size() == v_refs.size());
  assert(s_refs.size() == config_.horizon + 1);
  assert(s_bounds.size() == config_.horizon + 1);

  const auto &enable_dx_ref = frame_->mutable_session()
                                  ->mutable_planning_context()
                                  ->mutable_adaptive_cruise_control_result()
                                  .navi_speed_control_info.enable_v_cost;

  const auto &enable_stop_flag = frame_->mutable_session()
                                     ->mutable_planning_context()
                                     ->mutable_start_stop_result()
                                     .enable_stop;

  const auto &mrc_brake_type = frame_->mutable_session()
                                   ->mutable_planning_context()
                                   ->mrc_condition()
                                   ->mrc_brake_type();

  const bool mrc_condition_enable =
      mrc_brake_type == MrcBrakeType::SLOW_BRAKE ||
      mrc_brake_type == MrcBrakeType::HARD_BRAKE ||
      mrc_brake_type == MrcBrakeType::EMERGENCY_BRAKE;

  double weight_x;
  double weight_dx;
  double weight_ddx;
  double weight_dddx;
  double weight_ddddx;

  if (enable_stop_flag) {
    weight_dx = config_start_stop_.dx_ref_weight;
    weight_x = 0.;
  } else {
    if (mrc_condition_enable || enable_dx_ref) {
      weight_dx = config_acc_.dx_ref_weight;
      weight_x = 0.;
    } else {
      weight_dx = 0.;
      weight_x = 1.;
    }
  }
  LOG_DEBUG(
      "[LongitudinalMotionPlanner] enable_stop_flag: %d, enable_dx_ref: %d,"
      "acc_weight_dx_config: %.2f, stop_weight_dx_config: %.2f \n",
      enable_stop_flag, enable_dx_ref, config_acc_.dx_ref_weight,
      config_start_stop_.dx_ref_weight);

  weight_ddx = 1.0;
  weight_dddx = 100.;
  weight_ddddx = 1000.; // TBD: adjust in pnc tools;

  for (size_t i = 0; i < s_refs.size(); ++i) {
    planning_input_.mutable_ref_pos_vec()->Set(i, s_refs[i].first);
    planning_input_.mutable_ref_vel_vec()->Set(i, s_refs[i].first);
  }

  // FBI WARNING: s bound should know soft or hard?
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

  planning_input_.mutable_init_state()->set_s(
      planning_init_point.lon_init_state.s());

  planning_input_.mutable_init_state()->set_v(
      planning_init_point.lon_init_state.v());

  planning_input_.mutable_init_state()->set_a(
      planning_init_point.lon_init_state.a());

  planning_input_.mutable_init_state()->set_j(
      planning_init_point.lon_init_state.j());

  // set weights
  planning_input_.set_q_ref_pos(weight_x);
  planning_input_.set_q_ref_vel(weight_dx);
  planning_input_.set_q_acc(weight_ddx);
  planning_input_.set_q_jerk(weight_dddx);
  planning_input_.set_q_snap(weight_ddddx);

  planning_input_.set_q_pos_bound(
      config_.q_pos_bound); // TBD: adjust in pnc tools;
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

  auto &traj_points = pipeline_context_->planning_result.traj_points;

  // assemble trajectory that combining lateral and longitudinal planning_result
  for (size_t i = 0; i < N; ++i) {
    traj_points[i].s = s_vec_[i];
    traj_points[i].v = v_vec_[i];
    traj_points[i].a = a_vec_[i];
    traj_points[i].x = traj_spline.x_s_spline(s_vec_[i]);
    traj_points[i].y = traj_spline.y_s_spline(s_vec_[i]);
    traj_points[i].heading_angle =
        pnc::mathlib::DeltaAngleFix(traj_spline.theta_s_spline(s_vec_[i]));
  }

  frame_->mutable_session()
      ->mutable_planning_context()
      ->mutable_planning_result()
      .init_flag = true;

  // frenet is no longer used
  // std::vector<double> l;
  // std::vector<double> heading_angle;
  // std::vector<double> curvature;
  // l.reserve(N);
  // heading_angle.reserve(N);
  // curvature.reserve(N);

  // interpolate_frenet_lon(traj_points, s_vec_, l, heading_angle, curvature);

  // for (size_t i = 0; i < traj_points.size(); i++) {
  //   traj_points[i].s = s_vec_[i];
  //   traj_points[i].v = v_vec_[i];
  //   traj_points[i].a = a_vec_[i];
  //   traj_points[i].l = l[i];
  //   traj_points[i].heading_angle = heading_angle[i];
  //   traj_points[i].curvature = curvature[i];
  // }
}

// void LongitudinalMotionPlanner::interpolate_frenet_lon(
//     const std::vector<TrajectoryPoint> &traj_points,
//     const std::vector<double> &s, std::vector<double> &l,
//     std::vector<double> &heading_angle, std::vector<double> &curvature) {
//   const auto &state_result =
//       planning_problem_ptr_->GetiLqrCorePtr()->GetStateResultPtr();

//   const size_t N =
//       planning_problem_ptr_->GetiLqrCorePtr()->GetSolverConfigPtr()->horizon
//       + 1;

//   size_t j = 0;
//   for (size_t i = 1; i < N; i++) {
//     if (state_result->at(i)[pnc::longitudinal_planning::StateId::POS] >=
//         traj_points.back().s) {
//       l[i] = traj_points.back().l;
//       heading_angle[i] = traj_points.back().heading_angle;
//       curvature[i] = traj_points.back().curvature;
//       continue;

//     } else if (state_result->at(i)[pnc::longitudinal_planning::StateId::POS]
//     <=
//                traj_points.front().s) {
//       l[i] = traj_points.front().l;
//       heading_angle[i] = traj_points.front().heading_angle;
//       curvature[i] = traj_points.front().curvature;
//       continue;
//     }
//     bool found_left_right = false;
//     while (j + 1 < traj_points.size()) {
//       if (traj_points[j].s <= s[i] and s[i] < traj_points[j + 1].s) {
//         found_left_right = true;
//         break;
//       }
//       ++j;
//     }

//     // compute l_i
//     if (not found_left_right) {
//       l[i] = traj_points.back().l;
//       heading_angle[i] = traj_points.back().heading_angle;
//       curvature[i] = traj_points.back().curvature;
//     } else {
//       auto ratio = 1.0;
//       if ((traj_points[j + 1].s - traj_points[j].s) > 1e-2) {
//         ratio = (traj_points[j + 1].s - s[i]) /
//                 (traj_points[j + 1].s - traj_points[j].s);
//       }

//       l[i] = planning_math::Interpolate(traj_points[j].l, traj_points[j +
//       1].l,
//                                         ratio);
//       heading_angle[i] = planning_math::InterpolateAngle(
//           traj_points[j].heading_angle, traj_points[j + 1].heading_angle,
//           ratio);
//       curvature[i] = planning_math::Interpolate(
//           traj_points[j].curvature, traj_points[j + 1].curvature, ratio);
//     }
//   }
// }

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
// }

} // namespace planning