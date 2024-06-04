
#include "lateral_motion_planner.h"

#include <algorithm>
#include <cstddef>
#include <iostream>
#include <vector>

#include "debug_info_log.h"
#include "ilqr_define.h"
#include "lateral_motion_planner.pb.h"
#include "planning_context.h"
#include "spline.h"
#include "src/lateral_motion_planning_cost.h"
#include "src/lateral_motion_planning_weight.h"
#include "virtual_lane_manager.h"

static const double pi_const = 3.141592654;
static const double planning_loop_dt = 0.1;
namespace planning {
LateralMotionPlanner::LateralMotionPlanner(
    const EgoPlanningConfigBuilder *config_builder, framework::Session *session)
    : Task(config_builder, session) {
  config_ = config_builder->cast<LateralMotionPlannerConfig>();
  name_ = "LateralMotionPlanner";

  Init();
};

void LateralMotionPlanner::Init() {
  // init planning weight
  planning_weight_ptr_ =
      std::make_shared<pnc::lateral_planning::LateralMotionPlanningWeight>(
          config_);
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

bool LateralMotionPlanner::Execute() {
  LOG_DEBUG("=======LateralMotionPlanner======= \n");

  if (!PreCheck()) {
    LOG_DEBUG("PreCheck failed\n");
    return false;
  }

  auto start_time = IflyTime::Now_ms();

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

  auto end_time = IflyTime::Now_ms();
  JSON_DEBUG_VALUE("LateralMotionCostTime", end_time - start_time);

  return true;
}

void LateralMotionPlanner::AssembleInput() {
  // set init state
  const auto &reference_path_ptr = session_->planning_context()
                                       .lane_change_decider_output()
                                       .coarse_planning_info.reference_path;
  const auto &planning_init_point =
      reference_path_ptr->get_frenet_ego_state().planning_init_point();
  const auto &motion_planner_output =
      session_->planning_context().motion_planner_output();

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
  const auto &general_lateral_decider_output =  // result from lat decision
      session_->planning_context().general_lateral_decider_output();

  const auto &enu_ref_path = general_lateral_decider_output.enu_ref_path;
  const auto &enu_ref_theta = general_lateral_decider_output.enu_ref_theta;
  const bool &complete_follow = general_lateral_decider_output.complete_follow;
  const bool &lane_change_scene =
      general_lateral_decider_output.lane_change_scene;
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

  // set init theta by ref_theta: not solid
  // planning_input_.mutable_init_state()->set_theta(
  //     planning_input_.ref_theta_vec(0));

  // set last trajectory: temporarily same as reference: TODO
  double final_t = 5.0;  // hack now
  double tmp_t = 0.0;
  if (motion_planner_output.lat_init_flag == true) {
    for (size_t i = 0; i < enu_ref_path.size(); ++i) {
      tmp_t = std::fmin(planning_loop_dt + i * 0.2, final_t);
      planning_input_.mutable_last_x_vec()->Set(
          i, motion_planner_output.lateral_x_t_spline(tmp_t));
      planning_input_.mutable_last_y_vec()->Set(
          i, motion_planner_output.lateral_y_t_spline(tmp_t));
      planning_input_.mutable_last_theta_vec()->Set(
          i, motion_planner_output.lateral_theta_t_spline(tmp_t));
    }

  } else {
    for (size_t i = 0; i < enu_ref_path.size(); ++i) {
      planning_input_.mutable_last_x_vec()->Set(i, enu_ref_path[i].first);
      planning_input_.mutable_last_y_vec()->Set(i, enu_ref_path[i].second);
      planning_input_.mutable_last_theta_vec()->Set(i, enu_ref_theta[i]);
    }
  }

  // set safe (hard) and path (soft) bound
  const auto &safe_bounds = general_lateral_decider_output.safe_bounds;
  const auto &path_bounds = general_lateral_decider_output.path_bounds;
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
  }

  static const double min_v_cruise = 0.5;
  planning_input_.set_ref_vel(
      std::max(general_lateral_decider_output.v_cruise, min_v_cruise));

  planning_input_.set_curv_factor(config_.curv_factor);

  // set weights
  bool bend_scene = false;
  const double ego_s = reference_path_ptr->get_frenet_ego_state().s();
  const double preview_length = config_.curvature_preview_length;
  const double preview_step = config_.curvature_preview_step;
  double aver_close_kappa = 0.0;
  double aver_far_kappa = 0.0;
  ReferencePathPoint close_ref_path_point;
  ReferencePathPoint far_ref_path_point;
  for (double preview_distance = ego_s; preview_distance < preview_length; preview_distance += preview_step) {
    reference_path_ptr->get_reference_point_by_lon((preview_distance), close_ref_path_point);
    aver_close_kappa += close_ref_path_point.path_point.kappa;
    reference_path_ptr->get_reference_point_by_lon((preview_distance + config_.curvature_preview_distance), far_ref_path_point);
    aver_far_kappa += far_ref_path_point.path_point.kappa;
  }
  if ((std::fabs(preview_length) > 1e-6) &&(std::fabs(preview_step) > 1e-6)) {
    aver_close_kappa /= (preview_length / preview_step);
    aver_far_kappa /= (preview_length / preview_step);
    if (((1.0 / fabs(aver_close_kappa)) < config_.road_curvature_radius) || ((1.0 / fabs(aver_far_kappa)) < config_.road_curvature_radius)) {
      bend_scene = true;
    }
  }

  Point2D cart_ref0(planning_input_.ref_x_vec(0), planning_input_.ref_y_vec(0));
  Point2D frenet_ref0;
  Point2D cart_init(planning_input_.init_state().x(),
                    planning_input_.init_state().y());
  Point2D frenet_init;
  if (reference_path_ptr->get_frenet_coord() != nullptr &&
      reference_path_ptr->get_frenet_coord()->XYToSL(cart_ref0, frenet_ref0) &&
      reference_path_ptr->get_frenet_coord()->XYToSL(cart_init, frenet_init)) {
    planning_weight_ptr_->SetInitDisToRef((frenet_init.y - frenet_ref0.y));
    planning_weight_ptr_->SetInitRefThetaError((planning_input_.init_state().theta() - planning_input_.ref_theta_vec(0)) * 57.3);
  } else {
    planning_weight_ptr_->CalculateInitInfo(planning_input_);
  }
  planning_weight_ptr_->SetEgoVel(session_->environmental_model().get_ego_state_manager()->ego_v());
  planning_weight_ptr_->SetEgoL(reference_path_ptr->get_frenet_ego_state().l());
  planning_weight_ptr_->SetLCBackFlag(false);

  const LateralOffsetDeciderOutput &lateral_offset_decider_output =
      session_->mutable_planning_context()->lateral_offset_decider_output();
  if (lane_change_scene) {
    const auto target_state = session_->planning_context().lane_change_decider_output().coarse_planning_info.target_state;
    if (target_state == ROAD_LC_LBACK || target_state == ROAD_LC_RBACK) {
      planning_weight_ptr_->SetLCBackFlag(true);
    }
    planning_weight_ptr_->SetLateralMotionWeight(
        pnc::lateral_planning::LANE_CHANGE, planning_input_);
  } else if (lateral_offset_decider_output.is_valid) {
    planning_weight_ptr_->SetLateralMotionWeight(pnc::lateral_planning::AVOID,
                                                 planning_input_);
  } else if (bend_scene) {
    planning_weight_ptr_->SetLateralMotionWeight(pnc::lateral_planning::BEND,
                                                 planning_input_);
  } else {
    planning_weight_ptr_->SetLateralMotionWeight(
        pnc::lateral_planning::LANE_KEEP, planning_input_);
  }

  // set complete hold flag, concerned index
  planning_input_.set_complete_follow(complete_follow);
  planning_input_.set_motion_plan_concerned_index(
      config_.motion_plan_concerned_end_index);
}

void LateralMotionPlanner::Update() {
  const double concerned_start_q_jerk =  planning_weight_ptr_->GetConcernedStartQJerk();
  JSON_DEBUG_VALUE("concerned_start_q_jerk", concerned_start_q_jerk);
  const double ego_vel = std::max(session_->environmental_model().get_ego_state_manager()->ego_v(), config_.min_ego_vel);
  auto start_time = IflyTime::Now_ms();
  auto solver_condition = planning_problem_ptr_->Update(
      config_.motion_plan_concerned_start_index,
      concerned_start_q_jerk, ego_vel, planning_input_);
  JSON_DEBUG_VALUE("solver_condition", solver_condition);
  auto end_time = IflyTime::Now_ms();
  JSON_DEBUG_VALUE("iLqr_lat_update_time", end_time - start_time);

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
      t += 0.2;
    }
    s_vec[i + 1] = s;
    t_vec[i + 1] = t;
  }

  // generate motion planning output into planning_context
  auto &motion_planner_output =
      session_->mutable_planning_context()->mutable_motion_planner_output();

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
  t_vec[0] = -0.2;

  // set state spline
  motion_planner_output.x_s_spline.set_points(s_vec, x_vec);
  motion_planner_output.y_s_spline.set_points(s_vec, y_vec);
  motion_planner_output.theta_s_spline.set_points(s_vec, theta_vec);
  motion_planner_output.delta_s_spline.set_points(s_vec, delta_vec);
  motion_planner_output.omega_s_spline.set_points(s_vec, omega_vec);
  motion_planner_output.curv_s_spline.set_points(s_vec, curv_vec);
  motion_planner_output.d_curv_s_spline.set_points(s_vec, d_curv_vec);
  motion_planner_output.lateral_x_t_spline.set_points(t_vec, x_vec);
  motion_planner_output.lateral_y_t_spline.set_points(t_vec, y_vec);
  motion_planner_output.lateral_theta_t_spline.set_points(t_vec, theta_vec);
  motion_planner_output.s_lat_vec = s_vec;
  motion_planner_output.lat_init_flag = true;

  ilqr_solver::ControlVec u_vec;
  u_vec.resize(N);

  // set u_vec to motion_planner_output for warm start
  for (size_t i = 0; i < N; ++i) {
    ilqr_solver::Control u;
    u.resize(1);
    u[0] = omega_vec[i];
    u_vec[i] = u;
  }
  motion_planner_output.u_vec = u_vec;

  // assemble results
  const auto &general_lateral_decider_output =  // result from lat decision
      session_->planning_context().general_lateral_decider_output();

  auto &traj_points = session_->mutable_planning_context()
                          ->mutable_planning_result()
                          .traj_points;
  const auto &reference_path_ptr = session_->planning_context()
                                       .lane_change_decider_output()
                                       .coarse_planning_info.reference_path;
  for (size_t i = 0; i < N; i++) {
    traj_points[i].x = x_vec[i + 1];
    traj_points[i].y = y_vec[i + 1];
    traj_points[i].heading_angle = theta_vec[i + 1];
    traj_points[i].curvature = planning_input_.curv_factor() * delta_vec[i + 1];

    traj_points[i].v = general_lateral_decider_output.v_cruise;
    traj_points[i].t = planning_output.time_vec(i);

    // frenet state update
    Point2D cart_pt(traj_points[i].x, traj_points[i].y);
    Point2D frenet_pt;

    if (reference_path_ptr->get_frenet_coord() != nullptr &&
        reference_path_ptr->get_frenet_coord()->XYToSL(cart_pt, frenet_pt)) {
      traj_points[i].s = frenet_pt.x;
      traj_points[i].l = frenet_pt.y;
    } else {
      LOG_DEBUG(
          "XYToSL = FAILED !!!!!!!! index: %ld,  point.s : "
          "%f, point.l: %f ",
          i, traj_points[i].s, traj_points[i].l);
    }
  }
}

}  // namespace planning