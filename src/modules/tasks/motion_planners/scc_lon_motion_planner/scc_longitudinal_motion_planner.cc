#include "scc_longitudinal_motion_planner.h"

#include <algorithm>
#include <cmath>
#include <cstddef>

#include "debug_info_log.h"
#include "math_lib.h"

namespace planning {
SccLongitudinalMotionPlanner::SccLongitudinalMotionPlanner(
    const EgoPlanningConfigBuilder *config_builder,
    const std::shared_ptr<TaskPipelineContext> &pipeline_context)
    : Task(config_builder, pipeline_context) {
  config_ = config_builder->cast<SccLonMotionPlannerConfig>();
  config_start_stop_ = config_builder->cast<StartStopEnableConfig>();
  config_acc_ = config_builder->cast<AdaptiveCruiseControlConfig>();
  name_ = "LongitudinalMotionPlanner";

  Init();
};

void SccLongitudinalMotionPlanner::Init() {
  // init planning problem
  planning_problem_ptr_ = std::make_shared<
      pnc::scc_longitudinal_planning::SccLongitudinalMotionPlanningProblem>();
  planning_problem_ptr_->Init();

  // init planning input and output
  auto const N =
      planning_problem_ptr_->GetiLqrCorePtr()->GetSolverConfigPtr()->horizon +
      1;

  // init planning input
  planning_input_.mutable_ref_pos_vec()->Resize(N, 0.0);
  planning_input_.mutable_ref_vel_vec()->Resize(N, 0.0);

  planning_input_.mutable_soft_pos_max_vec()->Resize(N, 0.0);
  planning_input_.mutable_soft_pos_min_vec()->Resize(N, 0.0);
  planning_input_.mutable_hard_pos_max_vec()->Resize(N, 0.0);
  planning_input_.mutable_hard_pos_min_vec()->Resize(N, 0.0);
  // init sv bounds
  planning_input_.mutable_sv_bound_s_0()->Resize(N, 0.0);
  planning_input_.mutable_sv_bound_s_1()->Resize(N, 0.0);
  planning_input_.mutable_sv_bound_s_2()->Resize(N, 0.0);
  planning_input_.mutable_sv_bound_s_3()->Resize(N, 0.0);
  planning_input_.mutable_sv_bound_s_4()->Resize(N, 0.0);
  planning_input_.mutable_sv_bound_s_5()->Resize(N, 0.0);
  planning_input_.mutable_sv_bound_v_0()->Resize(N, 0.0);
  planning_input_.mutable_sv_bound_v_1()->Resize(N, 0.0);
  planning_input_.mutable_sv_bound_v_2()->Resize(N, 0.0);
  planning_input_.mutable_sv_bound_v_3()->Resize(N, 0.0);
  planning_input_.mutable_sv_bound_v_4()->Resize(N, 0.0);
  planning_input_.mutable_sv_bound_v_5()->Resize(N, 0.0);

  planning_input_.mutable_vel_max_vec()->Resize(N, 0.0);
  planning_input_.mutable_vel_min_vec()->Resize(N, 0.0);
  planning_input_.mutable_acc_max_vec()->Resize(N, 0.0);
  planning_input_.mutable_acc_min_vec()->Resize(N, 0.0);
  planning_input_.mutable_jerk_max_vec()->Resize(N, 0.0);
  planning_input_.mutable_jerk_min_vec()->Resize(N, 0.0);
}

bool SccLongitudinalMotionPlanner::Execute(planning::framework::Frame *frame) {
  LOG_DEBUG("=======SccLonMotionPlanner======= \n");
  auto start_time = IflyTime::Now_ms();
  frame_ = frame;
  if (Task::Execute(frame) == false) {
    return false;
  }

  AssembleInput();
  auto input_time = IflyTime::Now_ms();
  JSON_DEBUG_VALUE("SccLonMotionInputCostTime", input_time - start_time);
  LOG_DEBUG("AssembleInput time:%f\n", input_time - start_time);

  Update();
  auto calculate_time = IflyTime::Now_ms();
  JSON_DEBUG_VALUE("SccLonMotionCalCostTime", calculate_time - start_time);
  LOG_DEBUG("update time:%f\n", calculate_time - start_time);

  // record input and output
  DebugInfoManager::GetInstance()
      .GetDebugInfoPb()
      ->mutable_longitudinal_motion_planning_input()
      ->CopyFrom(planning_input_);

  DebugInfoManager::GetInstance()
      .GetDebugInfoPb()
      ->mutable_longitudinal_motion_planning_output()
      ->CopyFrom(planning_problem_ptr_->GetOutput());

  auto end_time = IflyTime::Now_ms();
  JSON_DEBUG_VALUE("SccLonMotionCostTime", end_time - start_time);

  return true;
}

void SccLongitudinalMotionPlanner::AssembleInput() {
  const auto &lon_ref_path =  // result from lon decision
      pipeline_context_->planning_info.lon_ref_path;

  const auto &s_refs = lon_ref_path.s_refs;
  const auto &v_refs = lon_ref_path.ds_refs;
  const auto &s_bounds = lon_ref_path.hard_bounds;
  const auto &s_soft_bounds = lon_ref_path.soft_bounds;
  // const auto &s_lead_bounds = lon_ref_path.lon_lead_bounds;
  const auto &sv_bounds = lon_ref_path.lon_sv_boundary.sv_bounds;
  const auto &v_bounds = lon_ref_path.lon_bound_v;
  const auto &a_bounds = lon_ref_path.lon_bound_a;
  const auto &jerk_bounds = lon_ref_path.lon_bound_jerk;

  // 1. set ref_pos and ref_vel
  for (size_t i = 0; i < s_refs.size(); ++i) {
    auto const &dt =
        planning_problem_ptr_->GetiLqrCorePtr()->GetSolverConfigPtr()->model_dt;

    auto const &v_ref = v_refs[i].first;
    auto s_ref = s_refs[i].first;

    // note that s_ref should be limited by v_ref
    if (i > 0) {
      auto const &last_s_ref = planning_input_.ref_pos_vec(i - 1);
      s_ref = pnc::mathlib::Clamp(s_ref, s_ref, last_s_ref + dt * v_ref);
    }

    planning_input_.mutable_ref_pos_vec()->Set(i, s_ref);
    planning_input_.mutable_ref_vel_vec()->Set(i, v_ref);
  }

  // 2. set bounds
  // 2.1. set hard bound: s bounds
  for (size_t i = 0; i < s_bounds.size(); ++i) {
    Bound tmp_bound{-1.0e4, 1.0e4};
    for (auto &bound : s_bounds[i]) {
      tmp_bound.lower = std::max(bound.lower, tmp_bound.lower);
      tmp_bound.upper = std::min(bound.upper, tmp_bound.upper);
    }

    s_limit_.lower =
        tmp_bound.lower > s_limit_.lower ? tmp_bound.lower : s_limit_.lower;
    s_limit_.upper =
        tmp_bound.upper < s_limit_.upper ? tmp_bound.upper : s_limit_.upper;

    planning_input_.mutable_hard_pos_max_vec()->Set(i, tmp_bound.upper);
    planning_input_.mutable_hard_pos_min_vec()->Set(i, tmp_bound.lower);
  }

  // 2.2. set soft bound: s_soft_bounds
  // attention: s_lead_bounds size < s_bounds size
  for (size_t i = 0; i < s_soft_bounds.size(); i++) {
    Bound tmp_bound{-1.0e4, 1.0e4};
    Bound tmp_soft_bound{-1.0e4, 1.0e4};
    Bound cal_bound{-1.0e4, 1.0e4};
    // get minimum s lead bound
    for (auto &soft_bound : s_soft_bounds[i]) {
      tmp_soft_bound.lower = std::max(soft_bound.lower, tmp_soft_bound.lower);
      tmp_soft_bound.upper = std::min(soft_bound.upper, tmp_soft_bound.upper);
    }
    // get minimum s bound
    for (auto &bound : s_bounds[i]) {
      tmp_bound.lower = std::max(bound.lower, tmp_bound.lower);
      tmp_bound.upper = std::min(bound.upper, tmp_bound.upper);
    }
    // get final bound
    cal_bound.lower = std::max(tmp_soft_bound.lower, tmp_bound.lower);
    cal_bound.upper = std::min(tmp_soft_bound.upper, tmp_bound.upper);

    s_limit_.lower =
        cal_bound.lower > s_limit_.lower ? cal_bound.lower : s_limit_.lower;
    s_limit_.upper =
        cal_bound.upper < s_limit_.upper ? cal_bound.upper : s_limit_.upper;

    planning_input_.mutable_soft_pos_max_vec()->Set(i, cal_bound.upper);
    planning_input_.mutable_soft_pos_min_vec()->Set(i, cal_bound.lower);
  }

  // 2.3. set s-v bounds 离散点降采样
  const int sample_step = sv_bounds.size() / 5;
  for (size_t i = 0; i < sv_bounds.size(); ++i) {
    planning_input_.mutable_sv_bound_s_0()->Set(i, sv_bounds[0].s);
    planning_input_.mutable_sv_bound_s_1()->Set(i,
                                                sv_bounds[1 * sample_step].s);
    planning_input_.mutable_sv_bound_s_2()->Set(i,
                                                sv_bounds[2 * sample_step].s);
    planning_input_.mutable_sv_bound_s_3()->Set(i,
                                                sv_bounds[3 * sample_step].s);
    planning_input_.mutable_sv_bound_s_4()->Set(i,
                                                sv_bounds[4 * sample_step].s);
    planning_input_.mutable_sv_bound_s_5()->Set(i,
                                                sv_bounds[5 * sample_step].s);

    planning_input_.mutable_sv_bound_v_0()->Set(i, sv_bounds[0].v_bound.upper);
    planning_input_.mutable_sv_bound_v_1()->Set(
        i, sv_bounds[1 * sample_step].v_bound.upper);
    planning_input_.mutable_sv_bound_v_2()->Set(
        i, sv_bounds[2 * sample_step].v_bound.upper);
    planning_input_.mutable_sv_bound_v_3()->Set(
        i, sv_bounds[3 * sample_step].v_bound.upper);
    planning_input_.mutable_sv_bound_v_4()->Set(
        i, sv_bounds[4 * sample_step].v_bound.upper);
    planning_input_.mutable_sv_bound_v_5()->Set(
        i, sv_bounds[5 * sample_step].v_bound.upper);
  }

  // 2.4. set vel bounds
  for (size_t i = 0; i < v_bounds.size(); ++i) {
    planning_input_.mutable_vel_max_vec()->Set(i, v_bounds[i].upper);
    planning_input_.mutable_vel_min_vec()->Set(i, v_bounds[i].lower);
  }

  // 2.5. set acc bounds
  for (size_t i = 0; i < a_bounds.size(); ++i) {
    planning_input_.mutable_acc_max_vec()->Set(i, a_bounds[i].upper);
    planning_input_.mutable_acc_min_vec()->Set(i, a_bounds[i].lower);
  }

  // 2.6. jerk bounds
  for (size_t i = 0; i < jerk_bounds.size(); ++i) {
    planning_input_.mutable_jerk_max_vec()->Set(i, jerk_bounds[i].upper);
    planning_input_.mutable_jerk_min_vec()->Set(i, jerk_bounds[i].lower);
  }

  // 3. set init state
  const auto &planning_init_point =
      reference_path_ptr_->get_frenet_ego_state().planning_init_point();

  // init s uses frenet state
  // planning_input_.mutable_init_state()->set_s(planning_init_point.frenet_state.s);
  // scc planner纵向使用相对关系，init s = 0.0
  planning_input_.mutable_init_state()->set_s(0.0);
  planning_input_.mutable_init_state()->set_v(
      planning_init_point.lon_init_state.v());
  planning_input_.mutable_init_state()->set_a(
      planning_init_point.lon_init_state.a());

  // 4. set weights
  planning_input_.set_q_ref_pos(config_.q_ref_pos);
  planning_input_.set_q_ref_vel(config_.q_ref_vel);
  planning_input_.set_q_acc(config_.q_acc);
  planning_input_.set_q_jerk(config_.q_jerk);
  planning_input_.set_q_stop_s(config_.q_stop_s);

  planning_input_.set_q_soft_pos_bound(config_.q_soft_pos_bound);
  planning_input_.set_q_hard_pos_bound(config_.q_hard_pos_bound);
  planning_input_.set_q_sv_bound(config_.q_sv_bound);
  planning_input_.set_q_vel_bound(config_.q_vel_bound);
  planning_input_.set_q_acc_bound(config_.q_acc_bound);
  planning_input_.set_q_jerk_bound(config_.q_jerk_bound);

  // what is s_stop?
  planning_input_.set_s_stop(1.0e4);  // TBD: hack for input;
}

void SccLongitudinalMotionPlanner::Update() {
  const auto &planning_init_point =
      reference_path_ptr_->get_frenet_ego_state().planning_init_point();

  // assembling planning output proto
  auto start_time = IflyTime::Now_ms();
  planning_problem_ptr_->Update(planning_input_);
  auto end_time = IflyTime::Now_ms();
  JSON_DEBUG_VALUE("iLqr_lon_update_time", end_time - start_time);

  const size_t N =
      planning_problem_ptr_->GetiLqrCorePtr()->GetSolverConfigPtr()->horizon +
      1;
  const auto &dt =
      planning_problem_ptr_->GetiLqrCorePtr()->GetSolverConfigPtr()->model_dt;

  auto start1_time = IflyTime::Now_ms();
  const auto &planning_output = planning_problem_ptr_->GetOutput();
  auto end1_time = IflyTime::Now_ms();
  JSON_DEBUG_VALUE("iLqr_lon_get_output_time", end1_time - start1_time);

  // state vector
  std::vector<double> s_vec(N);
  std::vector<double> v_vec(N);
  std::vector<double> a_vec(N);
  std::vector<double> j_vec(N);
  std::vector<double> t_vec(N);

  // assemble proto for pnc tools
  double t = 0.0;
  for (size_t i = 0; i < N; ++i) {
    t_vec[i] = t;

    t += dt;

    s_vec[i] = planning_output.pos_vec(i);
    v_vec[i] = planning_output.vel_vec(i);
    a_vec[i] = planning_output.acc_vec(i);
    j_vec[i] = planning_output.jerk_vec(i);
  }

  // generate motion planning output into planning_context
  auto &motion_planning_info = frame_->mutable_session()
                                   ->mutable_planning_context()
                                   ->mutable_planning_result()
                                   .motion_planning_info;

  // motion_planning_info.s_t_spline.set_points(t_vec, s_vec);
  motion_planning_info.v_t_spline.set_points(t_vec, v_vec);
  motion_planning_info.a_t_spline.set_points(t_vec, a_vec);
  motion_planning_info.j_t_spline.set_points(t_vec, j_vec);

  motion_planning_info.lon_enable_flag = true;
  // respline the lateral path a.c. longitudinal result
  std::vector<double> assembled_x(N);
  std::vector<double> assembled_y(N);
  std::vector<double> assembled_theta(N);
  std::vector<double> assembled_delta(N);
  std::vector<double> assembled_omega(N);

  std::vector<double> assembled_ref_x(N);
  std::vector<double> assembled_ref_y(N);

  // s postprocess
  for (size_t i = 1; i < s_vec.size(); ++i) {
    s_vec[i] = std::max(
        s_vec[i], s_vec[i - 1]);  // 1e-3 to avoid non-inremental spline input
  }

  // assemble trajectory that combines lateral and longitudinal planning_result
  auto &traj_points = pipeline_context_->planning_result.traj_points;
  const auto &s0 = s_vec[0];
  const double init_point_s = planning_init_point.frenet_state.s;
  for (size_t i = 0; i < N; ++i) {
    traj_points[i].v = v_vec[i];
    traj_points[i].a = a_vec[i];
    traj_points[i].t = t_vec[i];

    // lateral path resampling
    // s is lateral path rather than longitudinal path (frenet)
    // considering an offset that equals to init s
    // note that lateral s_spline starts from zero
    auto s = std::max(s_vec[i] - s0, 0.0);

    // limit s to avoid outer spline
    s = std::min(s, motion_planning_info.s_lat_vec.back());

    traj_points[i].x = motion_planning_info.x_s_spline(s);
    traj_points[i].y = motion_planning_info.y_s_spline(s);
    traj_points[i].heading_angle = motion_planning_info.theta_s_spline(s);
    // 相对自车的s需要还原成相对参考线起点的s
    traj_points[i].s = s + init_point_s;
    // frenet state is not considered

    // reassembling lateral path by long traj
    assembled_x[i] = traj_points[i].x;
    assembled_y[i] = traj_points[i].y;

    // assembled_ref_x[i] = motion_planning_info.ref_x_s_spline(s);
    // assembled_ref_y[i] = motion_planning_info.ref_y_s_spline(s);

    assembled_theta[i] = traj_points[i].heading_angle;
    assembled_delta[i] = motion_planning_info.delta_s_spline(s);
    assembled_omega[i] = motion_planning_info.omega_s_spline(s);
  }

  motion_planning_info.lat_enable_flag = true;

  motion_planning_info.x_t_spline.set_points(t_vec, assembled_x);
  motion_planning_info.y_t_spline.set_points(t_vec, assembled_y);
  motion_planning_info.theta_t_spline.set_points(t_vec, assembled_theta);
  motion_planning_info.delta_t_spline.set_points(t_vec, assembled_delta);
  motion_planning_info.omega_t_spline.set_points(t_vec, assembled_omega);

  // motion_planning_info.ref_x_t_spline.set_points(t_vec, assembled_ref_x);
  // motion_planning_info.ref_y_t_spline.set_points(t_vec, assembled_ref_y);
}
}  // namespace planning
