
#include "pwj_longitudinal_motion_planner.h"

#include <iomanip>
#include <sstream>
#include <vector>

#include "environmental_model.h"
#include "ifly_time.h"

// #include "core/common/trace.h"
// #include "core/modules/common/ego_prediction_utils.h"
// #include "core/modules/context/ego_state.h"
#include "debug_info_log.h"
#include "ifly_time.h"
#include "math/piecewise_jerk/piecewise_problem.h"
#include "mrc_condition.h"

namespace planning {

LongitudinalOptimizerV3::LongitudinalOptimizerV3(
    const EgoPlanningConfigBuilder *config_builder,
    const std::shared_ptr<TaskPipelineContext> &pipeline_context)
    : Task(config_builder, pipeline_context) {
  config_ = config_builder->cast<LongitudinalOptimizerV3Config>();
  name_ = "PWJLonMotionPlanner";
}

bool LongitudinalOptimizerV3::Execute(planning::framework::Frame *frame) {
  // NTRACE_CALL(7);
  LOG_DEBUG("=======LongitudinalOptimizerV3======= \n");
  double start_time = IflyTime::Now_ms();
  if (Task::Execute(frame) == false) {
    return false;
  }
  auto config_builder =
      frame->mutable_session()->mutable_environmental_model()->config_builder(
          planning::common::SceneType::HIGHWAY);
  config_acc_ = config_builder->cast<AdaptiveCruiseControlConfig>();
  config_start_stop_ = config_builder->cast<StartStopEnableConfig>();

  auto &ego_prediction_result = pipeline_context_->planning_result;
  auto &ego_prediction_info = pipeline_context_->planning_info;
  bool b_success = false;
  LongitudinalSolverOption solver_option;
  solver_option.enable_log = false;
  solver_option.loose_obstacle_bound = false;
  solver_option.use_raw_model_traj = false;

  // MDEBUG_JSON_BEGIN_DICT(LongitudinalOptimizerProblem)
  // MDEBUG_JSON_BEGIN_DICT(LongitudinalOptimizerProblem_Full_Constraint)
  b_success = optimize(solver_option, ego_prediction_info.lon_ref_path,
                       ego_prediction_result.traj_points);
  // MDEBUG_JSON_END_DICT(LongitudinalOptimizerProblem_Full_Constraint)
  if (!b_success) {
    LOG_ERROR("LongitudinalOptimizerV3::Execute failed");
  }

  // 暂时关闭：失败后是否开启重新优化
  // if (config_.enable_longitudinal_optimization_backup) {
  //   if (!b_success) {
  //     solver_option.loose_obstacle_bound = true;
  //     //
  //     MDEBUG_JSON_BEGIN_DICT(LongitudinalOptimizerProblem_Only_Map_Constraint)
  //     b_success = optimize(solver_option, ego_prediction_info.lon_ref_path,
  //                          ego_prediction_result.traj_points);
  //     //
  //     MDEBUG_JSON_END_DICT(LongitudinalOptimizerProblem_Only_Map_Constraint)
  //     if (!b_success) {
  //       LOG_ERROR(
  //           "LongitudinalOptimizerV3::Execute failed with only map
  //           constraint");
  //     }
  //   }

  //   if (!b_success) {
  //     solver_option.enable_log = true;
  //     solver_option.use_raw_model_traj = true;
  //     //
  //     MDEBUG_JSON_BEGIN_DICT(LongitudinalOptimizerProblem_Without_Constraint)
  //     b_success = optimize(solver_option, ego_prediction_info.lon_ref_path,
  //                          ego_prediction_result.traj_points);
  //     //
  //     MDEBUG_JSON_END_DICT(LongitudinalOptimizerProblem_Without_Constraint)
  //     if (!b_success) {
  //       LOG_ERROR(
  //           "LongitudinalOptimizerV3::Execute failed without constraint");
  //     }
  //   }
  // }
  // MDEBUG_JSON_END_DICT(LongitudinalOptimizerProblem)
  double end_time = IflyTime::Now_ms();
  LOG_DEBUG("=======LongitudinalOptimizerV3 time cost is [%f]ms:\n",
            end_time - start_time);

  return b_success;
}

bool LongitudinalOptimizerV3::optimize(
    const LongitudinalSolverOption &option, const LonRefPath &lon_ref_path,
    std::vector<TrajectoryPoint> &traj_points) {
  // Step 1) define optimization problem
  const auto &planning_init_point =
      reference_path_ptr_->get_frenet_ego_state().planning_init_point();
  auto ego_vel = planning_init_point.v;
  auto ego_acc = planning_init_point.a;

  auto &acc_info = frame_->mutable_session()
                       ->mutable_planning_context()
                       ->mutable_adaptive_cruise_control_result();
  bool enable_dx_ref = acc_info.navi_speed_control_info.enable_v_cost;
  double acc_weight_dx_config = config_acc_.dx_ref_weight;
  double stop_weight_dx_config = config_start_stop_.dx_ref_weight;
  LOG_DEBUG("HHLDEBUGB in optimize ego_vel: %.2f, ego_acc: %.2f \n",
            ego_vel * 3.6, ego_acc);
  common::StartStopInfo &start_stop_result = frame_->mutable_session()
                                                 ->mutable_planning_context()
                                                 ->mutable_start_stop_result();
  bool enable_stop_flag = start_stop_result.enable_stop();
  std::array<double, 3> s_init_state = {planning_init_point.frenet_state.s,
                                        ego_vel, ego_acc};  // todo recheck
  size_t num_t = lon_ref_path.t_list.size();
  planning_math::PiecewiseProblem lon_problem(num_t, lon_ref_path.t_list,
                                              s_init_state, false);
  auto mrc_brake =
      frame_->mutable_session()->mutable_planning_context()->mrc_condition();
  MrcBrakeType mrc_brake_type = mrc_brake->mrc_brake_type();
  bool mrc_condition_enable = mrc_brake_type == MrcBrakeType::SLOW_BRAKE ||
                              mrc_brake_type == MrcBrakeType::HARD_BRAKE ||
                              mrc_brake_type == MrcBrakeType::EMERGENCY_BRAKE;
  if (enable_stop_flag) {
    lon_problem.set_weight_dx(stop_weight_dx_config);
    lon_problem.set_weight_x(0.0);
  } else {
    if (mrc_condition_enable) {
      lon_problem.set_weight_dx(acc_weight_dx_config);
      lon_problem.set_weight_x(0.0);
    } else if (enable_dx_ref) {
      lon_problem.set_weight_dx(acc_weight_dx_config);
      lon_problem.set_weight_x(0.0);
    } else {
      lon_problem.set_weight_dx(0.0);
      lon_problem.set_weight_x(1.0);
    }
  }
  LOG_DEBUG(
      "enable_stop_flag: %d, enable_dx_ref: %d, acc_weight_dx_config: %.2f, "
      "stop_weight_dx_config: %.2f \n",
      enable_stop_flag, enable_dx_ref, acc_weight_dx_config,
      stop_weight_dx_config);
  lon_problem.set_weight_ddx(1.0);
  lon_problem.set_weight_dddx(100.0);
  lon_problem.set_weight_slack(1.0);

  std::vector<WeightedBounds> bounds;
  bounds.resize(lon_ref_path.hard_bounds.size());
  auto lon_bound_v = lon_ref_path.lon_bound_v;
  auto lon_bound_a = lon_ref_path.lon_bound_a;
  auto lon_bound_jerk = lon_ref_path.lon_bound_jerk;
  auto lon_lead_bounds = lon_ref_path.lon_lead_bounds;
  if (option.use_raw_model_traj) {
    bounds.clear();
    lon_bound_v.clear();
    lon_bound_a.clear();
    lon_lead_bounds.clear();
    lon_bound_jerk.clear();
  } else if (option.loose_obstacle_bound) {
    for (size_t i = 0; i < lon_ref_path.hard_bounds.size(); i++) {
      auto &ref_pos_bounds = bounds[i];
      for (auto iter = lon_ref_path.hard_bounds[i].begin();
           iter != lon_ref_path.hard_bounds[i].end(); iter++) {
        if (iter->bound_info.type != "obstacle") {
          ref_pos_bounds.emplace_back(*iter);
        }
      }
    }
  } else {
    bounds = lon_ref_path.hard_bounds;
  }

  lon_problem.set_x_ref(lon_ref_path.s_refs);
  lon_problem.set_x_bounds(bounds);
  lon_problem.set_dx_bounds(lon_bound_v);
  lon_problem.set_ddx_bounds(lon_bound_a);
  lon_problem.set_x_infer_bounds(lon_lead_bounds);
  lon_problem.set_dx_ref(lon_ref_path.ds_refs);
  lon_problem.set_dddx_bounds(lon_bound_jerk);

  // Step 2) optimize problem
  int status = 0;
  bool success = lon_problem.optimize(config_.max_iteration_num, status);
  pipeline_context_->planning_result.extra_json["lon_motion_error_info"] =
      "none";
  if (!success) {
    if (option.enable_log) {
      pipeline_context_->planning_result.extra_json["lon_motion_error_info"] =
          "solver_failed";
    }
  }

  auto s = lon_problem.x();
  auto v = lon_problem.dx();
  auto a = lon_problem.ddx();
  auto jerk = lon_problem.dddx();

  // // Step 3) debug result
  // MDEBUG_JSON_ADD_ITEM(status, status, LongitudinalOptimizeProblem)
  // MDEBUG_JSON_ADD_ITEM(success, success, LongitudinalOptimizeProblem)

  // MDEBUG_JSON_BEGIN_DICT(init_state)
  // MDEBUG_JSON_ADD_ITEM(s, s_init_state[0], init_state)
  // MDEBUG_JSON_ADD_ITEM(v, s_init_state[1], init_state)
  // MDEBUG_JSON_ADD_ITEM(a, s_init_state[2], init_state)
  // MDEBUG_JSON_END_DICT(init_state)

  JSON_DEBUG_VALUE("LonMotionOpt_status", status);
  JSON_DEBUG_VALUE("LonMotionOpt_success", success);
  JSON_DEBUG_VALUE("LonMotionOpt_init_state_s", s_init_state[0]);
  JSON_DEBUG_VALUE("LonMotionOpt_init_state_v", s_init_state[1]);
  JSON_DEBUG_VALUE("LonMotionOpt_init_state_a", s_init_state[2]);

  const auto &ego_prediction_raw_traj_points =
      pipeline_context_->planning_result.raw_traj_points;
  auto init_s = planning_init_point.frenet_state.s;
  constexpr size_t kMaxCheckIndex = 5;
  bool check_invalid_speed_bound{false};
  double s_last = init_s;
  double v_ok = 0.0;
  for (size_t i = 0; i < num_t; ++i) {
    auto t = lon_ref_path.t_list[i];
    auto s_ref = lon_ref_path.s_refs[i].first;
    v_ok = (s_ref - s_last) / 0.2;
    // LOG_DEBUG("v_ref from long behavior planner is: [%.2f] m/s \n", v_ok);
    s_last = s_ref;
    auto s_ref_raw = i < ego_prediction_raw_traj_points.size()
                         ? ego_prediction_raw_traj_points[i].s
                         : 0.0;
    auto s_lower = std::numeric_limits<double>::min();
    auto s_upper = std::numeric_limits<double>::max();
    BoundInfo s_lower_bound_info;
    BoundInfo s_upper_bound_info;
    if (i < bounds.size()) {
      for (auto &bound : bounds[i]) {
        if (bound.lower > s_lower) {
          s_lower = bound.lower;
          s_lower_bound_info = bound.bound_info;
        }
        if (bound.upper < s_upper) {
          s_upper = bound.upper;
          s_upper_bound_info = bound.bound_info;
        }
      }
    }
    if (s_lower > s_upper) {
      LOG_ERROR("NP_DEBUG: Error! Invalid s bound! \n");
      check_invalid_speed_bound = true;
      if (option.enable_log) {
        pipeline_context_->planning_result.extra_json["lon_motion_error_info"] =
            "invalid_s_bound";
      }
    }
    auto s_lead_bound = std::numeric_limits<double>::max();
    if (i < lon_lead_bounds.size()) {
      for (auto &bound : lon_lead_bounds[i]) {
        s_lead_bound =
            std::fmin(s_lead_bound, bound.s_lead + bound.v_lead * bound.t_lead);
      }
      s_lead_bound -= init_s;
      if (s_lead_bound > 1000) {
        s_lead_bound = -1.0;
      }
    }

    constexpr double kMaxAcc = 4.0;
    constexpr double kMinDec = -7.0;
    constexpr double kMaxJerk = 10.0;

    bool after_hot_start = pipeline_context_->status_info.planning_loop > 50;
    size_t max_check_index = std::min(kMaxCheckIndex, s.size());
    if (!check_invalid_speed_bound && after_hot_start && i < max_check_index) {
      if (s[i] < s_lower - 2.0 || s[i] > s_upper + 2.0) {
        LOG_ERROR("NP_DEBUG: Error! speed bound collide! %d %f [%f %f] \n", i,
                  s[i], s_lower, s_upper);
        if (option.enable_log) {
          pipeline_context_->planning_result
              .extra_json["lon_motion_error_info"] = "speed_bound_collide";
        }
      }
      if (a[i] > kMaxAcc || a[i] < kMinDec) {
        LOG_ERROR("NP_DEBUG: Error! Invalid acc %f ! \n", a[i]);
        if (option.enable_log) {
          pipeline_context_->planning_result
              .extra_json["lon_motion_error_info"] =
              "speed_dynamic_check_failed_for_invalid_acc";
        }
      }
      if (std::abs(jerk[i]) > kMaxJerk) {
        LOG_ERROR("NP_DEBUG: Error! Invalid jerk %f! \n", jerk[i]);
        if (option.enable_log) {
          pipeline_context_->planning_result
              .extra_json["lon_motion_error_info"] =
              "speed_dynamic_check_failed_for_invalid_jerk";
        }
      }
    }
  }

  // step 4) check result
  if (not success) {
    std::stringstream error_info;
    error_info << "lon_pwj_opt_fail:" << status;
    pipeline_context_->status_info.error_info = error_info.str();
    LOG_ERROR("LongitudinalOptimizerV3::optimize failed \n");
    return false;
  }

  if (success && s.size() >= num_t && v.size() >= num_t && a.size() >= num_t) {
    for (size_t i = 0; i < num_t; i++) {
      if (std::isnan(s[i]) || std::isnan(v[i]) || std::isnan(a[i])) {
        pipeline_context_->status_info.error_info = "invalid_lonopt_solution";
        LOG_ERROR(
            "LongitudinalOptimizerV3::optimize invalid_lonopt_solution \n");
        return false;
      } else if (v[i] < -1e-8 || (i > 0 && s[i] < s[i - 1])) {
        s[i] = i > 0 ? s[i - 1] - v[i - 1] * v[i - 1] / 2.0 / config_.acc_stop
                     : s[0];
        v[i] = 0.0;
        a[i] = 0.0;
      }
    }
  }

  // Step 5) interpolate l
  std::vector<double> l;
  std::vector<double> heading_angle;
  std::vector<double> curvature;

  interpolate_frenet_lon(traj_points, s, l, heading_angle, curvature);

  for (size_t i = 0; i < traj_points.size(); i++) {
    traj_points[i].s = s[i];
    traj_points[i].v = v[i];
    traj_points[i].a = a[i];
    traj_points[i].l = l[i];
    traj_points[i].heading_angle = heading_angle[i];
    traj_points[i].curvature = curvature[i];
  }

  return true;
}

void LongitudinalOptimizerV3::interpolate_frenet_lon(
    const std::vector<TrajectoryPoint> &traj_points,
    const std::vector<double> &s, std::vector<double> &l,
    std::vector<double> &heading_angle, std::vector<double> &curvature) {
  l.resize(traj_points.size());
  heading_angle.resize(traj_points.size());
  curvature.resize(traj_points.size());
  l[0] = traj_points.front().l;
  heading_angle[0] = traj_points.front().heading_angle;
  curvature[0] = traj_points.front().curvature;

  size_t j = 0;
  for (size_t i = 1; i < s.size(); i++) {
    // find j
    if (s[i] >= traj_points.back().s) {
      l[i] = traj_points.back().l;
      heading_angle[i] = traj_points.back().heading_angle;
      curvature[i] = traj_points.back().curvature;
      continue;
    }
    if (s[i] <= traj_points.front().s) {
      l[i] = traj_points.front().l;
      heading_angle[i] = traj_points.front().heading_angle;
      curvature[i] = traj_points.front().curvature;
      continue;
    }
    bool found_left_right = false;
    while (j + 1 < traj_points.size()) {
      if (traj_points[j].s <= s[i] and s[i] < traj_points[j + 1].s) {
        found_left_right = true;
        break;
      }
      ++j;
    }

    // compute l_i
    if (not found_left_right) {
      l[i] = traj_points.back().l;
      heading_angle[i] = traj_points.back().heading_angle;
      curvature[i] = traj_points.back().curvature;
    } else {
      auto ratio = 1.0;
      if ((traj_points[j + 1].s - traj_points[j].s) > 1e-2) {
        ratio = (traj_points[j + 1].s - s[i]) /
                (traj_points[j + 1].s - traj_points[j].s);
      }

      l[i] = planning_math::Interpolate(traj_points[j].l, traj_points[j + 1].l,
                                        ratio);
      heading_angle[i] = planning_math::InterpolateAngle(
          traj_points[j].heading_angle, traj_points[j + 1].heading_angle,
          ratio);
      curvature[i] = planning_math::Interpolate(
          traj_points[j].curvature, traj_points[j + 1].curvature, ratio);
    }
  }
}

}  // namespace planning
