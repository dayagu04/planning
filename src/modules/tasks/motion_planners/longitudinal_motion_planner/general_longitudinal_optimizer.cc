
#include "tasks/motion_planners/longitudinal_motion_planner/general_longitudinal_optimizer.h"

#include <iomanip>
#include <sstream>
#include <vector>

// #include "core/common/trace.h"
// #include "core/modules/common/ego_prediction_utils.h"
// #include "core/modules/context/ego_state.h"
#include "common/math/piecewise_jerk/piecewise_problem.h"
#include "scc_function/mrc_condition.h"

namespace planning {

LongitudinalOptimizerV3::LongitudinalOptimizerV3(
    const EgoPlanningConfigBuilder *config_builder,
    const std::shared_ptr<TaskPipelineContext> &pipeline_context)
    : Task(config_builder, pipeline_context) {
  config_ = config_builder->cast<LongitudinalOptimizerV3Config>();
  name_ = "LongitudinalOptimizerV3";
}

bool LongitudinalOptimizerV3::Execute(planning::framework::Frame *frame) {
  // NTRACE_CALL(7);

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
  StartStopInfo &start_stop_result = frame_->mutable_session()
                                         ->mutable_planning_context()
                                         ->mutable_start_stop_result();
  bool enable_stop_flag = start_stop_result.enable_stop;
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
      "HHLDEBUGB enable_stop_flag: %d, enable_dx_ref: %d,"
      "acc_weight_dx_config: %.2f, stop_weight_dx_config: %.2f \n",
      enable_stop_flag, enable_dx_ref, acc_weight_dx_config,
      stop_weight_dx_config);
  lon_problem.set_weight_ddx(1.0);
  lon_problem.set_weight_dddx(100.0);
  lon_problem.set_weight_slack(1.0);

  std::vector<WeightedBounds> bounds;
  bounds.resize(lon_ref_path.bounds.size());
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
    for (size_t i = 0; i < lon_ref_path.bounds.size(); i++) {
      auto &ref_pos_bounds = bounds[i];
      for (auto iter = lon_ref_path.bounds[i].begin();
           iter != lon_ref_path.bounds[i].end(); iter++) {
        if (iter->bound_info.type != "obstacle") {
          ref_pos_bounds.emplace_back(*iter);
        }
      }
    }
  } else {
    bounds = lon_ref_path.bounds;
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

  // mdebug
  std::string mdebug_table_name =
      "LongitudinalOptimizeProblem (target_lane_id:" +
      std::to_string(pipeline_context_->coarse_planning_info.target_lane_id) +
      ", option.loose_obstacle_bound:" +
      std::to_string(option.loose_obstacle_bound) +
      ", option.use_raw_model_traj:" +
      std::to_string(option.use_raw_model_traj) +
      ", s:" + std::to_string(planning_init_point.frenet_state.s) +
      ", v:" + std::to_string(planning_init_point.v) +
      ", a:" + std::to_string(planning_init_point.a) +
      ", status:" + std::to_string(status) + ", success:";
  // mdebug_table_name += success ? "true" : "false";
  // mdebug_table_name += ")";

  // MDEBUG_TABLE_BEGIN(mdebug_table_name)
  // MDEBUG_TABLE_BEGIN_COLUMNS()
  // MDEBUG_TABLE_ADD_COLUMN("t", "number")
  // MDEBUG_TABLE_ADD_COLUMN("s_ref", "number")
  // MDEBUG_TABLE_ADD_COLUMN("s_ref_raw", "number")
  // MDEBUG_TABLE_ADD_COLUMN("s_lower", "string")
  // MDEBUG_TABLE_ADD_COLUMN("s_upper", "string")
  // MDEBUG_TABLE_ADD_COLUMN("v_lower", "number")
  // MDEBUG_TABLE_ADD_COLUMN("v_upper", "number")
  // MDEBUG_TABLE_ADD_COLUMN("a_lower", "number")
  // MDEBUG_TABLE_ADD_COLUMN("a_upper", "number")
  // MDEBUG_TABLE_ADD_COLUMN("v_ref", "number");
  // MDEBUG_TABLE_ADD_COLUMN("jerk_lower", "number");
  // MDEBUG_TABLE_ADD_COLUMN("jerk_upper", "number");
  // MDEBUG_TABLE_ADD_COLUMN("s_lead_bound", "number")
  // MDEBUG_TABLE_ADD_COLUMN("s", "number")
  // MDEBUG_TABLE_ADD_COLUMN("v", "number")
  // MDEBUG_TABLE_ADD_COLUMN("a", "number")
  // MDEBUG_TABLE_ADD_COLUMN("s_lead", "number")
  // MDEBUG_TABLE_ADD_COLUMN("jerk", "number")
  // MDEBUG_TABLE_END_COLUMNS()
  // MDEBUG_TABLE_BEGIN_ROWS()

  // MDEBUG_JSON_BEGIN_ARRAY(points)
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
    LOG_DEBUG("HHLDEBUG from model v: %.2f \n", v_ok * 3.6);
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

    //   MDEBUG_JSON_BEGIN_OBJECT(object)
    //   MDEBUG_JSON_ADD_ITEM(t, t, object)
    //   MDEBUG_JSON_ADD_ITEM(s_ref, s_ref - init_s, object)
    //   MDEBUG_JSON_ADD_ITEM(s_ref_raw, s_ref_raw - init_s, object)
    //   MDEBUG_JSON_ADD_ITEM(s_lower, s_lower - init_s, object)
    //   MDEBUG_JSON_ADD_ITEM(s_upper, s_upper - init_s, object)
    //   MDEBUG_JSON_ADD_ITEM(s_lower_bound_info_id, s_lower_bound_info.id,
    //   object) MDEBUG_JSON_ADD_ITEM(s_upper_bound_info_id,
    //   s_upper_bound_info.id, object)
    //   MDEBUG_JSON_ADD_ITEM(s_lower_bound_info_type, s_lower_bound_info.type,
    //                        object)
    //   MDEBUG_JSON_ADD_ITEM(s_upper_bound_info_type, s_upper_bound_info.type,
    //                        object)
    //   MDEBUG_JSON_ADD_ITEM(
    //       v_lower, i < lon_bound_v.size() ? lon_bound_v[i].lower : -100.0,
    //       object)
    //   MDEBUG_JSON_ADD_ITEM(
    //       v_upper, i < lon_bound_v.size() ? lon_bound_v[i].upper : 100.0,
    //       object)
    //   MDEBUG_JSON_ADD_ITEM(
    //       a_lower, i < lon_bound_a.size() ? lon_bound_a[i].lower : -10.0,
    //       object)
    //   MDEBUG_JSON_ADD_ITEM(
    //       a_upper, i < lon_bound_a.size() ? lon_bound_a[i].upper : 10.0,
    //       object)
    //   MDEBUG_JSON_ADD_ITEM(v_ref, lon_ref_path.ds_refs[i].first, object)
    //   MDEBUG_JSON_ADD_ITEM(
    //       jerk_lower, i < lon_bound_jerk.size() ? lon_bound_jerk[i].lower :
    //       -10.0, object)
    //   MDEBUG_JSON_ADD_ITEM(
    //       jerk_upper, i < lon_bound_jerk.size() ? lon_bound_jerk[i].upper
    //       : 10.0, object)
    //   MDEBUG_JSON_ADD_ITEM(s_lead_bound, s_lead_bound, object)

    //   MDEBUG_TABLE_BEGIN_ONE_ROW()
    //   MDEBUG_TABLE_ADD_RAW_DATA(t)
    //   MDEBUG_TABLE_ADD_RAW_DATA(s_ref - init_s)
    //   MDEBUG_TABLE_ADD_RAW_DATA(s_ref_raw - init_s)
    //   MDEBUG_TABLE_ADD_RAW_DATA(std::to_string(s_lower - init_s) + "(id:" +
    //                             std::to_string(s_lower_bound_info.id) +
    //                             ",type:" + s_lower_bound_info.type + ")")
    //   MDEBUG_TABLE_ADD_RAW_DATA(std::to_string(s_upper - init_s) + "(id:" +
    //                             std::to_string(s_upper_bound_info.id) +
    //                             ",type:" + s_upper_bound_info.type + ")")
    //   MDEBUG_TABLE_ADD_RAW_DATA(i < lon_bound_v.size() ? lon_bound_v[i].lower
    //                                                    : -100.0)
    //   MDEBUG_TABLE_ADD_RAW_DATA(i < lon_bound_v.size() ? lon_bound_v[i].upper
    //                                                    : 100.0)
    //   MDEBUG_TABLE_ADD_RAW_DATA(i < lon_bound_a.size() ? lon_bound_a[i].lower
    //                                                    : -10.0)
    //   MDEBUG_TABLE_ADD_RAW_DATA(i < lon_bound_a.size() ? lon_bound_a[i].upper
    //                                                    : 10.0)
    //   MDEBUG_TABLE_ADD_RAW_DATA(lon_ref_path.ds_refs[i].first)
    //   MDEBUG_TABLE_ADD_RAW_DATA(
    //       i < lon_bound_jerk.size() ? lon_bound_jerk[i].lower : -10.0);
    //   MDEBUG_TABLE_ADD_RAW_DATA(
    //       i < lon_bound_jerk.size() ? lon_bound_jerk[i].upper : 10.0);
    //   MDEBUG_TABLE_ADD_RAW_DATA(s_lead_bound)

    //   if (success) {
    //     MDEBUG_JSON_ADD_ITEM(s, s[i] - init_s, object)
    //     MDEBUG_JSON_ADD_ITEM(v, v[i], object)
    //     MDEBUG_JSON_ADD_ITEM(a, a[i], object)
    //     MDEBUG_JSON_ADD_ITEM(s_lead, s[i] - init_s + 2 * v[i], object)
    //     MDEBUG_JSON_ADD_ITEM(jerk, i < jerk.size() ? jerk[i] : 0.0, object)
    //     LOG_DEBUG("HHLDEBUG optimize s: %.2f, v: %.2f, a: %.2f, jerk: %.2f",
    //     s[i],
    //           v[i] * 3.6, a[i], jerk[i]);

    //     MDEBUG_TABLE_ADD_RAW_DATA(s[i] - init_s)
    //     MDEBUG_TABLE_ADD_RAW_DATA(v[i])
    //     MDEBUG_TABLE_ADD_RAW_DATA(a[i])
    //     MDEBUG_TABLE_ADD_RAW_DATA(s[i] - init_s + 2 * v[i])
    //     MDEBUG_TABLE_ADD_RAW_DATA(i < jerk.size() ? jerk[i] : 0.0)
    //   } else {
    //     MDEBUG_TABLE_ADD_RAW_DATA(0.0)
    //     MDEBUG_TABLE_ADD_RAW_DATA(0.0)
    //     MDEBUG_TABLE_ADD_RAW_DATA(0.0)
    //     MDEBUG_TABLE_ADD_RAW_DATA(0.0)
    //     MDEBUG_TABLE_ADD_RAW_DATA(0.0)
    //   }
    //   MDEBUG_JSON_END_OBJECT(object)
    //   MDEBUG_TABLE_END_ONE_ROW()
  }
  // MDEBUG_TABLE_END_ROWS()
  // MDEBUG_TABLE_END()
  // MDEBUG_JSON_END_ARRAY(points)

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

      l[i] = planning_math::Interpolate(traj_points[j].l, traj_points[j + 1].l, ratio);
      heading_angle[i] = planning_math::InterpolateAngle(
          traj_points[j].heading_angle, traj_points[j + 1].heading_angle,
          ratio);
      curvature[i] = planning_math::Interpolate(traj_points[j].curvature,
                                 traj_points[j + 1].curvature, ratio);
    }
  }
}

}  // namespace planning
