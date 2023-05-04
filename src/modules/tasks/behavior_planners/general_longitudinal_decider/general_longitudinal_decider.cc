#include "src/modules/tasks/behavior_planners/general_longitudinal_decider/general_longitudinal_decider.h"

#include <assert.h>

#include <cmath>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "src/modules/scc_function/adaptive_cruise_control.h"
#include "src/modules/scc_function/mrc_condition.h"
#include "src/modules/scc_function/start_stop_enable.h"

namespace planning {

using namespace planning_math;

GeneralLongitudinalDecider::GeneralLongitudinalDecider(
    const EgoPlanningConfigBuilder *config_builder,
    const std::shared_ptr<TaskPipelineContext> &pipeline_context)
    : Task(config_builder, pipeline_context) {
  config_ = config_builder->cast<LongitudinalDeciderV3Config>();
  name_ = "GeneralLongitudinalDecider";
}

bool GeneralLongitudinalDecider::Execute(planning::framework::Frame *frame) {
  frame_ = frame;
  LOG_DEBUG("=======GeneralLongitudinalDecider======= \n");
  if (Task::Execute(frame) == false) {
    return false;
  }

  auto config_builder =
      frame->session()->mutable_environmental_model()->config_builder(
          planning::common::SceneType::HIGHWAY);
  config_acc_ = config_builder->cast<AdaptiveCruiseControlConfig>();
  config_start_stop_ = config_builder->cast<StartStopEnableConfig>();

  lon_yield_info_.min_ds = 1000.0;
  lon_yield_info_.min_ttc = 100.0;
  lon_yield_info_.min_acc = 100.0;
  lon_yield_info_.min_jerk = 100.0;
  lon_yield_info_.keep_stop = false;

  auto &ego_planning_result = pipeline_context_->planning_result;
  auto &ego_planning_info = pipeline_context_->planning_info;
  auto &lon_ref_path = ego_planning_info.lon_ref_path;

  auto planning_init_point =
      reference_path_ptr_->get_frenet_ego_state().planning_init_point();
  const double ego_v = planning_init_point.v;
  const double ego_a = planning_init_point.a;
  const double ego_s = planning_init_point.frenet_state.s;

  auto ego_state =
      frame->session()->environmental_model().get_ego_state_manager();

  // Step 1)
  ReferencePathPoints refpath_points;
  construct_refpath_points(ego_planning_result.traj_points, refpath_points);

  // Step 2) generate lon decisions & lon bounds info
  generate_lon_decision_from_path(
      ego_planning_result.traj_points, refpath_points,
      ego_planning_info.obstacle_decisions, lon_ref_path);

  // execution leadone & cutin information
  auto &lon_decision_information = frame->mutable_session()
                                       ->mutable_planning_context()
                                       ->mutable_lon_decision_result();
  get_lon_decision_info(lon_decision_information);

  // enter into the ACC function
  auto acc_function = frame->session()
                          ->mutable_planning_context()
                          ->adaptive_cruise_control_function();
  auto &acc_info = frame->mutable_session()
                       ->mutable_planning_context()
                       ->mutable_adaptive_cruise_control_result();
  acc_function->adaptive_cruise_control(lon_decision_information, acc_info,
                                        ego_planning_result);
  bool acc_update_s_bound = false;
  double s_set_a = config_acc_.s_set_a;
  double total_time = 4.0;

  // enable start & stop function
  auto start_stop_info = ego_state->start_stop();
  StartStopInfo &start_stop_result = frame->mutable_session()
                                         ->mutable_planning_context()
                                         ->mutable_start_stop_result();
  auto start_stop_function =
      frame->session()->mutable_planning_context()->start_stop();
  double stop_weight_dx_config = config_start_stop_.dx_ref_weight;
  if (start_stop_function->enable_start_stop()) {
    start_stop_function->go_trajectory(lon_decision_information,
                                       start_stop_info, start_stop_result);
  }
  //   bool traffic_light_stop =
  //       ego_planning_info.traffic_light_decision.stop_flag &&
  //       ego_planning_info.traffic_light_decision.stop_distance < 3.0 &&
  //       ego_v < 0.5;
  bool enable_stop_flag =
      ((lon_yield_info_.keep_stop || start_stop_result.enable_stop) &&
       !frame->session()->is_parking_scene());
  LOG_DEBUG(
      "HHLDEBUGDB start stop input: %d, enable stop_flag: %d, is_start: %d, "
      "is_stop: % d, enable_stop: % d \n",
      start_stop_info, enable_stop_flag, start_stop_result.is_start,
      start_stop_result.is_stop, start_stop_result.enable_stop);

  // Step 3: 计算轨迹上的最小减速度，判断是否为紧急制动
  std::vector<double> traj_v_list;
  std::vector<double> traj_a_list;
  double min_traj_a = std::numeric_limits<double>::max();
  for (size_t i = 0; i < ego_planning_result.traj_points.size() - 1; ++i) {
    auto delta_s = ego_planning_result.traj_points[i + 1].s -
                   ego_planning_result.traj_points[i].s;
    traj_v_list.push_back(std::max(delta_s / config_.delta_time, 0.0));
  }
  for (size_t i = 0; i < traj_v_list.size() - 1; ++i) {
    auto delta_v = traj_v_list[i + 1] - traj_v_list[i];
    traj_a_list.push_back(delta_v / config_.delta_time);
    min_traj_a = std::fmin(min_traj_a, traj_a_list.back());
  }

  // Step 4: 初始化纵向参考轨迹
  assert(config_.lon_num_step + 1 ==
         static_cast<int>(ego_planning_result.traj_points.size()));
  const double s_rel =
      lon_decision_information.leadone_info.leadone_information.obstacle_s;
  bool v_direction =
      // (acc_info.navi_speed_control_info.v_ref - ego_v) > 0 ? true : false;
      true;  // WB: hack
  LOG_DEBUG(
      "HHLDEBUGB v_direction: %d, ego_s: %.2f, ego_a: %.2f, s_rel: %.2f, "
      "ego_v: %.2f \n",
      v_direction, ego_s, ego_a, s_rel, ego_v * 3.6);
  bool model_speed_up =
      acc_function->recognize_model_speed_up(ego_planning_result);
  // LOG_DEBUG("HHLDEBUGB model_speed_up: %d", model_speed_up);
  auto mrc_brake =
      frame->session()->mutable_planning_context()->mrc_condition();
  // MDEBUG_JSON_BEGIN_DICT(Mrc_v_ref_debug)
  mrc_brake->update_inline_brake_by_obstacle(lon_decision_information,
                                             planning_init_point);
  MrcBrakeType mrc_brake_type = mrc_brake->mrc_brake_type();

  double ego_v_real = ego_state->ego_v();
  mrc_brake->mrc_engage_p_gear(ego_v_real);
  // MDEBUG_JSON_ADD_ITEM(ego_v_real, ego_v_real, Mrc_v_ref_debug)
  // MDEBUG_JSON_ADD_ITEM(mrc_brake_type, static_cast<int>(mrc_brake_type),
  //                      Mrc_v_ref_debug)
  bool mrc_condition_enable = mrc_brake_type == MrcBrakeType::SLOW_BRAKE ||
                              mrc_brake_type == MrcBrakeType::HARD_BRAKE ||
                              mrc_brake_type == MrcBrakeType::EMERGENCY_BRAKE;
  for (size_t i = 0; i < ego_planning_result.traj_points.size(); i++) {
    lon_ref_path.t_list.emplace_back(i * config_.delta_time);
    // adjust s_ref
    if (enable_stop_flag) {
      lon_ref_path.s_refs.emplace_back(ego_planning_result.traj_points[0].s,
                                       1.0);
    } else {
      lon_ref_path.s_refs.emplace_back(ego_planning_result.traj_points[i].s,
                                       1.0);
    }

    // adjust v_ref
    if (enable_stop_flag) {
      lon_ref_path.ds_refs.emplace_back(0.0, stop_weight_dx_config);
    } else if (mrc_condition_enable) {
      mrc_brake->mrc_brake_execute(lon_ref_path, ego_planning_result,
                                   planning_init_point);
    } else {
      if (acc_info.navi_speed_control_info.enable_v_cost && v_direction &&
          !model_speed_up) {
        acc_function->acc_update_ds_refs(
            acc_info, lon_ref_path, ego_planning_result, planning_init_point);
        acc_update_s_bound = true;
      } else {
        lon_ref_path.ds_refs.emplace_back(ego_v, 0.0);
      }
    }
    // adjust s_limit
    WeightedBounds s_bounds;
    if (acc_update_s_bound) {
      double s_buff = 0.5 * s_set_a * total_time * total_time;
      s_bounds.emplace_back(WeightedBound{
          planning_init_point.frenet_state.s - 10.0,
          ego_planning_result.traj_points.back().s + s_buff, -1.0});
      lon_ref_path.bounds.emplace_back(s_bounds);
    } else {
      s_bounds.emplace_back(
          WeightedBound{planning_init_point.frenet_state.s - 10.0,
                        ego_planning_result.traj_points.back().s, -1.0});
      lon_ref_path.bounds.emplace_back(s_bounds);
    }

    // adjust v_limit
    lon_ref_path.lon_bound_v.emplace_back(
        Bound{0.0, config_.velocity_upper_bound});

    // emergence brake
    if (min_traj_a < -4) {
      // emergence brake
      lon_ref_path.lon_bound_a.emplace_back(Bound{-7.0, 7.0});
    } else {
      lon_ref_path.lon_bound_a.emplace_back(Bound{-4.0, 7.0});
    }

    lon_ref_path.lon_lead_bounds.emplace_back(WeightedLonLeadBounds{});
  }

  // adjust jerk_bound
  acc_function->acc_update_jerk_bound(acc_info, lon_ref_path,
                                      ego_planning_result, planning_init_point);
  if (ego_planning_info.traffic_light_decision.stop_flag) {
    double stop_distance =
        std::max(0.1, ego_planning_info.traffic_light_decision.stop_distance);
    lon_yield_info_.min_ds = std::min(lon_yield_info_.min_ds, stop_distance);
    lon_yield_info_.min_ttc =
        std::min(lon_yield_info_.min_ttc, stop_distance / std::max(0.1, ego_v));
    lon_yield_info_.min_acc =
        std::min(lon_yield_info_.min_acc, -ego_v * ego_v / 2.0 / stop_distance);
    lon_yield_info_.min_jerk = std::min(
        lon_yield_info_.min_jerk, -ego_v * ego_v / 2.0 / stop_distance - ego_a);
  }
  bool is_emergency =
      lon_yield_info_.min_ttc < config_.ttc_thld ||
      lon_yield_info_.min_jerk <
          std::min(0.0, planning_init_point.jerk - config_.delta_jerk_thld) ||
      lon_yield_info_.min_acc <
          planning_init_point.a - config_.delta_acc_thld ||
      mrc_condition_enable;
  size_t num_jerk_bound =
      std::min(config_.num_jerk_bound, lon_ref_path.lon_bound_jerk.size());
  const double kEgoAccThreshold{0.1};
  const double kEgoVelThreshold{2.0};
  const bool enable_jerk_bound_reshape =
      !is_emergency && !frame->session()->is_parking_scene() &&
      ego_a < kEgoAccThreshold && ego_v > kEgoVelThreshold;
  if (enable_jerk_bound_reshape) {
    LOG_DEBUG(
        "[GeneralLongitudinalDecider::execute] enable jerk bound reshape \n");
    for (size_t i = 0; i < num_jerk_bound; ++i) {
      double jerk_min_i = planning_init_point.jerk - config_.jerk_buffer -
                          i * config_.delta_time * config_.jerk_rate;
      lon_ref_path.lon_bound_jerk[i].lower =
          std::min(lon_ref_path.lon_bound_jerk[i].upper - config_.jerk_buffer,
                   std::max(lon_ref_path.lon_bound_jerk[i].lower, jerk_min_i));
    }
  }
  // MDEBUG_JSON_END_DICT(Mrc_v_ref_debug)
  // MDEBUG_JSON_BEGIN_DICT(lon_yield_info)
  // MDEBUG_JSON_ADD_ITEM(min_ds, lon_yield_info_.min_ds, lon_yield_info)
  // MDEBUG_JSON_ADD_ITEM(min_ttc, lon_yield_info_.min_ttc, lon_yield_info)
  // MDEBUG_JSON_ADD_ITEM(min_acc, lon_yield_info_.min_acc, lon_yield_info)
  // MDEBUG_JSON_ADD_ITEM(min_jerk, lon_yield_info_.min_jerk, lon_yield_info)
  // MDEBUG_JSON_ADD_ITEM(keep_stop, lon_yield_info_.keep_stop, lon_yield_info)
  // MDEBUG_JSON_END_DICT(lon_yield_info)

  // Step 5) get speed, a bound
  set_velocity_acceleration_bound(lon_ref_path);

  // Step 6) set traffic light bound
  auto stop_line_s = planning_init_point.frenet_state.s +
                     ego_planning_info.traffic_light_decision.stop_distance;
  lon_ref_path.bounds.back().emplace_back(
      WeightedBound{std::numeric_limits<double>::min(), stop_line_s, -1.0,
                    BoundInfo{0, "traffic light"}});

  // set destination bound for PNP
  if (frame_->session()->is_parking_scene()) {
    // set destination bound
    double distance_to_destination = get_distance_to_destination();
    double stop_distance_to_destination = config_.stop_distance_to_destination;
    double destination_s = planning_init_point.frenet_state.s +
                           distance_to_destination -
                           stop_distance_to_destination;
    lon_ref_path.bounds.back().emplace_back(
        WeightedBound{std::numeric_limits<double>::min(), destination_s, -1.0,
                      BoundInfo{0, "destination"}});
    // adjust s_limit by target parking space
    auto s_bound_upper = get_s_bound_by_target_parking_space();
    lon_ref_path.bounds.back().emplace_back(
        WeightedBound{std::numeric_limits<double>::min(), s_bound_upper, -1.0,
                      BoundInfo{0, "destination_parking_space"}});
    if (s_bound_upper < std::numeric_limits<double>::max() &&
        s_bound_upper > destination_s) {
      LOG_DEBUG("s_bound_upper > destination_s !!! \n");
    }
  }

  // Step 7) refine lon_ref_traj by obstacle
  for (auto &obstacle_decision : ego_planning_info.obstacle_decisions) {
    for (auto &position_decision :
         obstacle_decision.second.position_decisions) {
      if (position_decision.lon_decision == LonObstacleDecisionType::IGNORE) {
        continue;
      }
      for (size_t i = 0; i < lon_ref_path.t_list.size(); ++i) {
        auto t = lon_ref_path.t_list[i];
        if (std::fabs(t - position_decision.tp.t) < 1e-2) {
          auto &bounds = lon_ref_path.bounds[i];
          for (auto &lon_bound : position_decision.lon_bounds) {
            bounds.push_back(lon_bound);
            bounds.back().bound_info.type = "obstacle";
            bounds.back().bound_info.id = obstacle_decision.second.id_;
          }

          auto &lon_lead_bounds = lon_ref_path.lon_lead_bounds[i];
          lon_lead_bounds.insert(lon_lead_bounds.end(),
                                 position_decision.lon_lead_bounds.begin(),
                                 position_decision.lon_lead_bounds.end());

          for (auto &bound : position_decision.lon_bounds) {
            if (bound.weight < 0 and
                bound.upper < lon_ref_path.s_refs[i].first) {
              LOG_DEBUG(
                  "Decider bound obstacle_id: %d, t: %f, s_ref: %f, "
                  "upper_bound: %f\n",
                  obstacle_decision.first, t, lon_ref_path.s_refs[i].first,
                  bound.upper);
            }
          }
        }
      }
    }
  }

  // Step 8) refine s_ref by bound
  for (int i = static_cast<int>(lon_ref_path.t_list.size()) - 1; i >= 0; i--) {
    auto &s_ref = lon_ref_path.s_refs[i].first;
    if (i < static_cast<int>(lon_ref_path.t_list.size()) - 1) {
      s_ref = std::min(s_ref, lon_ref_path.s_refs[i + 1].first);
    }
    for (auto &bound : lon_ref_path.bounds[i]) {
      if (bound.weight < 0) {
        s_ref = std::min(s_ref, bound.upper);
      }
    }
    s_ref = std::max(s_ref, 0.0);
    LOG_DEBUG("parking_space_id_s_ref: = %f, index = %d \n", s_ref, i);
  }

  return true;
}

BoundedConstantJerkTrajectory1d GeneralLongitudinalDecider::get_velocity_limit(
    const LonRefPath &lon_ref_path) {
  // NTRACE_CALL(9);

  // get curvature velocity limit
  const auto ego_state =
      frame_->session()->environmental_model().get_ego_state_manager();
  const auto &map_info_manager =
      frame_->session()->environmental_model().get_virtual_lane_manager();
  const auto &traj_points = pipeline_context_->planning_result.traj_points;
  // 查找横向优化后路径上最大曲率，以及最大曲率处距离当前位置的距离
  double max_curv_s = -1.0;
  double path_curvature = 1e-4;
  double s_accumulate = 0.0;
  for (size_t i = 0; i < traj_points.size(); ++i) {
    if (std::fabs(traj_points[i].curvature) > path_curvature) {
      path_curvature = std::fabs(traj_points[i].curvature);
      max_curv_s = s_accumulate;
    }
    if (i > 0) {
      s_accumulate +=
          planning::fast_hypot(traj_points[i].x - traj_points[i - 1].x,
                               traj_points[i].y - traj_points[i - 1].y);
    }
  }
  LOG_DEBUG(
      "[VirtualLaneManager::get_velocity_limit] path_curvature: %f, "
      "max_curv_s: %f \n",
      path_curvature, max_curv_s);
  double steer_angle = ego_state->ego_steer_angle();
  double steer_curvature = std::tan(steer_angle / vehicle_param_.steer_ratio) /
                           vehicle_param_.wheel_base;
  double max_curvature = std::max(path_curvature, std::fabs(steer_curvature));
  const double max_lat_acceleration = compute_max_lat_acceleration();
  double v_limit_path_curv =
      std::max(config_.velocity_lower_bound,
               std::min(config_.velocity_upper_bound,
                        std::sqrt(max_lat_acceleration / path_curvature)));
  double v_limit_curv =
      std::max(config_.velocity_lower_bound,
               std::min(config_.velocity_upper_bound,
                        std::sqrt(max_lat_acceleration / max_curvature)));
  vel_limit_info_.v_limit_curv = v_limit_curv;
  auto &curv_info = pipeline_context_->planning_result.curvature_info;
  curv_info.max_curvature = max_curvature;
  curv_info.curv_velocity_limit = v_limit_curv;
  double vlimit_jerk = 0.0;
  double vlimit_acc = 0.0;
  double time_to_brake = 1e-2;
  auto planning_init_point =
      reference_path_ptr_->get_frenet_ego_state().planning_init_point();
  const double ego_v = planning_init_point.v;
  const double ego_a =
      std::abs(planning_init_point.a) < 1e-2 ? 1e-2 : planning_init_point.a;
  const double ego_jerk = planning_init_point.jerk;
  const double b_square_minus_4ac =
      std::pow((2 * ego_v + v_limit_path_curv), 2) + 6 * ego_a * max_curv_s;
  // 曲率限速要低于当前速度 && 最大曲率距离>0 && 根号内值大于0
  const double kTimeDistanceThreshold = 0.5;
  const bool enable_vlimit_curv_dec =
      v_limit_path_curv < ego_v &&
      max_curv_s > ego_v * kTimeDistanceThreshold && b_square_minus_4ac > 0.0;
  if (enable_vlimit_curv_dec) {
    time_to_brake =
        (-(2 * ego_v + v_limit_path_curv) + std::sqrt(b_square_minus_4ac)) /
        ego_a;
    time_to_brake = std::max(time_to_brake, 0.01);
    vlimit_jerk = 2 * (v_limit_path_curv - ego_v - ego_a * time_to_brake) /
                  std::pow(time_to_brake, 2);
  }

  // get map velocity limit
  constexpr double kMinMapVelocityLimit = 60.0 / 3.6;
  // vel_limit_info_.v_limit_map = map_info_manager->map_velocity_limit();
  vel_limit_info_.v_limit_map =
      map_info_manager->get_current_lane()->velocity_limit();
  double map_velocity_limit =
      std::max(vel_limit_info_.v_limit_map, kMinMapVelocityLimit);
  double real_map_limit =
      vel_limit_info_.v_limit_map * config_.velocity_upper_bound_scale_rate;
  map_velocity_limit =
      map_velocity_limit * config_.velocity_upper_bound_scale_rate;
  // double user_velocity_limit = map_info_manager->user_velocity_limit();
  double user_velocity_limit = ego_state->ego_v_cruise();
  vel_limit_info_.v_limit_usr = user_velocity_limit;
  bool disable_user_limit =
      (std::fabs(user_velocity_limit - real_map_limit) < 0.01 &&
       (user_velocity_limit < kMinMapVelocityLimit));

  // get final_velocity_limit
  double final_velocity_limit = vel_limit_info_.v_limit_curv;
  if (disable_user_limit) {
    final_velocity_limit = std::min(final_velocity_limit, kMinMapVelocityLimit);
  } else {
    final_velocity_limit =
        std::min(final_velocity_limit,
                 std::min(user_velocity_limit, map_velocity_limit));
  }

  if (frame_->session()->is_parking_scene()) {
    static constexpr double kVelocityPreviewDistance = 3.0;
    // auto mff_cruise_velocity = frame_->session()
    //                                ->environmental_model()
    //                                .get_vehicle_status()
    //                                .velocity()
    //                                .cruise_velocity()
    //                                .value_mps();
    auto mff_cruise_velocity = user_velocity_limit;
    map_velocity_limit =
        std::min(config_.velocity_limit_parking / 3.6, mff_cruise_velocity);

    // enable narrow area velocity limit in low max_curvature
    double narrow_area_velocity = std::numeric_limits<double>::max();
    narrow_area_velocity = get_narrow_area_velocity_limit();
    vel_limit_info_.v_limit_narrow_area = narrow_area_velocity;
    LOG_DEBUG("The narrow_area_velocity is = : %f", narrow_area_velocity);
    // add curvature velocity limit
    const auto &frenet_ego_state = reference_path_ptr_->get_frenet_ego_state();
    ReferencePathPoint refpath_pt;
    if (reference_path_ptr_->get_reference_point_by_lon(
            frenet_ego_state.s() + kVelocityPreviewDistance, refpath_pt)) {
      user_velocity_limit = map_velocity_limit;
      if (std::fabs(refpath_pt.curvature) > 0.1) {
        user_velocity_limit = std::min(
            (1.0 - std::fabs(refpath_pt.curvature)) * map_velocity_limit,
            5 / 3.6);
        user_velocity_limit = std::max(user_velocity_limit, 2.0);
      }
      vel_limit_info_.v_limit_usr = user_velocity_limit;
      final_velocity_limit = std::min(map_velocity_limit, user_velocity_limit);
    }
    final_velocity_limit = std::min(final_velocity_limit, narrow_area_velocity);
    LOG_DEBUG("curvature: %f, curvature_velocity_limit:%f",
              refpath_pt.curvature, user_velocity_limit);
  }

  vel_limit_info_.v_limit_final = final_velocity_limit;

  // get comfort brake traj
  const auto &frenet_ego_state = reference_path_ptr_->get_frenet_ego_state();
  constexpr double kComfortBrakeJerk = -0.25;
  constexpr double kEmergencyBrakeJerk = -3.0;
  constexpr double kDeltaJerkThld = 0.2;
  constexpr double kTimeToBrakeThld = 0.5;
  double enable_jerk = kComfortBrakeJerk;
  // 保证曲率减速jerk比-0.25小
  if (vlimit_jerk < kComfortBrakeJerk && time_to_brake > kTimeToBrakeThld) {
    enable_jerk = std::min(std::max(ego_jerk - kDeltaJerkThld,
                                    std::max(vlimit_jerk, kEmergencyBrakeJerk)),
                           kComfortBrakeJerk);
    vlimit_acc = (std::pow(v_limit_path_curv, 2) - std::pow(ego_v, 2)) /
                 (2 * max_curv_s);
  }

  BoundedConstantJerkTrajectory1d brake_traj(
      lon_ref_path.s_refs.front().first,
      frenet_ego_state.planning_init_point().v,
      std::max(frenet_ego_state.planning_init_point().a, vlimit_acc),
      enable_jerk, config_.delta_time);
  LOG_DEBUG(
      "[GeneralLongitudinalDecider] final_velocity_limit: %f, "
      "map_velocity_limit: "
      "%f, user_velocity_limit: %f,"
      "init_v: %f, init_a: %f, ego_v: %f, ego_a: %f, ego_jerk: %f, "
      "enable_jerk: %f",
      final_velocity_limit, map_velocity_limit, user_velocity_limit,
      frenet_ego_state.planning_init_point().v,
      frenet_ego_state.planning_init_point().a, frenet_ego_state.velocity(),
      frenet_ego_state.acc(), ego_jerk, enable_jerk);
  // set traj v,a bound
  brake_traj.set_bound(final_velocity_limit, std::numeric_limits<double>::max(),
                       std::numeric_limits<double>::lowest(),
                       std::numeric_limits<double>::max());

  // MDEBUG_JSON_BEGIN_DICT(vel_limit_info)
  // MDEBUG_JSON_ADD_ITEM(v_limit_final, vel_limit_info_.v_limit_final,
  //                      vel_limit_info)
  // MDEBUG_JSON_ADD_ITEM(v_limit_map, vel_limit_info_.v_limit_map,
  // vel_limit_info) MDEBUG_JSON_ADD_ITEM(v_limit_usr,
  // vel_limit_info_.v_limit_usr, vel_limit_info)
  // MDEBUG_JSON_ADD_ITEM(v_limit_curv, vel_limit_info_.v_limit_curv,
  //                      vel_limit_info)
  // MDEBUG_JSON_ADD_ITEM(v_limit_narrow_area,
  // vel_limit_info_.v_limit_narrow_area,
  //                      vel_limit_info)
  // MDEBUG_JSON_ADD_ITEM(max_curvature, max_curvature, vel_limit_info)
  // MDEBUG_JSON_ADD_ITEM(path_curvature, path_curvature, vel_limit_info)
  // MDEBUG_JSON_ADD_ITEM(steer_curvature, steer_curvature, vel_limit_info)
  // MDEBUG_JSON_ADD_ITEM(max_curv_s, max_curv_s, vel_limit_info)
  // MDEBUG_JSON_ADD_ITEM(ego_v, ego_v, vel_limit_info)
  // MDEBUG_JSON_ADD_ITEM(ego_a, ego_a, vel_limit_info)
  // MDEBUG_JSON_ADD_ITEM(v_limit_path_curv, v_limit_path_curv, vel_limit_info)
  // MDEBUG_JSON_ADD_ITEM(time_to_brake, time_to_brake, vel_limit_info)
  // MDEBUG_JSON_ADD_ITEM(vlimit_acc, vlimit_acc, vel_limit_info)
  // MDEBUG_JSON_ADD_ITEM(vlimit_jerk, vlimit_jerk, vel_limit_info)
  // MDEBUG_JSON_ADD_ITEM(ego_jerk_thld, ego_jerk - kDeltaJerkThld,
  // vel_limit_info) MDEBUG_JSON_ADD_ITEM(enable_jerk, enable_jerk,
  // vel_limit_info) MDEBUG_JSON_END_DICT(vel_limit_info)

  return brake_traj;
}

const double GeneralLongitudinalDecider::compute_max_lat_acceleration() const {
  constexpr double kUrbanVelocityThld = 3.0;
  constexpr double kHighwayVelocityThld = 20.0;
  constexpr double kUrbanLatAccBuffMax = 0.7;
  constexpr double kHighwayLatAccBuffMax = 0.5;
  constexpr double kUrbanLatAccRate = 0.2;
  constexpr double kHighwayLatAccRate = 0.05;
  constexpr double kLowSpeedLaneChangeAccBuff = 1.5;
  const auto &frenet_ego_state = reference_path_ptr_->get_frenet_ego_state();
  const auto &map_info_manager =
      frame_->session()->environmental_model().get_virtual_lane_manager();
  auto ego_velocity = frenet_ego_state.planning_init_point().v;
  auto lc_acc_decay =
      ((pipeline_context_->planning_target_state == ROAD_LC_LCHANGE ||
        pipeline_context_->planning_target_state == ROAD_LC_RCHANGE) &&
       ego_velocity < kUrbanVelocityThld)
          ? kLowSpeedLaneChangeAccBuff
          : 0.0;
  if (frame_->session()->environmental_model().is_on_highway()) {
    return config_.highway_max_lat_acceleration +
           std::min(kHighwayLatAccBuffMax,
                    kHighwayLatAccRate *
                        std::max(ego_velocity - kHighwayVelocityThld, 0.0));
  } else {
    return config_.urban_max_lat_acceleration - lc_acc_decay +
           std::min(kUrbanLatAccBuffMax,
                    kUrbanLatAccRate *
                        std::max(ego_velocity - kUrbanVelocityThld, 0.0));
  }
}

void GeneralLongitudinalDecider::set_velocity_acceleration_bound(
    LonRefPath &lon_ref_path) {
  // NTRACE_CALL(8);

  auto map_velocity_limit_brake_traj = get_velocity_limit(lon_ref_path);
  auto &curv_info = pipeline_context_->planning_result.curvature_info;
  auto planning_init_point =
      reference_path_ptr_->get_frenet_ego_state().planning_init_point();
  const double ego_a = planning_init_point.a;

  const double kVelocityUpperBound = config_.velocity_upper_bound;
  const double kAccUpperBound = std::max(
      config_.acceleration_lower_bound,
      config_.acceleration_upper_bound -
          config_.acceleration_curvature_ratio * curv_info.max_curvature);

  for (size_t i = 0; i < lon_ref_path.t_list.size(); ++i) {
    auto relative_time = lon_ref_path.t_list[i];

    // clip s
    auto s_i = map_velocity_limit_brake_traj.evaluate(0, relative_time);
    auto &s_ref = lon_ref_path.s_refs[i].first;
    s_ref = std::min(s_ref, s_i);

    // set v,a bound
    auto &bound_v = lon_ref_path.lon_bound_v[i];
    auto &bound_a = lon_ref_path.lon_bound_a[i];

    bound_v.upper = std::min(bound_v.upper, kVelocityUpperBound);
    auto v_i = map_velocity_limit_brake_traj.evaluate(1, relative_time);
    bound_v.upper = std::min(bound_v.upper, v_i);

    bound_a.upper = std::min(config_.acceleration_upper_bound,
                             std::max(kAccUpperBound, ego_a + 1e-2));
    LOG_DEBUG(
        "[GeneralLongitudinalDecider] vel_acc_bound, i: %d, s_i: %f, v_i: "
        "%f, v_upper: %f, a_upper: %f \n",
        i, s_i, v_i, bound_v.upper, bound_a.upper);
  }
}

void GeneralLongitudinalDecider::generate_lon_decision_from_path(
    const TrajectoryPoints &lateral_trajectory_points,
    const ReferencePathPoints &refpath_points,
    ObstacleDecisions &obstacle_decisions, LonRefPath &lon_ref_path) {
  // NTRACE_CALL(8);

  // preserve lon overtake decision and disble yield decision
  for (auto &obstacle_decision : obstacle_decisions) {
    for (auto &position_decision :
         obstacle_decision.second.position_decisions) {
      if (position_decision.lon_decision == LonObstacleDecisionType::YIELD) {
        position_decision.lon_decision = LonObstacleDecisionType::IGNORE;
      }
    }
  }

  // construct longitudinal obstacle decisions
  construct_longitudinal_obstacle_decisions(lateral_trajectory_points,
                                            refpath_points, obstacle_decisions,
                                            lon_ref_path);

  construct_longitudinal_outer_decision(
      lateral_trajectory_points, refpath_points,
      pipeline_context_->coarse_planning_info, obstacle_decisions);
}

bool GeneralLongitudinalDecider::check_longitudinal_ignore_obstacle(
    const std::shared_ptr<FrenetObstacle> obstacle) {
  auto ego_sl_boundary = reference_path_ptr_->get_ego_frenet_boundary();

  auto obstacle_sl_boundary = obstacle->frenet_obstacle_boundary();
  auto l_center = obstacle->frenet_l();

  const auto &map_info_manager =
      frame_->session()->environmental_model().get_virtual_lane_manager();
  const bool is_low_right_intersection =
      false;  // hack
              // map_info_manager->curr_intersection_with_traffic_light().getRoadOffset()
              // <
              //     10.0 &&
              // map_info_manager->traffic_light_direction() !=
              // Direction::GO_STRAIGHT;
  const double kLonIgnoreDistance = is_low_right_intersection ? 10.0 : 0.0;

  double distance_to_destination = get_distance_to_destination();

  // LPNP: obstacle's frenet is wrong when it out of route
  if (distance_to_destination < 10.0 && (obstacle->b_frenet_valid() == false)) {
    LOG_DEBUG(
        "The obstacle's frenet is invalid but it is needed to be cared by LPNP "
        "whose id : %d \n",
        obstacle->id());
    return false;
  }
  // behind ego car
  if ((frame_->session()->is_parking_scene() &&
       obstacle->type() != Common::ObjectType::OBJECT_TYPE_PEDESTRIAN)) {
    // the obstacle will be ignored when it behind rear axle for PNP
    if (obstacle_sl_boundary.s_end + vehicle_param_.length / 2 <
        ego_sl_boundary.s_end) {
      return true;
    }
  } else {
    if (obstacle_sl_boundary.s_end + kLonIgnoreDistance <
        ego_sl_boundary.s_end) {
      return true;
    }
  }

  // #pragma GCC diagnostic push
  // #pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  //   bool b_ego_intersection_turn_left =
  //       map_info_manager->ego_in_intersection() &&
  //       map_info_manager->current_intersection_task() ==
  //       Direction::TURN_LEFT;
  // #pragma GCC diagnostic pop
  //   const double kLatIgnoreDistance = b_ego_intersection_turn_left ? 30.0
  //   : 10.0; if (std::fabs(l_center) > kLatIgnoreDistance) {
  //     return true;
  //   }

  return false;
}

void GeneralLongitudinalDecider::construct_longitudinal_obstacle_decisions(
    const TrajectoryPoints &traj_points,
    const ReferencePathPoints &refpath_points,
    ObstacleDecisions &obstacle_decisions, LonRefPath &lon_ref_path) {
  // NTRACE_CALL(9);

  std::vector<Polygon2d> lon_overlap_path;
  make_longitudinal_overlap_path(traj_points, lon_overlap_path);

  pipeline_context_->planning_result.extra_json["lon_decision_error_info"] =
      "none";
  auto ego_sl_boundary = reference_path_ptr_->get_ego_frenet_boundary();
  auto ego_corners = reference_path_ptr_->get_frenet_ego_state().corners();

  // MDEBUG_JSON_BEGIN_ARRAY(longitudinal_obstacle_decisions_based_on_lateral_path)
  // MDEBUG_JSON_BEGIN_OBJECT(object)
  // MDEBUG_JSON_ADD_ITEM(ego_s_rear, ego_sl_boundary.s_start, object)
  // MDEBUG_JSON_ADD_ITEM(ego_s_front, ego_sl_boundary.s_end, object)
  // MDEBUG_JSON_ADD_ITEM(ego_l_right, ego_sl_boundary.l_start, object)
  // MDEBUG_JSON_ADD_ITEM(ego_l_left, ego_sl_boundary.l_end, object)
  // MDEBUG_JSON_ADD_ITEM(ego_s_front_left, ego_corners.s_front_left, object)
  // MDEBUG_JSON_ADD_ITEM(ego_l_front_left, ego_corners.l_front_left, object)
  // MDEBUG_JSON_ADD_ITEM(ego_s_front_right, ego_corners.s_front_right, object)
  // MDEBUG_JSON_ADD_ITEM(ego_l_front_right, ego_corners.l_front_right, object)
  // MDEBUG_JSON_ADD_ITEM(ego_s_rear_left, ego_corners.s_rear_left, object)
  // MDEBUG_JSON_ADD_ITEM(ego_l_rear_left, ego_corners.l_rear_left, object)
  // MDEBUG_JSON_ADD_ITEM(ego_s_rear_right, ego_corners.s_rear_right, object)
  // MDEBUG_JSON_ADD_ITEM(ego_l_rear_right, ego_corners.l_rear_right, object)
  // MDEBUG_JSON_END_OBJECT(object)
  // for (auto &obstacle : reference_path_ptr_->get_obstacles()) {
  //   auto obstacle_sl_boundary = obstacle->frenet_obstacle_boundary();
  //   auto obstacle_corners = obstacle->frenet_obstacle_corners();

  //   MDEBUG_JSON_BEGIN_OBJECT(object)
  //   MDEBUG_JSON_ADD_ITEM(obj_id, obstacle->id(), object)
  //   MDEBUG_JSON_ADD_ITEM(obj_vel, obstacle->frenet_velocity_s(), object)
  //   MDEBUG_JSON_ADD_ITEM(obj_vel_dir,
  //   obstacle->frenet_relative_velocity_angle(),
  //                        object)
  //   MDEBUG_JSON_ADD_ITEM(obj_s_rear, obstacle_sl_boundary.s_start, object)
  //   MDEBUG_JSON_ADD_ITEM(obj_s_front, obstacle_sl_boundary.s_end, object)
  //   MDEBUG_JSON_ADD_ITEM(obj_l_right, obstacle_sl_boundary.l_start, object)
  //   MDEBUG_JSON_ADD_ITEM(obj_l_left, obstacle_sl_boundary.l_end, object)
  //   MDEBUG_JSON_ADD_ITEM(obj_s_front_left, obstacle_corners.s_front_left,
  //                        object)
  //   MDEBUG_JSON_ADD_ITEM(obj_l_front_left, obstacle_corners.l_front_left,
  //                        object)
  //   MDEBUG_JSON_ADD_ITEM(obj_s_front_right, obstacle_corners.s_front_right,
  //                        object)
  //   MDEBUG_JSON_ADD_ITEM(obj_l_front_right, obstacle_corners.l_front_right,
  //                        object)
  //   MDEBUG_JSON_ADD_ITEM(obj_s_rear_left, obstacle_corners.s_rear_left,
  //   object) MDEBUG_JSON_ADD_ITEM(obj_l_rear_left,
  //   obstacle_corners.l_rear_left, object)
  //   MDEBUG_JSON_ADD_ITEM(obj_s_rear_right, obstacle_corners.s_rear_right,
  //                        object)
  //   MDEBUG_JSON_ADD_ITEM(obj_l_rear_right, obstacle_corners.l_rear_right,
  //                        object)
  //   MDEBUG_JSON_ADD_ITEM(
  //       obj_rel_pos,
  //       static_cast<int>(obstacle_decisions[obstacle->id()].rel_pos_type),
  //       object)
  //   MDEBUG_JSON_END_OBJECT(object)
  // }
  // MDEBUG_JSON_END_ARRAY(longitudinal_obstacle_decisions_based_on_lateral_path)

  int traj_length = traj_points.size();
  lon_ref_path.lon_obstacle_yield_info.reserve(traj_length);
  for (int i = 0; i < traj_length; i++) {
    lon_ref_path.lon_obstacle_yield_info.emplace_back(
        LonObstalceYieldInfo{0, 1000, 1000});
  }
  for (auto &obstacle : reference_path_ptr_->get_obstacles()) {
    if (check_longitudinal_ignore_obstacle(obstacle)) {
      continue;
    }
    auto obstacle_id = obstacle->id();
    if (obstacle->b_frenet_valid()) {
      construct_longitudinal_obstacle_decision(
          traj_points, refpath_points, lon_overlap_path, obstacle,
          obstacle_decisions[obstacle_id], lon_ref_path);
    }
  }

  // MDEBUG_JSON_BEGIN_DICT(lon_yield_obstacle)
  // MDEBUG_JSON_BEGIN_ARRAY(lon_yield_obj)
  // for (size_t i = 0; i < traj_points.size(); ++i) {
  //   MDEBUG_JSON_BEGIN_OBJECT(object)
  //   MDEBUG_JSON_ADD_ITEM(
  //       yield_id, lon_ref_path.lon_obstacle_yield_info[i].yield_id, object)
  //   MDEBUG_JSON_ADD_ITEM(yield_upper,
  //                        lon_ref_path.lon_obstacle_yield_info[i].yield_upper,
  //                        object)
  //   MDEBUG_JSON_ADD_ITEM(
  //       yield_buff, lon_ref_path.lon_obstacle_yield_info[i].yield_buff,
  //       object)
  //   MDEBUG_JSON_END_OBJECT(object)
  // }
  // MDEBUG_JSON_END_ARRAY(lon_yield_obj)
  // MDEBUG_JSON_END_DICT(lon_yield_obstacle)
  // MDEBUG_JSON_BEGIN_DICT(lon_st_obstacle)
  // MDEBUG_JSON_BEGIN_ARRAY(lon_obstacle_overlap)
  if (!lon_ref_path.lon_obstacle_yield_info.empty()) {
    for (size_t i = 0; i < lon_ref_path.lon_obstacle_yield_info.size(); i++) {
      auto iter = lon_ref_path.lon_obstacle_overlap_info.find(
          lon_ref_path.lon_obstacle_yield_info[i].yield_id);
      if (iter != lon_ref_path.lon_obstacle_overlap_info.end()) {
        lon_ref_path.lon_obstacle_overlap_info.erase(
            lon_ref_path.lon_obstacle_yield_info[i].yield_id);
      }
    }
    // for (auto lon_overlap_iter =
    // lon_ref_path.lon_obstacle_overlap_info.begin();
    //      lon_overlap_iter != lon_ref_path.lon_obstacle_overlap_info.end();
    //      ++lon_overlap_iter) {
    //   MDEBUG_JSON_BEGIN_OBJECT(object)
    //   MDEBUG_JSON_BEGIN_ARRAY(lon_obstacle_overlap_iter)
    //   for (size_t j = 0; j < lon_overlap_iter->second.size(); j++) {
    //     MDEBUG_JSON_BEGIN_OBJECT(object)
    //     MDEBUG_JSON_ADD_ITEM(id, lon_overlap_iter->first, object)
    //     MDEBUG_JSON_ADD_ITEM(s, lon_overlap_iter->second[j].s, object)
    //     MDEBUG_JSON_ADD_ITEM(t, lon_overlap_iter->second[j].t, object)
    //     MDEBUG_JSON_END_OBJECT(object)
    //   }
    //   MDEBUG_JSON_END_ARRAY(lon_obstacle_overlap_iter)
    //   MDEBUG_JSON_END_OBJECT(object)
    // }
    lon_ref_path.lon_obstacle_overlap_info.clear();
  }
  // MDEBUG_JSON_END_ARRAY(lon_obstacle_overlap)
  // MDEBUG_JSON_END_DICT(lon_st_obstacle)
  lon_ref_path.lon_obstacle_yield_info.clear();
}

void GeneralLongitudinalDecider::make_longitudinal_overlap_path(
    const TrajectoryPoints &traj_points, std::vector<Polygon2d> &overlap_path) {
  assert(traj_points.size() > 0);
  // config
  // TODO(WB) config_.lon_care_width
  auto care_width = vehicle_param_.width;

  auto make_polygon = [](const Vec2d &start, const Vec2d &end, double width) {
    planning_math::Vec2d left_begin_point(start.x(), start.y() + width / 2);
    planning_math::Vec2d left_end_point(end.x(), end.y() + width / 2);
    planning_math::Vec2d right_begin_point(start.x(), start.y() - width / 2);
    planning_math::Vec2d right_end_point(end.x(), end.y() - width / 2);
    return Polygon2d(
        {left_begin_point, left_end_point, right_end_point, right_begin_point});
  };

  for (size_t i = 1; i < traj_points.size(); ++i) {
    auto &pre_point = traj_points[i - 1];
    auto &cur_point = traj_points[i];
    overlap_path.emplace_back(make_polygon(Vec2d(pre_point.s, pre_point.l),
                                           Vec2d(cur_point.s, cur_point.l),
                                           care_width));
  }

  auto s_end = traj_points.front().s + config_.lon_care_length;
  if (traj_points.back().s < s_end) {
    auto &last_point = traj_points.back();
    overlap_path.emplace_back(make_polygon(Vec2d(last_point.s, last_point.l),
                                           Vec2d(s_end, last_point.l),
                                           care_width));
  }
}

void GeneralLongitudinalDecider::construct_longitudinal_obstacle_decision(
    const TrajectoryPoints &traj_points,
    const ReferencePathPoints &refpath_points,
    const std::vector<planning_math::Polygon2d> &overlap_path,
    const std::shared_ptr<FrenetObstacle> obstacle,
    ObstacleDecision &obstacle_decision, LonRefPath &lon_ref_path) {
  // config： 需要统一
  auto care_width = vehicle_param_.width;
  auto dt_square = config_.delta_time * config_.delta_time;
  auto dt_cube = dt_square * config_.delta_time;

  constexpr double kSOvertakeBuffer = 0.0;
  constexpr double kTOvertakeThreshold = 0.0;
  constexpr double kLowSpeedThreshold = 0.5;
  constexpr double kDisToCIPVThreshold = 3.0;

  std::vector<std::pair<double, bool>> obstacle_overlap_with_ego;
  std::vector<std::pair<double, double>> obstacle_overtake_lowers;
  std::vector<std::pair<double, double>> obstacle_yield_uppers;
  std::vector<LonObstacleOverlapInfo> overlap_info;

  const bool is_cross_obj =
      obstacle_decision.rel_pos_type == ObsRelPosType::CROSSING;

  for (size_t i = 0; i < traj_points.size(); i++) {
    auto &traj_pt = traj_points[i];
    auto t = traj_pt.t;
    obstacle_overlap_with_ego.emplace_back(t, false);
    obstacle_overtake_lowers.emplace_back(t,
                                          std::numeric_limits<double>::min());
    obstacle_yield_uppers.emplace_back(t, std::numeric_limits<double>::max());

    Polygon2d obstacle_sl_polygon;
    auto ok = obstacle->get_polygon_at_time(t, reference_path_ptr_,
                                            obstacle_sl_polygon);
    if (not ok) {
      continue;
    }

    std::vector<Vec2d> overlap_points;
    for (auto &overlap_path_polygon : overlap_path) {
      Polygon2d overlap_polygon;
      bool b_overlap = obstacle_sl_polygon.ComputeOverlap(overlap_path_polygon,
                                                          &overlap_polygon);
      if (b_overlap) {
        auto points = overlap_polygon.GetAllVertices();
        overlap_points.insert(overlap_points.end(), points.begin(),
                              points.end());
      }
    }

    // obstain all lon overlap info
    if ((overlap_points.empty() || i == traj_points.size() - 1) &&
        !overlap_info.empty()) {
      lon_ref_path.lon_obstacle_overlap_info.emplace(obstacle->id(),
                                                     overlap_info);
      overlap_info.clear();
    }

    if (overlap_points.empty()) {
      continue;
    }

    Polygon2d care_overlap_polygon;
    ok = care_overlap_polygon.ComputeConvexHull(overlap_points,
                                                &care_overlap_polygon);
    if (not ok) {
      continue;
    }

    if (ok) {
      overlap_info.emplace_back(LonObstacleOverlapInfo{
          care_overlap_polygon.min_x() - vehicle_param_.length / 2, t});
    }

    obstacle_overlap_with_ego.back().second = true;
    // calculate buffer
    if (t < kTOvertakeThreshold) {
      obstacle_overtake_lowers.back().second =
          std::numeric_limits<double>::max();
    } else {
      obstacle_overtake_lowers.back().second =
          care_overlap_polygon.max_x() + kSOvertakeBuffer;
    }

    obstacle_yield_uppers.back().second =
        care_overlap_polygon.min_x() - vehicle_param_.length / 2;

    if (i == 0) {
      // check initial collision
      if (!(care_overlap_polygon.min_x() - vehicle_param_.length / 2 >
                reference_path_ptr_->get_frenet_ego_state()
                    .planning_init_point()
                    .frenet_state.s ||
            care_overlap_polygon.max_x() + vehicle_param_.length / 2 <
                reference_path_ptr_->get_frenet_ego_state()
                    .planning_init_point()
                    .frenet_state.s)) {
        const auto &frenet_ego_state =
            reference_path_ptr_->get_frenet_ego_state();
        if (!frenet_ego_state.polygon().HasOverlap(obstacle_sl_polygon) &&
            frenet_ego_state.polygon().DistanceTo(obstacle_sl_polygon) > 0.2) {
          LOG_DEBUG("NP_DEBUG: Error! detect initial point collision! \n");
          pipeline_context_->planning_result
              .extra_json["lon_decision_error_info"] =
              "detect_initial_point_collision";
        }
      }
    }
  }

  // todo: ignore obstacle
  auto lon_decision = LonObstacleDecisionType::IGNORE;
  for (auto &pos_decision : obstacle_decision.position_decisions) {
    if (pos_decision.lon_decision == LonObstacleDecisionType::YIELD) {
      lon_decision = LonObstacleDecisionType::YIELD;
      break;
    }
  }
  if (lon_decision != LonObstacleDecisionType::YIELD) {
    for (size_t i = 0; i < traj_points.size(); ++i) {
      auto &traj_pt = traj_points[i];

      if (traj_pt.s < obstacle_overtake_lowers[i].second) {
        lon_decision = LonObstacleDecisionType::YIELD;
      }
    }
  }

  for (size_t i = 0; i < traj_points.size(); ++i) {
    auto t = traj_points[i].t;

    auto exist = false;
    for (auto &pos_decision : obstacle_decision.position_decisions) {
      if (std::fabs(pos_decision.tp.t - t) < 1e-2) {
        pos_decision.lon_decision = lon_decision;
        exist = true;
        break;
      }
    }

    if (not exist) {
      auto need_create = false;
      if (lon_decision == LonObstacleDecisionType::YIELD and
          obstacle_yield_uppers[i].second < 1000) {
        need_create = true;
      } else if (lon_decision == LonObstacleDecisionType::OVERTAKE and
                 obstacle_overtake_lowers[i].second > 0) {
        // todo(xbliu): add overtake constraints
        // need_create = true;
      }

      if (need_create) {
        auto pos_decision = ObstaclePositionDecision{};
        pos_decision.tp = {t, traj_points[i].s, traj_points[i].l};
        pos_decision.lon_decision = lon_decision;
        pos_decision.lat_decision = LatObstacleDecisionType::IGNORE;
        obstacle_decision.position_decisions.push_back(std::move(pos_decision));
      }
    }
  }

  // add bounds
  // get ignore relative time
  auto obstacle_sl_boundary = obstacle->frenet_obstacle_boundary();
  auto ego_s = reference_path_ptr_->get_frenet_ego_state().s();
  auto ego_acc = reference_path_ptr_->get_frenet_ego_state().acc();

  double ignore_relative_time = std::numeric_limits<double>::max();
  // inverse direction car
  if (fabs(planning_math::NormalizeAngle(
          obstacle->frenet_relative_velocity_angle())) > M_PI / 4 * 3 &&
      obstacle->velocity() > 1.0) {
    ignore_relative_time =
        std::min(ignore_relative_time, config_.lon_max_ignore_relative_time);
  }

  auto ego_sl_boundary = reference_path_ptr_->get_ego_frenet_boundary();
  bool is_CIPV = !(obstacle_sl_boundary.l_start > ego_sl_boundary.l_end ||
                   obstacle_sl_boundary.l_end < ego_sl_boundary.l_start);
  auto ego_velocity = reference_path_ptr_->get_frenet_ego_state().velocity_s();
  auto distance_buff =
      is_CIPV ? std::min(5.0, std::max(0.0, ego_velocity -
                                                obstacle->frenet_velocity_s()) *
                                  0.5)
              : 0.0;
  auto ini_ds = obstacle_sl_boundary.s_start - ego_sl_boundary.s_end;
  auto lat_overlap_ratio =
      is_CIPV
          ? std::min(1.0,
                     std::max(0.0, std::min(ego_sl_boundary.l_end -
                                                obstacle_sl_boundary.l_start,
                                            obstacle_sl_boundary.l_end -
                                                ego_sl_boundary.l_start)) /
                         care_width)
          : 0.0;
  // keep stop before close leadone go away, ignore prediction
  const double ego_relative_heading =
      reference_path_ptr_->get_frenet_ego_state().heading_angle();
  const double ego_front_left_s =
      reference_path_ptr_->get_frenet_ego_state().corners().s_front_left;
  const double ego_front_left_l =
      reference_path_ptr_->get_frenet_ego_state().corners().l_front_left;
  const double ego_front_right_s =
      reference_path_ptr_->get_frenet_ego_state().corners().s_front_right;
  const double ego_front_right_l =
      reference_path_ptr_->get_frenet_ego_state().corners().l_front_right;
  const auto &frenet_obstacle_corners = obstacle->frenet_obstacle_corners();
  const double rad2deg = 180.0 / 3.1415926;
  const double min_radius = std::tan(vehicle_param_.max_steer_angle * rad2deg /
                                     vehicle_param_.steer_ratio) /
                                vehicle_param_.wheel_base +
                            vehicle_param_.width / 2.0;
  const bool unable_to_nudge_from_left =
      std::pow(frenet_obstacle_corners.s_rear_left - ego_front_right_s +
                   min_radius * std::sin(ego_relative_heading),
               2) +
              std::pow(frenet_obstacle_corners.l_rear_left - ego_front_right_l -
                           min_radius * std::cos(ego_relative_heading),
                       2) <
          min_radius * min_radius ||
      std::pow(frenet_obstacle_corners.s_front_left - ego_front_right_s +
                   min_radius * std::sin(ego_relative_heading),
               2) +
              std::pow(frenet_obstacle_corners.l_front_left -
                           ego_front_right_l -
                           min_radius * std::cos(ego_relative_heading),
                       2) <
          min_radius * min_radius ||
      std::pow(frenet_obstacle_corners.s_rear_right - ego_front_right_s +
                   min_radius * std::sin(ego_relative_heading),
               2) +
              std::pow(frenet_obstacle_corners.l_rear_right -
                           ego_front_right_l -
                           min_radius * std::cos(ego_relative_heading),
                       2) <
          min_radius * min_radius ||
      std::pow(frenet_obstacle_corners.s_front_right - ego_front_right_s +
                   min_radius * std::sin(ego_relative_heading),
               2) +
              std::pow(frenet_obstacle_corners.l_front_right -
                           ego_front_right_l -
                           min_radius * std::cos(ego_relative_heading),
                       2) <
          min_radius * min_radius;
  const bool unable_to_nudge_from_right =
      std::pow(frenet_obstacle_corners.s_rear_right - ego_front_left_s -
                   min_radius * std::sin(ego_relative_heading),
               2) +
              std::pow(frenet_obstacle_corners.l_rear_right - ego_front_left_l +
                           min_radius * std::cos(ego_relative_heading),
                       2) <
          min_radius * min_radius ||
      std::pow(frenet_obstacle_corners.s_front_right - ego_front_left_s -
                   min_radius * std::sin(ego_relative_heading),
               2) +
              std::pow(frenet_obstacle_corners.l_front_right -
                           ego_front_left_l +
                           min_radius * std::cos(ego_relative_heading),
                       2) <
          min_radius * min_radius ||
      std::pow(frenet_obstacle_corners.s_rear_left - ego_front_left_s -
                   min_radius * std::sin(ego_relative_heading),
               2) +
              std::pow(frenet_obstacle_corners.l_rear_left - ego_front_left_l +
                           min_radius * std::cos(ego_relative_heading),
                       2) <
          min_radius * min_radius ||
      std::pow(frenet_obstacle_corners.s_front_left - ego_front_left_s -
                   min_radius * std::sin(ego_relative_heading),
               2) +
              std::pow(frenet_obstacle_corners.l_front_left - ego_front_left_l +
                           min_radius * std::cos(ego_relative_heading),
                       2) <
          min_radius * min_radius;
  const bool too_close_to_nudge =
      unable_to_nudge_from_left && unable_to_nudge_from_right;
  LOG_DEBUG(
      "[GeneralLongitudinalDecider]: too close to nudge: %d  unable from left: "
      "% d unable from right: % d \n ",
      too_close_to_nudge, unable_to_nudge_from_left,
      unable_to_nudge_from_right);
  bool keep_stop =
      (pipeline_context_->planning_target_state != ROAD_LC_LCHANGE &&
       pipeline_context_->planning_target_state != ROAD_LC_RCHANGE) &&
      is_CIPV && ini_ds > 0.0 && ini_ds < kDisToCIPVThreshold &&
      ego_velocity < kLowSpeedThreshold && too_close_to_nudge;
  lon_yield_info_.keep_stop = keep_stop || lon_yield_info_.keep_stop;
  for (size_t i = 0; i < obstacle_decision.position_decisions.size(); ++i) {
    auto &pos_decision = obstacle_decision.position_decisions[i];
    auto t = pos_decision.tp.t;
    if (t > ignore_relative_time or
        pos_decision.lon_decision != LonObstacleDecisionType::YIELD) {
      continue;
    }

    double frenet_point_s = obstacle->frenet_s();
    auto obstacle_point = obstacle->obstacle()->get_point_at_time(t);
    double relative_yaw = obstacle_point.velocity_direction -
                          frenet_coord_->GetRefCurveHeading(frenet_point_s);
    double obstacle_v_lon =
        std::max(obstacle_point.v * std::cos(relative_yaw), 0.0);
    bool b_on_ego_path = std::min(std::min(fabs(obstacle_sl_boundary.l_start),
                                           fabs(obstacle_sl_boundary.l_end)),
                                  fabs(obstacle->frenet_l())) < 1.5;

    auto distance_safe = 2.0;
    if (frame_->session()->is_parking_scene()) {
      static constexpr double kSafeDistanceInverse = 3.0;
      static constexpr double kDistanceInverseTTC = 0.3;
      double relative_velocity = ego_velocity - obstacle->frenet_velocity_s();
      distance_safe = kSafeDistanceInverse +
                      std::fabs(relative_velocity) * kDistanceInverseTTC;
    }
    auto obstacle_type = obstacle->type();
    bool b_fast_car =
        obstacle_type != Common::ObjectType::OBJECT_TYPE_BICYCLE and
        obstacle_type != Common::ObjectType::OBJECT_TYPE_PEDESTRIAN &&
        obstacle_v_lon > ego_velocity &&
        obstacle->obstacle()->acceleration() > 0.5;
    if (t < 0.3 && obstacle_yield_uppers[i].second < (5.0 + ego_s) &&
        b_fast_car) {
      distance_safe = 1.0;
    }

    auto yield_upper = std::numeric_limits<double>::max();
    for (auto &t_upper_bound : obstacle_yield_uppers) {
      if (std::fabs(t - t_upper_bound.first) < 1e-2) {
        yield_upper = std::min(yield_upper, t_upper_bound.second);
        if (keep_stop) {
          yield_upper = std::min(yield_upper, obstacle_sl_boundary.s_start -
                                                  vehicle_param_.length / 2);
          LOG_DEBUG(
              "[GeneralLongitudinalDecider]: keep stop for close leadone");
        }
      }
    }

    pos_decision.lon_bounds.emplace_back(WeightedBound{
        std::numeric_limits<double>::min(), yield_upper - distance_safe, -1});

    for (size_t i = 0; i < traj_points.size(); ++i) {
      auto t_traj = traj_points[i].t;
      if (std::fabs(t_traj - t) < 1e-2 &&
          lon_ref_path.lon_obstacle_yield_info[i].yield_upper > yield_upper) {
        lon_ref_path.lon_obstacle_yield_info[i].yield_upper = yield_upper;
        lon_ref_path.lon_obstacle_yield_info[i].yield_id = obstacle->id();
        lon_ref_path.lon_obstacle_yield_info[i].yield_buff = distance_safe;
      }
    }

    // update longitudinal yield info:
    double yield_ds = yield_upper - ego_s;
    lon_yield_info_.min_ds = std::min(lon_yield_info_.min_ds, yield_ds);
    lon_yield_info_.min_ttc =
        (obstacle_sl_boundary.s_start > ego_s &&
         ego_velocity > obstacle->frenet_velocity_s())
            ? std::min(lon_yield_info_.min_ttc,
                       (obstacle_sl_boundary.s_start - ego_s) /
                           (ego_velocity - obstacle->frenet_velocity_s()))
            : lon_yield_info_.min_ttc;
    if (yield_ds > 0.0) {
      if (obstacle_v_lon < ego_velocity) {
        double a_yield =
            (obstacle_v_lon * obstacle_v_lon - ego_velocity * ego_velocity) /
            (2.0 *
             std::max(yield_ds - distance_safe, 0.5 * vehicle_param_.length));
        lon_yield_info_.min_acc = std::min(lon_yield_info_.min_acc, a_yield);
      } else if (yield_ds - distance_safe < ego_velocity * t) {
        double a_yield = 2.0 * (yield_ds - distance_safe - ego_velocity * t) /
                         std::max(dt_square, t * t);
        lon_yield_info_.min_acc = std::min(lon_yield_info_.min_acc, a_yield);
      }
    }
    double jerk_yield =
        6.0 *
        (yield_ds - distance_safe - ego_velocity * t - 0.5 * ego_acc * t * t) /
        std::max(dt_cube, t * t * t);
    lon_yield_info_.min_jerk =
        jerk_yield < 0.0 ? std::min(lon_yield_info_.min_jerk, jerk_yield)
                         : lon_yield_info_.min_jerk;

    // add ttc cost:
    const bool b_high_speed_diff = ego_velocity > 25.0 / 3.6 &&
                                   ego_velocity > obstacle->frenet_velocity_s();
    const double t_ego =
        std::min(1.0, std::max(0.4, 2.0 * lat_overlap_ratio + 0.4 * t));
    const double ttc_pre = std::max(1.0, yield_upper - ego_sl_boundary.s_end) /
                           std::max(0.1, ego_velocity - obstacle_v_lon);
    const double able_to_yield =
        yield_ds > distance_safe && obstacle_v_lon < ego_velocity &&
        (obstacle_v_lon * obstacle_v_lon - ego_velocity * ego_velocity) / 2.0 /
                (yield_ds - distance_safe) >
            config_.max_deceleration;
    const double ttc_buff =
        ((b_on_ego_path || is_cross_obj) && able_to_yield)
            ? std::max(0.0, std::min(1.0, 0.5 * std::max(0.0, 4.0 - ttc_pre)))
            : 0.0;
    const bool enable_ttc_cost =
        (b_high_speed_diff || b_on_ego_path || (is_cross_obj && able_to_yield));
    LOG_DEBUG(
        "[GeneralLongitudinalDecider]: lon cost: id=%d  t=%f  ttc_pre=%f  "
        "ttc_buff=%f  t_ego=%f  enable_ttc_cost=%d  able_to_yield=%d  "
        "is_cross=%d  on_ego_path=%d \n",
        obstacle->id(), t, ttc_pre, ttc_buff, t_ego, enable_ttc_cost,
        able_to_yield, is_cross_obj, b_on_ego_path);
    if (enable_ttc_cost) {
      WeightedLonLeadBound bound;
      bound.s_lead = yield_upper - distance_safe - distance_buff;
      bound.v_lead = obstacle_v_lon;
      if (b_on_ego_path) {
        bound.t_lead = 2.0 + ttc_buff - t_ego;
        bound.t_ego = t_ego;
      } else {
        bound.t_lead = 2.0 + ttc_buff;
        bound.t_ego = 0.0;
      }
      bound.weight = 2.0;
      pos_decision.lon_lead_bounds.emplace_back(bound);
    }
  }
}

void GeneralLongitudinalDecider::prepare_obstacle_decision(
    ObstacleDecisions &obstacle_decisions, int obstacle_id) {
  // NTRACE_CALL(10);

  if (obstacle_decisions.find(obstacle_id) == obstacle_decisions.end()) {
    ObstacleDecision decision;
    decision.id_ = obstacle_id;
    obstacle_decisions[obstacle_id] = decision;
  }
}

void GeneralLongitudinalDecider::prepare_obstacle_position_decision(
    ObstacleDecision &obstacle_decision, const TrajectoryPoint &traj_pt) {
  // NTRACE_CALL(10);
  auto exist = false;

  for (auto &position_decision : obstacle_decision.position_decisions) {
    if (std::fabs(position_decision.tp.t - traj_pt.t) < 1e-2) {
      exist = true;
      break;
    }
  }

  if (not exist) {
    ObstaclePositionDecision position_decision;
    position_decision.tp = {traj_pt.t, traj_pt.s, traj_pt.l};
    position_decision.lon_decision = LonObstacleDecisionType::IGNORE;
    position_decision.lat_decision = LatObstacleDecisionType::IGNORE;
    obstacle_decision.position_decisions.push_back(position_decision);
  }
}

void GeneralLongitudinalDecider::construct_longitudinal_outer_decision(
    const TrajectoryPoints &traj_points,
    const ReferencePathPoints &refpath_points,
    const CoarsePlanningInfo &coarse_planning_info,
    ObstacleDecisions &obstacle_decisions) {
  // NTRACE_CALL(9);

  bool b_enable_lane_wait_adjust_speed = false;
  // Step 1) wait speed
  if (b_enable_lane_wait_adjust_speed &&
          coarse_planning_info.target_state == ROAD_LC_LWAIT ||
      coarse_planning_info.target_state == ROAD_LC_RWAIT) {
    auto overtake_obstacle_id = -1;
    auto yield_obstacle_id = -1;
    if (not coarse_planning_info.overtake_obstacles.empty()) {
      overtake_obstacle_id = coarse_planning_info.overtake_obstacles.front();
    }

    if (not coarse_planning_info.yield_obstacles.empty()) {
      yield_obstacle_id = coarse_planning_info.yield_obstacles.front();
    }

    auto wait_effect_speed = 2.0;
    if (overtake_obstacle_id > 0) {
      prepare_obstacle_decision(obstacle_decisions, overtake_obstacle_id);
      auto &obstacle_decision = obstacle_decisions[overtake_obstacle_id];

      auto conflict_with_safe = false;
      for (auto &pos_decision : obstacle_decision.position_decisions) {
        if (pos_decision.lon_decision == LonObstacleDecisionType::YIELD) {
          conflict_with_safe = true;
          break;
        }
      }
      if (not conflict_with_safe) {
        for (auto &obstacle : reference_path_ptr_->get_obstacles()) {
          if (obstacle->id() != overtake_obstacle_id) {
            continue;
          }

          for (auto &traj_pt : traj_points) {
            if (traj_pt.t < wait_effect_speed) {
              continue;
            }

            prepare_obstacle_position_decision(obstacle_decision, traj_pt);
            for (auto &pos_decision : obstacle_decision.position_decisions) {
              if (std::fabs(pos_decision.tp.t - traj_pt.t) < 1e-2) {
                if (pos_decision.lon_decision ==
                    LonObstacleDecisionType::IGNORE) {
                  pos_decision.lon_decision = LonObstacleDecisionType::OVERTAKE;

                  Polygon2d obstacle_sl_polygon;
                  auto ok = obstacle->get_polygon_at_time(
                      traj_pt.t, reference_path_ptr_, obstacle_sl_polygon);
                  if (not ok) {
                    continue;
                  }
                  pos_decision.lon_bounds.push_back(WeightedBound{
                      obstacle_sl_polygon.max_x() + vehicle_param_.length / 2,
                      std::numeric_limits<double>::max(), 1.0});
                  LOG_DEBUG(
                      "wait_speed overtake obstacle_id:%d, t:%f, "
                      "lower_bound:%f \n",
                      overtake_obstacle_id, traj_pt.t,
                      obstacle_sl_polygon.max_x() + vehicle_param_.length / 2);
                }
              }
            }
          }
        }
      }
    }

    if (yield_obstacle_id > 0) {
      prepare_obstacle_decision(obstacle_decisions, yield_obstacle_id);
      auto &obstacle_decision = obstacle_decisions[yield_obstacle_id];

      auto conflict_with_safe = false;
      for (auto &pos_decision : obstacle_decision.position_decisions) {
        if (pos_decision.lon_decision == LonObstacleDecisionType::OVERTAKE) {
          conflict_with_safe = true;
          break;
        }
      }
      if (not conflict_with_safe) {
        for (auto &obstacle : reference_path_ptr_->get_obstacles()) {
          if (obstacle->id() != yield_obstacle_id) {
            continue;
          }

          for (auto &traj_pt : traj_points) {
            if (traj_pt.t < wait_effect_speed) {
              continue;
            }

            prepare_obstacle_position_decision(obstacle_decision, traj_pt);
            for (auto &pos_decision : obstacle_decision.position_decisions) {
              if (std::fabs(pos_decision.tp.t - traj_pt.t) < 1e-2) {
                if (pos_decision.lon_decision ==
                    LonObstacleDecisionType::IGNORE) {
                  pos_decision.lon_decision = LonObstacleDecisionType::YIELD;

                  Polygon2d obstacle_sl_polygon;
                  auto ok = obstacle->get_polygon_at_time(
                      traj_pt.t, reference_path_ptr_, obstacle_sl_polygon);
                  if (not ok) {
                    continue;
                  }
                  pos_decision.lon_bounds.push_back(WeightedBound{
                      std::numeric_limits<double>::min(),
                      obstacle_sl_polygon.min_x() - vehicle_param_.length / 2,
                      1.0});
                  LOG_DEBUG(
                      "wait_speed yield obstacle_id:%d, t:%f,upper_bound: % f "
                      "\n",
                      yield_obstacle_id, traj_pt.t,
                      obstacle_sl_polygon.min_x() - vehicle_param_.length / 2);
                }
              }
            }
          }
        }
      }
    }
  }
}

void GeneralLongitudinalDecider::construct_refpath_points(
    const TrajectoryPoints &traj_points, ReferencePathPoints &refpath_points) {
  // NTRACE_CALL(8);

  for (auto &traj_pt : traj_points) {
    ReferencePathPoint refpath_pt;
    auto success =
        reference_path_ptr_->get_reference_point_by_lon(traj_pt.s, refpath_pt);
    (void)success;
    assert(success);
    refpath_points.emplace_back(std::move(refpath_pt));
  }

  if (pipeline_context_->planning_target_state != ROAD_NONE) {
    for (auto &pt : refpath_points) {
      pt.distance_to_left_lane_border = 10;
      pt.distance_to_right_lane_border = 10;
    }
  }
}

void GeneralLongitudinalDecider::construct_lon_decision_trajectory(
    const TrajectoryPoints &traj_points,
    TrajectoryPoints &lon_decision_traj_points) {
  lon_decision_traj_points.clear();
  for (auto traj_pt : traj_points) {
    // todo: add l protection
    lon_decision_traj_points.push_back(traj_pt);
  }
}

void GeneralLongitudinalDecider::get_lon_decision_info(
    LonDecisionInfo &lon_decision_information) {
  // NTRACE_CALL(8);

  auto &ego_planning_info = pipeline_context_->planning_info;
  double leadone_min_s_leadone = 10000.0;
  int current_lane_virtual_id = frame_->session()
                                    ->environmental_model()
                                    .get_virtual_lane_manager()
                                    ->current_lane_virtual_id();
  auto reference_path_manager =
      frame_->session()->environmental_model().get_reference_path_manager();
  auto current_reference_path =
      reference_path_manager->get_reference_path_by_lane(
          current_lane_virtual_id);
  // auto frenet_obstacles_map =
  current_reference_path->get_obstacles_map();
  auto frenet_obstacles_map = reference_path_ptr_->get_obstacles_map();
  bool has_lon_decision_overtake = false;
  bool has_lon_decision_yield = false;
  auto ego_s = reference_path_ptr_->get_frenet_ego_state().s();
  const double half_ego_length = vehicle_param_.length / 2.0;

  ReferencePathPoint refpath_pt;
  bool get_lon_success = false;

  auto c_lane = reference_path_manager->get_reference_path_by_current_lane();
  get_lon_success = c_lane->get_reference_point_by_lon(ego_s, refpath_pt);

  lon_decision_information.cutin_info.cutin_information.clear();
  lon_decision_information.CIPV_info.CIPV_information.clear();
  lon_decision_information.leadone_info.has_leadone = false;
  lon_decision_information.CIPV_info.has_CIPV = false;
  lon_decision_information.cutin_info.has_cutin = false;
  lon_decision_information.nearby_obstacle = false;
  ObstacleInformation CIPV_object;
  ObstacleInformation cutin_object;
  if (!frenet_obstacles_map.empty()) {
    const double ego_l_start =
        reference_path_ptr_->get_frenet_ego_state().boundary().l_start;
    const double ego_l_end =
        reference_path_ptr_->get_frenet_ego_state().boundary().l_end;
    for (auto &obstacle_decision : ego_planning_info.obstacle_decisions) {
      auto find_object = frenet_obstacles_map.find(obstacle_decision.first);
      if (find_object == frenet_obstacles_map.end()) {
        continue;
      }
      auto &frenet_obstacle = find_object->second;

      // confirm nearby obstacle
      constexpr double lateral_v_threshold = 0.5;
      bool l_away_ego = false;
      bool left_satisfy =
          frenet_obstacle->frenet_obstacle_boundary().l_start > ego_l_end &&
          frenet_obstacle->frenet_velocity_l() > lateral_v_threshold;
      bool right_satisfy =
          frenet_obstacle->frenet_obstacle_boundary().l_end < ego_l_start &&
          frenet_obstacle->frenet_velocity_l() < -lateral_v_threshold;
      if (!lon_decision_information.nearby_obstacle) {
        if (left_satisfy || right_satisfy) {
          l_away_ego = true;
        }
        LOG_DEBUG(
            "HHLDEBUGW id: %d, frenet_velocity_l: %.2f, f_l_start: %.2f, "
            "f_l_end: %.2f, left_satisfy: %d, right_satisfy: %d, "
            "ego_l_start: %.2f, ego_l_end: %.2f, frenet_obstacle_s: %.2f, "
            "l_away_ego: %d, ego_s: %.2f, get_lon_success: %d \n",
            obstacle_decision.first, frenet_obstacle->frenet_velocity_l(),
            frenet_obstacle->frenet_obstacle_boundary().l_start,
            frenet_obstacle->frenet_obstacle_boundary().l_end, left_satisfy,
            right_satisfy, ego_l_start, ego_l_end, frenet_obstacle->frenet_s(),
            l_away_ego, ego_s, get_lon_success);
        if (!l_away_ego) {
          if (get_lon_success) {
            double distance_to_left_road_border =
                refpath_pt.distance_to_left_road_border;
            double distance_to_right_road_border =
                refpath_pt.distance_to_right_road_border;
            bool lateral_satisfy =
                (frenet_obstacle->frenet_obstacle_boundary().l_start >
                     ego_l_end &&
                 frenet_obstacle->frenet_obstacle_boundary().l_start <
                     distance_to_left_road_border) ||
                (frenet_obstacle->frenet_obstacle_boundary().l_end <
                     ego_l_start &&
                 frenet_obstacle->frenet_obstacle_boundary().l_end >
                     -distance_to_right_road_border);
            bool longitual_satisfy =
                (frenet_obstacle->frenet_s() - ego_s > -10.0) &&
                (frenet_obstacle->frenet_s() - ego_s < 100.0);
            lon_decision_information.nearby_obstacle =
                lon_decision_information.nearby_obstacle ||
                (lateral_satisfy && longitual_satisfy);
            LOG_DEBUG(
                "HHLDEBUGW distance_to_left_road_border: %.2f, "
                "distance_to_right_road_border: %.2f, lateral_satisfy: "
                "%d,longitual_satisfy: % d \n",
                distance_to_left_road_border, distance_to_right_road_border,
                lateral_satisfy, longitual_satisfy);
          } else {
            bool s_l_satisfy =
                (frenet_obstacle->frenet_obstacle_boundary().l_start >
                     ego_l_end ||
                 frenet_obstacle->frenet_obstacle_boundary().l_end <
                     ego_l_start) &&
                (frenet_obstacle->frenet_s() - ego_s > -50.0) &&
                (frenet_obstacle->frenet_s() - ego_s < 100.0);
            lon_decision_information.nearby_obstacle =
                lon_decision_information.nearby_obstacle || s_l_satisfy;
            LOG_DEBUG("HHLDEBUGW s_l_satisfy: %d \n", s_l_satisfy);
          }
        }
      }
      LOG_DEBUG("HHLDEBUGW nearby_obstacle: %d",
                lon_decision_information.nearby_obstacle);

      if (!obstacle_decision.second.position_decisions.empty()) {
        has_lon_decision_overtake =
            obstacle_decision.second.position_decisions[0].lon_decision ==
            LonObstacleDecisionType::OVERTAKE;
        has_lon_decision_yield =
            obstacle_decision.second.position_decisions[0].lon_decision ==
            LonObstacleDecisionType::YIELD;
      } else {
        continue;
      }
      const bool satisfy_s_leadone =
          (frenet_obstacle->frenet_s() - ego_s - half_ego_length <
           leadone_min_s_leadone) &&
          frenet_obstacle->frenet_s() - ego_s - half_ego_length > 0;
      const bool satisfy_s_CIPV =
          frenet_obstacle->frenet_s() - ego_s - half_ego_length > 0;
      const bool polygen_out_lane =
          frenet_obstacle->frenet_obstacle_boundary().l_start > ego_l_end ||
          frenet_obstacle->frenet_obstacle_boundary().l_end < ego_l_start;
      LOG_DEBUG(
          "HHLDEBUG: l_start: %.2f, l_end: %.2f, ego_s: %.2f, "
          "ego_l_start:%.2f, ego_l_end: %.2f, half_ego_length: %.2f \n",
          frenet_obstacle->frenet_obstacle_boundary().l_start,
          frenet_obstacle->frenet_obstacle_boundary().l_end, ego_s, ego_l_start,
          ego_l_end, half_ego_length);
      LOG_DEBUG(
          "HHLDEBUG: has_lon_decision_overtake: %d, satisfy_s_leadone: "
          "%d,satisfy_s_CIPV: % d, polygen_out_lane : % d \n",
          has_lon_decision_overtake, satisfy_s_leadone, satisfy_s_CIPV,
          polygen_out_lane);
      if (!polygen_out_lane) {
        // obstain leadone information
        lon_decision_information.leadone_info.has_leadone = true;
        if (satisfy_s_leadone) {
          lon_decision_information.leadone_info.leadone_information.obstacle_s =
              frenet_obstacle->frenet_s() - ego_s - half_ego_length;
          lon_decision_information.leadone_info.leadone_information
              .obstacle_id = obstacle_decision.first;
          lon_decision_information.leadone_info.leadone_information
              .obstacle_length =
              std::max(frenet_obstacle->frenet_obstacle_boundary().s_end -
                           frenet_obstacle->frenet_obstacle_boundary().s_start,
                       4.0);
          leadone_min_s_leadone = lon_decision_information.leadone_info
                                      .leadone_information.obstacle_s;
          lon_decision_information.leadone_info.leadone_information.obstacle_v =
              frenet_obstacle->velocity();
          if (frenet_obstacle->type() ==
                  Common::ObjectType::OBJECT_TYPE_PEDESTRIAN ||
              frenet_obstacle->type() ==
                  Common::ObjectType::OBJECT_TYPE_BICYCLE) {
            lon_decision_information.leadone_info.leadone_information
                .obstacle_type = 0;
          } else {
            lon_decision_information.leadone_info.leadone_information
                .obstacle_type = 1;
          }
        }
        // obtain CIPV information
        if (satisfy_s_CIPV) {
          lon_decision_information.CIPV_info.has_CIPV = true;
          CIPV_object.obstacle_s =
              frenet_obstacle->frenet_s() - ego_s - half_ego_length;
          CIPV_object.obstacle_id = obstacle_decision.first;
          CIPV_object.obstacle_v = frenet_obstacle->velocity();
          if (frenet_obstacle->type() ==
                  Common::ObjectType::OBJECT_TYPE_PEDESTRIAN ||
              frenet_obstacle->type() ==
                  Common::ObjectType::OBJECT_TYPE_BICYCLE) {
            CIPV_object.obstacle_type = 0;
          } else {
            CIPV_object.obstacle_type = 1;
          }
          lon_decision_information.CIPV_info.CIPV_information.emplace_back(
              CIPV_object);
        }
      }
      // obstain cutin information
      else if (polygen_out_lane && has_lon_decision_yield) {
        lon_decision_information.cutin_info.has_cutin = true;
        if (frenet_obstacle->type() ==
                Common::ObjectType::OBJECT_TYPE_PEDESTRIAN ||
            frenet_obstacle->type() ==
                Common::ObjectType::OBJECT_TYPE_BICYCLE) {
          cutin_object.obstacle_type = 0;
        } else {
          cutin_object.obstacle_type = 1;
        }
        cutin_object.obstacle_s =
            frenet_obstacle->frenet_s() - ego_s - half_ego_length;
        cutin_object.obstacle_v = frenet_obstacle->velocity();
        cutin_object.obstacle_id = obstacle_decision.first;
        lon_decision_information.cutin_info.cutin_information.emplace_back(
            cutin_object);
      }
    }
  }
  // debug for lon decision information
  if (lon_decision_information.leadone_info.has_leadone) {
    LOG_DEBUG(
        "HHLDEBUG: has_leadone: %d, s: %.2f, id: %d, v: %.2f, type: %d, "
        "length: %.2f, nearby_obstacle: %d",
        lon_decision_information.leadone_info.has_leadone,
        lon_decision_information.leadone_info.leadone_information.obstacle_s,
        lon_decision_information.leadone_info.leadone_information.obstacle_id,
        lon_decision_information.leadone_info.leadone_information.obstacle_v,
        lon_decision_information.leadone_info.leadone_information.obstacle_type,
        lon_decision_information.leadone_info.leadone_information
            .obstacle_length,
        lon_decision_information.nearby_obstacle);
  }
  if (lon_decision_information.CIPV_info.has_CIPV) {
    LOG_DEBUG("HHLDEBUG: has_CIPV: %d",
              lon_decision_information.leadone_info.has_leadone);
  }
  if (lon_decision_information.cutin_info.has_cutin) {
    LOG_DEBUG("HHLDEBUG: has_cutin: %d",
              lon_decision_information.cutin_info.has_cutin);
  }
  frenet_obstacles_map.clear();
}

bool GeneralLongitudinalDecider::check_obstacle_both_sides(
    const std::shared_ptr<FrenetObstacle> obstacle) {
  auto ego_sl_boundary = reference_path_ptr_->get_ego_frenet_boundary();

  auto obstacle_sl_boundary = obstacle->frenet_obstacle_boundary();

  const double kLonIgnoreDistance = 0.0;
  // obstacel is near the side of ego car
  if (obstacle_sl_boundary.s_end - kLonIgnoreDistance >
          ego_sl_boundary.s_start &&
      obstacle_sl_boundary.s_end + kLonIgnoreDistance < ego_sl_boundary.s_end) {
    return true;
  }
  return false;
}

double GeneralLongitudinalDecider::get_distance_to_destination() {
  // WB: 暂无地图，终点距离无穷大
  double distance_to_destination = 2000.0;
  // auto dest = frame_->session()
  //                 ->environmental_model()
  //                 .get_virtual_lane_manager()
  //                 ->get_dest();
  // auto distance_to_destination = std::numeric_limits<double>::max();
  // if (dest.isValid()) {
  //   distance_to_destination = dest.hasLanePathOffset()
  //                                 ? dest.getLanePathOffset()
  //                                 : dest.getRoadOffset();
  // }
  return distance_to_destination;
}

double GeneralLongitudinalDecider::get_narrow_area_velocity_limit() {
  // constexpr double narrow_space_width_thrshld_safe = 1.5;
  // constexpr double narrow_space_width_thrshld_easy = 0.8;
  // constexpr double narrow_space_width_thrshld_hard = 0.1;
  // constexpr double kVelocityAttenLimit = 2.7;    // 10km/h
  // constexpr double kVelocityWarnLimit = 1.38;    // 5km/h
  // constexpr double kVelocityDangerLimit = 0.83;  // 3km/h
  // std::vector<int> cared_obstacle_ids;
  // std::vector<PassableAreaInfo> passable_area_info;
  // PassableAreaInfo passable_area;

  // double narrow_space_width_stop_thrshld =
  //     config_.narrow_space_width_stop_thrshld;
  // double narrow_space_distance_stop_thrshld =
  //     config_.narrow_space_distance_stop_thrshld;
  double suggest_velocity_limit = std::numeric_limits<double>::max();
  // passable_area_info = pipeline_context_->planning_info.passable_area_info;
  // double width_minimum = std::numeric_limits<double>::max();
  // for (size_t i = 0; i < passable_area_info.size(); i++) {
  //   double width = passable_area_info[i].width;
  //   if (width < width_minimum) {
  //     cared_obstacle_ids.clear();
  //     width_minimum = width;
  //     // check if the bound has cared_types and get cared_obstacle_ids
  //     if (passable_area_info[i].left_bound_info.type ==
  //         "push_upper_bound_obstacle") {
  //       cared_obstacle_ids.emplace_back(
  //           passable_area_info[i].left_bound_info.id);
  //     }
  //     if (passable_area_info[i].right_bound_info.type ==
  //         "push_lower_bound_obstacle") {
  //       cared_obstacle_ids.emplace_back(
  //           passable_area_info[i].right_bound_info.id);
  //     }
  //   }
  //   LOG_DEBUG("The path width is = : %f", width);
  // }
  // // get the nearest distance of obstacle which pushed the bound
  // double closest_distance = std::numeric_limits<double>::max();
  // for (size_t i = 0; i < cared_obstacle_ids.size(); i++) {
  //   auto frenet_obstacles_map = reference_path_ptr_->get_obstacles_map();
  //   auto find_object = frenet_obstacles_map.find(cared_obstacle_ids[i]);
  //   // cared_obstacle
  //   if (find_object == frenet_obstacles_map.end()) {
  //     continue;
  //   }
  //   auto &cared_obstacle = find_object->second;
  //   auto planning_init_point =
  //       reference_path_ptr_->get_frenet_ego_state().planning_init_point();
  //   closest_distance =
  //       std::min(cared_obstacle.frenet_s() -
  //       planning_init_point.frenet_state.s,
  //                closest_distance);
  // }

  // if (width_minimum < narrow_space_width_thrshld_hard) {
  //   NTRACE("Dangerous! There is a narrow path.");
  //   NLOGW("Dangerous! The minimum path width is = : %f", width_minimum);
  //   suggest_velocity_limit = kVelocityDangerLimit;
  // } else if (width_minimum < narrow_space_width_thrshld_easy) {
  //   NTRACE("Warning! There is a narrow path.");
  //   NLOGW("Warning! The minimum path width is = : %f", width_minimum);
  //   suggest_velocity_limit = kVelocityWarnLimit;
  // } else if (width_minimum < narrow_space_width_thrshld_safe) {
  //   NTRACE("Attention! There is a narrow path.");
  //   NLOGW("Attention! The minimum path width is = : %f", width_minimum);
  //   suggest_velocity_limit =
  //       kVelocityWarnLimit +
  //       (kVelocityAttenLimit - kVelocityWarnLimit) /
  //           (narrow_space_width_thrshld_safe -
  //            narrow_space_width_thrshld_easy) *
  //           (width_minimum - narrow_space_width_thrshld_easy);
  // }
  // // The vehicle should stop
  // if (width_minimum < narrow_space_width_stop_thrshld &&
  //     closest_distance < narrow_space_distance_stop_thrshld) {
  //   NTRACE("Dangerous! The vehilce can't get through the path.");
  //   NLOGW(
  //       "Dangerous! The vehilce can't get through the area whose minimum
  //       path " "width is = : %f", width_minimum);
  //   suggest_velocity_limit = 0.0;
  // }
  // MDEBUG_JSON_BEGIN_DICT(narrow_area_info)
  // MDEBUG_JSON_ADD_ITEM(min_width, width_minimum, narrow_area_info)
  // MDEBUG_JSON_ADD_ITEM(closest_distance, closest_distance,
  // narrow_area_info) MDEBUG_JSON_ADD_ITEM(suggest_velocity_limit,
  // suggest_velocity_limit,
  //                      narrow_area_info)
  // MDEBUG_JSON_END_DICT(narrow_area_info)

  return suggest_velocity_limit;
}

double GeneralLongitudinalDecider::get_s_bound_by_target_parking_space() {
  // static double s_bound_upper_tmp = std::numeric_limits<double>::max();
  // auto planning_init_point =
  //     reference_path_ptr_->get_frenet_ego_state().planning_init_point();
  // auto aimed_parking_space_id = frame_->session()
  //                                   ->mutable_planning_context()
  //                                   ->mutable_parking_status_info()
  //                                   ->parking_space_id();
  // LOG_DEBUG("aimed_parking_space_id: id= %d", aimed_parking_space_id);
  // double distance_to_destination = get_destination_distance();
  // static double s_bound_upper_remaining_distance_to_destination =
  //     std::numeric_limits<double>::lowest();
  // double s_bound_upper = planning_init_point.frenet_state.s +
  //                        distance_to_destination -
  //                        s_bound_upper_remaining_distance_to_destination;
  // LOG_DEBUG(
  //     "aimed_parking_spaces_s_bound_upper : %f, s_bound_upper_tmp: %f, "
  //     "s_bound_upper_remaining_distance_to_destination : %f, "
  //     "distance_to_destination : %f",
  //     s_bound_upper, s_bound_upper_tmp,
  //     s_bound_upper_remaining_distance_to_destination,
  //     distance_to_destination);

  // Point2D target_parking_space_center;
  // std::vector<planning_math::Vec2d> target_parking_space_coners;
  // bool find_matched_parking_space = find_parking_space(
  //     static_cast<int>(aimed_parking_space_id),
  //     target_parking_space_center, target_parking_space_coners);
  // if (!find_matched_parking_space) {
  //   LOG_DEBUG("Can not find target parking space !!!");
  //   frame_->session()
  //       ->mutable_planning_context()
  //       ->mutable_parking_status_info()
  //       ->set_parking_area_status(
  //           planning::common::ParkingAreaStatus::PARKING_AREA_OUT);
  //   return s_bound_upper;
  // } else {
  //   LOG_DEBUG(
  //       "find target parking space: id= %d, x= %f,y= %f, "
  //       "planning_init_point.frenet_state.s = %f",
  //       aimed_parking_space_id, target_parking_space_center.x,
  //       target_parking_space_center.y, planning_init_point.frenet_state.s);
  // }

  // auto veh_status = frame_->session()->world_model().get_vehicle_status();
  // auto velocity = veh_status.velocity().heading_velocity().value_mps();
  // double dec_expected = config_acc_.s_set_a;
  // static bool entering_parking_area = false;
  // static bool drive_correct_direction = false;
  // auto dbw_status =
  // frame_->session()->world_model().get_vehicle_dbw_status(); double
  // nearest_parking_location_s = 0; auto reference_len =
  // reference_path_ptr_->get_points().back().frenet_point.x; bool
  // target_parking_space_on_right = true; const double
  // nearest_distance_to_parking_area = 4.5; const double
  // confindence_lateral_distance = 5.0; const double
  // confindence_longi_distance = 1.0; Point2D frenet_point;

  // LOG_DEBUG("dbw_status = %d", dbw_status);
  // if (dbw_status == false) {
  //   s_bound_upper_tmp = std::numeric_limits<double>::max();
  //   entering_parking_area = false;
  //   s_bound_upper_remaining_distance_to_destination =
  //       std::numeric_limits<double>::lowest();
  //   drive_correct_direction = false;
  //   frame_->session()
  //       ->mutable_planning_context()
  //       ->mutable_parking_status_info()
  //       ->set_parking_area_status(
  //           planning::common::ParkingAreaStatus::PARKING_AREA_OUT);
  // } else {
  //   if (frenet_coord_->CartCoord2FrenetCoord(
  //           target_parking_space_center, frenet_point) == TRANSFORM_SUCCESS
  //           &&
  //       std::fabs(frenet_point.y) < confindence_lateral_distance) {
  //     if (frenet_point.y > 0.) {
  //       target_parking_space_on_right = false;
  //     }
  //     LOG_DEBUG(
  //         "target_parking_space_center transform frenet success: s = %f, l
  //         =
  //         "
  //         "%f,target_parking_space_on_right=%d",
  //         frenet_point.x, frenet_point.y, target_parking_space_on_right);
  //     nearest_parking_location_s =
  //         frenet_point.x + nearest_distance_to_parking_area +
  //         (vehicle_param_.length / 2. -
  //         vehicle_param_.back_edge_to_center);
  //     if (reference_len < nearest_parking_location_s) {
  //       LOG_DEBUG("reference line length = %f < nearest parking location =
  //       %f",
  //             reference_len, nearest_parking_location_s);
  //       return s_bound_upper;
  //     }
  //     double dis_to_target =
  //         frenet_point.x - planning_init_point.frenet_state.s;
  //     if (!drive_correct_direction) {
  //       if (dis_to_target < confindence_longi_distance && dis_to_target >
  //       0)
  //         drive_correct_direction = true;
  //     }
  //     LOG_DEBUG(
  //         "nearest_parking_location_s = %f, entering_parking_area = %d, "
  //         "drive_correct_direction=%d",
  //         nearest_parking_location_s, entering_parking_area,
  //         drive_correct_direction);

  //     if (!entering_parking_area && drive_correct_direction) {
  //       if (planning_init_point.frenet_state.s >
  //       nearest_parking_location_s)
  //       {
  //         planning_math::Vec2d opening_vec =
  //             target_parking_space_coners[1] -
  //             target_parking_space_coners[0];
  //         if (!target_parking_space_on_right) {
  //           opening_vec = opening_vec * -1.;
  //         }
  //         double yaw =
  //         veh_status.heading_yaw().heading_yaw_data().value_rad();
  //         LOG_DEBUG("opening_vec angle = %f, ego_yaw = %f, yaw_diff_thre =
  //         %f",
  //               opening_vec.Angle(), yaw, (double)10. / 180 * PI);
  //         if (std::fabs(opening_vec.Angle() - yaw) < ((double)10. / 180 *
  //         PI)) {
  //           s_bound_upper_tmp =
  //               std::min((velocity * velocity) / (2 * dec_expected), 6.0);
  //           s_bound_upper_remaining_distance_to_destination =
  //               distance_to_destination - s_bound_upper_tmp;
  //           entering_parking_area = true;
  //           frame_->session()
  //               ->mutable_planning_context()
  //               ->mutable_parking_status_info()
  //               ->set_parking_area_status(
  //                   planning::common::ParkingAreaStatus::PARKING_AREA_IN);
  //           LOG_DEBUG(
  //               "PNP is ready for parking in s_bound_upper_tmp : %f, "
  //               "s_bound_upper_remaining_distance_to_destination : %f",
  //               s_bound_upper_tmp,
  //               s_bound_upper_remaining_distance_to_destination);
  //         }
  //       }
  //     }
  //   } else {
  //     LOG_DEBUG(
  //         "reference line is too short,or target parking space is too far
  //         !!!");
  //   }
  // }

  // if (entering_parking_area) {
  //   NTRACE("PNP has entered parking area");
  // }

  // return s_bound_upper;
  return true;
}

}  // namespace planning
