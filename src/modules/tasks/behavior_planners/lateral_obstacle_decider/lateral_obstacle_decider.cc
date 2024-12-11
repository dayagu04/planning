#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdlib>
#include <memory>
#include <vector>

#include "../../common/planning_gflags.h"
#include "debug_info_log.h"
#include "environment_model_debug_info.pb.h"
#include "environmental_model.h"
#include "lateral_obstacle_decider.h"
#include "planning_context.h"

namespace planning {

LateralObstacleDecider::LateralObstacleDecider(
    const EgoPlanningConfigBuilder *config_builder, framework::Session *session)
    : Task(config_builder, session),
      session_(session),
      config_(config_builder->cast<PotentialAvoidDeciderConfig>()),
      output_(session_->mutable_planning_context()
                  ->mutable_lateral_obstacle_decider_output()
                  .lat_obstacle_decision) {
  VehicleParam vehicle_param =
      VehicleConfigurationContext::Instance()->get_vehicle_param();
  ego_rear_axis_to_front_edge_ = vehicle_param.front_edge_to_rear_axle;
  ego_length_ = vehicle_param.length;
  ego_width_ = vehicle_param.width;
}

bool LateralObstacleDecider::Execute() {
  // get info of ego
  // const auto& reference_path_ptr = session_->planning_context().
  //                     lane_change_decider_output().coarse_planning_info.reference_path;
  const auto target_reference_virtual_id = session_->planning_context()
                                               .lane_change_decider_output()
                                               .fix_lane_virtual_id;
  auto reference_path_ptr =
      session_->environmental_model()
          .get_reference_path_manager()
          ->get_reference_path_by_lane(target_reference_virtual_id, false);
  ego_head_s_ = reference_path_ptr->get_frenet_ego_state().head_s();
  ego_head_l_ = reference_path_ptr->get_frenet_ego_state().head_l();
  ego_v_ = reference_path_ptr->get_frenet_ego_state().velocity();
  ego_v_s_ = reference_path_ptr->get_frenet_ego_state().velocity_s();
  ego_v_l_ = reference_path_ptr->get_frenet_ego_state().velocity_l();

  // lane_width
  const auto &coarse_planning_info = session_->planning_context()
                                         .lane_change_decider_output()
                                         .coarse_planning_info;
  auto lane_width =
      session_->environmental_model()
          .get_virtual_lane_manager()
          ->get_lane_with_virtual_id(coarse_planning_info.target_lane_id)
          ->width();
  // rightest_lane
  const std::shared_ptr<VirtualLaneManager> virtual_lane_manager =
      session_->environmental_model().get_virtual_lane_manager();
  bool rightest_lane;
  if (((virtual_lane_manager->current_lane_virtual_id() ==
        virtual_lane_manager->get_lane_num() - 1) ||
       (virtual_lane_manager->current_lane_virtual_id() ==
            virtual_lane_manager->get_lane_num() - 2 &&
        virtual_lane_manager->get_right_lane() != nullptr &&
        virtual_lane_manager->get_right_lane()->get_lane_type() ==
            iflyauto::LANETYPE_NON_MOTOR)) &&
      virtual_lane_manager->current_lane_virtual_id() - 1 >= 0) {
    rightest_lane = true;
  } else {
    rightest_lane = false;
  }

  // farthest_distance
  auto &last_traj_points =
      session_->planning_context().last_planning_result().traj_points;
  double farthest_distance = DBL_MAX;
  if (!last_traj_points.empty() && last_traj_points.back().frenet_valid) {
    farthest_distance = last_traj_points.back().s - last_traj_points.front().s;
  }

  auto last_fix_lane_id = session_->environmental_model()
                              .get_virtual_lane_manager()
                              ->get_last_fix_lane_id();
  auto current_fix_lane_id = session_->planning_context()
                                 .lane_change_decider_output()
                                 .fix_lane_virtual_id;
  double expand_vel =
      interp(ego_v_, config_.expand_ego_vel, config_.expand_obs_rel_vel);

  // determine is_avd_car
  std::vector<double> avd_car_id;
  for (auto frenet_obs : reference_path_ptr->get_obstacles()) {
    const Obstacle *obs = frenet_obs->obstacle();
    LateralObstacleHistoryInfo &history =
        lateral_obstacle_history_info_[obs->id()];

    // ignore obj without camera source
    if (!(obs->fusion_source() & OBSTACLE_SOURCE_CAMERA) ||
        !frenet_obs->b_frenet_valid() ||
        last_fix_lane_id != current_fix_lane_id) {
      history.is_avd_car = false;
      history.ncar_count = 0;
      history.ncar_count_in = false;
      continue;
    }
    if (frenet_obs->d_s_rel() <= 0) {
      history.is_avd_car = false;
      if (frenet_obs->d_s_rel() <= -1 * (obs->length() + ego_length_)) {
        history.ncar_count = 0;
        history.ncar_count_in = false;
      }
      continue;
    }

    // 判断车辆相对位置，前车，后车，旁车（从前方来的，从后方来的，不知道从哪来的）
    double expand_length = 0.0;
    if (history.side_car &&
        frenet_obs->frenet_relative_velocity_s() < expand_vel) {
      expand_length = 1.5;
    }
    if (frenet_obs->d_s_rel() > expand_length) {
      history.front_car = true;
      history.side_car = false;
      history.rear_car = false;
    } else if (frenet_obs->d_s_rel() >= -ego_length_ &&
               frenet_obs->d_s_rel() <= expand_length) {
      history.side_car = true;
    } else {
      history.front_car = false;
      history.side_car = false;
      history.rear_car = true;
    }

    history.is_avd_car = IsPotentialAvoidingCar(
        *frenet_obs, lane_width, rightest_lane, farthest_distance);

    if (history.is_avd_car) {
      avd_car_id.emplace_back(obs->id());
    }
  }

  // write decider output info
  output_.clear();
  for (auto frenet_obs : reference_path_ptr->get_obstacles()) {
    const Obstacle *obs = frenet_obs->obstacle();
    LateralObstacleHistoryInfo &history =
        lateral_obstacle_history_info_[obs->id()];
    // ignore obj without camera source
    if (!(obs->fusion_source() & OBSTACLE_SOURCE_CAMERA) ||
        !frenet_obs->b_frenet_valid()) {
      continue;
    }

    double expand_length = 0.0;
    if (history.side_car &&
        frenet_obs->frenet_relative_velocity_s() < expand_vel) {
      expand_length = 1.5;
    }

    LateralObstacleDecision(*frenet_obs, lane_width, expand_length);
  }

  JSON_DEBUG_VECTOR("avoid_car_id", avd_car_id, 0);
  return true;
}

bool LateralObstacleDecider::IsPotentialAvoidingCar(
    FrenetObstacle &frenet_obstacle, double lane_width, bool rightest_lane,
    double farthest_distance) {
  LOG_DEBUG("----is_potential_avoiding_car-----\n");
  const Obstacle &obstacle = *frenet_obstacle.obstacle();
  LateralObstacleHistoryInfo &history =
      lateral_obstacle_history_info_[obstacle.id()];
  double near_car_thr = config_.near_car_thr;
  double lat_safety_buffer = config_.lat_safety_buffer;
  double oversize_veh_addition_buffer = config_.oversize_veh_addition_buffer;
  double traffic_cone_thr = config_.traffic_cone_thr;
  double static_obs_buffer = config_.static_obs_buffer;
  double near_car_hysteresis = config_.near_car_hysteresis;
  double in_range_v = config_.in_range_v;
  double in_range_v_hysteresis = config_.in_range_v_hysteresis;
  double potential_near_car_thr = config_.potential_near_car_thr;
  double potential_near_car_v_ub = config_.potential_near_car_v_ub;
  double potential_near_car_v_lb = config_.potential_near_car_v_lb;
  bool enable_static_scene = config_.enable_static_scene;
  double car_addition_decre_buffer = config_.car_addition_decre_buffer;
  double emegency_cutin_ttc_lower = config_.emegency_cutin_ttc_lower;
  double emegency_cutin_ttc_upper = config_.emegency_cutin_ttc_upper;
  double emegency_cutin_front_area = config_.emegency_cutin_front_area;

  double planning_cycle_time = 1.0 / FLAGS_planning_loop_rate;
  bool is_ncar = false;
  double dist_limit;

  const auto &state =
      session_->planning_context().lane_change_decider_output().curr_state;
  const auto lc_request_direction =
      session_->planning_context().lane_change_decider_output().lc_request;
  bool is_LC_LCHANGE =
      ((state == kLaneChangeExecution) || (state == kLaneChangeComplete)) &&
      (lc_request_direction == LEFT_CHANGE);
  bool is_LC_RCHANGE =
      ((state == kLaneChangeExecution) || (state == kLaneChangeComplete)) &&
      (lc_request_direction == RIGHT_CHANGE);

  // calculate info of obstacle
  int id = obstacle.id();
  iflyauto::ObjectType type = obstacle.type();
  double s = frenet_obstacle.frenet_s();
  double l = frenet_obstacle.frenet_l();
  double v_s = frenet_obstacle.frenet_velocity_s();
  double v_l = frenet_obstacle.frenet_velocity_l();
  double v_lat = frenet_obstacle.frenet_velocity_lateral();
  double v_s_rel = frenet_obstacle.frenet_relative_velocity_s();
  double d_s_rel = frenet_obstacle.d_s_rel();
  double d_min_cpath = frenet_obstacle.d_min_cpath();
  double d_max_cpath = frenet_obstacle.d_max_cpath();

  // for Intersection
  if (d_s_rel > farthest_distance + ego_length_ ||
      (d_s_rel > farthest_distance - ego_length_ &&
       ((d_max_cpath < 0 && std::fabs(d_max_cpath) > lane_width * 0.5) ||
        (d_min_cpath > 0 && d_min_cpath > lane_width * 0.5)))) {
    return false;
  }

  std::array<double, 3> xp{20, 40, 60};
  std::array<double, 3> fp{near_car_thr, 0.12, 0.09};
  double near_car_d_lane_thr = interp(d_s_rel, xp, fp);
  // lower buffer for car
  if (type == iflyauto::ObjectType::OBJECT_TYPE_COUPE ||
      type == iflyauto::ObjectType::OBJECT_TYPE_MINIBUS ||
      type == iflyauto::ObjectType::OBJECT_TYPE_VAN ||
      type == iflyauto::ObjectType::OBJECT_TYPE_BUS) {
    // near_car_d_lane_thr = near_car_d_lane_thr * car_addition_decre_factor;
    near_car_d_lane_thr = near_car_d_lane_thr - car_addition_decre_buffer;
  }
  // addition buffer for oversize vehicle
  if (obstacle.is_oversize_vehicle()) {
    std::array<double, 2> vel_xp_oversize_veh{2.7, 5.6};
    std::array<double, 2> vel_fp_oversize_veh{1, 2.5};
    double vel_factor_for_oversize_veh =
        interp(ego_v_, vel_xp_oversize_veh, vel_fp_oversize_veh);
    std::array<double, 3> dis_fp_oversize_veh{1, 1.5, 2};
    double dis_factor_for_oversize_veh =
        interp(d_s_rel, xp, dis_fp_oversize_veh);
    near_car_d_lane_thr = near_car_d_lane_thr * vel_factor_for_oversize_veh *
                          dis_factor_for_oversize_veh;
    lat_safety_buffer += oversize_veh_addition_buffer;
  }

  // hysteresis
  if (history.is_avd_car) {
    near_car_d_lane_thr = near_car_d_lane_thr * near_car_hysteresis;
    in_range_v = in_range_v * in_range_v_hysteresis;
  }

  // addition buffer for VRU
  if (obstacle.is_VRU()) {
    lat_safety_buffer += 0.2;
  }

  // bool is_not_full_in_road = (std::fabs(item.y_rel) > 0.0);
  bool is_not_full_in_road = true;
  bool is_in_range = (d_s_rel < 20.0 && v_s_rel < in_range_v);
  double ttc_for_obs = 3.6;
  // hysteresis
  const auto lat_offset =
      session_->planning_context().lateral_behavior_planner_output().lat_offset;
  if (((lat_offset > 0 || is_LC_RCHANGE) && d_max_cpath < 0) ||
      ((lat_offset < 0 || is_LC_LCHANGE) && d_min_cpath > 0)) {
    if (obstacle.is_oversize_vehicle()) {
      ttc_for_obs = 8.0;
    } else {
      ttc_for_obs = 6.0;
    }
    std::array<double, 2> x1{50, 60};
    std::array<double, 2> f1{0, 0.1};
    double near_car_d_lane_buffer = interp(d_s_rel, x1, f1);
    near_car_d_lane_thr += near_car_d_lane_buffer;
  }
  double max_enter_range = std::fabs(v_s_rel) * ttc_for_obs;
  max_enter_range = max_enter_range > 100 ? 100 : max_enter_range;
  max_enter_range = max_enter_range < 50 ? 50 : max_enter_range;
  bool is_about_to_enter_range =
      (d_s_rel < std::min(std::fabs(10.0 * v_s_rel), max_enter_range) &&
       v_s_rel < -2.5);
  bool cross_solid_line = false;

  // decre lat_safety_buffer
  std::array<double, 3> x_lat_buffer{3.2, 3.5, 3.8};
  std::array<double, 3> f_lat_buffer{0.3, 0.15, 0};
  double decre_buffer_for_lane_width =
      interp(lane_width, x_lat_buffer, f_lat_buffer);
  lat_safety_buffer -= decre_buffer_for_lane_width;

  if (is_not_full_in_road && (is_in_range || is_about_to_enter_range)) {
    if (d_min_cpath != DBL_MAX && d_max_cpath != DBL_MAX) {
      if (!obstacle.is_traffic_facilities()) {
        dist_limit = lane_width * 0.5 + near_car_d_lane_thr;
      } else {
        dist_limit = lane_width * 0.5 - traffic_cone_thr;
      }
      bool is_same_side = ((d_min_cpath > 0 && d_max_cpath > 0) ||
                           (d_min_cpath <= 0 && d_max_cpath <= 0));

      double potential_dist_limit = lane_width * 0.5 + potential_near_car_thr;
      // need avoid flag
      bool is_need_avoid =
          (d_max_cpath < 0 && std::fabs(d_max_cpath) < dist_limit) ||
          (d_min_cpath > 0 && d_min_cpath < dist_limit) ||
          (d_max_cpath < 0 && std::fabs(d_max_cpath) < potential_dist_limit &&
           v_lat < potential_near_car_v_lb &&
           v_lat > potential_near_car_v_ub) ||
          (d_min_cpath > 0 && d_min_cpath < potential_dist_limit &&
           v_lat < potential_near_car_v_lb &&
           v_lat > potential_near_car_v_ub) ||
          (d_max_cpath > 0 && d_min_cpath < 0 && v_s < 0.5);

      double d_max_cpath_recursion = d_max_cpath;
      double d_min_cpath_recursion = d_min_cpath;
      std::array<double, 3> x_v_lat{-0.6, -0.4, -0.2};
      std::array<double, 3> f_times{10, 5, 1};
      double times = interp(v_lat, x_v_lat, f_times);
      if (d_max_cpath < 0) {
        d_max_cpath_recursion = d_max_cpath - v_lat * 0.1 * times;
      } else if (d_min_cpath > 0) {
        d_min_cpath_recursion = d_min_cpath + v_lat * 0.1 * times;
      }

      // can avoid flag
      bool can_avoid =
          (d_min_cpath_recursion >
           std::min((ego_width_ + lat_safety_buffer) - lane_width / 2, 1.8)) ||
          (d_max_cpath_recursion <
           std::max(lane_width / 2 - (ego_width_ + lat_safety_buffer), -1.8)) ||
          ((obstacle.is_static()) &&
           ((d_min_cpath_recursion >
             (ego_width_ + static_obs_buffer) - lane_width / 2) ||
            (d_max_cpath_recursion <
             lane_width / 2 - (ego_width_ + static_obs_buffer))));

      if (is_need_avoid && !can_avoid) {
        history.can_not_avoid = true;
      }

      auto reference_path_ptr = session_->planning_context()
                                    .lane_change_decider_output()
                                    .coarse_planning_info.reference_path;
      ReferencePathPoint refpath_pt{};
      double distance_to_left_road_border = 100;
      double distance_to_right_road_border = 100;
      if (reference_path_ptr != nullptr &&
          reference_path_ptr->get_reference_point_by_lon(s, refpath_pt)) {
        distance_to_left_road_border = refpath_pt.distance_to_left_road_border;
        distance_to_right_road_border =
            refpath_pt.distance_to_right_road_border;
      }

      constexpr double encroach_thr = 0.4;
      bool lane_borrow =
          enable_static_scene && is_need_avoid && can_avoid &&
          (obstacle.is_static()) &&
          ((d_min_cpath > 0 && d_min_cpath < lane_width / 2 - encroach_thr) ||
           (d_max_cpath < 0 &&
            std::fabs(d_max_cpath) < lane_width / 2 - encroach_thr)) &&
          ((d_min_cpath > 0 &&
            d_min_cpath > (ego_width_ + static_obs_buffer) -
                              distance_to_right_road_border) ||
           (d_max_cpath < 0 &&
            d_max_cpath < distance_to_left_road_border -
                              (ego_width_ + static_obs_buffer)));
      history.lane_borrow = lane_borrow;

      is_ncar = (is_same_side && is_need_avoid && can_avoid) ||
                (can_avoid && d_max_cpath > 0 &&
                 d_max_cpath < dist_limit + 2.2 && v_s < 0.5) ||
                (rightest_lane && d_max_cpath < 0 &&
                 std::fabs(d_max_cpath) < dist_limit && v_s < 0.5) ||
                cross_solid_line;
    }
  }

  double ncar_count;
  // for car
  if (obstacle.is_car()) {
    if (d_s_rel >= 20) {
      std::array<double, 2> xp2{-7.5, -2.5};
      std::array<double, 2> fp2{5, 20};
      ncar_count = interp(v_s_rel, xp2, fp2);
    } else {
      std::array<double, 4> xp2{-5, -2.499, 0, 1};
      std::array<double, 4> fp2{2, 3, 4, 20};
      std::array<double, 4> xp3{0, 2, 5, 10};
      std::array<double, 4> fp3{4, 3, 2, 0};

      ncar_count = interp(v_s_rel, xp2, fp2) + interp(ego_v_, xp3, fp3);
    }
    // for VRU
  } else if (obstacle.is_VRU()) {
    std::array<double, 2> xp2{20, 40};
    std::array<double, 2> fp2{5, 10};
    ncar_count = interp(d_s_rel, xp2, fp2);
    // TRAFFIC_BARRIER, TEMPORY_SIGN, FENCE, WATER_SAFETY_BARRIER, CTASH_BARREL
  } else {
    ncar_count = 1.0;
  }
  if (cross_solid_line) {
    std::array<double, 4> xp4{0, 0.4, 0.8};
    std::array<double, 4> fp4{30, 20, 5};
    ncar_count += interp(
        std::min(std::fabs(d_min_cpath), std::fabs(d_max_cpath)), xp4, fp4);
  }

  double gap = (history.last_recv_time == 0.0)
                   ? planning_cycle_time
                   : (obstacle.timestamp() - history.last_recv_time);
  int count = (int)((gap + 0.01) / planning_cycle_time);

  double lat_dis_thr = lane_width - ego_width_ + 0.8;
  bool in_lat_near_area =
      ((d_min_cpath > 0 &&
        d_min_cpath - ego_head_l_ - ego_width_ / 2 < lat_dis_thr) ||
       (d_max_cpath < 0 &&
        ego_head_l_ - d_max_cpath - ego_width_ / 2 < lat_dis_thr));
  // bool in_lon_near_area =
  //     (v_s_rel < 0 &&
  //      ((d_s_rel / (-v_s_rel) < emegency_cutin_ttc_lower) ||
  //       ((d_s_rel / (-v_s_rel) < emegency_cutin_ttc_upper &&
  //         d_s_rel < emegency_cutin_front_area))));
  bool in_lon_near_area = (d_s_rel + v_s * 5) < farthest_distance;

  // cut in/out factor
  std::array<double, 4> x_cut_factor{0.2, 0.4, 0.6, 0.8};
  std::array<double, 4> f_cut_factor{10, 20, 30, 40};
  double cut_factor = interp(std::fabs(v_lat), x_cut_factor, f_cut_factor);

  if (is_ncar) {
    // hack：missing prediction, considering v_lat
    // if (item.trajectory.intersection == 0 &&
    if ((v_lat > -0.3 && v_lat < 0.3) ||
        // 静止的车
        (obstacle.is_car() && std::fabs(v_s) < 0.5 && v_lat > -0.3 &&
         v_lat < 0.3) ||
        // 横向无运动的人或锥桶 || 自车在最右车道
        (!obstacle.is_car() && (std::fabs(v_lat) < 0.3)) || rightest_lane) {
      // hack: always true: 横向无运动的车 || 横向无运动的人或锥桶
      if (((v_lat > -0.3 && v_lat < 0.3 && obstacle.is_car()) ||
           (std::fabs(v_lat) < 0.3 && !obstacle.is_car()))) {
        history.ncar_count =
            std::min(history.ncar_count + gap, 100 * planning_cycle_time);
      }
    } else {
      history.ncar_count = std::max(
          history.ncar_count - cut_factor * count * planning_cycle_time, 0.0);
    }

    if (history.ncar_count < ncar_count * planning_cycle_time &&
        obstacle.timestamp() > history.close_time + 2.5) {
      is_ncar = false;
      return false;
    } else if (history.ncar_count >= ncar_count * planning_cycle_time) {
      if (history.ncar_count_in == false) {
        history.ncar_count = 100 * planning_cycle_time;
      }

      history.close_time = obstacle.timestamp();
      return true;
    }
  } else {
    if (!in_lon_near_area || !in_lat_near_area) {
      history.ncar_count =
          std::max(history.ncar_count - 2 * count * planning_cycle_time, 0.0);
    }

    if (v_s_rel > 1.5) {
      history.ncar_count =
          std::max(history.ncar_count - 5 * count * planning_cycle_time, 0.0);
    }

    // if (item.trajectory.intersection > 0) {
    //   history.ncar_count =
    //     std::max(history.ncar_count - 10 * count * planning_cycle_time, 0.0);
    // }

    // for cut in and cut out
    if (!in_lon_near_area && (v_lat > 0.3 || v_lat < -0.3)) {
      history.ncar_count = std::max(
          history.ncar_count - cut_factor * count * planning_cycle_time, 0.0);
    }

    if (d_max_cpath > 0 && d_min_cpath < 0) {
      history.ncar_count = 0;
    }

    if (history.ncar_count > 60 * planning_cycle_time) {
      history.ncar_count_in = true;
      return true;
    } else {
      history.ncar_count = 0;
      history.ncar_count_in = false;
      return false;
    }
  }

  return false;
}

void LateralObstacleDecider::LateralObstacleDecision(
    FrenetObstacle &frenet_obstacle, double lane_width, double expand_length) {
  const Obstacle &obstacle = *frenet_obstacle.obstacle();
  LateralObstacleHistoryInfo &history =
      lateral_obstacle_history_info_[obstacle.id()];

  // calculate info of obstacle
  int id = obstacle.id();
  iflyauto::ObjectType type = obstacle.type();
  double l = frenet_obstacle.frenet_l();
  double d_s_rel = frenet_obstacle.d_s_rel();
  double v_s_rel = frenet_obstacle.frenet_relative_velocity_s();
  double d_min_cpath = frenet_obstacle.d_min_cpath();
  double d_max_cpath = frenet_obstacle.d_max_cpath();

  double ref_dis = 1;
  double avoid_front_buffer = 0.0;

  bool lat_overlap =
      fabs(ego_head_l_ - l) < (ego_width_ + obstacle.width()) / 2;
  // 前方车辆
  if (history.is_avd_car) {
    if (d_max_cpath > 0 && d_min_cpath < 0) {
      output_[id] = LatObstacleDecisionType::IGNORE;
    } else if (d_max_cpath < 0) {
      output_[id] = LatObstacleDecisionType::LEFT;
    } else if (d_min_cpath > 0) {
      output_[id] = LatObstacleDecisionType::RIGHT;
    }
  } else if (d_s_rel > expand_length) {
    if (obstacle.is_traffic_facilities()) {
      avoid_front_buffer = config_.traffic_cone_thr;
    }
    if (d_max_cpath < 0 &&
        std::fabs(d_max_cpath) > lane_width * 0.5 - avoid_front_buffer) {
      output_[id] = LatObstacleDecisionType::LEFT;
    } else if (d_min_cpath > lane_width * 0.5 - avoid_front_buffer) {
      output_[id] = LatObstacleDecisionType::RIGHT;
    } else {
      output_[id] = LatObstacleDecisionType::IGNORE;
    }
    // 平行车辆
  } else if (d_s_rel <= expand_length && d_s_rel > -ego_length_) {
    if (ego_head_l_ < l) {
      output_[id] = LatObstacleDecisionType::RIGHT;
    } else {
      output_[id] = LatObstacleDecisionType::LEFT;
    }
    // 防止感知误检，同时有横向和纵向overlap
    if (lat_overlap) {
      output_[id] = LatObstacleDecisionType::IGNORE;
    }
    // 后方车辆
  } else {
    if (d_max_cpath < 0 && !lat_overlap && d_max_cpath < -ref_dis) {
      output_[id] = LatObstacleDecisionType::LEFT;
    } else if (d_min_cpath > 0 && !lat_overlap && d_min_cpath > ref_dis) {
      output_[id] = LatObstacleDecisionType::RIGHT;
    } else {
      output_[id] = LatObstacleDecisionType::IGNORE;
    }
  }
  // log
  auto &planning_debug_data = DebugInfoManager::GetInstance().GetDebugInfoPb();
  common::EnvironmentModelInfo *environment_model_debug_info =
      planning_debug_data->mutable_environment_model_info();
  for (size_t i = 0; i < environment_model_debug_info->obstacle_size(); ++i) {
    auto obstacle_new = environment_model_debug_info->mutable_obstacle(i);
    if (obstacle_new->id() == id) {
      obstacle_new->set_lat_decision(static_cast<uint32_t>(output_[id]));
    } else {
      continue;
    }
  }
}

}  // namespace planning