#include "lateral_obstacle_decider.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <memory>
#include <vector>

#include "../../common/planning_gflags.h"
#include "debug_info_log.h"
#include "environment_model_debug_info.pb.h"
#include "environmental_model.h"
#include "planning_context.h"
#include "edt_manager.h"
#include "task_interface/lateral_obstacle_decider_output.h"

namespace planning {

LateralObstacleDecider::LateralObstacleDecider(
    const EgoPlanningConfigBuilder *config_builder, framework::Session *session)
    : Task(config_builder, session),
      session_(session),
      config_(config_builder->cast<LateralObstacleDeciderConfig>()),
      search_result_(session_->mutable_planning_context()
                               ->mutable_lateral_obstacle_decider_output()
                               .search_result),
      left_borrow_(session_->mutable_planning_context()
              ->mutable_lateral_obstacle_decider_output()
              .left_borrow),
      right_borrow_(session_->mutable_planning_context()
              ->mutable_lateral_obstacle_decider_output()
              .right_borrow),
      in_intersection_(session_->mutable_planning_context()
              ->mutable_lateral_obstacle_decider_output()
              .in_intersection),
      lateral_obstacle_history_info_(
          session_->mutable_planning_context()
              ->mutable_lateral_obstacle_decider_output()
              .lateral_obstacle_history_info),
      output_(session_->mutable_planning_context()
                  ->mutable_lateral_obstacle_decider_output()
                  .lat_obstacle_decision) {
  VehicleParam vehicle_param =
      VehicleConfigurationContext::Instance()->get_vehicle_param();
  ego_rear_axis_to_front_edge_ = vehicle_param.front_edge_to_rear_axle;
  ego_length_ = vehicle_param.length;
  ego_width_ = vehicle_param.width;
  name_ = "LateralObstacleDecider";
  hybrid_ara_star_ = std::make_unique<HybridARAStar>(session);
  ego_rear_edge_to_rear_axle_ = vehicle_param.rear_edge_to_rear_axle;
}

bool LateralObstacleDecider::Execute() {
  if (!PreCheck()) {
    output_.clear();
    LOG_DEBUG("PreCheck failed\n");
    return false;
  }

  UpdateLaneBorrowDirection();

  UpdateIntersection();

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

  if (session_->is_hpp_scene()) {
    const auto &reference_path_ptr = session_->planning_context()
                                         .lane_change_decider_output()
                                         .coarse_planning_info.reference_path;
    DebugInfoManager::GetInstance()
        .GetDebugInfoPb()
        ->mutable_hybrid_ara_info()
        ->Clear();

    bool enable_search = CheckEnableSearch(reference_path_ptr, search_result_);

    auto time1 = IflyTime::Now_ms();
    if (enable_search) {
      if (ARAStar()) {
        search_result_ = SearchResult::SUCCESS;
      } else {
        search_result_ = SearchResult::FAILED;
      };
    } else {
      search_result_ = SearchResult::NO_SEARCH;
    }
    auto time2 = IflyTime::Now_ms();
    JSON_DEBUG_VALUE("ARAStarTime", time2 - time1);

    if (search_result_ != SearchResult::SUCCESS) {
      UpdateLatDecision(reference_path_ptr);
    } else {
      UpdateLatDecisionWithARAStar(reference_path_ptr);
    }

    Log(reference_path_ptr);

  } else {
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
      farthest_distance =
          last_traj_points.back().s - last_traj_points.front().s;
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
    std::vector<double> maintain_avoid;
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

      // 判断车辆相对位置，前车，后车，旁车（从前方来的，从后方来的，不知道从哪来的）
      history.front_expand_len = 0.0;
      history.rear_expand_len = 0.0;
      if (history.side_car &&
            frenet_obs->frenet_relative_velocity_s() < expand_vel) {
          history.front_expand_len = 1.5;
      }
      if (in_intersection_ && history.side_car &&
          frenet_obs->frenet_relative_velocity_s() > -2) {
        history.rear_expand_len = 1.5;
      }
      if (frenet_obs->d_s_rel() > history.front_expand_len) {
        history.front_car = true;
        history.side_car = false;
        history.rear_car = false;
      } else if (frenet_obs->d_s_rel() >=
                   -(ego_length_ + history.rear_expand_len) &&
                 frenet_obs->d_s_rel() <= history.front_expand_len) {
        if (CheckSideObstacle(reference_path_ptr, *frenet_obs)) {
          history.side_car = true;
        } else {
          history.side_car = false;
        }
      } else {
        history.front_car = false;
        history.side_car = false;
        history.rear_car = true;
      }
      if (CalculateCutInAndCross(*frenet_obs, reference_path_ptr, lane_width)) {
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
      history.is_avd_car = IsPotentialAvoidingCar(
          *frenet_obs, lane_width, rightest_lane, farthest_distance,
          left_borrow_, right_borrow_);
      history.last_recv_time = obs->timestamp();
    }

    // write decider output info
    last_output_ = output_;
    output_.clear();
    for (auto frenet_obs : reference_path_ptr->get_obstacles()) {
      const Obstacle *obs = frenet_obs->obstacle();
      LateralObstacleHistoryInfo &history =
          lateral_obstacle_history_info_[obs->id()];
      // ignore obj without camera source
      if (!(obs->fusion_source() & OBSTACLE_SOURCE_CAMERA) ||
          !frenet_obs->b_frenet_valid() ||
          frenet_obs->b_frenet_polygon_sequence_invalid()) {
        continue;
      }
      LateralObstacleDecision(*frenet_obs, lane_width);

      if (history.last_is_avd_car && !history.is_avd_car) {
        HoldLatOffset(*frenet_obs);
      }

      history.last_is_avd_car = history.is_avd_car;

      if (history.is_avd_car) {
        avd_car_id.emplace_back(obs->id());
      }
      if (history.maintain_avoid) {
        maintain_avoid.emplace_back(obs->id());
      }
    }

    JSON_DEBUG_VECTOR("maintain_avoid", maintain_avoid, 0);
    JSON_DEBUG_VECTOR("avoid_car_id", avd_car_id, 0);
    JSON_DEBUG_VALUE("can_left_borrow", left_borrow_);
    JSON_DEBUG_VALUE("can_right_borrow", right_borrow_);
  }
  return true;
}

void LateralObstacleDecider::HoldLatOffset(FrenetObstacle &frenet_obstacle) {
  const Obstacle &obstacle = *frenet_obstacle.obstacle();
  LateralObstacleHistoryInfo &history =
      lateral_obstacle_history_info_[obstacle.id()];
  const auto lat_obs_decision_iter =
      output_.find(obstacle.id());
  if (lat_obs_decision_iter == output_.end()){
    return;
  }
  double v_s_rel = frenet_obstacle.frenet_relative_velocity_s();
  double d_s_rel = frenet_obstacle.d_s_rel();
  bool maintain_avoid =
      (output_[obstacle.id()] == LatObstacleDecisionType::IGNORE &&
       !history.rear_car && v_s_rel < 0 &&
       ((d_s_rel / (-v_s_rel) < config_.emegency_cutin_ttc_lower) ||
        ((d_s_rel / (-v_s_rel) < config_.emegency_cutin_ttc_upper &&
          d_s_rel < config_.emegency_cutin_front_area))));
  if (maintain_avoid) {
    history.is_avd_car = true;
    history.maintain_avoid = true;
  } else {
    history.maintain_avoid = false;
  }
}

bool LateralObstacleDecider::IsPotentialAvoidingCar(
    FrenetObstacle &frenet_obstacle, double lane_width, bool rightest_lane,
    double farthest_distance, bool can_left_borrow, bool can_right_borrow) {
  LOG_DEBUG("----is_potential_avoiding_car-----\n");
  const Obstacle &obstacle = *frenet_obstacle.obstacle();
  LateralObstacleHistoryInfo &history =
      lateral_obstacle_history_info_[obstacle.id()];
  double near_car_thr = config_.near_car_thr;
  double lat_safety_buffer = config_.lat_safety_buffer;
  double oversize_veh_addition_buffer = config_.oversize_veh_addition_buffer;
  double traffic_cone_thr = config_.traffic_cone_thr;
  double static_obs_buffer = config_.small_static_obs_buffer;
  double near_car_hysteresis = config_.near_car_hysteresis;
  double in_range_v = config_.in_range_v;
  double in_range_v_hysteresis = config_.in_range_v_hysteresis;
  double potential_near_car_thr = config_.potential_near_car_thr;
  double potential_near_car_v_ub = config_.potential_near_car_v_ub;
  double potential_near_car_v_lb = config_.potential_near_car_v_lb;
  bool enable_static_scene = config_.enable_static_scene;
  double car_addition_decre_buffer = config_.car_addition_decre_buffer;

  double planning_cycle_time = 1.0 / FLAGS_planning_loop_rate;
  bool is_ncar = false;
  double dist_limit;
  // TODO(zkxie): 暂时关闭
  bool borrow_bicycle_lane = false;
  bool cross_solid_line = false;

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
  double v_lat = frenet_obstacle.frenet_velocity_lateral(); // 根据历史差分
  double v_s_rel = frenet_obstacle.frenet_relative_velocity_s();
  double d_s_rel = frenet_obstacle.d_s_rel();
  double d_min_cpath = frenet_obstacle.d_min_cpath();
  double d_max_cpath = frenet_obstacle.d_max_cpath();

  // static obs lat_safety_buffer
  if ((l > 0 && can_right_borrow) || (l < 0 && can_left_borrow)) {
    static_obs_buffer = config_.large_static_obs_buffer;
  }

  std::array<double, 3> xp{20, 40, 60};
  std::array<double, 3> fp{near_car_thr, 0.12, 0.09};
  double near_car_d_lane_thr = interp(d_s_rel, xp, fp);
  // lower buffer for car
  if (!obstacle.is_static() && (type == iflyauto::ObjectType::OBJECT_TYPE_COUPE ||
      type == iflyauto::ObjectType::OBJECT_TYPE_MINIBUS ||
      type == iflyauto::ObjectType::OBJECT_TYPE_VAN ||
      type == iflyauto::ObjectType::OBJECT_TYPE_BUS)) {
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

  // 减去大车后视镜
  if (obstacle.is_oversize_vehicle()) {
    lat_safety_buffer -= 0.2;
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

  // decre lat_safety_buffer
  std::array<double, 3> x_lat_buffer{3.2, 3.5, 3.8};
  std::array<double, 3> f_lat_buffer{0.3, 0.15, 0};
  double decre_buffer_for_lane_width =
      interp(lane_width, x_lat_buffer, f_lat_buffer);
  lat_safety_buffer -= decre_buffer_for_lane_width;

  double lat_safety_buffer_for_lateral_obstacle_decision = 0;
  double static_obs_buffer_for_lateral_obstacle_decision = 0;
  bool lateral_obstacle_decision_can_avoid = false;
  if (history.can_avoid) {
    lat_safety_buffer_for_lateral_obstacle_decision = lat_safety_buffer - 0.1;
    static_obs_buffer_for_lateral_obstacle_decision = static_obs_buffer - 0.1;
  } else {
    lat_safety_buffer_for_lateral_obstacle_decision = lat_safety_buffer;
    static_obs_buffer_for_lateral_obstacle_decision = static_obs_buffer;
  }

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
          (borrow_bicycle_lane && d_max_cpath > 0 && d_min_cpath < 0 &&
           v_s < 0.5);

      double d_max_cpath_updated = d_max_cpath;
      double d_min_cpath_updated= d_min_cpath;
      std::array<double, 3> x_v_lat{-0.6, -0.4, -0.2};
      std::array<double, 3> f_times{10, 5, 1};
      double times = interp(v_lat, x_v_lat, f_times);
      if (d_max_cpath < 0) {
        d_max_cpath_updated = d_max_cpath - v_lat * 0.1 * times;
      } else if (d_min_cpath > 0) {
        d_min_cpath_updated= d_min_cpath + v_lat * 0.1 * times;
      }

      // can avoid flag
      bool can_avoid =
          (d_min_cpath_updated>
           std::min((ego_width_ + lat_safety_buffer) - lane_width / 2, 1.8)) ||
          (d_max_cpath_updated <
           std::max(lane_width / 2 - (ego_width_ + lat_safety_buffer), -1.8)) ||
          ((obstacle.is_static()) &&
           ((d_min_cpath_updated>
             (ego_width_ + static_obs_buffer) - lane_width / 2) ||
            (d_max_cpath_updated <
             lane_width / 2 - (ego_width_ + static_obs_buffer))));

      // can avoid flag for LateralObstacleDecision
      lateral_obstacle_decision_can_avoid =
          (d_min_cpath_updated>
           std::min((ego_width_ + lat_safety_buffer_for_lateral_obstacle_decision) - lane_width / 2, 1.8)) ||
          (d_max_cpath_updated <
           std::max(lane_width / 2 - (ego_width_ + lat_safety_buffer_for_lateral_obstacle_decision), -1.8)) ||
          ((obstacle.is_static()) &&
           ((d_min_cpath_updated>
             (ego_width_ + static_obs_buffer_for_lateral_obstacle_decision) - lane_width / 2) ||
            (d_max_cpath_updated <
             lane_width / 2 - (ego_width_ + static_obs_buffer_for_lateral_obstacle_decision))));

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
                (can_avoid && borrow_bicycle_lane && d_max_cpath > 0 &&
                 d_max_cpath < dist_limit + 2.2 && v_s < 0.5) ||
                // (rightest_lane && d_max_cpath < 0 &&
                //  std::fabs(d_max_cpath) < dist_limit && v_s < 0.5) ||
                cross_solid_line;
    }
  }
  int max_can_avoid_count = 6;
  int can_avoid_count_thr = 2;
  if (lateral_obstacle_decision_can_avoid) {
    // 根据侵入的距离，动态调整计数
    double intrusion_distance = DBL_MAX;
    if (d_max_cpath < 0) {
      intrusion_distance =  lane_width * 0.5 - std::fabs(d_max_cpath);
    } else if (d_min_cpath > 0) {
      intrusion_distance =  lane_width * 0.5 - d_min_cpath;
    }
    std::array<double, 3> intrusion_distance_xp{0.1, 0.2, 0.3};
    std::array<double, 3> can_avoid_count_fp{3.1, 2.1, 1.1};
    int can_avoid_count =
      static_cast<int>(interp(intrusion_distance, intrusion_distance_xp, can_avoid_count_fp));
    history.can_avoid_count = std::min(history.can_avoid_count + can_avoid_count, max_can_avoid_count);
  } else {
    // 如果当前帧不能避让，则不避让
    history.can_avoid_count = 0;
  }
  history.can_avoid = history.can_avoid_count > can_avoid_count_thr;

  // for Intersection
  if (d_s_rel > farthest_distance + ego_length_ ||
      (d_s_rel > farthest_distance - ego_length_ &&
       ((d_max_cpath < 0 && std::fabs(d_max_cpath) > lane_width * 0.5) ||
        (d_min_cpath > 0 && d_min_cpath > lane_width * 0.5)))) {
    return false;
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
    if (std::fabs(v_lat) > 0.3) {
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
    FrenetObstacle &frenet_obstacle, double lane_width) {
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
  } else if (d_s_rel > history.front_expand_len) {
    if (history.can_avoid) {
      const bool last_was_avoid =
          last_output_.find(id) != last_output_.end() &&
          (last_output_[id] == LatObstacleDecisionType::LEFT ||
          last_output_[id] == LatObstacleDecisionType::RIGHT);
      if (last_was_avoid) {
        avoid_front_buffer = config_.avoid_persistence_front_buffer;
      }
      if (obstacle.is_traffic_facilities()) {
        avoid_front_buffer += config_.traffic_cone_thr;
      }
      if (d_max_cpath < 0 &&
          std::fabs(d_max_cpath) > lane_width * 0.5 - avoid_front_buffer) {
        output_[id] = LatObstacleDecisionType::LEFT;
      } else if (d_min_cpath > lane_width * 0.5 - avoid_front_buffer) {
        output_[id] = LatObstacleDecisionType::RIGHT;
      } else {
        output_[id] = LatObstacleDecisionType::IGNORE;
      }
    } else {
      output_[id] = LatObstacleDecisionType::IGNORE;
    }
    // 平行车辆
  } else if (d_s_rel <= history.front_expand_len &&
             d_s_rel > -(ego_length_ + history.rear_expand_len)) {
    if (ego_head_l_ < l) {
      output_[id] = LatObstacleDecisionType::RIGHT;
    } else {
      output_[id] = LatObstacleDecisionType::LEFT;
    }
    // 防止感知误检，同时有横向和纵向overlap
    if (!in_intersection_ && lat_overlap) {
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

  // cut_in 或 横穿
  if (history.cut_in_or_cross) {
    output_[id] = LatObstacleDecisionType::IGNORE;
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

bool LateralObstacleDecider::CalculateCutInAndCross(
    FrenetObstacle &frenet_obstacle,
    std::shared_ptr<ReferencePath> reference_path, double lane_width) {
  LateralObstacleHistoryInfo &history =
      lateral_obstacle_history_info_[frenet_obstacle.id()];
  if (!frenet_obstacle.obstacle()->is_static() &&
      !(frenet_obstacle.d_s_rel() <= 0 && in_intersection_) &&
      frenet_obstacle.obstacle()->trajectory_valid()) {
    double lat_safety_buffer = config_.lat_safety_buffer;
    // addition buffer for VRU
    if (frenet_obstacle.obstacle()->is_VRU()) {
      lat_safety_buffer += 0.2;
    }
    // decre lat_safety_buffer in narrow lane
    std::array<double, 3> x_lat_buffer{3.2, 3.5, 3.8};
    std::array<double, 3> f_lat_buffer{0.3, 0.15, 0};
    double decre_buffer_for_lane_width =
        interp(lane_width, x_lat_buffer, f_lat_buffer);
    lat_safety_buffer -= decre_buffer_for_lane_width;

    // 减去大车后视镜
    if (frenet_obstacle.obstacle()->is_oversize_vehicle()) {
      lat_safety_buffer -= 0.2;
    }

    // 滞回
    if (history.cut_in_or_cross) {
      lat_safety_buffer += 0.1;
    }
    auto &frenet_coord = reference_path->get_frenet_coord();
    const double kLThreshold =
        ego_width_ + lat_safety_buffer - 0.5 * lane_width;
    constexpr double kAdditionL = 0.15;
    constexpr double kVelLThreshold = 0.15;
    std::array<uint8_t, 6> timestamps{5, 4, 3, 2, 1, 0};
    double min_l = std::numeric_limits<double>::max();
    double max_l = std::numeric_limits<double>::lowest();
    for (auto &i : timestamps) {
      auto enu_polygon = frenet_obstacle.obstacle()->get_polygon_at_point(
          frenet_obstacle.obstacle()->get_point_at_time(i));
      for (auto &pt : enu_polygon.points()) {
        Point2D frenet_point, carte_point;
        carte_point.x = pt.x();
        carte_point.y = pt.y();
        if (frenet_coord->XYToSL(carte_point, frenet_point)) {
          min_l = std::min(min_l, frenet_point.y);
          max_l = std::max(max_l, frenet_point.y);
        }
      }
    }

    double extreme_l = (frenet_obstacle.frenet_l() > 0) ? min_l : max_l;
    if ((frenet_obstacle.frenet_l() > 0 &&
         extreme_l < kLThreshold - kAdditionL) ||
        (frenet_obstacle.frenet_l() < 0 &&
         extreme_l > -(kLThreshold - kAdditionL))) {
      history.cut_in_or_cross = true;
      history.cut_in_or_cross_count = 5;
      return true;
    } else if (std::abs(frenet_obstacle.frenet_velocity_l()) < kVelLThreshold &&
               ((frenet_obstacle.frenet_l() > 0 && extreme_l < kLThreshold) ||
                (frenet_obstacle.frenet_l() < 0 && extreme_l > -kLThreshold))) {
      history.cut_in_or_cross_count += 1;
      history.cut_in_or_cross_count = std::min(history.cut_in_or_cross_count, 5);
    } else {
      history.cut_in_or_cross_count -= 1;
      history.cut_in_or_cross_count = std::max(history.cut_in_or_cross_count, 0);
    }

    if (history.cut_in_or_cross_count > 2) {
      history.cut_in_or_cross = true;
      return true;
    } else {
      history.cut_in_or_cross = false;
      return false;
    }
  }
  history.cut_in_or_cross = false;
  return false;
}

void LateralObstacleDecider::UpdateIntersection() {
  const auto intersection_state = session_->environmental_model()
                              .get_virtual_lane_manager()
                              ->GetIntersectionState();
  const double distance_to_stopline = session_->environmental_model()
                                    .get_virtual_lane_manager()
                                    ->GetEgoDistanceToStopline();
  bool current_intersection_state =
      intersection_state == common::IntersectionState::IN_INTERSECTION ||
      intersection_state == common::IntersectionState::OFF_INTERSECTION ||
      (intersection_state == common::IntersectionState::APPROACH_INTERSECTION &&
      (distance_to_stopline > 500 || distance_to_stopline < 0));
  if (current_intersection_state) {
    intersection_count_ = 2;
  } else {
    intersection_count_ = std::max(intersection_count_ - 1, 0);
  }
  in_intersection_ = intersection_count_ > 0;
}

void LateralObstacleDecider::UpdateLaneBorrowDirection() {
  left_borrow_ = true;
  right_borrow_ = true;
  const auto& virtual_lane_manager =
      session_->environmental_model().get_virtual_lane_manager();
  const auto current_lane_ptr = virtual_lane_manager->get_current_lane();
  const auto left_lane_ptr = virtual_lane_manager->get_left_lane();
  const auto right_lane_ptr = virtual_lane_manager->get_right_lane();
  double lane_line_length = 0.0;
  const auto& left_lane_boundarys = current_lane_ptr->get_left_lane_boundary();
  const auto& right_lane_boundarys =
      current_lane_ptr->get_right_lane_boundary();
  const auto ego_frenet_boundary = session_->environmental_model()
                             .get_reference_path_manager()
                             ->get_reference_path_by_current_lane()
                             ->get_ego_frenet_boundary();
  iflyauto::LaneBoundaryType left_lane_boundary_type;
  iflyauto::LaneBoundaryType right_lane_boundary_type;
  // # Accumulate lane segment lengths.
  // Record current segment type and break loop when exceeding vehicle
  // wheelbase.
  const auto& lane_points = current_lane_ptr->lane_points();
  for (int i = 0; i < lane_points.size(); i++) {
    lane_line_length = lane_points[i].s;
    if (lane_line_length > ego_frenet_boundary.s_end) {
      left_lane_boundary_type = lane_points[i].left_lane_border_type;
      right_lane_boundary_type = lane_points[i].right_lane_border_type;
      break;
    }
  }
  // If the lane marking is not left dashed/right solid or double dashed, return
  // False.
  if (left_lane_boundary_type != iflyauto::LaneBoundaryType_MARKING_DASHED &&
      left_lane_boundary_type !=
          iflyauto::LaneBoundaryType_MARKING_LEFT_SOLID_RIGHT_DASHED &&
      left_lane_boundary_type !=
          iflyauto::LaneBoundaryType_MARKING_DOUBLE_DASHED) {
    left_borrow_ = false;
  }
  if (left_lane_ptr == nullptr) {
    left_borrow_ = false;
  }

  // todo: if left lane is reverse, then left_boorow is false
  if (right_lane_boundary_type != iflyauto::LaneBoundaryType_MARKING_DASHED &&
      right_lane_boundary_type !=
          iflyauto::LaneBoundaryType_MARKING_DOUBLE_DASHED) {
    right_borrow_ = false;
  }
  if (right_lane_ptr == nullptr) {
    right_borrow_ = false;
  }
}

bool LateralObstacleDecider::CheckEnableSearch(
    const std::shared_ptr<ReferencePath> &reference_path_ptr,
    const SearchResult search_result) {
  const auto ego_s = reference_path_ptr->get_frenet_ego_state().s();
  if (config_.enable_hybrid_ara) {
    for (auto &obstacle : reference_path_ptr->get_obstacles()) {
      if (obstacle->b_frenet_valid() &&
          !obstacle->b_frenet_polygon_sequence_invalid() &&
          obstacle->frenet_polygon_sequence()[0].second.max_x() >
              ego_s - ego_rear_edge_to_rear_axle_ &&
          obstacle->frenet_polygon_sequence()[0].second.min_x() - ego_s <
              config_.hybrid_ara_s_range) {
        auto min_abs_l = std::min(
            std::fabs(obstacle->frenet_polygon_sequence()[0].second.min_y()),
            std::fabs(obstacle->frenet_polygon_sequence()[0].second.max_y()));
        double l_threshold = 0.0;
        if (obstacle->type() ==
                iflyauto::ObjectType::OBJECT_TYPE_OCC_GROUDING_WIRE ||
            obstacle->type() == iflyauto::ObjectType::OBJECT_TYPE_COLUMN) {
          l_threshold = config_.column_static_buffer_for_search;
        } else {
          l_threshold = config_.static_buffer_for_search;
        }
        if (search_result == SearchResult::SUCCESS) {
          l_threshold += 0.3;
        }
        if (min_abs_l < l_threshold) {
          return true;
        }
      }
    }
  }
  return false;
}

bool LateralObstacleDecider::ARAStar() {
  auto &hybrid_ara_result = session_->mutable_planning_context()
                                ->mutable_lateral_obstacle_decider_output()
                                .hybrid_ara_result;
  hybrid_ara_result.Clear();
  bool find_path = hybrid_ara_star_->Plan(hybrid_ara_result, search_result_);

  // log
  auto &planning_debug_data = DebugInfoManager::GetInstance().GetDebugInfoPb();
  auto hybrid_ara_path =
      planning_debug_data->mutable_hybrid_ara_info()->mutable_hybrid_ara_path();
  auto hybrid_ara_path_cost = planning_debug_data->mutable_hybrid_ara_info()
                                  ->mutable_hybrid_ara_path_cost();
  hybrid_ara_path_cost->Clear();
  for (const auto x : hybrid_ara_result.x) {
    hybrid_ara_path->add_x(x);
  }
  for (const auto y : hybrid_ara_result.y) {
    hybrid_ara_path->add_y(y);
  }
  for (const auto phi : hybrid_ara_result.phi) {
    hybrid_ara_path->add_phi(phi);
  }
  for (const auto s : hybrid_ara_result.s) {
    hybrid_ara_path->add_s(s);
  }
  for (const auto l : hybrid_ara_result.l) {
    hybrid_ara_path->add_l(l);
  }

  return (find_path && hybrid_ara_result.Valid());
}

void LateralObstacleDecider::UpdateLatDecision(
    const std::shared_ptr<ReferencePath> &reference_path_ptr) {
  const auto &reference_path = session_->planning_context()
                                   .lane_change_decider_output()
                                   .coarse_planning_info.reference_path;
  auto ego_l_max = reference_path->get_frenet_ego_state().boundary().l_end;
  auto ego_l_min = reference_path->get_frenet_ego_state().boundary().l_start;
  auto &vehicle_param =
      VehicleConfigurationContext::Instance()->get_vehicle_param();
  auto ego_width = vehicle_param.width;
  auto &lat_obstacle_decision = session_->mutable_planning_context()
                                    ->mutable_lateral_obstacle_decider_output()
                                    .lat_obstacle_decision;
  lat_obstacle_decision.clear();
  constexpr double kNearFrontThreshold = 7;
  constexpr double kHeadLBuffer = 0.5;
  for (auto &obstacle : reference_path_ptr->get_obstacles()) {
    if (obstacle->b_frenet_valid() &&
        !obstacle->b_frenet_polygon_sequence_invalid()) {
      const double obstacle_l_start =
          obstacle->frenet_polygon_sequence()[0].second.min_y();
      const double obstacle_l_end =
          obstacle->frenet_polygon_sequence()[0].second.max_y();
      const double obstacle_s_start =
          obstacle->frenet_polygon_sequence()[0].second.min_x();
      const double obstacle_s_end =
          obstacle->frenet_polygon_sequence()[0].second.max_x();
      if (EdtManager::FilterObstacleForAra(*obstacle)) {
        double l_buffer = 0;
        if (obstacle->obstacle()->type() ==
                iflyauto::ObjectType::OBJECT_TYPE_COLUMN ||
            obstacle->obstacle()->type() ==
                iflyauto::ObjectType::OBJECT_TYPE_OCC_GROUDING_WIRE) {
          l_buffer = config_.column_l_buffer_for_decision;
        } else {
          l_buffer = config_.l_buffer_for_lat_decision;
        }

        const double ego_head_l_start = ego_head_l_ - ego_width_ / 2;
        const double ego_head_l_end = ego_head_l_ + ego_width_ / 2;
        const double ego_s_start =
            reference_path_ptr->get_frenet_ego_state().boundary().s_start;
        const double ego_s_end =
            reference_path_ptr->get_frenet_ego_state().boundary().s_end;
        double start_s = std::max(ego_s_start, obstacle_s_start);
        double end_s = std::min(ego_s_end, obstacle_s_end);
        bool lon_overlap = start_s < end_s;

        // 平行车辆
        if (lon_overlap) {
          ego_head_l_ = reference_path_ptr->get_frenet_ego_state().head_l();
          double ego_l = reference_path_ptr->get_frenet_ego_state().l();
          const double ego_l_start = ego_l - ego_width_ / 2;
          const double ego_l_end = ego_l + ego_width_ / 2;
          double start_l = std::max(ego_l_start, obstacle_l_start);
          double end_l = std::min(ego_l_end, obstacle_l_end);
          double start_head_l = std::max(ego_head_l_start, obstacle_l_start);
          double end_head_l = std::min(ego_head_l_end, obstacle_l_end);
          constexpr double kLatOverlapBuffer = 0.25;
          bool lat_overlap = (start_l < end_l - kLatOverlapBuffer) &&
                             (start_head_l < end_head_l - kLatOverlapBuffer);

          if (ego_s_end > obstacle_s_end) {
            if (ego_l < obstacle->frenet_l()) {
              lat_obstacle_decision[obstacle->id()] =
                  LatObstacleDecisionType::RIGHT;
            } else {
              lat_obstacle_decision[obstacle->id()] =
                  LatObstacleDecisionType::LEFT;
            }
          } else {
            if (ego_head_l_ < obstacle->frenet_l()) {
              lat_obstacle_decision[obstacle->id()] =
                  LatObstacleDecisionType::RIGHT;
            } else {
              lat_obstacle_decision[obstacle->id()] =
                  LatObstacleDecisionType::LEFT;
            }
          }
          // 防止感知误检，同时有横向和纵向overlap
          if (lat_overlap) {
            lat_obstacle_decision[obstacle->id()] =
                LatObstacleDecisionType::IGNORE;
          }
        } else {
          if (obstacle->frenet_l() > 0) {
            if (obstacle_l_start > -l_buffer) {
              lat_obstacle_decision[obstacle->id()] =
                  LatObstacleDecisionType::RIGHT;
            } else {
              lat_obstacle_decision[obstacle->id()] =
                  LatObstacleDecisionType::IGNORE;
            }
          } else {
            if (obstacle_l_end < l_buffer) {
              lat_obstacle_decision[obstacle->id()] =
                  LatObstacleDecisionType::LEFT;
            } else {
              lat_obstacle_decision[obstacle->id()] =
                  LatObstacleDecisionType::IGNORE;
            }
          }
          if (obstacle_s_start > ego_head_s_ &&
              obstacle_s_start < ego_head_s_ + kNearFrontThreshold) {
            if (ego_head_l_start > obstacle_l_end - kHeadLBuffer) {
              lat_obstacle_decision[obstacle->id()] =
                  LatObstacleDecisionType::LEFT;
            } else if (ego_head_l_end < obstacle_l_start + kHeadLBuffer) {
              lat_obstacle_decision[obstacle->id()] =
                  LatObstacleDecisionType::RIGHT;
            }
          }
        }
      } else {
        lat_obstacle_decision[obstacle->id()] = LatObstacleDecisionType::IGNORE;
      }
    } else {
      lat_obstacle_decision[obstacle->id()] = LatObstacleDecisionType::IGNORE;
    }
  }
}

void LateralObstacleDecider::UpdateLatDecisionWithARAStar(
    const std::shared_ptr<ReferencePath> &reference_path_ptr) {
  auto &hybrid_ara_result = session_->mutable_planning_context()
                                ->mutable_lateral_obstacle_decider_output()
                                .hybrid_ara_result;
  auto traj_size = hybrid_ara_result.x.size();
  std::vector<double> s_vec(traj_size);
  std::vector<double> l_vec(traj_size);
  double angle_offset = 0.0;
  bool behind_equal_l_point = true;
  for (size_t i = 0; i < traj_size; ++i) {
    s_vec[i] = hybrid_ara_result.s[i];
    l_vec[i] = hybrid_ara_result.l[i];
  }
  pnc::mathlib::spline l_s_spline;
  l_s_spline.set_points(s_vec, l_vec, pnc::mathlib::spline::linear);

  auto &lat_obstacle_decision = session_->mutable_planning_context()
                                    ->mutable_lateral_obstacle_decider_output()
                                    .lat_obstacle_decision;
  lat_obstacle_decision.clear();
  for (auto &obstacle : reference_path_ptr->get_obstacles()) {
    if (obstacle->b_frenet_valid() &&
        !obstacle->b_frenet_polygon_sequence_invalid()) {
      if (EdtManager::FilterObstacleForAra(*obstacle)) {
        double l_ara = 0;
        if (obstacle->frenet_s() < s_vec.front()) {
          l_ara = l_vec.front();
        } else if (obstacle->frenet_s() > s_vec.back()) {
          l_ara = l_vec.back();
        } else {
          l_ara = l_s_spline(obstacle->frenet_s());
        }
        if (obstacle->frenet_l() > l_ara) {
          lat_obstacle_decision[obstacle->id()] =
              LatObstacleDecisionType::RIGHT;
        } else {
          lat_obstacle_decision[obstacle->id()] = LatObstacleDecisionType::LEFT;
        }
      } else {
        lat_obstacle_decision[obstacle->id()] = LatObstacleDecisionType::IGNORE;
      }
    } else {
      lat_obstacle_decision[obstacle->id()] = LatObstacleDecisionType::IGNORE;
    }
  }
}

void LateralObstacleDecider::Log(
    const std::shared_ptr<ReferencePath> &reference_path_ptr) {
  auto &planning_debug_data = DebugInfoManager::GetInstance().GetDebugInfoPb();
  auto environment_model_debug_info =
      planning_debug_data->mutable_environment_model_info();
  environment_model_debug_info->clear_obstacle();
  auto &lat_obstacle_decision = session_->mutable_planning_context()
                                    ->mutable_lateral_obstacle_decider_output()
                                    .lat_obstacle_decision;
  for (auto &obstacle : reference_path_ptr->get_obstacles()) {
    // log
    planning::common::Obstacle *obstacle_log =
        environment_model_debug_info->add_obstacle();
    obstacle_log->set_id(obstacle->id());
    obstacle_log->set_type(obstacle->type());
    obstacle_log->set_is_static(obstacle->is_static());
    obstacle_log->set_lat_decision(
        static_cast<uint32_t>(lat_obstacle_decision[obstacle->id()]));
    obstacle_log->set_vs_lat_relative(obstacle->frenet_velocity_l());
    obstacle_log->set_vs_lon_relative(obstacle->frenet_velocity_s());
    if (obstacle->source_type() == SourceType::GroundLine ||
        obstacle->source_type() == SourceType::OCC ||
        obstacle->source_type() == SourceType::OD ||
        obstacle->source_type() == SourceType::MAP) {
      for (const auto &polygon :
           obstacle->obstacle()->perception_polygon().points()) {
        planning::common::Point2d *obstacle_polygon =
            obstacle_log->add_polygon_points();
        obstacle_polygon->set_x(polygon.x());
        obstacle_polygon->set_y(polygon.y());
      }
    }
  }
}

bool LateralObstacleDecider::CheckSideObstacle(
    const std::shared_ptr<ReferencePath> &reference_path_ptr,
    FrenetObstacle &frenet_obstacle) {
  const double KOverlapSThrt = 0.3;
  const double KOverlapLThrt = 0.5;
  const auto ego_frenet_state = reference_path_ptr->get_frenet_ego_state();
  const auto ego_s_start = ego_frenet_state.boundary().s_start;
  const auto ego_s_end = ego_frenet_state.boundary().s_start;
  const auto obstacle_boundary_s_start = frenet_obstacle.frenet_obstacle_boundary().s_start;
  const auto obstacle_boundary_s_end = frenet_obstacle.frenet_obstacle_boundary().s_end;
  if ((obstacle_boundary_s_end - ego_s_start > 0 &&
      obstacle_boundary_s_end - ego_s_start <= KOverlapSThrt)) {
    // 障碍物在自车前方先不考虑
    // (ego_s_end - obstacle_boundary_s_start > 0 &&
    // ego_s_end - obstacle_boundary_s_start <= KOverlapSThrt)
    Polygon2d obstacle_sl_polygon;
    const auto ok = frenet_obstacle.get_polygon_at_time(0, reference_path_ptr,
                                                obstacle_sl_polygon);
    const auto ego_sl_polygon = ego_frenet_state.polygon();
    if (ok) {
      Polygon2d care_overlap_polygon;
      bool b_overlap_with_care = false;
      b_overlap_with_care =
          obstacle_sl_polygon.ComputeOverlap(ego_sl_polygon, &care_overlap_polygon);
      if (b_overlap_with_care) {
        if (care_overlap_polygon.max_y() - care_overlap_polygon.min_y() > KOverlapLThrt) {
          return false;
        }
      }
    }
  }
  return true;
}
}  // namespace planning