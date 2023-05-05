#include "general_lateral_decider.h"

#include <assert.h>

#include <cmath>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace planning {

using namespace planning_math;

#ifndef KPH_40
#define KPH_40 11.11
#endif
#ifndef KPH_50
#define KPH_50 13.88
#endif
GeneralLateralDecider::GeneralLateralDecider(
    const EgoPlanningConfigBuilder *config_builder,
    const std::shared_ptr<TaskPipelineContext> &pipeline_context)
    : Task(config_builder, pipeline_context) {
  config_ = config_builder->cast<LateralDeciderConfig>();
  name_ = "Lateral Deicder";
}

bool GeneralLateralDecider::init_info() {
  // init input info
  cur_reference_path_ptr_ =
      frame_->session()
          ->environmental_model()
          .get_reference_path_manager()
          ->get_reference_path_by_lane(frame_->mutable_session()
                                           ->mutable_planning_context()
                                           ->mutable_scenario_state_machine()
                                           ->get_lane_change_lane_manager()
                                           ->flane()
                                           ->get_virtual_id());
  obs_vec_ = cur_reference_path_ptr_->get_obstacles();

  ego_frenet_state_ = cur_reference_path_ptr_->get_frenet_ego_state();

  ego_cart_state_manager_ =
      frame_->session()->environmental_model().get_ego_state_manager();

  is_lane_change_scene_ = false;

  if (cur_reference_path_ptr_ == nullptr ||
      ego_cart_state_manager_ == nullptr) {
    // add logs
    return false;
  }
  return true;
}

bool GeneralLateralDecider::Execute(planning::framework::Frame *frame) {
  frame_ = frame;  // TBD:  check api
  if (!init_info()) {
    return false;
  };

  cruise_vel_ = KPH_40;
  horizion_num_ = 25;
  delta_t_ = 0.25;
  // auto config_builder =
  //     frame->mutable_session()->mutable_planning_context()->config_builder(
  //         planning::common::SceneType::HIGHWAY);  // TBD:  check api

  // process
  handle_lane_change_scene();  // TODO:handle the lane change info;

  ReferencePathPoints refpath_points;
  construct_reference_path_points(refpath_points);

  MapObstacleDecision map_obstacle_decisions;
  construct_lane_and_boundary_bounds(refpath_points, map_obstacle_decisions);

  ObstacleDecisions obstacle_decisions;
  construct_lateral_obstacle_decisions(refpath_points, obstacle_decisions);

  LatDeciderOutput lat_decider_output;
  lat_decider_output.v_cruise = cruise_vel_;

  generate_boundary(map_obstacle_decisions, obstacle_decisions, refpath_points,
                    lat_decider_output);
  generate_lat_reference_traj(refpath_points, lat_decider_output);

  return true;
}

bool GeneralLateralDecider::ExecuteTest(planning::framework::Frame *frame,
                                        bool pipeline_test) {
  // pipeline test
  return true;
}

void GeneralLateralDecider::handle_lane_change_scene() {
  // logic;
}

bool GeneralLateralDecider::construct_reference_path_points(
    ReferencePathPoints &refpath_points) {
  ReferencePathPoint refpath_pt;

  double s = ego_frenet_state_.s();
  for (size_t i = 0; i < horizion_num_ + 1; i++) {
    auto success = cur_reference_path_ptr_->get_reference_point_by_lon(
        s + i * delta_t_ * cruise_vel_, refpath_pt);  // TBD: check API

    if (!success) {
      // add logs
      return false;
    }

    refpath_points.emplace_back(refpath_pt);
  }
  return true;
}

void GeneralLateralDecider::construct_lane_and_boundary_bounds(
    const ReferencePathPoints &refpath_points,
    MapObstacleDecision &map_obstacle_decisions) {
  const auto &vehicle_param =
      frame_->session()->vehicle_config_context().get_vehicle_param();

  double left_border_distance{10.};
  double right_border_distance{10.};

  for (size_t i = 0; i < refpath_points.size(); i++) {
    Bound path_bound{-10., 10.};
    Bound safe_bound{-10., 10.};
    MapObstaclePositionDecision map_obstacle_decision;

    map_obstacle_decision.tp.t = i * delta_t_;
    map_obstacle_decision.tp.s = refpath_points[i].frenet_point.x;
    map_obstacle_decision.tp.l = refpath_points[i].frenet_point.y;

    const auto &left_lane_distance =
        refpath_points[i].distance_to_left_lane_border;
    const auto &right_lane_distance =
        refpath_points[i].distance_to_right_lane_border;
    const auto &left_road_distance =
        refpath_points[i].distance_to_left_road_border;
    const auto &right_road_distance =
        refpath_points[i].distance_to_right_road_border;

    safe_bound.upper = std::fmin(
        left_lane_distance - vehicle_param.width - config_.buffer2lane,
        safe_bound.upper);
    safe_bound.lower = std::fmax(
        -right_lane_distance + vehicle_param.width + config_.buffer2lane,
        safe_bound.lower);
    path_bound.upper = std::fmin(
        right_road_distance - vehicle_param.width - config_.buffer2border,
        path_bound.upper);
    path_bound.lower = std::fmin(
        right_road_distance - vehicle_param.width - config_.buffer2border,
        path_bound.upper);

    map_obstacle_decision.lat_bounds.emplace_back(
        WeightedBound{safe_bound.lower, safe_bound.upper,
                      config_.kPhysicalBoundWeight, BoundInfo{i, "lane"}});
    map_obstacle_decision.lat_bounds.emplace_back(
        WeightedBound{path_bound.lower, path_bound.upper,
                      config_.kHardBoundWeight, BoundInfo{i, "road"}});

    map_obstacle_decisions.emplace_back(std::move(map_obstacle_decision));
  }
}

void GeneralLateralDecider::construct_lateral_obstacle_decisions(
    // const TrajectoryPoints &traj_points,
    const ReferencePathPoints &refpath_points,
    ObstacleDecisions &obstacle_decisions) {
  ObstaclePotentialDecisions obstacle_potential_decisions;

  int32_t obj_cnt = 0;

  for (auto &obstacle : obs_vec_) {
    const auto &otype = obstacle->type();
    if (otype == Common::ObjectType::OBJECT_TYPE_UNKNOWN or  // TBD: check
                                                             // obstacle type
        otype == Common::ObjectType::OBJECT_TYPE_UNKNOWN_MOVABLE or
        otype == Common::ObjectType::OBJECT_TYPE_UNKNOWN_IMMOVABLE or
        otype == Common::ObjectType::OBJECT_TYPE_VAN or
        otype == Common::ObjectType::OBJECT_TYPE_TRAILER or
        otype == Common::ObjectType::OBJECT_TYPE_TEMPORY_SIGN) {
      // add logs;
      continue;
    }
    const auto &obstacle_id = obstacle->id();
    auto obstacle_decision = ObstacleDecision{obstacle_id, {}, {}};
    auto obstacle_potential_decision =
        ObstaclePotentialDecision{obstacle_id, {}};

    construct_lateral_obstacle_decision(refpath_points, obstacle,
                                        obstacle_decision);

    obstacle_decisions[obstacle_id] = std::move(obstacle_decision);
    if (obstacle_potential_decision.extend_decisions.extended) {
      obstacle_potential_decisions[obstacle_id] =
          std::move(obstacle_potential_decision);
    }

    obj_cnt++;
  }
}

void GeneralLateralDecider::construct_lateral_obstacle_decision(
    const ReferencePathPoints &refpath_points,
    const std::shared_ptr<FrenetObstacle> obstacle,
    ObstacleDecision &obstacle_decision) {
  // NTRACE_CALL();
  using namespace planning_math;
  if (refpath_points.empty()) {
    return;
  }

  const auto &vehicle_param =
      frame_->session()->vehicle_config_context().get_vehicle_param();
  // TBD: consider the otype in CP scenario;

  // Step 1) configs
  const auto &l_care_width = config_.l_care_width;
  const auto &care_object_lat_distance_threshold =
      config_.care_obj_lat_distance_threshold;
  const auto &care_object_lon_distance_threshold =
      config_.care_obj_lon_distance_threshold;
  const auto &ego_cur_s = refpath_points.front().frenet_point.x;
  const auto &ego_cur_l = refpath_points.front().frenet_point.y;
  const auto &ego_velocity = cruise_vel_;

  auto safe_center_distance =
      config_.dynamic_obj_safe_buffer + vehicle_param.width / 2;
  auto collision_center_distance = vehicle_param.width / 2;
  auto l_offset_limit = 10.0;
  auto avoid_cross_lane = 0.2;
  auto pre_lateral_decision = LatObstacleDecisionType::IGNORE;

  double rear_axle_to_front_bumper =  // TBD：define as consexpr
      vehicle_param.length - vehicle_param.back_edge_to_rear_axis;
  const bool init_lon_no_overlap =
      (obstacle->frenet_obstacle_boundary().s_end <
           ego_cur_s - vehicle_param.back_edge_to_rear_axis ||
       obstacle->frenet_obstacle_boundary().s_start >
           ego_cur_s + rear_axle_to_front_bumper);
  const bool init_lat_overlap = !(obstacle->frenet_obstacle_boundary().l_end <
                                      ego_cur_l - vehicle_param.width / 2 ||
                                  obstacle->frenet_obstacle_boundary().l_start >
                                      ego_cur_l + vehicle_param.width / 2);

  Polygon2d obstacle_start_sl_polygon;
  Polygon2d obstacle_end_sl_polygon;
  const bool ok_start = obstacle->get_polygon_at_time(  // TBD: no prediction
      0., cur_reference_path_ptr_, obstacle_start_sl_polygon);
  const bool ok_end = obstacle->get_polygon_at_time(  // TBD: no prediction
      refpath_points.size() * delta_t_, cur_reference_path_ptr_,
      obstacle_end_sl_polygon);

  double dynamic_bound_gain_vel = std::max(config_.min_gain_vel, ego_velocity);
  double dynamic_bound_slack_coefficient =
      1.0 / dynamic_bound_gain_vel / dynamic_bound_gain_vel;
  double nudge_obj_extra_buffer = 0.2;
  bool nudge_obj_flag{false};
  bool reset_conflict_decision{false};

  obstacle_decision.rel_pos_type = ObsRelPosType::UNDEFINED;

  auto care_object_t_threshold = 2.0;
  // bool is_approach_to_destination =
  //     distance_to_destination <
  //     route_extend_length +
  //     config_.approach_distance_threshold_avoid_obstacle;  // // TBD:
  // didnt
  //     prepared yet

  // Step 2) filter far away objects
  if (std::fabs(obstacle->frenet_s() - ego_cur_s) >
          care_object_lon_distance_threshold or
      std::fabs(obstacle->frenet_l() - ego_cur_l) >
          care_object_lat_distance_threshold) {
    // TBD: add log
    return;
  }

  if (ego_cur_s - vehicle_param.back_edge_to_rear_axis -
          config_.dynamic_obj_safe_buffer - obstacle->frenet_s() -
          obstacle->obstacle()->length() >
      0.) {
    // TBD: add log
    return;
  }

  auto safe_extra_distance = 0.0;
  if (obstacle->type() == Common::ObjectType::OBJECT_TYPE_CONE ||
      obstacle->type() == Common::ObjectType::OBJECT_TYPE_BUS ||
      obstacle->type() == Common::ObjectType::OBJECT_TYPE_PEDESTRIAN ||
      obstacle->type() == Common::ObjectType::OBJECT_TYPE_TRUCK) {
    safe_extra_distance += 0.2;
  }

  // Step 3) filter rear objects
  if (obstacle->frenet_obstacle_boundary().s_end < ego_cur_s) {
    // TBD: add log
    return;
  }

  if (check_obj_nudge_condition(
          refpath_points,
          obstacle)) {  // TBD: execute nudge logic in this function:
                        //       1. whether to nudge?
                        //       2. calc the extra buffer
                        //       3. set the lat type as NUDGE
    // TBD: add log;
    safe_extra_distance += nudge_obj_extra_buffer;
  }

  bool is_cross_obj{false};
  bool has_lat_decision{false};
  bool has_lon_decision{false};
  // Step 5) calculate safe_bound, path_bound
  for (size_t i = 0; i < refpath_points.size(); i++) {
    auto &refpath_pt = refpath_points[i];
    auto t = i * delta_t_;
    if (t > care_object_t_threshold) {
      continue;
    }

    const auto &ego_s = refpath_pt.frenet_point.x;
    const auto &ego_l = refpath_pt.frenet_point.y;
    double rear_axle_to_front_bumper =
        vehicle_param.length - vehicle_param.back_edge_to_rear_axis;
    double care_area_s_buffer =
        std::max(std::min(5.0, 5.0 - 0.5 * (10.0 - ego_velocity)),
                 0.0);  // 5m -> 0m linear decay wrt ego_

    auto care_area_s_start = ego_s - vehicle_param.back_edge_to_rear_axis;
    auto care_area_s_end =
        ego_s + rear_axle_to_front_bumper + care_area_s_buffer;
    auto care_area_center =
        Vec2d((care_area_s_start + care_area_s_end) * 0.5, ego_l);
    auto care_area_length = care_area_s_end - care_area_s_start;
    auto care_polygon =
        Polygon2d(Box2d(care_area_center, 0, care_area_length, l_care_width));

    // auto obstacle_pos_time = 0.;  // TBD: wait for predication
    //  if (obstacle->type() == ObjectType::PEDESTRIAN ||
    //      obstacle->type() == ObjectType::OFO) {
    //    obstacle_pos_time = 0.;
    //  }

    Polygon2d obstacle_sl_polygon;
    auto ok = obstacle->get_polygon_at_time(0., cur_reference_path_ptr_,
                                            obstacle_sl_polygon);
    if (!ok) {
      // TBD add log
      return;
    }
    Polygon2d care_overlap_polygon;
    bool b_overlap_with_care = false;
    // default: invalid value
    double overlap_min_y = 100.0;
    double overlap_max_y = -100.0;
    // frenet_length is larger than raw length when obj is in lagre curvature
    // road
    double frenet_length =
        obstacle_sl_polygon.max_x() - obstacle_sl_polygon.min_x();

    b_overlap_with_care =
        obstacle_sl_polygon.ComputeOverlap(care_polygon, &care_overlap_polygon);
    if (b_overlap_with_care) {
      // TBD: add log
      overlap_min_y = care_overlap_polygon.min_y();
      overlap_max_y = care_overlap_polygon.max_y();
    } else {
      // TBD: add log
      continue;
    }

    // todo: high speed vehicle
    // do decision
    auto lat_decision = LatObstacleDecisionType::IGNORE;
    auto lon_decision = LonObstacleDecisionType::IGNORE;
    Bound path_bound{-l_offset_limit, l_offset_limit};
    Bound safe_bound{-l_offset_limit, l_offset_limit};

    std::pair<double, double> s_side_range{obstacle_sl_polygon.min_x(),
                                           obstacle_sl_polygon.max_x()};
    std::pair<double, double> ego_side_range{
        ego_s - vehicle_param.back_edge_to_rear_axis,
        ego_s + rear_axle_to_front_bumper + 2.};
    bool b_overlap_side = std::max(s_side_range.first, ego_side_range.first) <
                          std::min(s_side_range.second, ego_side_range.second);

    if (check_obj_crossing_condition(obstacle, is_cross_obj)) {
      // TBD:add logs
    }

    const auto &nudge_reference_center_l = ego_l;
    constexpr double kMaxAvoidEdgeL{2.0};  // m

    if ((overlap_min_y <= nudge_reference_center_l &&
         nudge_reference_center_l <= overlap_max_y) ||
        is_cross_obj) {
      lat_decision = LatObstacleDecisionType::IGNORE;
      lon_decision = LonObstacleDecisionType::YIELD;

    } else if (overlap_min_y > nudge_reference_center_l) {
      auto avoid_right_edge = overlap_min_y - safe_center_distance -
                              safe_extra_distance - vehicle_param.width / 2;
      if (avoid_right_edge >
              std::fmax(
                  -refpath_pt.distance_to_right_lane_border - avoid_cross_lane,
                  -kMaxAvoidEdgeL) or
          b_overlap_side) {
        lat_decision = LatObstacleDecisionType::RIGHT;
        lon_decision = LonObstacleDecisionType::IGNORE;
      } else {
        lat_decision = LatObstacleDecisionType::IGNORE;
        lon_decision = LonObstacleDecisionType::YIELD;
      }

    } else {
      assert(overlap_max_y < nudge_reference_center_l);
      auto avoid_left_edge = overlap_max_y + safe_center_distance +
                             safe_extra_distance + vehicle_param.width / 2;
      if (avoid_left_edge < std::min(refpath_pt.distance_to_left_lane_border +
                                         avoid_cross_lane,
                                     kMaxAvoidEdgeL) or
          b_overlap_side) {
        lat_decision = LatObstacleDecisionType::LEFT;
        lon_decision = LonObstacleDecisionType::IGNORE;
      } else {
        lat_decision = LatObstacleDecisionType::IGNORE;
        lon_decision = LonObstacleDecisionType::YIELD;
      }
    }

    if (pre_lateral_decision == LatObstacleDecisionType::IGNORE) {
      pre_lateral_decision = lat_decision;
      if (lat_decision != LatObstacleDecisionType::IGNORE) {
      }
    } else if (pre_lateral_decision == LatObstacleDecisionType::LEFT) {
      if (lat_decision == LatObstacleDecisionType::RIGHT) {
        lat_decision = LatObstacleDecisionType::IGNORE;
        lon_decision = LonObstacleDecisionType::YIELD;
        if (!reset_conflict_decision && init_lon_no_overlap) {
          refine_conflict_lat_decisions(ego_l, obstacle_decision);
          reset_conflict_decision = true;
          // add logs
        }
      }
    } else {
      assert(pre_lateral_decision == LatObstacleDecisionType::RIGHT);
      if (lat_decision == LatObstacleDecisionType::LEFT) {
        lat_decision = LatObstacleDecisionType::IGNORE;
        lon_decision = LonObstacleDecisionType::YIELD;
        if (!reset_conflict_decision && init_lon_no_overlap) {
          refine_conflict_lat_decisions(ego_l, obstacle_decision);
          reset_conflict_decision = true;
          // add logs
        }
      }
    }

    if (lat_decision == LatObstacleDecisionType::LEFT) {
      path_bound.lower =
          care_overlap_polygon.max_y() + collision_center_distance;
      safe_bound.lower = care_overlap_polygon.max_y() + safe_center_distance +
                         safe_extra_distance;

    } else if (lat_decision == LatObstacleDecisionType::RIGHT) {
      path_bound.upper =
          care_overlap_polygon.min_y() - collision_center_distance;
      safe_bound.upper = care_overlap_polygon.min_y() - safe_center_distance -
                         safe_extra_distance;

    } else {
      assert(lon_decision != LonObstacleDecisionType::IGNORE);
    }

    // refine safe bound a.c. model traj and path bound
    if (lat_decision == LatObstacleDecisionType::LEFT) {
      if (ego_l < safe_bound.lower) {
        if (ego_l > path_bound.lower + config_.min_obstacle_avoid_distance) {
          double origin_avoid_buffer = safe_bound.lower - path_bound.lower;
          double adjust_avoid_buffer = planning::planning_math::lerp(
              config_.min_obstacle_avoid_distance, 0.0, origin_avoid_buffer,
              1.0, (ego_l - path_bound.lower) / origin_avoid_buffer);
          safe_bound.lower =
              path_bound.lower + std::max(config_.min_obstacle_avoid_distance,
                                          adjust_avoid_buffer);

        } else {
          safe_bound.lower =
              path_bound.lower + config_.min_obstacle_avoid_distance;
        }
      }
      if (ego_l < safe_bound.lower) {
        double converge_coeff = std::min(
            1.0, std::pow(config_.lateral_bound_converge_speed * t, 2));
        safe_bound.lower = planning::planning_math::lerp(
            ego_l, 0, safe_bound.lower, 1.0, converge_coeff);
        safe_bound.lower =
            std::max(safe_bound.lower,
                     path_bound.lower + config_.min_obstacle_avoid_distance);
      }
    } else if (lat_decision == LatObstacleDecisionType::RIGHT) {
      if (ego_l > safe_bound.upper) {
        if (ego_l < path_bound.upper - config_.min_obstacle_avoid_distance) {
          double origin_avoid_buffer = path_bound.upper - safe_bound.upper;
          double adjust_avoid_buffer = planning::planning_math::lerp(
              config_.min_obstacle_avoid_distance, 0.0, origin_avoid_buffer,
              1.0, (path_bound.upper - ego_l) / origin_avoid_buffer);
          safe_bound.upper =
              path_bound.upper - std::max(config_.min_obstacle_avoid_distance,
                                          adjust_avoid_buffer);

        } else {
          safe_bound.upper =
              path_bound.upper - config_.min_obstacle_avoid_distance;
        }
      }
      if (ego_l > safe_bound.upper) {
        double converge_coeff = std::min(
            1.0, std::pow(config_.lateral_bound_converge_speed * t, 2));
        safe_bound.upper = planning::planning_math::lerp(
            ego_l, 0, safe_bound.upper, 1.0, converge_coeff);
        safe_bound.upper =
            std::min(safe_bound.upper,
                     path_bound.upper - config_.min_obstacle_avoid_distance);
      }
    }

    has_lat_decision =
        has_lat_decision || lat_decision != LatObstacleDecisionType::IGNORE;
    has_lon_decision =
        has_lon_decision || lon_decision != LonObstacleDecisionType::IGNORE;

    ObstaclePositionDecision position_decision;
    position_decision.tp.t = t;
    position_decision.tp.s = ego_s;
    position_decision.tp.l = ego_l;

    position_decision.lat_bounds.push_back(WeightedBound{
        path_bound.lower, path_bound.upper, config_.kHardBoundWeight, {}});
    position_decision.lat_bounds.push_back(WeightedBound{
        safe_bound.lower,
        safe_bound.upper,
        config_.dynamic_bound_slack_coefficient * config_.kPhysicalBoundWeight,
        {}});
    position_decision.lat_decision = lat_decision;
    position_decision.lon_decision = lon_decision;

    obstacle_decision.position_decisions.emplace_back(
        std::move(position_decision));
  }

  // update relative position:

  // TBD: ObsRelPosType not use yet
  // if (init_lat_overlap && obstacle->frenet_obstacle_boundary().s_start >
  //                             ego_cur_s + rear_axle_to_front_bumper) {
  //   obstacle_decision.rel_pos_type = ObsRelPosType::FRONT;
  // } else if (is_cross_obj) {
  //   obstacle_decision.rel_pos_type = ObsRelPosType::CROSSING;
  // } else if (has_lon_decision) {
  //   obstacle_decision.rel_pos_type = ObsRelPosType::CUTIN;
  // } else if (has_lat_decision) {
  //   obstacle_decision.rel_pos_type = ObsRelPosType::ADJACENT;
  // }
}

bool GeneralLateralDecider::check_obj_nudge_condition(
    const ReferencePathPoints &refpath_points,
    const std::shared_ptr<FrenetObstacle> &obstacle) {
  // nudge logic
  return false;
}

// bool GeneralLateralDecider::construct_lat_behavior_output(
//     const ObstacleDecisions &obstacle_decisions) {
//   // Step 2) update ego info
//   auto &init_point = reference_path_ptr_->get_frenet_ego_state()
//                          .planning_init_point();  // TBD: check api
//   lat_decider_output_.init_state_frenet_s = init_point.frenet_state.s;
//   lat_decider_output_.init_state_frenet_l = init_point.frenet_state.r;
//   lat_decider_output_.init_state_frenet_dl = init_point.frenet_state.dr_ds;
//   lat_decider_output_.init_state_frenet_ddl =
//   init_point.frenet_state.ddr_dsds; lat_decider_output_.s_start =
//   init_point.frenet_state.s; lat_decider_output_.adc_velocity =
//       std::max(reference_path_ptr_->get_frenet_ego_state().velocity(),
//                config_.min_init_velocity);
//   //  info_.adc_velocity = std::max(init_point.v, config_.kMinInitVelocity);

//   // Step 4) update ego_state (used by lateral optimization)
//   auto &ego_state = pipeline_context_->planning_info.ego_state;
//   ego_state.s_start = info_.init_state_frenet_s;
//   ego_state.velocity = info_.adc_velocity;
//   ego_state.frenet_l = info_.init_state_frenet_l;
//   ego_state.frenet_dl = info_.init_state_frenet_dl;
//   ego_state.frenet_ddl = info_.init_state_frenet_ddl;
// }

bool GeneralLateralDecider::check_obj_crossing_condition(
    const std::shared_ptr<FrenetObstacle> obstacle, bool &is_cross_obj) {
  // check crossing:
  // constexpr double kCrossVelThreshold{2.0};
  // auto obstacle_point =
  //     obstacle->obstacle()->get_point_at_time(t);  // TBD: no prediction yet
  // double relative_yaw =
  //     obstacle_point.velocity_direction -
  //     reference_path_ptr_->get_frenet_coord()->GetRefCurveHeading(
  //         (obstacle_sl_polygon.min_x() + obstacle_sl_polygon.max_x()) / 2.0);
  // double obstacle_v_lat = std::fabs(obstacle_point.v *
  // std::sin(relative_yaw)); if (ok_start && ok_end) {
  //   bool left2right =
  //       obstacle_start_sl_polygon.min_y() >
  //           ego_l + vehicle_param.width
  //           / 2.0
  //           &&
  //       obstacle_end_sl_polygon.max_y() <
  //           ego_l - vehicle_param.width
  //           / 2.0;
  //   bool right2left =
  //       obstacle_start_sl_polygon.max_y() <
  //           ego_l - vehicle_param.width
  //           / 2.0
  //           &&
  //       obstacle_end_sl_polygon.min_y() >
  //           ego_l + vehicle_param.width
  //           / 2.0;
  //   is_cross_obj = is_cross_obj || (obstacle_v_lat > kCrossVelThreshold &&
  //                                   (left2right || right2left));
  // }
  return false;
}

void GeneralLateralDecider::refine_conflict_lat_decisions(
    const double &ego_l, ObstacleDecision &obstacle_decision) {
  for (size_t n_pd = 0; n_pd < obstacle_decision.position_decisions.size();
       ++n_pd) {
    if (obstacle_decision.position_decisions[n_pd].lat_decision !=
        LatObstacleDecisionType::IGNORE) {
      for (size_t n_lb = 0;
           n_lb < obstacle_decision.position_decisions[n_pd].lat_bounds.size();
           ++n_lb) {
        obstacle_decision.position_decisions[n_pd].lat_bounds[n_lb].lower =
            ego_l - config_.l_offset_limit;
        obstacle_decision.position_decisions[n_pd].lat_bounds[n_lb].upper =
            ego_l + config_.l_offset_limit;
      }
      obstacle_decision.position_decisions[n_pd].lat_decision =
          LatObstacleDecisionType::IGNORE;
      obstacle_decision.position_decisions[n_pd].lon_decision =
          LonObstacleDecisionType::YIELD;
    }
  }
}

void GeneralLateralDecider::generate_boundary(
    const MapObstacleDecision &map_obstacle_decision,
    const ObstacleDecisions &obstacle_decisions,
    const ReferencePathPoints &refpath_points,
    LatDeciderOutput &lat_decider_output) {
  assert(map_obstacle_decision.size() == refpath_points.size());

  // auto &path_bounds = lat_decider_output.path_bounds;
  // auto &safe_bounds = lat_decider_output.path_bounds;

  std::vector<WeightedBounds> path_bounds;
  std::vector<WeightedBounds> safe_bounds;
  path_bounds.resize(refpath_points.size());
  safe_bounds.resize(refpath_points.size());

  for (size_t i = 0; i < map_obstacle_decision.size(); i++) {
    auto &map_obstacle_position_lat_bounds =
        map_obstacle_decision[i].lat_bounds;
    for (size_t j = 0; j < map_obstacle_position_lat_bounds.size(); j++) {
      if (map_obstacle_position_lat_bounds[j].weight < 0.) {
        path_bounds[i].emplace_back(map_obstacle_position_lat_bounds[j]);
      } else {
        safe_bounds[i].emplace_back(map_obstacle_position_lat_bounds[j]);
      }
    }
  }

  for (auto &obstacle_decision : obstacle_decisions) {
    for (auto &obstacle_position_decision :
         obstacle_decision.second.position_decisions) {
      for (size_t i = 0; i < refpath_points.size(); i++) {
        if (std::fabs(obstacle_position_decision.tp.t - i * delta_t_ < 1e-2)) {
          auto obstacle_pos_bounds = obstacle_position_decision.lat_bounds;
          for (auto &obstacle_pos_bound : obstacle_pos_bounds) {
            if (obstacle_pos_bound.weight < 0.) {
              path_bounds[i].emplace_back(obstacle_pos_bound);
            } else {
              safe_bounds[i].emplace_back(obstacle_pos_bound);
            }
          }
        }
      }
    }
  }

  for (auto &bounds : path_bounds) {
    std::pair<double, double> tmp_bound{-10., 10.};  // <lower ,upper >
    for (auto &bound : bounds) {
      if (bound.upper < tmp_bound.second) {
        tmp_bound.second = bound.upper;
      }
      if (bound.lower > tmp_bound.first) {
        tmp_bound.first = bound.lower;
      }
    }
    lat_decider_output.path_bounds.emplace_back(tmp_bound);
  }

  for (auto &bounds : safe_bounds) {
    std::pair<double, double> tmp_bound{-10., 10.};  // <lower ,upper >
    for (auto &bound : bounds) {
      if (bound.upper < tmp_bound.second) {
        tmp_bound.second = bound.upper;
      }
      if (bound.lower > tmp_bound.first) {
        tmp_bound.first = bound.lower;
      }
    }
    lat_decider_output.safe_bounds.emplace_back(tmp_bound);
  }

  assert(lat_decider_output.safe_bounds.size() == refpath_points.size());
  assert(lat_decider_output.path_bounds.size() == refpath_points.size());
}

void GeneralLateralDecider::generate_lat_reference_traj(
    const ReferencePathPoints &refpath_points,
    LatDeciderOutput &lat_decider_output) {
  auto &enu_ref_path = lat_decider_output.enu_ref_path;
  enu_ref_path.resize(refpath_points.size());

  for (size_t i = 0; i < refpath_points.size(); i++) {
    Point2D cart_point;
    const auto &ref_point = refpath_points[i];
    if (cur_reference_path_ptr_->get_frenet_coord()->FrenetCoord2CartCoord(
            Point2D{ref_point.frenet_point.x, ref_point.frenet_point.y},
            cart_point) != TRANSFORM_STATUS::TRANSFORM_FAILED) {
      enu_ref_path[i].first = cart_point.x;
      enu_ref_path[i].second = cart_point.y;
    }
  }
}
}  // namespace planning