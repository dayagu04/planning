#include "vision_longitudinal_behavior_planner.h"

#include <cmath>
#include <cstdint>
#include <string>

#include "debug_info_log.h"
#include "ifly_time.h"
#include "log.h"
#include "planning_context.h"
#include "vehicle_config_context.h"

namespace planning {

VisionLongitudinalBehaviorPlanner::VisionLongitudinalBehaviorPlanner(
    const EgoPlanningConfigBuilder *config_builder, framework::Session *session)
    : Task(config_builder, session) {
  config_ = config_builder->cast<VisionLongitudinalBehaviorPlannerConfig>();
  name_ = "VisionLongitudinalBehaviorPlanner";

  accel_vel_filter_.Init(-1.0, 0.7, 0.0, 42.0, 0.1);
}

bool VisionLongitudinalBehaviorPlanner::Execute() {
  LOG_DEBUG("=======VisionLongitudinalBehaviorPlanner======= \n");

  if (!PreCheck()) {
    LOG_DEBUG("PreCheck failed\n");
    return false;
  }

  if (calculate()) {
    session_->mutable_planning_context()->mutable_planning_success() = true;
    return true;
  } else {
    return false;
  }
}

/* void VisionLongitudinalBehaviorPlanner::init(
    std::shared_ptr<WorldModel> world_model) {
  Task::init(world_model);
}
 */
bool VisionLongitudinalBehaviorPlanner::calculate() {
  update();

  update_planner_output();
  log_planner_debug_info();

  return true;
}

bool VisionLongitudinalBehaviorPlanner::update() {
  auto current_time = IflyTime::Now_ms();
  LOG_DEBUG("=======VisionLongitudinalBehaviorPlanner======= \n");
  auto &ego_state_mgr = session_->environmental_model().get_ego_state_manager();
  auto &lateral_obstacle =
      session_->environmental_model().get_lateral_obstacle();
  const auto &lateral_output =
      session_->planning_context().lateral_behavior_planner_output();
  auto &function_info = session_->environmental_model().function_info();
  auto virtual_lane_manager =
      session_->environmental_model().get_virtual_lane_manager();

  // modify
  // lane_tracks_mgr_->update_ego_state(ego_state);

  if (lane_changing_decider_ == nullptr) {
    lane_changing_decider_ = std::make_unique<VisionOnlyLaneChangeDecider>();
  }

  LOG_DEBUG("ego_v_cruise is [%f], v_ego is [%f],  a_ego is [%f] \n",
            ego_state_mgr->ego_v_cruise(), ego_state_mgr->ego_v(),
            ego_state_mgr->ego_acc());
  double v_ego = ego_state_mgr->ego_v();
  JSON_DEBUG_VALUE("v_ego", v_ego)
  auto &vision_longitudinal_behavior_planner_output =
      session_->mutable_planning_context()
          ->mutable_vision_longitudinal_behavior_planner_output();

  double last_v_target =
      vision_longitudinal_behavior_planner_output.velocity_target;
  accel_vel_filter_.SetState(last_v_target);
  v_target_ = std::min(ego_state_mgr->ego_v_cruise(), 40.0);
  calc_cruise_accel_limits(v_ego);

  const auto &lane_change_decider_output =
      session_->planning_context().lane_change_decider_output();
  const auto fix_lane = virtual_lane_manager->get_lane_with_virtual_id(
      lane_change_decider_output.fix_lane_virtual_id);
  const std::vector<ReferencePathPoint> &fix_ref_points =
      fix_lane->get_reference_path()->get_points();
  limit_accel_velocity_in_turns(v_ego, ego_state_mgr->ego_steer_angle(),
                                lateral_output.d_poly, fix_ref_points);
  a_target_objective_ = a_target_;
  limit_accel_velocity_for_cutin(lateral_obstacle->front_tracks(),
                                 lateral_obstacle->side_tracks(),
                                 lateral_output.lc_status, v_ego);
  calc_speed_with_leads(lateral_obstacle->leadone(),
                        lateral_obstacle->leadtwo(), lateral_output.lc_request,
                        v_ego);
  calc_speed_with_temp_leads(
      lateral_obstacle->tleadone(), lateral_obstacle->tleadtwo(), v_ego,
      lateral_output.close_to_accident, lateral_output.lc_request,
      lateral_output.lc_status);
  // compute_speed_4_ramp(map_info_mgr.get_map_info().lc_map_decision(),
  //                      map_info_mgr.get_map_info().lc_end_dis(),
  //                      map_info_mgr.get_map_info().v_cruise(),
  //                      map_info_mgr.get_map_info().current_lane_type(),
  //                      lateral_output.lc_status, v_limit_in_turns_, v_ego);

  calc_speed_for_ramp(v_ego);
  calc_speed_with_potential_cutin_car(lateral_obstacle->front_tracks(),
                                      lateral_output.lc_request,
                                      ego_state_mgr->ego_v_cruise(), v_ego);
  // limit_speed_4_potential_object(lateral_obstacle->front_tracks(),
  //                                lateral_output.lc_request, interval,
  //                                v_ego);
  // compute_speed_4_merging();
  calc_speed_for_lane_change(
      lateral_obstacle->leadone(), ego_state_mgr->ego_v_cruise(), v_ego,
      lateral_output.lc_request, lateral_output.lc_status);

  double decel_base = std::min(v_target_ramp_ - v_ego + 2.0, 0.0);
  a_target_.first = std::min(a_target_.first, decel_base);

  // debug info
  vision_longitudinal_behavior_planner_output.decel_base = decel_base;

  a_target_.first = clip(a_target_.first, _A_MIN, _A_MAX);
  a_target_.second = clip(a_target_.second, _A_MIN, _A_MAX);
  v_target_ = clip(v_target_, 0.0, 40.0);

  if (v_target_ > v_ego) {
    if (v_ego > last_v_target) {
      accel_vel_filter_.SetState(v_ego);
      JSON_DEBUG_VALUE("VisionLonBehavior_acc_filter_reset", 1);
    } else {
      JSON_DEBUG_VALUE("VisionLonBehavior_acc_filter_reset", 0);
    }
    accel_vel_filter_.Update(v_target_);
    v_target_ = accel_vel_filter_.GetOutput();
    JSON_DEBUG_VALUE("VisionLonBehavior_acc_filter_work", 1);
  } else if (v_target_ < v_ego &&
             (v_limit_in_turns_ == v_target_ ||
              (v_limit_ramp_ == v_target_ && is_on_ramp_))) {
    accel_vel_filter_.Update(v_target_);
    v_target_ = accel_vel_filter_.GetOutput();
    JSON_DEBUG_VALUE("VisionLonBehavior_acc_filter_work", 1);
    JSON_DEBUG_VALUE("VisionLonBehavior_acc_filter_reset", 0);
  } else {
    JSON_DEBUG_VALUE("VisionLonBehavior_acc_filter_work", 0);
    JSON_DEBUG_VALUE("VisionLonBehavior_acc_filter_reset", 0);
  }

  JSON_DEBUG_VALUE("acc_target_high", a_target_.second);
  JSON_DEBUG_VALUE("acc_target_low", a_target_.first);

  // get start & stop state
  common::StartStopInfo::StateType stop_start_state =
      UpdateStartStopState(lateral_obstacle->leadone(), v_ego);
  v_target_ =
      (stop_start_state == common::StartStopInfo::STOP) ? 0.0 : v_target_;
  JSON_DEBUG_VALUE("stop_start_state", (int)stop_start_state);
  JSON_DEBUG_VALUE("v_target_start_stop", v_target_);

  // ACC : STANDSTILL to ACTIVE need confirmed by driver
  JSON_DEBUG_VALUE("STANDSTILL", 0.0);
  if (v_ego < 1.0 &&
      function_info.function_mode() == common::DrivingFunctionInfo::ACC &&
      function_info.function_state() ==
          common::DrivingFunctionInfo::STANDSTILL) {
    v_target_ = 0.0;
    JSON_DEBUG_VALUE("STANDSTILL", 1.0);
  }
  // JSON_DEBUG_VALUE("VisionLonBehavior_final_v_target", v_target_);
  JSON_DEBUG_VALUE("v_target", v_target_);

  // HMI: CIPV
  int CIPV_id = GetCIPV(lateral_obstacle, lateral_output.lc_status);
  JSON_DEBUG_VALUE("CIPV_id", CIPV_id);

  auto end_time = IflyTime::Now_ms();
  JSON_DEBUG_VALUE("VisionLongitudinalBehaviorPlannerCost",
                   end_time - current_time);
  return true;
}

bool VisionLongitudinalBehaviorPlanner::calc_cruise_accel_limits(
    const double v_ego) {
  a_target_.first = interp(v_ego, _A_CRUISE_MIN_BP, _A_CRUISE_MIN_V);
  a_target_.second = interp(v_ego, _A_CRUISE_MAX_BP, _A_CRUISE_MAX_V);
  LOG_DEBUG("----calc_cruise_accel_limits--- \n");
  LOG_DEBUG("a_target_.first : %f ,a_target_.first : %f\n", a_target_.first,
            a_target_.second);
  // v_target_ = 40.0;

  return true;
}

bool VisionLongitudinalBehaviorPlanner::limit_accel_velocity_in_turns(
    const double v_ego, const double angle_steers,
    const std::vector<double> &d_poly,
    const std::vector<ReferencePathPoint> &ref_points) {
  // *** this function returns a limited long acceleration allowed, depending on
  // the existing lateral acceleration
  //  this should avoid accelerating when losing the target in turns
  LOG_DEBUG("----limit_accel_velocity_in_turns--- \n");
  double angle_steers_deg = angle_steers * DEG_PER_RAD;

  double a_total_max = interp(v_ego, _A_TOTAL_MAX_BP, _A_TOTAL_MAX_V);
  const auto &vehicle_param =
      VehicleConfigurationContext::Instance()->get_vehicle_param();

  double steer_ratio = vehicle_param.steer_ratio;
  double wheel_base = vehicle_param.wheel_base;
  double a_y = std::pow(v_ego, 2) * angle_steers / (steer_ratio * wheel_base);
  double a_x_allowed =
      std::sqrt(std::max(std::pow(a_total_max, 2) - std::pow(a_y, 2), 0.0));

  // And limit the logitudinal velocity for a safe turn
  double a_y_max =
      interp(std::abs(angle_steers_deg), _AY_MAX_ABS_BP, _AY_MAX_STEERS);
  double v_limit_steering = std::sqrt((a_y_max * steer_ratio * wheel_base) /
                                      std::max(std::abs(angle_steers), 0.001));
  double v_limit_in_turns = v_limit_steering;
  // calculate the velocity limit according to the road curvature
  if (d_poly.size() == 4) {
    double preview_x = config_.dis_curv + config_.t_curv * v_ego;
    double curv =
        std::fabs(2 * d_poly[0] * preview_x + d_poly[1]) /
        std::pow(std::pow(2 * d_poly[0] * preview_x + d_poly[1], 2) + 1, 1.5);
    double road_radius = 1 / std::max(curv, 0.0001);
    if (road_radius < 750) {
      a_y_max = interp(road_radius, _AY_MAX_CURV_BP, _AY_MAX_CURV_V);
    }
    double v_limit_road = std::sqrt(a_y_max * road_radius) * 0.9;
    v_limit_in_turns = std::min(v_limit_in_turns, v_limit_road);
    LOG_DEBUG("road_radius is : [%f], a_y_max: [%f]\n", road_radius, a_y_max);
    LOG_DEBUG(
        "angle_steers: [%f], angle_steers_deg: [%f], v_limit_road: [%f]\n",
        angle_steers, angle_steers_deg, v_limit_road);
    JSON_DEBUG_VALUE("v_limit_road", v_limit_road);
    JSON_DEBUG_VALUE("road_radius", road_radius);
  }
  /*
  if(ref_points.size() > 0) {
    double curv_max_pt = -100.0;
    for (int i = 0; i < ref_points.size(); i++) {
      if(std::abs(ref_points[i].path_point.kappa) > curv_max_pt) {
        curv_max_pt = std::abs(ref_points[i].path_point.kappa);
      }
    }
    double road_radius = 1 / std::max(curv_max_pt, 0.0001);
    if (road_radius < 680) {
      a_y_max = interp(road_radius, _AY_MAX_CURV_BP, _AY_MAX_CURV_V);
    }
    double v_limit_curv_pt = std::sqrt(a_y_max * road_radius) * 0.9;
    LOG_DEBUG("ref points calced road_radius is : [%f]\n", road_radius);
    LOG_DEBUG("ref points max kappa is : [%f]\n", curv_max_pt);
    LOG_DEBUG("v_limit_curv_pt: [%f]\n", v_limit_curv_pt);
    JSON_DEBUG_VALUE("VisionLonBehavior_v_limit_curv_pt", v_limit_curv_pt);
    JSON_DEBUG_VALUE("road_radius_in_ref_pt", road_radius);
    JSON_DEBUG_VALUE("max_kappa_abs", curv_max_pt);
    v_limit_in_turns = std::min(v_limit_in_turns, v_limit_curv_pt);
  }
  */
  double a_target_in_turns = 0.0;
  if (v_limit_in_turns < v_ego - 2) {
    a_target_in_turns = -0.2;
  }

  v_limit_in_turns_ = v_limit_in_turns;
  JSON_DEBUG_VALUE("v_limit_steering", v_limit_steering);
  JSON_DEBUG_VALUE("v_limit_in_turns", v_limit_in_turns);

  a_target_.first = std::min(a_target_.first, a_target_in_turns);
  a_target_.second = std::min(a_target_.second, a_x_allowed);
  v_target_ = std::min(v_target_, v_limit_in_turns);

  // debug info
  auto &vision_longitudinal_behavior_planner_output =
      session_->mutable_planning_context()
          ->mutable_vision_longitudinal_behavior_planner_output();
  vision_longitudinal_behavior_planner_output.v_limit_in_turns =
      v_limit_in_turns;
  vision_longitudinal_behavior_planner_output.a_limit_in_turns = a_target_;

  LOG_DEBUG("v_target_ : [%f] \n", v_target_);
  LOG_DEBUG("a_target_.first : [%f] ,a_target_.second : [%f]\n",
            a_target_.first, a_target_.second);

  return true;
}

bool VisionLongitudinalBehaviorPlanner::limit_accel_velocity_for_cutin(
    const std::vector<TrackedObject> &front_tracks,
    const std::vector<TrackedObject> &side_tracks, const string &lc_status,
    const double v_ego) {
  const double safety_distance = 2.0 + v_ego * 0.2;
  const double p1min_speed = 2.0;
  const double p2min_speed = 3.0;

  std::vector<const TrackedObject *> near_cars, near_cars_sorted;
  std::array<int, 3> nearest_car_track_id{0, 0, 0};
  std::array<double, 3> v_limit_cutin{40.0, 40.0, 40.0};
  std::array<double, 3> a_limit_cutin{0.0, 0.0, 0.0};
  std::array<std::string, 3> cutin_condition{"", "", ""};
  std::map<int, double> a_now;
  bool check_cutin = false;

  LOG_DEBUG("----limit_accel_velocity_4_cutin--- \n");
  // filter near cars from front && side tracks
  near_cars.clear();
  auto &lateral_obstacle =
      session_->environmental_model().get_lateral_obstacle();
  for (auto &track : front_tracks) {
    // ignore obj without camera source
    if ((track.fusion_source & OBSTACLE_SOURCE_CAMERA) == 0) {
      continue;
    };
    if (std::abs(track.y_rel) < 10.0 && std::abs(track.d_rel) < 20.0 &&
        track.type != iflyauto::ObjectType::OBJECT_TYPE_UNKNOWN) {
      near_cars.push_back(&track);
    }
  }
  for (auto &track : side_tracks) {
    // ignore obj without camera source
    if ((track.fusion_source & OBSTACLE_SOURCE_CAMERA) == 0) {
      continue;
    };
    if (std::abs(track.y_rel) < 10.0 && std::abs(track.d_rel) < 20.0 &&
        track.type != iflyauto::ObjectType::OBJECT_TYPE_UNKNOWN) {
      near_cars.push_back(&track);
    }
  }
  // sort by abs(y_min) from min to max
  std::sort(near_cars.begin(), near_cars.end(),
            [](const TrackedObject *a, const TrackedObject *b) {
              return std::abs(a->y_min) < std::abs(b->y_min);
            });
  // take at most 3 nearest car
  near_cars_sorted.clear();
  for (int i = 0; i < near_cars.size() && near_cars_sorted.size() < 3; i++) {
    if ((near_cars[i]->location_tail < 2 * safety_distance) &&
        (near_cars[i]->location_tail > -safety_distance - 5))
      near_cars_sorted.push_back(near_cars[i]);
  }

  a_limit_cutin_now_.clear();
  for (int i = 0; i < near_cars_sorted.size(); i++) {
    if (lateral_obstacle->leadone() != nullptr &&
        lateral_obstacle->leadone()->track_id ==
            near_cars_sorted[i]->track_id) {
      continue;
    }
    nearest_car_track_id[i] = near_cars_sorted[i]->track_id;
    // check a history
    auto iter = a_limit_cutin_history_.find(near_cars_sorted[i]->track_id);
    if (iter != a_limit_cutin_history_.end()) {
      a_limit_cutin[i] = iter->second;
      cutin_condition[i] = "a_history";
    } else {
      a_limit_cutin[i] = 0.0;
      cutin_condition[i] = "zero";
    }

    double car_length =
        near_cars_sorted[i]->location_head - near_cars_sorted[i]->location_tail;
    double d_x_offset = std::max(car_length - 5.0, 0.0);

    // calculate y_rel for ttc
    double y_rel = 0.0;
    if (near_cars_sorted[i]->y_x0 != 0.0) {
      y_rel = std::abs(std::abs(near_cars_sorted[i]->y_x0) - 1.1);
    } else if (near_cars_sorted[i]->location_tail < 0.0) {
      y_rel = std::abs(std::abs(near_cars_sorted[i]->y_min) - 1.1);
    } else {
      double y_thres = interp(near_cars_sorted[i]->v_rel, _Y_THRES_SPEED_BP,
                              _Y_THRES_SPEED_V);
      y_rel = std::abs(std::abs(near_cars_sorted[i]->y_min) - y_thres);
    }

    double v_rel = near_cars_sorted[i]->v_rel - 0.5;
    double vy_rel = near_cars_sorted[i]->vy_rel;
    double d_offset = 2.0 + 0.1 * v_ego;
    double v_coeff = interp(std::abs(near_cars_sorted[i]->y_min),
                            _CUT_IN_COEFF_BP, _CUT_IN_COEFF_V);
    bool cutin_valid = true;

    // check cutin car
    bool cutin_car = false;
    bool potential_cutin_car_1 = false;
    bool potential_cutin_car_2 = false;
    bool NEAR_CAR_LAT_MOVING = fabs(vy_rel) > 0.1;
    if (near_cars_sorted[i]->y_min < 0) {
      if (lc_status == "left_lane_change") {
        vy_rel = std::max(vy_rel, (vy_rel + near_cars_sorted[i]->v_lat) / 2);
      }
      if (lc_status == "right_lane_change_wait") {
        if (NEAR_CAR_LAT_MOVING) {
          cutin_car = near_cars_sorted[i]->y_min + vy_rel * v_coeff >= -1.6;
          potential_cutin_car_1 =
              near_cars_sorted[i]->y_min + 1.25 * vy_rel * v_coeff >= -1.6;
          potential_cutin_car_2 =
              near_cars_sorted[i]->y_min + 1.5 * vy_rel * v_coeff >= -1.6;
        } else {
          cutin_car = (near_cars_sorted[i]->y_min + vy_rel * v_coeff >=
                       -CUIIN_WIDTH_STATIC);
          potential_cutin_car_1 =
              (near_cars_sorted[i]->y_min + 1.25 * vy_rel * v_coeff >=
               -CUIIN_WIDTH_STATIC);
          potential_cutin_car_2 =
              (near_cars_sorted[i]->y_min + 1.5 * vy_rel * v_coeff >=
               -CUIIN_WIDTH_STATIC);
        }
      } else {
        if (NEAR_CAR_LAT_MOVING) {
          cutin_car = near_cars_sorted[i]->y_min + vy_rel * v_coeff >= -1.6;
          potential_cutin_car_1 =
              near_cars_sorted[i]->y_min + 1.5 * vy_rel * v_coeff >=
              -1 * (1.6 + 0.01 * v_ego);
          potential_cutin_car_2 =
              near_cars_sorted[i]->y_min + 2.0 * vy_rel * v_coeff >=
              -1 * (1.6 + 0.01 * v_ego);
        } else {
          cutin_car = (near_cars_sorted[i]->y_min + vy_rel * v_coeff >=
                       -CUIIN_WIDTH_STATIC);
          potential_cutin_car_1 =
              (near_cars_sorted[i]->y_min + 1.5 * vy_rel * v_coeff >=
               -1 * (CUIIN_WIDTH_STATIC + 0.01 * v_ego));
          potential_cutin_car_2 =
              (near_cars_sorted[i]->y_min + 2.0 * vy_rel * v_coeff >=
               -1 * (CUIIN_WIDTH_STATIC + 0.01 * v_ego));
        }
      }
    } else {
      if (lc_status == "right_lane_change") {
        vy_rel = min(vy_rel, (vy_rel + near_cars_sorted[i]->v_lat) / 2);
      }
      if (lc_status == "left_lane_change_wait") {
        if (NEAR_CAR_LAT_MOVING) {
          cutin_car = near_cars_sorted[i]->y_min + vy_rel * v_coeff <= 1.6;
          potential_cutin_car_1 =
              near_cars_sorted[i]->y_min + 1.25 * vy_rel * v_coeff <= 1.6;
          potential_cutin_car_2 =
              near_cars_sorted[i]->y_min + 1.5 * vy_rel * v_coeff <= 1.6;
        } else {
          cutin_car = (near_cars_sorted[i]->y_min + vy_rel * v_coeff <=
                       CUIIN_WIDTH_STATIC);
          potential_cutin_car_1 =
              (near_cars_sorted[i]->y_min + 1.25 * vy_rel * v_coeff <=
               CUIIN_WIDTH_STATIC);
          potential_cutin_car_2 =
              (near_cars_sorted[i]->y_min + 1.5 * vy_rel * v_coeff <=
               CUIIN_WIDTH_STATIC);
        }
      } else {
        if (NEAR_CAR_LAT_MOVING) {
          cutin_car = near_cars_sorted[i]->y_min + vy_rel * v_coeff <= 1.6;
          potential_cutin_car_1 =
              near_cars_sorted[i]->y_min + 1.5 * vy_rel * v_coeff <=
              (1.6 + 0.01 * v_ego);
          potential_cutin_car_2 =
              near_cars_sorted[i]->y_min + 2.0 * vy_rel * v_coeff <=
              (1.6 + 0.01 * v_ego);
        } else {
          cutin_car = (near_cars_sorted[i]->y_min + vy_rel * v_coeff <=
                       CUIIN_WIDTH_STATIC);
          potential_cutin_car_1 =
              (near_cars_sorted[i]->y_min + 1.5 * vy_rel * v_coeff <=
               (CUIIN_WIDTH_STATIC + 0.01 * v_ego));
          potential_cutin_car_2 =
              (near_cars_sorted[i]->y_min + 2.0 * vy_rel * v_coeff <=
               (CUIIN_WIDTH_STATIC + 0.01 * v_ego));
        }
      }
    }

    // calculate ttc
    vy_rel = near_cars_sorted[i]->y_min > 0 ? vy_rel : -vy_rel;
    double ttc =
        std::max(y_rel / std::max(std::abs(std::min(vy_rel, 0.0)), 0.01), 0.2);
    double ttc_org = ttc;
    bool lead_car = std::abs(near_cars_sorted[i]->y_min) < 0.8;

    // fast cutin car
    double d_thres =
        interp(std::abs(vy_rel), _D_THRES_SPEED_BP, _D_THRES_SPEED_V);
    if (near_cars_sorted[i]->location_tail < 0.0 &&
        near_cars_sorted[i]->location_tail > -3.0 &&
        near_cars_sorted[i]->v_rel > 2.0) {
      double predict_d_x =
          near_cars_sorted[i]->v_rel * ttc + near_cars_sorted[i]->location_tail;
      if (predict_d_x > d_thres) {
        ttc = std::max(std::abs(near_cars_sorted[i]->y_min) /
                           std::max(std::abs(std::min(vy_rel, 0.0)), 0.01),
                       0.2);
      }
    }

    // calculate v(a)_limit_cutin
    if (cutin_valid &&
        ((0.8 <= std::abs(near_cars_sorted[i]->y_min)) &&
         (std::abs(near_cars_sorted[i]->y_min) <= 2.5)) &&
        cutin_car &&
        ((-3.5 - d_x_offset - min(near_cars_sorted[i]->v_rel, 1.0) <
          near_cars_sorted[i]->location_tail) &&
         (near_cars_sorted[i]->location_tail < safety_distance + 2))) {
      v_limit_cutin[i] =
          v_ego + v_rel -
          ((-(near_cars_sorted[i]->location_tail) + safety_distance) / ttc);
      a_limit_cutin[i] =
          std::min(-0.5 + 2 *
                              (near_cars_sorted[i]->location_tail +
                               near_cars_sorted[i]->v_rel * ttc - d_offset) /
                              std::pow(ttc, 2),
                   a_limit_cutin[i] + 0.1);
      cutin_condition[i] = "cutin_car, stage 1";
    } else if (cutin_valid &&
               ((0.8 <= std::abs(near_cars_sorted[i]->y_min)) &&
                (std::abs(near_cars_sorted[i]->y_min) <= 2.1)) &&
               cutin_car &&
               ((-6.0 - d_x_offset < near_cars_sorted[i]->location_tail) &&
                (near_cars_sorted[i]->location_tail < -3.5)) &&
               (near_cars_sorted[i]->v_rel > 0.5) && (v_ego > 2.0)) {
      v_limit_cutin[i] =
          v_ego + v_rel -
          ((-(near_cars_sorted[i]->location_tail) + safety_distance) / ttc);
      a_limit_cutin[i] =
          std::min(-0.5 + 2 *
                              (near_cars_sorted[i]->location_tail +
                               near_cars_sorted[i]->v_rel * ttc - d_offset) /
                              std::pow(ttc, 2),
                   a_limit_cutin[i] + 0.1);
      cutin_condition[i] = "cutin_car, stage 2";
    } else if (cutin_valid &&
               ((0.8 <= std::abs(near_cars_sorted[i]->y_min)) &&
                (std::abs(near_cars_sorted[i]->y_min) <= 3.0)) &&
               potential_cutin_car_1 &&
               ((-3.5 - d_x_offset < near_cars_sorted[i]->location_tail) &&
                (near_cars_sorted[i]->location_tail < safety_distance))) {
      v_limit_cutin[i] =
          near_cars_sorted[i]->location_tail < 0
              ? std::max(v_ego - 1, p1min_speed)
              : std::max({v_ego - 1, v_ego + v_rel - 1, p1min_speed});
      a_limit_cutin[i] =
          std::max(std::min(a_limit_cutin[i], -0.6) - 0.002, -0.8);
      cutin_condition[i] = "potential_cutin_car_1, stage 1";
      if (std::abs(near_cars_sorted[i]->y_min) < 1.7 ||
          std::abs(vy_rel) > 0.7) {
        v_limit_cutin[i] = std::max(
            {std::min(v_ego + v_rel - 1, v_ego - 1), v_ego - 3, p1min_speed});
        a_limit_cutin[i] = -1.0;
        cutin_condition[i] = "potential_cutin_car_1, stage 1.5";
      }
    } else if (cutin_valid &&
               ((0.8 <= std::abs(near_cars_sorted[i]->y_min)) &&
                (std::abs(near_cars_sorted[i]->y_min) <= 3.0)) &&
               potential_cutin_car_2 &&
               ((-3.5 - d_x_offset < near_cars_sorted[i]->location_tail) &&
                (near_cars_sorted[i]->location_tail < safety_distance))) {
      v_limit_cutin[i] =
          near_cars_sorted[i]->location_tail < 0
              ? std::max(v_ego - 0.1, p2min_speed)
              : std::max({v_ego - 0.1, v_ego + v_rel - 1, p2min_speed});
      a_limit_cutin[i] = -0.6;
      cutin_condition[i] = "potential_cutin_car_2, stage 1";
      if (std::abs(near_cars_sorted[i]->y_min) < 1.7) {
        v_limit_cutin[i] = std::max(
            {std::min(v_ego + v_rel - 1, v_ego - 1), v_ego - 3, p2min_speed});
        a_limit_cutin[i] = -1.0;
        cutin_condition[i] = "potential_cutin_car_2, stage 1.5";
      }
    } else if (cutin_valid &&
               ((0.8 <= std::abs(near_cars_sorted[i]->y_min)) &&
                (std::abs(near_cars_sorted[i]->y_min) <= 3.0)) &&
               potential_cutin_car_2 &&
               ((-6.0 - d_x_offset <= near_cars_sorted[i]->location_tail) &&
                (near_cars_sorted[i]->location_tail <= -3.5)) &&
               (near_cars_sorted[i]->v_rel > 1.0) &&
               (near_cars_sorted[i]->v_rel + v_ego > 10.0)) {
      v_limit_cutin[i] = std::max(v_ego - 0.1, p2min_speed);
      a_limit_cutin[i] = -0.6;
      cutin_condition[i] = "potential_cutin_car_2, stage 2";
      if (std::abs(near_cars_sorted[i]->y_min) < 1.7 ||
          std::abs(vy_rel) > 0.7) {
        v_limit_cutin[i] = std::max(v_ego - 1, p2min_speed);
        a_limit_cutin[i] = -1.0;
        cutin_condition[i] = "potential_cutin_car_2, stage 2.5";
      }
    } else if (lead_car && ((-3.5 < near_cars_sorted[i]->location_tail) &&
                            (near_cars_sorted[i]->location_tail <= 3.5))) {
      v_limit_cutin[i] = v_ego + v_rel - 1;
      a_limit_cutin[i] =
          -0.5 + calc_critical_decel(near_cars_sorted[i]->location_tail,
                                     -near_cars_sorted[i]->v_rel, d_offset, 0);
      cutin_condition[i] = "lead_car";
    } else {
      v_limit_cutin[i] = clip(v_limit_cutin[i] + 0.2, v_ego, 40.0);
      a_limit_cutin[i] = a_limit_cutin[i] + 0.1;
      cutin_condition[i] = "stage other";
    }

    // set cutin msg
    bool not_avoid =
        ((-3.5 - d_x_offset - std::min(near_cars_sorted[i]->v_rel, 0.0) >
              near_cars_sorted[i]->location_tail ||
          near_cars_sorted[i]->location_tail > 2.0) &&
         near_cars_sorted[i]->v_rel > 0) ||
        std::abs(near_cars_sorted[i]->y_min) < 0.8;

    if (cutin_msg_.cutin_dir == "none" && a_limit_cutin[i] <= -2.0 &&
        !not_avoid) {
      cutin_msg_.cutin_dir =
          near_cars_sorted[i]->y_min > 0.0 ? "left" : "right";
      cutin_msg_.a_limit_cutin = clip(a_limit_cutin[i], -4.0, 0.0);
      cutin_msg_.track_id = near_cars_sorted[i]->track_id;
      check_cutin = true;
    } else if (cutin_msg_.cutin_dir != "none" &&
               near_cars_sorted[i]->track_id == cutin_msg_.track_id) {
      if (cutin_msg_.a_limit_cutin < a_limit_cutin[i]) {
        cutin_msg_.a_limit_cutin += 0.1;
      }
      a_limit_cutin[i] = std::min(a_limit_cutin[i], cutin_msg_.a_limit_cutin);
      cutin_msg_.a_limit_cutin = clip(a_limit_cutin[i], -4.0, 0.0);
      if (not_avoid) {
        check_cutin = false;
      } else {
        check_cutin = true;
      }
    } else if (cutin_msg_.cutin_dir != "none" &&
               (a_limit_cutin[i] < cutin_msg_.a_limit_cutin) &&
               (cutin_msg_.a_limit_cutin < -2.0) && !not_avoid) {
      cutin_msg_.cutin_dir =
          near_cars_sorted[i]->y_min > 0.0 ? "left" : "right";
      cutin_msg_.a_limit_cutin = clip(a_limit_cutin[i], -4.0, 0.0);
      cutin_msg_.track_id = near_cars_sorted[i]->track_id;
      check_cutin = true;
    }

    v_limit_cutin[i] = std::max(v_limit_cutin[i], 0.0);
    a_limit_cutin[i] = clip(a_limit_cutin[i], -4.0, 0.0);
    a_limit_cutin_now_[near_cars_sorted[i]->track_id] = a_limit_cutin[i];

    // cutin debug info
    mjson::Json::object cutin_info_json;
    cutin_info_json["cutin_valid"] = mjson::Json(cutin_valid);
    cutin_info_json["y_rel"] = mjson::Json(y_rel);
    cutin_info_json["vy_rel"] = mjson::Json(vy_rel);
    cutin_info_json["cutin_car"] = mjson::Json(cutin_car);
    cutin_info_json["potential_cutin_car_1"] =
        mjson::Json(potential_cutin_car_1);
    cutin_info_json["potential_cutin_car_2"] =
        mjson::Json(potential_cutin_car_2);
    cutin_info_json["ttc"] = mjson::Json(ttc);
    cutin_info_json["ttc_org"] = mjson::Json(ttc_org);
    cutin_info_json["v_coeff"] = mjson::Json(v_coeff);
    cutin_info_json["not_avoid"] = mjson::Json(not_avoid);
    cutin_info_json["check_cutin"] = mjson::Json(check_cutin);
    cutin_info_[i] = mjson::Json(cutin_info_json).dump();
  }

  for (int i = 0; i < (3 - near_cars_sorted.size()); i++) {
    nearest_car_track_id[i + near_cars_sorted.size()] = 0;
    v_limit_cutin[i + near_cars_sorted.size()] =
        clip(v_limit_cutin[i + near_cars_sorted.size()] + 0.2, v_ego, 40.0);
    a_limit_cutin[i + near_cars_sorted.size()] =
        clip(a_limit_cutin[i + near_cars_sorted.size()] + 0.1, -4.0, 0.0);
    cutin_condition[i + near_cars_sorted.size()] = "near_cars_sorted None";
    cutin_info_[i + near_cars_sorted.size()] = "none";
  }

  if (!check_cutin) {
    cutin_msg_.cutin_dir = "none";
  }
  a_limit_cutin_history_.clear();
  for (auto &iter : a_limit_cutin_now_) {
    a_limit_cutin_history_[iter.first] = iter.second;
  }

  int v_min_index = 0;
  for (int i = 0; i < v_limit_cutin.size(); i++) {
    if (v_limit_cutin[i] < v_limit_cutin[v_min_index]) {
      v_min_index = i;
    }
  }

  a_target_.first = std::min(a_target_.first, a_limit_cutin[v_min_index]);
  v_target_ = std::min(v_target_, v_limit_cutin[v_min_index]);

  // debug info
  auto &vision_longitudinal_behavior_planner_output =
      session_->mutable_planning_context()
          ->mutable_vision_longitudinal_behavior_planner_output();

  vision_longitudinal_behavior_planner_output.v_limit_cutin = v_limit_cutin;
  vision_longitudinal_behavior_planner_output.a_limit_cutin = a_limit_cutin;
  vision_longitudinal_behavior_planner_output.nearest_car_track_id =
      nearest_car_track_id;
  vision_longitudinal_behavior_planner_output.cutin_condition = cutin_condition;
  vision_longitudinal_behavior_planner_output.a_limit_cutin_history =
      a_limit_cutin_history_;

  LOG_DEBUG("nearest_car_track_id : [%d],[%d],[%d] \n", nearest_car_track_id[0],
            nearest_car_track_id[1], nearest_car_track_id[2]);
  LOG_DEBUG("v_limit_cutin : [%f], v_target : [%f] \n",
            v_limit_cutin[v_min_index], v_target_);
  LOG_DEBUG("a_target_.first : [%f] ,a_target_.second : [%f]\n",
            a_target_.first, a_target_.second);

  JSON_DEBUG_VALUE("v_target_cutin", v_limit_cutin[v_min_index]);
  return true;
}

bool VisionLongitudinalBehaviorPlanner::calc_speed_with_leads(
    const TrackedObject *lead_one, const TrackedObject *lead_two,
    const string &lc_request, const double v_ego) {
  double a_lead_p = 0.0;
  double d_des = 0.0;
  double v_target_lead = 40.0;
  double a_lead_p_2 = 0.0;
  double d_des_2 = 0.0;
  double v_target_lead_2 = 40.0;
  std::pair<double, double> a_target = a_target_objective_;

  LOG_DEBUG("----compute_speed_with_leads--- \n");
  // leadone
  if (lead_one != nullptr && lead_one->type != 0) {
    LOG_DEBUG("target_lead_one's id : [%i], d_rel is : [%f], v_lead is: [%f]\n",
              lead_one->track_id, lead_one->d_rel, lead_one->v_lead);
    // process noisy a_lead signal from radar processing
    a_lead_p = process_a_lead(lead_one->a_lead_k);
    // compute desired distance
    d_des = calc_desired_distance(lead_one->v_lead, v_ego, lc_request,
                                  lead_one->is_accident_car);
    // compute desired speed
    // remove v_coast
    v_target_lead =
        calc_desired_speed(lead_one->d_rel, d_des, lead_one->v_lead);

    JSON_DEBUG_VALUE("lead_one_id", lead_one->track_id);
    JSON_DEBUG_VALUE("lead_one_dis", lead_one->d_rel);
    JSON_DEBUG_VALUE("lead_one_vel", lead_one->v_lead);
    JSON_DEBUG_VALUE("v_target_lead_one", v_target_lead);

    // leadtwo
    // 只用雷达和相机融合成功的障碍物
    bool is_camera_and_lidar = false;
    if (lead_two != nullptr) {
      is_camera_and_lidar =
          lead_two->fusion_source == OBSTACLE_SOURCE_F_RADAR_CAMERA;
    }
    if (is_camera_and_lidar && lead_two != nullptr && lead_two->type != 0) {
      LOG_DEBUG(
          "target_lead_two's id : [%i], d_rel is : [%f], v_lead is: [%f]\n",
          lead_two->track_id, lead_two->d_rel, lead_two->v_lead);
      a_lead_p_2 = process_a_lead(lead_two->a_lead_k);
      d_des_2 = calc_desired_distance(lead_two->v_lead, v_ego, lc_request,
                                      lead_two->is_accident_car);
      // leave enough space for leadOne
      d_des_2 += 7.0;
      v_target_lead_2 =
          calc_desired_speed(lead_two->d_rel, d_des_2, lead_two->v_lead);

      JSON_DEBUG_VALUE("lead_two_id", lead_two->track_id);
      JSON_DEBUG_VALUE("lead_two_dis", lead_two->d_rel);
      JSON_DEBUG_VALUE("lead_two_vel", lead_two->v_lead);
      JSON_DEBUG_VALUE("v_target_lead_two", v_target_lead_2);
    } else {
      JSON_DEBUG_VALUE("lead_two_id", 0);
      JSON_DEBUG_VALUE("lead_two_dis", 0);
      JSON_DEBUG_VALUE("lead_two_vel", 0);
      JSON_DEBUG_VALUE("v_target_lead_two", 0);
    }
    // listen to lead that makes you go slower
    if ((v_target_lead_2 < v_target_lead) && (lead_two != nullptr)) {
      // compute accel limits
      // use v_ego instead of v_pid, && remove v_coast
      calc_acc_accel_limits(
          lead_two->d_rel, d_des_2, v_ego, lead_two->v_lead, lead_two->v_rel,
          a_lead_p_2, v_target_lead_2, a_target,
          std::min(std::abs(lead_two->y_rel), lead_two->d_path));
      v_target_lead = v_target_lead_2;
    } else {
      calc_acc_accel_limits(
          lead_one->d_rel, d_des, v_ego, lead_one->v_lead, lead_one->v_rel,
          a_lead_p, v_target_lead, a_target,
          std::min(std::abs(lead_one->y_rel), lead_one->d_path));
    }

    a_target_.first = std::min(a_target_.first, a_target.first);
    a_target_.second = std::min(a_target_.second, a_target.second);
    v_target_ = std::min(v_target_, v_target_lead);

    LOG_DEBUG("desire_des : [%f] , v_target_ : [%f] \n", d_des, v_target_);

  } else {
    JSON_DEBUG_VALUE("lead_one_id", 0);
    JSON_DEBUG_VALUE("lead_one_dis", 0);
    JSON_DEBUG_VALUE("lead_one_vel", 0);
    JSON_DEBUG_VALUE("v_target_lead_one", 0);
    a_target.first = 0.0;
    a_target.second = 0.0;
  }

  // debug info
  // JSON_DEBUG_VALUE("VisionLonBehavior_v_target_lead_one", v_target_lead);
  // JSON_DEBUG_VALUE("VisionLonBehavior_v_target_lead_two", v_target_lead_2);

  auto &vision_longitudinal_behavior_planner_output =
      session_->mutable_planning_context()
          ->mutable_vision_longitudinal_behavior_planner_output();
  vision_longitudinal_behavior_planner_output.v_target_lead_one = v_target_lead;
  vision_longitudinal_behavior_planner_output.v_target_lead_two =
      v_target_lead_2;
  vision_longitudinal_behavior_planner_output.a_target_lead = a_target;

  LOG_DEBUG("a_target_.first : [%f] ,a_target_.second : [%f]\n",
            a_target_.first, a_target_.second);
  return true;
}

bool VisionLongitudinalBehaviorPlanner::calc_speed_with_temp_leads(
    const TrackedObject *temp_lead_one, const TrackedObject *temp_lead_two,
    const double v_ego, const bool close_to_accident, const string &lc_request,
    const string &lc_status) {
  double a_lead_p = 0.0;
  double d_des = 0.0;
  double v_target_temp_lead = 40.0;
  double a_lead_p_2 = 0.0;
  double d_des_2 = 0.0;
  double v_target_temp_lead_2 = 40.0;
  std::pair<double, double> a_target_temp = a_target_objective_;
  std::pair<double, double> a_target_temp2 = a_target_objective_;

  LOG_DEBUG("----compute_speed_with_temp_leads--- \n");
  // tleadone
  if (temp_lead_one != nullptr && !close_to_accident &&
      (temp_lead_one->d_path_self + std::min(temp_lead_one->v_lat, 0.3)) <
          1.0 &&
      temp_lead_one->type != 0) {
    LOG_DEBUG("temp_lead_one's id : [%i], d_rel is : [%f], v_lead is: [%f]\n ",
              temp_lead_one->track_id, temp_lead_one->d_rel,
              temp_lead_one->v_lead);
    // process noisy a_lead signal from radar processing
    a_lead_p = process_a_lead(temp_lead_one->a_lead_k);
    // compute desired distance
    d_des = calc_desired_distance(temp_lead_one->v_lead, v_ego, lc_request,
                                  temp_lead_one->is_accident_car,
                                  temp_lead_one->is_temp_lead);
    // compute desired speed
    // remove v_coast
    v_target_temp_lead =
        calc_desired_speed(temp_lead_one->d_rel, d_des, temp_lead_one->v_lead);
    calc_acc_accel_limits(
        temp_lead_one->d_rel, d_des, v_ego, temp_lead_one->v_lead,
        temp_lead_one->v_rel, a_lead_p, v_target_temp_lead, a_target_temp,
        std::min(std::abs(temp_lead_one->y_rel), temp_lead_one->d_path));

    a_target_.first = std::min(a_target_.first, a_target_temp.first);
    v_target_ = std::min(v_target_, v_target_temp_lead);

    JSON_DEBUG_VALUE("temp_lead_one_id", temp_lead_one->track_id);
    JSON_DEBUG_VALUE("temp_lead_one_dis", temp_lead_one->d_rel);
    JSON_DEBUG_VALUE("temp_lead_one_vel", temp_lead_one->v_lead);
    JSON_DEBUG_VALUE("v_target_temp_lead_one", v_target_temp_lead);
  } else {
    a_target_temp.first = 0.0;
    a_target_temp.second = 0.0;
    JSON_DEBUG_VALUE("temp_lead_one_id", 0);
    JSON_DEBUG_VALUE("temp_lead_one_dis", 0);
    JSON_DEBUG_VALUE("temp_lead_one_vel", 0);
    JSON_DEBUG_VALUE("v_target_temp_lead_one", 0);
  }
  // tleadtwo
  bool is_camera_and_lidar = false;
  if (temp_lead_two != nullptr) {
    is_camera_and_lidar =
        temp_lead_two->fusion_source == OBSTACLE_SOURCE_F_RADAR_CAMERA;
  }
  if (is_camera_and_lidar && temp_lead_two != nullptr && lc_status == "none" &&
      temp_lead_two->type != 0) {
    LOG_DEBUG("temp_lead_two's id : [%i], d_rel is : [%f], v_lead is: [%f]\n ",
              temp_lead_two->track_id, temp_lead_two->d_rel,
              temp_lead_two->v_lead);
    a_lead_p_2 = process_a_lead(temp_lead_two->a_lead_k);
    d_des_2 = calc_desired_distance(temp_lead_two->v_lead, v_ego, lc_request,
                                    temp_lead_two->is_accident_car);
    // leave enough space for temp leadOne
    d_des_2 += 7.0;
    v_target_temp_lead_2 = calc_desired_speed(temp_lead_two->d_rel, d_des_2,
                                              temp_lead_two->v_lead);

    calc_acc_accel_limits(
        temp_lead_two->d_rel, d_des_2, v_ego, temp_lead_two->v_lead,
        temp_lead_two->v_rel, a_lead_p_2, v_target_temp_lead_2, a_target_temp2,
        std::min(std::abs(temp_lead_two->y_rel), temp_lead_two->d_path));

    a_target_.first =
        std::min(a_target_.first, std::max(a_target_temp2.first, -0.6));
    double v_temp2 = std::max(v_target_temp_lead_2, v_ego - 2);
    v_target_ = std::min(v_target_, v_temp2);

    JSON_DEBUG_VALUE("temp_lead_two_id", temp_lead_two->track_id);
    JSON_DEBUG_VALUE("temp_lead_two_dis", temp_lead_two->d_rel);
    JSON_DEBUG_VALUE("temp_lead_two_vel", temp_lead_two->v_lead);
    JSON_DEBUG_VALUE("v_target_temp_lead_two", v_temp2);
  } else {
    a_target_temp2.first = 0.0;
    a_target_temp2.second = 0.0;
    JSON_DEBUG_VALUE("temp_lead_two_id", 0);
    JSON_DEBUG_VALUE("temp_lead_two_dis", 0);
    JSON_DEBUG_VALUE("temp_lead_two_vel", 0);
    JSON_DEBUG_VALUE("v_target_temp_lead_two", 0);
  }

  // debug info
  auto &vision_longitudinal_behavior_planner_output =
      session_->mutable_planning_context()
          ->mutable_vision_longitudinal_behavior_planner_output();

  vision_longitudinal_behavior_planner_output.v_target_temp_lead_one =
      v_target_temp_lead;
  vision_longitudinal_behavior_planner_output.a_target_temp_lead_one =
      a_target_temp;
  vision_longitudinal_behavior_planner_output.v_target_temp_lead_two =
      v_target_temp_lead_2;
  vision_longitudinal_behavior_planner_output.a_target_temp_lead_one =
      a_target_temp2;

  LOG_DEBUG("v_target : [%f] \n", v_target_);
  LOG_DEBUG("a_target_.first : [%f] ,a_target_.second : [%f]\n",
            a_target_.first, a_target_.second);

  return true;
}

bool VisionLongitudinalBehaviorPlanner::calc_speed_for_ramp(double v_ego) {
  LOG_DEBUG("----calc_speed_for_ramp--- \n");
  bool is_ramp_speed_limit = false;  // 匝道限速
  bool is_pre_deceleration = false;  // 匝道预减速
  auto ad_info = &(session_->mutable_planning_context()
                       ->mutable_planning_hmi_info()
                       ->ad_info);
  double v_target_ramp = 40.0;
  // config
  double dece_to_ramp = config_.dece_to_ramp;  // -1.0
  v_limit_ramp_ = config_.v_limit_ramp;        // 60km/h

  double dis_to_ramp =
      session_->environmental_model().get_virtual_lane_manager()->dis_to_ramp();
  is_on_ramp_ =
      session_->environmental_model().get_virtual_lane_manager()->is_on_ramp();

  double dis_to_merge = session_->environmental_model()
                            .get_virtual_lane_manager()
                            ->distance_to_first_road_merge();
  // 通过接口获取是否在匝道的信息
  if (is_on_ramp_) {
    if (dis_to_merge > 50) {
      v_target_ramp = v_limit_ramp_;
      is_ramp_speed_limit = true;
    }
    v_target_ = std::min(v_target_ramp, v_target_);

    LOG_DEBUG("v_target_ramp : [%f] \n", v_target_ramp);
    JSON_DEBUG_VALUE("v_target_ramp", v_target_ramp);
    JSON_DEBUG_VALUE("dis_to_ramp", dis_to_ramp);
    JSON_DEBUG_VALUE("dis_to_merge", dis_to_merge);
    LOG_DEBUG("v_target : [%f] \n", v_target_);

    // debug info
    ad_info->is_curva = is_ramp_speed_limit || is_pre_deceleration;

    return true;
  }
  double pre_brake_dis_to_ramp = std::max(dis_to_ramp - 50, 0.0);
  v_target_ramp = std::pow(
      std::pow(v_limit_ramp_, 2.0) - 2 * pre_brake_dis_to_ramp * dece_to_ramp,
      0.5);
  is_pre_deceleration = (v_target_ramp < v_target_) ? true : false;
  v_target_ = std::min(v_target_ramp, v_target_);

  a_target_.first = std::min(a_target_.first, dece_to_ramp);

  LOG_DEBUG("dis_to_ramp : [%f] \n", dis_to_ramp);
  LOG_DEBUG("v_target_ramp : [%f] \n", v_target_ramp);
  JSON_DEBUG_VALUE("v_target_ramp", v_target_ramp);
  JSON_DEBUG_VALUE("dis_to_ramp", dis_to_ramp);
  JSON_DEBUG_VALUE("dis_to_merge", dis_to_merge);
  LOG_DEBUG("v_target : [%f] \n", v_target_);
  // debug info
  ad_info->is_curva = is_ramp_speed_limit || is_pre_deceleration;
  return true;
}

// bool VisionLongitudinalBehaviorPlanner::compute_speed_4_ramp(
//     int lc_map_decision, double lc_end_dis, double ramp_max_speed,
//     MSDLaneType current_lane_type, std::string lc_status,
//     double v_target_in_turns, double v_ego) {
//   double v_target_terminus = 40.0;
//   double a_target_ramp = 0.0;

//   const double l_slope_terminus = 0.16;
//   const double p_slope_terminus = 2.7;
//   const double l_slope_ramp = 0.035;
//   const double p_slope_ramp = 0.123;

//   // this is where parabola && linear curves are tangents
//   const double x_linear_to_parabola_terminus =
//       p_slope_terminus / std::pow(l_slope_terminus, 2);
//   const double x_linear_to_parabola_ramp =
//       p_slope_ramp / std::pow(l_slope_ramp, 2);
//   // parabola offset to have the parabola being tangent to the linear curve
//   const double x_parabola_offset_terminus =
//       p_slope_terminus / (2 * std::pow(l_slope_terminus, 2));
//   const double x_parabola_offset_ramp =
//       p_slope_ramp / (2 * std::pow(l_slope_ramp, 2));

//   if (lc_map_decision != 0) {
//     if (lc_map_decision == 2 && current_lane_type == MSD_LANE_TYPE_NORMAL) {
//       if (lc_end_dis - 80 - 200 < 0) {
//         v_target_terminus = (lc_end_dis - 80) * l_slope_ramp / 3.0;
//       } else if (lc_end_dis - 80 - 200 < x_linear_to_parabola_ramp) {
//         v_target_terminus = (lc_end_dis - 80) * l_slope_ramp;
//       } else {
//         v_target_terminus =
//             std::sqrt(2 * ((lc_end_dis - 40 - 200) - x_parabola_offset_ramp)
//             *
//                       p_slope_ramp);
//       }
//       v_target_terminus = std::max(v_target_terminus, 0.0);
//       v_target_terminus = v_target_terminus + 17;
//     } else if (lc_map_decision == 1 &&
//                current_lane_type == MSD_LANE_TYPE_NORMAL) {
//       if (lc_end_dis - 120 < 0) {
//         v_target_terminus = (lc_end_dis - 120) * l_slope_ramp / 3.0;
//       } else if (lc_end_dis - 120 < x_linear_to_parabola_ramp) {
//         v_target_terminus = (lc_end_dis - 120) * l_slope_ramp;
//       } else {
//         v_target_terminus = std::sqrt(
//             2 * ((lc_end_dis - 120) - x_parabola_offset_ramp) *
//             p_slope_ramp);
//       }
//       v_target_terminus = std::max(v_target_terminus, 0.0);
//       v_target_terminus = v_target_terminus + 15;
//     } else if (current_lane_type == MSD_LANE_TYPE_NORMAL &&
//                lc_map_decision == -1 &&
//                ((lc_status != "left_lane_change" &&
//                  lc_status != "right_lane_change") ||
//                 lc_end_dis - 20 < 0)) {
//       if (lc_end_dis - 20 < 0) {
//         v_target_terminus = (lc_end_dis - 20) * l_slope_terminus / 3.0;
//       } else if (lc_end_dis - 20 < x_linear_to_parabola_terminus) {
//         v_target_terminus = (lc_end_dis - 20) * l_slope_terminus;
//       } else {
//         v_target_terminus =
//             std::sqrt(2 * ((lc_end_dis - 0) - x_parabola_offset_terminus) *
//                       p_slope_terminus);
//       }
//       v_target_terminus = std::max(v_target_terminus, 0.0);
//     } else if (current_lane_type == MSD_LANE_TYPE_ACCELERATE &&
//                ((lc_status != "left_lane_change" &&
//                  lc_status != "right_lane_change") ||
//                 lc_end_dis - 20 < 0)) {
//       if (lc_end_dis - 20 < 0) {
//         v_target_terminus = (lc_end_dis - 20) * l_slope_terminus / 3.0;
//       } else if (lc_end_dis - 20 < x_linear_to_parabola_terminus) {
//         v_target_terminus = (lc_end_dis - 20) * l_slope_terminus;
//       } else {
//         v_target_terminus =
//             std::sqrt(2 * ((lc_end_dis - 0) - x_parabola_offset_terminus) *
//                       p_slope_terminus);
//       }
//       v_target_terminus = std::max(v_target_terminus, 0.0);
//     } else {
//       // if v_target_terminus < 40:
//       //  v_target_terminus += 1
//       v_target_terminus = 40;
//     }
//   } else {
//     // if v_target_terminus < 40:
//     //  v_target_terminus += 1
//     v_target_terminus = 40;
//   }

//   v_limit_ramp_ = std::min(v_limit_ramp_, v_ego);
//   double dec_ramp = (v_limit_ramp_ - 50 / 3.6) / 100.0 + 0.08;
//   if (current_lane_type == MSD_LANE_TYPE_RAMP) {
//     if ((v_limit_ramp_ > 50 / 3.6) && (v_limit_ramp_ + 5 >
//     v_target_in_turns)) {
//       a_target_ramp = -0.5;
//       v_limit_ramp_ -= dec_ramp;
//     } else if (v_limit_ramp_ >= (ramp_max_speed - 2 / 3.6)) {
//       if (v_limit_ramp_ > ramp_max_speed) {
//         v_limit_ramp_ -= 0.06;
//       } else {
//         v_limit_ramp_ = ramp_max_speed;
//       }
//       a_target_ramp = -0.5;
//     } else if ((v_limit_ramp_ <= 50 / 3.6) &&
//                (v_limit_ramp_ + 5 > v_target_in_turns)) {
//       v_limit_ramp_ = 50 / 3.6;
//       a_target_ramp = 0.0;
//     } else if (v_limit_ramp_ + 5 <= v_target_in_turns) {
//       v_limit_ramp_ = std::min(
//           std::max({v_limit_ramp_ + 0.1, 50 / 3.6, v_ego + 1.0}), 80 / 3.6);
//       a_target_ramp = 0.0;
//     }
//   } else if (current_lane_type == MSD_LANE_TYPE_DECELERATE) {
//     if (v_limit_ramp_ > std::min(55 / 3.6, ramp_max_speed)) {
//       v_limit_ramp_ -= dec_ramp;
//       a_target_ramp = -0.5;
//     } else {
//       v_limit_ramp_ = std::min(55 / 3.6, ramp_max_speed);
//       a_target_ramp = 0.0;
//     }
//   } else if (current_lane_type == MSD_LANE_TYPE_ACCELERATE_DECELERATE) {
//     if (v_limit_ramp_ > std::min(55 / 3.6, ramp_max_speed)) {
//       v_limit_ramp_ -= dec_ramp;
//       a_target_ramp = -0.5;
//     } else {
//       v_limit_ramp_ = std::min(55 / 3.6, ramp_max_speed);
//       a_target_ramp = 0.0;
//     }
//   } else if (v_limit_ramp_ > ramp_max_speed) {
//     v_limit_ramp_ -= 0.10;
//     a_target_ramp = -0.5;
//   } else {
//     v_limit_ramp_ = min(v_target_terminus, ramp_max_speed);
//     a_target_ramp = 0.0;
//   }

//   if (lc_map_decision == 1 && current_lane_type == MSD_LANE_TYPE_NORMAL) {
//     a_target_ramp = -0.8;
//   } else if (current_lane_type == MSD_LANE_TYPE_ACCELERATE) {
//     a_target_ramp = -1.1;
//   }

//   v_target_ramp_ = std::min(v_limit_ramp_, v_target_terminus);

//   a_target_.first = std::min(a_target_.first, a_target_ramp);
//   v_target_ = std::min(v_target_, v_target_ramp_);

//   // debug info
//   auto &highway_longitudinal_output =
//       PlanningContext::Instance()
//           ->mutable_vision_only_longitudinal_motion_planner_output();
//   highway_longitudinal_output.v_target_terminus = v_target_terminus;
//   highway_longitudinal_output.v_target_ramp = v_target_ramp_;
//   highway_longitudinal_output.a_target_ramp = a_target_ramp;

//   return true;
// }

bool VisionLongitudinalBehaviorPlanner::calc_speed_with_potential_cutin_car(
    const std::vector<TrackedObject> &front_tracks, const string &lc_request,
    const double v_cruise, const double v_ego) {
  double v_target_potental_cutin = 40.0;
  double a_target_potential_min = 0.0;
  double v_limit = std::min({v_cruise, v_target_ramp_, v_ego});
  std::vector<int> front_cut_in_track_id;

  LOG_DEBUG("----calc_speed_with_potential_cutin_car--- \n");
  double cutinp_threshold = lc_request != "none" ? 0.6 : 0.2;
  front_cut_in_track_id.clear();
  std::pair<int, double> cutin_id_vt = {-1, 0.0};
  for (auto &track : front_tracks) {
    // ignore obj without camera source
    if ((track.fusion_source & OBSTACLE_SOURCE_CAMERA) == 0) {
      continue;
    };
    if (!track.is_lead && track.cutinp > cutinp_threshold &&
        track.v_lat < -0.01 && track.type < 50000) {
      std::pair<double, double> a_target = a_target_objective_;
      front_cut_in_track_id.push_back(track.track_id);
      double ttc =
          std::max(track.d_path - 1.5, 0.0) / std::max(-track.v_lat, 0.01);
      double drel_pre = std::max(0.0, track.d_rel + ttc * track.v_rel);
      double drel_pre_for_a = track.v_rel < 0.1 ? track.d_rel : drel_pre;

      double a_lead_p = process_a_lead(track.a_lead_k);
      double d_des = calc_desired_distance(track.v_lead, v_ego, lc_request);
      double v_target_car = calc_desired_speed(drel_pre, d_des, track.v_lead);
      calc_acc_accel_limits(drel_pre_for_a, d_des, v_ego, track.v_lead,
                            track.v_rel, a_lead_p, v_target_car, a_target,
                            std::min(std::abs(track.y_rel), track.d_path));

      double v_potental_cutin =
          (v_target_car - v_limit) * track.cutinp + v_limit;
      if (v_target_potental_cutin > v_potental_cutin) {
        v_target_potental_cutin = v_potental_cutin;
        cutin_id_vt.first = track.track_id;
        cutin_id_vt.second = v_target_potental_cutin;
      }

      a_target_potential_min =
          std::min({a_target_potential_min,
                    std::max(a_target.first, -4.0) * track.cutinp, -0.7});

      LOG_DEBUG("potential_cutin_car's id: [%d], track.v_lat is: [%f]\n",
                track.track_id, track.v_lat);
      LOG_DEBUG(
          "d_des: [%f], v_target_car: [%f], v_target_potental_cutin: [%f]\n",
          d_des, v_target_car, v_target_potental_cutin);
    }
  }

  JSON_DEBUG_VALUE("v_target_potental_cutin", cutin_id_vt.second);
  JSON_DEBUG_VALUE("potental_cutin_track_id", cutin_id_vt.first);

  a_target_.first = std::min(a_target_.first, a_target_potential_min);
  v_target_ = std::min(v_target_, v_target_potental_cutin);

  // debug info
  auto ad_info = &(session_->mutable_planning_context()
                       ->mutable_planning_hmi_info()
                       ->ad_info);
  // TODO c接口定义错误，要改成数组
  for (auto id : front_cut_in_track_id) {
    //   ad_info->add_cutin_track_id(id);
    ad_info->cutin_track_id = id;
    break;
  }

  auto &vision_longitudinal_behavior_planner_output =
      session_->mutable_planning_context()
          ->mutable_vision_longitudinal_behavior_planner_output();

  vision_longitudinal_behavior_planner_output.v_target_cutin_front =
      v_target_potental_cutin;
  vision_longitudinal_behavior_planner_output.a_target_cutin_front =
      a_target_potential_min;
  vision_longitudinal_behavior_planner_output.front_cut_in_track_id =
      front_cut_in_track_id;

  LOG_DEBUG("v_target_: [%f], a_target_.first: [%f]\n", v_target_,
            a_target_.first);

  return true;
}

bool VisionLongitudinalBehaviorPlanner::calc_speed_for_lane_change(
    const TrackedObject *lead_one, const double v_cruise, const double v_ego,
    const string &lc_request, const string &lc_status) {
  LOG_DEBUG("----compute_speed_4_lane_change--- \n");

  /*modify
  auto &state_machine_output =
  PlanningContext::Instance()->state_machine_output();
  virtual_lane_mgr_->restore_context(
      state_machine_output.virtual_lane_mgr_context);
  */
  const auto &lane_change_decider_output =
      session_->planning_context().lane_change_decider_output();
  const auto current_lane = session_->environmental_model()
                                .get_virtual_lane_manager()
                                ->get_current_lane();
  // modify
  // double lc_end_dis = map_info_mgr.get_map_info().lc_end_dis();
  double lc_end_dis = 0;
  int lc_map_decision = session_->environmental_model()
                            .get_virtual_lane_manager()
                            ->lc_map_decision(current_lane);
  double v_target = v_target_;
  double safety_dist = 2.0 + v_ego * 0.2;
  double v_limit_lc = 40.0;
  double a_target_lc = 0.0;
  std::vector<TrackedObject *> lane_changing_cars;
  std::vector<GapInfo> available_gap;
  int lane_changing_nearest_rear_car_track_id = -10;

  lane_changing_cars.clear();
  if ((lc_request != "none") &&
      ((lc_status == "none") || (lc_status == "left_lane_change_wait") ||
       (lc_status == "right_lane_change_wait"))) {
    LOG_DEBUG("!! lang change !! \n");
    // get target line tarcks
    if (lane_change_decider_output.has_target_lane) {
      auto &lane_tracks_mgr =
          session_->environmental_model().get_lane_tracks_manager();
      std::vector<TrackedObject> *front_target_tracks =
          lane_tracks_mgr->get_lane_tracks(
              lane_change_decider_output.target_lane_virtual_id, FRONT_TRACK);
      if (front_target_tracks != nullptr) {
        for (auto &track : *front_target_tracks) {
          // ignore obj without camera source
          if ((track.fusion_source & OBSTACLE_SOURCE_CAMERA) == 0) {
            continue;
          };
          lane_changing_cars.push_back(&track);
        }
      }
      std::vector<TrackedObject> *side_target_tracks =
          lane_tracks_mgr->get_lane_tracks(
              lane_change_decider_output.target_lane_virtual_id, SIDE_TRACK);
      if (side_target_tracks != nullptr) {
        for (auto &track : *side_target_tracks) {
          // ignore obj without camera source
          if ((track.fusion_source & OBSTACLE_SOURCE_CAMERA) == 0) {
            continue;
          };
          lane_changing_cars.push_back(&track);
        }
      }
    }

    LaneChangeParams lane_changing_params;
    lane_changing_params.most_front_car_dist = 110.0;
    lane_changing_params.most_rear_car_dist = -100.0;
    lane_changing_params.cost_minus = 10.0;
    lane_changing_params.v_rel_bufer = 1.0;
    lane_changing_decider_->feed_config_and_target_cars(
        false, lane_changing_params, lc_end_dis, lane_changing_cars, lead_one,
        v_ego);

    session_->mutable_planning_context()->set_v_limit(
        std::min(v_target, v_cruise));
    lane_changing_decider_->set_session(session_);
    lane_changing_decider_->process();

    available_gap = lane_changing_decider_->get_gap_list();
    if (available_gap.size() > 0) {
      auto gap = available_gap[0];
      if (gap.base_car_id == gap.front_id) {
        v_limit_lc =
            gap.base_car_vrel -
            clip((safety_dist - gap.base_car_drel) / safety_dist, 0.0, 2.0) -
            1.0;
        if (v_limit_lc < 0) {
          // no need to decel when front car is far away
          const std::vector<double> _V_LIMIT_DISTANCE_BP{
              safety_dist + std::max(-gap.base_car_vrel, 0.0) * 2,
              safety_dist * 2 + std::max(-gap.base_car_vrel, 0.0) * 2};
          const std::vector<double> _V_LIMIT_DISTANCE_V{1.0, 0.0};
          v_limit_lc =
              v_limit_lc * interp(gap.base_car_drel, _V_LIMIT_DISTANCE_BP,
                                  _V_LIMIT_DISTANCE_V);
        }
        v_limit_lc = std::max(v_ego - 3.0, v_ego + v_limit_lc);
        a_target_lc = 0.0;
      } else {
        v_limit_lc = gap.base_car_vrel +
                     clip((safety_dist + 5.0 + gap.base_car_drel) / safety_dist,
                          0.0, 2.0) +
                     1.0;
        if (v_limit_lc < 0) {
          // no need to decel when front car is far away
          const std::vector<double> _V_LIMIT_DISTANCE_BP{
              safety_dist + 5.0 + std::max(gap.base_car_vrel, 0.0) * 2,
              safety_dist * 2 + 5.0 + std::max(gap.base_car_vrel, 0.0) * 2};
          const std::vector<double> _V_LIMIT_DISTANCE_V{1.0, 0.0};
          v_limit_lc =
              v_limit_lc * interp(-gap.base_car_drel, _V_LIMIT_DISTANCE_BP,
                                  _V_LIMIT_DISTANCE_V);
        }
        v_limit_lc = std::max(v_ego - 2.8, v_ego + v_limit_lc);
        a_target_lc = 0.6;
      }
      if (v_limit_lc < 6.0) {
        v_limit_lc = 6.0;
        a_target_lc = 1.0;
      }
    } else {
      // decelerate to check next interval
      auto nearest_rear_car = lane_changing_decider_->nearest_rear_car_track();
      lane_changing_nearest_rear_car_track_id = nearest_rear_car.id;
      v_limit_lc =
          nearest_rear_car.v_rel -
          clip((safety_dist - nearest_rear_car.d_rel) / safety_dist, 0.0, 2.0) -
          v_ego / 10.0;
      v_limit_lc = std::max({v_ego - 3.2, v_ego + v_limit_lc,
                             6.0 + 4.0 * std::max(lc_map_decision - 2, 0)});
      a_target_lc = 0.0;
    }

    a_target_.second = std::max(a_target_.second, a_target_lc);
    v_target_ = std::min(v_target_, v_limit_lc);
  } else {
    v_limit_lc = 40.0;
    a_target_lc = 0.0;
    available_gap.clear();
  }

  // debug info
  auto &vision_longitudinal_behavior_planner_output =
      session_->mutable_planning_context()
          ->mutable_vision_longitudinal_behavior_planner_output();
  vision_longitudinal_behavior_planner_output.v_target_lane_change = v_limit_lc;
  vision_longitudinal_behavior_planner_output.a_target_lane_change =
      a_target_lc;
  vision_longitudinal_behavior_planner_output.lane_changing_cars.clear();
  for (auto &item : lane_changing_cars) {
    TargetObstacle car;
    car.id = item->track_id;
    car.d_rel = item->d_rel;
    car.v_rel = item->v_rel;
    vision_longitudinal_behavior_planner_output.lane_changing_cars.push_back(
        car);
    LOG_DEBUG("lane_changing_cars' id: [%d] \n", car.id);
  }
  vision_longitudinal_behavior_planner_output.lane_changing_available_gap =
      available_gap;
  vision_longitudinal_behavior_planner_output
      .lane_changing_nearest_rear_car_track_id =
      lane_changing_nearest_rear_car_track_id;

  LOG_DEBUG("v_target_: [%f], a_target.second: [%f]\n", v_target_,
            a_target_.second);
  return true;
}

double VisionLongitudinalBehaviorPlanner::interp(
    const double x, const std::vector<double> &xp,
    const std::vector<double> &fp) {
  const int N = xp.size() - 1;

  if (x < xp[0]) {
    return fp[0];
  }
  for (int i = 0; i <= N; ++i) {
    if (x < xp[i]) {
      return ((x - xp[i - 1]) * (fp[i] - fp[i - 1]) / (xp[i] - xp[i - 1]) +
              fp[i - 1]);
    }
  }

  return fp[N];
}

double VisionLongitudinalBehaviorPlanner::process_a_lead(const double a_lead) {
  // soft threshold of 0.5m/s^2 applied to a_lead to reject noise, also not
  // considered positive a_lead
  double a_lead_threshold = 0.5;
  return std::min(a_lead + a_lead_threshold, 0.0);
}

double VisionLongitudinalBehaviorPlanner::calc_desired_distance(
    const double v_lead, const double v_ego, const std::string &lc_request,
    const bool is_accident_car, const bool is_temp_lead) {
  LOG_DEBUG("-----calc_desired_distance \n");
  // 受限感知性能，取非负
  double v_lead_clip = std::max(v_lead, 0.0);

  double t_gap = interp(v_ego, _T_GAP_VEGO_BP, _T_GAP_VEGO_V);
  if (lc_request != "none") {
    t_gap = t_gap * (0.6 + v_ego * 0.01);
  }
  if (is_temp_lead) {
    t_gap = t_gap * 0.3;
  }
  // Brake hysteresis
  double v_relative = std::min(std::max(v_ego - v_lead_clip, 0.0), 5.0);
  double distance_hysteresis = v_relative * config_.ttc_brake_hysteresis;
  // distance when at zero speed
  double d_offset = config_.dis_zero_speed;
  std::cout << "d_offset from config === : " << d_offset << std::endl;
  if (is_accident_car) {
    d_offset = config_.dis_zero_speed_accident;
  }
  auto desired_distance_calibrate =
      d_offset + v_lead_clip * t_gap + distance_hysteresis;
  LOG_DEBUG("distance_hysteresis : [%f] \n", distance_hysteresis);
  LOG_DEBUG("ttc gap : [%f] \n", t_gap);
  LOG_DEBUG("desired_distance : [%f] \n", desired_distance_calibrate);
  JSON_DEBUG_VALUE("RealTime_desired_distance_calibrate",
                   desired_distance_calibrate);
  return desired_distance_calibrate;
}

double VisionLongitudinalBehaviorPlanner::calc_desired_speed(
    const double d_lead, const double d_des, const double v_lead) {
  // *** compute desired speed ***
  // the desired speed curve is divided in 4 portions:
  // 1-constant
  // 2-linear to regain distance
  // 3-linear to shorten distance
  // 4-parabolic (constant decel)
  const double max_runaway_speed = -2.;  // no slower than 2m/s over the lead
  //  interpolate the lookups to find the slopes for a give lead speed
  double l_slope = interp(v_lead, _L_SLOPE_BP, _L_SLOPE_V);
  double p_slope = interp(v_lead, _P_SLOPE_BP, _P_SLOPE_V);
  // this is where parabola && linear curves are tangents
  double x_linear_to_parabola = p_slope / std::pow(l_slope, 2);
  // parabola offset to have the parabola being tangent to the linear curve
  double x_parabola_offset = p_slope / (2 * std::pow(l_slope, 2));
  LOG_DEBUG("-----calc_desired_speed \n");
  LOG_DEBUG("l_slope : [%f] , p_slope : [%f]\n", l_slope, p_slope);
  LOG_DEBUG("x_linear_to_parabola : [%f] , x_parabola_offset : [%f]\n",
            x_linear_to_parabola, x_parabola_offset);

  double v_rel_des = 0.0;
  if (d_lead < d_des) {
    // calculate v_rel_des on the line that connects 0m at max_runaway_speed
    // to d_des
    double v_rel_des_1 = (-max_runaway_speed) / d_des * (d_lead - d_des);
    // calculate v_rel_des on one third of the linear slope
    double v_rel_des_2 = (d_lead - d_des) * l_slope / 3.0;
    // take the min of the 2 above
    v_rel_des = std::min(v_rel_des_1, v_rel_des_2);
    v_rel_des = std::max(v_rel_des, max_runaway_speed);
  } else if (d_lead < d_des + x_linear_to_parabola) {
    v_rel_des = (d_lead - d_des) * l_slope;
    v_rel_des = std::max(v_rel_des, max_runaway_speed);
  } else {
    v_rel_des = std::sqrt(2 * (d_lead - d_des - x_parabola_offset) * p_slope);
  }
  // compute desired speed
  double v_target = v_rel_des + v_lead;
  LOG_DEBUG("v_rel_des : [%f], v_target : [%f] \n", v_rel_des, v_target);
  return v_target;
}

bool VisionLongitudinalBehaviorPlanner::calc_acc_accel_limits(
    const double d_lead, const double d_des, const double v_ego,
    const double v_lead, const double v_rel_const, const double a_lead,
    const double v_target, std::pair<double, double> &a_target,
    const double y_min) {
  double v_rel = -v_rel_const;

  // this is how much lead accel we consider in assigning the desired decel
  double a_lead_contr =
      a_lead * interp(v_lead, _A_LEAD_LOW_SPEED_BP, _A_LEAD_LOW_SPEED_V) *
      interp(d_lead, _A_LEAD_DISTANCE_BP, _A_LEAD_DISTANCE_V) * 0.8;

  // first call of calc_positive_accel_limit is used to shape v_pid
  // use vego instead of vpid for v_ref, remove v_coast
  a_target.second = calc_positive_accel_limit(
      d_lead, d_des, v_ego, v_rel, v_target, a_lead_contr, a_target.second);
  // compute max decel
  // assume the car is 1m/s slower
  double v_offset = 1.2 * std::min(std::max(2.5 - y_min, 0.0) / 1.5, 1.0);
  // assume the distance is 1m lower
  double d_offset = 0.5 + 0.2 * v_ego;
  if (v_target - v_ego > 0.5) {
    // acc target speed is above vehicle speed, so we can use the cruise
    // limits pass
  } else {
    // add small value to avoid by zero divisions
    // compute needed accel to get to 1m distance with -1m/s rel speed
    double decel_offset = interp(v_lead, _DECEL_OFFSET_BP, _DECEL_OFFSET_V);

    double critical_decel =
        calc_critical_decel(d_lead, v_rel, d_offset, v_offset);
    a_target.first =
        std::min(decel_offset + critical_decel + a_lead_contr, a_target.first);
  }
  // a_min can't be higher than a_max
  a_target.first = min(a_target.first, a_target.second);
  // final check on limits
  a_target.first = clip(a_target.first, _A_MIN, _A_MAX);
  a_target.second = clip(a_target.second, _A_MIN, _A_MAX);
  return true;
}

double VisionLongitudinalBehaviorPlanner::calc_positive_accel_limit(
    const double d_lead, const double d_des, const double v_ego,
    const double v_rel, const double v_target, const double a_lead_contr,
    const double a_max_const) {
  // never coast faster then -1m/s^2
  double a_coast_min = -1.0;
  double a_max = a_max_const;
  // coasting behavior above v_coast. Forcing a_max to be negative will force
  // the pid_speed to decrease, regardless v_target
  if (v_ego > v_target + 0.5) {
    // for smooth coast we can be aggressive && target a point where car
    // would actually crash
    double v_offset_coast = 1.0;
    double d_offset_coast = d_des / 2.0 - 4.0;

    // acceleration value to smoothly coast until we hit v_target
    if (d_lead > d_offset_coast + 0.1) {
      double a_coast =
          calc_critical_decel(d_lead, v_rel, d_offset_coast, v_offset_coast);
      // if lead is decelerating, then offset the coast decel
      a_coast += a_lead_contr;
      a_max = std::max(a_coast, a_coast_min);
    } else {
      a_max = a_coast_min;
    }
  } else {
    // same as cruise accel, plus add a small correction based on relative
    // lead speed if the lead car is faster, we can accelerate more, if the
    // car is slower, then we can reduce acceleration
    a_max = a_max + interp(v_ego, _A_CORR_BY_SPEED_BP, _A_CORR_BY_SPEED_V) *
                        clip(-v_rel / 4.0, -0.5, 1.0);
  }
  return a_max;
}

double VisionLongitudinalBehaviorPlanner::calc_critical_decel(
    const double d_lead, const double v_rel, const double d_offset,
    const double v_offset) {
  // this function computes the required decel to avoid crashing, given safety
  // offsets
  double a_critical = -std::pow(std::max(0.0, v_rel + v_offset), 2) /
                      std::max(2 * (d_lead - d_offset), 0.5);
  return a_critical;
}

double VisionLongitudinalBehaviorPlanner::clip(const double x, const double lo,
                                               const double hi) {
  return std::max(lo, std::min(hi, x));
}

common::StartStopInfo::StateType
VisionLongitudinalBehaviorPlanner::UpdateStartStopState(
    const TrackedObject *lead_one, const double v_ego) {
  // The AION's resolution of vehicle speed is 0.3m/s
  double v_start = config_.v_start;
  double obstacle_v_start = config_.obstacle_v_start;
  double distance_stop = config_.distance_stop;
  double distance_start = config_.distance_start;

  common::StartStopInfo &start_stop_state_info =
      session_->mutable_planning_context()->mutable_start_stop_result();
  bool dbw_status = session_->environmental_model().GetVehicleDbwStatus();
  if (lead_one == nullptr || dbw_status == false) {
    // reset state as default
    start_stop_state_info.set_state(common::StartStopInfo::CRUISE);
  } else {
    // 1. Calculate the condition
    std::string lc_request = "none";
    double desire_distance =
        calc_desired_distance(lead_one->v_lead, v_ego, lc_request);
    bool is_lead_static = std::fabs(lead_one->v_lead) < obstacle_v_start;
    bool stop_condition =
        (v_ego < v_start && is_lead_static &&
         std::fabs(lead_one->d_rel - desire_distance) < distance_stop);
    bool cruise_condition = v_ego > v_start;
    bool lead_one_start =
        (lead_one->v_lead > obstacle_v_start &&
         (lead_one->d_rel - start_stop_state_info.stop_distance_of_leadone()) >
             distance_start);
    // lead_one change: obj stopped adc by cut_in, then leaved
    bool lead_one_change =
        (lead_one->d_rel - desire_distance) > (distance_stop + 1.0);
    bool start_condition = lead_one_start || lead_one_change;

    // 2. Update the state
    if (start_stop_state_info.state() == common::StartStopInfo::CRUISE &&
        stop_condition) {
      // CRUISE --> STOP
      start_stop_state_info.set_state(common::StartStopInfo::STOP);
      // store the distance of leadone
      start_stop_state_info.set_stop_distance_of_leadone(lead_one->d_rel);
      LOG_DEBUG("The distance error of STOP is [%f]m \n",
                lead_one->d_rel - desire_distance);
    } else if (start_stop_state_info.state() == common::StartStopInfo::STOP &&
               start_condition) {
      // STOP --> START
      start_stop_state_info.set_state(common::StartStopInfo::START);
    } else if (start_stop_state_info.state() == common::StartStopInfo::START &&
               cruise_condition) {
      // START --> CRUISE
      start_stop_state_info.set_state(common::StartStopInfo::CRUISE);
    }
  }
  LOG_DEBUG("The start_stop_state_info is [%d] \n",
            start_stop_state_info.state());
  return start_stop_state_info.state();
}

int VisionLongitudinalBehaviorPlanner::GetCIPV(
    const std::shared_ptr<LateralObstacle> &lateral_obstacle,
    const std::string &lc_status) {
  auto hmi_info =
      session_->mutable_planning_context()->mutable_planning_hmi_info();

  int error_id = -1;
  if ((lc_status != "left_lane_change") && (lc_status != "right_lane_change")) {
    if (lateral_obstacle->leadone() != nullptr &&
        lateral_obstacle->leadone()->type != 0) {
      hmi_info->cipv_info.has_cipv = true;
      hmi_info->cipv_info.cipv_id = lateral_obstacle->leadone()->track_id;
      return lateral_obstacle->leadone()->track_id;
    }
  } else {
    if (lateral_obstacle->tleadone() != nullptr &&
        lateral_obstacle->tleadone()->type != 0) {
      hmi_info->cipv_info.has_cipv = true;
      hmi_info->cipv_info.cipv_id = lateral_obstacle->tleadone()->track_id;
      return lateral_obstacle->tleadone()->track_id;
    }
  }
  hmi_info->cipv_info.has_cipv = false;
  hmi_info->cipv_info.cipv_id = error_id;
  return error_id;
}
}  // namespace planning