#include "speed_limit_decider.h"
#include "ego_state_manager.h"
#include "environmental_model.h"
#include "planning_context.h"
namespace {
constexpr double kHighVel = 100 / 3.6;
}
namespace planning {
SpeedLimitDecider::SpeedLimitDecider(
    const EgoPlanningConfigBuilder *config_builder, framework::Session *session)
    : Task(config_builder, session) {
  speed_limit_config_ = config_builder->cast<SpeedLimitConfig>();
  name_ = "SpeedLimitDecider";
}
bool SpeedLimitDecider::Execute() {
  LOG_DEBUG("=======SpeedLimitDecider======= \n");
  const auto &environmental_model = session_->environmental_model();
  const auto ego_state_mgr = environmental_model.get_ego_state_manager();
  double v_cruise = ego_state_mgr->ego_v_cruise();
  v_target_ = v_cruise;
  v_target_type_ = SpeedLimitType::CRUISE;
  // 1. speed limit from map: ramp & split
  CalculateMapSpeedLimit();
  // 2. speed limit from curvature/ lateral_acceleration
  CalculateCurveSpeedLimit();
  // 3. speed limit from static agents
  CalculateStaticAgentLimit();
  // 4. speed limit from intersection
  CalculateIntersectionSpeedLimit();
  // 5. speed limit from uncertainty of perception: perception visibility
  CalculatePerceptVisibSpeedLimit();
  // 6. speed limit from POI
  CalculatePOISpeedLimit();

  auto speed_limit_output = session_->mutable_planning_context()
                                ->mutable_speed_limit_decider_output();
  speed_limit_output->SetSpeedLimit(v_target_, v_target_type_);
  return true;
}

void SpeedLimitDecider::CalculateCurveSpeedLimit() {
  LOG_DEBUG("----CalculateCurveSpeedLimit--- \n");
  const auto &vehicle_param =
      VehicleConfigurationContext::Instance()->get_vehicle_param();
  double steer_ratio = vehicle_param.steer_ratio;
  double wheel_base = vehicle_param.wheel_base;
  const auto &environmental_model = session_->environmental_model();
  const auto ego_state_mgr = environmental_model.get_ego_state_manager();
  double angle_steers = ego_state_mgr->ego_steer_angle();
  double angle_steers_deg = angle_steers * DEG_PER_RAD;
  double v_ego = ego_state_mgr->ego_v();
  double acc_total_max = interp(v_ego, _A_TOTAL_MAX_BP, _A_TOTAL_MAX_V);
  double acc_lat =
      std::pow(v_ego, 2) * angle_steers / (steer_ratio * wheel_base);
  double acc_lon_allowed = std::sqrt(
      std::max(std::pow(acc_total_max, 2) - std::pow(acc_lat, 2), 0.0));

  // And limit the logitudinal velocity for a safe turn
  double acc_lat_max =
      interp(std::abs(angle_steers_deg), _AY_MAX_ABS_BP, _AY_MAX_STEERS);
  // HACK: close v_limit_steering in high vel
  bool is_high_vel = v_ego > kHighVel;
  double v_limit_steering = 100.0;
  if (!is_high_vel) {
    v_limit_steering = std::sqrt((acc_lat_max * steer_ratio * wheel_base) /
                                 std::max(std::abs(angle_steers), 0.001));
  }
  double v_limit_in_turns = v_limit_steering;
  // calculate the velocity limit according to the road curvature
  double preview_x =
      speed_limit_config_.dis_curv + speed_limit_config_.t_curv * v_ego;
  const auto &reference_path_ptr = session_->planning_context()
                                       .lane_change_decider_output()
                                       .coarse_planning_info.reference_path;
  const auto &frenet_ego_state = reference_path_ptr->get_frenet_ego_state();
  std::vector<double> curv_window_vec;
  for (int idx = -3; idx <= 3; ++idx) {
    double curv;
    ReferencePathPoint refpath_pt;
    if (reference_path_ptr->get_reference_point_by_lon(
            frenet_ego_state.s() + preview_x + idx * 2.0, refpath_pt)) {
      curv = std::fabs(refpath_pt.path_point.kappa());
    } else {
      curv = 0.0001;
    }
    curv_window_vec.emplace_back(curv);
  }
  double curv_sum = 0.0;
  for (int ind = 0; ind < curv_window_vec.size(); ++ind) {
    curv_sum = curv_sum + curv_window_vec[ind];
  }
  double avg_curv = curv_sum / curv_window_vec.size();
  double road_radius = 1 / std::max(avg_curv, 0.0001);
  if (road_radius < 400) {
    acc_lat_max = interp(road_radius, _AY_MAX_CURV_BP, _AY_MAX_CURV_V);
  }
  double v_limit_road = std::sqrt(acc_lat_max * road_radius);
  v_limit_in_turns = std::min(v_limit_in_turns, v_limit_road);
  LOG_DEBUG("road_radius is : [%f], acc_lat_max: [%f]\n", road_radius,
            acc_lat_max);
  LOG_DEBUG("angle_steers: [%f], angle_steers_deg: [%f], v_limit_road: [%f]\n",
            angle_steers, angle_steers_deg, v_limit_road);
  JSON_DEBUG_VALUE("v_limit_road", v_limit_road);
  JSON_DEBUG_VALUE("road_radius", road_radius);
  if (v_limit_in_turns < v_target_) {
    v_target_ = v_limit_in_turns;
    v_target_type_ = SpeedLimitType::CURVATURE;
  }
  JSON_DEBUG_VALUE("v_limit_steering", v_limit_steering);
  JSON_DEBUG_VALUE("v_limit_in_turns", v_limit_in_turns);
  auto speed_limit_output = session_->mutable_planning_context()
                                ->mutable_speed_limit_decider_output();
  speed_limit_output->SetSpeedLimitIntoMap(v_limit_in_turns,
                                           SpeedLimitType::CURVATURE);
}

void SpeedLimitDecider::CalculateMapSpeedLimit() {
  LOG_DEBUG("----CalculateMapSpeedLimit for ramp--- \n");
  const auto &environmental_model = session_->environmental_model();
  const auto &route_info_output =
      environmental_model.get_route_info()->get_route_info_output();
  double dis_to_ramp = route_info_output.dis_to_ramp;
  double dis_to_merge = route_info_output.distance_to_first_road_merge;
  bool is_on_ramp = route_info_output.is_on_ramp;
  const auto virtual_lane_manager =
      environmental_model.get_virtual_lane_manager();
  bool is_continuous_ramp = virtual_lane_manager->is_continuous_ramp();

  double v_target_ramp = 40;
  double v_target_near_ramp_zone = 40;
  double pre_acc_dis = speed_limit_config_.pre_accelerate_distance_for_merge;
  bool sdmap_has_curv = true;  //先不考虑匝道内小曲率的情况，后面补充
  // 通过接口获取是否在匝道的信息
  if (is_on_ramp) {
    if (dis_to_merge > pre_acc_dis || is_continuous_ramp) {
      if (sdmap_has_curv) {
        v_target_ramp = speed_limit_config_.v_limit_ramp;
      } else {
        v_target_ramp = speed_limit_config_.straight_ramp_v_limit;
      }
    }
    if (v_target_ramp < v_target_) {
      v_target_ = v_target_ramp;
      v_target_type_ = SpeedLimitType::MAP_ON_RAMP;
    }
    LOG_DEBUG("v_target_ramp : [%f] \n", v_target_ramp);
    JSON_DEBUG_VALUE("v_target_ramp", v_target_ramp);
    JSON_DEBUG_VALUE("dis_to_ramp", dis_to_ramp);
    JSON_DEBUG_VALUE("dis_to_merge", dis_to_merge);
    auto speed_limit_output = session_->mutable_planning_context()
                                  ->mutable_speed_limit_decider_output();
    speed_limit_output->SetSpeedLimitIntoMap(v_target_ramp,
                                             SpeedLimitType::MAP_ON_RAMP);
    return;
  }
  if (dis_to_ramp <= speed_limit_config_.dis_near_ramp_zone) {
    double pre_brake_dis_near_ramp_zone = std::max(
        dis_to_ramp - speed_limit_config_.brake_dis_near_ramp_zone, 0.0);
    v_target_near_ramp_zone = std::pow(
        std::pow(speed_limit_config_.v_limit_near_ramp_zone, 2.0) -
            2 * pre_brake_dis_near_ramp_zone * speed_limit_config_.acc_to_ramp,
        0.5);
  }
  double pre_brake_dis_to_ramp = std::max(dis_to_ramp - 50, 0.0);
  v_target_ramp =
      std::pow(std::pow(speed_limit_config_.v_limit_ramp, 2.0) -
                   2 * pre_brake_dis_to_ramp * speed_limit_config_.acc_to_ramp,
               0.5);
  v_target_ramp = std::min(v_target_near_ramp_zone, v_target_ramp);
  if (v_target_ramp < v_target_) {
    v_target_ = v_target_ramp;
    v_target_type_ = SpeedLimitType::MAP_NEAR_RAMP;
  }
  LOG_DEBUG("dis_to_ramp : [%f] \n", dis_to_ramp);
  LOG_DEBUG("v_target_ramp : [%f] \n", v_target_ramp);
  JSON_DEBUG_VALUE("v_target_ramp", v_target_ramp);
  JSON_DEBUG_VALUE("dis_to_ramp", dis_to_ramp);
  JSON_DEBUG_VALUE("dis_to_merge", dis_to_merge);
  auto speed_limit_output = session_->mutable_planning_context()
                                ->mutable_speed_limit_decider_output();
  speed_limit_output->SetSpeedLimitIntoMap(v_target_ramp,
                                           SpeedLimitType::MAP_NEAR_RAMP);
}

void SpeedLimitDecider::CalculateStaticAgentLimit() {}

void SpeedLimitDecider::CalculateIntersectionSpeedLimit() {
  LOG_DEBUG("----calc_speed_limit_for_intersection--- \n");
  const auto &environmental_model = session_->environmental_model();
  const auto ego_state_mgr = environmental_model.get_ego_state_manager();
  double v_ego = ego_state_mgr->ego_v();
  double v_target_intersection = 40.0;
  const auto virtual_lane_manager =
      environmental_model.get_virtual_lane_manager();
  current_intersection_state_ = virtual_lane_manager->GetIntersectionState();
  if (current_intersection_state_ == planning::common::APPROACH_INTERSECTION ||
      current_intersection_state_ == planning::common::IN_INTERSECTION) {
    if (v_limit_with_intersection_ <
        speed_limit_config_.v_intersection_min_limit) {
      /// v_target_intersection = std::max(v_ego - 3.0, 8.33);
      v_limit_with_intersection_ =
          std::max(v_ego - 3.0, speed_limit_config_.v_intersection_min_limit);
    }
    v_target_intersection = v_limit_with_intersection_;
  } else {
    v_limit_with_intersection_ = 0.0;
  }
  if (v_target_intersection < v_target_) {
    v_target_ = v_target_intersection;
    v_target_type_ = SpeedLimitType::INTERSECTION;
  }
  JSON_DEBUG_VALUE("v_target_intersection", v_target_intersection);
  JSON_DEBUG_VALUE("current_intersection_state",
                   int(current_intersection_state_));
  JSON_DEBUG_VALUE("last_intersection_state", int(last_intersection_state_));
  last_intersection_state_ = current_intersection_state_;
  auto speed_limit_output = session_->mutable_planning_context()
                                ->mutable_speed_limit_decider_output();
  speed_limit_output->SetSpeedLimitIntoMap(v_target_intersection,
                                           SpeedLimitType::INTERSECTION);
}

void SpeedLimitDecider::CalculatePerceptVisibSpeedLimit() {}

void SpeedLimitDecider::CalculatePOISpeedLimit() {}

}  // namespace planning
