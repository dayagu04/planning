#include "speed_limit_decider.h"

#include <algorithm>
#include <cstdint>
#include <memory>
#include <utility>
#include <vector>

#include "agent/agent.h"
#include "behavior_planners/speed_limit_decider/speed_limit_decider_output.h"
#include "debug_info_log.h"
#include "ego_state_manager.h"
#include "environmental_model.h"
#include "planning_context.h"
#include "utils/kd_path.h"
#include "vec2d.h"

namespace planning {
namespace {
constexpr double kEpsilon = 1e-6;
constexpr double kHighVel = 100 / 3.6;
constexpr double kLaneBorrowLimitedSpeed = 5.56;
constexpr double kSpeedlimitScale = 0.6;
constexpr double kSSharpBendRadius = 300.0;
constexpr double kSSharpBendCurvDis = 30.0;
constexpr double kSSharpBendSpeedScaleRatio = 0.8;
constexpr double kTFLSpeedLimitDis = 160.0;
constexpr double kMergePointDetectedDistance = 20.0;
constexpr double kStaticAgentAvoidLimitedSpeedHigh = 10.0;
constexpr double kStaticAgentAvoidLimitedSpeedLow = 4.17;
constexpr double kDynamicAgentAvoidLimitedSpeedHigh = 5.56;
constexpr double kDynamicAgentAvoidLimitedSpeedLow = 2.78;
const std::vector<double> _L_SLOPE_BP{0.0, 40.0};
const std::vector<double> _L_SLOPE_V{0.35, 0.08};
const std::vector<double> _P_SLOPE_BP{0., 40.0};
const std::vector<double> _P_SLOPE_V{0.8, 0.2};
constexpr double follow_time_gap = 1.0;
constexpr double min_follow_distance_m = 3.5;

bool CalculateAgentSLBoundary(
    const std::shared_ptr<planning_math::KDPath> &planned_path,
    const planning_math::Box2d &agent_box, double *const ptr_min_s,
    double *const ptr_max_s, double *const ptr_min_l, double *const ptr_max_l) {
  if (nullptr == ptr_min_s || nullptr == ptr_max_s || nullptr == ptr_min_l ||
      nullptr == ptr_max_l) {
    return false;
  }
  const auto &all_corners = agent_box.GetAllCorners();
  for (const auto &corner : all_corners) {
    Point2D agent_sl{0.0, 0.0};
    Point2D corner_sl{corner.x(), corner.y()};
    if (!planned_path->XYToSL(corner_sl, agent_sl)) {
      continue;
    }
    *ptr_min_s = std::fmin(*ptr_min_s, agent_sl.x);
    *ptr_max_s = std::fmax(*ptr_max_s, agent_sl.x);
    *ptr_min_l = std::fmin(*ptr_min_l, agent_sl.y);
    *ptr_max_l = std::fmax(*ptr_max_l, agent_sl.y);
  }
  if (*ptr_min_s > 200.0 || *ptr_max_s < -200.0 || *ptr_min_l > 50.0 ||
      *ptr_max_l < -50.0) {
    return false;
  }
  return true;
}

bool CalculateAgentSLBoundary(
    const std::shared_ptr<planning_math::KDPath> &planned_path,
    const planning::agent::Agent &agent, double *const ptr_min_s,
    double *const ptr_max_s, double *const ptr_min_l, double *const ptr_max_l) {
  const auto &agent_box = agent.box();
  bool is_success = CalculateAgentSLBoundary(planned_path, agent_box, ptr_min_s,
                                             ptr_max_s, ptr_min_l, ptr_max_l);
  return is_success;
}

bool LateralCollisionCheck(
    const double &start_s, const double &end_s, const double &agent_min_l,
    const double ego_length, const double ego_width,
    const std::shared_ptr<planning_math::KDPath> lat_path_coord) {
  const double sampling_interval_s = 1.0;
  for (double s = start_s; s <= end_s; s += sampling_interval_s) {
    double box_x = 0.0;
    double box_y = 0.0;
    lat_path_coord->SLToXY(s, 0.0, &box_x, &box_y);
    double theta = lat_path_coord->GetPathCurveHeading(s);
    planning_math::Box2d ego_box({box_x, box_y}, theta, ego_length, ego_width);
    const auto &all_corners = ego_box.GetAllCorners();
    double min_l = std::numeric_limits<double>::max();
    double max_l = -min_l;

    for (const auto &corner : all_corners) {
      double ego_s = 0.0;
      double ego_l = 0.0;
      lat_path_coord->XYToSL(corner.x(), corner.y(), &ego_s, &ego_l);
      min_l = std::fmin(min_l, ego_l);
      max_l = std::fmax(max_l, ego_l);
    }

    // check collision
    if ((agent_min_l < 0 && min_l < agent_min_l) ||
        (agent_min_l >= 0 && max_l > agent_min_l)) {
      return true;
    }
  }
  return false;
}

double CalcDesiredVelocity(const double d_rel, const double d_des,
                           const double v_lead, const double v_ego) {
  // *** compute desired speed ***
  double v_lead_clip = std::max(v_lead, 0.0);
  const double max_runaway_speed = -2.0;
  double l_slope = interp(v_lead, _L_SLOPE_BP, _L_SLOPE_V);
  double p_slope = interp(v_lead, _P_SLOPE_BP, _P_SLOPE_V);
  double x_linear_to_parabola = p_slope / std::pow(l_slope, 2);
  double x_parabola_offset = p_slope / (2 * std::pow(l_slope, 2));

  double v_rel = v_ego - v_lead;
  double v_rel_des = 0.0;
  double soft_brake_distance = 0.0;
  if (d_rel < d_des) {
    double v_rel_des_1 = (-max_runaway_speed) / d_des * (d_rel - d_des);
    double v_rel_des_2 = (d_rel - d_des) * l_slope / 3.0;
    v_rel_des = std::min(v_rel_des_1, v_rel_des_2);
    v_rel_des = std::max(v_rel_des, max_runaway_speed);
    soft_brake_distance = d_rel;
  } else if (d_rel < d_des + x_linear_to_parabola) {
    v_rel_des = (d_rel - d_des) * l_slope;
    v_rel_des = std::max(v_rel_des, max_runaway_speed);
    soft_brake_distance = v_rel / l_slope + d_des;
  } else {
    v_rel_des = std::sqrt(2 * (d_rel - d_des - x_parabola_offset) * p_slope);
    soft_brake_distance =
        std::pow(v_rel, 2) / (2 * p_slope) + x_parabola_offset + d_des;
  }
  // compute desired speed
  double v_target = v_rel_des + v_lead;
  return v_target;
}
}  // namespace

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
  const auto plan_init_point = ego_state_mgr->planning_init_point();
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
  // 7. speed limit from lane borrow agent
  CalculateLaneBorrowSpeedLimit();
  // 8. speed limit from tfl distance
  CalculateSpeedLimitFromTFLDis();
  // 9. speed limit from avoid agent
  CalculateAvoidAgentSpeedLimit();

  auto speed_limit_output = session_->mutable_planning_context()
                                ->mutable_speed_limit_decider_output();
  speed_limit_output->SetSpeedLimit(v_target_, v_target_type_);
  auto &ad_info = session_->mutable_planning_context()
                      ->mutable_planning_hmi_info()
                      ->ad_info;
  if (SpeedLimitType::CURVATURE == v_target_type_) {
    ad_info.is_curva = true;
  } else {
    ad_info.is_curva = false;
  }
  return true;
}

bool SpeedLimitDecider::IsSSharpBend(
    const std::vector<CurvInfo> &preview_curv_info_vec) {
  std::vector<std::pair<double, double>> pos_curv_list;
  std::vector<std::pair<double, double>> neg_curv_list;
  for (int idx = 0; idx < preview_curv_info_vec.size(); idx++) {
    if (preview_curv_info_vec[idx].curv_sign > 0 &&
        (1.0 / preview_curv_info_vec[idx].curv) < kSSharpBendRadius) {
      pos_curv_list.emplace_back(std::make_pair(
          preview_curv_info_vec[idx].s, preview_curv_info_vec[idx].curv));
    }
    if (preview_curv_info_vec[idx].curv_sign < 0 &&
        (1.0 / preview_curv_info_vec[idx].curv) < kSSharpBendRadius) {
      neg_curv_list.emplace_back(std::make_pair(
          preview_curv_info_vec[idx].s, preview_curv_info_vec[idx].curv));
    }
  }
  if (pos_curv_list.empty() || neg_curv_list.empty()) {
    return false;
  }

  auto comp_curv = [](std::pair<double, double> &a,
                      std::pair<double, double> &b) {
    return a.second > b.second;
  };
  std::sort(pos_curv_list.begin(), pos_curv_list.end(), comp_curv);
  std::sort(neg_curv_list.begin(), neg_curv_list.end(), comp_curv);
  if (std::fabs(pos_curv_list[0].first - neg_curv_list[0].first) <
      kSSharpBendCurvDis) {
    return true;
  }

  return false;
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
      interp(std::fabs(angle_steers_deg), _AY_MAX_ABS_BP, _AY_MAX_STEERS);
  // HACK: close v_limit_steering in high vel
  bool is_high_vel = v_ego > kHighVel;
  double v_limit_steering = 100.0;
  if (!is_high_vel) {
    v_limit_steering = std::sqrt((acc_lat_max * steer_ratio * wheel_base) /
                                 std::max(std::fabs(angle_steers), 0.001));
  }
  double v_limit_in_turns = v_limit_steering;
  // calculate the velocity limit according to the road curvature
  double preview_x = speed_limit_config_.dis_curv;
  const auto &reference_path_ptr = session_->planning_context()
                                       .lane_change_decider_output()
                                       .coarse_planning_info.reference_path;
  if (reference_path_ptr == nullptr) {
    JSON_DEBUG_VALUE("v_limit_steering", v_limit_steering);
    JSON_DEBUG_VALUE("v_limit_in_turns", v_limit_in_turns);
    auto speed_limit_output = session_->mutable_planning_context()
                                ->mutable_speed_limit_decider_output();
    speed_limit_output->SetSpeedLimitIntoMap(v_limit_in_turns,
                                           SpeedLimitType::CURVATURE);
    return;
  }
  const auto &frenet_ego_state = reference_path_ptr->get_frenet_ego_state();
  double ego_start_s = frenet_ego_state.s();
  std::vector<CurvInfo> preview_curv_info_vec;
  for (int idx = 0; idx * 2.0 < preview_x; idx++) {
    CurvInfo one_curv_info;
    // calc curv abs and using average curv abs in curv window
    std::vector<double> curv_window_vec;
    for (int j = -3; j <= 3; j++) {
      double curv;
      ReferencePathPoint refpath_pt;
      if (reference_path_ptr->get_reference_point_by_lon(
              ego_start_s + idx * 2.0 + j * 2.0, refpath_pt)) {
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
    one_curv_info.curv = avg_curv;

    // calc curv direction(-1 or 1, 0 for except)
    ReferencePathPoint refpath_pt;
    if (reference_path_ptr->get_reference_point_by_lon(ego_start_s + idx * 2.0,
                                                       refpath_pt)) {
      one_curv_info.curv_sign = refpath_pt.path_point.kappa() > 0 ? 1 : -1;
    } else {
      one_curv_info.curv_sign = 0;
    }

    one_curv_info.s = idx * 2.0;
    preview_curv_info_vec.emplace_back(one_curv_info);
  }
  /* double curv_sum = 0.0;
  for (int ind = 0; ind < curv_window_vec.size(); ++ind) {
    curv_sum = curv_sum + curv_window_vec[ind];
  }
  double avg_curv = curv_sum / curv_window_vec.size();
  double road_radius = 1 / std::max(avg_curv, 0.0001);*/

  auto &debug_info_pb = DebugInfoManager::GetInstance().GetDebugInfoPb();
  debug_info_pb->clear_dis_curv_list();
  for (int j = 0; j < preview_curv_info_vec.size(); j++) {
    planning::common::DoublePair *one_curv = debug_info_pb->add_dis_curv_list();
    one_curv->set_first(preview_curv_info_vec[j].s);
    one_curv->set_second(preview_curv_info_vec[j].curv);
  }

  double v_limit_road = 40.0;
  double road_radius = 10000.0;
  bool is_s_bend = IsSSharpBend(preview_curv_info_vec);
  double max_curv = 0.0001;
  for (int idx = 0; idx < preview_curv_info_vec.size(); idx++) {
    if (preview_curv_info_vec[idx].curv > max_curv) {
      max_curv = preview_curv_info_vec[idx].curv;
    }
  }
  road_radius = 1 / std::max(max_curv, 0.0001);
  if (road_radius < 400) {
    acc_lat_max = interp(road_radius, _AY_MAX_CURV_BP, _AY_MAX_CURV_V);
  }
  v_limit_road = std::sqrt(acc_lat_max * road_radius);
  if (is_s_bend) {
    v_limit_road = v_limit_road * kSSharpBendSpeedScaleRatio;
  }
  v_limit_in_turns = std::min(v_limit_in_turns, v_limit_road);
  LOG_DEBUG("road_radius is : [%f], acc_lat_max: [%f]\n", road_radius,
            acc_lat_max);
  LOG_DEBUG("angle_steers: [%f], angle_steers_deg: [%f], v_limit_road: [%f]\n",
            angle_steers, angle_steers_deg, v_limit_road);
  JSON_DEBUG_VALUE("v_limit_road", v_limit_road);
  JSON_DEBUG_VALUE("road_radius", road_radius);
  JSON_DEBUG_VALUE("is_s_bend", is_s_bend ? 1 : 0);
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
  bool sdmap_has_curv = true;  // 先不考虑匝道内小曲率的情况，后面补充
  // 通过接口获取是否在匝道的信息
  if (is_on_ramp) {
    if (dis_to_merge > pre_acc_dis || is_continuous_ramp) {
      if (sdmap_has_curv) {
        v_target_ramp = speed_limit_config_.v_limit_ramp;
      } else {
        v_target_ramp = speed_limit_config_.straight_ramp_v_limit;
      }
    } else {
      ReferencePathPoint detect_merge_front_pnt;
      double ego_s;
      const auto &current_lane = virtual_lane_manager->get_current_lane();
      if (current_lane != nullptr && current_lane->get_reference_path() != nullptr) {
        ego_s = current_lane->get_reference_path()->get_frenet_ego_state().s();
      } else {
        ego_s = -100.0;
      }
      if (ego_s > 0 && current_lane->get_reference_path()->get_reference_point_by_lon(
          ego_s + dis_to_merge + kMergePointDetectedDistance, detect_merge_front_pnt)) {
        double nearest_s = 0;
        double nearest_l = 0;
        double search_distance = 50.0;
        double max_heading_diff = PI / 4;
        double detected_heading_angle = detect_merge_front_pnt.path_point.theta();
        ad_common::math::Vec2d detected_point(detect_merge_front_pnt.path_point.x(), detect_merge_front_pnt.path_point.y());
        if (environmental_model.get_route_info()->get_sdmap_valid()) {
          const auto &sd_map = environmental_model.get_route_info()->get_sd_map();
          const auto segment = sd_map.GetNearestRoadWithHeading(
            detected_point, search_distance, detected_heading_angle, max_heading_diff,
            nearest_s, nearest_l);
          if (segment != nullptr && segment ->priority() != SdMapSwtx::RoadPriority::EXPRESSWAY
             && segment ->priority() != SdMapSwtx::RoadPriority::CITY_EXPRESSWAY) {
              v_target_ramp = speed_limit_config_.v_limit_ramp;
          }
        }
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

void SpeedLimitDecider::CalculateSpeedLimitFromTFLDis() {
  LOG_DEBUG("----calc_speed_limit_from_tfl_dis--- \n");
  double v_limit_tfl_dis = 40.0;
  const auto &environmental_model = session_->environmental_model();
  const auto tfl_manager =
      environmental_model.get_traffic_light_decision_manager();
  const auto traffic_status = tfl_manager->GetTrafficStatus();
  double dis_tfl = tfl_manager->GetNearestTFLDis();
  if (dis_tfl < kTFLSpeedLimitDis) {
    v_limit_tfl_dis = 55 / 3.6;
    if (traffic_status.go_straight == 1 || traffic_status.go_straight == 41 ||
      traffic_status.go_straight == 11 || traffic_status.go_straight == 10) {
      v_limit_tfl_dis = 50 / 3.6;
    }
  }
  if (v_limit_tfl_dis < v_target_) {
    v_target_ = v_limit_tfl_dis;
    v_target_type_ = SpeedLimitType::NEAR_TFL;
  }
  JSON_DEBUG_VALUE("dis_to_tfl", dis_tfl);
  JSON_DEBUG_VALUE("v_limit_tfl_dis", v_limit_tfl_dis);
  auto speed_limit_output = session_->mutable_planning_context()
                                ->mutable_speed_limit_decider_output();
  speed_limit_output->SetSpeedLimitIntoMap(v_limit_tfl_dis,
                                           SpeedLimitType::NEAR_TFL);
}

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

void SpeedLimitDecider::CalculateLaneBorrowSpeedLimit() {
  // get lane borrow agent
  bool is_exist_lane_borrow_agent = false;
  const auto &lane_borrow_output =
      session_->planning_context().lane_borrow_decider_output();
  const auto &blocked_obs_id = lane_borrow_output.blocked_obs_id;
  const auto borrow_direction = lane_borrow_output.borrow_direction;

  // get lane borrow agent
  const auto agent_manager =
      session_->environmental_model().get_agent_manager();
  if (agent_manager == nullptr) {
    return;
  }

  std::vector<SpeedLimitAgent> speed_limit_agents;
  std::vector<const agent::Agent *> lane_borrow_agents;
  for (const int lane_borrow_agent_id : blocked_obs_id) {
    const auto lane_borrow_agent =
        agent_manager->GetAgent(lane_borrow_agent_id);
    if (lane_borrow_agent != nullptr) {
      lane_borrow_agents.emplace_back(lane_borrow_agent);
    }
  }

  // get lateral path
  const auto &planned_kd_path =
      session_->planning_context().motion_planner_output().lateral_path_coord;
  if (planned_kd_path == nullptr) {
    return;
  }

  const auto &vehicle_param =
      VehicleConfigurationContext::Instance()->get_vehicle_param();
  const auto ego_state_mgr =
      session_->environmental_model().get_ego_state_manager();
  double v_ego = ego_state_mgr->ego_v();

  for (auto agent : lane_borrow_agents) {
    double min_s_by_lat_path = std::numeric_limits<double>::max();
    double max_s_by_lat_path = std::numeric_limits<double>::lowest();
    double min_l_by_lat_path = std::numeric_limits<double>::max();
    double max_l_by_lat_path = std::numeric_limits<double>::lowest();
    bool is_success = CalculateAgentSLBoundary(
        planned_kd_path, *agent, &min_s_by_lat_path, &max_s_by_lat_path,
        &min_l_by_lat_path, &max_l_by_lat_path);
    if (!is_success) {
      continue;
    }
    double min_lat_l_by_lat_path = 0.0;
    if (borrow_direction == LEFT_BORROW) {
      min_lat_l_by_lat_path = max_l_by_lat_path;
      // (agent_l * min_l_by_lat_path) > 0 ? min_l_by_lat_path : 0;
    } else if (borrow_direction == RIGHT_BORROW) {
      min_lat_l_by_lat_path = min_l_by_lat_path;
      // (agent_l * max_l_by_lat_path) > 0 ? max_l_by_lat_path : 0;
    } else {
      continue;
    }

    // lateral collision check
    double ego_length = vehicle_param.length;
    double ego_width = vehicle_param.width;
    double agent_half_width = 0.5 * agent->width();
    double start_s = min_s_by_lat_path;
    double end_s = start_s + ego_length + 0.5 * agent->length();
    bool is_collision =
        LateralCollisionCheck(start_s, end_s, min_lat_l_by_lat_path, ego_length,
                              ego_width, planned_kd_path);

    // calc limit speed
    std::array<double, 2> xp{agent_half_width, 1.8};
    std::array<double, 2> fp{kSpeedlimitScale * v_ego, v_ego};
    double v_limit = interp(fabs(min_lat_l_by_lat_path), xp, fp);
    v_limit = std::max(v_limit, kLaneBorrowLimitedSpeed);

    SpeedLimitAgent speed_limit_agent;
    speed_limit_agent.id = agent->agent_id();
    speed_limit_agent.min_s = min_s_by_lat_path;
    speed_limit_agent.v_limit = v_limit;
    speed_limit_agent.is_collison = is_collision;
    speed_limit_agents.emplace_back(speed_limit_agent);

    // sort
    auto comp = [](const SpeedLimitAgent &a, const SpeedLimitAgent &b) {
      return a.min_s < b.min_s;
    };
    std::sort(speed_limit_agents.begin(), speed_limit_agents.end(), comp);
  }

  if (speed_limit_agents.empty()) {
    JSON_DEBUG_VALUE("lane_borrow_agent_id", -1.0)
    JSON_DEBUG_VALUE("lane_borrow_agent_v_limit", 100.0)
    return;
  }
  // only use nearest lane borrow agent
  auto it = speed_limit_agents.begin();
  if (it != speed_limit_agents.end()) {
    if (it->v_limit < v_target_) {
      v_target_ = it->v_limit;
      v_target_type_ = SpeedLimitType::LANE_BORROW;
      JSON_DEBUG_VALUE("lane_borrow_agent_id", it->id)
      JSON_DEBUG_VALUE("lane_borrow_agent_v_limit", v_target_)
    }
  }
}

void SpeedLimitDecider::CalculateAvoidAgentSpeedLimit() {
  // get avoid agent id
  const auto avoid_agents_info = session_->planning_context()
                                     .lateral_obstacle_decider_output()
                                     .obstacle_intrusion_distance_thr;
  const auto avoid_ids =
      session_->planning_context().lateral_offset_decider_output().avoid_ids;

  const auto &lane_borrow_output =
      session_->planning_context().lane_borrow_decider_output();
  if (lane_borrow_output.is_in_lane_borrow_status) {
    return;
  }

  // get lane borrow agent
  const auto agent_manager =
      session_->environmental_model().get_agent_manager();
  if (agent_manager == nullptr) {
    return;
  }

  const auto &virtual_lane_manager =
      session_->environmental_model().get_virtual_lane_manager();
  if (virtual_lane_manager == nullptr) {
    return;
  }
  const auto &current_lane = virtual_lane_manager->get_current_lane();
  if (current_lane == nullptr) {
    return;
  }
  const auto &current_lane_coord = current_lane->get_lane_frenet_coord();
  if (current_lane_coord == nullptr) {
    return;
  }

  std::vector<const agent::Agent *> avoid_agents;
  for (const auto avoid_agent_id : avoid_ids) {
    const auto avoid_agent = agent_manager->GetAgent(avoid_agent_id);
    if (avoid_agent != nullptr) {
      avoid_agents.emplace_back(avoid_agent);
    }
  }

  std::vector<SpeedLimitAgent> speed_limit_agents;

  // get lateral path
  const auto &planned_kd_path =
      session_->planning_context().motion_planner_output().lateral_path_coord;
  if (planned_kd_path == nullptr) {
    return;
  }

  const auto &vehicle_param =
      VehicleConfigurationContext::Instance()->get_vehicle_param();
  const auto ego_state_mgr =
      session_->environmental_model().get_ego_state_manager();
  double v_ego = ego_state_mgr->ego_v();
  const auto init_point = ego_state_mgr->planning_init_point();

  for (auto avoid_agent : avoid_agents) {
    const bool is_static = avoid_agent->is_static();
    // const double speed_diff = v_ego - avoid_agent->speed();

    // bool is_need_v_hold = false;
    // double speed_buffer = 0.0;
    // if (is_static) {
    //   if (speed_diff >= kStaticAgentAvoidLimitedSpeedHigh) {
    //     speed_buffer = kStaticAgentAvoidLimitedSpeedHigh;
    //   } else if (speed_diff >= kStaticAgentAvoidLimitedSpeedLow) {
    //     is_need_v_hold = true;
    //   } else {
    //     continue;
    //   }
    // } else {
    //   if (speed_diff < kDynamicAgentAvoidLimitedSpeedLow) {
    //     continue;
    //   } else if (speed_diff < kDynamicAgentAvoidLimitedSpeedHigh) {
    //     is_need_v_hold = true;
    //   } else {
    //     speed_buffer = kDynamicAgentAvoidLimitedSpeedHigh;
    //   }
    // }

    Point2D agent_point(avoid_agent->x(), avoid_agent->y());
    Point2D frenet_point;
    if (!(current_lane_coord->XYToSL(agent_point, frenet_point))) {
      continue;
    }

    double ego_s, ego_l = 0.0;
    if (!(current_lane_coord->XYToSL(init_point.x, init_point.y, &ego_s,
                                     &ego_l))) {
      continue;
    }

    double min_s_by_lat_path = std::numeric_limits<double>::max();
    double max_s_by_lat_path = std::numeric_limits<double>::lowest();
    double min_l_by_lat_path = std::numeric_limits<double>::max();
    double max_l_by_lat_path = std::numeric_limits<double>::lowest();
    bool is_success = CalculateAgentSLBoundary(
        current_lane_coord, *avoid_agent, &min_s_by_lat_path,
        &max_s_by_lat_path, &min_l_by_lat_path, &max_l_by_lat_path);
    if (!is_success) {
      continue;
    }

    double min_lat_l = 0.0;
    if (frenet_point.y > planning_math::kMathEpsilon) {
      min_lat_l =
          (frenet_point.y * min_l_by_lat_path) > 0 ? min_l_by_lat_path : 0;
    } else if (frenet_point.y < planning_math::kMathEpsilon) {
      min_lat_l =
          (frenet_point.y * max_l_by_lat_path) > 0 ? max_l_by_lat_path : 0;
    } else {
      continue;
    }

    double agent_half_width = 0.5 * avoid_agent->width();
    double lane_half_width = 0.5 * current_lane->width_by_s(min_s_by_lat_path);
    // calc limit speed
    double s_desired =
        std::max(init_point.v * follow_time_gap + min_follow_distance_m,
                 min_follow_distance_m);
    const auto &vehicle_param =
        VehicleConfigurationContext::Instance()->get_vehicle_param();
    double d_rel = min_s_by_lat_path - ego_s - 0.5 * vehicle_param.length;
    double v_follow_desired =
        CalcDesiredVelocity(d_rel, s_desired, avoid_agent->speed(), v_ego);
    double invade_dis = avoid_agent->is_static() ? 0.8 : 0.4;
    if (avoid_agents_info.find(avoid_agent->agent_id()) !=
        avoid_agents_info.end()) {
      invade_dis = std::max(avoid_agents_info.at(avoid_agent->agent_id()), 0.0);
    }
    std::array<double, 2> xp{lane_half_width - invade_dis,
                             lane_half_width + 0.2};
    std::array<double, 2> fp{v_follow_desired, v_ego};
    double v_limit = interp(fabs(min_lat_l), xp, fp);

    std::array<double, 2> xp1{-0.2, invade_dis};
    std::array<double, 2> fp1{kStaticAgentAvoidLimitedSpeedHigh,
                              kStaticAgentAvoidLimitedSpeedLow};
    std::array<double, 2> fp2{kDynamicAgentAvoidLimitedSpeedHigh,
                              kDynamicAgentAvoidLimitedSpeedLow};
    double v_limit_lower =
        avoid_agent->is_static()
            ? interp(lane_half_width - fabs(min_lat_l), xp1, fp1)
            : interp(lane_half_width - fabs(min_lat_l), xp1, fp2);

    // const double v_limit_lower = avoid_agent->is_static()
    //                                  ? kStaticAgentAvoidLimitedSpeedHigh
    //                                  : kDynamicAgentAvoidLimitedSpeedHigh;
    v_limit = std::max(v_limit, v_limit_lower);

    SpeedLimitAgent speed_limit_agent;
    speed_limit_agent.id = avoid_agent->agent_id();
    speed_limit_agent.min_s = min_s_by_lat_path - ego_s;
    speed_limit_agent.v_limit = v_limit;
    speed_limit_agent.is_need_v_hold = false;
    speed_limit_agent.v_follow_desired = v_follow_desired;
    speed_limit_agents.emplace_back(speed_limit_agent);

    // sort
    auto comp = [](const SpeedLimitAgent &a, const SpeedLimitAgent &b) {
      return a.min_s < b.min_s;
    };
    std::sort(speed_limit_agents.begin(), speed_limit_agents.end(), comp);
  }

  if (speed_limit_agents.empty()) {
    v_avoid_hold_ = 0.0;
    JSON_DEBUG_VALUE("avoid_agent_id", -1.0)
    JSON_DEBUG_VALUE("avoid_agent_v_limit", 100.0)
    JSON_DEBUG_VALUE("aovid_agent_v_follow_desire", 100.0)
    return;
  }

  auto update_speed_limit = [&](double new_speed,
                                const SpeedLimitAgent &agent) {
    v_target_ = new_speed;
    v_target_type_ = SpeedLimitType::AVOID_AGENT;
    JSON_DEBUG_VALUE("avoid_agent_id", agent.id);
    JSON_DEBUG_VALUE("avoid_agent_v_limit", v_target_);
    JSON_DEBUG_VALUE("aovid_agent_v_follow_desire", agent.v_follow_desired);
  };

  // bool is_first_agent = true;
  double v_avoid_tmp = 40.0;
  for (const auto &agent : speed_limit_agents) {
    // if (is_first_agent) {
    //   if (agent.is_need_v_hold) {
    //     if (v_avoid_hold_ < 0.1) {
    //       v_avoid_hold_ = v_ego;
    //     }
    //     v_avoid_tmp = std::fmin(v_avoid_hold_, v_avoid_tmp);
    //   } else {
    //     v_avoid_tmp = agent.v_limit;
    //     v_avoid_hold_ = 0.0;
    //   }
    //   if (v_avoid_tmp < v_target_) {
    //     update_speed_limit(v_avoid_tmp, agent);
    //   }
    //   is_first_agent = false;
    //   continue;
    // }
    if (agent.v_limit < v_target_) {
      update_speed_limit(agent.v_limit, agent);
    }
  }
}
}  // namespace planning
