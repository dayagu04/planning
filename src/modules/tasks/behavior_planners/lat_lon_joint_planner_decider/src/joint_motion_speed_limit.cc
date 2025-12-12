#include "joint_motion_speed_limit.h"

#include <algorithm>
#include <cmath>
#include <unordered_set>

#include "ego_state_manager.h"
#include "environmental_model.h"
#include "math/math_utils.h"
#include "planning_context.h"
#include "traffic_light_decision_manager.h"
#include "utils/kd_path.h"
#include "vehicle_config_context.h"

namespace planning {

namespace {

bool CheckClustersConsecutiveDiffSlidingWindowImpl(
    const std::map<int, ConstructionAgentClusterArea>& cluster_map,
    const std::shared_ptr<planning_math::KDPath>& planned_kd_path,
    bool entering) {
  if (planned_kd_path == nullptr) {
    return false;
  }
  for (const auto& [cluster_id, construction_area] : cluster_map) {
    const auto& pts = construction_area.points;
    if (pts.size() < 3) {
      continue;
    }
    // Convert all points to l coordinates
    std::vector<double> ls;
    ls.reserve(pts.size());
    for (const auto& pt : pts) {
      double s = 0.0, l = 0.0;
      if (!planned_kd_path->XYToSL(pt.x, pt.y, &s, &l)) {
        continue;
      }
      ls.push_back(l);
    }
    if (ls.size() < 3) {
      continue;
    }
    // Sliding window to check 3 consecutive points
    for (size_t i = 0; i + 2 < ls.size(); ++i) {
      double diff01 = std::abs(ls[i + 1] - ls[i]);
      double diff12 = std::abs(ls[i + 2] - ls[i + 1]);
      if (diff01 > JointMotionSpeedLimit::kCAInvadeLatDisDiffThr &&
          diff12 > JointMotionSpeedLimit::kCAInvadeLatDisDiffThr) {
        double abs_l0 = std::abs(ls[i]);
        double abs_l1 = std::abs(ls[i + 1]);
        double abs_l2 = std::abs(ls[i + 2]);
        if (entering && abs_l0 > abs_l1 && abs_l1 > abs_l2 &&
            abs_l0 < JointMotionSpeedLimit::kCAInvadeLatMaxDis) {
          return true;
        }
        if (!entering && abs_l0 < abs_l1 && abs_l1 < abs_l2 &&
            abs_l0 < JointMotionSpeedLimit::kCAInvadeLatMinDis) {
          return true;
        }
      }
    }
  }
  return false;
}

}  // namespace

JointMotionSpeedLimit::JointMotionSpeedLimit(
    const EgoPlanningConfigBuilder* config_builder, framework::Session* session)
    : session_(session) {
  speed_limit_config_ = config_builder->cast<SpeedLimitConfig>();
  v_target_ = 0.0;
  v_target_type_ = 0;  // CRUISE
  v_cruise_limit_ = 120.0;
  poi_v_limit_set_ = false;
  v_limit_with_intersection_ = 0.0;
  construction_strong_deceleration_mode = false;
  construction_strong_mode_frame_count_ = 0;
  is_function_fading_away_ = false;
  last_vel_function_fading_away_ = 0.0;
  vel_slope_filter_function_fading_away_.Init(
      speed_limit_config_.min_acc_function_fading_away, 0.0, 1e-3, 40, 0.1);
}

JointMotionSpeedLimit::Result JointMotionSpeedLimit::CalculateSpeedLimit() {
  const auto& environmental_model = session_->environmental_model();
  const auto ego_state_mgr = environmental_model.get_ego_state_manager();
  double v_cruise = ego_state_mgr->ego_v_cruise();

  v_target_ = v_cruise;
  v_target_type_ = 0;  // CRUISE

  // 1. Speed limit from map: ramp & split
  CalculateMapSpeedLimit();

  // 2. Speed limit from curvature/lateral acceleration
  CalculateCurveSpeedLimit();

  // 3. Speed limit from intersection
  CalculateIntersectionSpeedLimit();

  // 4. Speed limit from POI
  CalculatePOISpeedLimit();

  // 5. Speed limit from TFL distance
  CalculateSpeedLimitFromTFLDis();

  // 6. Speed limit from construction zone
  CalculateConstructionZoneSpeedLimit();

  // 7. Speed limit for LCC/NOA fading away
  CalculateFunctionFadingAwaySpeedLimit();

  Result result;
  result.speed_limit = v_target_;
  result.type = v_target_type_;
  return result;
}

bool JointMotionSpeedLimit::IsSSharpBend(
    const std::vector<JointCurvInfo>& preview_curv_info_vec) const {
  std::vector<std::pair<double, double>> pos_curv_list;
  std::vector<std::pair<double, double>> neg_curv_list;

  for (size_t idx = 0; idx < preview_curv_info_vec.size(); idx++) {
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

  auto comp_curv = [](std::pair<double, double>& a,
                      std::pair<double, double>& b) {
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

double JointMotionSpeedLimit::JudgeCurvBySDProMap() const {
  if (!speed_limit_config_.enable_sdmap_curv_v_adjust) {
    return 300.0;
  }
  if (!session_->environmental_model().get_route_info()->get_sdpromap_valid()) {
    return 300.0;
  }

  ad_common::math::Vec2d current_point;
  const auto& ego_state =
      session_->environmental_model().get_ego_state_manager();
  const auto& pose = ego_state->location_enu();
  current_point.set_x(pose.position.x);
  current_point.set_y(pose.position.y);

  const auto& sdpro_map =
      session_->environmental_model().get_route_info()->get_sdpro_map();
  double nearest_s = 0;
  double nearest_l = 0;
  const double search_distance = 50.0;
  const double max_heading_diff = M_PI / 4;
  const double ego_heading_angle = ego_state->heading_angle();

  const auto current_segment = sdpro_map.GetNearestLinkWithHeading(
      current_point, search_distance, ego_heading_angle, max_heading_diff,
      nearest_s, nearest_l);
  if (!current_segment) {
    return 300.0;
  }

  std::vector<std::pair<double, double>> curv_list;
  curv_list =
      sdpro_map.GetCurvatureList(current_segment->id(), nearest_s,
                                 speed_limit_config_.search_sdmap_curv_dis);

  double min_curv_radius = 10000.0;
  for (size_t i = 0; i < curv_list.size(); i++) {
    double one_curv_radius = 1.0 / (std::abs(curv_list[i].second));
    if (one_curv_radius < min_curv_radius) {
      min_curv_radius = one_curv_radius;
    }
  }
  return min_curv_radius;
}

void JointMotionSpeedLimit::CalculateCurveSpeedLimit() {
  const auto& vehicle_param =
      VehicleConfigurationContext::Instance()->get_vehicle_param();
  double steer_ratio = vehicle_param.steer_ratio;
  double wheel_base = vehicle_param.wheel_base;

  const auto& environmental_model = session_->environmental_model();
  const auto ego_state_mgr = environmental_model.get_ego_state_manager();
  double angle_steers = ego_state_mgr->ego_steer_angle();
  double angle_steers_deg = angle_steers * DEG_PER_RAD;
  double v_ego = ego_state_mgr->ego_v();

  double acc_total_max = interp(v_ego, _A_TOTAL_MAX_BP, _A_TOTAL_MAX_V);
  double acc_lat =
      std::pow(v_ego, 2) * angle_steers / (steer_ratio * wheel_base);
  double acc_lon_allowed = std::sqrt(
      std::max(std::pow(acc_total_max, 2) - std::pow(acc_lat, 2), 0.0));

  double acc_lat_max =
      interp(std::fabs(angle_steers_deg), _AY_MAX_ABS_BP, _AY_MAX_STEERS);

  bool is_high_vel = v_ego > kHighVel;
  double v_limit_steering = 100.0;
  if (!is_high_vel) {
    v_limit_steering = std::sqrt((acc_lat_max * steer_ratio * wheel_base) /
                                 std::max(std::fabs(angle_steers), 0.001));
  }
  double v_limit_in_turns = v_limit_steering;

  const auto& reference_path_ptr = session_->planning_context()
                                       .lane_change_decider_output()
                                       .coarse_planning_info.reference_path;
  if (reference_path_ptr == nullptr) {
    return;
  }

  const auto& frenet_ego_state = reference_path_ptr->get_frenet_ego_state();
  double ego_start_s = frenet_ego_state.s();
  double preview_x = speed_limit_config_.dis_curv;

  std::vector<JointCurvInfo> preview_curv_info_vec;
  for (int idx = 0; idx * 2.0 < preview_x; idx++) {
    JointCurvInfo one_curv_info;
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
    for (size_t ind = 0; ind < curv_window_vec.size(); ++ind) {
      curv_sum = curv_sum + curv_window_vec[ind];
    }
    double avg_curv = curv_sum / curv_window_vec.size();
    one_curv_info.curv = avg_curv;

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

  double v_limit_road = 40.0;
  double road_radius = 10000.0;
  bool is_s_bend = IsSSharpBend(preview_curv_info_vec);
  double max_curv = 0.0001;

  for (size_t idx = 0; idx < preview_curv_info_vec.size(); idx++) {
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

  if (v_limit_in_turns < v_target_) {
    v_target_ = v_limit_in_turns;
    v_target_type_ = 1;  // CURVATURE
  }
}

void JointMotionSpeedLimit::CalculateMapSpeedLimit() {
  const auto& environmental_model = session_->environmental_model();
  const auto& function_state_machine_info =
      environmental_model.get_local_view().function_state_machine_info;
  double v_cruise_fsm =
      function_state_machine_info.pilot_req.acc_curise_real_spd;

  const auto& route_info_output =
      environmental_model.get_route_info()->get_route_info_output();
  double dis_to_ramp = route_info_output.dis_to_ramp;
  double dis_to_merge = route_info_output.distance_to_first_road_merge;
  bool is_on_ramp = route_info_output.is_on_ramp;

  uint64_t ramp_link_id = -1;
  double ramp_v_limit = 120;
  const auto& split_region_info_list = route_info_output.split_region_info_list;

  if (dis_to_ramp < 2000.0) {
    for (size_t i = 0; i < split_region_info_list.size(); ++i) {
      if (std::fabs(split_region_info_list[i].distance_to_split_point -
                    dis_to_ramp) < 1.0) {
        ramp_link_id = split_region_info_list[i].split_link_id;
        break;
      }
    }
  }

  // Set v_cruise_limit by map info
  if (!environmental_model.get_route_info()->get_sdpromap_valid()) {
    v_cruise_limit_ = std::round(v_cruise_fsm * 3.6 / 10.0) * 10;
  }

  const auto& sdpro_map = environmental_model.get_route_info()->get_sdpro_map();
  const auto ramp_link = sdpro_map.GetNextLinkOnRoute(ramp_link_id);
  if (ramp_link != nullptr) {
    ramp_v_limit = ramp_link->speed_limit();
  }

  ad_common::math::Vec2d current_point;
  const auto& ego_state = environmental_model.get_ego_state_manager();
  const auto& pose = ego_state->location_enu();
  current_point.set_x(pose.position.x);
  current_point.set_y(pose.position.y);

  double nearest_s = 0;
  double nearest_l = 0;
  const double search_distance = 50.0;
  const double max_heading_diff = M_PI / 4;
  const double ego_heading_angle = ego_state->heading_angle();

  const auto current_segment = sdpro_map.GetNearestLinkWithHeading(
      current_point, search_distance, ego_heading_angle, max_heading_diff,
      nearest_s, nearest_l);

  if (current_segment == nullptr) {
    v_cruise_limit_ = std::round(v_cruise_fsm * 3.6 / 10.0) * 10;
  } else {
    v_cruise_limit_ = current_segment->speed_limit();
  }

  double v_limit_gaode = 0;
  if (environmental_model.get_route_info()->get_sdmap_valid()) {
    const auto& sd_map = environmental_model.get_route_info()->get_sd_map();
    if (sd_map.GetNaviRoadInfo() != std::nullopt) {
      v_limit_gaode = sd_map.GetNaviRoadInfo().value().cur_road_speed_limit();
    }
  }

  if (v_limit_gaode > 30.0 - kEpsilon) {
    v_cruise_limit_ = v_limit_gaode;
  }

  const auto virtual_lane_manager =
      environmental_model.get_virtual_lane_manager();
  bool is_continuous_ramp = virtual_lane_manager->is_continuous_ramp();

  double v_target_ramp = 40;
  double v_target_near_ramp_zone = 40;
  double pre_acc_dis = speed_limit_config_.pre_accelerate_distance_for_merge;
  double sdpro_min_curv = JudgeCurvBySDProMap();

  // On ramp logic
  if (is_on_ramp) {
    if (dis_to_merge > pre_acc_dis || is_continuous_ramp) {
      if (sdpro_min_curv > speed_limit_config_.ramp_curv_radius_small &&
          sdpro_min_curv < speed_limit_config_.ramp_curv_radius_big) {
        v_target_ramp = speed_limit_config_.straight_ramp_v_limit_low;
      } else if (sdpro_min_curv >= speed_limit_config_.ramp_curv_radius_big) {
        v_target_ramp = speed_limit_config_.straight_ramp_v_limit_high;
      } else {
        v_target_ramp = speed_limit_config_.v_limit_ramp;
      }
    } else {
      ReferencePathPoint detect_merge_front_pnt;
      double ego_s;
      const auto& current_lane = virtual_lane_manager->get_current_lane();
      if (current_lane != nullptr &&
          current_lane->get_reference_path() != nullptr) {
        ego_s = current_lane->get_reference_path()->get_frenet_ego_state().s();
      } else {
        ego_s = -100.0;
      }

      if (ego_s > 0 &&
          current_lane->get_reference_path()->get_reference_point_by_lon(
              ego_s + dis_to_merge + kMergePointDetectedDistance,
              detect_merge_front_pnt)) {
        double nearest_s = 0;
        double nearest_l = 0;
        double search_distance = 50.0;
        double max_heading_diff = M_PI / 4;
        double detected_heading_angle =
            detect_merge_front_pnt.path_point.theta();
        ad_common::math::Vec2d detected_point(
            detect_merge_front_pnt.path_point.x(),
            detect_merge_front_pnt.path_point.y());

        if (environmental_model.get_route_info()->get_sdmap_valid()) {
          const auto& sd_map =
              environmental_model.get_route_info()->get_sd_map();
          const auto segment = sd_map.GetNearestRoadWithHeading(
              detected_point, search_distance, detected_heading_angle,
              max_heading_diff, nearest_s, nearest_l);
          if (segment != nullptr &&
              segment->priority() != SdMapSwtx::RoadPriority::EXPRESSWAY &&
              segment->priority() != SdMapSwtx::RoadPriority::CITY_EXPRESSWAY) {
            v_target_ramp = speed_limit_config_.v_limit_ramp;
          }
        }
      }
    }

    v_cruise_limit_ = std::max(v_cruise_limit_, 60.0);
    v_target_ramp = v_cruise_limit_ / 3.6;
    if (v_target_ramp < v_target_) {
      v_target_ = v_target_ramp;
      v_target_type_ = 2;  // MAP_ON_RAMP
    }
    return;
  }

  // Near ramp logic
  if ((dis_to_ramp <= speed_limit_config_.dis_near_ramp_zone) &&
      !(ramp_v_limit > 60.0)) {
    double pre_brake_dis_near_ramp_zone = std::max(
        dis_to_ramp - speed_limit_config_.brake_dis_near_ramp_zone, 0.0);
    v_target_near_ramp_zone = std::pow(
        std::pow(speed_limit_config_.v_limit_near_ramp_zone, 2.0) -
            2 * pre_brake_dis_near_ramp_zone * speed_limit_config_.acc_to_ramp,
        0.5);
  }

  double pre_brake_dis_to_ramp = std::max(dis_to_ramp - 50, 0.0);
  v_target_ramp = std::pow(
      std::pow(std::max(speed_limit_config_.v_limit_ramp, ramp_v_limit / 3.6),
               2.0) -
          2 * pre_brake_dis_to_ramp * speed_limit_config_.acc_to_ramp,
      0.5);
  v_target_ramp = std::min(v_target_near_ramp_zone, v_target_ramp);

  if (v_target_ramp < v_target_) {
    v_target_ = v_target_ramp;
    v_target_type_ = 3;  // MAP_NEAR_RAMP
  }
}

void JointMotionSpeedLimit::CalculateIntersectionSpeedLimit() {
  const auto& environmental_model = session_->environmental_model();
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
      v_limit_with_intersection_ = std::max(
          v_ego - speed_limit_config_.v_reduce_rate_intersection * v_ego,
          speed_limit_config_.v_intersection_min_limit);
    }
    v_target_intersection = v_limit_with_intersection_;
  } else {
    v_limit_with_intersection_ = 0.0;
  }

  if (v_target_intersection < v_target_) {
    v_target_ = v_target_intersection;
    v_target_type_ = 4;  // INTERSECTION
  }

  last_intersection_state_ = current_intersection_state_;
}

void JointMotionSpeedLimit::CalculatePOISpeedLimit() {
  const auto& environmental_model = session_->environmental_model();
  const auto& route_info_output =
      environmental_model.get_route_info()->get_route_info_output();

  if (!environmental_model.get_route_info()->get_sdpromap_valid()) {
    poi_v_limit_set_ = false;
    return;
  }

  ad_common::math::Vec2d current_point;
  const auto& ego_state = environmental_model.get_ego_state_manager();
  const auto& pose = ego_state->location_enu();
  current_point.set_x(pose.position.x);
  current_point.set_y(pose.position.y);

  const auto& sdpro_map = environmental_model.get_route_info()->get_sdpro_map();
  double nearest_s = 0;
  double nearest_l = 0;
  const double search_distance = 50.0;
  const double max_heading_diff = M_PI / 4;
  const double ego_heading_angle = ego_state->heading_angle();

  const auto current_segment = sdpro_map.GetNearestLinkWithHeading(
      current_point, search_distance, ego_heading_angle, max_heading_diff,
      nearest_s, nearest_l);

  if (current_segment == nullptr) {
    poi_v_limit_set_ = false;
    return;
  }

  poi_v_limit_set_ = false;
  double cur_link_v_limit = current_segment->speed_limit();
  double v_limit_dis = 10000.0;
  v_limit_dis =
      interp(cur_link_v_limit,
             speed_limit_config_.tunnel_vel_limit_dis_table.vel_limit_table,
             speed_limit_config_.tunnel_vel_limit_dis_table.dis_table) +
      kTunnelVelLimitDisOffset;

  auto tunnel_info =
      sdpro_map.GetTunnelInfo(current_segment->id(), nearest_s, 700.0);
  if (tunnel_info.first != nullptr && tunnel_info.second > 0 &&
      tunnel_info.second < v_limit_dis) {
    v_cruise_limit_ = speed_limit_config_.tunnel_vel_limit_kph;
    poi_v_limit_set_ = true;
  } else if (tunnel_info.second < kEpsilon &&
             current_segment->link_type() ==
                 iflymapdata::sdpro::LinkType::LT_TUNNEL) {
    v_cruise_limit_ = speed_limit_config_.tunnel_vel_limit_kph;
    poi_v_limit_set_ = true;
  } else {
    v_limit_dis = interp(
        cur_link_v_limit,
        speed_limit_config_.toll_station_vel_limit_dis_table.vel_limit_table,
        speed_limit_config_.toll_station_vel_limit_dis_table.dis_table);
    auto toll_station_info =
        sdpro_map.GetTollStationInfo(current_segment->id(), nearest_s, 700.0);
    if (toll_station_info.first != nullptr && toll_station_info.second > 0 &&
        toll_station_info.second <
            speed_limit_config_.function_off_dis_before_toll_station +
                v_limit_dis) {
      v_cruise_limit_ = speed_limit_config_.toll_station_vel_limit_kph;
      poi_v_limit_set_ = true;
    } else {
      v_limit_dis =
          interp(cur_link_v_limit,
                 speed_limit_config_.sapa_vel_limit_dis_table.vel_limit_table,
                 speed_limit_config_.sapa_vel_limit_dis_table.dis_table);
      auto sapa_info =
          sdpro_map.GetSaPaInfo(current_segment->id(), nearest_s, 700.0);
      if (sapa_info.first != nullptr && sapa_info.second > 0 &&
          sapa_info.second < v_limit_dis) {
        v_cruise_limit_ = speed_limit_config_.sapa_vel_limit_kph;
        poi_v_limit_set_ = true;
      } else if (sapa_info.second < kEpsilon &&
                 current_segment->link_type() ==
                     iflymapdata::sdpro::LinkType::LT_SAPA) {
        v_cruise_limit_ = speed_limit_config_.sapa_vel_limit_kph;
        poi_v_limit_set_ = true;
      } else {
        v_limit_dis = interp(
            cur_link_v_limit,
            speed_limit_config_.non_express_vel_limit_dis_table.vel_limit_table,
            speed_limit_config_.non_express_vel_limit_dis_table.dis_table);
        auto none_express_info = sdpro_map.GetNonExpressInfo(
            current_segment->id(), nearest_s, 700.0);
        if (none_express_info.first != nullptr &&
            none_express_info.second > 0 &&
            none_express_info.second < v_limit_dis) {
          v_cruise_limit_ = speed_limit_config_.non_express_vel_limit_kph;
          poi_v_limit_set_ = true;
        }
      }
    }
  }

  if (poi_v_limit_set_) {
    double v_near_poi = v_cruise_limit_ / 3.6;
    if (v_near_poi < v_target_) {
      v_target_ = v_near_poi;
      v_target_type_ = 5;  // NEAR_POI
    }
  }
}

void JointMotionSpeedLimit::CalculateSpeedLimitFromTFLDis() {
  const auto& local_view = session_->environmental_model().get_local_view();
  auto fsm_state = local_view.function_state_machine_info.current_state;
  bool noa_mode = (fsm_state == iflyauto::FunctionalState_NOA_ACTIVATE) ||
                  (fsm_state == iflyauto::FunctionalState_NOA_OVERRIDE);

  double v_limit_tfl_dis = 40.0;
  const auto& environmental_model = session_->environmental_model();
  const auto tfl_manager =
      environmental_model.get_traffic_light_decision_manager();
  const auto traffic_status = tfl_manager->GetTrafficStatus();
  double dis_tfl = tfl_manager->GetNearestTFLDis();

  if (speed_limit_config_.enable_tfl_v_limit && dis_tfl < kTFLSpeedLimitDis &&
      (!noa_mode)) {
    v_limit_tfl_dis = 55.0 / 3.6;
    if (traffic_status.go_straight == 1 || traffic_status.go_straight == 41 ||
        traffic_status.go_straight == 11 || traffic_status.go_straight == 10) {
      v_limit_tfl_dis = 50.0 / 3.6;
    }
  }

  if (v_limit_tfl_dis < v_target_) {
    v_target_ = v_limit_tfl_dis;
    v_target_type_ = 6;  // NEAR_TFL
  }
}

bool JointMotionSpeedLimit::CheckClustersConsecutiveDiffSlidingWindow(
    const std::map<int, ConstructionAgentClusterArea>& cluster_map,
    const std::shared_ptr<planning_math::KDPath>& planned_kd_path,
    bool entering) const {
  return CheckClustersConsecutiveDiffSlidingWindowImpl(
      cluster_map, planned_kd_path, entering);
}

void JointMotionSpeedLimit::CalculateConstructionZoneSpeedLimit() {
  double v_target_construction = 50.0;
  double v_target_near_construction = 50.0;
  double dis_to_construction = std::numeric_limits<double>::max();
  int construction_strong_mode_reason = 0;

  if (!speed_limit_config_.enable_construction_speed_limit) {
    return;
  }

  const auto& construction_scene_output =
      session_->environmental_model().get_construction_scene_manager()->get_construction_scene_output();
  if (!construction_scene_output.is_exist_construction_area ||
      construction_scene_output.construction_agent_cluster_attribute_map.empty()) {
    construction_strong_deceleration_mode = false;
    construction_strong_mode_frame_count_ = 0;
    return;
  }

  bool is_exist_construction = construction_scene_output.is_exist_construction_area;
  bool is_on_construction = construction_scene_output.is_pass_construction_area;

  const auto& environmental_model = session_->environmental_model();
  const auto& function_state_machine_info =
      environmental_model.get_local_view().function_state_machine_info;
  double v_cruise_fsm =
      function_state_machine_info.pilot_req.acc_curise_real_spd;
  const auto ego_state_mgr = environmental_model.get_ego_state_manager();
  double v_ego = ego_state_mgr->ego_v();
  const auto init_point = ego_state_mgr->planning_init_point();
  double v_cruise = ego_state_mgr->ego_v_cruise();

  const auto& planned_kd_path =
      session_->planning_context().motion_planner_output().lateral_path_coord;
  if (planned_kd_path == nullptr) {
    return;
  }

  double ego_s = 0.0;
  double ego_l = 0.0;
  if (!planned_kd_path->XYToSL(init_point.x, init_point.y, &ego_s, &ego_l)) {
    return;
  }

  double construction_s = 0.0;
  double construction_l = 0.0;
  double construction_s_nearest = std::numeric_limits<double>::max();

  for (const auto& [cluster_id, construction_area] :
       construction_scene_output.construction_agent_cluster_attribute_map) {
    auto agent_info = construction_area.points.front();
    if (!planned_kd_path->XYToSL(agent_info.x, agent_info.y, &construction_s,
                                 &construction_l)) {
      continue;
    }
    if (construction_s < construction_s_nearest) {
      construction_s_nearest = construction_s;
    }
  }

  // Construction need strong deceleration
  std::vector<SLPoint> sl_construction_points_all;
  sl_construction_points_all.reserve(20);
  double construction_nearest_l = std::numeric_limits<double>::max();

  for (const auto& [cluster_id, construction_area] :
       construction_scene_output.construction_agent_cluster_attribute_map) {
    for (const auto& pt : construction_area.points) {
      double s = 0.0, l = 0.0;
      if (!planned_kd_path->XYToSL(pt.x, pt.y, &s, &l)) {
        continue;
      }
      if (s >= ego_s && s <= ego_s + kCAInvadeVaildLonDis) {
        sl_construction_points_all.push_back({s, l});
      }
      if (std::abs(l) < std::abs(construction_nearest_l)) {
        construction_nearest_l = l;
      }
    }
  }

  int construction_invade_count = 0;
  for (const auto& p : sl_construction_points_all) {
    if (std::abs(p.l) < speed_limit_config_.ca_invade_entry_lat_dis_thr) {
      construction_invade_count++;
    }
  }

  // Check enter and exit conditions
  bool enter_condition = false;
  bool exit_condition = false;

  if (construction_invade_count >=
      speed_limit_config_.ca_invade_lat_dis_counter_thr) {
    enter_condition = true;
    construction_strong_mode_reason = 1;
  } else if (CheckClustersConsecutiveDiffSlidingWindow(
                 construction_scene_output.construction_agent_cluster_attribute_map,
                 planned_kd_path, true)) {
    enter_condition = true;
    construction_strong_mode_reason = 2;
  }

  if (std::abs(construction_nearest_l) >
      speed_limit_config_.ca_invade_exit_lat_dis_thr) {
    exit_condition = true;
    construction_strong_mode_reason = 11;
  } else if (CheckClustersConsecutiveDiffSlidingWindow(
                 construction_scene_output.construction_agent_cluster_attribute_map,
                 planned_kd_path, false)) {
    exit_condition = true;
    construction_strong_mode_reason = 12;
  }

  if (!construction_strong_deceleration_mode) {
    if (enter_condition) {
      construction_strong_deceleration_mode = true;
      construction_strong_mode_frame_count_ = 0;
    }
  } else {
    if (construction_strong_mode_frame_count_ <
        kConstructionStrongMaxHoldFrames) {
      construction_strong_mode_frame_count_++;
    }
    bool min_duration_met = (construction_strong_mode_frame_count_ >=
                             kConstructionStrongMinHoldFrames);
    if (exit_condition && min_duration_met) {
      construction_strong_deceleration_mode = false;
      construction_strong_mode_frame_count_ = 0;
    } else if (exit_condition && !min_duration_met) {
      construction_strong_mode_reason = 30;
    }
  }

  // Construction zone info
  dis_to_construction = std::max(construction_s_nearest - ego_s, 0.0);

  if (is_on_construction) {
    if (construction_strong_deceleration_mode) {
      v_target_construction =
          speed_limit_config_.v_limit_construction -
          std::max(speed_limit_config_.construction_invade_speed_diff, 0.0);
    } else {
      v_target_construction = speed_limit_config_.v_limit_construction;
    }

    if (v_target_construction < v_target_) {
      v_target_ = v_target_construction;
      v_target_type_ = 7;  // ON_CONSTRUCTION
    }
    return;
  }

  double construction_speed_threshold =
      speed_limit_config_.construction_speed_threshold;
  double v_limit_near_construction =
      speed_limit_config_.v_limit_near_construction;

  if (construction_strong_deceleration_mode) {
    construction_speed_threshold =
        construction_speed_threshold -
        std::max(speed_limit_config_.construction_invade_speed_diff, 0.0);
    v_limit_near_construction =
        v_limit_near_construction -
        std::max(speed_limit_config_.construction_invade_speed_diff, 0.0);
  }

  if (dis_to_construction <= speed_limit_config_.dis_near_construction &&
      v_cruise > construction_speed_threshold && is_exist_construction) {
    double pre_brake_dis_near_construction = std::max(
        dis_to_construction - speed_limit_config_.brake_dis_near_construction,
        0.0);
    v_target_near_construction =
        std::pow(std::pow(v_limit_near_construction, 2.0) -
                     2 * pre_brake_dis_near_construction *
                         speed_limit_config_.acc_to_construction,
                 0.5);
  }

  if (v_target_near_construction < v_target_) {
    v_target_ = v_target_near_construction;
    v_target_type_ = 8;  // NEAR_CONSTRUCTION
  }
}

void JointMotionSpeedLimit::CalculateFunctionFadingAwaySpeedLimit() {
  const auto& ego_state_mgr =
      session_->environmental_model().get_ego_state_manager();
  const auto& ego_vehi_param =
      VehicleConfigurationContext::Instance()->get_vehicle_param();
  const double ego_rear_axle_to_front_edge =
      ego_vehi_param.length - ego_vehi_param.rear_edge_to_rear_axle;
  const auto& virtual_lane_manager =
      session_->environmental_model().get_virtual_lane_manager();
  const auto current_ego_lane_mark =
      virtual_lane_manager->lane_mark_at_ego_front_edge_pos_current();

  if (!speed_limit_config_.left_right_turn_func_fading_away_switch) {
    return;
  }

  is_function_fading_away_ = false;
  request_reason_ = iflyauto::RequestReason::REQUEST_REASON_NO_REASON;
  static bool is_first_time_funciton_fading_away = true;

  const auto distance_to_stop_line =
      virtual_lane_manager->GetEgoDistanceToStopline();
  const auto distance_to_crosswalk =
      virtual_lane_manager->GetEgoDistanceToCrosswalk();

  static const std::unordered_set<iflyauto::LaneDrivableDirection>
      left_turning_direction_set = {
          iflyauto::LaneDrivableDirection::LaneDrivableDirection_DIRECTION_LEFT,
          iflyauto::LaneDrivableDirection::
              LaneDrivableDirection_DIRECTION_UTURN_LEFT,
          iflyauto::LaneDrivableDirection::
              LaneDrivableDirection_DIRECTION_LEFT_UTURN,
          iflyauto::LaneDrivableDirection::
              LaneDrivableDirection_DIRECTION_LEFT_RIGHT,
          iflyauto::LaneDrivableDirection::
              LaneDrivableDirection_DIRECTION_UTURNLEFT_RIGHT};

  static const std::unordered_set<iflyauto::LaneDrivableDirection>
      right_turning_direction_set = {
          iflyauto::LaneDrivableDirection::
              LaneDrivableDirection_DIRECTION_RIGHT,
          iflyauto::LaneDrivableDirection::
              LaneDrivableDirection_DIRECTION_UTURN_RIGHT,
          iflyauto::LaneDrivableDirection::
              LaneDrivableDirection_DIRECTION_RIGHT_UTURN};

  if (current_intersection_state_ == common::APPROACH_INTERSECTION &&
      (distance_to_stop_line <
           speed_limit_config_.function_fading_away_distance_to_intersection ||
       distance_to_crosswalk <
           speed_limit_config_.function_fading_away_distance_to_intersection)) {
    is_function_fading_away_ =
        !virtual_lane_manager->ego_currrent_pos_lane_has_straight_attributes();
  }

  if (is_function_fading_away_) {
    if (left_turning_direction_set.find(current_ego_lane_mark) !=
        left_turning_direction_set.end()) {
      request_reason_ =
          iflyauto::RequestReason::REQUEST_REASON_ON_INTERSECTION_LEFT_LANE;
    } else if (right_turning_direction_set.find(current_ego_lane_mark) !=
               right_turning_direction_set.end()) {
      request_reason_ =
          iflyauto::RequestReason::REQUEST_REASON_ON_INTERSECTION_RIGHT_LANE;
    }
  }

  if (!is_function_fading_away_) {
    is_first_time_funciton_fading_away = true;
    return;
  }
  if (is_first_time_funciton_fading_away) {
    vel_slope_filter_function_fading_away_.SetState(v_target_);
    is_first_time_funciton_fading_away = false;
  } else {
    vel_slope_filter_function_fading_away_.SetState(
        last_vel_function_fading_away_);
  }

  vel_slope_filter_function_fading_away_.Update(0.0);
  v_target_ =
      std::min(vel_slope_filter_function_fading_away_.GetOutput(), v_target_);
  last_vel_function_fading_away_ = v_target_;
}

}  // namespace planning
