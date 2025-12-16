#include "speed_limit_decider.h"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <utility>
#include <vector>

#include "agent/agent.h"
#include "behavior_planners/speed_limit_decider/speed_limit_decider_output.h"
#include "construction_scene_manager.h"
#include "debug_info_log.h"
#include "ego_state_manager.h"
#include "environmental_model.h"
#include "filters.h"
#include "mjson/json.hpp"
#include "planning_context.h"
#include "utils/kd_path.h"
#include "vec2d.h"
#include "vehicle_config_context.h"

namespace planning {
namespace {
constexpr double kEpsilon = 1e-6;
constexpr double kLaneBorrowLimitedSpeed = 8.33;
constexpr double kHighVel = 60 / 3.6;
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
constexpr double kEnterVruRoundEgoVelThr = 15 / 3.6;
constexpr double kEnterVruRoundDistanceThr = 50.0;
constexpr double kEnterVruRoundTtcThr = 4.5;
constexpr int32_t kTriggerVruRoundcounterThr = 5;
constexpr double kExitVruRoundLateralBufferThr = 1.0;
constexpr double kExitVruRoundDistanceThr = 80.0;
constexpr double kLowSpeedVruVelThr = 30 / 3.6;
constexpr double kVRURoundDecelRatio = 0.7;
constexpr double kTunnelVelLimitDisOffset = 100;
constexpr double kCAInvadeLatDisDiffThr = 0.25;
constexpr double kCAInvadeVaildLonDis = 80.0;
constexpr double kCAInvadeLatMaxDis = 2.5;
constexpr double kCAInvadeLatMinDis = 2;
constexpr int kConstructionStrongMinHoldFrames = 100;
constexpr int kConstructionStrongMaxHoldFrames = 600;
constexpr double kCAManualInterventionSpeedDetected = 4 / 3.6;
constexpr double kSamplingStep = 2.0;
// Dynamic EWMA alpha based on radius: [150, 300, 500, 600] -> [0.3, 0.15, 0.1, 0.05]
const std::vector<double> _EWMA_ALPHA_RADIUS_BP{150.0, 300.0, 500.0, 600.0};
const std::vector<double> _EWMA_ALPHA_V{0.3, 0.15, 0.1, 0.05};
constexpr double kSSharpBendCount = 3;
constexpr double kFarEnoughDisToMerge = 500.0;
constexpr double kCloseDisToMergeCancelVLimit = 120.0;
constexpr double kNearMergeCancelVLimitCurvRadius = 900.0;
constexpr double kFarAwayMergeCounterNums = 50;
constexpr double kShortDisReachMerge = 6.0;
constexpr double kSharpCurveEnterThreshold = 100.0;  // Enter threshold (m)
constexpr double kSharpCurveExitThreshold = 130.0;   // Exit threshold with hysteresis (m)
constexpr double kMinDistanceForDecel = 1;  // Min distance to avoid division by zero
constexpr double kCurvatureDecelThreshold = -1.0;  // Deceleration threshold for CURVATURE (m/s²)
constexpr double kCurvSpeedDifference = 3 / 3.6;
constexpr int kSharpCurveMinFrames = 5;  // Min frames to maintain sharp curve state
constexpr double kMinRampSampleLength = 150.0;
constexpr double kMapSharpCurveRadiusEnter = 100.0;  // Enter radius threshold for map sharp curve (m)
constexpr double kMapSharpCurveRadiusExit = 120.0;  // Exit radius threshold with hysteresis (m)
constexpr int kMapSharpCurveRawCountEnter = 4;  // Enter threshold count of k_raw points with radius < kMapSharpCurveRadiusEnter
constexpr int kMapSharpCurveRawCountExit = 3;  // Exit threshold count of k_raw points with radius < kMapSharpCurveRadiusEnter (hysteresis)
constexpr double kMapRadiusFirstEnterForDisCal = 65.0;  // Radius threshold 65m
constexpr double kMapSharpCurveMinDistance = 20.0;  // Minimum distance between first and last point satisfying condition (m)
constexpr double kMapSharpCurveDistThresholdV70 = 125.0;  // Distance threshold to sharp curve when speed > 70kph (m)
constexpr double kMapSharpCurveDistThresholdV60 = 75.0;   // Distance threshold to sharp curve when speed > 60kph (m)
constexpr double kMapSharpCurveDistThresholdV50 = 35.0;   // Distance threshold to sharp curve when speed > 50kph (m)
constexpr double kMapSharpCurveDistThresholdV45 = 17.0;   // Distance threshold to sharp curve when speed > 45kph (m)
constexpr double kMaxRampPointSpacing = 40.0;  // Maximum point spacing threshold, beyond which is considered sparse segment (m)
constexpr double kMaxRampPointSpacingRatio = 2.5;  // Ratio threshold of max spacing to average spacing, beyond which abnormal sparse segment is detected
constexpr int kMaxDensePointCountForSparseBack = 3;  // Maximum dense point count threshold for determining dense-front-sparse-back pattern
constexpr double kMaxDistanceToRamp = 2000.0;
constexpr double kDistanceTolerance = 1.0;
constexpr double preview_distance_0_80m = 80.0;
constexpr double sampling_step = 2.0;
constexpr double kAvgRadiusEnterSpeedDiff = 1.5;  // Speed difference threshold for entering avg radius EWMA (m/s)
constexpr double kAvgRadiusEnterRadius = 350.0;  // Road radius threshold for entering avg radius EWMA (m)
constexpr double kAvgRadiusExitSpeedDiff = 3.0;  // Speed difference threshold for exiting avg radius EWMA (m/s)
constexpr double kAvgRadiusExitRadius = 280.0;  // Road radius threshold for exiting avg radius EWMA (m)

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
bool IsPointOutsideLaneByLeftRightThred(
    const std::shared_ptr<VirtualLane> &ego_lane,
    const planning_math::Vec2d &point,
    const double distance_to_left_boundary_threshold,
    const double distance_to_right_boundary_threshold) {
  bool is_outside_lane = false;
  double match_s = 0.0;
  double match_l = 0.0;
  const auto &ego_reference_path = ego_lane->get_reference_path();
  if (ego_reference_path == nullptr) {
    return true;
  }
  const auto &ego_lane_coord = ego_reference_path->get_frenet_coord();
  if (ego_lane_coord == nullptr) {
    return true;
  }

  if (!ego_lane_coord->XYToSL(point.x(), point.y(), &match_s, &match_l)) {
    return true;
  }
  double lane_width = ego_lane->width_by_s(match_s);
  is_outside_lane =
      (match_l > lane_width / 2.0 + distance_to_left_boundary_threshold) ||
      (match_l < -lane_width / 2.0 - distance_to_right_boundary_threshold);
  return is_outside_lane;
}

bool CheckLateralConflict(
    const PlanningInitPoint &ego_init_point,
    const std::shared_ptr<VirtualLane> &ego_lane,
    const std::shared_ptr<planning_math::KDPath> &ego_lane_coord,
    const agent::Agent *agent, const double lane_width_ratio) {
  if (agent == nullptr) return false;

  double ego_s = 0.0, ego_l = 0.0;
  if (!ego_lane_coord->XYToSL(ego_init_point.x, ego_init_point.y, &ego_s,
                              &ego_l))
    return false;

  const auto &ego_vehicle_param =
      VehicleConfigurationContext::Instance()->get_vehicle_param();
  const double half_ego_width = ego_vehicle_param.width * 0.5;

  const double ego_l_min = ego_l - half_ego_width;
  const double ego_l_max = ego_l + half_ego_width;

  double agent_s = 0.0, agent_l = 0.0;
  const auto agent_center = agent->box().center();
  if (!ego_lane_coord->XYToSL(agent_center.x(), agent_center.y(), &agent_s,
                              &agent_l)) {
    return false;
  }

  const auto &agent_corners = agent->box().GetAllCorners();
  double obs_min_l = std::numeric_limits<double>::max(),
         obs_max_l = -std::numeric_limits<double>::max();
  for (const auto &agent_corner : agent_corners) {
    double agent_corner_s = 0.0, agent_corner_l = 0.0;
    if (ego_lane_coord->XYToSL(agent_corner.x(), agent_corner.y(),
                               &agent_corner_s, &agent_corner_l)) {
      obs_min_l = std::min(obs_min_l, agent_corner_l);
      obs_max_l = std::max(obs_max_l, agent_corner_l);
    }
  }

  double lateral_dist = 0.0;
  if (obs_min_l > ego_l_max)
    lateral_dist = obs_min_l - ego_l_max;
  else if (obs_max_l < ego_l_min)
    lateral_dist = ego_l_min - obs_max_l;

  bool has_lateral_distance_conflict =
      lateral_dist <= ego_lane->width_by_s(ego_s) * lane_width_ratio;

  return has_lateral_distance_conflict;
}

bool CheckClustersConsecutiveDiffSlidingWindow(
    const std::map<int, ConstructionAgentClusterArea> &cluster_map,
    const std::shared_ptr<planning_math::KDPath> &planned_kd_path,
    bool entering) {
  for (const auto & [ cluster_id, construction_area ] : cluster_map) {
    const auto &pts = construction_area.points;
    if (pts.size() < 3) {
      continue;
    }
    // Convert all points to l coordinates
    std::vector<double> ls;
    ls.reserve(pts.size());
    for (const auto &pt : pts) {
      double s = 0.0, l = 0.0;
      if (!planned_kd_path->XYToSL(pt.x, pt.y, &s, &l)) {
        continue;
      }
      ls.push_back(l);
    }
    if (ls.size() < 3) {
      continue;
    }
    // Sliding window to check consecutive 3 points
    for (size_t i = 0; i + 2 < ls.size(); ++i) {
      double diff01 = std::abs(ls[i + 1] - ls[i]);
      double diff12 = std::abs(ls[i + 2] - ls[i + 1]);
      if (diff01 > kCAInvadeLatDisDiffThr && diff12 > kCAInvadeLatDisDiffThr) {
        double abs_l0 = std::abs(ls[i]);
        double abs_l1 = std::abs(ls[i + 1]);
        double abs_l2 = std::abs(ls[i + 2]);
        if (entering && abs_l0 > abs_l1 && abs_l1 > abs_l2 &&
            abs_l0 < kCAInvadeLatMaxDis) {
          return true;
        }
        if (!entering && abs_l0 < abs_l1 && abs_l1 < abs_l2 &&
            abs_l0 < kCAInvadeLatMinDis) {
          return true;
        }
      }
    }
  }
  return false;
}
}  // namespace

SpeedLimitDecider::SpeedLimitDecider(
    const EgoPlanningConfigBuilder *config_builder, framework::Session *session)
    : Task(config_builder, session) {
  speed_limit_config_ = config_builder->cast<SpeedLimitConfig>();
  name_ = "SpeedLimitDecider";
  vel_slope_filter_function_fading_away_.Init(
      speed_limit_config_.min_acc_function_fading_away, 0.0, 1e-3, 40, 0.1);
}
bool SpeedLimitDecider::Execute() {
  ILOG_INFO << "=======SpeedLimitDecider=======";
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

  CalculateVRURoundSpeedLimit();
  // 9. speed limit from avoid agent
  CalculateAvoidAgentSpeedLimit();
  // speed limit from construction
  CalculateConstructionZoneSpeedLimit();

  CalculateSpeedLimitForDangerousObstacle();
  /* NOTE: CalculateFunctionFadingAwaySpeedLimit() need to be set up at the end
   * of speed limiter!!!*/
  // 10. speed limit for LCC/NOA fading away
  CalculateFunctionFadingAwaySpeedLimit();

  auto speed_limit_output = session_->mutable_planning_context()
                                ->mutable_speed_limit_decider_output();
  if (SpeedLimitType::CRUISE == v_target_type_ && construction_v_limit_set_) {
    v_target_type_ = SpeedLimitType::ON_CONSTRUCTION;
  } else if (SpeedLimitType::CRUISE == v_target_type_ && poi_v_limit_set_) {
    v_target_type_ = SpeedLimitType::NEAR_POI;
  }
  speed_limit_output->SetSpeedLimit(v_target_, v_target_type_);
  JSON_DEBUG_VALUE("v_target_decider", v_target_);
  JSON_DEBUG_VALUE("v_target_type_code",  std::underlying_type<SpeedLimitType>::type(v_target_type_));
  auto ad_info = &(session_->mutable_planning_context()
                       ->mutable_planning_hmi_info()
                       ->ad_info);
  speed_limit_output->set_is_function_fading_away(is_function_fading_away_);
  speed_limit_output->set_request_reason(request_reason_);

  if (SpeedLimitType::CURVATURE == v_target_type_ || 
      SpeedLimitType::SHARP_CURVATURE == v_target_type_) {
    ad_info->is_curva = true;
  } else {
    ad_info->is_curva = false;
  }
  v_cruise_limit_ = std::max(v_cruise_limit_, 60.0);
  JSON_DEBUG_VALUE("v_cruise_limit", v_cruise_limit_);
  ad_info->cruise_speed = v_cruise_limit_;
  speed_limit_output->set_map_speed_limit_value(v_cruise_limit_);
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

double SpeedLimitDecider::JudgeCurvBySDProMap(double search_dis) {
  if (!speed_limit_config_.enable_sdmap_curv_v_adjust) {
    return 300.0;
  }
  if (!session_->environmental_model().get_route_info()->get_sdpromap_valid()) {
    ILOG_INFO << "sd_map is invalid!!!";
    return 300.0;
  }
  ad_common::math::Vec2d current_point;
  const auto &ego_state =
      session_->environmental_model().get_ego_state_manager();
  const auto &pose = ego_state->location_enu();
  ILOG_INFO << "ego_pose_x_:" << pose.position.x
            << "ego_pose_y_:" << pose.position.y;
  current_point.set_x(pose.position.x);
  current_point.set_y(pose.position.y);
  const auto &sdpro_map =
      session_->environmental_model().get_route_info()->get_sdpro_map();
  double nearest_s = 0;
  double nearest_l = 0;
  // const double max_search_length = 7000.0;  // 搜索7km范围内得地图信息
  const double search_distance = 50.0;
  const double max_heading_diff = PI / 4;
  const double ego_heading_angle = ego_state->heading_angle();
  const auto current_segment = sdpro_map.GetNearestLinkWithHeading(
      current_point, search_distance, ego_heading_angle, max_heading_diff,
      nearest_s, nearest_l);
  if (!current_segment) {
    return 300.0;  // Small radius indicates curve, don't accelerate on error
  }
  std::vector<std::pair<double, double>> curv_list;
  curv_list =
      sdpro_map.GetCurvatureList(current_segment->id(), nearest_s, search_dis);
  double min_curv_radius = 10000.0;
  for (int i = 0; i < curv_list.size(); i++) {
    double one_curv_radius = 1.0 / (std::abs(curv_list[i].second));
    if (one_curv_radius < min_curv_radius) {
      min_curv_radius = one_curv_radius;
    }
  }
  JSON_DEBUG_VALUE("sdpromap_min_curv_radius", min_curv_radius)
  return min_curv_radius;
}

// Find ramp link ID from split region info list
uint64_t SpeedLimitDecider::FindRampLinkId(
    double dist_to_ramp,
    const std::vector<NOASplitRegionInfo> &split_region_info_list) {
  uint64_t ramp_link_id = static_cast<uint64_t>(-1);
  if (dist_to_ramp < kMaxDistanceToRamp) {
    for (size_t i = 0; i < split_region_info_list.size(); ++i) {
      if (std::fabs(split_region_info_list[i].distance_to_split_point -
                    dist_to_ramp) < kDistanceTolerance) {
        ramp_link_id = split_region_info_list[i].split_link_id;
        break;
      }
    }
  }
  return ramp_link_id;
}

// Collect points from a single link
bool SpeedLimitDecider::CollectPointsFromLink(
    const iflymapdata::sdpro::LinkInfo_Link *link,
    std::vector<ad_common::math::Vec2d> &enu_points, double &total_len,
    size_t start_idx) {
  if (link == nullptr) {
    return false;
  }

  const auto &pts = link->points().boot().points();
  if (pts.size() < 2 || start_idx >= pts.size()) {
    return false;
  }

  constexpr double kMinPointSpacing = 0.1;
  for (size_t i = start_idx; i < pts.size(); ++i) {
    ad_common::math::Vec2d p(pts[i].x(), pts[i].y());
    if (!enu_points.empty()) {
      const auto &last = enu_points.back();
      double ds = std::hypot(p.x() - last.x(), p.y() - last.y());
      if (ds < kMinPointSpacing) {
        continue;
      }
      total_len += ds;
    }
    enu_points.emplace_back(p);
    if (total_len >= kMinRampSampleLength) {
      return true;  // Reached target length
    }
  }
  return false;  // Not reached target length
}

// Collect ramp points from multiple consecutive links
void SpeedLimitDecider::CollectRampPointsFromLinks(
    const iflymapdata::sdpro::LinkInfo_Link *start_link,
    const std::function<bool(const iflymapdata::sdpro::LinkInfo_Link &)> &is_ramp,
    const std::function<const iflymapdata::sdpro::LinkInfo_Link *(uint64_t)> &get_next_link,
    std::vector<ad_common::math::Vec2d> &enu_points, double &total_len) {
  const iflymapdata::sdpro::LinkInfo_Link *cur_link = start_link;
  while (cur_link != nullptr && is_ramp(*cur_link) &&
         total_len < kMinRampSampleLength) {
    // Check if link has valid points before collecting
    const auto &pts = cur_link->points().boot().points();
    if (pts.size() < 2) {
      // Invalid link, break
      break;
    }
    // Collect points from current link
    CollectPointsFromLink(cur_link, enu_points, total_len, 0);
    
    if (total_len >= kMinRampSampleLength) {
      break;
    }
    cur_link = get_next_link(cur_link->id());
  }
}

// Calculate max curvature on ramp, supports both on-ramp and approaching-ramp cases
double SpeedLimitDecider::CalcRampMaxCurvFromSDProMap(
    double *dist_to_max_curv, std::vector<double>* k_raw,
    std::vector<double>* s_vec_output) {
  const auto &environmental_model = session_->environmental_model();
  const auto &route_info = environmental_model.get_route_info();
  if (!route_info->get_sdpromap_valid()) {
    if (dist_to_max_curv != nullptr) {
      *dist_to_max_curv = 1000.0;
    }
    return 0.0;
  }

  const auto &route_info_output = route_info->get_route_info_output();
  const auto &sdpro_map = route_info->get_sdpro_map();
  bool is_on_ramp = route_info_output.is_on_ramp;
  double dis_to_ramp = route_info_output.dis_to_ramp;

  const iflymapdata::sdpro::LinkInfo_Link *start_link = nullptr;

  // Collect ENU points on ramp until total length >= 150m or leaving ramp
  std::vector<ad_common::math::Vec2d> enu_points;
  enu_points.reserve(50);
  double total_len = 0.0;

  const auto is_ramp =
      [&sdpro_map](const iflymapdata::sdpro::LinkInfo_Link &link) {
        return sdpro_map.isRamp(link.link_type());
      };

  if (is_on_ramp) {
    // Case 1: On ramp, start from current vehicle link
    ad_common::math::Vec2d current_point;
    const auto &ego_state = environmental_model.get_ego_state_manager();
    const auto &pose = ego_state->location_enu();
    current_point.set_x(pose.position.x);
    current_point.set_y(pose.position.y);
    double nearest_s = 0;
    double nearest_l = 0;
    const double search_distance = 50.0;
    const double max_heading_diff = PI / 4;
    const double ego_heading_angle = ego_state->heading_angle();
    const auto current_segment = sdpro_map.GetNearestLinkWithHeading(
        current_point, search_distance, ego_heading_angle, max_heading_diff,
        nearest_s, nearest_l);

    if (current_segment == nullptr) {
      if (dist_to_max_curv != nullptr) {
        *dist_to_max_curv = 1000.0;
      }
      return 0.0;
    }
    // Verify current link is ramp
    if (!sdpro_map.isRamp(current_segment->link_type())) {
      if (dist_to_max_curv != nullptr) {
        *dist_to_max_curv = 1000.0;
      }
      return 0.0;
    }

    start_link = current_segment;
    const iflymapdata::sdpro::LinkInfo_Link *cur_link = start_link;
    const auto &pts = cur_link->points().boot().points();
    if (pts.size() < 2) {
      if (dist_to_max_curv != nullptr) {
        *dist_to_max_curv = 1000.0;
      }
      return 0.0;
    }

    // Find nearest point to vehicle
    double min_dist = std::numeric_limits<double>::max();
    size_t nearest_idx = 0;
    for (size_t i = 0; i < pts.size(); ++i) {
      double dist = std::hypot(pts[i].x() - current_point.x(),
                               pts[i].y() - current_point.y());
      if (dist < min_dist) {
        min_dist = dist;
        nearest_idx = i;
      }
    }

    // Collect from nearest point (inclusive)
    for (size_t i = nearest_idx; i < pts.size(); ++i) {
      ad_common::math::Vec2d p(pts[i].x(), pts[i].y());
      if (!enu_points.empty()) {
        const auto &last = enu_points.back();
        double ds = std::hypot(p.x() - last.x(), p.y() - last.y());
        if (ds < 0.1) continue;
        total_len += ds;
      }
      enu_points.emplace_back(p);
      if (total_len >= kMinRampSampleLength) break;
    }

    // Continue collecting points from subsequent links
    if (total_len < kMinRampSampleLength) {
      // Get next link first to avoid re-collecting current link
      const iflymapdata::sdpro::LinkInfo_Link *next_link = 
          sdpro_map.GetNextLinkOnRoute(cur_link->id());
      if (next_link != nullptr) {
        auto get_next_link = [&sdpro_map](uint64_t link_id) {
          return sdpro_map.GetNextLinkOnRoute(link_id);
        };
        CollectRampPointsFromLinks(next_link, is_ramp, get_next_link, enu_points,
                                    total_len);
      }
    }

  } else {
    // Case 2: Approaching ramp, start from next link of ramp_link_id
    const auto &split_region_info_list =
        route_info_output.split_region_info_list;
    uint64_t ramp_link_id = FindRampLinkId(dis_to_ramp, split_region_info_list);

    if (ramp_link_id == static_cast<uint64_t>(-1)) {
      if (dist_to_max_curv != nullptr) {
        *dist_to_max_curv = 1000.0;
      }
      return 0.0;
    }

    const auto ramp_link = sdpro_map.GetNextLinkOnRoute(ramp_link_id);
    if (ramp_link == nullptr) {
      if (dist_to_max_curv != nullptr) {
        *dist_to_max_curv = 1000.0;
      }
      return 0.0;
    }

    start_link = ramp_link;
    auto get_next_link = [&sdpro_map](uint64_t link_id) {
      return sdpro_map.GetNextLinkOnRoute(link_id);
    };
    CollectRampPointsFromLinks(start_link, is_ramp, get_next_link, enu_points,
                               total_len);
  }

  // Calculate max curvature using 3-point geometry method
  if (enu_points.size() < 3) {
    if (dist_to_max_curv != nullptr) {
      *dist_to_max_curv = 1000.0;
    }
    return 0.0;
  }

  // Check point spacing, filter dense-front-sparse-back pattern
  std::vector<double> point_spacings;
  point_spacings.reserve(enu_points.size() - 1);
  for (size_t i = 1; i < enu_points.size(); ++i) {
    double spacing = std::hypot(enu_points[i].x() - enu_points[i - 1].x(),
                                 enu_points[i].y() - enu_points[i - 1].y());
    point_spacings.emplace_back(spacing);
  }
  
  if (point_spacings.empty()) {
    if (dist_to_max_curv != nullptr) {
      *dist_to_max_curv = 1000.0;
    }
    return 0.0;
  }
  
  // Check for dense-front-sparse-back pattern
  if (point_spacings.size() >= 2) {
    double last_spacing = point_spacings.back();
    
    // Calculate average spacing of front part (excluding last spacing)
    double front_avg_spacing = 0.0;
    const size_t front_size = point_spacings.size() - 1;
    for (size_t i = 0; i < front_size; ++i) {
      front_avg_spacing += point_spacings[i];
    }
    front_avg_spacing /= static_cast<double>(front_size);
    
    JSON_DEBUG_VALUE("ramp_points_front_avg_spacing", front_avg_spacing);
    JSON_DEBUG_VALUE("ramp_points_last_spacing", last_spacing);
    
    // Check if dense-front-sparse-back: last spacing is abnormally large
    bool has_dense_front_then_sparse_back = false;
    if (front_avg_spacing > 1e-6 && 
        last_spacing > kMaxRampPointSpacing && 
        last_spacing > front_avg_spacing * kMaxRampPointSpacingRatio) {
      // Count dense points in front part (spacing < average spacing)
      // Only consider as dense-front-sparse-back if dense points <= threshold
      int dense_count = 0;
      for (size_t i = 0; i < front_size; ++i) {
        if (point_spacings[i] < front_avg_spacing) {
          dense_count++;
        }
      }
      if (dense_count <= kMaxDensePointCountForSparseBack) {
        has_dense_front_then_sparse_back = true;
      }
    }
    
    JSON_DEBUG_VALUE("ramp_points_has_dense_front_sparse_back", has_dense_front_then_sparse_back ? 1.0 : 0.0);
    
    if (has_dense_front_then_sparse_back) {
      if (dist_to_max_curv != nullptr) {
        *dist_to_max_curv = 1000.0;
      }
      return 0.0;
    }
  }

  // Build cumulative s and 3-point geometry curvature
  std::vector<double> s_vec;
  s_vec.reserve(enu_points.size());
  s_vec.emplace_back(0.0);
  for (size_t i = 1; i < enu_points.size(); ++i) {
    double ds = std::hypot(enu_points[i].x() - enu_points[i - 1].x(),
                           enu_points[i].y() - enu_points[i - 1].y());
    s_vec.emplace_back(s_vec.back() + ds);
  }

  const double total_s = s_vec.back();
  JSON_DEBUG_VALUE("ramp_curv_sample_total_s", total_s);
  JSON_DEBUG_VALUE("ramp_curv_sample_points_num",
                   static_cast<int>(enu_points.size()));

  if (total_s < 1.0) {
    if (dist_to_max_curv != nullptr) {
      *dist_to_max_curv = 1000.0;
    }
    return 0.0;
  }

  std::vector<double> k_raw_local;
  k_raw_local.reserve(enu_points.size());
  k_raw_local.emplace_back(0.0);  // First point not calculated
  for (size_t i = 1; i + 1 < enu_points.size(); ++i) {
    const auto &p0 = enu_points[i - 1];
    const auto &p1 = enu_points[i];
    const auto &p2 = enu_points[i + 1];

    const double a = std::hypot(p1.x() - p0.x(), p1.y() - p0.y());
    const double b = std::hypot(p2.x() - p1.x(), p2.y() - p1.y());
    const double c = std::hypot(p2.x() - p0.x(), p2.y() - p0.y());

    const double denom = a * b * c;
    if (denom < 1e-3) {
      k_raw_local.emplace_back(0.0);
      continue;
    }

    const double cross = (p1.x() - p0.x()) * (p2.y() - p0.y()) -
                         (p1.y() - p0.y()) * (p2.x() - p0.x());
    const double area = 0.5 * std::fabs(cross);
    const double k = 4.0 * area / denom;
    k_raw_local.emplace_back(k);
  }
  k_raw_local.emplace_back(0.0);  // Last point not calculated
  
  // Output k_raw if requested
  if (k_raw != nullptr) {
    *k_raw = k_raw_local;
  }
  
  // Output s_vec if requested
  if (s_vec_output != nullptr) {
    *s_vec_output = s_vec;
  }

  // Smooth curvature with sliding window (40m), optimized to O(n) with two pointers
  std::vector<double> k_smooth(k_raw_local.size(), 0.0);
  constexpr double window_len = 40.0;
  constexpr double half_window = window_len * 0.5;
  
  if (k_raw_local.empty() || s_vec.empty() || k_raw_local.size() != s_vec.size()) {
    // Handle boundary case
    if (dist_to_max_curv != nullptr) {
      *dist_to_max_curv = 1000.0;
    }
    return 0.0;
  }
  
  // Maintain sliding window with two pointers
  size_t left = 0;   // Left boundary of window
  size_t right = 0;  // Right boundary of window
  double sum_k = 0.0;
  int cnt = 0;
  
  for (size_t i = 0; i < k_raw_local.size(); ++i) {
    const double center_s = s_vec[i];
    const double window_left = center_s - half_window;
    const double window_right = center_s + half_window;
    
    // Remove points on left side of window
    while (left < k_raw_local.size() && s_vec[left] < window_left) {
      sum_k -= k_raw_local[left];
      --cnt;
      ++left;
    }
    
    // Add points on right side of window
    while (right < k_raw_local.size() && s_vec[right] <= window_right) {
      sum_k += k_raw_local[right];
      ++cnt;
      ++right;
    }
    
    // Calculate average
    if (cnt > 0) {
      k_smooth[i] = sum_k / static_cast<double>(cnt);
    }
  }

  double max_k = 0.0;
  double max_k_s = 0.0;  // Distance to max curvature point
  double first_small_radius_s = -1.0;  // First position with radius < 65m
  for (size_t i = 0; i < k_smooth.size(); ++i) {
    if (k_smooth[i] > max_k) {
      max_k = k_smooth[i];
      max_k_s = s_vec[i];
    }
    // Check for first point with radius < 65m
    if (k_smooth[i] > 1e-6) {
      double radius = 1.0 / k_smooth[i];
      if (radius < kMapRadiusFirstEnterForDisCal && first_small_radius_s < 0) {
        first_small_radius_s = s_vec[i];
      }
    }
  }

  // Prefer first position with radius < 65m, otherwise use max curvature position
  if (first_small_radius_s >= 0) {
    max_k_s = first_small_radius_s;
  }

  JSON_DEBUG_VALUE("ramp_curv_max_k", max_k);
  if (max_k > 1e-6) {
    double min_radius = 1.0 / max_k;
    JSON_DEBUG_VALUE("ramp_curv_min_radius", min_radius);
  }

  // Calculate distance to max curvature point
  if (dist_to_max_curv != nullptr) {
    if (is_on_ramp) {
      *dist_to_max_curv = max_k_s;
    } else {
      *dist_to_max_curv = dis_to_ramp + max_k_s;
    }
    JSON_DEBUG_VALUE("ramp_curv_dist_to_max_curv", *dist_to_max_curv);
  }

  return max_k;
}

double SpeedLimitDecider::GetRampVelLimit() {
  const auto &environmental_model = session_->environmental_model();
  const auto &route_info_output =
      environmental_model.get_route_info()->get_route_info_output();
  uint64_t ramp_link_id = -1;
  double ramp_v_limit = 120;
  double dis_to_ramp = route_info_output.dis_to_ramp;
  const auto &split_region_info_list = route_info_output.split_region_info_list;
  if (dis_to_ramp < 2000.0) {
    for (int i = 0; i < split_region_info_list.size(); ++i) {
      if (std::fabs(split_region_info_list[i].distance_to_split_point -
                    dis_to_ramp) < 1.0) {
        ramp_link_id = split_region_info_list[i].split_link_id;
        break;
      }
    }
  }
  if (environmental_model.get_route_info()->get_sdpromap_valid()) {
    const auto &sdpro_map = environmental_model.get_route_info()->get_sdpro_map();
    const auto ramp_link = sdpro_map.GetNextLinkOnRoute(ramp_link_id);
    if (ramp_link == nullptr) {
      ILOG_INFO << "ramp_link is null!!!";
    } else {
      ramp_v_limit = ramp_link->speed_limit();
    }
  }
  return ramp_v_limit;

}

bool SpeedLimitDecider::IsNearMergeCancelRampVelLimit() {
  const auto &environmental_model = session_->environmental_model();
  const auto &route_info_output =
      environmental_model.get_route_info()->get_route_info_output();
  double dis_to_merge =
      route_info_output.merge_region_info_list.empty()
          ? NL_NMAX
          : route_info_output.merge_region_info_list[0].distance_to_split_point;
  if (!route_info_output.is_ramp_merge_to_road_on_expressway) {
    dis_to_merge = NL_NMAX;
  }
  dis_to_merge_window_.pop_front();
  dis_to_merge_window_.push_back(dis_to_merge);
  if (dis_to_merge_window_[0] < kFarEnoughDisToMerge && dis_to_merge_window_[1] < kFarEnoughDisToMerge &&
      dis_to_merge_window_[2] < kFarEnoughDisToMerge) {
    distance_to_merge_ = dis_to_merge;
  } else if (dis_to_merge_window_[0] > kFarEnoughDisToMerge && dis_to_merge_window_[1] > kFarEnoughDisToMerge &&
    dis_to_merge_window_[2] > kFarEnoughDisToMerge) {
    distance_to_merge_ = NL_NMAX;
  }
  JSON_DEBUG_VALUE("distance_to_merge", distance_to_merge_);
  if (route_info_output.is_on_ramp) {
    if (distance_to_merge_ < kCloseDisToMergeCancelVLimit) {
      pass_merge_counter_ = 0;
      pass_merge_counter_has_set_ = false;
      return true;
    }
  } else {
    //ego is going to pass merge point
    if (distance_to_merge_ < kShortDisReachMerge && dis_to_merge > kFarEnoughDisToMerge) {
      //this counter mainly solve the problem of gaode vlimit update late
      if (pass_merge_counter_has_set_) {
        pass_merge_counter_--;
      } else {
        pass_merge_counter_has_set_ = true;
        pass_merge_counter_ = kFarAwayMergeCounterNums;
      }
      return true;
    } else if (distance_to_merge_ > kFarEnoughDisToMerge && pass_merge_counter_ > 0) {
      pass_merge_counter_--;
      return true;
    } else {
      pass_merge_counter_ = 0;
      pass_merge_counter_has_set_ = false;
      return false;
    }
  }
  pass_merge_counter_ = 0;
  pass_merge_counter_has_set_ = false;
  return false;
}

void SpeedLimitDecider::CalculateCurveSpeedLimit() {
  ILOG_DEBUG << "----CalculateCurveSpeedLimit---";
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

  // Limit longitudinal velocity for safe turn
  double acc_lat_max =
      interp(std::fabs(angle_steers_deg), _AY_MAX_ABS_BP, _AY_MAX_STEERS);
  // Disable steering limit at high velocity
  bool is_high_vel = v_ego > kHighVel;
  double v_limit_steering = 100.0;
  if (!is_high_vel) {
    v_limit_steering = std::sqrt((acc_lat_max * steer_ratio * wheel_base) /
                                 std::max(std::fabs(angle_steers), 0.001));
  }
  double v_limit_in_turns = v_limit_steering;
  // Calculate velocity limit based on road curvature
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
  const pnc::mathlib::spline &raw_spline =
      reference_path_ptr->GetRawCurveSpline();
  //bool is_ref_path_smoothed = reference_path_ptr->GetIsSmoothed();
  bool is_ref_path_smoothed = false;
  std::vector<CurvInfo> preview_curv_info_vec;
  for (int idx = 0; idx * 2.0 < preview_x; idx++) {
    CurvInfo one_curv_info;
    if (is_ref_path_smoothed) {
      ReferencePathPoint refpath_pt;
      if (reference_path_ptr->get_reference_point_by_lon(
              ego_start_s + idx * 2.0, refpath_pt)) {
        one_curv_info.curv = std::fabs(refpath_pt.path_point.kappa());
        one_curv_info.curv_sign = refpath_pt.path_point.kappa() > 0 ? 1 : -1;
      } else {
        one_curv_info.curv = 0.0001;
        one_curv_info.curv_sign = 0;
      }
    } else {
      // Calculate average curvature in window
      std::vector<double> curv_window_vec;
      int curv_sign = 0;
      for (int j = -6; j <= 6; j++) {
        double curv = 0.0001;
        ReferencePathPoint refpath_pt;
        if (reference_path_ptr->get_reference_point_by_lon(
                ego_start_s + idx * 2.0 + j * 2.0, refpath_pt)) {
          curv = std::fabs(refpath_pt.path_point.kappa());
          if (j == 0) {
            curv_sign = refpath_pt.path_point.kappa() > 0 ? 1 : -1;
          }
        }
        curv_window_vec.emplace_back(curv);
      }
      double curv_sum = 0.0;
      for (int ind = 0; ind < curv_window_vec.size(); ++ind) {
        curv_sum = curv_sum + curv_window_vec[ind];
      }
      double avg_curv = curv_sum / curv_window_vec.size();
      one_curv_info.curv = avg_curv;
      one_curv_info.curv_sign = curv_sign;
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

  double v_limit_road = 40.0;
  double road_radius = 10000.0;
  const auto &ref_curve_info = reference_path_ptr->GetReferencePathCurveInfo();
  bool is_s_bend = (ref_curve_info.curve_type ==
                    ReferencePathCurveInfo::CurveType::SHARP_CURVE) &&
                   IsSSharpBend(preview_curv_info_vec);
  double max_curv = 0.0001;
  double max_curv_s = 0.0;  // Distance to max curvature point
  for (int idx = 0; idx < preview_curv_info_vec.size(); idx++) {
    if (preview_curv_info_vec[idx].curv > max_curv) {
      max_curv = preview_curv_info_vec[idx].curv;
      max_curv_s = preview_curv_info_vec[idx].s;
    }
  }
  double road_radius_origin = 1 / std::max(max_curv, 0.0001);
  

  // Distance to max curvature point
  double dist_to_max_curv = max_curv_s;
  
  // Calculate average radius in 0-80m range and conditionally use it for EWMA
  double avg_radius_0_80m = 10000.0;
  bool use_avg_radius_for_ewma = false;
  
  // Collect curvature data for 0-80m range
  std::vector<double> radius_vec_0_80m;
  radius_vec_0_80m.reserve(static_cast<size_t>(preview_distance_0_80m / sampling_step) + 1);
  for (double s = 0.0; s <= preview_distance_0_80m; s += sampling_step) {
    ReferencePathPoint refpath_pt;
    if (reference_path_ptr->get_reference_point_by_lon(ego_start_s + s, refpath_pt)) {
      double curv = std::fabs(refpath_pt.path_point.kappa());
      if (curv > 1e-6) {
        double radius = 1.0 / curv;
        radius_vec_0_80m.emplace_back(radius);
      }
    }
  }
  
  // Calculate median radius and filtered average radius
  if (speed_limit_config_.enable_avg_radius_for_ewma && !radius_vec_0_80m.empty()) {
    // 1) compute median radius in 0-80m
    std::vector<double> radius_sorted = radius_vec_0_80m;
    std::sort(radius_sorted.begin(), radius_sorted.end());
    const size_t n = radius_sorted.size();
    double median_radius = 0.0;
    if (n % 2 == 1) {
      median_radius = radius_sorted[n / 2];
    } else {
      median_radius = 0.5 * (radius_sorted[n / 2 - 1] + radius_sorted[n / 2]);
    }

    // 2) compute mean radius, excluding outliers > 2 * median_radius
    double radius_sum = 0.0;
    size_t valid_cnt = 0;
    const double radius_upper_bound = 2.0 * median_radius;
    for (const auto &r : radius_vec_0_80m) {
      if (r <= radius_upper_bound) {
        radius_sum += r;
        ++valid_cnt;
      }
    }
    if (valid_cnt > 0) {
      avg_radius_0_80m = radius_sum / static_cast<double>(valid_cnt);
    } else {
      // Fallback: if all points are filtered out, use median radius
      avg_radius_0_80m = median_radius;
    }
    
    // Check conditions: use road_radius_origin to lookup expected speed
    double acc_lat_max_origin = interp(0.5*(road_radius_origin + last_road_radius_origin_), _AY_MAX_CURV_BP, _AY_MAX_CURV_V);
    double v_expected_origin = std::sqrt(acc_lat_max_origin * road_radius_origin);
    double speed_diff = std::fabs(v_expected_origin - v_ego);
    
    // Hysteresis logic: enter when speed_diff < kEWMAAvgRadiusEnterSpeedDiff && road_radius_origin > kEWMAAvgRadiusEnterRadius
    //                   exit when speed_diff >= kEWMAAvgRadiusExitSpeedDiff || road_radius_origin < kEWMAAvgRadiusExitRadius
    if (last_use_avg_radius_for_ewma_) {
      // Currently in the state, check exit conditions
      if (speed_diff >= kAvgRadiusExitSpeedDiff || road_radius_origin < kAvgRadiusExitRadius) {
        use_avg_radius_for_ewma = false;
      } else {
        use_avg_radius_for_ewma = true;
      }
    } else {
      // Currently not in the state, check enter conditions
      if (speed_diff < kAvgRadiusEnterSpeedDiff && road_radius_origin > kAvgRadiusEnterRadius) {
        use_avg_radius_for_ewma = true;
      } else {
        use_avg_radius_for_ewma = false;
      }
    }
    
    // Update last state for next iteration
    last_use_avg_radius_for_ewma_ = use_avg_radius_for_ewma;
  }
  last_road_radius_origin_ = road_radius_origin;
  // Apply EWMA filter
  double curv_for_ewma = max_curv;
  if (use_avg_radius_for_ewma) {
    // Convert average radius back to curvature
    curv_for_ewma = 1.0 / std::max(avg_radius_0_80m, 0.0001);
  }
  
  // Calculate dynamic EWMA alpha based on radius
  double radius_for_ewma = 1.0 / std::max(curv_for_ewma, 0.0001);
  double ewma_alpha = 0.2;  // Default value
  if (radius_for_ewma < _EWMA_ALPHA_RADIUS_BP.front()) {
    // Below lower bound, use maximum alpha
    ewma_alpha = _EWMA_ALPHA_V.front();
  } else if (radius_for_ewma > _EWMA_ALPHA_RADIUS_BP.back()) {
    // Above upper bound, use minimum alpha
    ewma_alpha = _EWMA_ALPHA_V.back();
  } else {
    // Interpolate within bounds
    ewma_alpha = interp(radius_for_ewma, _EWMA_ALPHA_RADIUS_BP, _EWMA_ALPHA_V);
  }
  
  if (raw_curv_spline_ < kEpsilon) {
    raw_curv_spline_ = curv_for_ewma;
  } else {
    raw_curv_spline_ =
        ewma_alpha * curv_for_ewma + (1 - ewma_alpha) * raw_curv_spline_;
  }
  road_radius = 1 / std::max(raw_curv_spline_, 0.0001);
  // if (road_radius < 400) {
  acc_lat_max = interp(road_radius, _AY_MAX_CURV_BP, _AY_MAX_CURV_V);
  //}
  v_limit_road = std::sqrt(acc_lat_max * road_radius);
  if (is_s_bend) {
    v_limit_road = v_limit_road * kSSharpBendSpeedScaleRatio;
  }
  v_limit_in_turns = std::min(v_limit_in_turns, v_limit_road);
  
  // Apply EWMA filter to v_limit_in_turns
  if (v_limit_in_turns_filtered_ < kEpsilon) {
    v_limit_in_turns_filtered_ = v_limit_in_turns;
  } else {
    double ewma_alpha_spd = speed_limit_config_.ewma_alpha_v_limit_in_turns;
    v_limit_in_turns_filtered_ =
        ewma_alpha_spd * v_limit_in_turns + 
        (1 - ewma_alpha_spd) * v_limit_in_turns_filtered_;
  }
  v_limit_in_turns = v_limit_in_turns_filtered_;

  ILOG_DEBUG << "road_radius is :" << road_radius
             << ", acc_lat_max:" << acc_lat_max;

  ILOG_DEBUG << "angle_steers :" << angle_steers
             << ", angle_steers_deg:" << angle_steers_deg
             << ", v_limit_road:" << v_limit_road
             << ", dist_to_max_curv:" << dist_to_max_curv;
  JSON_DEBUG_VALUE("v_limit_road", v_limit_road);
  JSON_DEBUG_VALUE("road_radius", road_radius);
  JSON_DEBUG_VALUE("is_s_bend", is_s_bend ? 1 : 0);
  JSON_DEBUG_VALUE("road_radius_origin", road_radius_origin);
  JSON_DEBUG_VALUE("dist_to_max_curv", dist_to_max_curv);
  JSON_DEBUG_VALUE("avg_radius_0_80m", avg_radius_0_80m);
  JSON_DEBUG_VALUE("use_avg_radius_for_ewma", use_avg_radius_for_ewma ? 1 : 0);

  // Determine sharp curve with hysteresis
  bool is_sharp_curve = false;
  if (last_is_sharp_curve_) {
    // Exit when radius >= exit threshold
    is_sharp_curve = (road_radius < kSharpCurveExitThreshold);
  } else {
    // Enter when radius < enter threshold
    is_sharp_curve = (road_radius < kSharpCurveEnterThreshold);
  }
  last_is_sharp_curve_ = is_sharp_curve;

  // Calculate required deceleration
  double required_deceleration = 0.0;
  if (dist_to_max_curv > kMinDistanceForDecel && v_ego > v_limit_road) {
    required_deceleration = (std::pow(v_limit_road, 2) - std::pow(v_ego, 2)) /
                            (2.0 * dist_to_max_curv);
  } else if (dist_to_max_curv <= kMinDistanceForDecel &&
             (v_ego - v_limit_road) > kCurvSpeedDifference) {
    required_deceleration = -2.0; 
  }

  // Determine sharp curve by deceleration with frame count hysteresis
  bool is_sharp_curve_by_decel = false;
  
  if (speed_limit_config_.enable_sharp_curve_by_decel) {
    bool should_enter_sharp_curve =
        (required_deceleration < kCurvatureDecelThreshold) && is_sharp_curve;
    if (last_is_sharp_curve_by_decel_) {
      if (should_enter_sharp_curve) {
        is_sharp_curve_by_decel = true;
        sharp_curve_frame_count_ = 0;
      } else {
        // Maintain state for minimum frames
        if (sharp_curve_frame_count_ < kSharpCurveMinFrames) {
          is_sharp_curve_by_decel = true;
          sharp_curve_frame_count_++;
        } else {
          is_sharp_curve_by_decel = false;
          sharp_curve_frame_count_ = 0;
        }
      }
    } else {
      if (should_enter_sharp_curve) {
        is_sharp_curve_by_decel = true;
        sharp_curve_frame_count_ = 0;
      } else {
        is_sharp_curve_by_decel = false;
        sharp_curve_frame_count_ = 0;
      }
    }
  } else {
    // Reset if disabled
    is_sharp_curve_by_decel = false;
    sharp_curve_frame_count_ = 0;
  }
  last_is_sharp_curve_by_decel_ = is_sharp_curve_by_decel;

  // Determine map sharp curve based on ramp curvature with hysteresis
  bool is_map_sharp_curve = false;
  double dist_to_ramp_max_curv = 0.0;
  
  if (speed_limit_config_.enable_map_sharp_curve_speed_limit) {
    const auto &route_info_output =
        environmental_model.get_route_info()->get_route_info_output();
    double dis_to_ramp = route_info_output.dis_to_ramp;
    bool is_on_ramp = route_info_output.is_on_ramp;
    
    bool condition_ramp_location = (dis_to_ramp < speed_limit_config_.map_sharp_curve_dis_to_ramp) || is_on_ramp;
    bool condition_ramp_curv = false;
    bool condition_ramp_raw_count = false;
    
    if (condition_ramp_location) {
      std::vector<double> k_raw;
      std::vector<double> s_vec;
      double ramp_max_curv = CalcRampMaxCurvFromSDProMap(&dist_to_ramp_max_curv, &k_raw, &s_vec);
      if (ramp_max_curv > 1e-6) {
        double ramp_min_radius = 1.0 / ramp_max_curv;
        // Hysteresis logic based on ramp radius
        if (last_is_map_sharp_curve_ramp_) {
          // Exit when radius >= exit threshold
          condition_ramp_curv = (ramp_min_radius < kMapSharpCurveRadiusExit);
        } else {
          // Enter when radius < enter threshold
          condition_ramp_curv = (ramp_min_radius < kMapSharpCurveRadiusEnter);
        }
      } else {
        // Keep last state if curvature unavailable
        condition_ramp_curv = last_is_map_sharp_curve_ramp_;
      }
      
      // Check if k_raw has more than threshold count of points with radius < kMapSharpCurveRadiusEnter
      // Hysteresis logic: enter when count > 4, exit when count < 3
      if (!k_raw.empty()) {
        int count_small_radius = 0;
        int first_small_radius_idx = -1;
        int last_small_radius_idx = -1;
        for (size_t i = 0; i < k_raw.size(); ++i) {
          const double k = k_raw[i];
          if (k > 1e-6) {
            double radius = 1.0 / k;
            if (radius < kMapSharpCurveRadiusEnter) {
              count_small_radius++;
              if (first_small_radius_idx < 0) {
                first_small_radius_idx = static_cast<int>(i);
              }
              last_small_radius_idx = static_cast<int>(i);
            }
          }
        }
        
        // Calculate distance between first and last point with small radius
        bool distance_condition = false;
        if (first_small_radius_idx >= 0 && last_small_radius_idx >= 0 &&
            first_small_radius_idx != last_small_radius_idx &&
            !s_vec.empty() && last_small_radius_idx < static_cast<int>(s_vec.size()) &&
            first_small_radius_idx < static_cast<int>(s_vec.size())) {
          double distance = std::fabs(s_vec[last_small_radius_idx] - s_vec[first_small_radius_idx]);
          distance_condition = (distance > kMapSharpCurveMinDistance);
        }
        
        // Hysteresis logic based on raw count
        if (last_condition_ramp_raw_count_) {
          // Exit when count < exit threshold (3)
          condition_ramp_raw_count = !(count_small_radius < kMapSharpCurveRawCountExit);
        } else {
          // Enter when count > enter threshold (4) and distance condition is satisfied
          condition_ramp_raw_count = (count_small_radius > kMapSharpCurveRawCountEnter) && distance_condition;
        }
      } else {
        // Keep last state if k_raw is empty
        condition_ramp_raw_count = last_condition_ramp_raw_count_;
      }
    } else {
      condition_ramp_curv = false;
      condition_ramp_raw_count = false;
    }
    
    // Calculate base condition without distance check
    bool base_condition = condition_ramp_location && condition_ramp_curv && condition_ramp_raw_count && (!IsNearMergeCancelRampVelLimit());
    
    // Check if entering state (from false to true)
    bool is_entering = !last_is_map_sharp_curve_ && base_condition;
    
    // Distance condition check (only when entering)
    bool distance_condition_for_enter = true;
    if (is_entering) {
      // Get ego vehicle speed
      const auto ego_state_mgr = environmental_model.get_ego_state_manager();
      double v_ego_kph = ego_state_mgr->ego_v() * 3.6;  // Convert m/s to kph
      
      // Determine distance threshold based on speed
      double dist_threshold = 0.0;
      if (v_ego_kph > 70.0) {
        dist_threshold = kMapSharpCurveDistThresholdV70;
      } else if (v_ego_kph > 60.0) {
        dist_threshold = kMapSharpCurveDistThresholdV60;
      } else if (v_ego_kph > 50.0) {
        dist_threshold = kMapSharpCurveDistThresholdV50;
      } else if (v_ego_kph > 45.0) {
        dist_threshold = kMapSharpCurveDistThresholdV45;
      } else {
        // Speed <= 50kph, no distance limit
        dist_threshold = 0.0;  // Large value to pass condition
      }
      
      // Check distance condition only when entering
      distance_condition_for_enter = (dist_to_ramp_max_curv < dist_threshold);
    }
    
    // Final condition: base condition and (not entering or distance condition satisfied)
    is_map_sharp_curve = base_condition && distance_condition_for_enter;
    
    last_is_map_sharp_curve_ramp_ = condition_ramp_curv;
    last_condition_ramp_raw_count_ = condition_ramp_raw_count;
    last_is_map_sharp_curve_ = is_map_sharp_curve;
  } else {
    last_is_map_sharp_curve_ramp_ = false;
    last_condition_ramp_raw_count_ = false;
    last_is_map_sharp_curve_ = false;
  }
  
  // Apply map sharp curve speed limit
  double v_limit_map_sharp_curve = 100.0;
  double map_sharp_curve_required_decel = 0.0;
  if (is_map_sharp_curve) {
    v_limit_map_sharp_curve = speed_limit_config_.map_sharp_curve_speed_limit;
    
    // Calculate required deceleration
    if (dist_to_ramp_max_curv > kMinDistanceForDecel && v_ego > v_limit_map_sharp_curve) {
      map_sharp_curve_required_decel = (std::pow(v_limit_map_sharp_curve, 2) - std::pow(v_ego, 2)) /
                                       (2.0 * dist_to_ramp_max_curv);
    } else if (dist_to_ramp_max_curv <= kMinDistanceForDecel &&
               (v_ego - v_limit_map_sharp_curve) > kCurvSpeedDifference) {
      map_sharp_curve_required_decel = -2.0;
    }
  }
  v_limit_in_turns = std::min(v_limit_in_turns, v_limit_map_sharp_curve);
  
  // Priority: is_sharp_curve_by_decel > map_sharp_curve
  SpeedLimitType v_limit_type = SpeedLimitType::CURVATURE;

  if (is_sharp_curve_by_decel ||
      (speed_limit_config_.enable_map_sharp_curve_by_decel &&
       is_map_sharp_curve &&
       map_sharp_curve_required_decel < kCurvatureDecelThreshold)) {
    // Highest priority: sharp curve by deceleration
    v_limit_type = SpeedLimitType::SHARP_CURVATURE;
  }

  if (v_limit_in_turns < v_target_) {
    v_target_ = v_limit_in_turns;
    v_target_type_ = v_limit_type;
  }
  
  JSON_DEBUG_VALUE("is_map_sharp_curve", is_map_sharp_curve ? 1 : 0);
  JSON_DEBUG_VALUE("v_limit_map_sharp_curve", v_limit_map_sharp_curve);
  JSON_DEBUG_VALUE("v_limit_steering", v_limit_steering);
  JSON_DEBUG_VALUE("v_limit_in_turns", v_limit_in_turns);
  JSON_DEBUG_VALUE("is_sharp_curve", is_sharp_curve ? 1 : 0);
  JSON_DEBUG_VALUE("is_sharp_curve_by_decel", is_sharp_curve_by_decel ? 1 : 0);
  JSON_DEBUG_VALUE("sharp_curve_frame_count", sharp_curve_frame_count_);
  JSON_DEBUG_VALUE("required_deceleration", required_deceleration);
  auto speed_limit_output = session_->mutable_planning_context()
                                ->mutable_speed_limit_decider_output();
  speed_limit_output->SetSpeedLimitIntoMap(v_limit_in_turns, v_limit_type);
}

void SpeedLimitDecider::CalculateMapSpeedLimit() {
  ILOG_DEBUG << "----CalculateMapSpeedLimit for ramp---";
  const auto &environmental_model = session_->environmental_model();

  const auto &function_state_machine_info =
      environmental_model.get_local_view().function_state_machine_info;
  double v_cruise_fsm =
      function_state_machine_info.pilot_req.acc_curise_real_spd;
  double v_cruise_fsm_kph = std::round(v_cruise_fsm * 3.6 / 10.0) * 10;
  const auto &route_info_output =
      environmental_model.get_route_info()->get_route_info_output();
  double dis_to_ramp = route_info_output.dis_to_ramp;
  double dis_to_merge = route_info_output.distance_to_first_road_merge;
  bool is_on_ramp = route_info_output.is_on_ramp;
  double ramp_v_limit = GetRampVelLimit();
  double ramp_max_curv = CalcRampMaxCurvFromSDProMap();

  // set v_cruise_limit by map info
  if (!environmental_model.get_route_info()->get_sdpromap_valid()) {
    ILOG_INFO << "sd_map is invalid!!!";
    // map info invalid, using fsm cruise speed
    v_cruise_limit_ = v_cruise_fsm_kph;
  }
  const auto &sdpro_map = environmental_model.get_route_info()->get_sdpro_map();

  ad_common::math::Vec2d current_point;
  const auto &ego_state = environmental_model.get_ego_state_manager();
  const auto &pose = ego_state->location_enu();
  current_point.set_x(pose.position.x);
  current_point.set_y(pose.position.y);
  double nearest_s = 0;
  double nearest_l = 0;
  const double search_distance = 50.0;
  const double max_heading_diff = PI / 4;
  const double ego_heading_angle = ego_state->heading_angle();
  const auto current_segment = sdpro_map.GetNearestLinkWithHeading(
      current_point, search_distance, ego_heading_angle, max_heading_diff,
      nearest_s, nearest_l);
  double v_limit_tencent = 0;
  if (current_segment == nullptr) {
    // get ego link failed, using fsm cruise speed
    v_cruise_limit_ = v_cruise_fsm_kph;
  } else {
    v_limit_tencent = current_segment->speed_limit();  // kph
    v_cruise_limit_ = v_limit_tencent;  // kph
    JSON_DEBUG_VALUE("v_limit_tencent", v_limit_tencent);
  }

  double v_limit_gaode = 0;
  if (!environmental_model.get_route_info()->get_sdmap_valid()) {
    ILOG_INFO << "sd_map is invalid!!!";
  } else {
    const auto &sd_map = environmental_model.get_route_info()
                             ->get_sd_map();  // cur_road_speed_limit()
    if (sd_map.GetNaviRoadInfo() != std::nullopt) {
      v_limit_gaode = sd_map.GetNaviRoadInfo().value().cur_road_speed_limit();
    }
    const auto cur_seg = sd_map.GetNearestRoadWithHeading(
        current_point, search_distance, ego_heading_angle, max_heading_diff,
        nearest_s, nearest_l);
    if (cur_seg != nullptr) {
      auto v_limit_camera_list_info =
          sd_map.GetCameraInfoList(cur_seg->id(), nearest_s, 400.0);
      int camera_num = 10;
      camera_num = v_limit_camera_list_info.size();
      JSON_DEBUG_VALUE("camera_num", camera_num);
    }
    // GetCameraInfoList();
  }
  JSON_DEBUG_VALUE("v_limit_gaode", v_limit_gaode);

  if (v_limit_gaode > 30.0 - kEpsilon) {
    v_cruise_limit_ = v_limit_gaode;
  }

  if (IsNearMergeCancelRampVelLimit() && current_segment != nullptr) {
    if (!(current_segment->link_type() & iflymapdata::sdpro::LinkType::LT_IC) &&
        !(current_segment->link_type() & iflymapdata::sdpro::LinkType::LT_JCT)) {
        v_cruise_limit_ = current_segment->speed_limit();
    } else {
      auto link_id_prev = current_segment->id();
      const iflymapdata::sdpro::LinkInfo_Link* next_link = nullptr;
      do {
        next_link = sdpro_map.GetNextLinkOnRoute(link_id_prev);
        if (next_link == nullptr) {
          break;
        } else if (!(next_link->link_type() & iflymapdata::sdpro::LinkType::LT_IC) &&
                   !(next_link->link_type() & iflymapdata::sdpro::LinkType::LT_JCT)) {
          v_cruise_limit_ = next_link->speed_limit();
          break;
        } else {
          link_id_prev = next_link->id();
        }

      } while (true);

    }
    return;
  }

  const auto virtual_lane_manager =
      environmental_model.get_virtual_lane_manager();
  bool is_continuous_ramp = virtual_lane_manager->is_continuous_ramp();

  double v_target_ramp = 40;
  double v_target_near_ramp_zone = 40;
  double pre_acc_dis = speed_limit_config_.pre_accelerate_distance_for_merge;
  double search_sdmap_curv_dis = speed_limit_config_.search_sdmap_curv_dis;
  double sdpro_min_curv = JudgeCurvBySDProMap(search_sdmap_curv_dis);
  // 通过接口获取是否在匝道的信息
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
      const auto &current_lane = virtual_lane_manager->get_current_lane();
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
        double max_heading_diff = PI / 4;
        double detected_heading_angle =
            detect_merge_front_pnt.path_point.theta();
        ad_common::math::Vec2d detected_point(
            detect_merge_front_pnt.path_point.x(),
            detect_merge_front_pnt.path_point.y());
        if (environmental_model.get_route_info()->get_sdmap_valid()) {
          const auto &sd_map =
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

    if (v_limit_tencent > (30.0 - kEpsilon) && v_limit_gaode > (30.0 - kEpsilon)) {
      v_cruise_limit_ = std::max(std::min(v_limit_tencent, v_limit_gaode), 60.0);
    } else {
      v_cruise_limit_ = std::max(v_cruise_limit_, 60.0);
    }

    if (dis_to_ramp < 1000.0) {
      double ramp_in_ramp_v_limit = GetRampVelLimit();
      if (ramp_in_ramp_v_limit < std::max(v_cruise_limit_, v_cruise_fsm_kph)) {
        double pre_brake_dis_to_ramp_in_ramp = std::max(dis_to_ramp - 50, 0.0);
        v_target_ramp = std::pow(
            std::pow(std::max(speed_limit_config_.v_limit_ramp, ramp_in_ramp_v_limit / 3.6),
                    2.0) -
                2 * pre_brake_dis_to_ramp_in_ramp * speed_limit_config_.acc_to_ramp,
            0.5);
        if (v_target_ramp > std::max(v_cruise_limit_ / 3.6, v_cruise_fsm)) {
          v_target_ramp = std::max(v_cruise_limit_ / 3.6, v_cruise_fsm);
          double speed_increase_in_ramp = v_cruise_fsm - last_v_cruise_fsm_ramp_;
          if (ramp_v_limit_set_ && speed_increase_in_ramp > kCAManualInterventionSpeedDetected) {
            ramp_manual_intervention_detected_ = true;
          }
          if (v_target_ramp < v_target_ && !ramp_manual_intervention_detected_) {
            ramp_v_limit_set_ = true;
            v_target_ = v_target_ramp;
            v_target_type_ = SpeedLimitType::MAP_ON_RAMP;
          }
          last_v_cruise_fsm_ramp_ = v_cruise_fsm;
        } else {
          if (v_target_ramp < v_target_) {
            v_target_ = v_target_ramp;
            v_target_type_ = SpeedLimitType::MAP_NEAR_RAMP;
          }
          ramp_v_limit_set_ = false;
          ramp_manual_intervention_detected_ = false;
          last_v_cruise_fsm_ramp_ = 40.0;
        }
        //v_target_ramp = std::min(std::max(v_cruise_limit_ / 3.6, v_cruise_fsm), v_target_ramp);
        ILOG_DEBUG << "v_target_ramp :" << v_target_ramp;
        JSON_DEBUG_VALUE("v_target_ramp", v_target_ramp);
        JSON_DEBUG_VALUE("dis_to_ramp", dis_to_ramp);
        JSON_DEBUG_VALUE("dis_to_merge", dis_to_merge);
        auto speed_limit_output = session_->mutable_planning_context()
                                  ->mutable_speed_limit_decider_output();
        speed_limit_output->SetSpeedLimitIntoMap(v_target_ramp,
                                             SpeedLimitType::MAP_ON_RAMP);
        return;
      }
    }

    double speed_increase = v_cruise_fsm - last_v_cruise_fsm_ramp_;
    if (ramp_v_limit_set_ && speed_increase > kCAManualInterventionSpeedDetected) {
      ramp_manual_intervention_detected_ = true;
    }

    v_target_ramp = v_cruise_limit_ / 3.6;
    if (v_target_ramp < v_target_ && !ramp_manual_intervention_detected_) {
      ramp_v_limit_set_ = true;
      v_target_ = v_target_ramp;
      v_target_type_ = SpeedLimitType::MAP_ON_RAMP;
    }

    last_v_cruise_fsm_ramp_ = v_cruise_fsm;
    ILOG_DEBUG << "v_target_ramp :" << v_target_ramp;
    JSON_DEBUG_VALUE("v_target_ramp", v_target_ramp);
    JSON_DEBUG_VALUE("dis_to_ramp", dis_to_ramp);
    JSON_DEBUG_VALUE("dis_to_merge", dis_to_merge);
    auto speed_limit_output = session_->mutable_planning_context()
                                  ->mutable_speed_limit_decider_output();
    speed_limit_output->SetSpeedLimitIntoMap(v_target_ramp,
                                             SpeedLimitType::MAP_ON_RAMP);
    return;
  }
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
    v_target_type_ = SpeedLimitType::MAP_NEAR_RAMP;
  }

  ramp_v_limit_set_ = false;
  ramp_manual_intervention_detected_ = false;
  last_v_cruise_fsm_ramp_ = 40.0;
  ILOG_DEBUG << "dis_to_ramp :" << dis_to_ramp;
  ILOG_DEBUG << "v_target_ramp :" << v_target_ramp;
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
  ILOG_DEBUG << "----calc_speed_limit_from_tfl_dis---";
  const auto &local_view = session_->environmental_model().get_local_view();
  auto fsm_state = local_view.function_state_machine_info.current_state;
  bool noa_mode = (fsm_state == iflyauto::FunctionalState_NOA_ACTIVATE) ||
                  (fsm_state == iflyauto::FunctionalState_NOA_OVERRIDE);

  double v_limit_tfl_dis = 40.0;
  const auto &environmental_model = session_->environmental_model();
  const auto tfl_manager =
      environmental_model.get_traffic_light_decision_manager();
  const auto traffic_status = tfl_manager->GetTrafficStatus();
  double dis_tfl = tfl_manager->GetNearestTFLDis();
  if (speed_limit_config_.enable_tfl_v_limit && dis_tfl < kTFLSpeedLimitDis &&
      (!noa_mode)) {
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
  ILOG_DEBUG << "----calc_speed_limit_for_intersection---";
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

void SpeedLimitDecider::CalculateSpeedLimitForDangerousObstacle() {
  LOG_DEBUG("----calc_speed_limit_for_dangerous_obstacle--- \n");
  const auto &environmental_model = session_->environmental_model();
  const auto ego_state_mgr = environmental_model.get_ego_state_manager();
  const auto init_point = ego_state_mgr->planning_init_point();
  double v_ego = ego_state_mgr->ego_v();
  double v_cruise = ego_state_mgr->ego_v_cruise();
  const auto &function_state_machine_info =
      environmental_model.get_local_view().function_state_machine_info;
  double v_cruise_fsm =
      function_state_machine_info.pilot_req.acc_curise_real_spd;
  double v_target_for_dangerous_obs = 40.0;
  if (!speed_limit_config_.enable_dangerous_obs_speed_limit) {
    JSON_DEBUG_VALUE("v_target_for_dangerous_obs", v_target_for_dangerous_obs);
    return;
  }
  if (v_ego > speed_limit_config_.high_speed_scene_ego_v_thred ||
      v_cruise > speed_limit_config_.high_speed_scene_cruise_v_thred ||
      v_cruise_fsm > speed_limit_config_.high_speed_scene_cruise_v_thred) {
    JSON_DEBUG_VALUE("v_target_for_dangerous_obs", v_target_for_dangerous_obs);
    return;
  }
  const auto agent_manager =
      session_->environmental_model().get_agent_manager();
  if (agent_manager == nullptr) {
    JSON_DEBUG_VALUE("v_target_for_dangerous_obs", v_target_for_dangerous_obs);
    return;
  }
  std::vector<const agent::Agent *> danger_agents;
  const auto &all_current_agents = agent_manager->GetAllCurrentAgents();
  for (const auto agt_ptr : all_current_agents) {
    if (agt_ptr->is_dangerous() == true && agt_ptr->is_reverse() == false &&
        (agt_ptr->d_rel() > speed_limit_config_.dangerous_obs_lon_dis_low &&
         agt_ptr->d_rel() < speed_limit_config_.dangerous_obs_lon_dis_high)) {
      danger_agents.emplace_back(agt_ptr.get());
    }
  }
  if (danger_agents.empty()) {
    JSON_DEBUG_VALUE("v_target_for_dangerous_obs", v_target_for_dangerous_obs);
    return;
  }
  bool all_agents_static = true;
  for (const auto agt_ptr : danger_agents) {
    if (!agt_ptr->is_static()) {
      all_agents_static = false;
      break;
    }
  }
  if (all_agents_static) {
    if (danger_agents.size() == 1) {
      double lat_dis_static = danger_agents[0]->d_path();
      double vel_by_lat_dis = interp(
          lat_dis_static,
          speed_limit_config_.static_lat_dis_rel_vel_table.lat_dis_table,
          speed_limit_config_.static_lat_dis_rel_vel_table.rel_vel_table);
      v_target_for_dangerous_obs = std::max(
          speed_limit_config_.v_limit_one_still_danger_obs, vel_by_lat_dis);
    } else {
      double min_lat_dis = 100.0;
      for (const auto agt_ptr : danger_agents) {
        if (agt_ptr->d_path() < min_lat_dis) {
          min_lat_dis = agt_ptr->d_path();
        }
      }
      v_target_for_dangerous_obs = interp(
          min_lat_dis,
          speed_limit_config_.static_lat_dis_rel_vel_table.lat_dis_table,
          speed_limit_config_.static_lat_dis_rel_vel_table.rel_vel_table);
      ;
    }
  } else {
    if (danger_agents.size() == 1) {
      if (danger_agents[0]->type() == agent::AgentType::PEDESTRIAN ||
          danger_agents[0]->type() == agent::AgentType::CYCLE_RIDING ||
          danger_agents[0]->type() == agent::AgentType::MOTORCYCLE_RIDING ||
          danger_agents[0]->type() == agent::AgentType::TRICYCLE_RIDING) {
        v_target_for_dangerous_obs =
            danger_agents[0]->speed() +
            interp(danger_agents[0]->d_path(),
                   speed_limit_config_.vru_lat_dis_rel_vel_table.lat_dis_table,
                   speed_limit_config_.vru_lat_dis_rel_vel_table.rel_vel_table);
      } else {
        v_target_for_dangerous_obs =
            danger_agents[0]->speed() +
            interp(
                danger_agents[0]->d_path(),
                speed_limit_config_.vehicle_lat_dis_rel_vel_table.lat_dis_table,
                speed_limit_config_.vehicle_lat_dis_rel_vel_table
                    .rel_vel_table);
      }
    } else {
      // average vel of dangerous_obs by distance weight
      std::vector<double> dis_vec;
      std::vector<double> vel_vec;
      std::vector<double> weight_vec;
      std::vector<double> weight_numerator_vec;
      double weight_denominator = 0.0;
      auto compare_danger_obs_by_dis = [&](const agent::Agent *agt_ptr_1,
                                           const agent::Agent *agt_ptr_2) {
        return std::fabs(agt_ptr_1->d_rel()) < std::fabs(agt_ptr_2->d_rel());
      };
      std::sort(danger_agents.begin(), danger_agents.end(),
                compare_danger_obs_by_dis);
      int used_num = danger_agents.size() > 4 ? 4 : danger_agents.size();
      for (int i = 0; i < used_num; i++) {
        double rel_vel = 40.0;
        if (danger_agents[i]->type() == agent::AgentType::PEDESTRIAN ||
            danger_agents[i]->type() == agent::AgentType::CYCLE_RIDING ||
            danger_agents[i]->type() == agent::AgentType::MOTORCYCLE_RIDING ||
            danger_agents[i]->type() == agent::AgentType::TRICYCLE_RIDING) {
          if (danger_agents[i]->is_static()) {
            double vel_by_lat_dis = interp(
                danger_agents[i]->d_path(),
                speed_limit_config_.static_lat_dis_rel_vel_table.lat_dis_table,
                speed_limit_config_.static_lat_dis_rel_vel_table.rel_vel_table);
            rel_vel = vel_by_lat_dis - danger_agents[i]->speed();
          } else {
            rel_vel = interp(
                danger_agents[i]->d_path(),
                speed_limit_config_.vru_lat_dis_rel_vel_table.lat_dis_table,
                speed_limit_config_.vru_lat_dis_rel_vel_table.rel_vel_table);
          }
        } else {
          if (danger_agents[i]->is_static()) {
            double vel_by_lat_dis = interp(
                danger_agents[i]->d_path(),
                speed_limit_config_.static_lat_dis_rel_vel_table.lat_dis_table,
                speed_limit_config_.static_lat_dis_rel_vel_table.rel_vel_table);
            rel_vel = vel_by_lat_dis - danger_agents[i]->speed();
          } else {
            rel_vel = interp(
                danger_agents[i]->d_path(),
                speed_limit_config_.vehicle_lat_dis_rel_vel_table.lat_dis_table,
                speed_limit_config_.vehicle_lat_dis_rel_vel_table
                    .rel_vel_table);
          }
        }
        vel_vec.emplace_back(danger_agents[i]->speed() + rel_vel);
        dis_vec.emplace_back(
            std::max(std::fabs(danger_agents[i]->d_rel()), 0.1));
      }
      for (int i = 0; i < used_num; i++) {
        double base = 1.0;
        for (int j = 0; j < used_num; j++) {
          if (j == i) {
            continue;
          } else {
            base = base * dis_vec[j];
          }
        }
        weight_denominator = weight_denominator + base;
        weight_numerator_vec.emplace_back(base);
      }
      double avg_vel = 0.0;
      for (int i = 0; i < used_num; i++) {
        avg_vel = avg_vel +
                  (weight_numerator_vec[i] / weight_denominator) * vel_vec[i];
      }
      v_target_for_dangerous_obs = avg_vel;
    }
  }

  if (v_target_for_dangerous_obs < v_target_) {
    v_target_ = v_target_for_dangerous_obs;
    v_target_type_ = SpeedLimitType::DANGEROUS_OBSTACLE;
  }
  JSON_DEBUG_VALUE("v_target_for_dangerous_obs", v_target_for_dangerous_obs);
  auto speed_limit_output = session_->mutable_planning_context()
                                ->mutable_speed_limit_decider_output();
  speed_limit_output->SetSpeedLimitIntoMap(v_target_for_dangerous_obs,
                                           SpeedLimitType::DANGEROUS_OBSTACLE);
}

void SpeedLimitDecider::CalculatePerceptVisibSpeedLimit() {}

void SpeedLimitDecider::CalculatePOISpeedLimit() {
  const auto &environmental_model = session_->environmental_model();
  const auto &route_info_output =
      environmental_model.get_route_info()->get_route_info_output();
  if (!environmental_model.get_route_info()->get_sdpromap_valid()) {
    poi_v_limit_set_ = false;
    return;
  }
  ad_common::math::Vec2d current_point;
  const auto &ego_state = environmental_model.get_ego_state_manager();
  const auto &pose = ego_state->location_enu();
  current_point.set_x(pose.position.x);
  current_point.set_y(pose.position.y);
  const auto &sdpro_map = environmental_model.get_route_info()->get_sdpro_map();
  double nearest_s = 0;
  double nearest_l = 0;
  const double search_distance = 50.0;
  const double max_heading_diff = PI / 4;
  const double ego_heading_angle = ego_state->heading_angle();
  const auto current_segment = sdpro_map.GetNearestLinkWithHeading(
      current_point, search_distance, ego_heading_angle, max_heading_diff,
      nearest_s, nearest_l);
  if (current_segment == nullptr) {
    poi_v_limit_set_ = false;
    return;
  } else {
    poi_v_limit_set_ = false;
    const auto &function_state_machine_info =
      environmental_model.get_local_view().function_state_machine_info;
    double v_cruise_fsm =
      function_state_machine_info.pilot_req.acc_curise_real_spd;
    double v_cruise_fsm_kph = std::round(v_cruise_fsm * 3.6 / 10.0) * 10;
    double cur_road_map_v_limit = current_segment->speed_limit();
    double cur_link_v_limit = std::max(cur_road_map_v_limit, v_cruise_fsm_kph);
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
      // less than calibration distance before entering tunnel speed limit works
      v_cruise_limit_ = speed_limit_config_.tunnel_vel_limit_kph;
      poi_v_limit_set_ = true;
    } else if (tunnel_info.second < kEpsilon &&
               current_segment->link_type() ==
                   iflymapdata::sdpro::LinkType::LT_TUNNEL) {
      // inside tunnel speed limit works
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
        // less than calibration distance before entering toll station speed
        // limit works
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
          // less than calibration distance before entering sapa speed limit
          // works
          v_cruise_limit_ = speed_limit_config_.sapa_vel_limit_kph;
          poi_v_limit_set_ = true;
        } else if (sapa_info.second < kEpsilon &&
                   current_segment->link_type() ==
                       iflymapdata::sdpro::LinkType::LT_SAPA) {
          v_cruise_limit_ = speed_limit_config_.sapa_vel_limit_kph;
          poi_v_limit_set_ = true;
        } else {
          // GetNonExpressInfo, less than calibration distance before entering
          // non-express speed limit works
          v_limit_dis = interp(
              cur_link_v_limit,
              speed_limit_config_.non_express_vel_limit_dis_table
                  .vel_limit_table,
              speed_limit_config_.non_express_vel_limit_dis_table.dis_table);
          auto none_express_info = sdpro_map.GetNonExpressInfo(
              current_segment->id(), nearest_s, 700.0);
          if (none_express_info.first != nullptr &&
              none_express_info.second > 0 &&
              none_express_info.second < v_limit_dis) {
            v_cruise_limit_ = speed_limit_config_.non_express_vel_limit_kph;
            poi_v_limit_set_ = true;
          } else {
            bool function_need_inhibited = false;
            auto speed_limit_output =
                session_->mutable_planning_context()
                    ->mutable_speed_limit_decider_output();
            speed_limit_output->set_function_inhibited_near_roundabout(false);
            auto roundabout_info = sdpro_map.GetRoundAboutInfo(
                current_segment->id(), nearest_s, 300.0);
            if (roundabout_info.first != nullptr &&
                roundabout_info.second < 50.0) {
              // in map mode, roundabout distance meets the condition
              function_need_inhibited = true;
            } else if (roundabout_info.first == nullptr) {
              auto roundabout_info_list = sdpro_map.GetRoundAboutList(
                  current_segment->id(), nearest_s, 300.0);
              if (roundabout_info_list.size() == 0) {
                return;
              } else {
                auto closest_roundabout = roundabout_info_list[0];
                if (closest_roundabout.first != nullptr &&
                    closest_roundabout.second < 60.0) {
                  // not in map mode, closest roundabout distance meets the
                  // condition
                  function_need_inhibited = true;
                }
              }
            }
            if (function_need_inhibited) {
              speed_limit_output->set_function_inhibited_near_roundabout(
                  function_need_inhibited);
            }
          }
        }
      }
    }

    JSON_DEBUG_VALUE("v_target_near_poi", v_cruise_limit_ / 3.6);
    auto speed_limit_output = session_->mutable_planning_context()
                                  ->mutable_speed_limit_decider_output();
    speed_limit_output->SetSpeedLimitIntoMap(v_cruise_limit_ / 3.6,
                                             SpeedLimitType::NEAR_POI);
  }
}

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

  const auto &parallel_longitudinal_avoid_output =
      session_->planning_context().parallel_longitudinal_avoid_decider_output();
  const bool is_parallel_longitudinal_avoid_active =
      parallel_longitudinal_avoid_output.is_need_parallel_longitudinal_avoid();
  const bool is_parallel_overtake =
      parallel_longitudinal_avoid_output.is_parallel_overtake();
  int32_t parallel_overtake_agent_id = -1;
  if (is_parallel_longitudinal_avoid_active && is_parallel_overtake) {
    parallel_overtake_agent_id =
        parallel_longitudinal_avoid_output.parallel_target_agent_id();
  }

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
  bool is_exist_construction = false;
  const auto &construction_scene = session_->environmental_model()
                                       .get_construction_scene_manager()
                                       ->get_construction_scene_output();
  if (construction_scene.is_exist_construction_area &&
      !construction_scene.construction_agent_cluster_attribute_map.empty() &&
      !speed_limit_config_.enable_construction_avoid_agent_speed_limit) {
    is_exist_construction = true;
  }

  std::vector<const agent::Agent *> avoid_agents;
  bool is_triggered_vru_in_avoid_agent = false;
  for (const auto avoid_agent_id : avoid_ids) {
    if (parallel_overtake_agent_id != -1 &&
        avoid_agent_id == parallel_overtake_agent_id) {
      continue;
    }

    if (avoid_agent_id == triggered_vru_.id) {
      is_triggered_vru_in_avoid_agent = true;
    }
    const auto avoid_agent = agent_manager->GetAgent(avoid_agent_id);
    if (avoid_agent != nullptr) {
      avoid_agents.emplace_back(avoid_agent);
    }
  }

  if (triggered_vru_.id != -1 && !is_triggered_vru_in_avoid_agent) {
    const auto avoid_vru = agent_manager->GetAgent(triggered_vru_.id);
    if (avoid_vru != nullptr) {
      avoid_agents.emplace_back(avoid_vru);
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
    const double avoid_agent_v = avoid_agent->speed();
    const bool is_triggered_vru =
        avoid_agent->agent_id() == triggered_vru_.id ? true : false;

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

    if (!CheckLateralConflict(init_point, current_lane, current_lane_coord,
                              avoid_agent, 0.3)) {
      continue;
    }
    if ((avoid_agent->type() == agent::AgentType::WATER_SAFETY_BARRIER ||
         avoid_agent->type() == agent::AgentType::TRAFFIC_CONE ||
         avoid_agent->type() == agent::AgentType::CTASH_BARREL) &&
        is_exist_construction) {
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
    double agent_s = (min_s_by_lat_path + max_s_by_lat_path) * 0.5;
    double d_rel_center = agent_s - ego_s;
    double d_rel =
        min_s_by_lat_path - ego_s - vehicle_param.front_edge_to_rear_axle;
    if (d_rel_center <= 0.0) {
      continue;
    }
    double v_follow_desired =
        CalcDesiredVelocity(d_rel, s_desired, avoid_agent->speed(), v_ego);
    double invade_dis = avoid_agent->is_static() ? 0.8 : 0.4;
    if (avoid_agents_info.find(avoid_agent->agent_id()) !=
        avoid_agents_info.end()) {
      invade_dis = std::max(avoid_agents_info.at(avoid_agent->agent_id()), 0.0);
    }
    // invade_dis = is_triggered_vru ? 0.2 : invade_dis;
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
    // v_limit_lower = is_triggered_vru ? 1.0 : v_limit_lower;
    // const double v_limit_lower = avoid_agent->is_static()
    //                                  ? kStaticAgentAvoidLimitedSpeedHigh
    //                                  : kDynamicAgentAvoidLimitedSpeedHigh;
    // double v_limit_lower_tmp = is_triggered_vru ? 1.0 : avoid_agent_v + 2.0;
    double v_limit_lower_tmp = avoid_agent_v + 2.0;
    v_limit = std::max(v_limit, v_limit_lower);
    v_limit = std::max(v_limit, v_limit_lower_tmp);

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
  auto speed_limit_output = session_->mutable_planning_context()
                                ->mutable_speed_limit_decider_output();
  double v_avoid_tmp = 40.0;
  for (const auto &agent : speed_limit_agents) {
    if (agent.v_limit < v_target_) {
      update_speed_limit(agent.v_limit, agent);
      speed_limit_output->set_avoid_speed_limit_info(agent);
    }
  }
}

void SpeedLimitDecider::CalculateFunctionFadingAwaySpeedLimit() {
  const auto &ego_state_mgr =
      session_->environmental_model().get_ego_state_manager();
  // const auto ego_blinker = ego_state_mgr->ego_blinker();
  const auto &ego_vehi_param =
      VehicleConfigurationContext::Instance()->get_vehicle_param();
  const double ego_rear_axle_to_front_edge =
      ego_vehi_param.length - ego_vehi_param.rear_edge_to_rear_axle;
  const auto &virtual_lane_manager =
      session_->environmental_model().get_virtual_lane_manager();
  const auto current_ego_lane_mark =
      virtual_lane_manager->lane_mark_at_ego_front_edge_pos_current();

  if (!speed_limit_config_.left_right_turn_func_fading_away_switch) {
    JSON_DEBUG_VALUE("v_target_func_fade_away", v_target_)
    return;
  }

  is_function_fading_away_ = false;
  request_reason_ = iflyauto::RequestReason::REQUEST_REASON_NO_REASON;
  static bool is_first_time_funciton_fading_away = true;

  const auto distance_to_stop_line =
      virtual_lane_manager->GetEgoDistanceToStopline();
  const auto distance_to_crosswalk =
      virtual_lane_manager->GetEgoDistanceToCrosswalk();

  // static const std::unordered_set<iflyauto::LaneDrivableDirection>
  //     turning_directions_set = {
  //         iflyauto::LaneDrivableDirection::
  //             LaneDrivableDirection_DIRECTION_RIGHT,
  //         iflyauto::LaneDrivableDirection::LaneDrivableDirection_DIRECTION_LEFT,
  //         iflyauto::LaneDrivableDirection::
  //             LaneDrivableDirection_DIRECTION_UTURN_LEFT,
  //         iflyauto::LaneDrivableDirection::
  //             LaneDrivableDirection_DIRECTION_UTURN_RIGHT,
  //         iflyauto::LaneDrivableDirection::
  //             LaneDrivableDirection_DIRECTION_LEFT_UTURN,
  //         iflyauto::LaneDrivableDirection::
  //             LaneDrivableDirection_DIRECTION_RIGHT_UTURN,
  //         iflyauto::LaneDrivableDirection::
  //             LaneDrivableDirection_DIRECTION_LEFT_RIGHT,
  //         iflyauto::LaneDrivableDirection::
  //             LaneDrivableDirection_DIRECTION_UTURNLEFT_RIGHT};

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

  // const auto &ego_lane_mark_it =
  //     turning_directions_set.find(current_ego_lane_mark);
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

  // JSON_DEBUG_VALUE("ego_blinker", ego_blinker)
  if (!is_function_fading_away_) {
    is_first_time_funciton_fading_away = true;
    JSON_DEBUG_VALUE("v_target_func_fade_away", v_target_)
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
  JSON_DEBUG_VALUE("v_target_func_fade_away", v_target_)
}
void SpeedLimitDecider::CalculateVRURoundSpeedLimit() {
  LOG_DEBUG("----calc_speed_limit_for_vru_round--- \n");
  const auto agent_manager =
      session_->environmental_model().get_agent_manager();
  if (agent_manager == nullptr) {
    vru_round_map_.clear();
    return;
  }
  const auto &agents = agent_manager->GetAllCurrentAgents();
  const auto &agents_set = agent_manager->GetAgentSet();
  if (agents.empty()) {
    vru_round_map_.clear();
    return;
  }

  const auto &virtual_lane_manager =
      session_->environmental_model().get_virtual_lane_manager();
  if (virtual_lane_manager == nullptr) {
    vru_round_map_.clear();
    return;
  }
  const auto &current_lane = virtual_lane_manager->get_current_lane();
  if (current_lane == nullptr) {
    vru_round_map_.clear();
    return;
  }

  const auto &planned_kd_path =
      session_->planning_context().motion_planner_output().lateral_path_coord;
  if (planned_kd_path == nullptr) {
    vru_round_map_.clear();
    return;
  }

  const auto &vehicle_param =
      VehicleConfigurationContext::Instance()->get_vehicle_param();
  const double front_edge_to_rear_axle = vehicle_param.front_edge_to_rear_axle;
  const double rear_edge_to_rear_axle = vehicle_param.rear_edge_to_rear_axle;

  const auto ego_state_mgr =
      session_->environmental_model().get_ego_state_manager();
  double v_ego = ego_state_mgr->ego_v();
  const auto init_point = ego_state_mgr->planning_init_point();

  /* if (init_point.v < kEnterVruRoundEgoVelThr) {
    vru_round_map_.clear();
    return;
  } */
  double ego_s = 0.0;
  double ego_l = 0.0;
  if (!planned_kd_path->XYToSL(init_point.x, init_point.y, &ego_s, &ego_l)) {
    vru_round_map_.clear();
    return;
  }
  const double ego_back_s = ego_s - rear_edge_to_rear_axle;
  const double ego_front_s = ego_s + front_edge_to_rear_axle;

  historical_vru_round_map_ = vru_round_map_;
  for (const auto &agent : agents) {
    if (agent->type() != agent::AgentType::PEDESTRIAN &&
        agent->type() != agent::AgentType::CYCLE_RIDING &&
        agent->type() != agent::AgentType::MOTORCYCLE_RIDING &&
        agent->type() != agent::AgentType::TRICYCLE_RIDING) {
      continue;
    }
    double agent_s = 0.0;
    double agent_l = 0.0;
    bool is_longotidinal_satisfied = false;
    if (!planned_kd_path->XYToSL(agent->x(), agent->y(), &agent_s, &agent_l)) {
      return;
    }
    double agent_to_ego_distance =
        agent_s - ego_front_s - agent->length() * 0.5;

    planning_math::Vec2d agent_pose(agent->x(), agent->y());
    bool is_lateral_buffer_satisfied = false;
    is_lateral_buffer_satisfied = !IsPointOutsideLaneByLeftRightThred(
        current_lane, agent_pose, kExitVruRoundLateralBufferThr,
        kExitVruRoundLateralBufferThr);
    double agent_speed = agent->is_reverse() ? -agent->speed() : agent->speed();
    double ttc = (agent_s) / std::max(init_point.v - agent_speed,
                                      std::numeric_limits<double>::min());
    if (ttc < kEnterVruRoundTtcThr &&
        agent_s > std::numeric_limits<double>::min()) {
      is_longotidinal_satisfied = true;
    }

    if (agent->speed() < kLowSpeedVruVelThr && is_longotidinal_satisfied &&
        is_lateral_buffer_satisfied) {
      if (vru_round_map_.find(agent->agent_id()) != vru_round_map_.end()) {
        vru_round_map_[agent->agent_id()].distance_to_ego =
            agent_to_ego_distance;
        vru_round_map_[agent->agent_id()].ttc = ttc;
        vru_round_map_[agent->agent_id()].id = agent->agent_id();
        vru_round_map_[agent->agent_id()].last_is_satisfied_round =
            vru_round_map_[agent->agent_id()].is_satisfied_round;
        vru_round_map_[agent->agent_id()].is_satisfied_round = true;
        vru_round_map_[agent->agent_id()].enter_counter++;
        vru_round_map_[agent->agent_id()].is_trigger =
            vru_round_map_[agent->agent_id()].enter_counter >=
                    kTriggerVruRoundcounterThr
                ? true
                : false;

      } else {
        vru_round_map_[agent->agent_id()].distance_to_ego =
            agent_to_ego_distance;
        vru_round_map_[agent->agent_id()].ttc = ttc;
        vru_round_map_[agent->agent_id()].id = agent->agent_id();
        vru_round_map_[agent->agent_id()].is_satisfied_round = true;
        vru_round_map_[agent->agent_id()].last_is_satisfied_round = true;
        vru_round_map_[agent->agent_id()].enter_counter = 1;
        vru_round_map_[agent->agent_id()].is_trigger = false;
      }
    } else {
      // erase unsatisfied vru
      vru_round_map_.erase(agent->agent_id());
    }
  }
  for (const auto &historical_vru : historical_vru_round_map_) {
    if (agents_set.find(historical_vru.first) == agents_set.end()) {
      vru_round_map_.erase(historical_vru.first);
    }
  }

  if (HasTriggeredVRU(vru_round_map_)) {
    if (triggered_vru_.enter_counter == kTriggerVruRoundcounterThr) {
      vru_round_triggered_ = true;
    }
    JSON_DEBUG_VALUE("vru_round_triggered_id", triggered_vru_.id);

  } else {
    vru_round_triggered_ = false;
  }
}

bool SpeedLimitDecider::HasTriggeredVRU(
    const std::map<int32_t, VRURoundInfo> &vru_round_map) {
  triggered_vru_.id = -1;
  triggered_vru_.is_trigger = false;
  if (vru_round_map_.empty()) {
    return false;
  }
  int32_t triggered_vru_id = -1;
  double min_distance = std::numeric_limits<double>::max();
  for (const auto &vru : vru_round_map_) {
    if (vru.second.is_trigger && vru.second.distance_to_ego < min_distance) {
      min_distance = vru.second.distance_to_ego;
      triggered_vru_id = vru.first;
    }
  }

  if (triggered_vru_id != -1 && min_distance < kEnterVruRoundDistanceThr) {
    triggered_vru_ = vru_round_map_[triggered_vru_id];
    return true;
  } else {
    return false;
  }
}
void SpeedLimitDecider::CalculateConstructionZoneSpeedLimit() {
  ILOG_DEBUG << "----CalculateConstructionZoneSpeedLimit for Construction---";

  double v_target_construction = 50.0;
  double v_target_near_construction = 50.0;
  double dis_to_construction = std::numeric_limits<double>::max();
  int construction_strong_mode_reason = 0;

  // Configuration flag: enable/disable construction zone speed limiting
  if (!speed_limit_config_.enable_construction_speed_limit) {
    JSON_DEBUG_VALUE("v_target_construction", v_target_construction);
    JSON_DEBUG_VALUE("v_target_near_construction", v_target_near_construction);
    JSON_DEBUG_VALUE("dis_to_construction", dis_to_construction);
    JSON_DEBUG_VALUE("construction_strong_deceleration_mode",
                     construction_strong_deceleration_mode_);
    JSON_DEBUG_VALUE("construction_strong_mode_reason",
                     construction_strong_mode_reason);
    JSON_DEBUG_VALUE("construction_strong_mode_frame_count",
                     construction_strong_mode_frame_count_);
    JSON_DEBUG_VALUE("construction_lat_dist_flag", construction_lat_dist_flag_);
    return;
  }
  // Check if the construction zone is valid
  const auto &construction_scene =
      session_->environmental_model()
          .get_construction_scene_manager()
          ->get_construction_scene_output();
  if (!construction_scene.is_exist_construction_area ||
      construction_scene.construction_agent_cluster_attribute_map.empty()) {
    construction_strong_deceleration_mode_ = false;
    construction_strong_mode_frame_count_ = 0;
    construction_lat_dist_flag_ = false;
    construction_v_limit_set_ = false;
    construction_manual_intervention_detected_ = false;
    last_v_cruise_fsm_ = 40.0;
    ILOG_DEBUG << "Construction scene is invalid or empty";
    JSON_DEBUG_VALUE("v_target_construction", v_target_construction);
    JSON_DEBUG_VALUE("v_target_near_construction", v_target_near_construction);
    JSON_DEBUG_VALUE("dis_to_construction", dis_to_construction);
    JSON_DEBUG_VALUE("construction_strong_deceleration_mode",
                     construction_strong_deceleration_mode_);
    JSON_DEBUG_VALUE("construction_strong_mode_reason",
                     construction_strong_mode_reason);
    JSON_DEBUG_VALUE("construction_strong_mode_frame_count",
                     construction_strong_mode_frame_count_);
    JSON_DEBUG_VALUE("construction_lat_dist_flag", construction_lat_dist_flag_);
    JSON_DEBUG_VALUE("construction_manual_intervention_detected", construction_manual_intervention_detected_);
    return;
  }

  bool is_exist_construction = construction_scene.is_exist_construction_area;
  bool is_on_construction = construction_scene.is_pass_construction_area;
  const auto &environmental_model = session_->environmental_model();
  const auto &function_state_machine_info =
      environmental_model.get_local_view().function_state_machine_info;
  double v_cruise_fsm =
      function_state_machine_info.pilot_req.acc_curise_real_spd;
  const auto ego_state_mgr = environmental_model.get_ego_state_manager();
  double v_ego = ego_state_mgr->ego_v();
  const auto init_point = ego_state_mgr->planning_init_point();
  double v_cruise = ego_state_mgr->ego_v_cruise();
  // Construction manual intervention detected
  double speed_increase = v_cruise_fsm - last_v_cruise_fsm_;
  if (construction_v_limit_set_ && is_exist_construction &&
      speed_increase > kCAManualInterventionSpeedDetected) {
    construction_manual_intervention_detected_ = true;
  }
  last_v_cruise_fsm_ = v_cruise_fsm;

  // get lateral path
  const auto &planned_kd_path =
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
  for (const auto & [ cluster_id, construction_area ] :
       construction_scene.construction_agent_cluster_attribute_map) {
    auto agent_info = construction_area.points.front();
    if (!planned_kd_path->XYToSL(agent_info.x, agent_info.y, &construction_s,
                                 &construction_l)) {
      continue;
    }
    if (construction_s < construction_s_nearest) {
      construction_s_nearest = construction_s;
    }
  }
  // Construciotn need strong deceleration
  std::vector<SLPoint> sl_construction_points_all;
  sl_construction_points_all.reserve(20);
  double construction_nearest_l = std::numeric_limits<double>::max();
  for (const auto & [ cluster_id, construction_area ] :
       construction_scene.construction_agent_cluster_attribute_map) {
    for (const auto &pt : construction_area.points) {
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
  //
  int construction_invade_count = 0;
  for (const auto &p : sl_construction_points_all) {
    if (std::abs(p.l) < speed_limit_config_.ca_invade_entry_lat_dis_thr) {
      construction_invade_count++;
    }
  }
  // Check entry and exit conditions for strong deceleration
  bool enter_condition = false;
  bool exit_condition = false;

  if (construction_invade_count >=
      speed_limit_config_.ca_invade_lat_dis_counter_thr) {
    enter_condition = true;
    construction_strong_mode_reason = 1;
  } else if (CheckClustersConsecutiveDiffSlidingWindow(
                 construction_scene.construction_agent_cluster_attribute_map,
                 planned_kd_path, true)) {
    enter_condition = true;
    construction_strong_mode_reason = 2;
  }

  if (std::abs(construction_nearest_l) >
      speed_limit_config_.ca_invade_exit_lat_dis_thr) {
    exit_condition = true;
    construction_strong_mode_reason = 11;
  } else if (CheckClustersConsecutiveDiffSlidingWindow(
                 construction_scene.construction_agent_cluster_attribute_map,
                 planned_kd_path, false)) {
    exit_condition = true;
    construction_strong_mode_reason = 12;
  }
  //
  if (!construction_strong_deceleration_mode_) {
    if (enter_condition) {
      construction_strong_deceleration_mode_ = true;
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
      construction_strong_deceleration_mode_ = false;
      construction_strong_mode_frame_count_ = 0;
    } else if (exit_condition && !min_duration_met) {
      construction_strong_mode_reason = 30;
    }
  }
  ILOG_DEBUG << "construction_strong_deceleration_mode :"
             << construction_strong_deceleration_mode_;
  JSON_DEBUG_VALUE("construction_strong_deceleration_mode",
                   construction_strong_deceleration_mode_);
  JSON_DEBUG_VALUE("construction_strong_mode_reason",
                   construction_strong_mode_reason);
  JSON_DEBUG_VALUE("construction_strong_mode_frame_count",
                   construction_strong_mode_frame_count_);
  // Construction Lateral Dis Flag
  if (!construction_lat_dist_flag_) {
    if (std::abs(construction_nearest_l) <
        speed_limit_config_.construction_lat_dist_entry) {
      construction_lat_dist_flag_ = true;
    }
  } else {
    if (std::abs(construction_nearest_l) >
        speed_limit_config_.construction_lat_dist_exit) {
      construction_lat_dist_flag_ = false;
    }
  }
  JSON_DEBUG_VALUE("construction_lat_dist_flag", construction_lat_dist_flag_);
  // Construction zone info
  dis_to_construction = std::max(construction_s_nearest - ego_s, 0.0);
  if (is_on_construction && construction_lat_dist_flag_) {
    if (construction_strong_deceleration_mode_) {
      v_target_construction =
          speed_limit_config_.v_limit_construction -
          std::max(speed_limit_config_.construction_invade_speed_diff, 0.0);
    } else {
      v_target_construction = speed_limit_config_.v_limit_construction;
    }
    double v_target_construction_kph =
        std::round(v_target_construction * 3.6 / 10.0) * 10;
    if (v_target_construction_kph < v_cruise_limit_) {
      v_cruise_limit_ = v_target_construction_kph;
      construction_v_limit_set_ = true;
    }
    if (v_target_construction < v_target_ && !construction_manual_intervention_detected_) {
      construction_v_limit_set_ = true;
      v_target_ = v_target_construction;
      v_target_type_ = SpeedLimitType::ON_CONSTRUCTION;
    } else if(v_target_ > speed_limit_config_.construction_speed_upper && construction_manual_intervention_detected_){
      v_target_ = speed_limit_config_.construction_speed_upper;
      v_target_type_ = SpeedLimitType::ON_CONSTRUCTION;
    }
    JSON_DEBUG_VALUE("construction_manual_intervention_detected", construction_manual_intervention_detected_);
    ILOG_DEBUG << "v_target_construction :" << v_target_construction;
    JSON_DEBUG_VALUE("v_target_construction", v_target_construction);
    JSON_DEBUG_VALUE("v_target_near_construction", v_target_near_construction);
    JSON_DEBUG_VALUE("dis_to_construction", dis_to_construction);
    auto speed_limit_output = session_->mutable_planning_context()
                                  ->mutable_speed_limit_decider_output();
    speed_limit_output->SetSpeedLimitIntoMap(v_target_construction,
                                             SpeedLimitType::ON_CONSTRUCTION);
    return;
  }
  double construction_speed_threshold =
      speed_limit_config_.construction_speed_threshold;
  double v_limit_near_construction =
      speed_limit_config_.v_limit_near_construction;
  if (construction_strong_deceleration_mode_) {
    construction_speed_threshold =
        construction_speed_threshold -
        std::max(speed_limit_config_.construction_invade_speed_diff, 0.0);
    v_limit_near_construction =
        v_limit_near_construction -
        std::max(speed_limit_config_.construction_invade_speed_diff, 0.0);
  }

  if (dis_to_construction <= speed_limit_config_.dis_near_construction &&
      v_cruise > construction_speed_threshold && is_exist_construction &&
      construction_lat_dist_flag_) {
    double pre_brake_dis_near_construction = std::max(
        dis_to_construction - speed_limit_config_.brake_dis_near_construction,
        0.0);
    v_target_near_construction =
        std::pow(std::pow(v_limit_near_construction, 2.0) -
                     2 * pre_brake_dis_near_construction *
                         speed_limit_config_.acc_to_construction,
                 0.5);
  }
  double v_target_near_construction_kph =
      std::round(v_target_near_construction * 3.6 / 10.0) * 10;
  if (v_target_near_construction_kph < v_cruise_limit_) {
    v_cruise_limit_ = v_target_near_construction_kph;
    construction_v_limit_set_ = true;
    if (v_target_ > speed_limit_config_.construction_speed_upper) {
      v_target_ = speed_limit_config_.construction_speed_upper;
      v_target_type_ = SpeedLimitType::NEAR_CONSTRUCTION;
    }

    if (v_target_near_construction < v_target_ &&
        !construction_manual_intervention_detected_) {
      construction_v_limit_set_ = true;
      v_target_ = v_target_near_construction;
      v_target_type_ = SpeedLimitType::NEAR_CONSTRUCTION;
    } else if (v_target_ > speed_limit_config_.construction_speed_upper &&
               construction_manual_intervention_detected_) {
      v_target_ = speed_limit_config_.construction_speed_upper;
      v_target_type_ = SpeedLimitType::NEAR_CONSTRUCTION;
    }
  } else {
    construction_v_limit_set_ = false;
    construction_manual_intervention_detected_ = false;
  }
  JSON_DEBUG_VALUE("construction_manual_intervention_detected", construction_manual_intervention_detected_);
  ILOG_DEBUG << "dis_to_construction :" << dis_to_construction;
  ILOG_DEBUG << "v_target_near_construction :" << v_target_near_construction;
  auto speed_limit_output = session_->mutable_planning_context()
                                  ->mutable_speed_limit_decider_output();
  speed_limit_output->SetSpeedLimitIntoMap(v_target_near_construction,
                                             SpeedLimitType::NEAR_CONSTRUCTION);
  JSON_DEBUG_VALUE("v_target_construction", v_target_construction);
  JSON_DEBUG_VALUE("v_target_near_construction", v_target_near_construction);
  JSON_DEBUG_VALUE("dis_to_construction", dis_to_construction);
}

}  // namespace planning





