#include "speed_limit_decider.h"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <map>
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
#include "config/basic_type.h"

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
const std::vector<double> kEwmaAlphaRadiusBreakpoints{150.0, 300.0, 500.0, 600.0};
const std::vector<double> kEwmaAlphaValues{0.3, 0.15, 0.1, 0.05};
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
constexpr double kRoadBoundarySharpDecelThreshold = -1.2;  // Deceleration threshold for ROAD_BOUNDARY_SHARP_DECEL (m/s²)
constexpr double kRoadBoundaryCooldownDeltaThreshold = 1.11;  // m/s threshold to treat limit change as significant
constexpr double kRoadBoundaryDefaultLimitMps = 100.0;  // Default value meaning no road boundary limit applied
constexpr double kCurvatureDefaultLimitMps = 100.0;  // Default value meaning no curvature limit applied
constexpr double kConstructionDefaultLimitMps = 100.0;  // Default value meaning no construction limit applied
constexpr double kRampDefaultLimitMps = 100.0;  // Default value meaning no ramp limit applied
constexpr double kCurvSpeedDifference = 3 / 3.6;
constexpr int kSharpCurveMinFrames = 5;  // Min frames to maintain sharp curve state
constexpr double kMinRampSampleLength = 150.0;
constexpr double kMapSharpCurveRadiusEnter = 100.0;  // Enter radius threshold for map sharp curve (m)
constexpr double kMapSharpCurveRadiusExit = 120.0;  // Exit radius threshold with hysteresis (m)
constexpr int kMapSharpCurveRawCountEnter = 4;  // Enter threshold count of k_raw points with radius < kMapSharpCurveRadiusEnter
constexpr int kMapSharpCurveRawCountExit = 3;  // Exit threshold count of k_raw points with radius < kMapSharpCurveRadiusEnter (hysteresis)
constexpr double kMapRadiusFirstEnterForDisCal = 65.0;  // Radius threshold 65m
constexpr double kMapSharpCurveMinDistance = 20.0;  // Minimum distance between first and last point satisfying condition (m)
constexpr double kMapSharpCurveSpeedLimitThreshold = 60.0;  // Speed limit threshold (kph) to skip counting small radius points
constexpr double kMapSharpCurveGroupingDistanceThreshold = 40.0;
constexpr size_t kRampCurvatureVectorReserveSize = 50;  // Expected size for ramp curvature data vectors (~50 points for 150m ramp)
constexpr size_t kRampCurvatureIndicesReserveSize = 30;  // Expected size for turn direction indices (fewer points satisfy small radius condition)
constexpr double kMapSharpCurveDistThresholdV70 = 125.0;  // Distance threshold to sharp curve when speed > 70kph (m)
constexpr double kMapSharpCurveDistThresholdV60 = 75.0;   // Distance threshold to sharp curve when speed > 60kph (m)
constexpr double kMapSharpCurveDistThresholdV50 = 35.0;   // Distance threshold to sharp curve when speed > 50kph (m)
constexpr double kMapSharpCurveDistThresholdV45 = 17.0;   // Distance threshold to sharp curve when speed > 45kph (m)
constexpr double kMaxRampPointSpacing = 40.0;  // Maximum point spacing threshold, beyond which is considered sparse segment (m)
constexpr double kMaxRampPointSpacingRatio = 2.5;  // Ratio threshold of max spacing to average spacing, beyond which abnormal sparse segment is detected
constexpr int kMaxDensePointCountForSparseBack = 3;  // Maximum dense point count threshold for determining dense-front-sparse-back pattern
constexpr double kMaxDistanceToRamp = 2000.0;
constexpr double kDistanceTolerance = 1.0;
constexpr double kCurvaturePreviewDistance = 80.0;  // Preview distance for curvature calculation (m)
constexpr double kAvgRadiusEnterSpeedDiff = 1.5;  // Speed difference threshold for entering avg radius EWMA (m/s)
constexpr double kAvgRadiusEnterRadius = 350.0;  // Road radius threshold for entering avg radius EWMA (m)
constexpr double kAvgRadiusExitSpeedDiff = 3.0;  // Speed difference threshold for exiting avg radius EWMA (m/s)
constexpr double kAvgRadiusExitRadius = 280.0;  // Road radius threshold for exiting avg radius EWMA (m)
constexpr double map_curv_window_len = 25.0;
constexpr double kMapModeRoundaboutQuitDis = 50.0;
constexpr double kNoMapModeRoundaboutQuitDis = 60.0;
constexpr double kRoundaboutQuitCurvRadiusThr = 300.0;
constexpr double kRoundaboutQuitRecoverCounter = 3;
constexpr double kRoadBoundarySearchDistanceOffset = 25.0;  // Offset distance added to search_distance_min for road boundary search (m)
constexpr double kRoadBoundaryDefaultVLimit = 40.0;

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
  // 10. speed limit from road boundary
  CalculateRoadBoundarySpeedLimit();
  // 11. speed limit from construction
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
  bool is_search_cur_link = true;
  // const double max_search_length = 7000.0;  // 搜索7km范围内得地图信息
  const double search_distance = 50.0;
  const double max_heading_diff = PI / 4;
  const double ego_heading_angle = ego_state->heading_angle();
  const auto current_segment = sdpro_map.GetNearestLinkWithHeading(
      current_point, search_distance, ego_heading_angle, max_heading_diff,
      nearest_s, nearest_l, is_search_cur_link);
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
    const std::vector<MapSplitRegionInfo> &split_region_info_list) {
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
    std::vector<ad_common::math::Vec2d> &enu_points, double &total_len,
    std::vector<double> *map_speed_limits) {
  const iflymapdata::sdpro::LinkInfo_Link *cur_link = start_link;
  while (cur_link != nullptr && is_ramp(*cur_link) &&
         total_len < kMinRampSampleLength) {
    // Check if link has valid points before collecting
    const auto &pts = cur_link->points().boot().points();
    if (pts.size() < 2) {
      // Invalid link, break
      break;
    }
    // Get speed limit for current link
    double cur_link_speed_limit = cur_link->speed_limit();
    // Record number of points before collecting
    size_t points_before = enu_points.size();
    // Collect points from current link
    CollectPointsFromLink(cur_link, enu_points, total_len, 0);
    // Add speed limits for newly collected points
    if (map_speed_limits != nullptr) {
      size_t points_after = enu_points.size();
      size_t points_added = points_after - points_before;
      for (size_t i = 0; i < points_added; ++i) {
        map_speed_limits->emplace_back(cur_link_speed_limit);
      }
    }

    if (total_len >= kMinRampSampleLength) {
      break;
    }
    cur_link = get_next_link(cur_link->id());
  }
}

// Collect and process ramp curvature data from SDProMap
// Supports both on-ramp and approaching-ramp cases
void SpeedLimitDecider::CollectRampCurvatureData(
    std::vector<double>* k_raw,
    std::vector<double>* s_vec_output, std::vector<double>* map_speed_limits,
    std::vector<double>* k_smooth_output) {
  const auto &environmental_model = session_->environmental_model();
  const auto &route_info = environmental_model.get_route_info();
  if (!route_info->get_sdpromap_valid()) {
    return;
  }

  const auto &route_info_output = route_info->get_route_info_output();
  const auto &sdpro_map = route_info->get_sdpro_map();
  bool is_on_ramp = route_info_output.is_on_ramp;
  double dis_to_ramp = route_info_output.dis_to_ramp;

  const iflymapdata::sdpro::LinkInfo_Link *start_link = nullptr;

  // Collect ENU points on ramp until total length >= 150m or leaving ramp
  std::vector<ad_common::math::Vec2d> enu_points;
  enu_points.reserve(50);
  std::vector<double> map_speed_limits_local;  // Local storage for map speed limits
  if (map_speed_limits != nullptr) {
    map_speed_limits_local.reserve(50);
  }
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
    bool is_search_cur_link = true;
    const double search_distance = 50.0;
    const double max_heading_diff = PI / 4;
    const double ego_heading_angle = ego_state->heading_angle();
    const auto current_segment = sdpro_map.GetNearestLinkWithHeading(
        current_point, search_distance, ego_heading_angle, max_heading_diff,
        nearest_s, nearest_l, is_search_cur_link);

    if (current_segment == nullptr) {
      return;
    }
    // Verify current link is ramp
    if (!sdpro_map.isRamp(current_segment->link_type())) {
      return;
    }

    start_link = current_segment;
    const iflymapdata::sdpro::LinkInfo_Link *cur_link = start_link;
    const auto &pts = cur_link->points().boot().points();
    if (pts.size() < 2) {
      return;
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
    double cur_link_speed_limit = cur_link->speed_limit();
    size_t points_before = enu_points.size();
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
    // Add speed limits for newly collected points
    if (map_speed_limits != nullptr) {
      size_t points_after = enu_points.size();
      size_t points_added = points_after - points_before;
      for (size_t i = 0; i < points_added; ++i) {
        map_speed_limits_local.emplace_back(cur_link_speed_limit);
      }
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
                                    total_len, map_speed_limits != nullptr ? &map_speed_limits_local : nullptr);
      }
    }

  } else {
    // Case 2: Approaching ramp, start from next link of ramp_link_id
    const auto &split_region_info_list =
        route_info_output.map_split_region_info_list;
    uint64_t ramp_link_id = FindRampLinkId(dis_to_ramp, split_region_info_list);

    if (ramp_link_id == static_cast<uint64_t>(-1)) {
      return;
    }

    const auto ramp_link = sdpro_map.GetNextLinkOnRoute(ramp_link_id);
    if (ramp_link == nullptr) {
      return;
    }

    start_link = ramp_link;
    auto get_next_link = [&sdpro_map](uint64_t link_id) {
      return sdpro_map.GetNextLinkOnRoute(link_id);
    };
    CollectRampPointsFromLinks(start_link, is_ramp, get_next_link, enu_points,
                               total_len, map_speed_limits != nullptr ? &map_speed_limits_local : nullptr);
  }

  // Calculate max curvature using 3-point geometry method
  if (enu_points.size() < 3) {
    return;
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
    return;
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
    // Keep sign: positive for left turn, negative for right turn
    // This allows opposite turns to cancel out during sliding window filtering
    const double k = (cross >= 0 ? 1.0 : -1.0) * (4.0 * area / denom);
    k_raw_local.emplace_back(k);
  }
  k_raw_local.emplace_back(0.0);  // Last point not calculated

  // Output k_raw if requested (keep sign for internal processing)
  if (k_raw != nullptr) {
    *k_raw = k_raw_local;
  }

  // Output s_vec if requested
  // Update s_vec when is_on_ramp is false (add dis_to_ramp offset for unified distance calculation)
  if (s_vec_output != nullptr) {
    if (!is_on_ramp) {
      std::vector<double> s_vec_updated = s_vec;
      for (size_t i = 0; i < s_vec_updated.size(); ++i) {
        s_vec_updated[i] += dis_to_ramp;
      }
      *s_vec_output = s_vec_updated;
    } else {
      *s_vec_output = s_vec;
    }
  }

  // Output map_speed_limits if requested
  if (map_speed_limits != nullptr) {
    *map_speed_limits = map_speed_limits_local;
  }

  // Smooth curvature with sliding window (40m), optimized to O(n) with two pointers
  std::vector<double> k_smooth(k_raw_local.size(), 0.0);
  double half_window = map_curv_window_len * 0.5;

  if (k_raw_local.empty() || s_vec.empty() || k_raw_local.size() != s_vec.size()) {
    // Handle boundary case
    return;
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

  // Output k_smooth if requested
  if (k_smooth_output != nullptr) {
    *k_smooth_output = k_smooth;
  }
}

double SpeedLimitDecider::GetRampVelLimit() {
  const auto &environmental_model = session_->environmental_model();
  const auto &route_info_output =
      environmental_model.get_route_info()->get_route_info_output();
  uint64_t ramp_link_id = -1;
  double ramp_v_limit = 120;
  double dis_to_ramp = route_info_output.dis_to_ramp;
  const auto &split_region_info_list = route_info_output.map_split_region_info_list;
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
      route_info_output.map_merge_region_info_list.empty()
          ? NL_NMAX
          : route_info_output.map_merge_region_info_list[0].distance_to_merge_point;
  if (!route_info_output.gaode_route_info_output.is_ramp_merge_to_road_on_expressway) {
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
    road_radius_origin_ = 10000.0;
    JSON_DEBUG_VALUE("road_radius_origin", road_radius_origin_);
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
  road_radius_origin_ = road_radius_origin;

  // Distance to max curvature point
  double dist_to_max_curv = max_curv_s;

  // Calculate average radius in 0-80m range and conditionally use it for EWMA
  double avg_radius_0_80m = 10000.0;
  bool use_avg_radius_for_ewma = false;

  // Collect curvature data for 0-80m range
  std::vector<double> radius_vec_0_80m;
  radius_vec_0_80m.reserve(static_cast<size_t>(kCurvaturePreviewDistance / kSamplingStep) + 1);
  for (double s = 0.0; s <= kCurvaturePreviewDistance; s += kSamplingStep) {
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
  if (radius_for_ewma < kEwmaAlphaRadiusBreakpoints.front()) {
    // Below lower bound, use maximum alpha
    ewma_alpha = kEwmaAlphaValues.front();
  } else if (radius_for_ewma > kEwmaAlphaRadiusBreakpoints.back()) {
    // Above upper bound, use minimum alpha
    ewma_alpha = kEwmaAlphaValues.back();
  } else {
    // Interpolate within bounds
    ewma_alpha = interp(radius_for_ewma, kEwmaAlphaRadiusBreakpoints, kEwmaAlphaValues);
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
      // Pre-allocate vectors based on expected ramp sample size
      std::vector<double> k_raw_signed;
      k_raw_signed.reserve(kRampCurvatureVectorReserveSize);
      std::vector<double> s_vec;
      s_vec.reserve(kRampCurvatureVectorReserveSize);
      std::vector<double> map_speed_limits;
      map_speed_limits.reserve(kRampCurvatureVectorReserveSize);
      std::vector<double> k_smooth;
      k_smooth.reserve(kRampCurvatureVectorReserveSize);
      CollectRampCurvatureData(&k_raw_signed, &s_vec, &map_speed_limits, &k_smooth);

      // Step 1: Determine curve direction by counting left and right turn points separately
      int map_sharp_curve_direction = 0;  // 1 for left turn, -1 for right turn, 0 if not determined
      int count_left_turn = 0;   // Count of left turn points with small radius
      int count_right_turn = 0;  // Count of right turn points with small radius
      // Pre-allocate indices vectors (typically fewer points satisfy small radius condition)
      std::vector<int> left_turn_indices;
      left_turn_indices.reserve(kRampCurvatureIndicesReserveSize);
      std::vector<int> right_turn_indices;
      right_turn_indices.reserve(kRampCurvatureIndicesReserveSize);

      if (!k_raw_signed.empty() && k_raw_signed.size() == map_speed_limits.size()) {
        // Count left and right turn points separately
        for (size_t i = 0; i < k_raw_signed.size(); ++i) {
          const double k_signed = k_raw_signed[i];
          if (std::fabs(k_signed) > 1e-6) {
            double radius = 1.0 / std::fabs(k_signed);
            if (radius < kMapSharpCurveRadiusEnter) {
              // Check if map speed limit > threshold, if so, skip counting this point
              if (map_speed_limits[i] <= kMapSharpCurveSpeedLimitThreshold) {
                if (k_signed >= 0) {
                  // Left turn
                  count_left_turn++;
                  left_turn_indices.push_back(static_cast<int>(i));
                } else {
                  // Right turn
                  count_right_turn++;
                  right_turn_indices.push_back(static_cast<int>(i));
                }
              }
            }
          }
        }
        // Group indices by S distance (gap > threshold) and select the group with most points
        auto group_and_select = [&s_vec](const std::vector<int>& indices) -> std::vector<int> {
          if (indices.empty() || s_vec.empty()) {
            return indices;
          }

          // Create pairs of (index, s_value) and sort by S value
          std::vector<std::pair<int, double>> index_s_pairs;
          index_s_pairs.reserve(indices.size());
          for (int idx : indices) {
            if (idx >= 0 && static_cast<size_t>(idx) < s_vec.size()) {
              index_s_pairs.push_back({idx, s_vec[idx]});
            }
          }

          if (index_s_pairs.empty()) {
            return indices;
          }

          // Sort by S value
          std::sort(index_s_pairs.begin(), index_s_pairs.end(),
                    [](const std::pair<int, double>& a, const std::pair<int, double>& b) {
                      return a.second < b.second;
                    });

          // Group indices based on S distance gap > threshold
          std::vector<std::vector<int>> groups;
          std::vector<int> current_group;
          current_group.push_back(index_s_pairs[0].first);

          for (size_t i = 1; i < index_s_pairs.size(); ++i) {
            double s_diff = index_s_pairs[i].second - index_s_pairs[i-1].second;
            if (s_diff > kMapSharpCurveGroupingDistanceThreshold) {
              // Start a new group
              groups.push_back(current_group);
              current_group.clear();
            }
            current_group.push_back(index_s_pairs[i].first);
          }
          // Add the last group
          if (!current_group.empty()) {
            groups.push_back(current_group);
          }

          // If no grouping occurred (all points are in one group), return original
          if (groups.size() <= 1) {
            return indices;
          }

          // Find the group with most points (if equal, choose the earlier one)
          size_t max_group_idx = 0;
          size_t max_size = groups[0].size();
          for (size_t i = 1; i < groups.size(); ++i) {
            if (groups[i].size() > max_size) {
              max_size = groups[i].size();
              max_group_idx = i;
            }
          }

          return groups[max_group_idx];
        };

        // Apply grouping to left_turn_indices
        if (!left_turn_indices.empty()) {
          left_turn_indices = group_and_select(left_turn_indices);
          count_left_turn = static_cast<int>(left_turn_indices.size());
        }

        // Apply grouping to right_turn_indices
        if (!right_turn_indices.empty()) {
          right_turn_indices = group_and_select(right_turn_indices);
          count_right_turn = static_cast<int>(right_turn_indices.size());
        }


        // Determine curve direction based on point count
        if (count_left_turn > count_right_turn) {
          map_sharp_curve_direction = 1;  // Left turn
        } else if (count_right_turn > count_left_turn) {
          map_sharp_curve_direction = -1;  // Right turn
        } else if (count_left_turn > 0 || count_right_turn > 0) {
          // Counts are equal, determine by max curvature direction
          double max_curv_left = 0.0;
          double max_curv_right = 0.0;
          for (size_t i = 0; i < k_raw_signed.size(); ++i) {
            const double k_signed = k_raw_signed[i];
            if (std::fabs(k_signed) > 1e-6) {
              double k_abs = std::fabs(k_signed);
              if (k_signed >= 0) {
                if (k_abs > max_curv_left) {
                  max_curv_left = k_abs;
                }
              } else {
                if (k_abs > max_curv_right) {
                  max_curv_right = k_abs;
                }
              }
            }
          }
          // Choose direction with larger max curvature
          map_sharp_curve_direction = (max_curv_left >= max_curv_right ? 1 : -1);
        }

        // Step 2: Calculate distance_condition based on determined direction
        bool distance_condition = false;
        int first_small_radius_idx = -1;
        int last_small_radius_idx = -1;

        if (map_sharp_curve_direction != 0) {
          // Get indices for the determined direction
          const std::vector<int>& direction_indices =
              (map_sharp_curve_direction > 0) ? left_turn_indices : right_turn_indices;

          if (!direction_indices.empty()) {
            first_small_radius_idx = direction_indices.front();
            last_small_radius_idx = direction_indices.back();

            if (first_small_radius_idx >= 0 && last_small_radius_idx >= 0 &&
                first_small_radius_idx != last_small_radius_idx &&
                !s_vec.empty() && last_small_radius_idx < static_cast<int>(s_vec.size()) &&
                first_small_radius_idx < static_cast<int>(s_vec.size())) {
              double distance = std::fabs(s_vec[last_small_radius_idx] - s_vec[first_small_radius_idx]);
              distance_condition = (distance > kMapSharpCurveMinDistance);
            }
          }
        }

        // Step 3: Calculate count_small_radius for the determined direction and check condition
        int count_small_radius = (map_sharp_curve_direction > 0) ? count_left_turn : count_right_turn;

        // Hysteresis logic based on raw count
        if (map_sharp_curve_direction != 0) {
          if (last_condition_ramp_raw_count_) {
            // Exit when count < exit threshold (3)
            condition_ramp_raw_count = !(count_small_radius < kMapSharpCurveRawCountExit);
          } else {
            // Enter when count > enter threshold (4) and distance condition is satisfied
            condition_ramp_raw_count = (count_small_radius > kMapSharpCurveRawCountEnter) && distance_condition;
          }
        } else {
          // Direction not determined, do not satisfy raw-count condition
          condition_ramp_raw_count = false;
        }
      } else {
        // Keep last state if k_raw_signed is empty
        condition_ramp_raw_count = last_condition_ramp_raw_count_;
      }

      // Step 4: Calculate max curvature in the determined curve direction and check condition_ramp_curv
      double ramp_max_curv = 0.0;
      if (!k_smooth.empty() && map_sharp_curve_direction != 0) {
        // Only consider points with the same direction as determined curve direction
        for (size_t i = 0; i < k_smooth.size(); ++i) {
          double k_signed = k_smooth[i];
          int k_sign = (k_signed >= 0 ? 1 : -1);
          // Only consider points with the same direction
          if (k_sign * map_sharp_curve_direction > 0) {
            double k_abs = std::fabs(k_signed);
            if (k_abs > ramp_max_curv) {
              ramp_max_curv = k_abs;
            }
          }
        }
      }

      // Check condition_ramp_curv based on ramp_max_curv
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

      // Step 5: Calculate dist_to_ramp_max_curv in the determined curve direction
      if (!k_smooth.empty() && !s_vec.empty() &&
          k_smooth.size() == s_vec.size() &&
          map_sharp_curve_direction != 0) {
        double max_k = 0.0;
        double max_k_s = 0.0;  // Distance to max curvature point
        double first_small_radius_s = -1.0;  // First position with radius < 65m

        // Find max curvature and first small radius point in the determined direction
        for (size_t i = 0; i < k_smooth.size(); ++i) {
          double k_signed = k_smooth[i];
          int k_sign = (k_signed >= 0 ? 1 : -1);
          // Only consider points with the same direction
          if (k_sign * map_sharp_curve_direction > 0) {
            double k_abs = std::fabs(k_signed);
            // Update max curvature
            if (k_abs > max_k) {
              max_k = k_abs;
              max_k_s = s_vec[i];
            }
            // Check for first point with radius < 65m
            if (k_abs > 1e-6 && first_small_radius_s < 0) {
              double radius = 1.0 / k_abs;
              if (radius < kMapRadiusFirstEnterForDisCal) {
                first_small_radius_s = s_vec[i];
              }
            }
          }
        }

        // Prefer first position with radius < 65m, otherwise use max curvature position
        if (first_small_radius_s >= 0) {
          max_k_s = first_small_radius_s;
        }

        // Update dist_to_ramp_max_curv with calculated max_k_s
        dist_to_ramp_max_curv = max_k_s;

        JSON_DEBUG_VALUE("ramp_curv_max_k", max_k);
        if (max_k > 1e-6) {
          double min_radius = 1.0 / max_k;
          JSON_DEBUG_VALUE("ramp_curv_min_radius", min_radius);
        }
        JSON_DEBUG_VALUE("ramp_curv_dist_to_max_curv", dist_to_ramp_max_curv);
        JSON_DEBUG_VALUE("ramp_curv_direction",
                         static_cast<double>(map_sharp_curve_direction));  // 1 for left, -1 for right
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
  // Set current type and clear the other type
  speed_limit_output->SetSpeedLimitIntoMap(v_limit_in_turns, v_limit_type);

  // Always set the other type to default value to ensure proper clearing
  if (v_limit_type == SpeedLimitType::CURVATURE) {
    speed_limit_output->SetSpeedLimitIntoMap(kCurvatureDefaultLimitMps, SpeedLimitType::SHARP_CURVATURE);
  } else if (v_limit_type == SpeedLimitType::SHARP_CURVATURE) {
    speed_limit_output->SetSpeedLimitIntoMap(kCurvatureDefaultLimitMps, SpeedLimitType::CURVATURE);
  }
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
  bool is_search_cur_link = true;
  const double search_distance = 50.0;
  const double max_heading_diff = PI / 4;
  const double ego_heading_angle = ego_state->heading_angle();
  const auto current_segment = sdpro_map.GetNearestLinkWithHeading(
      current_point, search_distance, ego_heading_angle, max_heading_diff,
      nearest_s, nearest_l, is_search_cur_link);
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
        // Set current type and clear the other type
        speed_limit_output->SetSpeedLimitIntoMap(v_target_ramp,
                                             SpeedLimitType::MAP_ON_RAMP);
        // Always set the other type to default value to ensure proper clearing
        speed_limit_output->SetSpeedLimitIntoMap(kRampDefaultLimitMps, SpeedLimitType::MAP_NEAR_RAMP);
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
    // Set current type and clear the other type
    speed_limit_output->SetSpeedLimitIntoMap(v_target_ramp,
                                             SpeedLimitType::MAP_ON_RAMP);
    // Always set the other type to default value to ensure proper clearing
    speed_limit_output->SetSpeedLimitIntoMap(kRampDefaultLimitMps, SpeedLimitType::MAP_NEAR_RAMP);
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
  // Set current type and clear the other type
  speed_limit_output->SetSpeedLimitIntoMap(v_target_ramp,
                                           SpeedLimitType::MAP_NEAR_RAMP);
  // Always set the other type to default value to ensure proper clearing
  speed_limit_output->SetSpeedLimitIntoMap(kRampDefaultLimitMps, SpeedLimitType::MAP_ON_RAMP);
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
  bool is_search_cur_link = true;
  const double search_distance = 50.0;
  const double max_heading_diff = PI / 4;
  const double ego_heading_angle = ego_state->heading_angle();
  const auto current_segment = sdpro_map.GetNearestLinkWithHeading(
      current_point, search_distance, ego_heading_angle, max_heading_diff,
      nearest_s, nearest_l, is_search_cur_link);
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
                roundabout_info.second < kMapModeRoundaboutQuitDis &&
                road_radius_origin_ < kRoundaboutQuitCurvRadiusThr) {
              // in map mode, roundabout distance meets the condition
              function_need_inhibited = true;
            } else if (roundabout_info.first == nullptr) {
              auto roundabout_info_list = sdpro_map.GetRoundAboutList(
                  current_segment->id(), nearest_s, 300.0);
              if (roundabout_info_list.size() == 0) {
                roundabout_quit_flag_ = false;
                return;
              } else {
                auto closest_roundabout = roundabout_info_list[0];
                if (closest_roundabout.first != nullptr &&
                    closest_roundabout.second < kNoMapModeRoundaboutQuitDis &&
                    road_radius_origin_ < kRoundaboutQuitCurvRadiusThr) {
                  // not in map mode, closest roundabout distance meets the
                  // condition
                  function_need_inhibited = true;
                }
              }
            }
            if (function_need_inhibited) {
              roundabout_quit_flag_ = function_need_inhibited;
              roundabout_recover_counter_ = 0;
              speed_limit_output->set_function_inhibited_near_roundabout(
                  function_need_inhibited);
            } else {
              if (roundabout_quit_flag_ && roundabout_recover_counter_ < kRoundaboutQuitRecoverCounter) {
                speed_limit_output->set_function_inhibited_near_roundabout(
                  roundabout_quit_flag_);
                roundabout_recover_counter_++;
              } else {
                roundabout_quit_flag_ = false;
              }
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
    // Set current type and clear the other type
    speed_limit_output->SetSpeedLimitIntoMap(v_target_construction,
                                             SpeedLimitType::ON_CONSTRUCTION);
    // Always set the other type to default value to ensure proper clearing
    speed_limit_output->SetSpeedLimitIntoMap(kConstructionDefaultLimitMps, SpeedLimitType::NEAR_CONSTRUCTION);
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
  // Set current type and clear the other type
  speed_limit_output->SetSpeedLimitIntoMap(v_target_near_construction,
                                             SpeedLimitType::NEAR_CONSTRUCTION);
  // Always set the other type to default value to ensure proper clearing
  speed_limit_output->SetSpeedLimitIntoMap(kConstructionDefaultLimitMps, SpeedLimitType::ON_CONSTRUCTION);
  JSON_DEBUG_VALUE("v_target_construction", v_target_construction);
  JSON_DEBUG_VALUE("v_target_near_construction", v_target_near_construction);
  JSON_DEBUG_VALUE("dis_to_construction", dis_to_construction);
}

void SpeedLimitDecider::CalculateRoadBoundarySpeedLimit() {
  // ========== Part 1: Input Preparation ==========
  // Configuration input
  const auto &config = speed_limit_config_.road_boundary_speed_limit_config;

  // Environmental model and function info
  const auto &environmental_model = session_->environmental_model();
  const auto &function_info = environmental_model.function_info();

  // Ego state manager
  const auto ego_state_mgr = environmental_model.get_ego_state_manager();
  if (ego_state_mgr == nullptr) {
    return;
  }

  // Road boundary
  const auto virtual_lane_manager = environmental_model.get_virtual_lane_manager();
  if (virtual_lane_manager == nullptr) {
    return;
  }
  const auto &road_boundary = virtual_lane_manager->GetRoadboundary();

  // Planned path
  const auto &planned_kd_path = session_->planning_context().motion_planner_output().lateral_path_coord;
  if (planned_kd_path == nullptr) {
    return;
  }

  // Vehicle parameter
  const auto vehicle_config_instance = VehicleConfigurationContext::Instance();
  if (vehicle_config_instance == nullptr) {
    return;
  }
  const auto &vehicle_param = vehicle_config_instance->get_vehicle_param();

  // Function state machine info
  const auto &function_state_machine_info =
      environmental_model.get_local_view().function_state_machine_info;
  double v_cruise_fsm =
      function_state_machine_info.pilot_req.acc_curise_real_spd;

  // Route info
  const auto route_info = environmental_model.get_route_info();
  if (route_info == nullptr) {
    return;
  }
  const auto &route_info_output = route_info->get_route_info_output();

  // Lane change decider output
  const auto &lane_change_decider_output = session_->planning_context().lane_change_decider_output();
  const auto target_state = lane_change_decider_output.coarse_planning_info.target_state;

  // ========== Part 2: Feature Check ==========
  if (!config.enable_road_boundary_speed_limit) {
    road_boundary_manual_intervention_detected_ = false;
    road_boundary_manual_intervention_reset_count_ = 0;
    road_boundary_v_limit_set_ = false;
    road_boundary_left_confirmed_range_idx_ = -1;
    road_boundary_right_confirmed_range_idx_ = -1;
    curve_road_boundary_left_confirmed_gear_ = -1;
    curve_road_boundary_right_confirmed_gear_ = -1;
    s_curve_road_boundary_confirmed_ = false;
    return;
  }

  if (function_info.function_mode() == common::DrivingFunctionInfo::ACC &&
      !config.enable_road_boundary_speed_limit_in_acc_mode) {
    road_boundary_manual_intervention_detected_ = false;
    road_boundary_manual_intervention_reset_count_ = 0;
    road_boundary_v_limit_set_ = false;
    road_boundary_left_confirmed_range_idx_ = -1;
    road_boundary_right_confirmed_range_idx_ = -1;
    curve_road_boundary_left_confirmed_gear_ = -1;
    curve_road_boundary_right_confirmed_gear_ = -1;
    s_curve_road_boundary_confirmed_ = false;
    return;
  }

  if (road_boundary.empty() || planned_kd_path == nullptr) {
    road_boundary_manual_intervention_detected_ = false;
    road_boundary_manual_intervention_reset_count_ = 0;
    road_boundary_v_limit_set_ = false;
    road_boundary_left_confirmed_range_idx_ = -1;
    road_boundary_right_confirmed_range_idx_ = -1;
    curve_road_boundary_left_confirmed_gear_ = -1;
    curve_road_boundary_right_confirmed_gear_ = -1;
    s_curve_road_boundary_confirmed_ = false;
    return;
  }

  // ========== Part 3: Ego State ==========
  double v_ego = ego_state_mgr->ego_v();
  const auto init_point = ego_state_mgr->planning_init_point();
  double ego_s = 0.0, ego_l = 0.0;
  if (!planned_kd_path->XYToSL(init_point.x, init_point.y, &ego_s, &ego_l)) {
    return;
  }

  // ========== Part 4: Config Selection ==========
  bool is_on_expressway = route_info_output.is_ego_on_expressway;
  bool is_not_on_ramp = !route_info_output.is_on_ramp;
  double dis_to_ramp = route_info_output.dis_to_ramp;
  bool is_far_from_ramp = (dis_to_ramp > speed_limit_config_.map_sharp_curve_dis_to_ramp) ||
                           (dis_to_ramp >= speed_limit_config_.dis_near_ramp_zone);
  bool is_highway = route_info_output.is_ego_on_expressway_hmi;
  bool is_city_expressway = route_info_output.is_ego_on_city_expressway_hmi;
  bool use_highway_config = (is_on_expressway && is_not_on_ramp && is_far_from_ramp && is_highway);
  bool use_city_expressway_config = (is_on_expressway && is_not_on_ramp && is_far_from_ramp && is_city_expressway && !is_highway);

  int confirmation_frames_to_use = use_highway_config ? config.confirmation_frames_highway
                                                       : (use_city_expressway_config ? config.confirmation_frames_city_expressway
                                                                              : config.confirmation_frames);
  int min_points_in_range_to_use = use_highway_config ? config.min_points_in_range_highway
                                                       : (use_city_expressway_config ? config.min_points_in_range_city_expressway
                                                                             : config.min_points_in_range);
  const std::vector<double> &speed_limits_to_use = use_highway_config ? config.speed_limits_highway : config.speed_limits;

  // ========== Part 5: Manual Intervention Detection ==========
  double speed_increase = v_cruise_fsm - last_v_cruise_fsm_road_boundary_;
  last_v_cruise_fsm_road_boundary_ = v_cruise_fsm;

  if (road_boundary_v_limit_set_ && speed_increase > kCAManualInterventionSpeedDetected) {
    road_boundary_manual_intervention_detected_ = true;
    road_boundary_manual_intervention_reset_count_ = 0;
  }

  if (road_boundary_manual_intervention_detected_) {
    int reset_frames = use_highway_config ? config.manual_intervention_reset_frames_highway
                                           : (use_city_expressway_config ? config.manual_intervention_reset_frames_city_expressway
                                                                         : config.manual_intervention_reset_frames);
    road_boundary_manual_intervention_reset_count_++;
    if (road_boundary_manual_intervention_reset_count_ >= reset_frames) {
      road_boundary_manual_intervention_detected_ = false;
      road_boundary_manual_intervention_reset_count_ = 0;
    }
  }

  // ========== Part 6: Search Distance ==========
  double search_distance_min = v_ego * config.headway_min;
  double search_distance_max = std::min(v_ego * config.headway_max, config.max_search_distance);
  search_distance_max = std::max(search_distance_max, search_distance_min + kRoadBoundarySearchDistanceOffset);

  // ========== Part 7: Road Boundary Processing ==========
  std::vector<RoadBoundaryPoint> filtered_points;
  double collision_distance = -1.0;  // -1 means no collision

  for (const auto &boundary_segment : road_boundary) {
    // Store points in this segment that are within search distance range
    std::vector<std::pair<double, double>> segment_points;  // (s, l) pairs

    for (const auto &point_pair : boundary_segment) {
      Point2D global_point(point_pair.second.x, point_pair.second.y);
      double s = 0.0, l = 0.0;

      if (!planned_kd_path->XYToSL(global_point.x, global_point.y, &s, &l)) {
        continue;
      }

      // Filter points within search distance range
      double relative_s = s - ego_s;
      if (relative_s < search_distance_min || relative_s > search_distance_max) {
        continue;
      }

      segment_points.push_back({s, l});
      RoadBoundaryPoint rb_point;
      rb_point.cartesian_point = global_point;
      rb_point.s = s;
      rb_point.l = l;
      rb_point.is_left = (l > 0.0);  // Left: l>0, Right: l<0
      filtered_points.push_back(rb_point);
    }

    if (segment_points.size() >= 2) {
      std::sort(segment_points.begin(), segment_points.end(),
                [](const std::pair<double, double> &a, const std::pair<double, double> &b) {
                  return a.first < b.first;
                });

      for (size_t i = 0; i + 1 < segment_points.size(); ++i) {
        double l1 = segment_points[i].second;
        double l2 = segment_points[i + 1].second;
        if ((l1 > 0.0 && l2 < 0.0) || (l1 < 0.0 && l2 > 0.0)) {
          double dist_to_collision = segment_points[i + 1].first - ego_s;
          if (collision_distance < 0.0 || dist_to_collision < collision_distance) {
            collision_distance = dist_to_collision;
          }
          break;
        }
      }
    }
  }

  if (collision_distance >= 0.0) {
    collision_confirmation_frame_count_++;
    if (collision_confirmation_frame_count_ >= config.collision_confirmation_frames) {
      confirmed_collision_distance_ = collision_distance;
    }
  } else {
    collision_confirmation_frame_count_ = 0;
    confirmed_collision_distance_ = -1.0;
  }

  if (filtered_points.empty()) {
    return;
  }

  std::sort(filtered_points.begin(), filtered_points.end(),
            [](const RoadBoundaryPoint &a, const RoadBoundaryPoint &b) {
              return a.s < b.s;
            });

  // ========== Part 8: Vehicle Corner Points ==========
  double half_width = vehicle_param.width / 2.0;
  double front_edge_to_rear_axle = vehicle_param.front_edge_to_rear_axle;
  std::vector<planning_math::PathPoint> left_front_corner_points;
  std::vector<planning_math::PathPoint> right_front_corner_points;

  for (const auto &path_point : planned_kd_path->path_points()) {
    double relative_s = path_point.s() - ego_s;
    if (relative_s < search_distance_min || relative_s > search_distance_max) {
      continue;
    }

    // Calculate left/right front corner points
    double path_heading = path_point.theta();
    double normal_heading = path_heading + M_PI / 2.0;

    // Left front corner
    double left_corner_x = path_point.x() + half_width * std::cos(normal_heading);
    double left_corner_y = path_point.y() + half_width * std::sin(normal_heading);
    double left_front_x = left_corner_x + front_edge_to_rear_axle * std::cos(path_heading);
    double left_front_y = left_corner_y + front_edge_to_rear_axle * std::sin(path_heading);

    planning_math::PathPoint left_front_point;
    left_front_point.set_x(left_front_x);
    left_front_point.set_y(left_front_y);
    left_front_point.set_theta(path_heading);
    left_front_point.set_s(path_point.s());
    left_front_corner_points.push_back(left_front_point);

    // Right front corner
    double right_corner_x = path_point.x() - half_width * std::cos(normal_heading);
    double right_corner_y = path_point.y() - half_width * std::sin(normal_heading);
    double right_front_x = right_corner_x + front_edge_to_rear_axle * std::cos(path_heading);
    double right_front_y = right_corner_y + front_edge_to_rear_axle * std::sin(path_heading);

    planning_math::PathPoint right_front_point;
    right_front_point.set_x(right_front_x);
    right_front_point.set_y(right_front_y);
    right_front_point.set_theta(path_heading);
    right_front_point.set_s(path_point.s());
    right_front_corner_points.push_back(right_front_point);
  }

  // Check minimum path point size requirement for KDPath construction
  if (left_front_corner_points.size() < planning_math::KDPath::kKDPathMinPathPointSize ||
      right_front_corner_points.size() < planning_math::KDPath::kKDPathMinPathPointSize) {
    return;
  }

  auto left_front_corner_path = std::make_shared<planning_math::KDPath>(std::move(left_front_corner_points));
  auto right_front_corner_path = std::make_shared<planning_math::KDPath>(std::move(right_front_corner_points));

  // ========== Part 9: Lateral Distance Calculation ==========
  std::vector<RoadBoundaryPointWithVehicleDist> left_points_with_dist;
  std::vector<RoadBoundaryPointWithVehicleDist> right_points_with_dist;

  for (const auto &rb_point : filtered_points) {
    RoadBoundaryPointWithVehicleDist point_with_dist;
    point_with_dist.point = rb_point;

    double projected_s = 0.0, projected_l = 0.0;
    bool projection_success = false;

    if (rb_point.is_left) {
      projection_success = left_front_corner_path->XYToSL(
          rb_point.cartesian_point.x, rb_point.cartesian_point.y, &projected_s, &projected_l);
    } else {
      projection_success = right_front_corner_path->XYToSL(
          rb_point.cartesian_point.x, rb_point.cartesian_point.y, &projected_s, &projected_l);
    }

    if (projection_success) {
      point_with_dist.min_lateral_dist_to_vehicle = std::fabs(projected_l);
      if (rb_point.is_left) {
        left_points_with_dist.push_back(point_with_dist);
      } else {
        right_points_with_dist.push_back(point_with_dist);
      }
    }
  }

  // ========== Part 10: Range Grouping ==========
  auto group_points_by_range = [&config](const std::vector<RoadBoundaryPointWithVehicleDist> &points) {
    std::map<int, std::vector<RoadBoundaryPointWithVehicleDist>> range_groups;
    for (const auto &point : points) {
      double lateral_dist_m = point.min_lateral_dist_to_vehicle;
      for (size_t i = 0; i < config.range_mins.size() && i < config.range_maxs.size(); ++i) {
        if (lateral_dist_m < config.range_mins[i]) {
          break;
        }
        if (lateral_dist_m >= config.range_mins[i] && lateral_dist_m < config.range_maxs[i]) {
          range_groups[static_cast<int>(i)].push_back(point);
        }
      }
    }
    return range_groups;
  };

  auto left_range_groups = group_points_by_range(left_points_with_dist);
  auto right_range_groups = group_points_by_range(right_points_with_dist);

  // ========== Part 11: Trigger Detection ==========
  int current_left_range_idx = -1;
  int current_right_range_idx = -1;
  double trigger_distance_left = 0.0;
  double trigger_distance_right = 0.0;

  for (const auto &pair : left_range_groups) {
    if (static_cast<int>(pair.second.size()) >= min_points_in_range_to_use) {
      current_left_range_idx = pair.first;
      auto min_s_point = std::min_element(pair.second.begin(), pair.second.end(),
                                         [](const RoadBoundaryPointWithVehicleDist &a,
                                            const RoadBoundaryPointWithVehicleDist &b) {
                                           return a.point.s < b.point.s;
                                         });
      trigger_distance_left = min_s_point->point.s - ego_s;
      break;
    }
  }

  for (const auto &pair : right_range_groups) {
    if (static_cast<int>(pair.second.size()) >= min_points_in_range_to_use) {
      current_right_range_idx = pair.first;
      auto min_s_point = std::min_element(pair.second.begin(), pair.second.end(),
                                         [](const RoadBoundaryPointWithVehicleDist &a,
                                            const RoadBoundaryPointWithVehicleDist &b) {
                                           return a.point.s < b.point.s;
                                         });
      trigger_distance_right = min_s_point->point.s - ego_s;
      break;
    }
  }

  // ========== Part 12: Multi-Frame Confirmation ==========
  // Left side confirmation
  if (current_left_range_idx != -1) {
    if (road_boundary_left_pending_confirmation_range_idx_ == current_left_range_idx) {
      road_boundary_left_confirmation_frame_count_++;
      if (road_boundary_left_confirmation_frame_count_ >= confirmation_frames_to_use) {
        road_boundary_left_confirmed_range_idx_ = current_left_range_idx;
        road_boundary_left_pending_confirmation_range_idx_ = -1;
        road_boundary_left_confirmation_frame_count_ = 0;
      }
    } else {
      road_boundary_left_pending_confirmation_range_idx_ = current_left_range_idx;
      road_boundary_left_confirmation_frame_count_ = 1;
    }
  } else {
    road_boundary_left_pending_confirmation_range_idx_ = -1;
    road_boundary_left_confirmation_frame_count_ = 0;
  }

  // Right side confirmation
  if (current_right_range_idx != -1) {
    if (road_boundary_right_pending_confirmation_range_idx_ == current_right_range_idx) {
      road_boundary_right_confirmation_frame_count_++;
      if (road_boundary_right_confirmation_frame_count_ >= confirmation_frames_to_use) {
        road_boundary_right_confirmed_range_idx_ = current_right_range_idx;
        road_boundary_right_pending_confirmation_range_idx_ = -1;
        road_boundary_right_confirmation_frame_count_ = 0;
      }
    } else {
      road_boundary_right_pending_confirmation_range_idx_ = current_right_range_idx;
      road_boundary_right_confirmation_frame_count_ = 1;
    }
  } else {
    road_boundary_right_pending_confirmation_range_idx_ = -1;
    road_boundary_right_confirmation_frame_count_ = 0;
  }

  // ========== Part 13: Regular Speed Limit Calculation ==========
  bool is_left_triggered = (road_boundary_left_confirmed_range_idx_ != -1);
  bool is_right_triggered = (road_boundary_right_confirmed_range_idx_ != -1);


  double v_limit_left = kRoadBoundaryDefaultVLimit;
  double v_limit_right = kRoadBoundaryDefaultVLimit;
  if (is_left_triggered) {
    int idx = road_boundary_left_confirmed_range_idx_;
    v_limit_left = (idx < static_cast<int>(speed_limits_to_use.size()))
                   ? speed_limits_to_use[idx]
                   : speed_limits_to_use.back();
  }
  if (is_right_triggered) {
    int idx = road_boundary_right_confirmed_range_idx_;
    v_limit_right = (idx < static_cast<int>(speed_limits_to_use.size()))
                    ? speed_limits_to_use[idx]
                    : speed_limits_to_use.back();
  }

  double v_limit_regular = kRoadBoundaryDefaultVLimit;
  double trigger_distance_regular = 0.0;
  if (is_left_triggered && is_right_triggered) {
    bool is_left_last_gear = (road_boundary_left_confirmed_range_idx_ >= static_cast<int>(speed_limits_to_use.size()) - 1);
    bool is_right_last_gear = (road_boundary_right_confirmed_range_idx_ >= static_cast<int>(speed_limits_to_use.size()) - 1);
    bool any_side_last_gear = is_left_last_gear || is_right_last_gear;

    if (!any_side_last_gear) {
      if (road_boundary_left_confirmed_range_idx_ == road_boundary_right_confirmed_range_idx_) {
        int gear_idx = road_boundary_left_confirmed_range_idx_;
        if (gear_idx >= 0 && gear_idx < static_cast<int>(config.speed_limits_both_sides_same.size())) {
          v_limit_regular = config.speed_limits_both_sides_same[gear_idx];
        } else {
          v_limit_regular = config.speed_limits_both_sides_same.empty() ? kRoadBoundaryDefaultVLimit : config.speed_limits_both_sides_same.back();
        }
        trigger_distance_regular = std::min(trigger_distance_left, trigger_distance_right);
      } else {
        int lower_gear_idx = std::min(road_boundary_left_confirmed_range_idx_, road_boundary_right_confirmed_range_idx_);
        if (lower_gear_idx >= 0 && lower_gear_idx < static_cast<int>(config.speed_limits_both_sides_different.size())) {
          v_limit_regular = config.speed_limits_both_sides_different[lower_gear_idx];
        } else {
          v_limit_regular = config.speed_limits_both_sides_different.empty() ? kRoadBoundaryDefaultVLimit : config.speed_limits_both_sides_different.back();
        }
        trigger_distance_regular = (road_boundary_left_confirmed_range_idx_ < road_boundary_right_confirmed_range_idx_)
                                   ? trigger_distance_left
                                   : trigger_distance_right;
      }
    } else {
      v_limit_regular = std::min(v_limit_left, v_limit_right);
      trigger_distance_regular = std::min(trigger_distance_left, trigger_distance_right);
    }
  } else if (is_left_triggered) {
    v_limit_regular = v_limit_left;
    trigger_distance_regular = trigger_distance_left;
  } else if (is_right_triggered) {
    v_limit_regular = v_limit_right;
    trigger_distance_regular = trigger_distance_right;
  }

  if (is_on_expressway && is_not_on_ramp && is_far_from_ramp) {
    bool is_city_expressway = route_info_output.is_ego_on_city_expressway_hmi;
    if (is_highway && v_limit_regular < config.highway_min_speed) {
      v_limit_regular = config.highway_min_speed;
    } else if (is_city_expressway && v_limit_regular < config.expressway_min_speed) {
      v_limit_regular = config.expressway_min_speed;
    }
  }

  if (v_target_type_ == SpeedLimitType::AVOID_AGENT) {
    v_limit_regular = last_road_boundary_v_limit_;
    trigger_distance_regular = last_road_boundary_trigger_distance_;
  }

  // ========== Part 14: Cooldown ==========
  double current_v_limit_regular = v_limit_regular;
  double current_trigger_distance_regular = trigger_distance_regular;
  bool will_be_applied_regular = (current_v_limit_regular < v_target_);

  // Check if lane change is complete, interrupt cooldown if so
  bool is_lane_change_complete = (target_state == kLaneChangeComplete);

  if (is_lane_change_complete) {
    // Interrupt cooldown mechanism when lane change is complete
    road_boundary_cooldown_count_ = 0;
    last_road_boundary_v_limit_ = kRoadBoundaryDefaultLimitMps;
    last_road_boundary_trigger_distance_ = 0.0;
    road_boundary_v_limit_set_ = false;
  } else if (will_be_applied_regular) {
    bool is_first_time = (std::fabs(last_road_boundary_v_limit_ - kRoadBoundaryDefaultLimitMps) < 1e-3);
    if (is_first_time) {
      last_road_boundary_v_limit_ = current_v_limit_regular;
      last_road_boundary_trigger_distance_ = current_trigger_distance_regular;
      road_boundary_cooldown_count_ = config.cooldown_frames;
    } else if (road_boundary_cooldown_count_ > 0) {
      road_boundary_cooldown_count_--;
      v_limit_regular = last_road_boundary_v_limit_;
      trigger_distance_regular = last_road_boundary_trigger_distance_;
    } else {
      bool value_changed = (std::fabs(current_v_limit_regular - last_road_boundary_v_limit_) > kRoadBoundaryCooldownDeltaThreshold);
      if (value_changed) {
        last_road_boundary_v_limit_ = current_v_limit_regular;
        last_road_boundary_trigger_distance_ = current_trigger_distance_regular;
        road_boundary_cooldown_count_ = config.cooldown_frames;
      }
    }
  } else {
    if (road_boundary_cooldown_count_ > 0) {
      v_limit_regular = last_road_boundary_v_limit_;
      trigger_distance_regular = last_road_boundary_trigger_distance_;
      road_boundary_cooldown_count_--;
    } else {
      last_road_boundary_v_limit_ = kRoadBoundaryDefaultLimitMps;
      last_road_boundary_trigger_distance_ = 0.0;
      road_boundary_v_limit_set_ = false;
    }
  }

  double required_decel_regular = 0.0;
  if (trigger_distance_regular > kMinDistanceForDecel) {
    required_decel_regular = (v_limit_regular * v_limit_regular - v_ego * v_ego) / (2.0 * trigger_distance_regular);
  }
  SpeedLimitType road_boundary_type_regular = SpeedLimitType::ROAD_BOUNDARY;
  if (required_decel_regular < kRoadBoundarySharpDecelThreshold) {
    road_boundary_type_regular = SpeedLimitType::ROAD_BOUNDARY_SHARP_DECEL;
  }

  // ========== Part 15: Apply Regular Limit ==========
  double v_limit_regular_kph = std::round(v_limit_regular * 3.6 / 10.0) * 10;
  if (v_limit_regular_kph < v_cruise_limit_ && !road_boundary_manual_intervention_detected_) {
    v_cruise_limit_ = v_limit_regular_kph;
    road_boundary_v_limit_set_ = true;
  }
  if (v_limit_regular < v_target_ && !road_boundary_manual_intervention_detected_) {
    v_target_ = v_limit_regular;
    v_target_type_ = road_boundary_type_regular;
    road_boundary_v_limit_set_ = true;
  }

  // ========== Part 16: Regular Output ==========
  auto speed_limit_output = session_->mutable_planning_context()
                               ->mutable_speed_limit_decider_output();
  // Set current type and clear the other type
  speed_limit_output->SetSpeedLimitIntoMap(v_limit_regular, road_boundary_type_regular);

  // Always set the other type to default value to ensure proper clearing
  if (road_boundary_type_regular == SpeedLimitType::ROAD_BOUNDARY) {
    speed_limit_output->SetSpeedLimitIntoMap(kRoadBoundaryDefaultLimitMps, SpeedLimitType::ROAD_BOUNDARY_SHARP_DECEL);
  } else if (road_boundary_type_regular == SpeedLimitType::ROAD_BOUNDARY_SHARP_DECEL) {
    speed_limit_output->SetSpeedLimitIntoMap(kRoadBoundaryDefaultLimitMps, SpeedLimitType::ROAD_BOUNDARY);
  }

  JSON_DEBUG_VALUE("road_boundary_regular_v_limit", v_limit_regular);
  JSON_DEBUG_VALUE("road_boundary_regular_trigger_distance", trigger_distance_regular);
  JSON_DEBUG_VALUE("road_boundary_regular_required_decel", required_decel_regular);
  JSON_DEBUG_VALUE("road_boundary_regular_type", static_cast<int>(road_boundary_type_regular));
  JSON_DEBUG_VALUE("road_boundary_regular_is_left_triggered", is_left_triggered);
  JSON_DEBUG_VALUE("road_boundary_regular_is_right_triggered", is_right_triggered);
  JSON_DEBUG_VALUE("road_boundary_regular_left_range_idx", road_boundary_left_confirmed_range_idx_);
  JSON_DEBUG_VALUE("road_boundary_regular_right_range_idx", road_boundary_right_confirmed_range_idx_);
  JSON_DEBUG_VALUE("road_boundary_regular_cooldown_count", road_boundary_cooldown_count_);
  JSON_DEBUG_VALUE("road_boundary_regular_manual_intervention_detected", road_boundary_manual_intervention_detected_);
  JSON_DEBUG_VALUE("road_boundary_regular_manual_intervention_reset_count", road_boundary_manual_intervention_reset_count_);
  JSON_DEBUG_VALUE("road_boundary_regular_v_limit_set", road_boundary_v_limit_set_);

  // ========== Part 17: Strictest Limit Calculation ==========
  double curve_road_boundary_v_limit = kRoadBoundaryDefaultVLimit;
  double curve_road_boundary_trigger_distance = 0.0;
  bool curve_road_boundary_valid = false;

  if (config.enable_curve_road_boundary_speed_limit && road_radius_origin_ < config.curve_road_radius_threshold) {
    double curve_half_width = vehicle_param.width / 2.0;
    double curve_front_edge_to_rear_axle = vehicle_param.front_edge_to_rear_axle;

    if (!config.curve_front_edge_expansion_speed_breakpoints.empty() &&
        !config.curve_front_edge_expansion_values.empty() &&
        config.curve_front_edge_expansion_speed_breakpoints.size() == config.curve_front_edge_expansion_values.size()) {
      double expansion_length = interp(v_target_, config.curve_front_edge_expansion_speed_breakpoints,
                                        config.curve_front_edge_expansion_values);
      curve_front_edge_to_rear_axle += expansion_length;
    }
    std::vector<planning_math::PathPoint> curve_left_front_corner_points;
    std::vector<planning_math::PathPoint> curve_right_front_corner_points;

    for (const auto &path_point : planned_kd_path->path_points()) {
      double relative_s = path_point.s() - ego_s;
      if (relative_s < search_distance_min || relative_s > search_distance_max) {
        continue;
      }

      double kappa = path_point.kappa();
      double point_radius = 1.0 / std::max(std::fabs(kappa), 0.0001);

      // Calculate width expansion based on radius (kappa)
      double width_expansion = 0.0;
      if (!config.curve_radius_breakpoints.empty() && !config.width_expansion_values.empty()) {
        width_expansion = interp(point_radius, config.curve_radius_breakpoints, config.width_expansion_values);
      }

      // Calculate width expansion based on dkappa
      // kappa > 0: add to right side only; kappa < 0: add to left side only
      double dkappa_width_expansion_left = 0.0;
      double dkappa_width_expansion_right = 0.0;
      if (!config.curve_dkappa_breakpoints.empty() && !config.dkappa_width_expansion_values.empty()) {
        double dkappa = std::fabs(path_point.dkappa());
        double dkappa_expansion = interp(dkappa, config.curve_dkappa_breakpoints, config.dkappa_width_expansion_values);
        if (kappa > 0.0) {
          // kappa > 0: add to right side only
          dkappa_width_expansion_right = dkappa_expansion;
        } else if (kappa < 0.0) {
          // kappa < 0: add to left side only
          dkappa_width_expansion_left = dkappa_expansion;
        }
      }

      // Total width expansion for each side = radius-based + dkappa-based
      double expanded_half_width_left = curve_half_width + width_expansion + dkappa_width_expansion_left;
      double expanded_half_width_right = curve_half_width + width_expansion + dkappa_width_expansion_right;

      double path_heading = path_point.theta();
      double normal_heading = path_heading + M_PI / 2.0;
      double left_corner_x = path_point.x() + expanded_half_width_left * std::cos(normal_heading);
      double left_corner_y = path_point.y() + expanded_half_width_left * std::sin(normal_heading);
      double left_front_x = left_corner_x + curve_front_edge_to_rear_axle * std::cos(path_heading);
      double left_front_y = left_corner_y + curve_front_edge_to_rear_axle * std::sin(path_heading);

      planning_math::PathPoint left_front_point;
      left_front_point.set_x(left_front_x);
      left_front_point.set_y(left_front_y);
      left_front_point.set_theta(path_heading);
      left_front_point.set_s(path_point.s());
      curve_left_front_corner_points.push_back(left_front_point);

      double right_corner_x = path_point.x() - expanded_half_width_right * std::cos(normal_heading);
      double right_corner_y = path_point.y() - expanded_half_width_right * std::sin(normal_heading);
      double right_front_x = right_corner_x + curve_front_edge_to_rear_axle * std::cos(path_heading);
      double right_front_y = right_corner_y + curve_front_edge_to_rear_axle * std::sin(path_heading);

      planning_math::PathPoint right_front_point;
      right_front_point.set_x(right_front_x);
      right_front_point.set_y(right_front_y);
      right_front_point.set_theta(path_heading);
      right_front_point.set_s(path_point.s());
      curve_right_front_corner_points.push_back(right_front_point);
    }

    // Check minimum path point size requirement for KDPath construction
    if (curve_left_front_corner_points.size() < planning_math::KDPath::kKDPathMinPathPointSize ||
        curve_right_front_corner_points.size() < planning_math::KDPath::kKDPathMinPathPointSize) {
      curve_road_boundary_left_confirmed_gear_ = -1;
      curve_road_boundary_right_confirmed_gear_ = -1;
      curve_road_boundary_left_confirmation_frame_count_ = 0;
      curve_road_boundary_right_confirmation_frame_count_ = 0;
      curve_road_boundary_left_pending_confirmation_gear_ = -1;
      curve_road_boundary_right_pending_confirmation_gear_ = -1;
    } else {
      auto curve_left_front_corner_path = std::make_shared<planning_math::KDPath>(std::move(curve_left_front_corner_points));
      auto curve_right_front_corner_path = std::make_shared<planning_math::KDPath>(std::move(curve_right_front_corner_points));
      std::vector<RoadBoundaryPointWithVehicleDist> curve_left_points_with_dist;
      std::vector<RoadBoundaryPointWithVehicleDist> curve_right_points_with_dist;

      for (const auto &rb_point : filtered_points) {
        RoadBoundaryPointWithVehicleDist point_with_dist;
        point_with_dist.point = rb_point;

        double projected_s = 0.0, projected_l = 0.0;
        bool projection_success = false;

        if (rb_point.is_left) {
          projection_success = curve_left_front_corner_path->XYToSL(
              rb_point.cartesian_point.x, rb_point.cartesian_point.y, &projected_s, &projected_l);
        } else {
          projection_success = curve_right_front_corner_path->XYToSL(
              rb_point.cartesian_point.x, rb_point.cartesian_point.y, &projected_s, &projected_l);
        }

        if (projection_success) {
          point_with_dist.min_lateral_dist_to_vehicle = std::fabs(projected_l);
          if (rb_point.is_left) {
            curve_left_points_with_dist.push_back(point_with_dist);
          } else {
            curve_right_points_with_dist.push_back(point_with_dist);
          }
        }
      }

      auto group_curve_points_by_range = [&config](const std::vector<RoadBoundaryPointWithVehicleDist> &points) {
        std::map<int, std::vector<RoadBoundaryPointWithVehicleDist>> range_groups;
        for (const auto &point : points) {
          double lateral_dist_m = point.min_lateral_dist_to_vehicle;
          for (size_t i = 0; i < config.lateral_distance_ranges.size(); ++i) {
            if (lateral_dist_m >= 0.0 && lateral_dist_m <= config.lateral_distance_ranges[i]) {
              range_groups[static_cast<int>(i)].push_back(point);
            }
          }
        }
        return range_groups;
      };

      auto curve_left_range_groups = group_curve_points_by_range(curve_left_points_with_dist);
      auto curve_right_range_groups = group_curve_points_by_range(curve_right_points_with_dist);

      // Step 5: Detect Trigger Range (find first range that meets point count threshold)
      int current_left_gear = -1;
      int current_right_gear = -1;
      double trigger_distance_left_curve = 0.0;
      double trigger_distance_right_curve = 0.0;

      for (const auto &pair : curve_left_range_groups) {
        if (static_cast<int>(pair.second.size()) >= config.curve_min_points_in_range) {
          current_left_gear = pair.first;
          auto min_s_point = std::min_element(pair.second.begin(), pair.second.end(),
                                             [](const RoadBoundaryPointWithVehicleDist &a,
                                                const RoadBoundaryPointWithVehicleDist &b) {
                                               return a.point.s < b.point.s;
                                             });
          trigger_distance_left_curve = min_s_point->point.s - ego_s;
          break;
        }
      }

      for (const auto &pair : curve_right_range_groups) {
        if (static_cast<int>(pair.second.size()) >= config.curve_min_points_in_range) {
          current_right_gear = pair.first;
          auto min_s_point = std::min_element(pair.second.begin(), pair.second.end(),
                                             [](const RoadBoundaryPointWithVehicleDist &a,
                                                const RoadBoundaryPointWithVehicleDist &b) {
                                               return a.point.s < b.point.s;
                                             });
          trigger_distance_right_curve = min_s_point->point.s - ego_s;
          break;
        }
      }

      // Step 6: Multi-Frame Confirmation (similar to road boundary speed limit)

      if (current_left_gear != -1) {
        if (curve_road_boundary_left_pending_confirmation_gear_ <= current_left_gear) {
          curve_road_boundary_left_confirmation_frame_count_++;
          if (curve_road_boundary_left_confirmation_frame_count_ >= config.curve_confirmation_frames) {
            curve_road_boundary_left_confirmed_gear_ = current_left_gear;
            curve_road_boundary_left_pending_confirmation_gear_ = -1;
            curve_road_boundary_left_confirmation_frame_count_ = 0;
          }
        } else {
          curve_road_boundary_left_pending_confirmation_gear_ = current_left_gear;
          curve_road_boundary_left_confirmation_frame_count_ = 1;
        }
      } else {
        curve_road_boundary_left_pending_confirmation_gear_ = -1;
        curve_road_boundary_left_confirmation_frame_count_ = 0;
        curve_road_boundary_left_confirmed_gear_ = -1;
      }

      if (current_right_gear != -1) {
        if (curve_road_boundary_right_pending_confirmation_gear_ <= current_right_gear) {
          curve_road_boundary_right_confirmation_frame_count_++;
          if (curve_road_boundary_right_confirmation_frame_count_ >= config.curve_confirmation_frames) {
            curve_road_boundary_right_confirmed_gear_ = current_right_gear;
            curve_road_boundary_right_pending_confirmation_gear_ = -1;
            curve_road_boundary_right_confirmation_frame_count_ = 0;
          }
        } else {
          curve_road_boundary_right_pending_confirmation_gear_ = current_right_gear;
          curve_road_boundary_right_confirmation_frame_count_ = 1;
        }
      } else {
        curve_road_boundary_right_pending_confirmation_gear_ = -1;
        curve_road_boundary_right_confirmation_frame_count_ = 0;
        curve_road_boundary_right_confirmed_gear_ = -1;
      }

      // Step 7: Calculate curve road boundary speed limit based on minimum gear
      bool left_valid = (curve_road_boundary_left_confirmed_gear_ >= 0 &&
                         curve_road_boundary_left_confirmed_gear_ < config.curve_max_gear_threshold);
      bool right_valid = (curve_road_boundary_right_confirmed_gear_ >= 0 &&
                          curve_road_boundary_right_confirmed_gear_ < config.curve_max_gear_threshold);

      if (left_valid || right_valid) {
        int min_gear = -1;
        if (left_valid && right_valid) {
          min_gear = std::min(curve_road_boundary_left_confirmed_gear_, curve_road_boundary_right_confirmed_gear_);
        } else if (left_valid) {
          min_gear = curve_road_boundary_left_confirmed_gear_;
        } else {
          min_gear = curve_road_boundary_right_confirmed_gear_;
        }

        if (min_gear >= 0 && min_gear < static_cast<int>(config.curve_speed_limits.size())) {
          curve_road_boundary_v_limit = config.curve_speed_limits[min_gear];
          curve_road_boundary_valid = true;

          // Use the trigger distance from confirmed gear (already calculated in Step 5)
          if (left_valid && right_valid) {
            // Both sides valid: use the one corresponding to min_gear
            if (curve_road_boundary_left_confirmed_gear_ == min_gear &&
                curve_road_boundary_right_confirmed_gear_ == min_gear) {
              curve_road_boundary_trigger_distance = std::min(trigger_distance_left_curve, trigger_distance_right_curve);
            } else if (curve_road_boundary_left_confirmed_gear_ == min_gear) {
              curve_road_boundary_trigger_distance = trigger_distance_left_curve;
            } else {
              curve_road_boundary_trigger_distance = trigger_distance_right_curve;
            }
          } else if (left_valid) {
            curve_road_boundary_trigger_distance = trigger_distance_left_curve;
          } else {
            curve_road_boundary_trigger_distance = trigger_distance_right_curve;
          }
        }
      }

      JSON_DEBUG_VALUE("curve_road_boundary_enabled", true);
      JSON_DEBUG_VALUE("curve_road_boundary_road_radius", road_radius_origin_);
      JSON_DEBUG_VALUE("curve_road_boundary_left_gear", curve_road_boundary_left_confirmed_gear_);
      JSON_DEBUG_VALUE("curve_road_boundary_right_gear", curve_road_boundary_right_confirmed_gear_);
      JSON_DEBUG_VALUE("curve_road_boundary_v_limit", curve_road_boundary_v_limit);
      JSON_DEBUG_VALUE("curve_road_boundary_trigger_distance", curve_road_boundary_trigger_distance);
    }
  } else {
    curve_road_boundary_left_confirmed_gear_ = -1;
    curve_road_boundary_right_confirmed_gear_ = -1;
    curve_road_boundary_left_confirmation_frame_count_ = 0;
    curve_road_boundary_right_confirmation_frame_count_ = 0;
    curve_road_boundary_left_pending_confirmation_gear_ = -1;
    curve_road_boundary_right_pending_confirmation_gear_ = -1;
    JSON_DEBUG_VALUE("curve_road_boundary_enabled", false);
    JSON_DEBUG_VALUE("curve_road_boundary_road_radius", road_radius_origin_);
    JSON_DEBUG_VALUE("curve_road_boundary_left_gear", curve_road_boundary_left_confirmed_gear_);
    JSON_DEBUG_VALUE("curve_road_boundary_right_gear", curve_road_boundary_right_confirmed_gear_);
    JSON_DEBUG_VALUE("curve_road_boundary_v_limit", curve_road_boundary_v_limit);
    JSON_DEBUG_VALUE("curve_road_boundary_trigger_distance", curve_road_boundary_trigger_distance);
  }

  double collision_v_limit = kRoadBoundaryDefaultVLimit;
  double collision_trigger_distance = 0.0;
  bool collision_valid = false;
  if (config.enable_road_boundary_collision_speed_limit && confirmed_collision_distance_ >= 0.0) {
    collision_v_limit = config.collision_speed_limit;
    collision_trigger_distance = confirmed_collision_distance_;
    collision_valid = true;
  }
  JSON_DEBUG_VALUE("road_boundary_collision_v_limit", collision_v_limit);
  JSON_DEBUG_VALUE("road_boundary_collision_trigger_distance", collision_trigger_distance);
  JSON_DEBUG_VALUE("road_boundary_collision_valid", collision_valid);
  JSON_DEBUG_VALUE("road_boundary_collision_distance", confirmed_collision_distance_);
  double lateral_acc_v_limit = kRoadBoundaryDefaultVLimit;
  double lateral_acc_trigger_distance = 0.0;
  bool lateral_acc_valid = false;

  if (config.enable_lateral_acceleration_limit) {
    // Step 1: Calculate minimum speed from v_limit_regular, v_target_, and v_ego
    double min_speed = std::min({v_limit_regular, v_target_, v_ego});

    // Step 2: Check lateral acceleration in [search_distance_min, search_distance_max]
    int exceed_count = 0;
    double first_exceed_distance = -1.0;  // First point where lat_acc exceeds threshold
    double first_exceed_kappa = 0.0;  // Kappa of first point where lat_acc exceeds threshold

    for (const auto &path_point : planned_kd_path->path_points()) {
      double relative_s = path_point.s() - ego_s;
      if (relative_s < search_distance_min || relative_s > search_distance_max) {
        continue;
      }

      // Calculate lateral acceleration: lat_acc = v^2 * kappa
      double kappa = path_point.kappa();
      double lat_acc = min_speed * min_speed * std::fabs(kappa);

      if (lat_acc > config.lateral_acceleration_threshold) {
        exceed_count++;
        if (first_exceed_distance < 0.0) {
          first_exceed_distance = relative_s;
          first_exceed_kappa = std::fabs(kappa);
        }
      }
    }

    // Step 3: Check if exceed_count >= min_points
    bool current_frame_exceeds = (exceed_count >= config.lateral_acceleration_min_points);

    // Step 4: Multi-frame confirmation
    if (current_frame_exceeds) {
      if (lateral_acceleration_limit_pending_) {
        lateral_acceleration_limit_confirmation_frame_count_++;
        if (lateral_acceleration_limit_confirmation_frame_count_ >= config.lateral_acceleration_confirmation_frames) {
          // Confirmed: calculate speed limit based on desired lateral acceleration
          // Formula: lat_acc = v^2 * kappa, so v = sqrt(lat_acc_desired / kappa)
          lateral_acceleration_limit_confirmed_ = true;
          if (first_exceed_kappa > 1e-6) {  // Avoid division by zero
            double calculated_v_limit = std::sqrt(config.lateral_acceleration_desired / first_exceed_kappa);
            lateral_acceleration_limit_v_limit_ = std::max(0.0, calculated_v_limit);
          } else {
            // Fallback to original method if kappa is too small
            lateral_acceleration_limit_v_limit_ = std::max(0.0, min_speed - config.lateral_acceleration_speed_reduction);
          }
          lateral_acceleration_limit_trigger_distance_ = first_exceed_distance;
          lateral_acceleration_limit_pending_ = false;
          lateral_acceleration_limit_confirmation_frame_count_ = 0;
        }
      } else {
        // Start pending confirmation
        lateral_acceleration_limit_pending_ = true;
        lateral_acceleration_limit_confirmation_frame_count_ = 1;
      }
      // Reset reset counter when condition is met
      lateral_acceleration_limit_reset_frame_count_ = 0;
    } else {
      // Condition not met: reset pending state or count reset frames
      if (lateral_acceleration_limit_confirmed_) {
        // Already confirmed: need multiple frames to reset
        lateral_acceleration_limit_reset_frame_count_++;
        if (lateral_acceleration_limit_reset_frame_count_ >= config.lateral_acceleration_confirmation_frames) {
          // Reset confirmed state after multiple frames
          lateral_acceleration_limit_confirmed_ = false;
          lateral_acceleration_limit_reset_frame_count_ = 0;
        }
      } else {
        // Not confirmed yet: reset pending state immediately
        lateral_acceleration_limit_pending_ = false;
        lateral_acceleration_limit_confirmation_frame_count_ = 0;
      }
    }

    // Step 5: If confirmed, set lateral acceleration limit values
    if (lateral_acceleration_limit_confirmed_) {
      lateral_acc_v_limit = lateral_acceleration_limit_v_limit_;
      lateral_acc_trigger_distance = lateral_acceleration_limit_trigger_distance_;
      lateral_acc_valid = true;

      JSON_DEBUG_VALUE("lateral_acceleration_limit_triggered", true);
      JSON_DEBUG_VALUE("lateral_acceleration_limit_v_limit", lateral_acc_v_limit);
      JSON_DEBUG_VALUE("lateral_acceleration_limit_trigger_distance", lateral_acc_trigger_distance);
      JSON_DEBUG_VALUE("lateral_acceleration_exceed_count", exceed_count);
    } else {
      JSON_DEBUG_VALUE("lateral_acceleration_limit_triggered", false);
      JSON_DEBUG_VALUE("lateral_acceleration_limit_v_limit", lateral_acc_v_limit);
      JSON_DEBUG_VALUE("lateral_acceleration_limit_trigger_distance", lateral_acc_trigger_distance);
      JSON_DEBUG_VALUE("lateral_acceleration_exceed_count", exceed_count);
    }
  }

  // ========== Part 19: S-Curve Road Boundary Speed Limit ==========
  double s_curve_v_limit = kRoadBoundaryDefaultVLimit;
  double s_curve_trigger_distance = 0.0;
  bool s_curve_valid = false;

  if (config.enable_s_curve_road_boundary_speed_limit) {
    // Step 1: Detect S-curve in [search_distance_min, search_distance_max]
    bool is_s_curve = false;
    double min_radius_in_range = 10000.0;
    bool kappa_sign_changed = false;
    double max_dkappa_at_sign_change = 0.0;
    int first_curve_direction = 0;  // 1 for left (kappa < 0), -1 for right (kappa > 0), 0 for unknown
    double first_curve_min_radius_distance = 0.0;  // Distance to first curve's minimum radius point
    double first_curve_min_radius = 10000.0;  // Minimum radius in first curve

    double prev_kappa = 0.0;
    bool first_point = true;
    bool sign_changed_occurred = false;
    std::vector<CurvInfo> kd_path_curv_info_vec;  // Collect kappa data for IsSSharpBend

    for (const auto &path_point : planned_kd_path->path_points()) {
      double relative_s = path_point.s() - ego_s;
      if (relative_s < search_distance_min || relative_s > search_distance_max) {
        continue;
      }

      double kappa = path_point.kappa();
      double radius = 1.0 / std::max(std::fabs(kappa), 0.0001);

      // Collect kappa data for IsSSharpBend
      CurvInfo curv_info;
      curv_info.s = relative_s;
      curv_info.curv = std::fabs(kappa);
      curv_info.curv_sign = (kappa > 0.0) ? 1 : ((kappa < 0.0) ? -1 : 0);
      kd_path_curv_info_vec.push_back(curv_info);

      // Update minimum radius in entire range
      if (radius < min_radius_in_range) {
        min_radius_in_range = radius;
      }

      // Check for kappa sign change
      if (!first_point) {
        if ((prev_kappa > 0.0 && kappa < 0.0) || (prev_kappa < 0.0 && kappa > 0.0)) {
          kappa_sign_changed = true;
          sign_changed_occurred = true;
          double dkappa = std::fabs(path_point.dkappa());
          if (dkappa > max_dkappa_at_sign_change) {
            max_dkappa_at_sign_change = dkappa;
          }
        }
      }

      // Determine first curve direction (use first curve with radius < kSSharpBendRadius)
      if (first_curve_direction == 0 && radius < kSSharpBendRadius && std::fabs(kappa) > 1e-6) {
        if (kappa > 0.0) {
          first_curve_direction = 1;  // Left curve
        } else {
          first_curve_direction = -1;  // Right curve
        }
        first_curve_min_radius = radius;
        first_curve_min_radius_distance = relative_s;
      }

      // Update first curve's minimum radius distance (only before sign change)
      if (first_curve_direction != 0 && !sign_changed_occurred) {
        // Check if we're still in the first curve (same sign as first curve)
        bool in_first_curve = ((first_curve_direction == 1 && kappa < 0.0) ||
                               (first_curve_direction == -1 && kappa > 0.0));
        if (in_first_curve) {
          if (radius < first_curve_min_radius) {
            first_curve_min_radius = radius;
            first_curve_min_radius_distance = relative_s;
          }
        }
      }

      prev_kappa = kappa;
      first_point = false;
    }

    // Step 2: Check S-curve conditions
    // Condition 1: Minimum radius < threshold
    bool condition1 = (min_radius_in_range < config.s_curve_min_radius_threshold);
    // Condition 2: Kappa sign changed
    bool condition2 = kappa_sign_changed;
    // Condition 3: dkappa at sign change > threshold
    bool condition3 = (max_dkappa_at_sign_change > config.s_curve_dkappa_threshold);
    // Condition 4: Not in lane change state
    bool is_lane_change_state = (target_state == kLaneChangeExecution ||
                                  target_state == kLaneChangeComplete ||
                                  target_state == kLaneChangeCancel ||
                                  target_state == kLaneChangeHold);
    bool condition4 = !is_lane_change_state;
    // Condition 5: IsSSharpBend check using kd_path kappa
    bool condition5 = IsSSharpBend(kd_path_curv_info_vec);

    bool current_frame_is_s_curve = condition1 && condition2 && condition3 && condition4 && condition5;

    // Step 3: Multi-frame confirmation
    if (current_frame_is_s_curve) {
      if (s_curve_road_boundary_pending_) {
        s_curve_road_boundary_confirmation_frame_count_++;
        if (s_curve_road_boundary_confirmation_frame_count_ >= config.s_curve_confirmation_frames) {
          // Confirmed: calculate speed limit based on first curve direction and gear
          s_curve_road_boundary_confirmed_ = true;

          int gear = -1;
          if (first_curve_direction == 1) {
            // Left curve: check right side gear
            gear = curve_road_boundary_right_confirmed_gear_;
          } else if (first_curve_direction == -1) {
            // Right curve: check left side gear
            gear = curve_road_boundary_left_confirmed_gear_;
          }

          // Get speed limit based on gear (0-6)
          if (gear >= 0 && gear < static_cast<int>(config.s_curve_speed_limits.size())) {
            s_curve_road_boundary_v_limit_ = config.s_curve_speed_limits[gear];
          } else if (gear >= static_cast<int>(config.s_curve_speed_limits.size())) {
            s_curve_road_boundary_v_limit_ = kRoadBoundaryDefaultVLimit;
          } else {
            // Gear not available, use default (first value)
            s_curve_road_boundary_v_limit_ = kRoadBoundaryDefaultVLimit;
          }
          s_curve_road_boundary_pending_ = false;
          s_curve_road_boundary_confirmation_frame_count_ = 0;
        }
      } else {
        // Start pending confirmation
        s_curve_road_boundary_pending_ = true;
        s_curve_road_boundary_confirmation_frame_count_ = 1;
      }
    } else {
      // Condition not met: reset pending state
      s_curve_road_boundary_pending_ = false;
      s_curve_road_boundary_confirmation_frame_count_ = 0;
      if (s_curve_road_boundary_confirmed_) {
        // Reset confirmed state immediately when condition not met
        s_curve_road_boundary_confirmed_ = false;
      }
    }

    // Step 4: If confirmed, set S-curve limit values
    if (s_curve_road_boundary_confirmed_) {
      s_curve_v_limit = s_curve_road_boundary_v_limit_;
      s_curve_trigger_distance = first_curve_min_radius_distance;
      s_curve_valid = true;

      JSON_DEBUG_VALUE("s_curve_road_boundary_triggered", true);
      JSON_DEBUG_VALUE("s_curve_road_boundary_v_limit", s_curve_v_limit);
      JSON_DEBUG_VALUE("s_curve_road_boundary_trigger_distance", s_curve_trigger_distance);
      JSON_DEBUG_VALUE("s_curve_min_radius", min_radius_in_range);
      JSON_DEBUG_VALUE("s_curve_first_direction", first_curve_direction);
    } else {
      JSON_DEBUG_VALUE("s_curve_road_boundary_triggered", false);
      JSON_DEBUG_VALUE("s_curve_road_boundary_v_limit", s_curve_v_limit);
      JSON_DEBUG_VALUE("s_curve_road_boundary_trigger_distance", s_curve_trigger_distance);
      JSON_DEBUG_VALUE("s_curve_min_radius", min_radius_in_range);
      JSON_DEBUG_VALUE("s_curve_first_direction", first_curve_direction);
    }
  }

  // Select the strictest limit from confirmed collision, curve road boundary, and lateral acceleration
  // Priority: collision > curve > lateral acc
  double v_limit_road_boundary_strictest = kRoadBoundaryDefaultVLimit;
  double trigger_distance_road_boundary_strictest = 0.0;
  SpeedLimitType road_boundary_type_strictest = SpeedLimitType::ROAD_BOUNDARY;
  bool road_boundary_strictest_valid = false;

  if (collision_valid) {
    v_limit_road_boundary_strictest = collision_v_limit;
    trigger_distance_road_boundary_strictest = collision_trigger_distance;
    road_boundary_type_strictest = SpeedLimitType::ROAD_BOUNDARY;
    road_boundary_strictest_valid = true;
  }  else if (curve_road_boundary_valid) {
    v_limit_road_boundary_strictest = curve_road_boundary_v_limit;
    trigger_distance_road_boundary_strictest = curve_road_boundary_trigger_distance;
    road_boundary_type_strictest = SpeedLimitType::ROAD_BOUNDARY;
    road_boundary_strictest_valid = true;
  } else if (lateral_acc_valid) {
    v_limit_road_boundary_strictest = lateral_acc_v_limit;
    trigger_distance_road_boundary_strictest = lateral_acc_trigger_distance;
    road_boundary_type_strictest = SpeedLimitType::ROAD_BOUNDARY;
    road_boundary_strictest_valid = true;
  } else if (s_curve_valid) {
    v_limit_road_boundary_strictest = s_curve_v_limit;
    trigger_distance_road_boundary_strictest = s_curve_trigger_distance;
    road_boundary_type_strictest = SpeedLimitType::ROAD_BOUNDARY;
    road_boundary_strictest_valid = true;
  }

  // Cooldown mechanism for strictest road boundary limit
  double current_v_limit_strictest = v_limit_road_boundary_strictest;
  double current_trigger_distance_strictest = trigger_distance_road_boundary_strictest;
  bool will_be_applied_strictest = (road_boundary_strictest_valid && current_v_limit_strictest < v_target_);

  // Use curve_road_boundary_cooldown_frames for all strictest road boundary limits
  int cooldown_frames_strictest = config.curve_road_boundary_cooldown_frames;

  if (will_be_applied_strictest) {
    bool is_first_time_strictest = (std::fabs(last_road_boundary_strictest_v_limit_ - kRoadBoundaryDefaultLimitMps) < 1e-3);
    if (is_first_time_strictest) {
      last_road_boundary_strictest_v_limit_ = current_v_limit_strictest;
      last_road_boundary_strictest_trigger_distance_ = current_trigger_distance_strictest;
      road_boundary_strictest_cooldown_count_ = cooldown_frames_strictest;
    } else if (road_boundary_strictest_cooldown_count_ > 0) {
      road_boundary_strictest_cooldown_count_--;
      v_limit_road_boundary_strictest = last_road_boundary_strictest_v_limit_;
      trigger_distance_road_boundary_strictest = last_road_boundary_strictest_trigger_distance_;
    } else {
      bool value_changed_strictest = (std::fabs(current_v_limit_strictest - last_road_boundary_strictest_v_limit_) > kRoadBoundaryCooldownDeltaThreshold);
      if (value_changed_strictest) {
        last_road_boundary_strictest_v_limit_ = current_v_limit_strictest;
        last_road_boundary_strictest_trigger_distance_ = current_trigger_distance_strictest;
        road_boundary_strictest_cooldown_count_ = cooldown_frames_strictest;
      }
    }
  } else {
    if (road_boundary_strictest_cooldown_count_ > 0) {
      v_limit_road_boundary_strictest = last_road_boundary_strictest_v_limit_;
      trigger_distance_road_boundary_strictest = last_road_boundary_strictest_trigger_distance_;
      road_boundary_strictest_cooldown_count_--;
      road_boundary_strictest_valid = true;  // Keep valid during cooldown
    } else {
      last_road_boundary_strictest_v_limit_ = kRoadBoundaryDefaultLimitMps;
      last_road_boundary_strictest_trigger_distance_ = 0.0;
    }
  }

  // Calculate required deceleration for strictest road boundary limit
  double required_decel_strictest = 0.0;
  if (trigger_distance_road_boundary_strictest > kMinDistanceForDecel) {
    required_decel_strictest = (v_limit_road_boundary_strictest * v_limit_road_boundary_strictest - v_ego * v_ego) / (2.0 * trigger_distance_road_boundary_strictest);
  }
  if (required_decel_strictest < kRoadBoundarySharpDecelThreshold) {
    road_boundary_type_strictest = SpeedLimitType::ROAD_BOUNDARY_SHARP_DECEL;
  }

  // Apply strictest road boundary limit to v_target_ only (not v_cruise_limit_)
  bool road_boundary_strictest_applied = false;
  if (road_boundary_strictest_valid) {
    if (v_limit_road_boundary_strictest < v_target_) {
      v_target_ = v_limit_road_boundary_strictest;
      v_target_type_ = road_boundary_type_strictest;
      road_boundary_v_limit_set_ = true;
      road_boundary_strictest_applied = true;
    }
  }

  // ========== Part 18: Output and Debug for Strictest Road Boundary ==========
  // Set strictest road boundary limit into map with separate clearing mechanism for ROAD_BOUNDARY and ROAD_BOUNDARY_SHARP_DECEL
  auto speed_limit_output_strictest = session_->mutable_planning_context()
                                      ->mutable_speed_limit_decider_output();

  if (road_boundary_strictest_valid) {
    // Set current type and clear the other type
    speed_limit_output_strictest->SetSpeedLimitIntoMap(v_limit_road_boundary_strictest, road_boundary_type_strictest);

    // Always set the other type to default value to ensure proper clearing
    if (road_boundary_type_strictest == SpeedLimitType::ROAD_BOUNDARY) {
      speed_limit_output_strictest->SetSpeedLimitIntoMap(kRoadBoundaryDefaultLimitMps, SpeedLimitType::ROAD_BOUNDARY_SHARP_DECEL);
    } else if (road_boundary_type_strictest == SpeedLimitType::ROAD_BOUNDARY_SHARP_DECEL) {
      speed_limit_output_strictest->SetSpeedLimitIntoMap(kRoadBoundaryDefaultLimitMps, SpeedLimitType::ROAD_BOUNDARY);
    }
  } else {
    // Clear both types when invalid
    speed_limit_output_strictest->SetSpeedLimitIntoMap(kRoadBoundaryDefaultLimitMps, SpeedLimitType::ROAD_BOUNDARY);
    speed_limit_output_strictest->SetSpeedLimitIntoMap(kRoadBoundaryDefaultLimitMps, SpeedLimitType::ROAD_BOUNDARY_SHARP_DECEL);
  }

  // Debug output for strictest road boundary limit
  JSON_DEBUG_VALUE("road_boundary_strictest_v_limit", v_limit_road_boundary_strictest);
  JSON_DEBUG_VALUE("road_boundary_strictest_trigger_distance", trigger_distance_road_boundary_strictest);
  JSON_DEBUG_VALUE("road_boundary_strictest_required_decel", required_decel_strictest);
  JSON_DEBUG_VALUE("road_boundary_strictest_type", static_cast<int>(road_boundary_type_strictest));
  JSON_DEBUG_VALUE("road_boundary_strictest_valid", road_boundary_strictest_valid);
  JSON_DEBUG_VALUE("road_boundary_strictest_applied", road_boundary_strictest_applied);
  JSON_DEBUG_VALUE("road_boundary_strictest_cooldown_count", road_boundary_strictest_cooldown_count_);
  JSON_DEBUG_VALUE("s_curve_road_boundary_valid", s_curve_valid)
}

}  // namespace planning





