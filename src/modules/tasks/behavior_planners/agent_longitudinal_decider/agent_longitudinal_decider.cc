#include "agent_longitudinal_decider.h"

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

#include "agent/agent.h"
#include "common_platform_type_soc.h"
#include "config/basic_type.h"
#include "debug_info_log.h"
#include "environmental_model.h"
#include "interface/src/c/common_c.h"
#include "log.h"
#include "planning_context.h"
#include "virtual_lane.h"

namespace planning {
using namespace planning_math;

namespace {

constexpr double kEpsilon = 1.0e-4;
// constexpr double kMaxHeadingDiff = 1.04;  // 60 degree
// constexpr double kMinLowSpeedTrajLength = 3.0;
constexpr double kIgnoreSpeedDiffThd = 2.5;
constexpr double kAgentFrontEdgeSDiffThd = -0.2;
constexpr double kHalf = 0.5;
constexpr double kKphToMps = 0.2778;
constexpr double kMpsToKph = 3.6;
constexpr double kLaneWidth = 3.75;
constexpr double kLargeAgentLengthM = 8.0;
constexpr double kLargeSpeedDiff = 3.0;

// Param for cut-in check
constexpr double kFtpCutInDeactivationSpeedMps = 100.0 * kKphToMps;
constexpr double kCutInLateralTtcThresholdS = 2.0;
constexpr double kLargeCutInLateralTtcThresholdS = 3.0;
constexpr double kCutInHeadwayRangeS = 3.6;
constexpr double kMinCutInDistanceM = 30;
constexpr double kMaxCutInDistanceM = 60;
constexpr double kLargeYawThresholdRad = 2.0 / 57.3;
constexpr double kLargeYawLateralDistanceBufferM = 0.5;
constexpr double kLargeYawLateralSpeedThresholdMps = 0.17;
constexpr double kCutInLateralSpeedThresholdMps = 0.1;
constexpr double kCutInSpeedLimitMps = 0.5;
constexpr double kEgoStopSpeedThresholdMps = 0.5;
constexpr double kCurrentKappaThreshold = 0.0025;
constexpr double kCurrentKappaThresholdForLargeAgent = 0.003;
constexpr int32_t kRuleBasedCutInCount = 3;
constexpr int32_t kPredBasedCutInCount = 3;
constexpr int32_t kDefaultCutInCount = 4;

// param for filter ultradistant obstacle
// constexpr double kFilterUltraEgoSpeedLowThd = 100 / 3.6;
// constexpr double kFilterUltraEgoSpeedHighThd = 110 / 3.6;
// constexpr double kFilterUltraDistanceLowThd = 150.0;
// constexpr double kFilterUltraDistanceMiddleThd = 165.0;
constexpr double kFilterUltraDistanceHighThd = 300.0;

// param for reverse agent filter
constexpr double kReverseHeadingThreshold = 2.09;
constexpr double kLowSpeedThreshold = 3.0;
constexpr double kMinFilterDistance = 30.0;
constexpr double kLowSpeedLateralIgnoreDist = 10.0;
// constexpr double kLongitudalTtc = 5.0; // too radicalness
constexpr double kLongitudalTtc = 6.0;
constexpr double kDefaultLaneHalfWidth = 1.875;
constexpr double kCrossLaneThreshold = 0.1;
constexpr double kHightSpeedLateralIgnoreDist = 15.0;
constexpr double kLanesDistanceThr = 5.0;
constexpr double kIntersectionFactor = 0.3;
constexpr double kconsideredLonDistanceInCurve = 80.0;
constexpr double kDistanceCurvature = 30.0;
constexpr double kTimeCurvature = 2.0;
constexpr double kLateralSafeBuffer = 0.3;

void CalculateAgentSLBoundary(const std::shared_ptr<KDPath>& planned_path,
                              const planning_math::Box2d& agent_box,
                              double* const ptr_min_s, double* const ptr_max_s,
                              double* const ptr_min_l,
                              double* const ptr_max_l) {
  if (nullptr == ptr_min_s || nullptr == ptr_max_s || nullptr == ptr_min_l ||
      nullptr == ptr_max_l) {
    return;
  }
  const auto& all_corners = agent_box.GetAllCorners();
  for (const auto& corner : all_corners) {
    double agent_s = 0.0;
    double agent_l = 0.0;
    planned_path->XYToSL(corner.x(), corner.y(), &agent_s, &agent_l);
    *ptr_min_s = std::fmin(*ptr_min_s, agent_s);
    *ptr_max_s = std::fmax(*ptr_max_s, agent_s);
    *ptr_min_l = std::fmin(*ptr_min_l, agent_l);
    *ptr_max_l = std::fmax(*ptr_max_l, agent_l);
  }
}

void CalculateAgentSLBoundary(const std::shared_ptr<KDPath>& planned_path,
                              const agent::Agent& agent,
                              double* const ptr_min_s, double* const ptr_max_s,
                              double* const ptr_min_l,
                              double* const ptr_max_l) {
  const auto& agent_box = agent.box();
  CalculateAgentSLBoundary(planned_path, agent_box, ptr_min_s, ptr_max_s,
                           ptr_min_l, ptr_max_l);
}
}  // namespace

AgentLongitudinalDecider::AgentLongitudinalDecider(
    const EgoPlanningConfigBuilder* config_builder, framework::Session* session)
    : Task(config_builder, session) {
  name_ = "AgentLongitudinalDecider";
  if (crossing_agent_decider_ == nullptr) {
    crossing_agent_decider_ =
        std::make_shared<CrossingAgentDecider>(config_builder, session);
  }
}

bool AgentLongitudinalDecider::Reset() {
  // if (crossing_agent_decider_ != nullptr) {
  //   crossing_agent_decider_->Reset();
  // }
  return false;
}

bool AgentLongitudinalDecider::Execute() {
  LOG_DEBUG("=======AgentLongitudinalDecider======= \n");

  if (!PreCheck()) {
    LOG_DEBUG("PreCheck failed\n");
    return false;
  }

  // Init
  Init();

  // Update
  Update();

  return true;
}

void AgentLongitudinalDecider::Init() {
  // get virtual lane manager
  virtual_lane_manager_ =
      session_->environmental_model().get_virtual_lane_manager();
  // get dynamic world
  dynamic_world_ = session_->environmental_model().get_dynamic_world();
  // get ego_state_manager
  ego_state_manager_ = session_->environmental_model().get_ego_state_manager();
}

bool AgentLongitudinalDecider::Update() {
  FilterRearAgents();

  FilterUltradistantObs();

  FilterReverseAgents();

  // Check cut in/out agent.
  DeciderCutInAndOutAgents();

  // AddCutInForLaneChange();

  UpdateCutInAgentTable();

  // crossing agent decider
  // 横穿障碍物依赖障碍物预测轨迹
  if (crossing_agent_decider_ != nullptr) {
    crossing_agent_decider_->Execute();
  }

  return true;
}

void AgentLongitudinalDecider::DeciderCutInAndOutAgents() {
  // get lateral output
  const auto& lateral_behavior_planner_output =
      session_->planning_context().lateral_behavior_planner_output();
  const auto& lc_request = lateral_behavior_planner_output.lc_request;
  const auto& lc_status = lateral_behavior_planner_output.lc_status;
  const bool is_in_lane_change =
      (lc_request != "none") &&
      ((lc_status == "left_lane_change") || (lc_status == "right_lane_change"));

  // ignore cutin in lane change
  /* if (is_in_lane_change) {
    return;
  } */

  const auto* agent_manager = dynamic_world_->agent_manager();
  auto* mutable_agent_manager = dynamic_world_->mutable_agent_manager();

  if (agent_manager == nullptr || mutable_agent_manager == nullptr) {
    LOG_ERROR("agent_manager is empty");
    return;
  }

  const auto& agents = agent_manager->GetAllCurrentAgents();
  if (agents.empty()) {
    return;
  }

  const auto& vehicle_param =
      VehicleConfigurationContext::Instance()->get_vehicle_param();
  const double ego_half_length = vehicle_param.length * kHalf;
  const double ego_half_width = vehicle_param.width * kHalf + 0.15;
  const double cut_in_lateral_threshold_m = ego_half_width + 2.0;
  const auto& init_point = ego_state_manager_->planning_init_point();
  const double ego_speed_mps = init_point.v;
  const double ego_theta = init_point.heading_angle;

  double cut_in_distance_range_m = ego_speed_mps * kCutInHeadwayRangeS;
  cut_in_distance_range_m =
      std::fmax(cut_in_distance_range_m, kMinCutInDistanceM);
  cut_in_distance_range_m =
      std::fmin(cut_in_distance_range_m, kMaxCutInDistanceM);

  for (const auto agent : agents) {
    if (nullptr == agent) {
      continue;
    }
    if (!(agent->fusion_source() & OBSTACLE_SOURCE_CAMERA)) {
      continue;
    }
    const int32_t agent_id = agent->agent_id();
    current_agent_ids_.insert(agent_id);

    DeciderCutInAgent(is_in_lane_change, *agent, ego_speed_mps, ego_theta,
                      ego_half_length, ego_half_width, init_point,
                      cut_in_distance_range_m, cut_in_lateral_threshold_m,
                      mutable_agent_manager);
  }
  DeciderCutOutAgent(mutable_agent_manager);
}

void AgentLongitudinalDecider::DeciderCutInAgent(
    const bool is_lane_change, const agent::Agent& agent,
    const double ego_speed_mps, const double ego_theta,
    const double ego_half_length, const double ego_half_width,
    const PlanningInitPoint init_point, const double cut_in_distance_range_m,
    const double cut_in_lateral_threshold_m,
    agent::AgentManager* const mutable_agent_manager) {
  constexpr double kLargeAgentLowerSmallHeadingDiff = 1.5 / 57.3;
  constexpr double kLargeAgentUpperSmallHeadingDiff = 45 / 57.3;

  if (nullptr == mutable_agent_manager) {
    return;
  }

  // get ego_lane
  const auto& ego_lane = virtual_lane_manager_->get_current_lane();
  if (ego_lane == nullptr) {
    return;
  }
  // get reference path from ego lane
  const auto& ego_reference_path = ego_lane->get_reference_path();
  if (ego_reference_path == nullptr) {
    return;
  }
  const auto& ego_lane_coord = ego_reference_path->get_frenet_coord();
  if (ego_lane_coord == nullptr) {
    return;
  }
  double ego_s = 0;
  double ego_l = 0;
  if (!ego_lane_coord->XYToSL(init_point.x, init_point.y, &ego_s, &ego_l)) {
    return;
  }

  double agent_s_base_path = 0.0;
  double agent_l_base_path = 0.0;
  if (!ego_lane_coord->XYToSL(agent.x(), agent.y(), &agent_s_base_path,
                              &agent_l_base_path)) {
    return;
  }
  const auto agent_matched_path_point =
      ego_lane_coord->GetPathPointByS(agent_s_base_path);
  const double agent_matched_lane_theta = agent_matched_path_point.theta();

  const int32_t agent_id = agent.agent_id();
  const double agent_relative_theta =
      planning_math::NormalizeAngle(agent.theta() - agent_matched_lane_theta);
  double object_s_speed_mps = agent.speed() * std::cos(agent_relative_theta);
  double object_l_speed_mps = agent.speed() * std::sin(agent_relative_theta);

  const bool is_large_agent = IsLargeAgent(agent);

  // min/max sl
  double min_s = std::numeric_limits<double>::max();
  double max_s = std::numeric_limits<double>::lowest();
  double min_l = std::numeric_limits<double>::max();
  double max_l = std::numeric_limits<double>::lowest();
  CalculateAgentSLBoundary(ego_lane_coord, agent, &min_s, &max_s, &min_l,
                           &max_l);
  const bool is_large_agent_cutin = IsLargeAgentCutIn(
      ego_lane, ego_lane_coord, agent, max_s, cut_in_distance_range_m,
      ego_half_length, ego_s, ego_theta, ego_speed_mps);

  // lateral distance
  double small_lateral_distance = std::numeric_limits<double>::max();
  double small_lateral_distance_with_ego_l = std::numeric_limits<double>::max();
  double large_lateral_distance = std::numeric_limits<double>::max();
  CalculateAgentLateralDistance(
      object_l_speed_mps, min_l, max_l, max_s, ego_speed_mps, ego_s, ego_l,
      &small_lateral_distance, &small_lateral_distance_with_ego_l,
      &large_lateral_distance);

  // drel
  std::array<double, 3> xp{0, 30, 60};
  std::array<double, 3> fp{1, 2, kRuleBasedCutInCount};
  double min_dis_to_front_bump = std::max(min_s - ego_s - ego_half_length, 0.0);
  double rule_base_cut_in_count = interp(min_dis_to_front_bump, xp, fp);

  // current kappa
  const double current_kappa = ego_lane_coord->GetPathPointByS(max_s).kappa();

  // ttc: time to collison
  double small_lateral_ttc = std::numeric_limits<double>::max();
  if (object_l_speed_mps * small_lateral_distance < 0.0) {
    small_lateral_ttc =
        std::fmax((std::fabs(small_lateral_distance) - ego_half_width), 0) /
        std::fmax(0.1, std::fabs(object_l_speed_mps));
  } else {
    small_lateral_ttc = 0;
  }

  const double current_kappa_threshold =
      is_large_agent ? kCurrentKappaThresholdForLargeAgent
                     : kCurrentKappaThreshold;

  const double cut_in_ttc_threshold = is_large_agent
                                          ? kLargeCutInLateralTtcThresholdS
                                          : kCutInLateralTtcThresholdS;

  const bool is_agent_closer_to_ego =
      object_l_speed_mps * small_lateral_distance < 0.0;
  const bool is_agent_ahead_of_ego = (max_s - ego_s) > ego_half_length;
  const bool is_agent_not_too_far = (max_s - ego_s) < cut_in_distance_range_m;
  const bool agent_speed_meet = agent.speed() > kCutInSpeedLimitMps;
  const bool current_kappa_meet =
      std::fabs(current_kappa) < current_kappa_threshold;
  const bool lateral_ttc_meet =
      small_lateral_ttc < kCutInLateralTtcThresholdS &&
      std::fabs(small_lateral_distance) < cut_in_lateral_threshold_m &&
      std::fabs(object_l_speed_mps) > kCutInLateralSpeedThresholdMps;
  const bool low_speed_and_large_yaw_meet =
      agent_relative_theta * small_lateral_distance < 0 &&
      std::fabs(agent_relative_theta) > kLargeYawThresholdRad &&
      std::fabs(small_lateral_distance) <
          kLaneWidth * kHalf + kLargeYawLateralDistanceBufferM &&
      std::fabs(object_l_speed_mps) > kLargeYawLateralSpeedThresholdMps;
  const bool is_reverse_agent = (object_s_speed_mps < -3.0);

  bool current_rule_base_cutin =
      is_agent_closer_to_ego && is_agent_ahead_of_ego && is_agent_not_too_far &&
      agent_speed_meet && current_kappa_meet &&
      (lateral_ttc_meet || low_speed_and_large_yaw_meet);

  if (is_large_agent && !is_large_agent_cutin) {
    double agent_s = 0.0;
    double agent_l = 0.0;
    if (!ego_lane_coord->XYToSL(agent.box().center_x(), agent.box().center_y(),
                                &agent_s, &agent_l)) {
      return;
    }
    const double heading_diff_with_path = planning_math::NormalizeAngle(
        agent.theta() - ego_lane_coord->GetPathPointByS(agent_s).theta());
    const bool is_large_agent_heading_diff_meet =
        agent_l < kEpsilon
            ? (heading_diff_with_path > kLargeAgentLowerSmallHeadingDiff &&
               heading_diff_with_path < kLargeAgentUpperSmallHeadingDiff)
            : (heading_diff_with_path < -kLargeAgentLowerSmallHeadingDiff &&
               heading_diff_with_path > -kLargeAgentUpperSmallHeadingDiff);
    current_rule_base_cutin &= is_large_agent_heading_diff_meet;
  }

  bool is_slow_need_suppression = false;
  IsSlowSpeedCutinSuppression(ego_lane_coord, init_point, is_lane_change, agent,
                              &is_slow_need_suppression);

  current_rule_base_cutin |= is_large_agent_cutin;
  current_rule_base_cutin &= !is_lane_change;
  if (is_slow_need_suppression) {
    current_rule_base_cutin = false;
  }
  const bool is_prediction_cut_in = agent.is_prediction_cutin();

  auto* mutable_agent = mutable_agent_manager->mutable_agent(agent_id);
  if (nullptr == mutable_agent) {
    return;
  }
  mutable_agent->set_is_rule_base_cutin(current_rule_base_cutin);
  mutable_agent->set_is_reverse_cutin(is_reverse_agent);

  mutable_agent->set_d_path(
      std::fmax(std::fabs(small_lateral_distance) - ego_half_width, 0.0));
  mutable_agent->set_d_rel(std::fmax(min_s - ego_s - ego_half_length, 0.0));

  bool current_cut_in_rule = false;
  bool current_cut_in_pred = false;
  if ((kFtpCutInDeactivationSpeedMps > ego_speed_mps) &&
      (ego_speed_mps > kEgoStopSpeedThresholdMps) &&
      (is_prediction_cut_in || current_rule_base_cutin)) {
    // 1.consider rule base cut in or prediction cut in when ego's speed <
    // 100kph
    current_cut_in_rule = current_rule_base_cutin;
    current_cut_in_pred = is_prediction_cut_in;
  } else if (kFtpCutInDeactivationSpeedMps <= ego_speed_mps &&
             current_rule_base_cutin) {
    // 2.only consider rule base cut in when ego's speed >= 100 kph
    current_cut_in_rule = current_rule_base_cutin;
  }

  rule_based_cut_in_agent_count_[agent_id] =
      current_cut_in_rule == true ? ++rule_based_cut_in_agent_count_[agent_id]
                                  : 0;
  pred_cut_in_agent_count_[agent_id] =
      current_cut_in_pred ? ++pred_cut_in_agent_count_[agent_id] : 0;

  if (rule_based_cut_in_agent_count_[agent_id] >= rule_base_cut_in_count ||
      pred_cut_in_agent_count_[agent_id] >= kPredBasedCutInCount) {
    cut_in_agent_count_[agent_id] = kDefaultCutInCount;
  } else {
    cut_in_agent_count_[agent_id] =
        std::fmax(0, cut_in_agent_count_[agent_id] - 1);
  }
  if (cut_in_agent_count_[agent_id] > 0) {
    mutable_agent->set_is_cutin(true);
  }
}

bool AgentLongitudinalDecider::IsLargeAgentCutIn(
    const std::shared_ptr<VirtualLane> ego_lane,
    const std::shared_ptr<KDPath>& planned_path, const agent::Agent& agent,
    const double agent_max_s, const double cut_in_distance_range_m,
    const double ego_half_length, const double ego_s, const double ego_theta,
    const double ego_speed_mps) {
  constexpr double kSpeedThresholdKph = 30.0;
  const bool speed_meet = ego_speed_mps * kMpsToKph > kSpeedThresholdKph &&
                          agent.speed() * kMpsToKph > kSpeedThresholdKph;

  const bool is_large_agent = IsLargeAgent(agent);
  // 1. large agent and fast enough
  // 2. agent is not in ego lane,but must be neighbor to ego lane
  // 3. agent is not far,and agent's front s is larger than ego's front s
  // 4. heading diff in range [3,45]
  // 5. may cross boundary
  if (!is_large_agent || !speed_meet) {
    return false;
  }

  double agent_s = 0.0;
  double agent_l = 0.0;
  if (!planned_path->XYToSL(agent.x(), agent.y(), &agent_s, &agent_l)) {
    return false;
  }

  double half_lane_width_by_s = 0.5 * ego_lane->width_by_s(agent_s);
  if (std::fabs(agent_l) < half_lane_width_by_s) {
    return false;
  }

  // TODO: add neighbor lane width
  double neighbor_lane_width_by_s = 0.0;
  if (agent_l < kEpsilon) {
    // get right lane
    const auto& right_lane = virtual_lane_manager_->get_right_lane();
    if (right_lane == nullptr) {
      return false;
    }
    neighbor_lane_width_by_s = right_lane->width_by_s(agent_s);
  } else {
    // get left lane
    const auto& left_lane = virtual_lane_manager_->get_left_lane();
    if (left_lane == nullptr) {
      return false;
    }
    neighbor_lane_width_by_s = left_lane->width_by_s(agent_s);
  }
  const bool is_neighbor =
      std::fabs(agent_l) <= half_lane_width_by_s + neighbor_lane_width_by_s &&
      std::fabs(agent_l) >= half_lane_width_by_s;
  if (!is_neighbor) {
    return false;
  }

  const bool is_agent_ahead_of_ego = (agent_max_s - ego_s) > ego_half_length;
  const bool is_agent_not_too_far =
      (agent_max_s - ego_s) < cut_in_distance_range_m;
  if (!(is_agent_ahead_of_ego && is_agent_not_too_far)) {
    return false;
  }

  constexpr double kLowerSmallHeadingDiff = 3 / 57.3;
  constexpr double kUpperSmallHeadingDiff = 45 / 57.3;
  double agent_box_s = 0.0;
  double agent_box_l = 0.0;
  planned_path->XYToSL(agent.box().center_x(), agent.box().center_y(),
                       &agent_box_s, &agent_box_l);
  const double heading_diff_with_path = planning_math::NormalizeAngle(
      agent.theta() - planned_path->GetPathPointByS(agent_s).theta());
  const bool is_heading_diff_in_range =
      agent_l < kEpsilon ? (heading_diff_with_path > kLowerSmallHeadingDiff &&
                            heading_diff_with_path < kUpperSmallHeadingDiff)
                         : (heading_diff_with_path < -kLowerSmallHeadingDiff &&
                            heading_diff_with_path > -kUpperSmallHeadingDiff);
  if (!is_heading_diff_in_range) {
    return false;
  }

  bool is_cross_boundary = false;
  const auto& corners = agent.box().GetAllCorners();
  for (const auto& corner : corners) {
    double corner_s = 0.0;
    double corner_l = 0.0;
    if (!planned_path->XYToSL(corner.x(), corner.y(), &corner_s, &corner_l)) {
      continue;
    }
    double current_left_width = 0.5 * ego_lane->width_by_s(corner_s);
    double current_right_width = 0.5 * ego_lane->width_by_s(corner_s);
    is_cross_boundary = corner_l < kEpsilon
                            ? corner_l > -(current_right_width - 0.2)
                            : corner_l < (current_left_width - 0.2);
    if (is_cross_boundary) {
      break;
    }
  }
  if (!is_cross_boundary) {
    return false;
  }
  return true;
};

bool AgentLongitudinalDecider::IsLargeAgent(const agent::Agent& agent) {
  return agent::AgentType::BUS == agent.type() ||
         agent::AgentType::TRUCK == agent.type() ||
         agent::AgentType::TRAILER == agent.type() ||
         agent.length() > kLargeAgentLengthM;
};

void AgentLongitudinalDecider::IsSlowSpeedCutinSuppression(
    const std::shared_ptr<KDPath>& planned_path,
    const PlanningInitPoint init_point, const bool is_lane_change,
    const agent::Agent& agent, bool* is_slow_need_suppression){};

void AgentLongitudinalDecider::CalculateAgentLateralDistance(
    const double object_l_speed_mps, const double min_l, const double max_l,
    const double max_s, const double ego_speed_mps, const double ego_s,
    const double ego_l, double* const ptr_small_lateral_distance,
    double* const ptr_small_lateral_distance_with_ego_l,
    double* const ptr_large_lateral_distance) {
  if (nullptr == ptr_small_lateral_distance ||
      nullptr == ptr_small_lateral_distance_with_ego_l ||
      nullptr == ptr_large_lateral_distance) {
    return;
  }

  if (object_l_speed_mps < -0.1) {
    *ptr_small_lateral_distance = min_l;
    *ptr_small_lateral_distance_with_ego_l = min_l - ego_l;
    *ptr_large_lateral_distance = max_l - ego_l;
  } else if (object_l_speed_mps > 0.1) {
    *ptr_small_lateral_distance = max_l;
    *ptr_small_lateral_distance_with_ego_l = max_l - ego_l;
    *ptr_large_lateral_distance = min_l - ego_l;
  } else {
    *ptr_small_lateral_distance =
        std::fabs(min_l) < std::fabs(max_l) ? min_l : max_l;
    *ptr_small_lateral_distance_with_ego_l =
        *ptr_small_lateral_distance - ego_l;
    *ptr_large_lateral_distance =
        std::fabs(min_l) > std::fabs(max_l) ? min_l : max_l - ego_l;
  }

  const double centering_t_s = 1.0;
  double dist_ratio = (max_s - ego_s) / (ego_speed_mps * centering_t_s);
  dist_ratio = std::fmax(dist_ratio, 0.0);
  dist_ratio = std::fmin(dist_ratio, 1.0);
  *ptr_small_lateral_distance =
      *ptr_small_lateral_distance * dist_ratio +
      (1 - dist_ratio) * (*ptr_small_lateral_distance_with_ego_l);
};

void AgentLongitudinalDecider::DeciderCutOutAgent(
    agent::AgentManager* const mutable_agent_manager) {
  if (nullptr == mutable_agent_manager) {
    return;
  }

  const auto& cipv_decider_output =
      session_->planning_context().cipv_decider_output();
  const int32_t cipv_id = cipv_decider_output.cipv_id();

  auto* ptr_agent = mutable_agent_manager->mutable_agent(cipv_id);
  if (nullptr == ptr_agent || ptr_agent->trajectories().empty() ||
      ptr_agent->trajectories().front().empty()) {
    return;
  }

  const bool is_cut_out = CheckCutOutAgent(*ptr_agent);
  cut_out_agent_count_[cipv_id] =
      is_cut_out ? ++cut_out_agent_count_[cipv_id] : 0;

  constexpr int32_t kSteadyCutOutCheckCount = 2;
  constexpr int32_t kSteadyCutOutKeepCount = 4;
  if (cut_out_agent_count_[cipv_id] >= kSteadyCutOutCheckCount) {
    steady_cut_out_agent_count_[cipv_id] = kSteadyCutOutKeepCount;
  } else {
    steady_cut_out_agent_count_[cipv_id] =
        std::fmax(0, steady_cut_out_agent_count_[cipv_id] - 1);
  }

  if (steady_cut_out_agent_count_[cipv_id] > 0) {
    ptr_agent->set_need_backward_extend(true);
    ptr_agent->set_is_cutout(true);
  }
}

bool AgentLongitudinalDecider::CheckCutOutAgent(const agent::Agent& agent) {
  const auto& trajectory = agent.trajectories().front();
  const auto& end_point = trajectory.back();
  const double agent_end_time = end_point.absolute_time();

  const auto& lane_change_decider_output =
      session_->planning_context().lane_change_decider_output();
  const auto& lane_change_status = lane_change_decider_output.curr_state;
  const bool is_in_lane_change = (lane_change_status == kLaneChangeExecution ||
                                  lane_change_status == kLaneChangeComplete);

  const bool is_vru = agent.is_vru();

  const auto& current_lane = virtual_lane_manager_->get_current_lane();
  if (current_lane == nullptr) {
    return false;
  }
  const auto current_lane_coord = current_lane->get_lane_frenet_coord();
  if (current_lane_coord == nullptr) {
    return false;
  }

  // 0.check the cipv agent
  // 1.should not reverse agent
  // if (IsReverseAgent(&agent, current_lane)) {
  //   return false;
  // }

  // 2.agent's speed < 30kph
  constexpr double kSpeedThresholdKph = 30.0;
  const bool speed_meet = agent.speed() * kMpsToKph < kSpeedThresholdKph;
  if (!speed_meet) {
    return false;
  }

  double agent_s_in_lane = 0.0;
  double agent_l_in_lane = 0.0;
  if (!current_lane_coord->XYToSL(agent.x(), agent.y(), &agent_s_in_lane,
                                  &agent_l_in_lane)) {
    return false;
  }

  // 3.agent's heading diff with lane is at [3,90]
  const auto matched_point_in_lane =
      current_lane_coord->GetPathPointByS(agent_s_in_lane);
  const double heading_diff = std::fabs(planning_math::NormalizeAngle(
      agent.theta() - matched_point_in_lane.theta()));
  constexpr double kLowerHeadingDiff = 3.0 / 57.3;
  constexpr double kUpperHeadingDiff = 90.0 / 57.3;
  const bool heading_diff_meet =
      heading_diff > kLowerHeadingDiff && heading_diff < kUpperHeadingDiff;
  if (!heading_diff_meet) {
    return false;
  }

  // 4.agent's trajectory end point l - agent's l > 3m, relative point s -
  // agent's s < 15.0m
  constexpr double kRelativeTime = 3.5;
  const auto relative_point = kRelativeTime < agent_end_time
                                  ? trajectory.Evaluate(kRelativeTime)
                                  : end_point;
  double end_point_s_in_lane = 0.0;
  double end_point_l_in_lane = 0.0;
  double relative_point_s_in_lane = 0.0;
  double relative_point_l_in_lane = 0.0;
  if (!current_lane_coord->XYToSL(end_point.x(), end_point.y(),
                                  &end_point_s_in_lane, &end_point_l_in_lane) ||
      !current_lane_coord->XYToSL(relative_point.x(), relative_point.y(),
                                  &relative_point_s_in_lane,
                                  &relative_point_l_in_lane)) {
    return false;
  }
  // const double kLateralDiff = is_vru ? agent.width() : 3.0;
  const double kLateralDiff = agent.width();
  constexpr double kLongitudinalDiff = 20.0;
  const bool lateral_diff_meet =
      std::fabs(agent_l_in_lane - end_point_l_in_lane) > kLateralDiff;
  const bool longitudinal_diff_meet =
      std::fabs(relative_point_s_in_lane - agent_s_in_lane) < kLongitudinalDiff;
  if (!lateral_diff_meet || !longitudinal_diff_meet) {
    return false;
  }

  // cut-out intention
  // 5. agent has cut-out intention
  const bool have_cut_out_intention = HaveCutOutIntention(
      agent, current_lane_coord, matched_point_in_lane, end_point_l_in_lane);
  if (!have_cut_out_intention) {
    return false;
  }

  // 6.agent is in lateral range of planned path,range is half of vehicle width
  // + 0.35
  const auto& vehicle_param =
      VehicleConfigurationContext::Instance()->get_vehicle_param();
  const double vehicle_width = vehicle_param.width;
  const double left_border_position = vehicle_width * 0.5;
  const double right_border_position = -left_border_position;
  constexpr double kExpandBuffer = 0.35;

  auto tmp_box = agent.box();
  tmp_box.LateralExtend(kExpandBuffer * 2);
  double min_s = std::numeric_limits<double>::max();
  double max_s = std::numeric_limits<double>::lowest();
  double min_l = std::numeric_limits<double>::max();
  double max_l = std::numeric_limits<double>::lowest();
  CalculateAgentSLBoundary(current_lane_coord, tmp_box, &min_s, &max_s, &min_l,
                           &max_l);
  bool in_lateral_range =
      !(min_l > left_border_position || max_l < right_border_position);
  return in_lateral_range;
}

bool AgentLongitudinalDecider::HaveCutOutIntention(
    const agent::Agent& agent,
    const std::shared_ptr<planning_math::KDPath>& current_lane_coord,
    const planning_math::PathPoint& agent_matched_point_in_lane,
    const double agent_trajetory_end_point_l_in_lane) {
  const auto& agent_center_point = agent.box().center();
  const double rear_theta = planning_math::NormalizeAngle(agent.theta() + M_PI);
  const auto rear_point =
      agent_center_point + planning_math::Vec2d::CreateUnitVec2d(rear_theta) *
                               agent.length() * kHalf;
  const auto front_point =
      agent_center_point +
      planning_math::Vec2d::CreateUnitVec2d(agent.theta()) * agent.length() *
          kHalf;
  double rear_point_s_in_lane = 0.0;
  double rear_point_l_in_lane = 0.0;
  if (!current_lane_coord->XYToSL(rear_point.x(), rear_point.y(),
                                  &rear_point_s_in_lane,
                                  &rear_point_l_in_lane)) {
    return false;
  }
  double front_point_s_in_lane = 0.0;
  double front_point_l_in_lane = 0.0;
  if (!current_lane_coord->XYToSL(front_point.x(), front_point.y(),
                                  &front_point_s_in_lane,
                                  &front_point_l_in_lane)) {
    return false;
  }
  const double signed_heading_diff = planning_math::NormalizeAngle(
      agent.theta() - agent_matched_point_in_lane.theta());
  bool has_cut_out_intention = false;
  if (signed_heading_diff > kEpsilon) {
    has_cut_out_intention =
        std::fabs(front_point_l_in_lane) > std::fabs(rear_point_l_in_lane) &&
        (agent_trajetory_end_point_l_in_lane > front_point_l_in_lane) &&
        (front_point_l_in_lane > kEpsilon);
  } else if (signed_heading_diff < -kEpsilon) {
    has_cut_out_intention =
        std::fabs(front_point_l_in_lane) > std::fabs(rear_point_l_in_lane) &&
        (agent_trajetory_end_point_l_in_lane < front_point_l_in_lane) &&
        (front_point_l_in_lane < -kEpsilon);
  }
  return has_cut_out_intention;
}

void AgentLongitudinalDecider::UpdateCutInAgentTable() {
  for (auto iter = cut_in_agent_count_.begin();
       iter != cut_in_agent_count_.end();) {
    if (current_agent_ids_.count(iter->first) == 0) {
      iter = cut_in_agent_count_.erase(iter);
    } else {
      ++iter;
    }
  }

  for (auto iter = pred_cut_in_agent_count_.begin();
       iter != pred_cut_in_agent_count_.end();) {
    if (current_agent_ids_.count(iter->first) == 0) {
      iter = pred_cut_in_agent_count_.erase(iter);
    } else {
      ++iter;
    }
  }

  for (auto iter = rule_based_cut_in_agent_count_.begin();
       iter != rule_based_cut_in_agent_count_.end();) {
    if (current_agent_ids_.count(iter->first) == 0) {
      iter = rule_based_cut_in_agent_count_.erase(iter);
    } else {
      ++iter;
    }
  }

  for (auto iter = cut_out_agent_count_.begin();
       iter != cut_out_agent_count_.end();) {
    if (current_agent_ids_.count(iter->first) == 0) {
      iter = cut_out_agent_count_.erase(iter);
    } else {
      ++iter;
    }
  }

  for (auto iter = steady_cut_out_agent_count_.begin();
       iter != steady_cut_out_agent_count_.end();) {
    if (current_agent_ids_.count(iter->first) == 0) {
      iter = steady_cut_out_agent_count_.erase(iter);
    } else {
      ++iter;
    }
  }
  current_agent_ids_.clear();

  // debug
  std::vector<double> cutin_id;
  std::vector<double> cutin_count;
  for (const auto cut_in_agent : cut_in_agent_count_) {
    if (cut_in_agent.second > 0) {
      cutin_id.emplace_back(cut_in_agent.first);
      cutin_count.emplace_back(cut_in_agent.second);
    }
  }
  JSON_DEBUG_VECTOR("new_cutin_id", cutin_id, 0)
  JSON_DEBUG_VECTOR("new_cutin_id_count", cutin_count, 0)

  std::vector<double> cutout_id;
  std::vector<double> cutout_count;
  for (const auto cut_out_agent : steady_cut_out_agent_count_) {
    if (cut_out_agent.second > 0) {
      cutout_id.emplace_back(cut_out_agent.first);
      cutout_count.emplace_back(cut_out_agent.second);
    }
  }
  JSON_DEBUG_VECTOR("new_cutout_id", cutout_id, 0)
  JSON_DEBUG_VECTOR("new_cutout_id_count", cutout_count, 0)
};

void AgentLongitudinalDecider::FilterRearAgents() {
  const auto& agent_manager =
      session_->environmental_model().get_agent_manager();
  auto& mutable_agent_manager =
      session_->mutable_environmental_model()->mutable_agent_manager();
  if (agent_manager == nullptr || mutable_agent_manager == nullptr) {
    return;
  }
  const auto& agents = agent_manager->GetAllCurrentAgents();
  if (agents.empty()) {
    return;
  }

  // get lane change info
  const auto& lane_change_decider_output =
      session_->planning_context().lane_change_decider_output();
  // const auto& coarse_planning_info =
  //     lane_change_decider_output.coarse_planning_info;
  const auto lc_request_direction = lane_change_decider_output.lc_request;
  const auto lane_change_state = lane_change_decider_output.curr_state;
  const auto is_in_lane_change_execution =
      lane_change_state == kLaneChangeExecution;
  const auto is_lane_change_complete = lane_change_state == kLaneChangeComplete;

  const auto& ego_state_manager =
      session_->environmental_model().get_ego_state_manager();
  if (ego_state_manager == nullptr) {
    return;
  }
  const auto& planning_init_point = ego_state_manager->planning_init_point();
  // const double distance_thr_for_rear = 50.0;

  // get ego_lane
  const auto& ego_lane = virtual_lane_manager_->get_current_lane();
  if (ego_lane == nullptr) {
    return;
  }
  // get reference path from ego lane
  const auto& ego_reference_path = ego_lane->get_reference_path();
  if (ego_reference_path == nullptr) {
    return;
  }
  const auto& ego_lane_coord = ego_reference_path->get_frenet_coord();
  if (ego_lane_coord == nullptr) {
    return;
  }

  double ego_s = 0;
  double ego_l = 0;
  if (!ego_lane_coord->XYToSL(planning_init_point.x, planning_init_point.y,
                              &ego_s, &ego_l)) {
    return;
  }

  // TODO: only insert ego_right_rear_node_id in target_lane_rear_agents
  //       if want consider more rear agent, insert there
  int64_t target_lane_rear_node_id = -1;
  std::unordered_set<int32_t> target_lane_rear_agents;
  if (lc_request_direction == LEFT_CHANGE) {
    if (lane_change_state == kLaneChangeExecution ||
        lane_change_state == kLaneChangeComplete) {
      target_lane_rear_node_id = dynamic_world_->ego_rear_node_id();
    } else {
      target_lane_rear_node_id = dynamic_world_->ego_left_rear_node_id();
    }
  } else if (lc_request_direction == RIGHT_CHANGE) {
    if (lane_change_state == kLaneChangeExecution ||
        lane_change_state == kLaneChangeComplete) {
      target_lane_rear_node_id = dynamic_world_->ego_rear_node_id();
    } else {
      target_lane_rear_node_id = dynamic_world_->ego_right_rear_node_id();
    }
  }
  if (target_lane_rear_node_id != -1) {
    auto* target_lane_rear_node =
        dynamic_world_->GetNode(target_lane_rear_node_id);
    if (target_lane_rear_node != nullptr) {
      target_lane_rear_agents.insert(target_lane_rear_node->node_agent_id());
    }
  }

  // vehicle param
  const auto& vehicle_param =
      VehicleConfigurationContext::Instance()->get_vehicle_param();
  const double vehicle_length = vehicle_param.length;
  const double vehicle_width = vehicle_param.width;
  const double rear_axle_to_front_axle = vehicle_param.wheel_base;
  const double rear_axle_to_rear_edge =
      vehicle_length - vehicle_param.front_edge_to_rear_axle;
  const double rear_axle_to_front_edge = vehicle_param.front_edge_to_rear_axle;
  const double rear_axle_to_center = vehicle_param.rear_axle_to_center;
  double ego_rear_edge_s = ego_s - rear_axle_to_rear_edge;
  double ego_front_edge_s = ego_s + rear_axle_to_front_edge;
  double ego_center_s = ego_s + rear_axle_to_center;
  double ego_front_axle_s = ego_s + rear_axle_to_front_axle;
  if (session_->is_rads_scene()) {
    ego_rear_edge_s = ego_s - rear_axle_to_front_edge;
    ego_front_edge_s = ego_s + rear_axle_to_rear_edge;
    ego_center_s = ego_s - rear_axle_to_center;
  }

  for (const auto agent : agents) {
    if (agent == nullptr) {
      continue;
    }
    if (agent->is_tfl_virtual_obs() ||
        agent->is_stop_destination_virtual_obs()) {
      continue;
    }
    double agent_s = 0.0;
    double agent_l = 0.0;
    if (!ego_lane_coord->XYToSL(agent->x(), agent->y(), &agent_s, &agent_l)) {
      continue;
    }

    const double half_length = agent->length() * 0.5;

    bool is_no_need_expand_agent = false;
    // update no need for expansion agents.
    if (ego_front_axle_s > agent_s + half_length) {
      is_no_need_expand_agent = true;
    }

    double front_corner_s = -std::numeric_limits<double>::max();
    double corner_max_l = -std::numeric_limits<double>::max();
    double corner_min_l = std::numeric_limits<double>::max();
    std::vector<planning_math::Vec2d> corners;
    agent->box().GetAllCorners(&corners);
    for (const auto& corner : corners) {
      double corner_s = 0.0;
      double corner_l = 0.0;
      if (!ego_lane_coord->XYToSL(corner.x(), corner.y(), &corner_s,
                                  &corner_l)) {
        continue;
      }
      if (corner_s > front_corner_s) {
        front_corner_s = corner_s;
      }
      corner_min_l = std::fmin(corner_min_l, corner_l);
      corner_max_l = std::fmax(corner_max_l, corner_l);
    }

    double min_lat_l_from_ego = 0.0;
    double corner_l_from_ego = 0.0;
    if (agent_l > kEpsilon) {
      corner_l_from_ego = corner_min_l - ego_l;
      min_lat_l_from_ego =
          ((agent_l - ego_l) * corner_l_from_ego) > 0 ? corner_l_from_ego : 0.0;
    } else {
      corner_l_from_ego = corner_max_l - ego_l;
      min_lat_l_from_ego =
          ((agent_l - ego_l) * corner_l_from_ego) > 0 ? corner_l_from_ego : 0.0;
    }

    if (std::fabs(front_corner_s) < kEpsilon) {
      continue;
    }

    const double front_edge_s_diff = front_corner_s - ego_front_axle_s;
    // filter front agent
    if (front_edge_s_diff > kEpsilon) {
      continue;
    }

    // filter consider rear agent
    if (IsConsiderBackObs(ego_lane_coord, planning_init_point, agent.get(),
                          ego_front_edge_s, front_corner_s, ego_center_s,
                          front_edge_s_diff, min_lat_l_from_ego)) {
      continue;
    }

    if (is_in_lane_change_execution || is_lane_change_complete) {
      if (target_lane_rear_agents.find(agent->agent_id()) !=
          target_lane_rear_agents.end()) {
        continue;
      }
    }

    auto* mutable_agent =
        mutable_agent_manager->mutable_agent(agent->agent_id());
    if (mutable_agent == nullptr) {
      continue;
    }
    mutable_agent->mutable_agent_decision()->set_agent_decision_type(
        agent::AgentDecisionType::IGNORE);
  }
}

bool AgentLongitudinalDecider::IsConsiderBackObs(
    const std::shared_ptr<KDPath> planned_path,
    const PlanningInitPoint& init_point, const agent::Agent* agent,
    const double ego_front_s, const double front_corner_s,
    const double ego_center_s, const double front_edge_s_diff,
    const double min_lat_l_from_ego) const {
  const auto& vehicle_param =
      VehicleConfigurationContext::Instance()->get_vehicle_param();
  const double k_agent_ignore_only_position_diff_thd =
      -(vehicle_param.length / 2.0 + 8.0);
  const double ego_width = vehicle_param.width;

  auto& mutable_agent_manager =
      session_->mutable_environmental_model()->mutable_agent_manager();
  const double agent_speed_diff = agent->speed() - init_point.v;

  // 0.ego is slow(<15kph) and agent front s < ego center s ---> ignore
  constexpr double kEgoSlowSpeedKph = 15.0;
  const bool ignore_rear_agent_when_ego_slow =
      (init_point.v * kMpsToKph < kEgoSlowSpeedKph) &&
      (front_corner_s < ego_center_s);
  if (ignore_rear_agent_when_ego_slow) {
    return false;
  }

  // 1.agent is not cut-in and rear of ego
  const bool filter_rear_no_cut_in_agent =
      FilterRearNoCutInAgent(planned_path, init_point, ego_front_s, ego_width,
                             front_corner_s, agent, min_lat_l_from_ego);
  if (filter_rear_no_cut_in_agent) {
    return false;
  }

  // 2.车速相近，agent在自车后方 ---> ignore
  if (agent_speed_diff < kIgnoreSpeedDiffThd &&
      front_edge_s_diff < kAgentFrontEdgeSDiffThd) {
    return false;
  }

  // 3.filter too far rear agent ---> ignore
  if (front_edge_s_diff < k_agent_ignore_only_position_diff_thd) {
    return false;
  }

  return true;
}

bool AgentLongitudinalDecider::FilterRearNoCutInAgent(
    const std::shared_ptr<KDPath> planned_path,
    const PlanningInitPoint& init_point, const double ego_front_s,
    const double ego_width, const double agent_front_s,
    const agent::Agent* ptr_agent, const double min_lat_l_from_ego) const {
  constexpr double kAgentLowerSpeedKph = 40.0;
  const auto& virtual_lane_manager =
      session_->environmental_model().get_virtual_lane_manager();
  const auto& lateral_behavior_planner_output =
      session_->planning_context().lateral_behavior_planner_output();
  const auto lane_change_status = lateral_behavior_planner_output.lc_status;
  const auto lane_change_request = lateral_behavior_planner_output.lc_request;
  const bool is_lane_keeping = lane_change_request == "none" ||
                               lane_change_status == "none" ||
                               lane_change_status == "left_lane_change_wait" ||
                               lane_change_status == "right_lane_change_wait";

  if (nullptr == virtual_lane_manager) {
    return true;
  }
  const auto& ego_lane = virtual_lane_manager->get_current_lane();
  if (nullptr == ego_lane) {
    return true;
  }
  if (ptr_agent->trajectories().empty()) {
    return true;
  }

  // 1.agent is not in ego lane or nearest_l > half_ego_lane_width ---> consider
  double nearest_s = 0.0;
  double nearest_l = 0.0;
  bool need_judge = false;
  const auto& ptr_obj_lane = virtual_lane_manager->GetNearestLane(
      {ptr_agent->x(), ptr_agent->y()}, &nearest_s, &nearest_l);
  const double half_ego_lane_width = 0.5 * ego_lane->width_by_s(nearest_s);
  if (ptr_obj_lane == nullptr) {
    return true;
  }
  bool is_in_ego_lane =
      ptr_obj_lane->get_virtual_id() == ego_lane->get_virtual_id();
  const double nearest_distance_from_ego =
      std::fabs(min_lat_l_from_ego) - 0.5 * ego_width;
  if (is_in_ego_lane) {
    if (nearest_distance_from_ego > kLateralSafeBuffer) {
      need_judge = true;
    }
  }

  // 2.not lane keeping
  // 3.cut-in
  // 4.agent speed is fast(>40kph)  ---> consider
  // if (!is_lane_keeping || nullptr == ptr_agent || ptr_agent->is_cutin() ||
  //     ptr_agent->speed() * kMpsToKph < kAgentLowerSpeedKph ||
  //     ptr_agent->trajectories().empty()) {
  //   // return false;
  //   need_judge = true;
  // }
  if (!is_lane_keeping || nullptr == ptr_agent || ptr_agent->is_cutin()) {
    // return false;
    need_judge = true;
  }

  // 5.agent and ego lane is parallel  --->consider
  // 6.agent must be rear
  bool is_neighbor = false;
  double neighbor_speed_diff = ptr_agent->speed() - init_point.v;
  bool is_fast_agent = ptr_agent->speed() * kMpsToKph > kAgentLowerSpeedKph &&
                       neighbor_speed_diff > kLargeSpeedDiff;
  bool has_no_overlap_with_ego = nearest_distance_from_ego > kLateralSafeBuffer;
  if (ptr_obj_lane != nullptr) {
    is_neighbor = (std::abs(ptr_obj_lane->get_virtual_id() -
                            ego_lane->get_virtual_id()) == 1) &&
                  (nearest_l < std::fabs(half_ego_lane_width)) &&
                  has_no_overlap_with_ego;
  }
  // const bool is_parallel =
  // IsParallelToEgoLane(ptr_obj_lane->get_virtual_id());
  if (is_neighbor && is_fast_agent) {
    // return false;
    need_judge = true;
  }

  // 7.small heading diff
  double agent_s = 0.0;
  double agent_l = 0.0;
  const auto& ego_lane_coord = ego_lane->get_lane_frenet_coord();
  if (ego_lane_coord == nullptr) {
    return true;
  }
  ego_lane_coord->XYToSL(ptr_agent->x(), ptr_agent->y(), &agent_s, &agent_l);
  if (agent_s < 0.0 || agent_s > ego_lane_coord->Length()) {
    return true;
  }
  const auto ego_lane_point = ego_lane_coord->GetPathPointByS(agent_s);
  const double heading_diff_with_lane = planning_math::NormalizeAngle(
      ptr_agent->theta() - ego_lane_point.theta());
  // const double heading_diff_with_ego = planning_math::NormalizeAngle(
  //     ptr_agent->theta() - init_point.heading_angle);
  constexpr double kSmallHeadingDiff = 3 / 57.3;
  const bool small_heading_diff =
      agent_l < kEpsilon ? (heading_diff_with_lane < kSmallHeadingDiff /* &&
                                         heading_diff_with_ego < kSmallHeadingDiff*/)
                         : (heading_diff_with_lane > -kSmallHeadingDiff /*&&
                                         heading_diff_with_ego > -kSmallHeadingDiff*/);
  if (!small_heading_diff) {
    // return false;
    need_judge = true;
  }

  // 8.agent point(0s,1s,2s) has no overlap with ego planned_path
  const auto& vehicle_param =
      VehicleConfigurationContext::Instance()->get_vehicle_param();
  const double vehicle_width = vehicle_param.width;
  const double left_border_position = vehicle_width * 0.5;
  const double right_border_position = -left_border_position;
  const auto& trajectory = ptr_agent->trajectories().front();
  const double relative_time = init_point.relative_time;
  constexpr double kExpandBuffer = 0.35;
  std::vector<double> check_time{relative_time + 0.1, relative_time + 1.0,
                                 relative_time + 2.0, relative_time + 3.0,
                                 relative_time + 4.0, relative_time + 5.0};
  for (const double t : check_time) {
    const auto point = trajectory.Evaluate(t);

    Box2d obs_box(Vec2d(point.x(), point.y()), point.theta(),
                  ptr_agent->length(),
                  ptr_agent->width() + 2.0 * kExpandBuffer);
    double min_s = std::numeric_limits<double>::max();
    double max_s = std::numeric_limits<double>::lowest();
    double min_l = std::numeric_limits<double>::max();
    double max_l = std::numeric_limits<double>::lowest();
    CalculateAgentSLBoundary(planned_path, obs_box, &min_s, &max_s, &min_l,
                             &max_l);
    const double check_l = agent_l < kEpsilon ? max_l : min_l;
    const bool has_overlap = agent_l < kEpsilon
                                 ? check_l > right_border_position
                                 : check_l < left_border_position;
    if (has_overlap && need_judge) {
      return false;
    }
  }
  return true;
}

void AgentLongitudinalDecider::FilterUltradistantObs() {
  const auto& agent_manager =
      session_->environmental_model().get_agent_manager();
  auto& mutable_agent_manager =
      session_->mutable_environmental_model()->mutable_agent_manager();
  if (agent_manager == nullptr || mutable_agent_manager == nullptr) {
    return;
  }
  const auto& agents = agent_manager->GetAllCurrentAgents();
  if (agents.empty()) {
    return;
  }

  // get ego_lane
  const auto& ego_lane = virtual_lane_manager_->get_current_lane();
  if (ego_lane == nullptr) {
    return;
  }
  // get reference path from ego lane
  const auto& ego_reference_path = ego_lane->get_reference_path();
  if (ego_reference_path == nullptr) {
    return;
  }
  const auto& ego_lane_coord = ego_reference_path->get_frenet_coord();
  if (ego_lane_coord == nullptr) {
    return;
  }
  const auto& ego_state_manager =
      session_->environmental_model().get_ego_state_manager();
  if (ego_state_manager == nullptr) {
    return;
  }
  const auto& planning_init_point = ego_state_manager->planning_init_point();

  double ego_s = 0;
  double ego_l = 0;
  if (!ego_lane_coord->XYToSL(planning_init_point.x, planning_init_point.y,
                              &ego_s, &ego_l)) {
    return;
  }

  // vehicle param
  const auto& vehicle_param =
      VehicleConfigurationContext::Instance()->get_vehicle_param();
  // const double vehicle_length = vehicle_param.length;
  const double rear_axle_to_front_edge = vehicle_param.front_edge_to_rear_axle;
  const double rear_axle_to_rear_edge = vehicle_param.rear_edge_to_rear_axle;
  double ego_front_edge_s = ego_s + rear_axle_to_front_edge;
  if (session_->is_rads_scene()) {
    ego_front_edge_s = ego_s + rear_axle_to_rear_edge;
  }

  double filter_ultra_distance = kFilterUltraDistanceHighThd;
  filter_ultra_distance =
      GetFilterUltraDistanceWithEgoVel(planning_init_point.v);

  for (const auto agent : agents) {
    if (agent == nullptr) {
      continue;
    }
    double agent_s = 0.0;
    double agent_l = 0.0;
    if (!ego_lane_coord->XYToSL(agent->x(), agent->y(), &agent_s, &agent_l)) {
      continue;
    }

    const double half_length = agent->length() * 0.5;
    const double s_diff = (agent_s - half_length) - ego_front_edge_s;

    if (s_diff < filter_ultra_distance) {
      continue;
    }
    auto* mutable_agent =
        mutable_agent_manager->mutable_agent(agent->agent_id());
    if (mutable_agent == nullptr) {
      continue;
    }
    mutable_agent->mutable_agent_decision()->set_agent_decision_type(
        agent::AgentDecisionType::IGNORE);
  }
}

void AgentLongitudinalDecider::FilterReverseAgents() {
  const auto& agent_manager =
      session_->environmental_model().get_agent_manager();
  auto mutable_agent_manager =
      session_->mutable_environmental_model()->mutable_agent_manager();
  if (agent_manager == nullptr || mutable_agent_manager == nullptr) {
    LOG_ERROR("[FilterReverseAgents] agent manager is empty");
    return;
  }

  const auto& agents = agent_manager->GetAllCurrentAgents();
  if (agents.empty()) {
    return;
  }

  const auto& virtual_lane_manager =
      session_->environmental_model().get_virtual_lane_manager();
  const auto& current_lane = session_->environmental_model()
                                 .get_virtual_lane_manager()
                                 ->get_current_lane();
  if (current_lane == nullptr) {
    return;
  }
  const auto& current_lane_coord = current_lane->get_lane_frenet_coord();
  if (current_lane_coord == nullptr) {
    return;
  }

  const auto& ego_state_manager =
      session_->environmental_model().get_ego_state_manager();
  if (ego_state_manager == nullptr) {
    return;
  }
  const auto planning_init_point = ego_state_manager->planning_init_point();
  const double v_ego = ego_state_manager->ego_v();
  const double road_curvature_radius = CalculateRoadCurvature(v_ego);
  JSON_DEBUG_VALUE("road_curvature_radius", road_curvature_radius)

  double planning_init_point_s = 0.0;
  double planning_init_point_l = 0.0;
  current_lane_coord->XYToSL(planning_init_point.x, planning_init_point.y,
                             &planning_init_point_s, &planning_init_point_l);

  for (const auto agent : agents) {
    if (agent == nullptr) {
      continue;
    }

    if (agent->type() == agent::AgentType::VIRTUAL) {
      continue;
    }

    // ignored agent
    auto* mutable_agent =
        mutable_agent_manager->mutable_agent(agent->agent_id());
    if (mutable_agent == nullptr ||
        mutable_agent->agent_decision().agent_decision_type() ==
            agent::AgentDecisionType::IGNORE) {
      continue;
    }

    const auto agent_box_center = agent->box().center();

    // only consider reverse agent
    if (!IsReverseAgent(agent.get(), current_lane)) {
      continue;
    }
    mutable_agent->set_is_reverse(true);

    const auto& corners = agent->box().GetAllCorners();
    double agent_max_l = std::numeric_limits<double>::lowest();
    double agent_min_l = std::numeric_limits<double>::max();
    double agent_max_s = std::numeric_limits<double>::lowest();
    double agent_min_s = std::numeric_limits<double>::max();
    for (const auto& corner : corners) {
      double project_l = 0.0;
      double projrct_s = 0.0;
      if (!current_lane_coord->XYToSL(corner.x(), corner.y(), &projrct_s,
                                      &project_l)) {
        continue;
      }
      agent_max_l = std::fmax(agent_max_l, project_l);
      agent_min_l = std::fmin(agent_min_l, project_l);
      agent_max_s = std::fmax(agent_max_s, projrct_s);
      agent_min_s = std::fmin(agent_min_s, projrct_s);
    }

    // calculate agent center s and l
    double agent_s_in_ego_lane = 0.0;
    double agent_l_in_ego_lane = 0.0;
    current_lane_coord->XYToSL(agent_box_center.x(), agent_box_center.y(),
                               &agent_s_in_ego_lane, &agent_l_in_ego_lane);

    // calculate agent end point s and l
    double agent_end_point_s = 0.0;
    double agent_end_point_l = 0.0;
    const bool has_trajectory = !agent->trajectories().empty() &&
                                !agent->trajectories().front().empty();
    double end_point_heading_from_start = 0.0;
    if (has_trajectory) {
      const auto& end_point = agent->trajectories().front().back();
      current_lane_coord->XYToSL(end_point.x(), end_point.y(),
                                 &agent_end_point_s, &agent_end_point_l);
      end_point_heading_from_start = Vec2d(end_point.x() - agent_box_center.x(),
                                           end_point.y() - agent_box_center.y())
                                         .Angle();
    }

    double half_lane_width =
        0.5 * current_lane->width_by_s(agent_s_in_ego_lane);
    const auto current_lane_path_point =
        current_lane_coord->GetPathPointByS(agent_s_in_ego_lane);
    const double mathed_point_heading = current_lane_path_point.theta();

    // 1. ignore low speed agent
    if (agent->speed() < kLowSpeedThreshold) {
      if (IsIgnoredLowSpeedReverseAgent(
              *agent, planning_init_point_s, planning_init_point.v, agent_max_s,
              agent_min_s, agent_max_l, agent_min_l)) {
        mutable_agent->mutable_agent_decision()->set_agent_decision_type(
            agent::AgentDecisionType::IGNORE);
      }
      continue;
    }

    // 2. not ignore large heading diff cross agent
    const double crossing_dist_thr =
        planning_init_point_s +
        std::fmax(kMinFilterDistance, planning_init_point.v * kLongitudalTtc);

    if (has_trajectory && agent_end_point_l * agent_l_in_ego_lane < kEpsilon &&
        agent_s_in_ego_lane < crossing_dist_thr) {
      const double heading_diff = std::fabs(planning_math::NormalizeAngle(
          end_point_heading_from_start - (mathed_point_heading + M_PI)));
      // agent's trajectory cross the planned path,and the heading diff is large
      constexpr double kLargeCrossHeadingDiff = 30.0 / 57.3;
      if (heading_diff > kLargeCrossHeadingDiff) {
        continue;
      }
    }

    // 3. ignore far reverse agent by vel
    const double considered_lon_distance =
        std::fmax(planning_init_point_s + kMinFilterDistance,
                  std::fmin(planning_init_point_s +
                                planning_init_point.v * kLongitudalTtc,
                            current_lane_coord->Length()));
    if (considered_lon_distance < agent_s_in_ego_lane) {
      mutable_agent->mutable_agent_decision()->set_agent_decision_type(
          agent::AgentDecisionType::IGNORE);
      continue;
    }

    // // 3*. HACK: ignore reverse agent in curvature
    // if (agent_s_in_ego_lane > kconsideredLonDistanceInCurve &&
    //     road_curvature_radius < 2000.0) {
    //   mutable_agent->mutable_agent_decision()->set_agent_decision_type(
    //       agent::AgentDecisionType::IGNORE);
    //   continue;
    // }

    // 3**. not ignore intersection_length > 0.3 * lane_width
    const double intersection_length = CalculateIntersectionLength(
        agent_min_l, agent_max_l, -half_lane_width, half_lane_width);
    if (intersection_length > kIntersectionFactor * half_lane_width * 2) {
      continue;
    }

    // 4. ignore agent_l and agent_l_end > LateralIgnoreDist(15m)
    if (has_trajectory) {
      if ((agent_l_in_ego_lane > kHightSpeedLateralIgnoreDist &&
           agent_end_point_l > kHightSpeedLateralIgnoreDist) ||
          (agent_l_in_ego_lane < -kHightSpeedLateralIgnoreDist &&
           agent_end_point_l < -kHightSpeedLateralIgnoreDist)) {
        mutable_agent->mutable_agent_decision()->set_agent_decision_type(
            agent::AgentDecisionType::IGNORE);
        continue;
      }
    }

    // 5. ignore agent_min_l > half_lane_width
    if ((agent_min_l > half_lane_width - kCrossLaneThreshold) ||
        (agent_max_l < -half_lane_width + kCrossLaneThreshold)) {
      mutable_agent->mutable_agent_decision()->set_agent_decision_type(
          agent::AgentDecisionType::IGNORE);
    }
  }
}

bool AgentLongitudinalDecider::IsReverseAgent(
    const agent::Agent* agent,
    const std::shared_ptr<VirtualLane> ego_lane) const {
  double agent_s = 0.0;
  double agent_l = 0.0;
  const auto agent_box_center = agent->box().center();
  const auto& ego_lane_coord = ego_lane->get_lane_frenet_coord();
  if (ego_lane_coord == nullptr) {
    return false;
  }
  ego_lane_coord->XYToSL(agent_box_center.x(), agent_box_center.y(), &agent_s,
                         &agent_l);
  const auto matched_path_point = ego_lane_coord->GetPathPointByS(agent_s);
  const double heading_diff = std::fabs(planning_math::NormalizeAngle(
      matched_path_point.theta() - agent->box().heading()));
  const bool is_perception_reverse = heading_diff > kReverseHeadingThreshold;
  bool is_prediction_reverse = false;

  if (!agent->trajectories().empty() &&
      !agent->trajectories().front().empty()) {
    const auto& end_point = agent->trajectories().front().back();
    double end_s = 0.0;
    double end_l = 0.0;
    ego_lane_coord->XYToSL(end_point.x(), end_point.y(), &end_s, &end_l);
    is_prediction_reverse = end_s < agent_s;
  }
  return is_perception_reverse && is_prediction_reverse;
}

bool AgentLongitudinalDecider::IsIgnoredLowSpeedReverseAgent(
    const agent::Agent& agent, const double init_point_s,
    const double init_point_spd, const double agent_max_s,
    const double agent_min_s, const double agent_max_l,
    const double agent_min_l) const {
  if (agent.speed() > kLowSpeedThreshold) {
    return false;
  }
  constexpr double KEgoTtc = 6.0;
  const double low_speed_lon_ignore_dist =
      std::max(init_point_s + kMinFilterDistance,
               init_point_s + init_point_spd * KEgoTtc);
  if (agent_max_l < -kLowSpeedLateralIgnoreDist ||
      agent_min_l > kLowSpeedLateralIgnoreDist ||
      agent_min_s > low_speed_lon_ignore_dist) {
    return true;
  }
  return false;
}

double AgentLongitudinalDecider::CalculateRoadCurvature(const double v_ego) {
  double preview_x = kDistanceCurvature + kTimeCurvature * v_ego;
  const auto& reference_path_ptr = session_->planning_context()
                                       .lane_change_decider_output()
                                       .coarse_planning_info.reference_path;
  const auto& frenet_ego_state = reference_path_ptr->get_frenet_ego_state();
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
  return road_radius;
}

double AgentLongitudinalDecider::GetFilterUltraDistanceWithEgoVel(
    const double ego_vel) const {
  std::array<double, 3> speed{100.0, 110.0, 120.0};
  std::array<double, 3> max_distance{150.0, 170.0, 200.0};
  const double mps_to_kph = 3.6;
  double distance = interp(ego_vel * mps_to_kph, speed, max_distance);  // kmph
  return distance;
}

double AgentLongitudinalDecider::CalculateIntersectionLength(
    const double start_1, const double end_1, const double start_2,
    const double end_2) const {
  double a1 = start_1;
  double b1 = end_1;
  double a2 = start_2;
  double b2 = end_2;

  if (a1 > b1) {
    std::swap(a1, b1);
  }
  if (a2 > b2) {
    std::swap(a2, b2);
  }
  if (b1 < a2 || b2 < a1) {
    return 0.0;
  }

  double lower = std::max(a1, a2);
  double upper = std::min(b1, b2);
  return upper - lower;
}

}  // namespace planning