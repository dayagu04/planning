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

namespace {

constexpr double kEpsilon = 1.0e-4;
// constexpr double kMaxHeadingDiff = 1.04;  // 60 degree
// constexpr double kMinLowSpeedTrajLength = 3.0;
constexpr double kIgnoreSpeedDiffThd = 5.0 / 3.6;
constexpr double kAgentFrontEdgeSDiffThd = -0.2;
constexpr double kHalf = 0.5;
constexpr double kKphToMps = 0.2778;
constexpr double kMpsToKph = 3.6;
constexpr double kLaneWidth = 3.75;
constexpr double kLargeAgentLengthM = 8.0;

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

  // FilterReverseAgents();

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
  if (is_in_lane_change) {
    return;
  }

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

  for (const auto* agent : agents) {
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
  // DeciderCutOutAgent();
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

  // vehicle param
  const auto& vehicle_param =
      VehicleConfigurationContext::Instance()->get_vehicle_param();
  const double vehicle_length = vehicle_param.length;
  const double rear_axle_to_rear_edge =
      vehicle_length - vehicle_param.front_edge_to_rear_axle;
  const double rear_axle_to_front_edge = vehicle_param.front_edge_to_rear_axle;
  const double rear_axle_to_center = vehicle_param.rear_axle_to_center;
  const double ego_rear_edge_s = ego_s - rear_axle_to_rear_edge;
  const double ego_front_edge_s = ego_s + rear_axle_to_front_edge;
  const double ego_center_s = ego_s + rear_axle_to_center;

  for (const auto* agent : agents) {
    if (agent == nullptr) {
      continue;
    }
    if (agent->is_tfl_virtual_obs()) {
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
    if (ego_front_edge_s > agent_s + half_length) {
      is_no_need_expand_agent = true;
    }

    // filter consider rear agent
    if (IsConsiderBackObs(ego_lane_coord, planning_init_point, agent,
                          ego_front_edge_s, ego_rear_edge_s, ego_center_s,
                          agent_s, agent_l)) {
      continue;
    }

    // rear agents
    // TODO: only insert ego_right_rear_node_id in target_lane_rear_agents
    //       if want consider more rear agent, insert there
    int64_t target_lane_rear_node_id = -1;
    std::unordered_set<int32_t> target_lane_rear_agents;
    if (is_in_lane_change_execution) {
      if (lc_request_direction == LEFT_CHANGE) {
        target_lane_rear_node_id = dynamic_world_->ego_left_rear_node_id();
      } else if (lc_request_direction == RIGHT_CHANGE) {
        target_lane_rear_node_id = dynamic_world_->ego_right_rear_node_id();
      }
    } else if (is_lane_change_complete) {
      target_lane_rear_node_id = dynamic_world_->ego_rear_node_id();
    }
    if (target_lane_rear_node_id != -1) {
      auto* target_lane_rear_node =
          dynamic_world_->GetNode(target_lane_rear_node_id);
      if (target_lane_rear_node != nullptr) {
        target_lane_rear_agents.insert(target_lane_rear_node->node_agent_id());
      }
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
    const double ego_front_s, const double ego_back_s,
    const double ego_center_s, const double agent_s,
    const double agent_l) const {
  const auto& vehicle_param =
      VehicleConfigurationContext::Instance()->get_vehicle_param();
  const double k_agent_ignore_only_position_diff_thd =
      -vehicle_param.length / 2.0;

  auto& mutable_agent_manager =
      session_->mutable_environmental_model()->mutable_agent_manager();
  // const double agent_half_length = agent->length() * 0.5;
  const double agent_speed_diff = agent->speed() - init_point.v;
  double front_corner_s = -std::numeric_limits<double>::max();
  // double front_corner_l = -std::numeric_limits<double>::max();
  std::vector<planning_math::Vec2d> corners;
  agent->box().GetAllCorners(&corners);
  for (const auto& corner : corners) {
    double corner_s = 0.0;
    double corner_l = 0.0;
    if (!planned_path->XYToSL(corner.x(), corner.y(), &corner_s, &corner_l)) {
      continue;
    }
    if (corner_s > front_corner_s) {
      front_corner_s = corner_s;
    }
  }
  if (std::fabs(front_corner_s) < -1.0e6) {
    return true;
  }

  // 0.ego is slow(<15kph) and agent front s < ego center s
  constexpr double kEgoSlowSpeedKph = 15.0;
  const bool ignore_rear_agent_when_ego_slow =
      (init_point.v * kMpsToKph < kEgoSlowSpeedKph) &&
      (front_corner_s < ego_center_s);

  // 1.agent is not cut-in and rear of ego
  const bool filter_rear_no_cut_in_agent = FilterRearNoCutInAgent(
      planned_path, init_point, ego_front_s, front_corner_s, agent);

  const double front_edge_s_diff = front_corner_s - ego_front_s;
  if ((agent_speed_diff < kIgnoreSpeedDiffThd &&
       front_edge_s_diff < kAgentFrontEdgeSDiffThd) ||
      front_edge_s_diff < k_agent_ignore_only_position_diff_thd ||
      ignore_rear_agent_when_ego_slow || filter_rear_no_cut_in_agent) {
    auto* mutable_agent =
        mutable_agent_manager->mutable_agent(agent->agent_id());
    if (mutable_agent == nullptr) {
      return true;
    }
    return false;
  }
  return true;
}

bool AgentLongitudinalDecider::FilterRearNoCutInAgent(
    const std::shared_ptr<KDPath> planned_path,
    const PlanningInitPoint& init_point, const double ego_front_s,
    const double agent_front_s, const agent::Agent* ptr_agent) const {
  // 1.lane keeping
  // 2.not cut-in
  // 3.agent speed is fast(>40kph)
  constexpr double kAgentLowerSpeedKph = 40.0;
  const auto& virtual_lane_manager =
      session_->environmental_model().get_virtual_lane_manager();
  const auto& ego_lane = virtual_lane_manager->get_current_lane();
  const auto& lateral_behavior_planner_output =
      session_->planning_context().lateral_behavior_planner_output();
  const auto lane_change_status = lateral_behavior_planner_output.lc_status;
  const auto lane_change_request = lateral_behavior_planner_output.lc_request;
  const bool is_lane_keeping = lane_change_request == "none" ||
                               lane_change_status == "none" ||
                               lane_change_status == "left_lane_change_wait" ||
                               lane_change_status == "right_lane_change_wait";
  if (nullptr == virtual_lane_manager || nullptr == ego_lane ||
      !is_lane_keeping || nullptr == ptr_agent ||
      ptr_agent->is_prediction_cutin() ||
      ptr_agent->speed() * kMpsToKph < kAgentLowerSpeedKph ||
      ptr_agent->trajectories().empty()) {
    return false;
  }

  // 4.agent is not in ego lane,but must at neighbor lane
  double nearest_s = 0.0;
  double nearest_l = 0.0;
  const auto& ptr_obj_lane = virtual_lane_manager->GetNearestLane(
      {ptr_agent->x(), ptr_agent->y()}, &nearest_s, &nearest_l);
  if (nullptr == ptr_obj_lane ||
      ptr_obj_lane->get_virtual_id() == ego_lane->get_virtual_id()) {
    return false;
  }

  // 5.agent and ego lane is parallel
  // 6.agent must be rear
  const bool is_neighbor = std::abs(ptr_obj_lane->get_virtual_id() -
                                    ego_lane->get_virtual_id()) == 1;
  // const bool is_parallel =
  // IsParallelToEgoLane(ptr_obj_lane->get_virtual_id());
  const bool is_rear_agent = ego_front_s > agent_front_s;
  if (!is_rear_agent || !is_neighbor) {
    return false;
  }

  // 7.small heading diff
  double agent_s = 0.0;
  double agent_l = 0.0;
  const auto& ego_lane_coord = ego_lane->get_lane_frenet_coord();
  if (ego_lane_coord == nullptr) {
    return false;
  }
  ego_lane_coord->XYToSL(ptr_agent->x(), ptr_agent->y(), &agent_s, &agent_l);
  if (agent_s < 0.0 || agent_s > ego_lane_coord->Length()) {
    return false;
  }
  const auto ego_lane_point = ego_lane_coord->GetPathPointByS(agent_s);
  const double heading_diff_with_lane = planning_math::NormalizeAngle(
      ptr_agent->theta() - ego_lane_point.theta());
  // const double heading_diff_with_ego = planning_math::NormalizeAngle(
  //     ptr_agent->theta() - init_point.heading_angle);
  constexpr double kSmallHeadingDiff = 3 / 57.3;
  const bool small_heading_diff = agent_l < kEpsilon
                                      ? (heading_diff_with_lane < kSmallHeadingDiff/* &&
                                         heading_diff_with_ego < kSmallHeadingDiff*/)
                                      : (heading_diff_with_lane > -kSmallHeadingDiff /*&&
                                         heading_diff_with_ego > -kSmallHeadingDiff*/);
  if (!small_heading_diff) {
    return false;
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
                                 relative_time + 2.0};
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
    if (has_overlap) {
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
  const double ego_front_edge_s = ego_s + rear_axle_to_front_edge;

  double filter_ultra_distance = kFilterUltraDistanceHighThd;
  filter_ultra_distance =
      GetFilterUltraDistanceWithEgoVel(planning_init_point.v);

  for (const auto* agent : agents) {
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

// void AgentLongitudinalDecider::FilterReverseAgents() {}

double AgentLongitudinalDecider::GetFilterUltraDistanceWithEgoVel(
    const double ego_vel) const {
  std::array<double, 3> speed{100.0, 110.0, 120.0};
  std::array<double, 3> max_distance{150.0, 170.0, 200.0};
  const double mps_to_kph = 3.6;
  double distance = interp(ego_vel * mps_to_kph, speed, max_distance);  // kmph
  return distance;
}

}  // namespace planning