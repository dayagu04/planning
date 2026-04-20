
#include "agent_longitudinal_decider.h"

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

#include "agent/agent.h"
#include "agent_trajectory_calculator.h"
#include "common_platform_type_soc.h"
#include "config/basic_type.h"
#include "debug_info_log.h"
#include "environmental_model.h"
#include "interface/src/c/common_c.h"
#include "log.h"
#include "planning_context.h"
#include "vehicle_config_context.h"
#include "virtual_lane.h"

namespace planning {
using namespace planning_math;

namespace {

constexpr double kEpsilon = 1.0e-4;
constexpr double kIgnoreSpeedDiffThd = 2.5;
constexpr double kAgentFrontEdgeSDiffThd = -0.2;
constexpr double kHalf = 0.5;
constexpr double kKphToMps = 0.2778;
constexpr double kMpsToKph = 3.6;
constexpr double kLargeSpeedDiff = 5.56;

constexpr double kBayesCutInThreshold = 0.5;
constexpr double kBayesCutOutThreshold = 0.5;
constexpr double kBayesCutinPrior = 0.15;
constexpr double kBayesCutoutPrior = 0.15;
constexpr double kBayesNormalPrior = 0.70;
constexpr double kBayesSmoothAlpha = 0.3;
constexpr int kMinHistoryForBayes = 5;
constexpr double kBayesHistWeight = 0.75;
constexpr double kBayesPredWeight = 0.25;

constexpr int kPredFrames = 6;
constexpr double kPredFrameInterval = 0.2;

constexpr double kFilterUltraDistanceHighThd = 300.0;

constexpr double kReverseHeadingThreshold = 2.09;
constexpr double kLowSpeedThreshold = 3.0;
constexpr double kMinFilterDistance = 30.0;
constexpr double kLowSpeedLateralIgnoreDist = 10.0;
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
constexpr double kLowSpeedThresholdMps = 2.0;
constexpr double kSuppressionLateralSpeedThresholdMps = 0.1;
constexpr double kSuppressionLateralPenetrationM = 0.3;
constexpr size_t kMaxHistorySize = 5;
constexpr double kCutInLateralDistanceRange = 5.625;
constexpr double kConfluenceCutInLateralDistanceRange = 9.375;
constexpr double kMinCutInHeadway = 5.0;
constexpr double kMinCutInLateralDistance = 1.0;
constexpr double kLongitudinalTtcLowerThreshold = 1.5;
constexpr double kLongitudinalTtcUpperThreshold = 3.0;

constexpr std::array<double, 2> kBayesNormalMu = {0.0, 3.75};
constexpr std::array<double, 2> kBayesNormalSigma = {0.20, 1.0};

constexpr std::array<double, 2> kBayesCutinMu = {-0.35, 2.275};
constexpr std::array<double, 2> kBayesCutinSigma = {0.25, 0.4};

constexpr std::array<double, 2> kBayesCutOutMu = {0.35, 2.275};
constexpr std::array<double, 2> kBayesCutOutSigma = {0.25, 0.4};

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
  if (config_builder != nullptr) {
    reverse_filter_config_ = config_builder->cast<AgentLongitudinalDeciderConfig>();
  }
}

bool AgentLongitudinalDecider::Reset() { return false; }

bool AgentLongitudinalDecider::Execute() {
  ILOG_INFO << "=======AgentLongitudinalDecider=======";

  if (!PreCheck()) {
    ILOG_DEBUG << "PreCheck failed";
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
  ego_frenet_boundary_ = session_->environmental_model()
                             .get_reference_path_manager()
                             ->get_reference_path_by_current_lane()
                             ->get_ego_frenet_boundary();
}

bool AgentLongitudinalDecider::Update() {
  FilterRearAgents();

  FilterUltradistantObs();

  // Scene-based reverse agent filtering
  if (session_->is_hpp_scene()) {
    FilterReverseAgentsHpp();
  } else {
    // Clear HPP hysteresis state on scene switch to prevent stale entries from
    // polluting a future HPP session (e.g. via agent-id reuse).
    if (!hpp_reverse_ignore_state_.empty()) {
      hpp_reverse_want_ignore_count_.clear();
      hpp_reverse_want_keep_count_.clear();
      hpp_reverse_ignore_state_.clear();
    }
    FilterReverseAgents();
  }

  DeciderCutInAndOutAgents();

  AgentTrajectoryCalculator agent_trajectory_calculator(session_);
  agent_trajectory_calculator.Process();

  UpdateAgentTable();

  if (crossing_agent_decider_ != nullptr && !session_->is_rads_scene()) {
    crossing_agent_decider_->Execute();
  }

  return true;
}

void AgentLongitudinalDecider::DeciderCutInAndOutAgents() {
  const auto& lane_change_decider_output =
      session_->planning_context().lane_change_decider_output();
  const auto lane_change_state = lane_change_decider_output.curr_state;
  const bool is_in_lane_change_execution =
      lane_change_state == StateMachineLaneChangeStatus::kLaneChangeExecution ||
      lane_change_state == StateMachineLaneChangeStatus::kLaneChangeComplete ||
      lane_change_state == StateMachineLaneChangeStatus::kLaneChangeHold;

  current_agent_ids_.clear();
  processed_cut_in_agent_ids_.clear();
  processed_cut_out_agent_ids_.clear();
  const auto* agent_manager = dynamic_world_->agent_manager();
  auto* mutable_agent_manager = dynamic_world_->mutable_agent_manager();

  if (agent_manager == nullptr || mutable_agent_manager == nullptr) {
    ILOG_ERROR << "agent_manager is empty";
    return;
  }

  const auto& agents = agent_manager->GetAllCurrentAgents();
  if (agents.empty()) {
    agent_history_map_.clear();
    return;
  }

  const auto& vehicle_param =
      VehicleConfigurationContext::Instance()->get_vehicle_param();
  const double ego_half_length = vehicle_param.length * kHalf;
  const double ego_half_width = vehicle_param.width * kHalf + 0.15;
  const auto& route_info = session_->environmental_model().get_route_info();
  const auto& ego_lane_road_right_output =
      session_->planning_context().ego_lane_road_right_decider_output();
  bool is_confluence_area = false;
  if (route_info != nullptr) {
    const auto& ego_state =
        session_->environmental_model().get_ego_state_manager();
    const double ego_v = ego_state->ego_v();
    const double dis_threshold = ego_v * 10.0;
    const auto& route_info_output = route_info->get_route_info_output();
    bool is_closing_split =
        route_info_output.mlc_decider_scene_type_info.mlc_scene_type ==
            SPLIT_SCENE &&
        route_info_output.mlc_decider_scene_type_info
                .dis_to_link_topo_change_point < dis_threshold;
    bool is_closing_merge =
        route_info_output.mlc_decider_scene_type_info.mlc_scene_type ==
            MERGE_SCENE &&
        route_info_output.mlc_decider_scene_type_info
                .dis_to_link_topo_change_point < dis_threshold;

    if (is_closing_merge || is_closing_split) {
      is_confluence_area = true;
    }
  }

  if (ego_lane_road_right_output.is_merge_region ||
      ego_lane_road_right_output.is_split_region) {
    is_confluence_area = true;
  }

  const double cut_in_lateral_distance_range =
      is_confluence_area ? kConfluenceCutInLateralDistanceRange
                         : kCutInLateralDistanceRange;

  const auto& init_point = ego_state_manager_->planning_init_point();
  const double ego_speed_mps = init_point.v;

  auto* agent_longitudinal_decider_output =
      session_->mutable_planning_context()
          ->mutable_agent_longitudinal_decider_output();
  if (agent_longitudinal_decider_output != nullptr) {
    agent_longitudinal_decider_output->closest_cutin_ttc_info.agent_id = -1;
    agent_longitudinal_decider_output->closest_cutin_ttc_info.ttc = -1.0;
  }

  const auto& lat_lon_joint_planner_output =
      session_->planning_context().lat_lon_joint_planner_decider_output();
  const auto& prediction_cut_in_ids =
      lat_lon_joint_planner_output.GetDangerObstacleIds();
  for (const auto agent_id : prediction_cut_in_ids) {
    auto mutable_agent = mutable_agent_manager->mutable_agent(agent_id);
    if (mutable_agent == nullptr) {
      continue;
    }
    mutable_agent->set_is_prediction_cutin(true);
  }

  for (const auto agent : agents) {
    if (nullptr == agent) {
      continue;
    }
    if (!(agent->fusion_source() & OBSTACLE_SOURCE_CAMERA)) {
      continue;
    }
    const int32_t agent_id = agent->agent_id();
    current_agent_ids_.insert(agent_id);

    ProcessCutInCutOutAgent(
        is_in_lane_change_execution, *agent, ego_speed_mps, ego_half_length,
        ego_half_width, init_point, cut_in_lateral_distance_range,
        mutable_agent_manager, agent_longitudinal_decider_output);
  }

  for (auto it = agent_history_map_.begin(); it != agent_history_map_.end();) {
    if (current_agent_ids_.find(it->first) == current_agent_ids_.end()) {
      it = agent_history_map_.erase(it);
    } else {
      ++it;
    }
  }
}

void AgentLongitudinalDecider::ProcessCutInCutOutAgent(
    const bool is_in_lane_change, const agent::Agent& agent,
    const double ego_speed_mps, const double ego_half_length,
    const double ego_half_width, const PlanningInitPoint init_point,
    const double lateral_distance_range,
    agent::AgentManager* const mutable_agent_manager,
    AgentLongitudinalDeciderOutput* output) {
  if (nullptr == mutable_agent_manager) {
    return;
  }

  const auto& ego_lane = virtual_lane_manager_->get_current_lane();
  if (ego_lane == nullptr) {
    return;
  }
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

  double agent_s = 0.0;
  double agent_l = 0.0;
  if (!ego_lane_coord->XYToSL(agent.x(), agent.y(), &agent_s, &agent_l)) {
    return;
  }

  const auto agent_matched_path_point =
      ego_lane_coord->GetPathPointByS(agent_s);
  const double agent_matched_lane_theta = agent_matched_path_point.theta();
  const double agent_relative_theta =
      planning_math::NormalizeAngle(agent.theta() - agent_matched_lane_theta);
  const double object_s_speed_mps =
      agent.speed() * std::cos(agent_relative_theta);
  const double object_l_speed_mps =
      agent.speed() * std::sin(agent_relative_theta);

  double min_s = std::numeric_limits<double>::max();
  double max_s = std::numeric_limits<double>::lowest();
  double min_l = std::numeric_limits<double>::max();
  double max_l = std::numeric_limits<double>::lowest();
  CalculateAgentSLBoundary(ego_lane_coord, agent, &min_s, &max_s, &min_l,
                           &max_l);

  double small_lateral_distance = std::numeric_limits<double>::max();
  double small_lateral_distance_with_ego_l = std::numeric_limits<double>::max();
  double large_lateral_distance = std::numeric_limits<double>::max();
  CalculateAgentLateralDistance(
      object_l_speed_mps, min_l, max_l, max_s, ego_speed_mps, ego_s, ego_l,
      &small_lateral_distance, &small_lateral_distance_with_ego_l,
      &large_lateral_distance);

  auto* mutable_agent = mutable_agent_manager->mutable_agent(agent.agent_id());
  if (mutable_agent != nullptr) {
    mutable_agent->set_d_path(
        std::fmax(std::fabs(small_lateral_distance) - ego_half_width, 0.0));
    mutable_agent->set_d_rel(std::fmax(min_s - ego_s - ego_half_length, 0.0));
  }

  if (agent.is_static()) {
    return;
  }

  if (agent_s <= ego_s ||
      (agent_s - ego_s) >= ego_speed_mps * kMinCutInHeadway) {
    return;
  }

  if (std::fabs(agent_l) < kMinCutInLateralDistance) {
    return;
  }

  if (is_in_lane_change) {
    return;
  }

  if (std::fabs(agent_l) > lateral_distance_range) {
    return;
  }

  const int32_t agent_id = agent.agent_id();
  if (nullptr == mutable_agent) {
    return;
  }

  AgentHistoryState current_state;
  UpdateAndGetAgentState(agent, init_point, ego_lane_coord, current_state);

  BayesFeatures features =
      ExtractBayesFeatures(agent_id, agent, ego_lane_coord);

  if (!features.valid) {
    return;
  }

  const int dim = static_cast<int>(features.norm_l_dot.size());
  const int pred_frames_count = static_cast<int>(features.pred_norm_vy.size());
  if (pred_frames_count < kPredFrames - 1) {
    return;
  }
  double log_L_hist_cutin = 0.0;
  double log_L_hist_cutout = 0.0;
  double log_L_hist_normal = 0.0;

  for (int i = 0; i < dim; ++i) {
    const std::array<double, 2> x_hist = {features.norm_l_dot[i],
                                          features.hist_lateral_dist[i]};
    for (int j = 0; j < 2; ++j) {
      const double dc_cutin = x_hist[j] - kBayesCutinMu[j];
      const double dc_cutout = x_hist[j] - kBayesCutOutMu[j];
      const double dn = x_hist[j] - kBayesNormalMu[j];
      log_L_hist_cutin +=
          -0.5 * dc_cutin * dc_cutin / (kBayesCutinSigma[j] * kBayesCutinSigma[j]) -
          0.5 *
              std::log(2.0 * M_PI * kBayesCutinSigma[j] * kBayesCutinSigma[j]);
      log_L_hist_cutout +=
          -0.5 * dc_cutout * dc_cutout / (kBayesCutOutSigma[j] * kBayesCutOutSigma[j]) -
          0.5 * std::log(2.0 * M_PI * kBayesCutOutSigma[j] *
                         kBayesCutOutSigma[j]);
      log_L_hist_normal +=
          -0.5 * dn * dn / (kBayesNormalSigma[j] * kBayesNormalSigma[j]) -
          0.5 * std::log(2.0 * M_PI * kBayesNormalSigma[j] *
                         kBayesNormalSigma[j]);
    }
  }

  double log_L_pred_cutin = 0.0;
  double log_L_pred_cutout = 0.0;
  double log_L_pred_normal = 0.0;

  for (int i = 0; i < pred_frames_count; ++i) {
    const std::array<double, 2> x_pred = {features.pred_norm_vy[i],
                                          features.pred_lateral_dist[i]};
    for (int j = 0; j < 2; ++j) {
      const double dc_cutin = x_pred[j] - kBayesCutinMu[j];
      const double dc_cutout = x_pred[j] - kBayesCutOutMu[j];
      const double dn = x_pred[j] - kBayesNormalMu[j];
      log_L_pred_cutin +=
          -0.5 * dc_cutin * dc_cutin /
              (kBayesCutinSigma[j] * kBayesCutinSigma[j]) -
          0.5 * std::log(2.0 * M_PI * kBayesCutinSigma[j] *
                         kBayesCutinSigma[j]);
      log_L_pred_cutout +=
          -0.5 * dc_cutout * dc_cutout /
              (kBayesCutOutSigma[j] * kBayesCutOutSigma[j]) -
          0.5 * std::log(2.0 * M_PI * kBayesCutOutSigma[j] *
                         kBayesCutOutSigma[j]);
      log_L_pred_normal +=
          -0.5 * dn * dn / (kBayesNormalSigma[j] * kBayesNormalSigma[j]) -
          0.5 * std::log(2.0 * M_PI * kBayesNormalSigma[j] *
                         kBayesNormalSigma[j]);
    }
  }

  double cutin_posterior = 0.0;
  double cutout_posterior = 0.0;
  UpdateBayesianPosteriors(agent_id, log_L_hist_cutin, log_L_hist_cutout,
                           log_L_hist_normal, log_L_pred_cutin,
                           log_L_pred_cutout, log_L_pred_normal,
                           &cutin_posterior, &cutout_posterior);

  processed_cut_in_agent_ids_.insert(agent_id);
  processed_cut_out_agent_ids_.insert(agent_id);

  if (cutin_posterior > kBayesCutInThreshold) {
    cut_in_agent_count_[agent_id] = 1;
  } else {
    cut_in_agent_count_[agent_id] = 0;
  }

  if (cutout_posterior > kBayesCutOutThreshold) {
    cut_out_agent_count_[agent_id] = 1;
  } else {
    cut_out_agent_count_[agent_id] = 0;
  }

  if (cut_in_agent_count_[agent_id] > 0) {
    mutable_agent->set_rule_base_cutin_score(cutin_posterior);
    mutable_agent->set_is_rule_base_cutin(true);

    double longitudinal_ttc = std::numeric_limits<double>::max();
    const double longitudinal_distance = min_s - ego_s - ego_half_length;
    const double relative_speed = object_s_speed_mps - ego_speed_mps;

    if (longitudinal_distance > 0.0 && relative_speed < 0.0) {
      longitudinal_ttc = longitudinal_distance / std::fabs(relative_speed);
    }

    if (output != nullptr &&
        longitudinal_ttc <= kLongitudinalTtcUpperThreshold &&
        longitudinal_ttc >= kLongitudinalTtcLowerThreshold) {
      if (output->closest_cutin_ttc_info.agent_id == -1 ||
          longitudinal_ttc < output->closest_cutin_ttc_info.ttc) {
        output->closest_cutin_ttc_info.agent_id = agent_id;
        output->closest_cutin_ttc_info.ttc = longitudinal_ttc;
      }
    }
  }

  if (cutout_posterior > kBayesCutOutThreshold) {
    mutable_agent->set_is_cutout(true);
  }
}

void AgentLongitudinalDecider::UpdateAndGetAgentState(
    const agent::Agent& agent, const PlanningInitPoint& init_point,
    const std::shared_ptr<planning_math::KDPath>& ego_lane_coord,
    AgentHistoryState& current_state) {
  const int32_t agent_id = agent.agent_id();
  current_state.timestamp = agent.timestamp_s();
  current_state.x = agent.x();
  current_state.y = agent.y();

  double current_s = 0.0, current_l = 0.0;
  if (ego_lane_coord->XYToSL(current_state.x, current_state.y, &current_s,
                             &current_l)) {
    current_state.l = current_l;
    auto it = agent_history_map_.find(agent_id);
    if (it != agent_history_map_.end() && !it->second.empty()) {
      const auto& prev_state = it->second.back();
      double dt = current_state.timestamp - prev_state.timestamp;
      if (dt > 1e-3) {
        current_state.l_dot = (current_state.l - prev_state.l) / dt;
      } else {
        current_state.l_dot = prev_state.l_dot;
      }
    } else {
      double current_s = 0.0, current_l = 0.0;
      ego_lane_coord->XYToSL(current_state.x, current_state.y, &current_s,
                             &current_l);
      const auto agent_matched_path_point =
          ego_lane_coord->GetPathPointByS(current_s);
      const double agent_relative_theta = planning_math::NormalizeAngle(
          agent.theta() - agent_matched_path_point.theta());
      current_state.l_dot = agent.speed() * std::sin(agent_relative_theta);
    }
  }

  agent_history_map_[agent_id].push_back(current_state);

  if (agent_history_map_[agent_id].size() > kMaxHistorySize) {
    agent_history_map_[agent_id].pop_front();
  }
}

BayesFeatures AgentLongitudinalDecider::ExtractBayesFeatures(
    int32_t agent_id, const agent::Agent& agent,
    const std::shared_ptr<planning_math::KDPath>& ego_lane_coord) const {
  BayesFeatures features;

  auto it = agent_history_map_.find(agent_id);
  if (it == agent_history_map_.end() ||
      static_cast<int>(it->second.size()) < kMinHistoryForBayes) {
    return features;
  }

  const auto& history = it->second;
  double agent_s = 0.0, agent_l = 0.0;
  if (!ego_lane_coord->XYToSL(agent.x(), agent.y(), &agent_s, &agent_l)) {
    return features;
  }

  const double sign_l = (agent_l > 0) ? 1.0 : -1.0;

  for (int i = 0; i < static_cast<int>(history.size()); ++i) {
    const double hist_sign_l = (history[i].l > 0) ? 1.0 : -1.0;
    features.norm_l_dot.push_back(history[i].l_dot * hist_sign_l);
    features.hist_lateral_dist.push_back(std::fabs(history[i].l));
  }

  if (agent.trajectories().empty() || agent.trajectories().front().empty()) {
    return features;
  }

  const auto& traj = agent.trajectories().front();

  for (int i = 1; i < kPredFrames && i < static_cast<int>(traj.size()); ++i) {
    double pred_s = 0.0, pred_l = 0.0;
    if (!ego_lane_coord->XYToSL(traj[i].x(), traj[i].y(), &pred_s, &pred_l)) {
      break;
    }

    double pred_l_prev = 0.0;
    double pred_s_prev = 0.0;
    if (!ego_lane_coord->XYToSL(traj[i - 1].x(), traj[i - 1].y(), &pred_s_prev,
                                &pred_l_prev)) {
      break;
    }

    const double pred_vy = (pred_l - pred_l_prev) / kPredFrameInterval;
    features.pred_norm_vy.push_back(pred_vy * sign_l);
    features.pred_lateral_dist.push_back(std::fabs(pred_l));
  }

  features.valid = !features.pred_norm_vy.empty();
  return features;
}

void AgentLongitudinalDecider::UpdateBayesianPosteriors(
    int32_t agent_id, double log_L_hist_cutin, double log_L_hist_cutout,
    double log_L_hist_normal, double log_L_pred_cutin,
    double log_L_pred_cutout, double log_L_pred_normal,
    double* cutin_posterior, double* cutout_posterior) {
  const double log_L_cutin =
      kBayesHistWeight * log_L_hist_cutin + kBayesPredWeight * log_L_pred_cutin;
  const double log_L_cutout = kBayesHistWeight * log_L_hist_cutout +
                               kBayesPredWeight * log_L_pred_cutout;
  const double log_L_normal = kBayesHistWeight * log_L_hist_normal +
                               kBayesPredWeight * log_L_pred_normal;

  const double max_log =
      std::max({log_L_cutin, log_L_cutout, log_L_normal});
  const double L_cutin = std::exp(log_L_cutin - max_log);
  const double L_cutout = std::exp(log_L_cutout - max_log);
  const double L_normal = std::exp(log_L_normal - max_log);

  const double denominator = L_cutin * kBayesCutinPrior +
                             L_cutout * kBayesCutoutPrior +
                             L_normal * kBayesNormalPrior;

  double p_cutin =
      (denominator < 1e-10) ? 0.0 : L_cutin * kBayesCutinPrior / denominator;
  double p_cutout =
      (denominator < 1e-10) ? 0.0 : L_cutout * kBayesCutoutPrior / denominator;

  auto it_cutin = bayes_cutin_posterior_.find(agent_id);
  const double prev_cutin =
      (it_cutin != bayes_cutin_posterior_.end()) ? it_cutin->second
                                                 : kBayesCutinPrior;
  p_cutin = kBayesSmoothAlpha * p_cutin + (1.0 - kBayesSmoothAlpha) * prev_cutin;

  auto it_cutout = bayes_cutout_posterior_.find(agent_id);
  const double prev_cutout =
      (it_cutout != bayes_cutout_posterior_.end()) ? it_cutout->second
                                                   : kBayesCutoutPrior;
  p_cutout = kBayesSmoothAlpha * p_cutout + (1.0 - kBayesSmoothAlpha) * prev_cutout;

  bayes_cutin_posterior_[agent_id] = p_cutin;
  bayes_cutout_posterior_[agent_id] = p_cutout;

  *cutin_posterior = p_cutin;
  *cutout_posterior = p_cutout;
}

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
}

void AgentLongitudinalDecider::UpdateAgentTable() {
  for (auto iter = cut_in_agent_count_.begin();
       iter != cut_in_agent_count_.end();) {
    if (current_agent_ids_.count(iter->first) == 0) {
      iter = cut_in_agent_count_.erase(iter);
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

  for (auto iter = bayes_cutin_posterior_.begin();
       iter != bayes_cutin_posterior_.end();) {
    if (current_agent_ids_.count(iter->first) == 0) {
      iter = bayes_cutin_posterior_.erase(iter);
    } else {
      if (processed_cut_in_agent_ids_.count(iter->first) == 0) {
        iter->second = kBayesCutinPrior;
      }
      ++iter;
    }
  }

  for (auto iter = bayes_cutout_posterior_.begin();
       iter != bayes_cutout_posterior_.end();) {
    if (current_agent_ids_.count(iter->first) == 0) {
      iter = bayes_cutout_posterior_.erase(iter);
    } else {
      if (processed_cut_out_agent_ids_.count(iter->first) == 0) {
        iter->second = kBayesCutoutPrior;
      }
      ++iter;
    }
  }

  processed_cut_in_agent_ids_.clear();
  processed_cut_out_agent_ids_.clear();
  current_agent_ids_.clear();

  std::vector<double> bayes_cutin_agent_ids;
  std::vector<double> bayes_cutin_scores;
  for (const auto& bayes_result : bayes_cutin_posterior_) {
    if (bayes_result.second > kBayesCutinPrior) {
      bayes_cutin_agent_ids.emplace_back(bayes_result.first);
      bayes_cutin_scores.emplace_back(bayes_result.second);
    }
  }
  JSON_DEBUG_VECTOR("bayes_cutin_agent_ids", bayes_cutin_agent_ids, 0);
  JSON_DEBUG_VECTOR("bayes_cutin_scores", bayes_cutin_scores, 2);

  std::vector<double> bayes_cutout_agent_ids;
  std::vector<double> bayes_cutout_scores;
  for (const auto& bayes_result : bayes_cutout_posterior_) {
    if (bayes_result.second > kBayesCutoutPrior) {
      bayes_cutout_agent_ids.emplace_back(bayes_result.first);
      bayes_cutout_scores.emplace_back(bayes_result.second);
    }
  }
  JSON_DEBUG_VECTOR("bayes_cutout_agent_ids", bayes_cutout_agent_ids, 0);
  JSON_DEBUG_VECTOR("bayes_cutout_scores", bayes_cutout_scores, 2);
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

  const auto& lane_change_decider_output =
      session_->planning_context().lane_change_decider_output();

  const auto is_in_lane_change_execution =
      lane_change_decider_output.curr_state == kLaneChangeExecution;

  const auto& ego_state_manager =
      session_->environmental_model().get_ego_state_manager();
  if (ego_state_manager == nullptr) {
    return;
  }
  const auto& planning_init_point = ego_state_manager->planning_init_point();

  const auto& ego_lane = virtual_lane_manager_->get_current_lane();
  if (ego_lane == nullptr) {
    return;
  }

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

  std::unordered_set<int32_t> target_lane_rear_agents;
  if (is_in_lane_change_execution) {
    const auto& lc_gap_info = lane_change_decider_output.lc_gap_info;
    const int64_t target_lane_rear_node_id = lc_gap_info.rear_node_id;
    if (target_lane_rear_node_id != -1) {
      auto* target_lane_rear_node =
          dynamic_world_->GetNode(target_lane_rear_node_id);
      if (target_lane_rear_node != nullptr) {
        target_lane_rear_agents.insert(target_lane_rear_node->node_agent_id());
      }
    }
  }

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
    ego_front_axle_s = ego_s;
  }

  for (const auto agent : agents) {
    if (agent == nullptr) {
      continue;
    }
    if (agent->is_tfl_virtual_obs() ||
        agent->is_stop_destination_virtual_obs() ||
        agent->is_turnstile_virtual_obs()) {
      continue;
    }
    double agent_s = 0.0;
    double agent_l = 0.0;
    if (!ego_lane_coord->XYToSL(agent->x(), agent->y(), &agent_s, &agent_l)) {
      continue;
    }

    const double half_length = agent->length() * 0.5;

    bool is_no_need_expand_agent = false;
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
    if (front_edge_s_diff > kEpsilon) {
      continue;
    }

    if (IsConsiderBackObs(ego_lane_coord, planning_init_point, agent.get(),
                          ego_front_edge_s, front_corner_s, ego_center_s,
                          front_edge_s_diff, min_lat_l_from_ego)) {
      continue;
    }

    if (target_lane_rear_agents.find(agent->agent_id()) !=
        target_lane_rear_agents.end()) {
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
    ILOG_ERROR << "[FilterReverseAgents] agent manager is empty";
    return;
  }

  const auto& vehicle_param =
      VehicleConfigurationContext::Instance()->get_vehicle_param();
  const double rear_axle_to_front_edge = vehicle_param.front_edge_to_rear_axle;

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

  double planning_init_point_s = 0.0;
  double planning_init_point_l = 0.0;
  if (!current_lane_coord->XYToSL(planning_init_point.x, planning_init_point.y,
                                  &planning_init_point_s,
                                  &planning_init_point_l)) {
    return;
  }

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
    const double considered_lon_distance =
        std::fmax(planning_init_point_s + kMinFilterDistance,
                  std::fmin(planning_init_point_s +
                                planning_init_point.v * kLongitudalTtc,
                            current_lane_coord->Length()));

    // only consider reverse agent
    if (!IsReverseAgent(agent.get(), current_lane, considered_lon_distance)) {
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
    // bool is_end_point_s_valid = false;
    // if (has_trajectory) {
    //   const auto& end_point = agent->trajectories().front().back();
    //   current_lane_coord->XYToSL(end_point.x(), end_point.y(),
    //                              &agent_end_point_s, &agent_end_point_l);
    //   end_point_heading_from_start = Vec2d(end_point.x() -
    //   agent_box_center.x(),
    //                                        end_point.y() -
    //                                        agent_box_center.y())
    //                                      .Angle();
    //   is_end_point_s_valid =
    //       agent_end_point_s - planning_init_point_s - rear_axle_to_front_edge
    //       > 0.0;
    // }

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
        continue;
      }
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
    if (considered_lon_distance < agent_s_in_ego_lane) {
      mutable_agent->mutable_agent_decision()->set_agent_decision_type(
          agent::AgentDecisionType::IGNORE);
      continue;
    }

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
    const agent::Agent* agent, const std::shared_ptr<VirtualLane> ego_lane,
    const double consider_distance, bool* is_perception_reverse_out,
    bool* is_prediction_reverse_out) const {
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

  // Output intermediate results if requested
  if (is_perception_reverse_out != nullptr) {
    *is_perception_reverse_out = is_perception_reverse;
  }
  if (is_prediction_reverse_out != nullptr) {
    *is_prediction_reverse_out = is_prediction_reverse;
  }

  bool is_reverse = false;
  if (agent_s < consider_distance) {
    is_reverse = is_perception_reverse && is_prediction_reverse;
  } else {
    is_reverse = is_perception_reverse || is_prediction_reverse;
  }
  return is_reverse && !agent->is_static();
}

void AgentLongitudinalDecider::FilterReverseAgentsHpp() {
  const auto& agent_manager =
      session_->environmental_model().get_agent_manager();
  auto mutable_agent_manager =
      session_->mutable_environmental_model()->mutable_agent_manager();
  if (agent_manager == nullptr || mutable_agent_manager == nullptr) {
    ILOG_ERROR << "[FilterReverseAgentsHpp] agent manager is empty";
    return;
  }

  const auto& agents = agent_manager->GetAllCurrentAgents();
  if (agents.empty()) {
    return;
  }

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

  double planning_init_point_s = 0.0;
  {
    double unused_l = 0.0;
    if (!current_lane_coord->XYToSL(planning_init_point.x, planning_init_point.y,
                                    &planning_init_point_s, &unused_l)) {
      CleanupHppReverseHysteresis({});
      return;
    }
  }

  const double consider_distance =
      std::fmax(planning_init_point_s + reverse_filter_config_.hpp_reverse_min_filter_distance_m,
                std::fmin(planning_init_point_s + v_ego * reverse_filter_config_.hpp_reverse_longitudinal_ttc_s,
                          current_lane_coord->Length()));

  std::unordered_set<int32_t> active_agent_ids;
  for (const auto agent : agents) {
    if (agent == nullptr) {
      continue;
    }
    if (agent->type() == agent::AgentType::VIRTUAL) {
      continue;
    }

    auto* mutable_agent =
        mutable_agent_manager->mutable_agent(agent->agent_id());
    if (mutable_agent == nullptr ||
        mutable_agent->agent_decision().agent_decision_type() ==
            agent::AgentDecisionType::IGNORE) {
      continue;
    }

    // Track all valid agents for stale-entry cleanup of hysteresis maps
    active_agent_ids.insert(agent->agent_id());

    // Evaluate reverse agent geometry and classification for this agent
    const ReverseEvidenceResult reverse_info =
        ComputeHppReverseAgentInfo(*agent, current_lane, consider_distance,
                                   planning_init_point_s);

    // If lane projection is invalid, keep the agent (fail-safe)
    if (!reverse_info.has_valid_lane_projection) {
      continue;
    }

    // Only process reverse agents
    if (!reverse_info.is_reverse_current) {
      continue;
    }
    mutable_agent->set_is_reverse(true);

    // Frame-count hysteresis wrapping the stateless geometry check
    const int32_t agent_id = agent->agent_id();
    const bool want_ignore = ShouldIgnoreReverseAgentInHpp(reverse_info);

    hpp_reverse_want_ignore_count_[agent_id] =
        want_ignore ? ++hpp_reverse_want_ignore_count_[agent_id] : 0;
    hpp_reverse_want_keep_count_[agent_id] =
        !want_ignore ? ++hpp_reverse_want_keep_count_[agent_id] : 0;

    const int kFramesToIgnore =
        reverse_filter_config_.hpp_reverse_hysteresis_frames_to_ignore;
    const int kFramesToKeep =
        reverse_filter_config_.hpp_reverse_hysteresis_frames_to_keep;

    if (hpp_reverse_want_ignore_count_[agent_id] >= kFramesToIgnore) {
      hpp_reverse_ignore_state_[agent_id] = true;
    } else if (hpp_reverse_want_keep_count_[agent_id] >= kFramesToKeep) {
      hpp_reverse_ignore_state_[agent_id] = false;
    }

    if (hpp_reverse_ignore_state_[agent_id]) {
      mutable_agent->mutable_agent_decision()->set_agent_decision_type(
          agent::AgentDecisionType::IGNORE);
    }
  }

  // Clean up stale entries for agents no longer in perception
  CleanupHppReverseHysteresis(active_agent_ids);
}

AgentLongitudinalDecider::ReverseEvidenceResult
AgentLongitudinalDecider::ComputeHppReverseAgentInfo(
    const agent::Agent& agent, const std::shared_ptr<VirtualLane>& ego_lane,
    double consider_distance, double ego_s) const {
  ReverseEvidenceResult result;

  const auto& ego_lane_coord = ego_lane->get_lane_frenet_coord();
  if (ego_lane_coord == nullptr) {
    return result;
  }

  // Project agent center to ego lane
  const auto agent_box_center = agent.box().center();
  double agent_s = 0.0;
  double agent_l = 0.0;
  if (!ego_lane_coord->XYToSL(agent_box_center.x(), agent_box_center.y(),
                              &agent_s, &agent_l)) {
    return result;
  }
  result.has_valid_lane_projection = true;
  // Store distance from ego to agent (used for zone classification)
  result.agent_s_center = agent_s - ego_s;

  // Compute per-corner SL boundary
  // 若任意角点投影失败，残缺的 l 边界会导致 lat_gap 被误算为极大值，
  // 进而在近/中/远分层中将目标误判为可过滤，违背 fail-safe 原则，
  // 因此整体降级为 has_valid_lane_projection=false，保留障碍物。
  double agent_max_l = std::numeric_limits<double>::lowest();
  double agent_min_l = std::numeric_limits<double>::max();
  const auto& corners = agent.box().GetAllCorners();
  bool all_corners_valid = true;
  for (const auto& corner : corners) {
    double corner_l = 0.0;
    double unused_s = 0.0;
    if (!ego_lane_coord->XYToSL(corner.x(), corner.y(), &unused_s, &corner_l)) {
      all_corners_valid = false;
      break;
    }
    agent_max_l = std::fmax(agent_max_l, corner_l);
    agent_min_l = std::fmin(agent_min_l, corner_l);
  }

  // If any corner projection failed, invalidate the entire lane projection
  if (!all_corners_valid) {
    result.has_valid_lane_projection = false;
    return result;
  }

  result.agent_min_l = agent_min_l;
  result.agent_max_l = agent_max_l;

  // Compute lateral gap to ego lane
  const double half_lane_width = kHalf * ego_lane->width_by_s(agent_s);
  result.lat_gap_to_ego_lane =
      CalculateLatGapToEgoLane(agent_min_l, agent_max_l, half_lane_width);

  // Reuse IsReverseAgent to compute perception/prediction reverse and is_reverse_current
  result.is_reverse_current = IsReverseAgent(
      &agent, ego_lane, consider_distance, &result.is_perception_reverse,
      &result.is_prediction_reverse);

  return result;
}

double AgentLongitudinalDecider::CalculateLatGapToEgoLane(
    double agent_min_l, double agent_max_l, double half_lane_width) const {
  // Lateral gap = minimum gap between [agent_min_l, agent_max_l]
  // and the ego lane interval [-half_lane_width, +half_lane_width]
  if (agent_max_l < -half_lane_width) {
    // Agent is entirely to the right of ego lane
    return -half_lane_width - agent_max_l;
  } else if (agent_min_l > half_lane_width) {
    // Agent is entirely to the left of ego lane
    return agent_min_l - half_lane_width;
  } else {
    // Agent overlaps with ego lane
    return 0.0;
  }
}

bool AgentLongitudinalDecider::ShouldIgnoreReverseAgentInHpp(
    const ReverseEvidenceResult& reverse_evidence) const {
  // Fail-safe: if lane projection is invalid, do not ignore the agent
  if (!reverse_evidence.has_valid_lane_projection) {
    return false;
  }

  const double agent_rel_s = reverse_evidence.agent_s_center;
  const double lat_gap = reverse_evidence.lat_gap_to_ego_lane;

  const double near_s_thr = reverse_filter_config_.hpp_reverse_near_s_threshold_m;
  const double mid_s_thr = reverse_filter_config_.hpp_reverse_mid_s_threshold_m;
  const double near_lat_gap_ignore =
      reverse_filter_config_.hpp_reverse_near_lat_gap_ignore_m;
  const double mid_lat_gap_ignore =
      reverse_filter_config_.hpp_reverse_mid_lat_gap_ignore_m;
  const double far_keep_lat_gap =
      reverse_filter_config_.hpp_reverse_far_keep_lat_gap_m;

  if (agent_rel_s < near_s_thr) {
    // 近区：横向间隙足够大时忽略（不影响自车道）
    return lat_gap > near_lat_gap_ignore;
  } else if (agent_rel_s < mid_s_thr) {
    // 中区：横向间隙足够大时忽略
    return lat_gap > mid_lat_gap_ignore;
  } else {
    // 远区：横向间隙超过阈值时忽略
    return lat_gap > far_keep_lat_gap;
  }
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

void AgentLongitudinalDecider::CleanupHppReverseHysteresis(
    const std::unordered_set<int32_t>& active_agent_ids) {
  for (auto it = hpp_reverse_want_ignore_count_.begin();
       it != hpp_reverse_want_ignore_count_.end();) {
    it = active_agent_ids.count(it->first) ? ++it
                                           : hpp_reverse_want_ignore_count_.erase(it);
  }
  for (auto it = hpp_reverse_want_keep_count_.begin();
       it != hpp_reverse_want_keep_count_.end();) {
    it = active_agent_ids.count(it->first) ? ++it
                                           : hpp_reverse_want_keep_count_.erase(it);
  }
  for (auto it = hpp_reverse_ignore_state_.begin();
       it != hpp_reverse_ignore_state_.end();) {
    it = active_agent_ids.count(it->first) ? ++it
                                           : hpp_reverse_ignore_state_.erase(it);
  }
}

}  // namespace planning