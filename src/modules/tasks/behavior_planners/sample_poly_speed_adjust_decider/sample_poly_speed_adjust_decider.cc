#include "sample_poly_speed_adjust_decider.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <memory>
#include <vector>

#include "behavior_planners/sample_poly_speed_adjust_decider/sample_poly_const.h"
#include "config/basic_type.h"
#include "dynamic_world/dynamic_agent_node.h"
#include "lateral_obstacle.h"
#include "planning_context.h"
#include "st_graph/st_point.h"
#include "trajectory1d/quartic_poly_trajectory1d.h"

using planning::planning_data::DynamicAgentNode;
using planning::planning_data::DynamicWorld;
namespace {
constexpr double kNormalSceneWeightMatchGapVel = 1.5;
constexpr double kNormalSceneWeightMatchGapS = 1.5;
constexpr double kNormalSceneWeightFollowVel = 2.0;
constexpr double kNormalSceneWeightStopLine = 25.0;
constexpr double kNormalSceneWeightLeadingSafeS = 5.5;
constexpr double kNormalSceneWeightLeadingSafeV = 0.0;
constexpr double kNormalSceneWeightVelVariable = 1.4;
constexpr double kNormalSceneWeightGapAvailable = 1.0;

constexpr double kPurseFlowVelSceneWeightMatchGapVel = 0.0;
constexpr double kPurseFlowVelSceneWeightMatchGapS = 0.0;
constexpr double kPurseFlowVelSceneWeightFollowVel = 2.5;
constexpr double kPurseFlowVelSceneWeightStopLine = 25.0;
constexpr double kPurseFlowVelSceneWeightLeadingSafeS = 5.5;
constexpr double kPurseFlowVelSceneWeightLeadingSafeV = 0.0;
constexpr double kPurseFlowVelSceneWeightVelVariable = 0.8;
constexpr double kPurseFlowVelSceneWeightGapAvailable = 0.5;

constexpr double kDeclerationSceneWeightMatchGapVel = 1.5;
constexpr double kDeclerationSceneWeightMatchGapS = 1.5;
constexpr double kDeclerationSceneWeightFollowVel = 0.0;
constexpr double kDeclerationSceneWeightStopLine = 25.0;
constexpr double kDeclerationSceneWeightLeadingSafeS = 5.5;
constexpr double kDeclerationSceneWeightLeadingSafeV = 0.0;
constexpr double kDeclerationSceneWeightVelVariable = 1.4;
constexpr double kDeclerationSceneWeightGapAvailable = 1.0;
}  // namespace
namespace planning {

SamplePolySpeedAdjustDecider::SamplePolySpeedAdjustDecider() {  // for pybind
  name_ = "SamplePolySpeedAdjustDecider";
  config_ = SamplePolySpeedAdjustDeciderConfig();

  const double front_edge_to_rear_axle = 4.025;
  const double rear_edge_to_rear_axle = 0.925;
  st_sample_space_base_ =
      STSampleSpaceBase(front_edge_to_rear_axle, rear_edge_to_rear_axle);
  sample_trajs_ = std::vector<std::vector<SampleQuarticPolynomialCurve>>();
  min_cost_traj_ptr_ = nullptr;
  evaulation_t_ = 5.0;
}

SamplePolySpeedAdjustDecider::SamplePolySpeedAdjustDecider(  // for pipeline
    const EgoPlanningConfigBuilder* config_builder, framework::Session* session)
    : Task(config_builder, session) {
  name_ = "SamplePolySpeedAdjustDecider";
  config_ = config_builder->cast<SamplePolySpeedAdjustDeciderConfig>();

  const double front_edge_to_rear_axle = VehicleConfigurationContext::Instance()
                                             ->get_vehicle_param()
                                             .front_edge_to_rear_axle;
  const double rear_edge_to_rear_axle = VehicleConfigurationContext::Instance()
                                            ->get_vehicle_param()
                                            .rear_edge_to_rear_axle;
  st_sample_space_base_ =
      STSampleSpaceBase(front_edge_to_rear_axle, rear_edge_to_rear_axle);

  sample_trajs_ = std::vector<std::vector<SampleQuarticPolynomialCurve>>();
  min_cost_traj_ptr_ = nullptr;

  SetNormalSceneWeight();
  evaulation_t_ = 5.0;
}

bool SamplePolySpeedAdjustDecider::Execute() {
  bool ok = true;
  std::chrono::time_point<std::chrono::high_resolution_clock> start_time =
      std::chrono::high_resolution_clock::now();
  ok = ProcessEnvInfos();

  if (ok) {
    ok = SamplePolys();
  }

  std::chrono::time_point<std::chrono::high_resolution_clock> sample_end_time =
      std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> sample_cost_time = sample_end_time - start_time;

  if (ok) {
    ok = Evaluate();
  }

  if (ok) {
    if (min_cost_traj_ptr_ != nullptr) {
      ok = BestTrajCheck();
    }
  }

  std::chrono::time_point<std::chrono::high_resolution_clock>
      evaluate_end_time = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> evaluate_cost_time =
      evaluate_end_time - sample_end_time;
  std::chrono::duration<double> all_cost_time = evaluate_end_time - start_time;

  auto& search_path = session_->mutable_planning_context()
                          ->mutable_lane_change_decider_output()
                          .st_search_vec;
  session_->mutable_planning_context()
      ->mutable_lane_change_decider_output()
      .s_search_status = false;

  if (ok && min_cost_traj_ptr_ != nullptr) {
    session_->mutable_planning_context()
        ->mutable_lane_change_decider_output()
        .s_search_status = true;
    search_path.clear();
    search_path.resize(kPlanningHorizions);
    for (size_t i = 0; i < kPlanningHorizions; i++) {
      double s = min_cost_traj_ptr_->CalcS(i * kPlanningStep) - ego_s_;
      search_path[i] = std::move(s);
    }

    last_min_cost_traj_ = SampleQuarticPolynomialCurve(*min_cost_traj_ptr_);
    last_ego_cart_point_ = ego_cart_point_;
  }

  LogDebugInfo(sample_cost_time.count(), evaluate_cost_time.count(),
               all_cost_time.count());
  return true;
};

bool SamplePolySpeedAdjustDecider::SamplePolys() {
  delta_t_ = (config_.sample_t_upper - config_.sample_t_lower) /
             (config_.sample_t_nums - 1);
  delta_v_ = (speed_adjust_range_.first - speed_adjust_range_.second) /
             (config_.sample_v_nums - 1);

  for (int i = 0; i < config_.sample_v_nums; i++) {
    const double v = speed_adjust_range_.second + i * delta_v_;
    std::vector<SampleQuarticPolynomialCurve> sample_traj_at_t;
    for (int j = 0; j < config_.sample_t_nums; j++) {
      const double t = config_.sample_t_lower + j * delta_t_;
      QuarticPolyState quartic_start{ego_s_, ego_v_, ego_a_,
                                     0.0};  // bind p0, v0, a0, ve, ae
      QuarticPolyState quartic_end{0.0, v, 0.0, t};
      QuarticPolynomial quartic_sample_polynomial(quartic_start, quartic_end);
      SampleQuarticPolynomialCurve quartic_sample_traj(
          quartic_sample_polynomial, evaulation_t_, 0.5 * evaulation_t_,
          weight_match_gap_vel_, weight_match_gap_s_, weight_follow_vel_,
          weight_stop_line_, weight_leading_safe_s_, weight_vel_variable_,
          weight_gap_avaliable_);

      sample_traj_at_t.emplace_back(std::move(quartic_sample_traj));
    }
    sample_trajs_.emplace_back(std::move(sample_traj_at_t));
  }
  return true;
};

bool SamplePolySpeedAdjustDecider::Evaluate() {
  std::chrono::time_point<std::chrono::high_resolution_clock> start_time =
      std::chrono::high_resolution_clock::now();
  double min_cost = kMaxPenalty;
  double leading_veh_s = kMaxPathLength;
  double leading_veh_v = kAgentNoValidVel;
  if (leading_veh_.id != kNoAgentId) {
    leading_veh_s =
        ego_s_ + leading_veh_.center_s + leading_veh_.v * evaulation_t_;
    leading_veh_v = leading_veh_.v;
  }

  for (size_t i = 0; i < sample_trajs_.size(); i++) {
    auto& sample_traj_at_v = sample_trajs_[i];
    for (size_t j = 0; j < sample_traj_at_v.size(); j++) {
      auto& sample_traj = sample_traj_at_v[j];
      sample_traj.CalcCost(st_sample_space_base_, ego_v_, ego_a_, v_suggestted_,
                           merge_stop_line_distance_, leading_veh_s,
                           leading_veh_v, leading_veh_.id);

      if (sample_traj.cost_sum_ < min_cost) {
        min_cost_traj_ptr_ = &sample_traj;
        min_cost = sample_traj.cost_sum_;
      }
    }
  }
  return true;
}

void SamplePolySpeedAdjustDecider::CalcTargetLaneObjsFlowVel() {
  if (agent_info_.empty()) {
    target_lane_objs_flow_vel_ = ego_v_;
    return;
  }

  double dis_weight = 0.0;
  double vel_sum = 0.0;

  for (const auto& veh_info : agent_info_) {
    if (veh_info.id < 0) {
      continue;
    }

    double veh_dis_weight = 1.0 / (std::fabs(veh_info.center_s) + 1e-6);
    dis_weight += veh_dis_weight;
    vel_sum += veh_dis_weight * veh_info.v;
  }

  if (dis_weight < 1e-6) {
    target_lane_objs_flow_vel_ = ego_v_;
  } else {
    target_lane_objs_flow_vel_ = vel_sum / dis_weight;
  }
  return;
}

void SamplePolySpeedAdjustDecider::CalcTargetLaneVehDensity() {
  if (agent_info_.size() < kJudgeCongestedVehNumThreshold) {
    traffic_density_ = 0.0;
    traffic_density_status_ = LineClear;
    return;
  }

  int total_vehicles = 0;
  double total_length = 0.0;

  for (const auto& veh : agent_info_) {
    if (std::fabs(veh.center_s) > kVehDensityDistanceThreshold) {
      continue;
    }

    total_length += std::fabs(veh.center_s);
    total_vehicles += 1;
  }

  traffic_density_ = total_vehicles / total_length;
  if (traffic_density_ >= kJudgeCongestedSceneDensity) {
    traffic_density_status_ = Congested;
  }
  return;
}

bool SamplePolySpeedAdjustDecider::ProcessEnvInfos() {
  // target lane ids
  DebugInfoManager::GetInstance()
      .GetDebugInfoPb()
      ->mutable_st_search_decider_info()
      ->mutable_sample_poly_speed_info()
      ->Clear();
  agent_info_.clear();

  leading_veh_ = AgentInfo();
  sample_status_ = OK;
  const auto& ego_state_manager =
      session_->environmental_model().get_ego_state_manager();
  const auto& ego_frenet_state = session_->environmental_model()
                                     .get_reference_path_manager()
                                     ->get_reference_path_by_current_lane()
                                     ->get_frenet_ego_state();
  const double ego_s = ego_frenet_state.s();
  const auto target_lane_virtual_id = session_->planning_context()
                                          .lane_change_decider_output()
                                          .target_lane_virtual_id;
  const std::shared_ptr<DynamicWorld> dynamic_world =
      session_->environmental_model().get_dynamic_world();
  const TrackedObject* lead_one =
      session_->environmental_model().get_lateral_obstacle()->leadone();
  const auto& coarse_planning_info = session_->planning_context()
                                         .lane_change_decider_output()
                                         .coarse_planning_info;
  lane_change_request_ =
      session_->planning_context().lane_change_decider_output().lc_request;
  lane_change_source_ = session_->planning_context()
                            .lane_change_decider_output()
                            .lc_request_source;
  is_in_merge_region_ = session_->mutable_planning_context()
                            ->ego_lane_road_right_decider_output()
                            .is_merge_region;
  merge_stop_line_distance_ = kMaxMergeDistance;
  // process lane change status
  if (coarse_planning_info.target_state != kLaneChangePropose) {
    count_wait_state_ = 0;
    lane_change_request_ = 0;
    sample_scene_ = SampleScene::NormalSampleScene;
    count_normal_to_hover_state_ = 0;
    count_hover_to_normal_state_ = 0;
    ClearStitchedPolyPtr();
    return false;
  }
  if (coarse_planning_info.target_state == kLaneChangePropose) {
    count_wait_state_++;
  }

  if (count_wait_state_ <= 3) {
    ClearStitchedPolyPtr();
    return false;
  }

  if (lead_one != nullptr) {
    if (lead_one->track_id > 0) {
      leading_veh_.half_length = lead_one->length * 0.5;
      leading_veh_.id = lead_one->track_id;
      leading_veh_.v = lead_one->v;
      leading_veh_.center_s = lead_one->d_rel;
    }
  }

  const std::vector<const DynamicAgentNode*> target_lane_nodes =
      dynamic_world->GetNodesByLaneId(target_lane_virtual_id);
  for (const auto* target_lane_node : target_lane_nodes) {
    AgentInfo target_lane_agnet_info{
        target_lane_node->node_agent_id(),
        target_lane_node->node_s() - ego_s,
        target_lane_node->node_length() * 0.5,
        target_lane_node->node_speed(),
    };

    agent_info_.emplace_back(std::move(target_lane_agnet_info));
  }

  std::sort(agent_info_.begin(), agent_info_.end(),
            [&](AgentInfo& a, AgentInfo& b) -> bool {
              return a.center_s < b.center_s;
            });

  // ego state info
  ego_s_ = ego_s;
  ego_v_ = ego_state_manager->ego_v();
  ego_a_ = ego_state_manager->ego_acc();

  ego_cart_point_.first = ego_state_manager->ego_pose().x;
  ego_cart_point_.second = ego_state_manager->ego_pose().y;

  v_suggestted_ = ego_state_manager->ego_v_cruise();

  // init sample space
  st_sample_space_base_.Init(agent_info_, ego_s);

  // sample v upper and lower
  speed_adjust_range_.first = std::fmin(
      config_.sample_v_upper, ego_v_ + config_.maximum_speed_adjustment);
  speed_adjust_range_.second = std::fmax(
      config_.sample_v_lower, ego_v_ - config_.maximum_speed_adjustment);

  // calc flow vel
  StitchLastBestPoly();
  min_cost_traj_ptr_ = nullptr;  // clear last ptr;
  sample_trajs_.clear();

  CalcTargetLaneObjsFlowVel();
  CalcTargetLaneVehDensity();

  RunSampleSceneStateMachine();

  return !agent_info_.empty();
}

bool SamplePolySpeedAdjustDecider::IsInDeceleartionScene() {
  const auto& virtual_lane_mgr =
      session_->environmental_model().get_virtual_lane_manager();
  boundary_merge_point_valid_ = session_->planning_context()
                                    .ego_lane_road_right_decider_output()
                                    .boundary_merge_point_valid;

  const auto& route_info_output =
      session_->environmental_model().get_route_info()->get_route_info_output();

  is_nearing_ramp_ =
      session_->planning_context().lane_change_decider_output().is_nearing_ramp;
  distance_to_road_merge_ =
      virtual_lane_mgr->get_distance_to_first_road_merge();
  distance_to_road_split_ =
      virtual_lane_mgr->get_distance_to_first_road_split();
  distance_to_ramp_ = route_info_output.dis_to_ramp;

  if (lane_change_source_ == MAP_REQUEST ||
      lane_change_source_ == MERGE_REQUEST) {
    if (boundary_merge_point_valid_) {
      const auto& boundary_merge_point =
          session_->planning_context()
              .ego_lane_road_right_decider_output()
              .boundary_merge_point;
      merge_stop_line_distance_ =
          std::hypot(boundary_merge_point.x - ego_cart_point_.first,
                     boundary_merge_point.y - ego_cart_point_.second);
      return true;
    } else {
      if (distance_to_road_split_ < kDistanceToMapRequestPoint ||
          distance_to_road_merge_ < kDistanceToMapRequestPoint ||
          is_in_merge_region_) {
        merge_stop_line_distance_ =
            std::fmin(distance_to_road_merge_, distance_to_road_split_);
        return true;
      }
    }
  } else if (is_nearing_ramp_) {
    merge_stop_line_distance_ = distance_to_ramp_;
    return true;
  }

  return false;
}

void SamplePolySpeedAdjustDecider::StitchLastBestPoly() {
  if (min_cost_traj_ptr_ != nullptr) {
    // step1: get t hit by current ego point
    std::vector<double> s_vec(kPlanningHorizions, 0.0);
    std::vector<double> t_vec(kPlanningHorizions, 0.0);

    for (size_t i = 1; i < kPlanningHorizions; ++i) {
      double s_delta = min_cost_traj_ptr_->CalcS(i * kPlanningStep) -
                       min_cost_traj_ptr_->CalcS((i - 1) * kPlanningStep);
      s_vec[i] = s_vec[i - 1] + s_delta;  // 累计和构建
      t_vec[i] = i * kPlanningStep;
    }
    double delta_s =
        std::hypot(ego_cart_point_.first - last_ego_cart_point_.first,
                   ego_cart_point_.second - last_ego_cart_point_.second);
    std::cout << "Delta s between the adjacent frames: " << delta_s
              << std::endl;

    auto iter_delta_s = std::lower_bound(s_vec.begin(), s_vec.end(), delta_s);
    if (iter_delta_s == s_vec.end()) {
      --iter_delta_s;
    }

    size_t index = std::distance(s_vec.begin(), iter_delta_s);
    size_t next_index = (index + 1 < kPlanningHorizions) ? index + 1 : index;
    double s_curr = s_vec[index];
    double s_next = s_vec[next_index];
    double t_curr = t_vec[index];
    double t_next = t_vec[next_index];

    double t_hit = t_curr;
    if (s_next > s_curr) {
      t_hit += (delta_s - s_curr) / (s_next - s_curr) * (t_next - t_curr);
    }

    std::cout << "interpolated t_hit: " << t_hit << std::endl;

    // step2: resample the last best traj
    const double resample_t = min_cost_traj_ptr_->poly().T() - t_hit;
    const double resample_v = min_cost_traj_ptr_->arrived_v();
    QuarticPolyState resample_start{ego_s_, ego_v_, ego_a_,
                                    0.0};  // bind p0, v0, a0, ve, ae
    QuarticPolyState resample_end{0.0, resample_v, 0.0, resample_t};
    QuarticPolynomial resample_polynomial(resample_start, resample_end);
    stitched_last_best_quartic_poly_ptr_ =
        std::make_shared<SampleQuarticPolynomialCurve>(
            resample_polynomial, evaulation_t_, 0.5 * evaulation_t_,
            weight_match_gap_vel_, weight_match_gap_s_, weight_follow_vel_,
            weight_stop_line_, weight_leading_safe_s_, weight_vel_variable_,
            weight_gap_avaliable_);
    const double stitched_poly_checked_s =
        stitched_last_best_quartic_poly_ptr_->CalcS(evaulation_t_);
    planning::speed::STPoint stitched_poly_checked_lower_st_point,
        stitched_poly_checked_upper_st_point;
    st_sample_space_base_.GetBorderByAvailable(
        stitched_poly_checked_s, evaulation_t_,
        &stitched_poly_checked_lower_st_point,
        &stitched_poly_checked_upper_st_point);
    stitched_last_best_quartic_poly_ptr_->set_end_point_matched_gap_front_id(
        stitched_poly_checked_upper_st_point.agent_id());
    stitched_last_best_quartic_poly_ptr_->set_end_point_matched_gap_back_id(
        stitched_poly_checked_lower_st_point.agent_id());
  }
}

void SamplePolySpeedAdjustDecider::RunSampleSceneStateMachine() {
  //  sample scene state machine
  if (sample_scene_ == NormalSampleScene) {
    if (std::fabs(target_lane_objs_flow_vel_ - ego_v_) >
            kJudePurseFlowVelValue &&
        traffic_density_status_ == Congested) {
      count_normal_to_hover_state_ =
          std::min(count_normal_to_hover_state_ + 2, kNormalToHoverThreshold);
    } else {
      count_normal_to_hover_state_ =
          std::max(count_normal_to_hover_state_ - 1, 0);
    }

    // smooth the count
    // count_normal_to_hover_state_ = count_normal_to_hover_state_ * 0.9 + 1;
    if (count_normal_to_hover_state_ >= kNormalToHoverThreshold &&
        target_lane_objs_flow_vel_ >= 1.0) {
      sample_scene_ = PurseFlowVelScene;
      count_normal_to_hover_state_ = 0;  // reset
      ClearStitchedPolyPtr();
    }
  } else if (sample_scene_ == PurseFlowVelScene) {
    if (std::fabs(target_lane_objs_flow_vel_ - ego_v_) <
        kJudePurseFlowVelValue) {
      count_hover_to_normal_state_ =
          std::min(count_hover_to_normal_state_ + 2, kHoverToNormalThreshold);
    } else {
      count_hover_to_normal_state_ =
          std::max(count_hover_to_normal_state_ - 1, 0);
    }

    if (count_hover_to_normal_state_ >= kHoverToNormalThreshold) {
      sample_scene_ = NormalSampleScene;
      count_hover_to_normal_state_ = 0;
      ClearStitchedPolyPtr();
    }
  }

  if (IsInDeceleartionScene()) {
    sample_scene_ = DecelerationPriorityScene;
    count_hover_to_normal_state_ = 0;
    count_normal_to_hover_state_ = 0;
    SetDeclerationSceneWeight();
    ClearStitchedPolyPtr();
  } else {
    if (sample_scene_ == NormalSampleScene) {
      SetNormalSceneWeight();
    } else if (sample_scene_ == PurseFlowVelScene) {
      v_suggestted_ = target_lane_objs_flow_vel_;
      SetPurseFlowVelSceneWeight();
    }
  }
}

void SamplePolySpeedAdjustDecider::SetNormalSceneWeight() {
  weight_match_gap_vel_ = kNormalSceneWeightMatchGapVel;
  weight_match_gap_s_ = kNormalSceneWeightMatchGapS;
  weight_follow_vel_ = kNormalSceneWeightFollowVel;
  weight_stop_line_ = kNormalSceneWeightStopLine;
  weight_leading_safe_s_ = kNormalSceneWeightLeadingSafeS;
  weight_leading_safe_v_ = kNormalSceneWeightLeadingSafeV;
  weight_vel_variable_ = kNormalSceneWeightVelVariable;
  weight_gap_avaliable_ = kNormalSceneWeightGapAvailable;
}

void SamplePolySpeedAdjustDecider::SetPurseFlowVelSceneWeight() {
  weight_match_gap_vel_ = kPurseFlowVelSceneWeightMatchGapVel;
  weight_match_gap_s_ = kPurseFlowVelSceneWeightMatchGapS;
  weight_follow_vel_ = kPurseFlowVelSceneWeightFollowVel;
  weight_stop_line_ = kPurseFlowVelSceneWeightStopLine;
  weight_leading_safe_s_ = kPurseFlowVelSceneWeightLeadingSafeS;
  weight_leading_safe_v_ = kPurseFlowVelSceneWeightLeadingSafeV;
  weight_vel_variable_ = kPurseFlowVelSceneWeightVelVariable;
  weight_gap_avaliable_ = kPurseFlowVelSceneWeightGapAvailable;
}

void SamplePolySpeedAdjustDecider::SetDeclerationSceneWeight() {
  weight_match_gap_vel_ = kDeclerationSceneWeightMatchGapVel;
  weight_match_gap_s_ = kDeclerationSceneWeightMatchGapS;
  weight_follow_vel_ = kDeclerationSceneWeightFollowVel;
  weight_stop_line_ = kDeclerationSceneWeightStopLine;
  weight_leading_safe_s_ = kDeclerationSceneWeightLeadingSafeS;
  weight_leading_safe_v_ = kDeclerationSceneWeightLeadingSafeV;
  weight_vel_variable_ = kDeclerationSceneWeightVelVariable;
  weight_gap_avaliable_ = kDeclerationSceneWeightGapAvailable;
}

double SamplePolySpeedAdjustDecider::CalcHeadwayDistance(
    const double& headway_v, const double ego_v,
    const std::vector<double>& t_gap_ego_v_bp,
    const std::vector<double>& t_gap_ego_v) {
  double v_lead_clip = std::max(headway_v, 0.0);
  double t_gap = interp(ego_v, t_gap_ego_v_bp, t_gap_ego_v);
  t_gap = t_gap * (0.6 * ego_v * 0.01);  // why?
  double v_rel = std::min(std::max(ego_v - v_lead_clip, 0.0), 5.0);
  double distance_hysteresis = v_rel * 0.3;
  double fix_safe_distance = 3.5;
  return fix_safe_distance + t_gap * v_lead_clip + distance_hysteresis;
}

bool SamplePolySpeedAdjustDecider::BestTrajCheck() {
  if (leading_veh_.id != kNoAgentId && leading_veh_.id != -1) {
    const double ego_pred_end_s = min_cost_traj_ptr_->CalcS(evaulation_t_);
    if (ego_pred_end_s >
        leading_veh_.center_s + ego_s_ + leading_veh_.v * evaulation_t_ -
            CalcHeadwayDistance(leading_veh_.v, ego_v_, t_gap_ego_v_bp_,
                                t_gap_ego_v_)) {
      std::cout << "ego pred s is exceed upper" << std::endl;
      min_cost_traj_ptr_ = nullptr;
      sample_status_ = kEgoPredSExceedLeadOne;
      return false;
    }
  }

  return true;
}

bool SamplePolySpeedAdjustDecider::CheckInitVelTraj() {
  double init_vel_is_ok = true;
  const double ego_init_vel_pred_end_s = ego_s_ + ego_v_ * kPlanningDuration;

  if (leading_veh_.id != kNoAgentId && leading_veh_.id != -1) {
    if (ego_init_vel_pred_end_s >
        leading_veh_.center_s + ego_s_ + leading_veh_.v * evaulation_t_ -
            CalcHeadwayDistance(leading_veh_.v, ego_v_, t_gap_ego_v_bp_,
                                t_gap_ego_v_)) {
      init_vel_is_ok = false;
    }
  }
  if (init_vel_is_ok) {
    planning::speed::STPoint lower_st_point;
    planning::speed::STPoint upper_st_point;

    st_sample_space_base_.GetBorderByAvailable(
        ego_init_vel_pred_end_s, kPlanningDuration, &lower_st_point,
        &upper_st_point);
    if (lower_st_point.agent_id() != kNoAgentId) {
      if (ego_init_vel_pred_end_s < lower_st_point.s() + kBasicSafeDistance ||
          ego_v_ < lower_st_point.velocity()) {
        init_vel_is_ok = false;
        return init_vel_is_ok;
      }
    }

    if (upper_st_point.agent_id() != kNoAgentId) {
      if (ego_init_vel_pred_end_s > upper_st_point.s() - kBasicSafeDistance ||
          ego_v_ > upper_st_point.velocity()) {
        init_vel_is_ok = false;
        return init_vel_is_ok;
      }
    }
  }
  return init_vel_is_ok;
}

void SamplePolySpeedAdjustDecider::ClearStitchedPolyPtr() {
  stitched_last_best_quartic_poly_ptr_ = nullptr;
}

void SamplePolySpeedAdjustDecider::LogDebugInfo(const double sample_cost_time,
                                                const double evaluate_cost_time,
                                                const double all_cost_time) {
  auto sample_poly_speed_pb_info = DebugInfoManager::GetInstance()
                                       .GetDebugInfoPb()
                                       ->mutable_st_search_decider_info()
                                       ->mutable_sample_poly_speed_info();

  sample_poly_speed_pb_info->mutable_sample_print_table_info()
      ->set_dist_to_ramp(distance_to_ramp_);
  sample_poly_speed_pb_info->mutable_sample_print_table_info()
      ->set_is_nearing_ramp(is_nearing_ramp_);
  sample_poly_speed_pb_info->mutable_sample_print_table_info()
      ->set_is_in_merge_region(is_in_merge_region_);
  sample_poly_speed_pb_info->mutable_sample_print_table_info()
      ->set_merge_emegency_distance(merge_stop_line_distance_);
  sample_poly_speed_pb_info->mutable_sample_print_table_info()
      ->set_distance_to_road_merge(distance_to_road_merge_);
  sample_poly_speed_pb_info->mutable_sample_print_table_info()
      ->set_distance_to_road_split(distance_to_road_split_);

  sample_poly_speed_pb_info->mutable_sample_print_table_info()->set_ego_s(
      ego_s_);
  sample_poly_speed_pb_info->mutable_sample_print_table_info()->set_ego_v(
      ego_v_);
  sample_poly_speed_pb_info->mutable_sample_print_table_info()->set_ego_acc(
      ego_a_);

  sample_poly_speed_pb_info->mutable_sample_print_table_info()
      ->set_v_suggestted(v_suggestted_);

  for (size_t i = 0; i < agent_info_.size(); i++) {
    auto* agent_info = sample_poly_speed_pb_info->mutable_agent_infos()->Add();
    agent_info->set_id(agent_info_[i].id);
    agent_info->set_center_s(agent_info_[i].center_s);
    agent_info->set_half_length(agent_info_[i].half_length);
    agent_info->set_v(agent_info_[i].v);
  }

  sample_poly_speed_pb_info->mutable_leading_veh_info()->set_id(-1);
  if (leading_veh_.id > 0) {
    sample_poly_speed_pb_info->mutable_leading_veh_info()->set_id(
        leading_veh_.id);
    sample_poly_speed_pb_info->mutable_leading_veh_info()->set_center_s(
        leading_veh_.center_s);
    sample_poly_speed_pb_info->mutable_leading_veh_info()->set_half_length(
        leading_veh_.half_length);
    sample_poly_speed_pb_info->mutable_leading_veh_info()->set_v(
        leading_veh_.v);
  }

  const auto& search_path = session_->mutable_planning_context()
                                ->mutable_lane_change_decider_output()
                                .st_search_vec;

  sample_poly_speed_pb_info->mutable_sample_print_table_info()
      ->mutable_current_sample_match_gap_id()
      ->set_gap_front_id(kNoAgentId);
  sample_poly_speed_pb_info->mutable_sample_print_table_info()
      ->mutable_current_sample_match_gap_id()
      ->set_gap_back_id(kNoAgentId);
  sample_poly_speed_pb_info->mutable_sample_print_table_info()
      ->mutable_stitched_sample_match_gap_id()
      ->set_gap_front_id(kNoAgentId);
  sample_poly_speed_pb_info->mutable_sample_print_table_info()
      ->mutable_stitched_sample_match_gap_id()
      ->set_gap_back_id(kNoAgentId);
  sample_poly_speed_pb_info->mutable_sample_print_table_info()
      ->set_sample_status(0);
  if (min_cost_traj_ptr_ != nullptr) {
    sample_poly_speed_pb_info->mutable_sample_print_table_info()
        ->set_sample_status(1);
    for (double t = 0; t < kPlanningDuration + 0.21; t += 0.2) {
      const double s = min_cost_traj_ptr_->CalcS(t) - ego_s_;
      sample_poly_speed_pb_info->mutable_sample_s_vec()->Add(s);
      sample_poly_speed_pb_info->mutable_sample_v_vec()->Add(
          min_cost_traj_ptr_->CalcV(t));
      sample_poly_speed_pb_info->mutable_sample_a_vec()->Add(
          min_cost_traj_ptr_->CalcAcc(t));
      sample_poly_speed_pb_info->mutable_sample_j_vec()->Add(
          min_cost_traj_ptr_->CalcJerk(t));
      sample_poly_speed_pb_info->mutable_sample_t_vec()->Add(t);
    }

    sample_poly_speed_pb_info->mutable_sample_print_table_info()
        ->mutable_current_sample_match_gap_id()
        ->set_gap_front_id(
            min_cost_traj_ptr_->end_point_matched_gap_front_id());
    sample_poly_speed_pb_info->mutable_sample_print_table_info()
        ->mutable_current_sample_match_gap_id()
        ->set_gap_back_id(min_cost_traj_ptr_->end_point_matched_gap_back_id());

    sample_poly_speed_pb_info->mutable_sample_print_table_info()
        ->mutable_best_poly_cost_info()
        ->set_end_point_match_gap_s_cost(
            min_cost_traj_ptr_->end_point_match_gap_cost().match_s_cost());
    sample_poly_speed_pb_info->mutable_sample_print_table_info()
        ->mutable_best_poly_cost_info()
        ->set_end_point_match_gap_vel_cost(
            min_cost_traj_ptr_->end_point_match_gap_cost().match_v_cost());
    sample_poly_speed_pb_info->mutable_sample_print_table_info()
        ->mutable_best_poly_cost_info()
        ->set_end_point_match_gap_center_cost(
            min_cost_traj_ptr_->end_point_match_gap_cost()
                .match_gap_center_cost());
    sample_poly_speed_pb_info->mutable_sample_print_table_info()
        ->mutable_best_poly_cost_info()
        ->set_mid_point_match_gap_s_cost(
            min_cost_traj_ptr_->mid_point_match_gap_cost().match_s_cost());
    sample_poly_speed_pb_info->mutable_sample_print_table_info()
        ->mutable_best_poly_cost_info()
        ->set_mid_point_match_gap_vel_cost(
            min_cost_traj_ptr_->mid_point_match_gap_cost().match_v_cost());
    sample_poly_speed_pb_info->mutable_sample_print_table_info()
        ->mutable_best_poly_cost_info()
        ->set_mid_point_match_gap_center_cost(
            min_cost_traj_ptr_->mid_point_match_gap_cost()
                .match_gap_center_cost());
    sample_poly_speed_pb_info->mutable_sample_print_table_info()
        ->mutable_best_poly_cost_info()
        ->set_follow_vel_cost(min_cost_traj_ptr_->follow_vel_cost().cost());
    sample_poly_speed_pb_info->mutable_sample_print_table_info()
        ->mutable_best_poly_cost_info()
        ->set_stop_line_cost(min_cost_traj_ptr_->stop_line_cost().cost());
    sample_poly_speed_pb_info->mutable_sample_print_table_info()
        ->mutable_best_poly_cost_info()
        ->set_leading_veh_safe_cost(
            min_cost_traj_ptr_->leading_veh_safe_cost().cost());
    sample_poly_speed_pb_info->mutable_sample_print_table_info()
        ->mutable_best_poly_cost_info()
        ->set_vel_variable_cost(
            min_cost_traj_ptr_->speed_variable_cost().cost());
    sample_poly_speed_pb_info->mutable_sample_print_table_info()
        ->mutable_best_poly_cost_info()
        ->set_gap_avaliable_cost(
            min_cost_traj_ptr_->gap_avaliable_cost().cost());
  }

  if (stitched_last_best_quartic_poly_ptr_ != nullptr) {
    for (double t = 0; t < kPlanningDuration + 0.21; t += 0.2) {
      const double s = stitched_last_best_quartic_poly_ptr_->CalcS(t) - ego_s_;
      sample_poly_speed_pb_info->mutable_stitched_sample_s_vec()->Add(s);
      sample_poly_speed_pb_info->mutable_stitched_sample_v_vec()->Add(
          stitched_last_best_quartic_poly_ptr_->CalcV(t));
      sample_poly_speed_pb_info->mutable_stitched_sample_a_vec()->Add(
          stitched_last_best_quartic_poly_ptr_->CalcAcc(t));
      sample_poly_speed_pb_info->mutable_stitched_sample_j_vec()->Add(
          stitched_last_best_quartic_poly_ptr_->CalcJerk(t));
      sample_poly_speed_pb_info->mutable_stitched_sample_t_vec()->Add(t);
    }

    sample_poly_speed_pb_info->mutable_sample_print_table_info()
        ->mutable_stitched_sample_match_gap_id()
        ->set_gap_front_id(stitched_last_best_quartic_poly_ptr_
                               ->end_point_matched_gap_front_id());
    sample_poly_speed_pb_info->mutable_sample_print_table_info()
        ->mutable_stitched_sample_match_gap_id()
        ->set_gap_back_id(stitched_last_best_quartic_poly_ptr_
                              ->end_point_matched_gap_back_id());

    sample_poly_speed_pb_info->mutable_sample_print_table_info()
        ->mutable_stitched_poly_cost_info()
        ->set_end_point_match_gap_s_cost(
            stitched_last_best_quartic_poly_ptr_->end_point_match_gap_cost()
                .match_s_cost());
    sample_poly_speed_pb_info->mutable_sample_print_table_info()
        ->mutable_stitched_poly_cost_info()
        ->set_end_point_match_gap_vel_cost(
            stitched_last_best_quartic_poly_ptr_->end_point_match_gap_cost()
                .match_v_cost());
    sample_poly_speed_pb_info->mutable_sample_print_table_info()
        ->mutable_stitched_poly_cost_info()
        ->set_end_point_match_gap_center_cost(
            stitched_last_best_quartic_poly_ptr_->end_point_match_gap_cost()
                .match_gap_center_cost());
    sample_poly_speed_pb_info->mutable_sample_print_table_info()
        ->mutable_stitched_poly_cost_info()
        ->set_mid_point_match_gap_s_cost(
            stitched_last_best_quartic_poly_ptr_->mid_point_match_gap_cost()
                .match_s_cost());
    sample_poly_speed_pb_info->mutable_sample_print_table_info()
        ->mutable_stitched_poly_cost_info()
        ->set_mid_point_match_gap_vel_cost(
            stitched_last_best_quartic_poly_ptr_->mid_point_match_gap_cost()
                .match_v_cost());
    sample_poly_speed_pb_info->mutable_sample_print_table_info()
        ->mutable_stitched_poly_cost_info()
        ->set_mid_point_match_gap_center_cost(
            stitched_last_best_quartic_poly_ptr_->mid_point_match_gap_cost()
                .match_gap_center_cost());
    sample_poly_speed_pb_info->mutable_sample_print_table_info()
        ->mutable_stitched_poly_cost_info()
        ->set_follow_vel_cost(
            stitched_last_best_quartic_poly_ptr_->follow_vel_cost().cost());
    sample_poly_speed_pb_info->mutable_sample_print_table_info()
        ->mutable_stitched_poly_cost_info()
        ->set_stop_line_cost(
            stitched_last_best_quartic_poly_ptr_->stop_line_cost().cost());
    sample_poly_speed_pb_info->mutable_sample_print_table_info()
        ->mutable_stitched_poly_cost_info()
        ->set_leading_veh_safe_cost(
            stitched_last_best_quartic_poly_ptr_->leading_veh_safe_cost()
                .cost());
    sample_poly_speed_pb_info->mutable_sample_print_table_info()
        ->mutable_stitched_poly_cost_info()
        ->set_vel_variable_cost(
            stitched_last_best_quartic_poly_ptr_->speed_variable_cost().cost());
    sample_poly_speed_pb_info->mutable_sample_print_table_info()
        ->mutable_stitched_poly_cost_info()
        ->set_gap_avaliable_cost(
            stitched_last_best_quartic_poly_ptr_->gap_avaliable_cost().cost());
  }

  sample_poly_speed_pb_info->mutable_sample_print_table_info()->set_flow_vel(
      target_lane_objs_flow_vel_);
  sample_poly_speed_pb_info->mutable_sample_print_table_info()
      ->set_sample_cost_time(sample_cost_time);
  sample_poly_speed_pb_info->mutable_sample_print_table_info()
      ->set_all_cost_time(all_cost_time);
  sample_poly_speed_pb_info->mutable_sample_print_table_info()
      ->set_evaluate_cost_time(evaluate_cost_time);

  sample_poly_speed_pb_info->mutable_sample_print_table_info()
      ->set_sample_scene(sample_scene_);
  sample_poly_speed_pb_info->mutable_sample_print_table_info()
      ->set_count_normal_to_hover_state(count_normal_to_hover_state_);
  sample_poly_speed_pb_info->mutable_sample_print_table_info()
      ->set_count_hover_to_normal_state(count_hover_to_normal_state_);
  sample_poly_speed_pb_info->mutable_sample_print_table_info()
      ->set_traffic_density(traffic_density_);
  sample_poly_speed_pb_info->mutable_sample_print_table_info()
      ->set_traffic_density_status(traffic_density_status_);
  sample_poly_speed_pb_info->mutable_sample_param()->set_sample_delta_t(
      delta_t_);
  sample_poly_speed_pb_info->mutable_sample_param()->set_sample_delta_v(
      delta_v_);
  sample_poly_speed_pb_info->mutable_sample_param()->set_sample_upper_v(
      speed_adjust_range_.first);
  sample_poly_speed_pb_info->mutable_sample_param()->set_sample_lower_v(
      speed_adjust_range_.second);
  sample_poly_speed_pb_info->mutable_sample_param()->set_weight_match_gap_vel(
      weight_match_gap_vel_);
  sample_poly_speed_pb_info->mutable_sample_param()->set_weight_match_gap_s(
      weight_match_gap_s_);
  sample_poly_speed_pb_info->mutable_sample_param()->set_weight_follow_vel(
      weight_follow_vel_);
  sample_poly_speed_pb_info->mutable_sample_param()->set_weight_stop_line(
      weight_stop_line_);
  sample_poly_speed_pb_info->mutable_sample_param()->set_weight_leading_safe_s(
      weight_leading_safe_s_);
  sample_poly_speed_pb_info->mutable_sample_param()->set_weight_leading_safe_v(
      weight_leading_safe_v_);
  sample_poly_speed_pb_info->mutable_sample_param()->set_weight_vel_variable(
      weight_vel_variable_);
  sample_poly_speed_pb_info->mutable_sample_param()->set_weight_gap_avaliable(
      weight_gap_avaliable_);
}
}  // namespace planning