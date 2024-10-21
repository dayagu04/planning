#include "speed_adjust_decider.h"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <random>
#include <unordered_map>
#include <utility>
#include <vector>

#include "behavior_planners/lane_change_decider/lane_change_state_machine_manager.h"
#include "config/basic_type.h"
#include "debug_info_log.h"
#include "dynamic_world/dynamic_agent_node.h"
#include "dynamic_world/dynamic_world.h"
#include "ego_planning_config.h"
#include "lateral_obstacle.h"
#include "lon_behavior_planner.pb.h"
#include "planning_context.h"
#include "planning_debug_info.pb.h"
#include "reference_path.h"
#include "session.h"
#include "st_search_decider.pb.h"
#include "task.h"

using planning::planning_data::DynamicAgentNode;
using planning::planning_data::DynamicWorld;

namespace {
constexpr int kPlanningHorizions = 26;
constexpr double kFarawayObjFilterDis = 105.0;
constexpr double kPenaltyMinDecAdjustSpeed = 8.0 / 3.6;
constexpr double kMaxExceedAdjustSpeed = 12.0 / 3.6;
constexpr double kPlanningDuration = 5.0;
constexpr int kTimeHorizion = 26;
constexpr double kPlanningStep = 0.2;
constexpr double kEgoHalfLength = 2.6;
constexpr double kForwardRange = 150.0;
constexpr double kBackwardRange = 100.0;
constexpr double kMinSlotLength = 15.0;
constexpr int32_t kNoAgentId = -1;
constexpr double kStaticCarFilterVel = 1.5;
constexpr double kAlignedDistanceBuffer = 2.5;
constexpr double kDistanceToMapRequestPoint = 120.0;
}  // namespace

namespace planning {

SpeedAdjustDecider::SpeedAdjustDecider(
    const EgoPlanningConfigBuilder* config_builder, framework::Session* session)
    : Task(config_builder, session) {
  name_ = "SpeedAdjustDecider";
  config_ = config_builder->cast<SpeedAdjustDeciderConfig>();
  speed_adjust_status_buffer_ = {false, false, false};
  vehicle_param_ = VehicleConfigurationContext::Instance()->get_vehicle_param();

  max_acc_ego_v_bp_.emplace_back(config_.max_acc_limit_upper);
  max_acc_ego_v_bp_.emplace_back(config_.max_acc_limit_lower);
  max_jerk_ego_v_bp_.emplace_back(config_.max_jerk_limit_upper);
  max_jerk_ego_v_bp_.emplace_back(config_.max_jerk_limit_lower);
  max_v_max_ego_v_bp_.emplace_back(config_.max_acc_adjust_ratio_upper);
  max_v_max_ego_v_bp_.emplace_back(config_.max_acc_adjust_ratio_lower);
  min_acc_ego_v_bp_.emplace_back(config_.min_acc_limit_lower);
  min_acc_ego_v_bp_.emplace_back(config_.min_acc_limit_upper);
  min_jerk_ego_v_bp_.emplace_back(config_.min_jerk_limit_lower);
  min_jerk_ego_v_bp_.emplace_back(config_.min_jerk_limit_upper);
}

void SpeedAdjustDecider::ClearStatus() {
  count_wait_state_ = 0;
  last_best_slot_id_ = -1;
  speed_adjust_status_buffer_.current_frame_status = false;
}

bool SpeedAdjustDecider::ProcessLaneChangeStatus() {
  auto* speed_decider_pb_info = DebugInfoManager::GetInstance()
                                    .GetDebugInfoPb()
                                    ->mutable_st_search_decider_info();
  speed_decider_pb_info->Clear();
  speed_decider_pb_info->set_front_gap_id(kNoAgentId);
  speed_decider_pb_info->set_back_gap_id(kNoAgentId);
  for (auto i = 0; i < kPlanningHorizions; i++) {
    speed_decider_pb_info->add_search_s_vec(0.0);
    speed_decider_pb_info->add_search_t_vec(0.0);
    speed_decider_pb_info->add_search_v_vec(0.0);
    speed_decider_pb_info->add_search_a_vec(0.0);
    speed_decider_pb_info->add_search_j_vec(0.0);
  }
  variable_time_optimal_trajs_.clear();

  session_->mutable_planning_context()
      ->mutable_lane_change_decider_output()
      .st_search_vec.clear();
  slot_changed_ = false;

  // preprocess
  speed_adjust_status_buffer_.last_two_frame_status =
      speed_adjust_status_buffer_.last_frame_status;
  speed_adjust_status_buffer_.last_frame_status =
      speed_adjust_status_buffer_.current_frame_status;

  const auto& coarse_planning_info = session_->planning_context()
                                         .lane_change_decider_output()
                                         .coarse_planning_info;
  const auto& lc_request =
      session_->planning_context().lane_change_decider_output().lc_request;
  const auto& ego_state_manager =
      session_->environmental_model().get_ego_state_manager();
  const auto& ego_point = ego_state_manager->ego_pose();
  const auto& virtual_lane_mgr =
      session_->environmental_model().get_virtual_lane_manager();
  const auto& lc_request_source = session_->planning_context()
                                      .lane_change_decider_output()
                                      .lc_request_source;
  const auto& is_merge_region = session_->mutable_planning_context()
                                    ->mutable_lane_change_decider_output()
                                    .is_merge_region;

  if (coarse_planning_info.target_state != kLaneChangePropose) {
    ClearStatus();
    last_request_ = lc_request;
    session_->mutable_planning_context()
        ->mutable_lane_change_decider_output()
        .s_search_status = false;
    speed_adjust_status_buffer_.current_frame_status = false;
    return false;
  }

  if (coarse_planning_info.target_state == kLaneChangePropose &&
      lc_request == last_request_) {
    count_wait_state_++;
  } else {
    count_wait_state_ = 0;
  }

  // judge the lane change source and scene
  boundary_merge_point_valid_ = session_->planning_context()
                                    .lane_change_decider_output()
                                    .boundary_merge_point_valid;
  if (boundary_merge_point_valid_ && lc_request_source == MERGE_REQUEST) {
    const auto& boundary_merge_point = session_->planning_context()
                                           .lane_change_decider_output()
                                           .boundary_merge_point;
    deceleration_priority_scene_ = true;
    merge_emegency_distance_ = std::hypot(ego_point.x - boundary_merge_point.x,
                                          ego_point.y - boundary_merge_point.y);
  } else if (lc_request_source == MAP_REQUEST) {
    const double& distance_to_road_merge =
        virtual_lane_mgr->get_distance_to_first_road_merge();
    const double& distance_to_road_split =
        virtual_lane_mgr->get_distance_to_first_road_split();
    if (distance_to_road_split < kDistanceToMapRequestPoint ||
        distance_to_road_merge < kDistanceToMapRequestPoint ||
        is_merge_region) {
      deceleration_priority_scene_ = true;
      merge_emegency_distance_ =
          std::fmin(distance_to_road_split, distance_to_road_merge);
    }
  } else {
    deceleration_priority_scene_ = false;
    merge_emegency_distance_ =
        std::numeric_limits<double>::max();  // cailiu2:no use currently
  }

  if (count_wait_state_ < 3) {
    std::cout << " pass lc wait state, count is: " << count_wait_state_
              << std::endl;
    last_request_ = lc_request;
    session_->mutable_planning_context()
        ->mutable_lane_change_decider_output()
        .s_search_status = false;
    speed_adjust_status_buffer_.current_frame_status = false;
    return false;
  } else {
    last_request_ = lc_request;
    speed_adjust_status_buffer_.current_frame_status = true;
    return true;
  }
  return true;
}

void SpeedAdjustDecider::ProcessEnvInfos() {
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

  // leading veh
  leading_veh_ = LaneChangeVehInfo();
  if (lead_one != nullptr) {
    if (lead_one->track_id > 0) {
      leading_veh_.half_length = lead_one->length * 0.5;
      leading_veh_.id = lead_one->track_id;
      leading_veh_.v = lead_one->v;
      leading_veh_.center_s = lead_one->d_rel;
    }
  }

  // target lane ids
  const std::vector<const DynamicAgentNode*> target_lane_nodes =
      dynamic_world->GetNodesByLaneId(target_lane_virtual_id);
  lane_change_veh_info_id_map_.clear();
  lane_change_veh_info_.clear();
  front_target_lane_id_set_.clear();
  rear_target_lane_id_set_.clear();
  for (const auto* target_lane_node : target_lane_nodes) {
    LaneChangeVehInfo target_lane_veh_info;
    target_lane_veh_info.id = target_lane_node->node_agent_id();
    target_lane_veh_info.v = target_lane_node->node_speed();
    target_lane_veh_info.center_s = target_lane_node->node_s() - ego_s;
    target_lane_veh_info.half_length = target_lane_node->node_length() * 0.5;
    lane_change_veh_info_.emplace_back(target_lane_veh_info);
    lane_change_veh_info_id_map_[target_lane_node->node_agent_id()] =
        std::move(target_lane_veh_info);
  }

  std::sort(lane_change_veh_info_.begin(), lane_change_veh_info_.end(),
            [&](LaneChangeVehInfo& a, LaneChangeVehInfo& b) -> bool {
              return a.center_s < b.center_s;
            });

  for (auto i = 0; i < lane_change_veh_info_.size(); i++) {
    if (lane_change_veh_info_[i].center_s > 0.) {
      front_target_lane_id_set_.insert(lane_change_veh_info_[i].id);
    } else {
      rear_target_lane_id_set_.insert(lane_change_veh_info_[i].id);
    }
  }

  if (!lane_change_veh_info_.empty()) {
    const auto& elem_back = lane_change_veh_info_.back();
    if (elem_back.center_s + elem_back.half_length + kMinSlotLength <
        kForwardRange) {  // add a front range virtual obj
      LaneChangeVehInfo slot_elem;
      slot_elem.id = kNoAgentId - 1;
      slot_elem.center_s = kForwardRange;
      slot_elem.half_length = elem_back.half_length;
      slot_elem.v = 50.0;
      lane_change_veh_info_.emplace_back(std::move(slot_elem));
    }
    const auto& elem = lane_change_veh_info_.front();
    if (elem.center_s - kMinSlotLength - elem.half_length > -kBackwardRange) {
      LaneChangeVehInfo slot_elem;
      slot_elem.id = kNoAgentId - 2;
      slot_elem.center_s = -kBackwardRange;
      slot_elem.half_length = elem.half_length;
      slot_elem.v = 0.;
      lane_change_veh_info_.insert(lane_change_veh_info_.begin(),
                                   std::move(slot_elem));
    }
  }

  // ego state info
  init_sl_.first = ego_s;
  init_sl_.second = ego_frenet_state.l();
  init_va_.first = ego_state_manager->ego_v();
  init_va_.second = ego_state_manager->ego_acc();
  v_cruise_ = ego_state_manager->ego_v_cruise();

  if (speed_adjust_status_buffer_.current_frame_status == true &&
      speed_adjust_status_buffer_.last_frame_status == false) {
    retriggered_ego_speed_ = ego_state_manager->ego_v();
    origin_ego_speed_ = ego_state_manager->ego_v();
    max_ego_speed_in_speed_adjust_ = ego_state_manager->ego_v();
    min_ego_speed_in_speed_adjust_ = ego_state_manager->ego_v();
  }

  CalcTargetObjsFlowVel();
}

std::pair<double, double> SpeedAdjustDecider::GetSafeAlignedDistance(
    const double& ego_v, const SlotInfo& slot) {
  const auto& front_car = slot.front_veh_info();
  const auto& rear_car = slot.back_veh_info();

  auto headway_distace = [](const double& headway_v, const double ego_v,
                            const std::vector<double>& t_gap_ego_v_bp,
                            const std::vector<double>& t_gap_ego_v) -> double {
    double v_lead_clip = std::max(headway_v, 0.0);

    double t_gap = interp(ego_v, t_gap_ego_v_bp, t_gap_ego_v);
    t_gap = t_gap * (0.6 * ego_v * 0.01);  // why?
    double v_rel = std::min(std::max(ego_v - v_lead_clip, 0.0), 5.0);
    double distance_hysteresis = v_rel * 0.3;
    double fix_safe_distance = 5.5;
    return fix_safe_distance + t_gap * v_lead_clip + distance_hysteresis;
  };

  double aligned_front_safe_distance =
      headway_distace(slot.front_veh_info().v, ego_v, t_gap_ego_v_bp_,
                      t_gap_ego_v_) +
      front_car.half_length + kEgoHalfLength;
  double aligned_rear_safe_distance =
      headway_distace(ego_v, slot.back_veh_info().v, t_gap_ego_v_bp_,
                      t_gap_ego_v_) +
      rear_car.half_length + kEgoHalfLength;
  return std::make_pair(aligned_front_safe_distance,
                        aligned_rear_safe_distance);
}

bool SpeedAdjustDecider::GenerateCandidateSlotInfo() {
  slot_point_info_.clear();

  auto tailgating_leading_car = [this](const LaneChangeVehInfo& leading_veh,
                                       const SlotInfo& slot) -> bool {
    const double leading_veh_pred_s = leading_veh.center_s + init_sl_.first +
                                      leading_veh.v * kPlanningDuration;
    const double slot_aligned_raw_s =
        slot.aligned_s() + slot.aligned_v() * kPlanningDuration;
    const double safety_distance =
        std::fmax(init_va_.first * init_va_.first * 0.02 + 2.5,
                  6.0);  // ref: lane_change_state machine;
    const double pred_s_error = leading_veh_pred_s - slot_aligned_raw_s -
                                kEgoHalfLength - leading_veh.half_length -
                                safety_distance;
    return pred_s_error < 0;
  };

  auto static_slot_car = [](const SlotInfo& slot) -> bool {
    if (slot.front_veh_info().v < kStaticCarFilterVel) return true;
    return false;
  };

  auto filter_in_deceleration_priority_scene =
      [this](const SlotInfo& slot,
             const double& aligned_back_safe_dis) -> bool {
    return slot.aligned_s() > init_sl_.first;
  };

  auto far_away_front_obj = [&](const SlotInfo& slot) -> bool {
    return slot.front_veh_info().center_s -
               (init_va_.first - slot.front_veh_info().v) * kPlanningDuration -
               vehicle_param_.length * 0.5 - slot.front_veh_info().half_length >
           kFarawayObjFilterDis;
  };

  auto calc_slot_v = [&](SlotInfo& slot) -> void {
    if (slot.front_veh_info().id < 0) {
      slot.SetSlotV(slot.back_veh_info().v);
      return;
    } else if (slot.back_veh_info().id < 0) {
      slot.SetSlotV(slot.front_veh_info().v);
    } else {
      slot.SetSlotV(slot.front_veh_info().v > slot.back_veh_info().v
                        ? slot.back_veh_info().v
                        : slot.front_veh_info().v);
    }
  };

  for (auto idx = 1; idx < lane_change_veh_info_.size(); ++idx) {
    SlotInfo slot(lane_change_veh_info_[idx], lane_change_veh_info_[idx - 1]);
    // safety gap distance
    double safety_distance = std::fmax(  // MSFD: (v^2 / 2) / (2* dec)
        init_va_.first * init_va_.first * 0.02 + vehicle_param_.length * 0.5,
        6.0);
    // 1. filter too tight gap
    if (slot.front_veh_info().center_s - slot.front_veh_info().half_length -
            slot.back_veh_info().center_s - slot.back_veh_info().half_length -
            (slot.back_veh_info().v - slot.front_veh_info().v) *
                kPlanningDuration <=
        2 * safety_distance) {
      std::cout << "The slot front id: " << slot.front_veh_info().id
                << ", slot rear id: " << slot.back_veh_info().id
                << " has been filtered!" << std::endl;
      continue;
    }

    // 2. calc headway distance
    std::pair<double, double> safe_distacne_pair =
        GetSafeAlignedDistance(init_va_.first, slot);

    const double& aligned_front_s = slot.front_veh_info().center_s -
                                    safe_distacne_pair.first -
                                    kAlignedDistanceBuffer;
    const double& aligned_back_s = slot.back_veh_info().center_s +
                                   safe_distacne_pair.second +
                                   kAlignedDistanceBuffer;
    if (slot.back_veh_info().center_s > 0.) {
      slot.SetAlignedFront(false);
      slot.SetAlignedS(aligned_back_s + init_sl_.first);
    } else if (slot.front_veh_info().center_s < 0.0) {
      slot.SetAlignedFront(true);
      slot.SetAlignedS(aligned_front_s + init_sl_.first);
    } else {
      if (aligned_front_s > 0.0 && aligned_back_s < 0.0) {
        slot.SetAlignedS(init_sl_.first);  // ego aligned
        if (fabs(init_va_.first - slot.front_veh_info().v) <
            fabs(init_va_.first - slot.back_veh_info().v)) {
          slot.SetAlignedFront(slot.front_veh_info().v < v_cruise_);
        } else {
          slot.SetAlignedFront(slot.back_veh_info().v >= v_cruise_);
        }
      } else {
        slot.SetAlignedFront(fabs(aligned_front_s) < fabs(aligned_back_s));
        slot.SetAlignedS(slot.is_align_front()
                             ? aligned_front_s + init_sl_.first
                             : aligned_back_s + init_sl_.first);
      }
    }

    // step3: calc slot v
    const double fv = slot.front_veh_info().v;
    const double bv = slot.back_veh_info().v;
    if (fv > bv) {
      const bool is_far_away_front_obj = far_away_front_obj(slot);
      slot.SetAlignedV(std::min(
          slot.front_veh_info().id < 0 || is_far_away_front_obj ? 40.0 : fv,
          std::max(init_va_.first, bv)));

    } else {
      slot.SetAlignedV(slot.is_align_front()
                           ? std::fmin(slot.front_veh_info().id < 0 ? 40.0 : fv,
                                       init_va_.first)
                           : bv);
    }

    // v cruise limit
    slot.SetAlignedV(std::fmin(
        slot.aligned_v(), v_cruise_ * interp(init_va_.first, max_v_max_ego_v_,
                                             max_v_max_ego_v_bp_)));
    if (deceleration_priority_scene_) {
      if (filter_in_deceleration_priority_scene(slot,
                                                safe_distacne_pair.second)) {
        std::cout << " The slot: <<" << idx
                  << " is filter by deceleration priority scene " << std::endl;
        continue;
      }
    }

    double min_dec_filter_speed = config_.min_dec_filter_speed;
    if (deceleration_priority_scene_) {
      min_dec_filter_speed = config_.min_dec_filter_speed_in_deceleration_scene;
    }

    // calc slot v
    calc_slot_v(slot);

    // lower aligned v limit
    if (slot.aligned_v() <
            min_ego_speed_in_speed_adjust_ - min_dec_filter_speed / 3.6 ||
        slot.aligned_v() > max_ego_speed_in_speed_adjust_ +
                               config_.max_acc_filter_speed / 3.6) {
      std::cout << " The slot: <<" << idx << " is too slow" << std::endl;
      continue;
    }
    if (leading_veh_.id > 0) {
      if (tailgating_leading_car(leading_veh_, slot)) {
        std::cout << " The slot: " << idx << " is tailgating leading car!"
                  << " front car id: " << slot.front_veh_info().id << std::endl;
        continue;
      }
    }
    if (static_slot_car(slot)) {
      std::cout << " The slot is static, continue, front id: "
                << slot.front_veh_info().id << std::endl;
      continue;
    }
    slot_point_info_.emplace_back(std::move(slot));
  }

  return slot_point_info_.size() > 0;
}

void SpeedAdjustDecider::GenerateTimeOptimalAdjustProfile() {
  for (auto i = 0; i < slot_point_info_.size(); i++) {
    const double rel_coord_vel = slot_point_info_[i].aligned_v();
    CoordinateParam slot_aligned_coord{slot_point_info_[i].aligned_s(),
                                       rel_coord_vel};
    LonState relative_init_state;
    relative_init_state.p = init_sl_.first;
    relative_init_state.v = init_va_.first;
    relative_init_state.a = init_va_.second;

    StateLimit relative_state_limit;
    relative_state_limit.a_max =
        interp(init_va_.first, max_acc_ego_v_, max_acc_ego_v_bp_);
    relative_state_limit.a_min =
        deceleration_priority_scene_
            ? config_.min_acc_limit_lower
            : interp(init_va_.first, min_acc_ego_v_, min_acc_ego_v_bp_);
    relative_state_limit.v_min =
        std::fmax(std::fmax(std::fmin(retriggered_ego_speed_, init_va_.first),
                            origin_ego_speed_) -
                      config_.min_dec_adjust_limit / 3.6,
                  0.0);
    relative_state_limit.v_max =
        v_cruise_ *
        interp(init_va_.first, max_v_max_ego_v_, max_v_max_ego_v_bp_);
    relative_state_limit.p_end = 0.0;
    relative_state_limit.j_max =
        interp(init_va_.first, max_jerk_ego_v_, max_jerk_ego_v_bp_);
    relative_state_limit.j_min =
        deceleration_priority_scene_
            ? config_.min_jerk_limit_lower
            : interp(init_va_.first, min_jerk_ego_v_, min_jerk_ego_v_bp_);
    auto speed_adjust_curve =
        VariableCoordinateTimeOptimalTrajectory::ConstructInstance(
            relative_init_state, relative_state_limit, slot_aligned_coord, 0.3);

    variable_time_optimal_trajs_.emplace_back(std::move(speed_adjust_curve));
  }
  return;
}
bool SpeedAdjustDecider::Execute() {
  auto speed_decider_pb_info = DebugInfoManager::GetInstance()
                                   .GetDebugInfoPb()
                                   ->mutable_st_search_decider_info();
  speed_decider_pb_info->Clear();
  if (!ProcessLaneChangeStatus()) {
    std::cout << " No speed adjust status!" << std::endl;
    session_->mutable_planning_context()
        ->mutable_lane_change_decider_output()
        .s_search_status = false;
    speed_decider_pb_info->set_st_search_status(false);
    return true;
  }

  ProcessEnvInfos();

  if (!GenerateCandidateSlotInfo()) {
    std::cout << " The slot is empty!" << std::endl;
    session_->mutable_planning_context()
        ->mutable_lane_change_decider_output()
        .s_search_status = false;
    speed_decider_pb_info->set_st_search_status(false);
    return true;
  }

  GenerateTimeOptimalAdjustProfile();
  int best_id = SelectBestSlot();

  GenerateAdjustTraj(best_id, &session_->mutable_planning_context()
                                   ->mutable_lane_change_decider_output()
                                   .st_search_vec);

  session_->mutable_planning_context()
      ->mutable_lane_change_decider_output()
      .s_search_status = true;
  speed_decider_pb_info->set_st_search_status(true);
  speed_decider_pb_info->set_front_gap_id(last_best_slot_.front_veh_info().id);
  speed_decider_pb_info->set_back_gap_id(last_best_slot_.back_veh_info().id);

  // proto info store
  if (session_->mutable_planning_context()
          ->mutable_lane_change_decider_output()
          .s_search_status) {
    // store search res;
    auto& s_vec = session_->mutable_planning_context()
                      ->mutable_lane_change_decider_output()
                      .st_search_vec;
    const auto& best_profile = variable_time_optimal_trajs_[best_id];
    for (auto i = 0; i < s_vec.size(); i++) {
      speed_decider_pb_info->add_search_s_vec(s_vec[i]);
      speed_decider_pb_info->add_search_t_vec(i * kPlanningStep);
      speed_decider_pb_info->add_search_v_vec(
          best_profile.Evaluate(1, i * 0.2));
      speed_decider_pb_info->add_search_a_vec(
          best_profile.Evaluate(2, i * 0.2));
      speed_decider_pb_info->add_search_j_vec(
          best_profile.Evaluate(3, i * 0.2));
    }
  }
  for (auto& lane_change_veh : lane_change_veh_info_) {
    planning::common::LaneChangeVehInfo* lane_change_veh_pb =
        speed_decider_pb_info->add_lane_change_veh_info();
    lane_change_veh_pb->set_id(lane_change_veh.id);
    lane_change_veh_pb->set_center_s(lane_change_veh.center_s);
    lane_change_veh_pb->set_half_length(lane_change_veh.half_length);
    lane_change_veh_pb->set_v(lane_change_veh.v);

    planning::common::ObstacleSTInfo* lane_change_obs_st_info =
        speed_decider_pb_info->add_obstacle_st_infos();
    lane_change_obs_st_info->set_id(lane_change_veh.id);
    for (auto i = 0; i < kTimeHorizion; i++) {
      double s = lane_change_veh.center_s + i * lane_change_veh.v * 0.2 +
                 init_sl_.first;
      lane_change_obs_st_info->add_s_vec_upper(s + lane_change_veh.center_s);
      lane_change_obs_st_info->add_s_vec_lower(s - lane_change_veh.center_s);
      lane_change_obs_st_info->add_t_vec(i * 0.2);
    }
  }

  for (auto& slot_info : slot_point_info_) {
    planning::common::LaneChangeSlotInfo* lane_change_slot_pb =
        speed_decider_pb_info->add_slot_info_vec();
    lane_change_slot_pb->set_slot_v(slot_info.slot_v());
    lane_change_slot_pb->set_aligned_v(slot_info.aligned_v());
    lane_change_slot_pb->set_aligned_s(slot_info.aligned_s());
    lane_change_slot_pb->set_cost(slot_info.cost());
    lane_change_slot_pb->set_front_veh_id(slot_info.front_veh_info().id);
    lane_change_slot_pb->set_back_veh_id(slot_info.back_veh_info().id);
    lane_change_slot_pb->set_is_aligned_front(slot_info.is_align_front());
  }

  speed_decider_pb_info->mutable_leading_veh_info()->set_v(leading_veh_.v);
  speed_decider_pb_info->mutable_leading_veh_info()->set_id(leading_veh_.id);
  speed_decider_pb_info->mutable_leading_veh_info()->set_center_s(
      leading_veh_.center_s);
  speed_decider_pb_info->mutable_leading_veh_info()->set_half_length(
      leading_veh_.half_length);
  speed_decider_pb_info->set_ego_s(init_sl_.first);
  speed_decider_pb_info->set_ego_l(init_sl_.second);
  speed_decider_pb_info->set_ego_v(init_va_.first);
  speed_decider_pb_info->set_ego_a(init_va_.second);
  speed_decider_pb_info->set_ego_v_cruise(v_cruise_);
  speed_decider_pb_info->set_target_objs_flow_vel(target_objs_flow_vel_);
  speed_decider_pb_info->set_slot_changed(slot_changed_);
  return true;
}

int SpeedAdjustDecider::SelectBestSlot() {
  std::vector<double> slot_costs;
  int best_slot_id = -1;
  double min_cost = std::numeric_limits<double>::max();
  slot_costs.resize(variable_time_optimal_trajs_.size(), 0.0);

  // double speed_adjust_weight = 1.0;
  double v_adjust_weight = 1.5;
  double s_dis_adjust_weight = 0.35;
  double same_slot_ratio = 0.45;
  double acc_excced_weight = 4.0;
  double dec_excced_weight = 5.0;

  // double rear_faster_coming_car_weight = 7.5;

  auto is_same_slot = [](const SlotInfo& slot_1,
                         const SlotInfo& slot_2) -> bool {
    const bool front_match =
        slot_1.front_veh_info().id ==
        slot_2.front_veh_info().id;  // consider cut out slot
    const bool back_match = slot_1.back_veh_info().id ==
                            slot_2.back_veh_info().id;  // consider cut in slot
    return front_match && back_match;
  };

  for (auto i = 0; i < variable_time_optimal_trajs_.size(); i++) {
    // 1. speed adjust cost
    // 2. aligned v cost
    const auto& slot = slot_point_info_[i];
    const double aligned_v_cost =
        std::fabs(slot.aligned_v() - init_va_.first) * v_adjust_weight;
    slot_costs[i] += aligned_v_cost;

    // 3.1 aligned v excced cost
    const double aligned_v_dec_penaty =
        slot.aligned_v() - init_va_.first >= -kPenaltyMinDecAdjustSpeed ||
                deceleration_priority_scene_
            ? 0.0
            : (init_va_.first - slot.aligned_v()) * dec_excced_weight;
    slot_costs[i] += aligned_v_dec_penaty;

    // 3.2 aligned v acc max cost
    const double aligned_v_acc_penaty =
        slot.aligned_v() - init_va_.first > kMaxExceedAdjustSpeed
            ? (slot.aligned_v() - init_va_.first) * acc_excced_weight
            : 0.0;
    slot_costs[i] += aligned_v_acc_penaty;
    // 3. aligned s cost
    const double pred_aligned_time = std::fmax(
        variable_time_optimal_trajs_[i].SecondOrderParamLength(), 0.0);
    const double pred_aligned_rel_dis =
        slot.is_align_front()
            ? (slot.front_veh_info().v - init_va_.first) * pred_aligned_time
            : (slot.back_veh_info().v - init_va_.first) * pred_aligned_time;
    const double s_dis_adjust_cost =
        std::fabs(slot.aligned_s() + pred_aligned_rel_dis - init_sl_.first) *
        s_dis_adjust_weight;
    slot_costs[i] += s_dis_adjust_cost;

    // 4. consider rear faster coming car
    // const auto &back_car = slot.back_info;
    // double rear_faster_coming_cost = 0.0;
    // if (back_car.center_s < 0 && back_car.v > init_va_.first) {
    //   const double adjust_length = candidate_traj.SecondOrderParamLength();
    //   const double ttc = back_car.center_s / (init_va_.first - back_car.v);
    //   if (ttc < adjust_length - kSafeAdjustDuration) {
    //     rear_faster_coming_cost = (adjust_length - kSafeAdjustDuration -
    //     ttc)
    //     *
    //                               rear_faster_coming_car_weight;
    //   }
    // }
    // slot_costs[i] += rear_faster_coming_cost;
    // 5. last same cost;
    slot_costs[i] = is_same_slot(slot, last_best_slot_)
                        ? slot_costs[i] * same_slot_ratio
                        : slot_costs[i];

    if (slot_costs[i] < min_cost) {
      best_slot_id = i;
      min_cost = slot_costs[i];
    }
    slot_point_info_[i].SetSlotCost(slot_costs[i]);
  }

  if (!is_same_slot(slot_point_info_[best_slot_id], last_best_slot_)) {
    std::cout
        << "origin ego speed has been register again, beacuse slot changed!"
        << std::endl;
    retriggered_ego_speed_ = init_va_.first;
    max_ego_speed_in_speed_adjust_ =
        std::fmax(max_ego_speed_in_speed_adjust_, init_va_.first);
    min_ego_speed_in_speed_adjust_ =
        std::fmax(std::fmin(min_ego_speed_in_speed_adjust_, init_va_.first),
                  origin_ego_speed_);
    slot_changed_ = true;
  }
  last_best_slot_ = slot_point_info_[best_slot_id];

  return best_slot_id;
}

void SpeedAdjustDecider::GenerateAdjustTraj(int best_slot_id,
                                            std::vector<double>* search_path) {
  search_path->reserve(kPlanningHorizions);
  const auto& best_profile = variable_time_optimal_trajs_[best_slot_id];
  for (size_t i = 0; i < kPlanningHorizions; i++) {
    double s = best_profile.Evaluate(0, i * 0.2) - init_sl_.first;
    search_path->emplace_back(std::move(s));
  }
}

void SpeedAdjustDecider::CalcTargetObjsFlowVel() {
  if (lane_change_veh_info_.size() < 2) {
    target_objs_flow_vel_ = v_cruise_;
    return;
  }

  double d_norm = 0.0;
  for (size_t i = 0; i < lane_change_veh_info_.size(); i++) {
    if (lane_change_veh_info_[i].id < 0) {
      continue;
    }
    d_norm += std::fabs(lane_change_veh_info_[i].center_s);
  }

  double d_norm_inverse = 1 / d_norm;
  double temp_sum = 0.0;
  for (size_t i = 0; i < lane_change_veh_info_.size(); i++) {
    if (lane_change_veh_info_[i].id < 0) {
      continue;
    }
    temp_sum += std::fabs(lane_change_veh_info_[i].center_s) * d_norm_inverse *
                lane_change_veh_info_[i].v;
  }
  target_objs_flow_vel_ = temp_sum;
}

}  // namespace planning