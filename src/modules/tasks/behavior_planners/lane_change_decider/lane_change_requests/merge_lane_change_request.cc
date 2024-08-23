#include "merge_lane_change_request.h"

#include <glog/logging.h>
#include <math.h>

#include <cassert>
#include <cmath>
#include <complex>
#include <limits>

#include "common.pb.h"
#include "config/basic_type.h"
#include "debug_info_log.h"
#include "define/geometry.h"
#include "ego_state_manager.h"
#include "log.h"
#include "planning_context.h"
#include "tasks/behavior_planners/lane_change_decider/lane_change_requests/lane_change_request.h"
#include "tracked_object.h"
#include "virtual_lane_manager.h"

namespace planning {

namespace {
constexpr uint32_t kMergeAlcCountThre = 3;
constexpr int kMergeAlcCountLowerThre = 0;

}  // namespace
// class: MergeRequest
MergeRequest::MergeRequest(
    planning::framework::Session* session,
    std::shared_ptr<VirtualLaneManager> virtual_lane_mgr,
    std::shared_ptr<LaneChangeLaneManager> lane_change_lane_mgr)
    : LaneChangeRequest(session, virtual_lane_mgr, lane_change_lane_mgr) {
  base_frenet_coord_ = std::make_shared<KDPath>();
}

void MergeRequest::Update(int lc_status) {
  std::cout << "MergeRequest::Update::coming merge lane change request"
            << std::endl;

  // trigger merge lane change when lane keep status.
  if (lc_status != kLaneKeeping && lc_status != kLaneChangePropose) {
    LOG_DEBUG("MergeRequest::Update: ego not in lane keeping!");
    return;
  }

  const auto& ego_state =
      session_->environmental_model().get_ego_state_manager();
  planning_init_point_ = ego_state->planning_init_point();
  const auto& lane_change_decider_output =
      session_->planning_context().lane_change_decider_output();
  int current_lane_virtual_id = virtual_lane_mgr_->current_lane_virtual_id();
  int fix_lane_virtual_id = lane_change_decider_output.fix_lane_virtual_id;
  int olane_virtual_id = lane_change_decider_output.origin_lane_virtual_id;
  int target_lane_virtual_id =
      lane_change_decider_output.target_lane_virtual_id;
  std::shared_ptr<ReferencePathManager> reference_path_mgr =
      session_->mutable_environmental_model()->get_reference_path_manager();
  auto flane = virtual_lane_mgr_->get_lane_with_virtual_id(fix_lane_virtual_id);
  auto olane = virtual_lane_mgr_->get_lane_with_virtual_id(olane_virtual_id);
  auto tlane =
      virtual_lane_mgr_->get_lane_with_virtual_id(target_lane_virtual_id);
  const auto& clane = virtual_lane_mgr_->get_current_lane();
  const auto& llane = virtual_lane_mgr_->get_left_lane();
  const auto& rlane = virtual_lane_mgr_->get_right_lane();
  if (llane != nullptr) {
    left_reference_path_ = reference_path_mgr->get_reference_path_by_lane(
        llane->get_virtual_id(), false);
    LOG_DEBUG("MergeRequest::Update: for left_lane: update %d\n",
              llane->get_virtual_id());
  } else {
    left_reference_path_ = nullptr;
  }
  if (rlane != nullptr) {
    right_reference_path_ = reference_path_mgr->get_reference_path_by_lane(
        rlane->get_virtual_id(), false);
    LOG_DEBUG("MergeRequest::Update: for right_lane: update %d\n",
              rlane->get_virtual_id());
  } else {
    right_reference_path_ = nullptr;
  }
  enable_l_ = llane && left_reference_path_;
  enable_r_ = rlane && right_reference_path_;

  if (lane_change_lane_mgr_->has_origin_lane()) {
    auto origin_lane = lane_change_lane_mgr_->olane();
    origin_lane_virtual_id_ = origin_lane->get_virtual_id();
  } else {
    origin_lane_virtual_id_ = current_lane_virtual_id;
  }
  is_merge_lane_change_situation_ = false;

  UpdateLaneMergeSituation(lc_status);
  LOG_DEBUG("MergeRequest::Update: is_merge_lane_change_situation_ %d",
            is_merge_lane_change_situation_);
  JSON_DEBUG_VALUE("is_merge_lane_change_situation_",
                   is_merge_lane_change_situation_);
  JSON_DEBUG_VALUE("merge_alc_trigger_counter_", 
                    merge_alc_trigger_counter_);

  if (!is_merge_lane_change_situation_) {
    if (request_type_ != NO_CHANGE &&
        (lane_change_lane_mgr_->has_origin_lane() &&
         lane_change_lane_mgr_->is_ego_on(olane))) {
      Finish();
      Reset();
      set_target_lane_virtual_id(origin_lane_virtual_id_);
      LOG_DEBUG(
          "[MergeRequest::update] %s:%d finish request, "
          "!trigger_left_clc and !trigger_right_clc\n",
          __FUNCTION__, __LINE__);
    }
    return;
  }

  setLaneChangeRequestByMerge(lc_status);
  LOG_DEBUG("request_type_: [%d] turn_signal_: [%d]\n", request_type_,
            turn_signal_);
}

void MergeRequest::UpdateLaneMergeSituation(int lc_status) {
  const auto& ego_state =
      session_->environmental_model().get_ego_state_manager();
  const auto base_lane =
      virtual_lane_mgr_->get_lane_with_virtual_id(origin_lane_virtual_id_);
  if (base_lane == nullptr) {
    LOG_DEBUG("base lane not exist");
    is_merge_lane_change_situation_ = false;
    return;
  }
  std::shared_ptr<ReferencePath> origin_refline =
      session_->mutable_environmental_model()
          ->get_reference_path_manager()
          ->get_reference_path_by_lane(origin_lane_virtual_id_, false);

  base_frenet_coord_ = origin_refline->get_frenet_coord();
  Point2D ego_frenet_point;
  Point2D ego_cart_point{planning_init_point_.lat_init_state.x(),
                         planning_init_point_.lat_init_state.y()};
  if (!base_frenet_coord_->XYToSL(ego_cart_point, ego_frenet_point)) {
    LOG_DEBUG("fail to get ego position on base lane");
    is_merge_lane_change_situation_ = false;
    return;
  }
  const auto& lane_change_decider_output =
      session_->planning_context().lane_change_decider_output();
  const bool is_merge_region = lane_change_decider_output.is_merge_region;
  const int lane_nums = virtual_lane_mgr_->get_lane_num();
  const auto& current_lane = virtual_lane_mgr_->get_current_lane();
  const int current_lane_order_id = current_lane->get_order_id();
  bool is_edge_side_lane = 
      (current_lane_order_id == 0 || current_lane_order_id == lane_nums - 1);

  if (is_edge_side_lane && is_merge_region) {
    merge_alc_trigger_counter_++;
  } else {
    merge_alc_trigger_counter_ =
        std::max(merge_alc_trigger_counter_ - 1, kMergeAlcCountLowerThre);
    is_merge_lane_change_situation_ = false;
    return;
  }
  if (merge_alc_trigger_counter_ >= kMergeAlcCountThre) {
    is_merge_lane_change_situation_ = true;
    return;
  }
  return;
}

void MergeRequest::setLaneChangeRequestByMerge(int lc_status) {
  const auto& lane_change_decider_output =
      session_->planning_context().lane_change_decider_output();
  int olane_virtual_id = lane_change_decider_output.origin_lane_virtual_id;
  auto olane = virtual_lane_mgr_->get_lane_with_virtual_id(olane_virtual_id);
  int target_lane_virtual_id_tmp{origin_lane_virtual_id_};
  
  MakesureLaneMergeDirection(origin_lane_virtual_id_);

  if (merge_lane_change_direction_ == LEFT_CHANGE && enable_l_) {
    if (request_type_ != LEFT_CHANGE && compute_lc_valid_info(LEFT_CHANGE)) {
      // 获取左车道线型
      iflyauto::LaneBoundaryType left_boundary_type =
          MakesureCurrentBoundaryType(LEFT_CHANGE, origin_lane_virtual_id_);
      target_lane_virtual_id_tmp = origin_lane_virtual_id_ - 1;
      GenerateRequest(LEFT_CHANGE);
      set_target_lane_virtual_id(target_lane_virtual_id_tmp);
      LOG_DEBUG(
          "[MergeRequest::update] Ask for merge "
          "changing lane to left "
          "\n");
      if (left_boundary_type ==
              iflyauto::LaneBoundaryType::LaneBoundaryType_MARKING_SOLID &&
          request_type_ != NO_CHANGE &&
          (lc_status == kLaneKeeping || lc_status == kLaneChangePropose ||
          (lc_status == kLaneChangeCancel &&
              (lane_change_lane_mgr_->has_origin_lane() &&
              lane_change_lane_mgr_->is_ego_on(olane))))) {
        Finish();
        set_target_lane_virtual_id(target_lane_virtual_id_tmp);
        LOG_DEBUG(
            "[MergeRequest::update] %s:%d finish request, dash not "
            "enough \n",
            __FUNCTION__, __LINE__);
      }
    }
  } else if (merge_lane_change_direction_ == RIGHT_CHANGE && enable_r_) {
    if (request_type_ != RIGHT_CHANGE && compute_lc_valid_info(RIGHT_CHANGE)) {
      // 获取右车道线型
      iflyauto::LaneBoundaryType right_boundary_type =
          MakesureCurrentBoundaryType(RIGHT_CHANGE, origin_lane_virtual_id_);
      target_lane_virtual_id_tmp = origin_lane_virtual_id_ + 1;
      GenerateRequest(RIGHT_CHANGE);
      set_target_lane_virtual_id(target_lane_virtual_id_tmp);
      LOG_DEBUG(
          "[MergeRequest::update] Ask for merge "
          "changing lane to right "
          "\n");
      if (right_boundary_type ==
              iflyauto::LaneBoundaryType::LaneBoundaryType_MARKING_SOLID &&
          request_type_ != NO_CHANGE &&
          (lc_status == kLaneKeeping || lc_status == kLaneChangePropose ||
          (lc_status == kLaneChangeCancel &&
              (lane_change_lane_mgr_->has_origin_lane() &&
              lane_change_lane_mgr_->is_ego_on(olane))))) {
        Finish();
        set_target_lane_virtual_id(target_lane_virtual_id_tmp);
        LOG_DEBUG(
            "[MergeRequest::update] %s:%d finish request, dash not "
            "enough \n",
            __FUNCTION__, __LINE__);
      }
    }
  } else if (merge_lane_change_direction_ == NO_CHANGE &&
             request_type_ != NO_CHANGE &&
             (lane_change_lane_mgr_->has_origin_lane() &&
              lane_change_lane_mgr_->is_ego_on(olane))) {
    Finish();
    set_target_lane_virtual_id(target_lane_virtual_id_tmp);
    LOG_DEBUG(
        "[MergeRequest::update] %s:%d finish request, "
        "merge_lane_change_direction == NO_CHANGE\n",
        __FUNCTION__, __LINE__);
  } else if (merge_lane_change_direction_ == NO_CHANGE) {
    // do nothing
    return;
  }
}

void MergeRequest::MakesureLaneMergeDirection(const int origin_lane_id) {
  const auto base_lane =
      virtual_lane_mgr_->get_lane_with_virtual_id(origin_lane_virtual_id_);
  const std::vector<std::shared_ptr<VirtualLane>>& relative_id_lanes =
      virtual_lane_mgr_->get_virtual_lanes();
  merge_lane_change_direction_ = NO_CHANGE;
  bool left_boundary_exist_virtual_type = false;
  bool right_boundary_exist_virtual_type = false;

  if (base_lane != nullptr) {
    // 判断左侧车道线类型
    auto left_lane_boundarys = base_lane->get_left_lane_boundary();
    for (int i = 0; i < left_lane_boundarys.type_segments_size; i++) {
      if (left_lane_boundarys.type_segments[i].type ==
              iflyauto::LaneBoundaryType_MARKING_VIRTUAL) {
        left_boundary_exist_virtual_type = true;
        break;
      } else {
        continue;
      }
    }
    // 判断右侧车道线类型
    auto right_lane_boundarys = base_lane->get_right_lane_boundary();
    for (int i = 0; i < right_lane_boundarys.type_segments_size; i++) {
      if (right_lane_boundarys.type_segments[i].type ==
              iflyauto::LaneBoundaryType_MARKING_VIRTUAL) {
        right_boundary_exist_virtual_type = true;
        break;
      } else {
        continue;
      }
    }
  } else {
    return;
  }

  if (left_boundary_exist_virtual_type) {
    merge_lane_change_direction_ = LEFT_CHANGE;
  } else if (right_boundary_exist_virtual_type &&
      !left_boundary_exist_virtual_type) {
    merge_lane_change_direction_ = RIGHT_CHANGE;
  } else {
    merge_lane_change_direction_ = NO_CHANGE;
  }

  JSON_DEBUG_VALUE("left_boundary_exist_virtual_type", left_boundary_exist_virtual_type);
  JSON_DEBUG_VALUE("right_boundary_exist_virtual_type", right_boundary_exist_virtual_type);
}

void MergeRequest::Reset() {
  is_merge_lane_change_situation_ = false;
  merge_alc_trigger_counter_ = 0;
  merge_lane_change_direction_ = NO_CHANGE;
}

}  // namespace planning