#include "merge_lane_change_request.h"

#include <glog/logging.h>
#include <math.h>

#include <cassert>
#include <cmath>
#include <complex>
#include <limits>

#include "common.pb.h"
#include "common_c.h"
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
  base_frenet_coord_ = std::make_shared<planning_math::KDPath>();
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
  // is_merge_lane_change_situation_ = false;
  both_lane_line_exist_virtual_or_not_ = false;
  // is_exist_left_merge_direction_ = false;
  // is_exist_right_merge_direction_ = false;

  MakesureLaneMergeDirection(origin_lane_virtual_id_);
  LOG_DEBUG("MergeRequest::Update: both_lane_line_exist_virtual_or_not_ %d",
            both_lane_line_exist_virtual_or_not_);
  JSON_DEBUG_VALUE("both_lane_line_exist_virtual_or_not_",
                   both_lane_line_exist_virtual_or_not_);

  UpdateLaneMergeSituation(lc_status);
  LOG_DEBUG("MergeRequest::Update: is_merge_lane_change_situation_ %d",
            is_merge_lane_change_situation_);
  JSON_DEBUG_VALUE("is_merge_lane_change_situation_",
                   is_merge_lane_change_situation_);
  JSON_DEBUG_VALUE("merge_alc_trigger_counter_", merge_alc_trigger_counter_);

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
  const auto& ego_lane_road_right_decider_output =
      session_->planning_context().ego_lane_road_right_decider_output();
  const bool is_merge_region =
      ego_lane_road_right_decider_output.is_merge_region;
  const int lane_nums = virtual_lane_mgr_->get_lane_num();
  const auto& current_lane = virtual_lane_mgr_->get_current_lane();
  const int current_lane_order_id = current_lane->get_order_id();

  if (merge_lane_change_direction_ != NO_CHANGE) {
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

  if (merge_lane_change_direction_ == LEFT_CHANGE && enable_l_) {
    if (request_type_ != LEFT_CHANGE) {
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
    if (request_type_ != RIGHT_CHANGE) {
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
  const auto& ego_state =
      session_->environmental_model().get_ego_state_manager();
  const double default_consider_lane_marks_length = 80.0;
  const auto& plannig_init_point = ego_state->planning_init_point();
  double ego_x = plannig_init_point.lat_init_state.x();
  double ego_y = plannig_init_point.lat_init_state.y();
  Point2D ego_cart_point{plannig_init_point.lat_init_state.x(),
                         plannig_init_point.lat_init_state.y()};
  const auto& ego_lane_road_right_decider_output =
      session_->planning_context().ego_lane_road_right_decider_output();
  const bool is_merge_region =
      ego_lane_road_right_decider_output.is_merge_region;
  const auto& function_info = session_->environmental_model().function_info();
  const bool is_split_region =
      ego_lane_road_right_decider_output.is_split_region;
  const bool cur_lane_is_continue =
      ego_lane_road_right_decider_output.cur_lane_is_continue;

  merge_lane_change_direction_ = NO_CHANGE;
  bool left_boundary_exist_virtual_type = false;
  bool right_boundary_exist_virtual_type = false;
  bool target_left_boundary_exist_virtual_type = false;
  bool target_right_boundary_exist_virtual_type = false;

  bool exist_left_direction_merge = false;
  bool exist_right_direction_merge = false;
  const auto& llane = virtual_lane_mgr_->get_left_lane();
  const auto& rlane = virtual_lane_mgr_->get_right_lane();
  const int lane_nums = virtual_lane_mgr_->get_lane_num();
  const auto& current_lane = virtual_lane_mgr_->get_current_lane();
  const int current_lane_order_id = current_lane->get_order_id();
  bool is_left_edge_side_lane = current_lane_order_id == 0;
  bool is_right_edge_side_lane = current_lane_order_id == lane_nums - 1;

  std::shared_ptr<planning_math::KDPath> left_base_boundary_path;
  std::shared_ptr<planning_math::KDPath> right_base_boundary_path;

  if (base_lane != nullptr) {
    double left_lane_line_length = 0.0;
    int left_current_segment_count = 0;
    double left_ego_s = 0.0, left_ego_l = 0.0;
    // 判断左侧车道线类型
    auto left_lane_boundarys = base_lane->get_left_lane_boundary();
    left_base_boundary_path =
        virtual_lane_mgr_->MakeBoundaryPath(left_lane_boundarys);
    if (left_base_boundary_path != nullptr) {
      if (!left_base_boundary_path->XYToSL(ego_x, ego_y, &left_ego_s,
                                           &left_ego_l)) {
        return;
      }
    } else {
      return;
    }
    for (int i = 0; i < left_lane_boundarys.type_segments_size; i++) {
      left_lane_line_length += left_lane_boundarys.type_segments[i].length;
      if (left_lane_line_length > left_ego_s) {
        left_current_segment_count = i;
        break;
      }
    }
    for (int i = left_current_segment_count;
         i < left_lane_boundarys.type_segments_size; i++) {
      if (left_lane_boundarys.type_segments[i].type ==
          iflyauto::LaneBoundaryType_MARKING_VIRTUAL) {
        left_boundary_exist_virtual_type = true;
        break;
      } else {
        continue;
      }
    }

    std::shared_ptr<planning_math::KDPath> base_lane_frenet_crd =
        base_lane->get_lane_frenet_coord();
    int segment = -1;
    double ego_s = 0.0;
    if (base_lane_frenet_crd != nullptr) {
      Point2D ego_cart_frenet_point;
      if (!base_lane_frenet_crd->XYToSL(ego_cart_point,
                                        ego_cart_frenet_point)) {
        ego_s = 0.0;
      } else {
        ego_s = ego_cart_frenet_point.x;
      }
      std::vector<iflyauto::LaneMarkMsg> lane_marks = base_lane->lane_marks();
      double lane_line_length = 0.0;
      for (int i = 0; i < lane_marks.size(); i++) {
        lane_line_length = lane_marks[i].end;
        if (lane_line_length > ego_s && lane_marks[i].begin <= ego_s) {
          segment = i;
          break;
        } else {
          continue;
        }
      }
    }

    if (llane != nullptr) {
      double target_right_lane_line_length = 0.0;
      int target_right_current_segment_count = 0;
      double right_ego_s = 0.0, right_ego_l = 0.0;
      auto right_lane_boundarys = llane->get_right_lane_boundary();
      right_base_boundary_path =
          virtual_lane_mgr_->MakeBoundaryPath(right_lane_boundarys);
      if (right_base_boundary_path != nullptr) {
        if (!right_base_boundary_path->XYToSL(ego_x, ego_y, &right_ego_s,
                                              &right_ego_l)) {
          right_ego_s = 50.0;
        }

        for (int i = 0; i < right_lane_boundarys.type_segments_size; i++) {
          target_right_lane_line_length +=
              right_lane_boundarys.type_segments[i].length;
          if (target_right_lane_line_length > right_ego_s) {
            target_right_current_segment_count = i;
            break;
          }
        }
        for (int i = target_right_current_segment_count;
             i < right_lane_boundarys.type_segments_size; i++) {
          if (right_lane_boundarys.type_segments[i].type ==
              iflyauto::LaneBoundaryType_MARKING_VIRTUAL) {
            target_right_boundary_exist_virtual_type = true;
            break;
          } else {
            continue;
          }
        }
      }
    }

    // 根据地面标识判断是否向左汇流
    if (segment >= 0 && ego_s != 0.0) {
      std::vector<iflyauto::LaneMarkMsg> lane_marks = base_lane->lane_marks();

      for (int i = segment; i < lane_marks.size(); i++) {
        if (lane_marks[i].begin > ego_s + default_consider_lane_marks_length) {
          break;
        }
        if (lane_marks[i].lane_mark ==
            iflyauto::LaneDrivableDirection_DIRECTION_LEFT_MERGE) {
          exist_left_direction_merge = true;
          break;
        }
      }
    }

    double right_lane_line_length = 0.0;
    int right_current_segment_count = 0;
    double right_ego_s = 0.0, right_ego_l = 0.0;
    auto right_lane_boundarys = base_lane->get_right_lane_boundary();
    right_base_boundary_path =
        virtual_lane_mgr_->MakeBoundaryPath(right_lane_boundarys);
    if (right_base_boundary_path != nullptr) {
      if (!right_base_boundary_path->XYToSL(ego_x, ego_y, &right_ego_s,
                                            &right_ego_l)) {
        return;
      }
    } else {
      return;
    }
    for (int i = 0; i < right_lane_boundarys.type_segments_size; i++) {
      right_lane_line_length += right_lane_boundarys.type_segments[i].length;
      if (right_lane_line_length > right_ego_s) {
        right_current_segment_count = i;
        break;
      }
    }
    for (int i = right_current_segment_count;
         i < right_lane_boundarys.type_segments_size; i++) {
      if (right_lane_boundarys.type_segments[i].type ==
          iflyauto::LaneBoundaryType_MARKING_VIRTUAL) {
        right_boundary_exist_virtual_type = true;
        break;
      } else {
        continue;
      }
    }

    if (rlane != nullptr) {
      double target_left_lane_line_length = 0.0;
      int target_left_current_segment_count = 0;
      double left_ego_s = 0.0, left_ego_l = 0.0;
      // 判断左侧车道线类型
      auto left_lane_boundarys = rlane->get_left_lane_boundary();
      left_base_boundary_path =
          virtual_lane_mgr_->MakeBoundaryPath(left_lane_boundarys);
      if (left_base_boundary_path != nullptr) {
        if (!left_base_boundary_path->XYToSL(ego_x, ego_y, &left_ego_s,
                                             &left_ego_l)) {
          left_ego_s = 50.0;
        }

        for (int i = 0; i < left_lane_boundarys.type_segments_size; i++) {
          target_left_lane_line_length +=
              left_lane_boundarys.type_segments[i].length;
          if (target_left_lane_line_length > left_ego_s) {
            target_left_current_segment_count = i;
            break;
          }
        }
        for (int i = target_left_current_segment_count;
             i < left_lane_boundarys.type_segments_size; i++) {
          if (left_lane_boundarys.type_segments[i].type ==
              iflyauto::LaneBoundaryType_MARKING_VIRTUAL) {
            target_left_boundary_exist_virtual_type = true;
            break;
          } else {
            continue;
          }
        }
      }
    }

    //根据地面标识判断是否向右汇流
    if (segment >= 0 && ego_s != 0.0) {
      std::vector<iflyauto::LaneMarkMsg> lane_marks = base_lane->lane_marks();

      for (int i = segment; i < lane_marks.size(); i++) {
        if (lane_marks[i].begin > ego_s + default_consider_lane_marks_length) {
          break;
        }
        if (lane_marks[i].lane_mark ==
            iflyauto::LaneDrivableDirection_DIRECTION_RIGHT_MERGE) {
          exist_right_direction_merge = true;
          break;
        }
      }
    }

    if (is_right_edge_side_lane && !is_split_region &&
        function_info.function_mode() == common::DrivingFunctionInfo::NOA) {
      if (exist_left_direction_merge) {
        is_exist_left_merge_direction_ = true;
        merge_lane_change_direction_ = LEFT_CHANGE;
        return;
      }
    }
    if (!cur_lane_is_continue && is_left_edge_side_lane && !is_split_region &&
        function_info.function_mode() == common::DrivingFunctionInfo::NOA) {
      if (exist_right_direction_merge) {
        is_exist_right_merge_direction_ = true;
        merge_lane_change_direction_ = RIGHT_CHANGE;
        return;
      }
    }
  } else {
    return;
  }

  if (target_right_boundary_exist_virtual_type &&
      target_left_boundary_exist_virtual_type &&
      left_boundary_exist_virtual_type && right_boundary_exist_virtual_type) {
    merge_lane_change_direction_ = NO_CHANGE;
    both_lane_line_exist_virtual_or_not_ = true;
  } else if (left_boundary_exist_virtual_type &&
             !target_right_boundary_exist_virtual_type &&
             is_right_edge_side_lane && is_merge_region) {
    merge_lane_change_direction_ = LEFT_CHANGE;
  } else if (right_boundary_exist_virtual_type &&
             !target_left_boundary_exist_virtual_type &&
             is_left_edge_side_lane && is_merge_region) {
    merge_lane_change_direction_ = RIGHT_CHANGE;
  } else if (!right_boundary_exist_virtual_type &&
             !left_boundary_exist_virtual_type) {
    merge_lane_change_direction_ = NO_CHANGE;
    both_lane_line_exist_virtual_or_not_ = true;
  } else {
    merge_lane_change_direction_ = NO_CHANGE;
  }

  JSON_DEBUG_VALUE("left_boundary_exist_virtual_type",
                   left_boundary_exist_virtual_type);
  JSON_DEBUG_VALUE("right_boundary_exist_virtual_type",
                   right_boundary_exist_virtual_type);
}

void MergeRequest::Reset() {
  is_merge_lane_change_situation_ = false;
  merge_alc_trigger_counter_ = 0;
  merge_lane_change_direction_ = NO_CHANGE;
  is_exist_left_merge_direction_ = false;
  is_exist_right_merge_direction_ = false;
}

}  // namespace planning