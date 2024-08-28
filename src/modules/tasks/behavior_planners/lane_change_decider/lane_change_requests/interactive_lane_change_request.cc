#include "interactive_lane_change_request.h"

#include <string>
#include <vector>

#include "common.pb.h"
#include "config/basic_type.h"
#include "debug_info_log.h"
#include "interactive_lane_change_request.h"
#include "planning_context.h"

namespace planning {
// class: IntRequest
namespace {
constexpr double kLCCoolingTime = 3.0;
}

IntRequest::IntRequest(
    planning::framework::Session* session,
    std::shared_ptr<VirtualLaneManager> virtual_lane_mgr,
    std::shared_ptr<LaneChangeLaneManager> lane_change_lane_mgr)
    : LaneChangeRequest(session, virtual_lane_mgr, lane_change_lane_mgr) {
  auto config_builder = session_->mutable_environmental_model()->config_builder(
      planning::common::SceneType::HIGHWAY);
  int_request_config_ = config_builder->cast<ScenarioDisplayStateConfig>();
  enable_int_request_ = int_request_config_.enable_int_request_function;
  count_threshold_ = int_request_config_.int_rqt_cnt_threshold;
}

void IntRequest::Update(int lc_status) {
  const auto& lc_req_source = session_->planning_context()
                                  .lane_change_decider_output()
                                  .lc_request_source;
  if (lc_status != kLaneKeeping && lc_status != kLaneChangePropose &&
      lc_req_source != INT_REQUEST) {
    LOG_DEBUG("IntRequest::Update: ego not in lane keeping!");
    return;
  }
  // ego_blinker 0-lane follow, 1-left, 2-right
  request_cancel_reason_ = NO_CANCEL;
  ilc_virtual_req_ = NO_CHANGE;
  lane_change_cmd_ = session_->mutable_environmental_model()
                         ->get_ego_state_manager()
                         ->ego_blinker();
  JSON_DEBUG_VALUE("lane_change_cmd_", lane_change_cmd_);
  if (lane_change_cmd_ == 0 || is_lever_status_valid_last_frame_) {
    is_lever_status_valid_ = true;
  }
  is_lever_status_valid_last_frame_ =
      lane_change_cmd_;  //反方向拨杆时，能触发反方向的变道请求
  // init lanes with id
  auto current_lane_virtual_id = virtual_lane_mgr_->current_lane_virtual_id();
  auto tlane = lane_change_lane_mgr_->tlane();
  std::shared_ptr<ReferencePathManager> reference_path_mgr =
      session_->mutable_environmental_model()->get_reference_path_manager();
  auto target_reference_path = reference_path_mgr->get_reference_path_by_lane(
      lane_change_lane_mgr_->target_lane_virtual_id(), false);
  double frenet_ego_state_l =
      target_reference_path != nullptr
          ? target_reference_path->get_frenet_ego_state().l()
          : 0.;

  if (lane_change_lane_mgr_->has_origin_lane()) {
    auto origin_lane = lane_change_lane_mgr_->olane();
    origin_lane_virtual_id_ = origin_lane->get_virtual_id();
  } else {
    origin_lane_virtual_id_ = current_lane_virtual_id;
  }
  int target_lane_virtual_id_tmp{current_lane_virtual_id};

  const int origin_relative_id_zero_nums =
      virtual_lane_mgr_->origin_relative_id_zero_nums();
  std::vector<int> zero_relative_id_order_ids =
      virtual_lane_mgr_->GetZeroRelativeIdOrderIds();

  // 获取左车道线型
  iflyauto::LaneBoundaryType left_boundary_type =
      MakesureCurrentBoundaryType(LEFT_CHANGE, origin_lane_virtual_id_);

  // 获取右车道线型,实线禁止换道
  iflyauto::LaneBoundaryType right_boundary_type =
      MakesureCurrentBoundaryType(RIGHT_CHANGE, origin_lane_virtual_id_);

  JSON_DEBUG_VALUE("left_boundary_type", (int)left_boundary_type);
  JSON_DEBUG_VALUE("right_boundary_type", (int)right_boundary_type);
  LOG_DEBUG("[IntRequest::update] lane_change_cmd: %d\n", lane_change_cmd_);
  LOG_DEBUG(
      "[IntRequest::update] current_lane_virtual_id: %d, "
      "origin_lane_virtual_id_: %d, target_lane_virtual_id_: %d \n",
      current_lane_virtual_id, origin_lane_virtual_id_,
      target_lane_virtual_id_);
  count_threshold_ = -1;
  if (lane_change_cmd_ == iflyauto::TURN_SIGNAL_TYPE_LEFT &&
      request_type_ != LEFT_CHANGE && is_lever_status_valid_) {
    counter_right_ = 0;
    counter_left_++;
    // 实线禁止换道
    if (left_boundary_type ==
        iflyauto::LaneBoundaryType::LaneBoundaryType_MARKING_SOLID) {
      request_cancel_reason_ = SOLID_LC;
      ilc_virtual_req_ = LEFT_CHANGE;
      counter_left_ = -5;
    }

    target_lane_virtual_id_tmp = origin_lane_virtual_id_ - 1;
    if (!zero_relative_id_order_ids.empty() &&
        origin_relative_id_zero_nums > 1) {
      std::shared_ptr<VirtualLane> origin_lane =
          virtual_lane_mgr_->get_lane_with_virtual_id(origin_lane_virtual_id_);
      int origin_lane_order_id = origin_lane->get_order_id();
      std::shared_ptr<VirtualLane> tem_target_lane =
          virtual_lane_mgr_->get_lane_with_virtual_id(
              target_lane_virtual_id_tmp);
      if (tem_target_lane != nullptr) {
        int temp_target_lane_order_id = tem_target_lane->get_order_id();
        auto origin_id_iter =
            std::find(zero_relative_id_order_ids.begin(),
                      zero_relative_id_order_ids.end(), origin_lane_order_id);
        auto target_id_iter = std::find(zero_relative_id_order_ids.begin(),
                                        zero_relative_id_order_ids.end(),
                                        temp_target_lane_order_id);
        if (origin_id_iter != zero_relative_id_order_ids.end() &&
            target_id_iter != zero_relative_id_order_ids.end()) {
          is_in_diverted_lane_change_ = true;
        }
      } else {
        is_in_diverted_lane_change_ = false;
      }
    }
    if (counter_left_ > count_threshold_ || is_in_diverted_lane_change_) {
      auto tlane = virtual_lane_mgr_->get_lane_with_virtual_id(
          target_lane_virtual_id_tmp);
      if (tlane != nullptr) {
        GenerateRequest(LEFT_CHANGE);
        set_target_lane_virtual_id(target_lane_virtual_id_tmp);
        LOG_DEBUG(
            "[IntRequest::update] Ask for interactive changing lane to left\n");
        if ((lc_status == kLaneKeeping || lc_status == kLaneChangePropose) &&
            IsRoadBorderSurpressLaneChange(LEFT_CHANGE, origin_lane_virtual_id_,
                                           target_lane_virtual_id_)) {
          LOG_DEBUG(
              "[IntRequest::update] Road border surpress lane change to "
              "left\n");
          Finish();
          set_target_lane_virtual_id(current_lane_virtual_id);
        }
      } else {
        LOG_WARNING(
            "[IntRequest::update] Ask for interactive changing lane to left "
            "but left lane is null \n");
      }
    } else {
      LOG_DEBUG(
          "[IntRequest::update] waiting counter for interactive changing lane "
          "to left \n");
    }
  } else if (lane_change_cmd_ == iflyauto::TURN_SIGNAL_TYPE_RIGHT &&
             request_type_ != RIGHT_CHANGE && is_lever_status_valid_) {
    counter_left_ = 0;
    counter_right_ = counter_right_ + 1;
    if (right_boundary_type ==
        iflyauto::LaneBoundaryType::LaneBoundaryType_MARKING_SOLID) {
      request_cancel_reason_ = SOLID_LC;
      ilc_virtual_req_ = RIGHT_CHANGE;
      counter_right_ = -5;
    }

    target_lane_virtual_id_tmp = origin_lane_virtual_id_ + 1;
    if (!zero_relative_id_order_ids.empty() &&
        origin_relative_id_zero_nums > 1) {
      std::shared_ptr<VirtualLane> origin_lane =
          virtual_lane_mgr_->get_lane_with_virtual_id(origin_lane_virtual_id_);
      int origin_lane_order_id = origin_lane->get_order_id();
      std::shared_ptr<VirtualLane> tem_target_lane =
          virtual_lane_mgr_->get_lane_with_virtual_id(
              target_lane_virtual_id_tmp);
      if (tem_target_lane != nullptr) {
        int temp_target_lane_order_id = tem_target_lane->get_order_id();
        auto origin_id_iter =
            std::find(zero_relative_id_order_ids.begin(),
                      zero_relative_id_order_ids.end(), origin_lane_order_id);
        auto target_id_iter = std::find(zero_relative_id_order_ids.begin(),
                                        zero_relative_id_order_ids.end(),
                                        temp_target_lane_order_id);
        if (origin_id_iter != zero_relative_id_order_ids.end() &&
            target_id_iter != zero_relative_id_order_ids.end()) {
          is_in_diverted_lane_change_ = true;
        }
      } else {
        is_in_diverted_lane_change_ = false;
      }
    }
    if (counter_right_ > count_threshold_ || is_in_diverted_lane_change_) {
      auto tlane = virtual_lane_mgr_->get_lane_with_virtual_id(
          target_lane_virtual_id_tmp);
      if (tlane != nullptr) {
        GenerateRequest(RIGHT_CHANGE);
        set_target_lane_virtual_id(target_lane_virtual_id_tmp);
        LOG_DEBUG(
            "[IntRequest::update] Ask for interactive changing lane to right "
            "\n");
        if ((lc_status == kLaneKeeping || lc_status == kLaneChangePropose) &&
            IsRoadBorderSurpressLaneChange(RIGHT_CHANGE,
                                           origin_lane_virtual_id_,
                                           target_lane_virtual_id_)) {
          LOG_DEBUG(
              "[IntRequest::update] Road border surpress lane change to "
              "right\n");
          Finish();
          set_target_lane_virtual_id(current_lane_virtual_id);
        }
      } else {
        LOG_WARNING(
            "[IntRequest::update] Ask for interactive changing lane to right "
            "but right lane is null \n");
      }
    } else {
      LOG_DEBUG(
          "[IntRequest::update] waiting counter for interactive changing lane "
          "to right \n");
    }
  } else if (lane_change_cmd_ == iflyauto::TURN_SIGNAL_TYPE_NONE &&
             request_type_ != NO_CHANGE) {
    // 3.换道过程中取消拨杆
    if (lane_change_lane_mgr_->has_target_lane() &&
        (std::fabs(frenet_ego_state_l) <
         tlane->width() / 2 +
             int_request_config_.disallow_cancel_int_lc_lateral_thr)) {
      // 取消换道，但此时已经进入目标车道，则保持至换道完成
      LOG_DEBUG(
          "[IntRequest::update]: Cancel int lc blinker when ego car on target "
          "lane and continue lc state  \n");
    } else {
      request_cancel_reason_ = MANUAL_CANCEL;
      Finish();
      set_target_lane_virtual_id(current_lane_virtual_id);
      counter_left_ = 0;
      counter_right_ = 0;
    }
  } else if (lane_change_cmd_ == iflyauto::TURN_SIGNAL_TYPE_NONE ||
             lane_change_cmd_ == iflyauto::TURN_SIGNAL_TYPE_EMERGENCY_FLASH) {
    Finish();
    set_target_lane_virtual_id(current_lane_virtual_id);
    counter_left_ = 0;
    counter_right_ = 0;
    is_in_diverted_lane_change_ = false;
  } else if (lane_change_lane_mgr_->has_target_lane() &&
             (std::fabs(frenet_ego_state_l) >=
              tlane->width() / 2 +
                  int_request_config_.disallow_cancel_int_lc_lateral_thr) &&
             !is_in_diverted_lane_change_) {
    if ((request_type_ == LEFT_CHANGE &&
         left_boundary_type ==
             iflyauto::LaneBoundaryType::LaneBoundaryType_MARKING_SOLID) ||
        (request_type_ == RIGHT_CHANGE &&
         right_boundary_type ==
             iflyauto::LaneBoundaryType::LaneBoundaryType_MARKING_SOLID)) {
      request_cancel_reason_ = SOLID_LC;
      Finish();
      set_target_lane_virtual_id(current_lane_virtual_id);
      counter_left_ = 0;
      counter_right_ = 0;
    }
  }
}

void IntRequest::finish_and_clear() {
  Finish();
  counter_left_ = 0;
  counter_right_ = 0;
  lane_change_cmd_ = iflyauto::TURN_SIGNAL_TYPE_NONE;
  LOG_DEBUG(
      "[IntRequest::update] %s: clear int request and set lane_change_cmd_ off "
      "\n",
      __FUNCTION__);
  is_lever_status_valid_ = false;
  is_in_diverted_lane_change_ = false;
}

void IntRequest::PrintForbidGeneratingReason(
    const std::vector<std::string> forbid_generating_reason) {
  for (const auto& reason : forbid_generating_reason) {
    std::cout << "[IntRequest], Disable Reason: " << reason.c_str()
              << std::endl;
  }
}

// void IntRequest::check_lc_forbid_reason(
//     std::vector<std::string>& forbid_generating_left_reason,
//     std::vector<std::string>& forbid_generating_right_reason) {
//   forbid_generating_left_reason.clear();
//   forbid_generating_right_reason.clear();
//   // lc forbid reason, top down by priority
//   // 需要可视化接口
//   // 1) remind diver close to ramp, should not change to opposite lane unless
//   // persists
//   // 2) solid line or low speed
//   bool enable_display = false;
//   if (enable_display &&
//       request_cancel_reason_ == IntCancelReasonType::NO_CANCEL) {
//     // request_cancel_reason_ =
//     //     scenario_state_display->display_cancel_intreq_reason(
//     //         origin_lane_order_id_, origin_lane_virtual_id_);
//     if (request_cancel_reason_ == IntCancelReasonType::UNSUITABLE_VEL) {
//       if (request_type_ == RIGHT_CHANGE)
//         forbid_generating_right_reason.push_back("low_speed");
//       if (request_type_ == LEFT_CHANGE)
//         forbid_generating_left_reason.push_back("low_speed");
//     } else if (request_cancel_reason_ == IntCancelReasonType::SOLID_LC) {
//       if (request_type_ == RIGHT_CHANGE)
//         forbid_generating_right_reason.push_back("solid_line");
//       if (request_type_ == LEFT_CHANGE)
//         forbid_generating_left_reason.push_back("solid_line");
//     }
//   }

//   // 3) lane change logit too low
//   auto origin_lane = lane_change_lane_mgr_->olane();
//   if (request_type_ != NO_CHANGE &&
//       request_cancel_reason_ == IntCancelReasonType::NO_CANCEL) {
//     std::shared_ptr<VirtualLane> target_lane = nullptr;
//     if (request_type_ == LEFT_CHANGE) {
//       target_lane = virtual_lane_mgr_->get_left_lane();
//     } else if (request_type_ == RIGHT_CHANGE) {
//       target_lane = virtual_lane_mgr_->get_right_lane();
//     }

//     if (target_lane == nullptr) {
//       LOG_DEBUG("int request invalid, target lane not exist \n");
//     }
//   }
// }

}  // namespace planning