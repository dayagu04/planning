#include "lane_change_requests/interactive_lane_change_request.h"

#include <string>

#include "interactive_lane_change_request.h"

namespace planning {
// class: IntRequest
IntRequest::IntRequest(
    planning::framework::Session* session,
    std::shared_ptr<VirtualLaneManager> virtual_lane_mgr,
    std::shared_ptr<LaneChangeLaneManager> lane_change_lane_mgr)
    : LaneChangeRequest(session, virtual_lane_mgr, lane_change_lane_mgr) {
  auto config_builder = session_->mutable_environmental_model()->config_builder(
      planning::common::SceneType::HIGHWAY);
  auto int_request_config = config_builder->cast<ScenarioDisplayStateConfig>();
  enable_int_request_ = int_request_config.enable_int_request_function;
  count_threshold_ = int_request_config.int_rqt_cnt_threshold;
}

void IntRequest::Update(int lc_status) {
  // ego_blinker 0-lane follow, 1-left, 2-reght
  lane_change_cmd_ = session_->mutable_environmental_model()
                         ->get_ego_state_manager()
                         ->ego_blinker();
  // init lanes with id
  auto current_lane_virtual_id = virtual_lane_mgr_->current_lane_virtual_id();
  if (lane_change_lane_mgr_->has_origin_lane()) {
    auto origin_lane = lane_change_lane_mgr_->olane();
    origin_lane_virtual_id_ = origin_lane->get_virtual_id();
  } else {
    origin_lane_virtual_id_ = current_lane_virtual_id;
  }
  int target_lane_virtual_id_tmp{current_lane_virtual_id};

  LOG_DEBUG("[IntRequest::update] lane_change_cmd: %d\n", lane_change_cmd_);
  LOG_DEBUG(
      "[IntRequest::update] current_lane_virtual_id: %d, "
      "origin_lane_virtual_id_: %d, target_lane_virtual_id_: %d \n",
      current_lane_virtual_id, origin_lane_virtual_id_,
      target_lane_virtual_id_);

  if (lane_change_cmd_ == common::TurnSignalType::LEFT &&
      request_type_ != LEFT_CHANGE) {
    counter_right_ = 0;
    counter_left_++;
    // 获取左车道线型
    auto left_boundary_type = virtual_lane_mgr_->get_current_lane()
                                  ->get_left_lane_boundary()
                                  .segment(0)
                                  .type();
    // 实线禁止换道
    if (left_boundary_type == Common::LaneBoundaryType::MARKING_SOLID) {
      counter_left_ = -5;
    }
    if (counter_left_ > count_threshold_) {
      target_lane_virtual_id_tmp = origin_lane_virtual_id_ - 1;
      auto tlane = virtual_lane_mgr_->get_lane_with_virtual_id(
          target_lane_virtual_id_tmp);
      if (tlane != nullptr) {
        GenerateRequest(LEFT_CHANGE);
        set_target_lane_virtual_id(target_lane_virtual_id_tmp);
        LOG_DEBUG(
            "[IntRequest::update] Ask for interactive changing lane to left\n");
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
  } else if (lane_change_cmd_ == common::TurnSignalType::RIGHT &&
             request_type_ != RIGHT_CHANGE) {
    counter_left_ = 0;
    counter_right_ = counter_right_ + 1;
    // 获取右车道线型,实线禁止换道
    auto right_boundary_type = virtual_lane_mgr_->get_current_lane()
                                   ->get_right_lane_boundary()
                                   .segment(0)
                                   .type();
    if (right_boundary_type == Common::LaneBoundaryType::MARKING_SOLID) {
      counter_left_ = -5;
    }
    if (counter_right_ > count_threshold_) {
      target_lane_virtual_id_tmp = origin_lane_virtual_id_ + 1;
      auto tlane = virtual_lane_mgr_->get_lane_with_virtual_id(
          target_lane_virtual_id_tmp);
      if (tlane != nullptr) {
        GenerateRequest(RIGHT_CHANGE);
        set_target_lane_virtual_id(target_lane_virtual_id_tmp);
        LOG_DEBUG(
            "[IntRequest::update] Ask for interactive changing lane to right "
            "\n");
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
  } else if (lane_change_cmd_ == common::TurnSignalType::NONE &&
             request_type_ != NO_CHANGE) {
    // 3.换道过程中取消拨杆
    auto tlane = lane_change_lane_mgr_->tlane();
    if (lane_change_lane_mgr_->has_target_lane() &&
       (lane_change_lane_mgr_->is_ego_on(tlane) ||
        std::fabs(tlane->get_ego_lateral_offset()) < tlane->width() / 2 + 1.5)) {
      // 取消换道，但此时已经进入目标车道，则保持至换道完成
      LOG_DEBUG(
          "[IntRequest::update]: Cancel int lc blinker when ego car on target "
          "lane and continue lc state  \n");
    } else {
      Finish();
      set_target_lane_virtual_id(current_lane_virtual_id);
      counter_left_ = 0;
      counter_right_ = 0;
    }
  } else if (lane_change_cmd_ == common::TurnSignalType::NONE ||
             lane_change_cmd_ == common::TurnSignalType::EMERGENCY_FLASHER) {
    Finish();
    set_target_lane_virtual_id(current_lane_virtual_id);
    counter_left_ = 0;
    counter_right_ = 0;
  }
}

void IntRequest::finish_and_clear() {
  Finish();
  counter_left_ = 0;
  counter_right_ = 0;
  lane_change_cmd_ = planning::LeverStatus::LEVER_STATE_OFF;
  LOG_DEBUG(
      "[IntRequest::update] %s: clear int request and set lane_change_cmd_ off "
      "\n",
      __FUNCTION__);
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