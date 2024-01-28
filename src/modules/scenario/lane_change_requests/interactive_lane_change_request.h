#pragma once

#include "lane_change_requests/lane_change_request.h"

namespace planning {

/// @brief 交互式(Interactive)换道请求
class IntRequest : public LaneChangeRequest {
 public:
  IntRequest(planning::framework::Session* session,
             std::shared_ptr<VirtualLaneManager> virtual_lane_mgr,
             std::shared_ptr<LaneChangeLaneManager> lane_change_lane_mgr);
  virtual ~IntRequest() = default;

  void Update(int lc_status);

  IntCancelReasonType request_cancel_reason() {
    return request_cancel_reason_;
  };

  void reset_freeze_cnt() {
    left_cancel_freeze_cnt_ = 0;
    right_cancel_freeze_cnt_ = 0;
  }

  void reset_int_cnt() { reset_freeze_cnt(); }

  bool enable_int_request() { return enable_int_request_; }

  void finish_and_clear();

  const int get_left_cancel_freeze_cnt() const {
    return left_cancel_freeze_cnt_;
  }
  const int get_right_cancel_freeze_cnt() const {
    return right_cancel_freeze_cnt_;
  }
  void set_left_cancel_freeze_cnt(int freeze_cnt) {
    left_cancel_freeze_cnt_ = freeze_cnt;
  }
  void set_right_cancel_freeze_cnt(int freeze_cnt) {
    right_cancel_freeze_cnt_ = freeze_cnt;
  }

 private:
  void PrintForbidGeneratingReason(
      const std::vector<std::string> forbid_generating_reason);

  // void check_lc_forbid_reason(
  //     std::vector<std::string>& forbid_generating_left_reason,
  //     std::vector<std::string>& forbid_generating_right_reason);

 private:
  int counter_left_ = 0;
  int counter_right_ = 0;
  int left_cancel_freeze_cnt_ = 0;
  int right_cancel_freeze_cnt_ = 0;
  bool enable_int_request_ = false;
  int count_threshold_ = 3;
  std::uint8_t lane_change_cmd_ = LeverStatus::LEVER_STATE_OFF;
  IntCancelReasonType request_cancel_reason_ = NO_CANCEL;
  ScenarioDisplayStateConfig int_request_config_;
};

}  // namespace planning