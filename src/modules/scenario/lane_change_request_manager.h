#pragma once

#include "framework/session.h"
#include "modules/common/config/basic_type.h"
#include "modules/context/ego_planning_config.h"

namespace planning {

class VirtualLane;
class VirtualLaneManager;
class LaneChangeLaneManager;

class LaneChangeRequest {
 public:
  LaneChangeRequest(
      planning::framework::Session* session,
      std::shared_ptr<VirtualLaneManager> virtual_lane_mgr,
      std::shared_ptr<LaneChangeLaneManager> lane_change_lane_mgr);
  virtual ~LaneChangeRequest() = default;

  void gen_request(RequestType direction);
  void finish();

  bool aggressive_change() const;

  RequestType request() const { return request_; }
  int target_lane_virtual_id() { return target_lane_virtual_id_; }
  void set_target_lane_virtual_id(int target_lane_virtual_id) {
    target_lane_virtual_id_ = target_lane_virtual_id;
  }
  RequestType turn_signal() const { return turn_signal_; }
  double tstart() const { return tstart_; }
  double tfinish() const { return tfinish_; }

  bool dashed_enough(RequestType direction, const double ego_vel,
                     std::shared_ptr<VirtualLaneManager> map_info_mgr);

 protected:
  RequestType request_ = NO_CHANGE;
  int target_lane_virtual_id_ = -1;
  int origin_lane_virtual_id_ = -1;
  int origin_lane_order_id_ = -1;
  RequestType turn_signal_ = NO_CHANGE;
  double tstart_ = 0.0;
  double tfinish_ = 0.0;
  planning::framework::Session* session_;
  std::shared_ptr<VirtualLaneManager> virtual_lane_mgr_;
  std::shared_ptr<LaneChangeLaneManager> lane_change_lane_mgr_;
};

class MapRequest : public LaneChangeRequest {
 public:
  MapRequest(planning::framework::Session* session,
             const EgoPlanningConfigBuilder* config_builder,
             std::shared_ptr<VirtualLaneManager> virtual_lane_mgr,
             std::shared_ptr<LaneChangeLaneManager> lane_change_lane_mgr);
  virtual ~MapRequest() = default;
  void update(int lc_status, int left_int_freeze_cnt, int right_int_freeze_cnt);

 private:
  void print_forbid_generating_reason(
      const std::vector<std::string> forbid_generating_reason);
  void check_lc_forbid_reason(
      std::vector<std::string>& forbid_generating_left_reason,
      std::vector<std::string>& forbid_generating_right_reason,
      int left_int_freeze_cnt, int right_int_freeze_cnt);
  bool must_change_before_curr_intersection();
};

class ActRequest : public LaneChangeRequest {
public:
  ActRequest(planning::framework::Session* session,
             std::shared_ptr<VirtualLaneManager> virtual_lane_mgr,
             std::shared_ptr<LaneChangeLaneManager> lane_change_lane_mgr);
  virtual ~ActRequest() = default;

  void reset(int direction = NO_CHANGE);

  void update(int lc_status);

  bool enforced_l() const { return enforced_l_; }
  bool enforced_r() const { return enforced_r_; }
  bool enable_l() const { return enable_l_; }

  void set_enforced_l(bool value) { enforced_l_ = value; }
  void set_enforced_r(bool value) { enforced_r_ = value; }

  std::string act_request_source() {return act_request_source_;}

private:
  int pos_cnt_l_ = 0;
  int pos_cnt_r_ = 0;
  int neg_cnt_l_ = 0;
  int neg_cnt_r_ = 0;
  bool left_faster_ = false;
  bool right_faster_ = false;
  bool enable_l_ = false;
  bool enable_r_ = false;
  bool enforced_l_ = false;
  bool enforced_r_ = false;

  // double vrel_f_ = DBL_MAX;
  // double vrel_l_ = DBL_MAX;
  // double vrel_r_ = DBL_MAX;
  // tmp ignore DBL_MAX
  double vrel_f_ = 1000.;
  double vrel_l_ = 1000.;
  double vrel_r_ = 1000.;
  std::string act_request_source_{"none"};
  void print_forbid_generating_reason(
      const std::vector<std::string> forbid_generating_reason);
  void check_lc_forbid_reason(
      std::vector<std::string>& forbid_generating_left_reason,
      std::vector<std::string>& forbid_generating_right_reason);
};


class IntRequest : public LaneChangeRequest {
 public:
  IntRequest(planning::framework::Session* session,
             std::shared_ptr<VirtualLaneManager> virtual_lane_mgr,
             std::shared_ptr<LaneChangeLaneManager> lane_change_lane_mgr);
  virtual ~IntRequest() = default;

  void update(int lc_status);
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
  void print_forbid_generating_reason(
      const std::vector<std::string> forbid_generating_reason);
  void check_lc_forbid_reason(
      std::vector<std::string>& forbid_generating_left_reason,
      std::vector<std::string>& forbid_generating_right_reason);
  int counter_left_ = 0;
  int counter_right_ = 0;
  int left_cancel_freeze_cnt_ = 0;
  int right_cancel_freeze_cnt_ = 0;
  bool enable_int_request_ = false;
  int count_trsh_ = 2;
  std::uint8_t lane_change_cmd_ = LeverStatus::LEVER_STATE_OFF;
};

class LaneChangeRequestManager {
 public:
  LaneChangeRequestManager(
      planning::framework::Session* session,
      const EgoPlanningConfigBuilder* config_builder,
      std::shared_ptr<VirtualLaneManager> virtual_lane_mgr,
      std::shared_ptr<LaneChangeLaneManager> lane_change_lane_mgr);
  virtual ~LaneChangeRequestManager() = default;

  void finish_request();

  void update(int lc_status, const bool hd_map_valid);

  RequestType request() const { return request_; }
  RequestSource request_source() const { return request_source_; }
  int target_lane_virtual_id() { return target_lane_virtual_id_; }
  void set_target_lane_virtual_id(int target_lane_virtual_id) {
    target_lane_virtual_id_ = target_lane_virtual_id;
    map_request_.set_target_lane_virtual_id(target_lane_virtual_id);
    int_request_.set_target_lane_virtual_id(target_lane_virtual_id);
    act_request_.set_target_lane_virtual_id(target_lane_virtual_id);
  }
  RequestType turn_signal() const { return turn_signal_; }
  bool aggressive_change() const {
    if (request_source_ == NO_REQUEST) {
      return false;
    } else if (request_source_ == INT_REQUEST) {
      return true;
    } else if (request_source_ == MAP_REQUEST) {
      return map_request_.aggressive_change();
    } else if (request_source_ == ACT_REQUEST) {
      return act_request_.aggressive_change();
    }
    return false;
  }

  double get_req_tstart(int source) const;
  double get_req_tfinish(int source) const;

 private:
  RequestType request_ = NO_CHANGE;
  RequestSource request_source_ = NO_REQUEST;
  RequestType turn_signal_ = NO_CHANGE;
  int target_lane_virtual_id_ = -1;
  IntRequest int_request_;
  MapRequest map_request_;
  ActRequest act_request_;

  std::shared_ptr<VirtualLaneManager> virtual_lane_mgr_;
  planning::framework::Session* session_;
};

}  // namespace planning
