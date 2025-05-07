#pragma once

#include "basic_types.pb.h"
namespace planning {

struct StopSpeedDecisonInfo {
 public:
  bool& mutable_is_valid() { return is_valid_; }
  const bool is_valid() const { return is_valid_; }
  double& mutable_s() { return s_; }
  const double s() const { return s_; }
  double& mutable_v() { return v_; }
  const double v() const { return v_; }
  double& mutable_a() { return a_; }
  const double a() const { return a_; }

 private:
  bool is_valid_ = false;
  double s_;
  double v_;
  double a_;
};

class StartStopDeciderOutPut {
 public:
  StartStopDeciderOutPut() = default;
  ~StartStopDeciderOutPut() = default;
  const common::StartStopInfo& ego_start_stop_info() const {
    return ego_start_stop_info_;
  }

  common::StartStopInfo& mutable_ego_start_stop_info() {
    return ego_start_stop_info_;
  }

  const StopSpeedDecisonInfo& stop_speed_decision_info() const {
    return stop_speed_decision_info_;
  }

  StopSpeedDecisonInfo& mutable_stop_speed_decision_info() {
    return stop_speed_decision_info_;
  }

  const bool& rads_scene_is_completed() const {
    return rads_scene_is_completed_;
  }

  bool& mutable_rads_scene_is_completed() {
    return rads_scene_is_completed_;
  }

 private:
  common::StartStopInfo ego_start_stop_info_;
  StopSpeedDecisonInfo stop_speed_decision_info_;
  bool rads_scene_is_completed_ = false;
};

}  // namespace planning