#pragma once

#include "basic_types.pb.h"
namespace planning {

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

  const bool& rads_scene_is_completed() const {
    return rads_scene_is_completed_;
  }

  bool& mutable_rads_scene_is_completed() { return rads_scene_is_completed_; }

 private:
  common::StartStopInfo ego_start_stop_info_;
  bool rads_scene_is_completed_ = false;
};

}  // namespace planning