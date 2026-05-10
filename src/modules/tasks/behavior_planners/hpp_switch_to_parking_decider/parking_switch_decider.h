#pragma once

#include "hpp_switch_info.h"
#include "tasks/task.h"

namespace planning {

// stage1: If ego is stopping, try to switch parking.
// stage2: If ego is near memory slot and ego is moving, try to switch parking.
// stage3: Searching any slot in memory route.
class ParkingSwitchDecider : public Task {
 public:
  ParkingSwitchDecider(const EgoPlanningConfigBuilder *config_builder,
                       framework::Session *session);
  virtual ~ParkingSwitchDecider() = default;

  bool Execute() override;
 private:
  bool IsTargetSlotAllowedToPark();
  void UpdateHppObsBlockedTimeout();

 private:
  HppParkingSwitchConfig config_;
  HppParkingSwitchInfo parking_switch_info_;

  /****************** internal use ***************/
  double timestamp_at_standstill_near_dest_ = 0.0;
  bool last_is_standstill_near_target_slot_ = false;

  // HPP 巡航前方非虚拟障碍物刹停超时计时器
  int  hpp_obs_blocked_frame_count_ = 0;
  int  hpp_non_stop_traj_count_ = 0;
  bool is_hpp_obs_blocked_timeout_ = false;
};

}  // namespace planning
