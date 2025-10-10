#pragma once

#include <cstdint>
#include "agent/agent.h"
namespace planning {

class MRCBrakeDeciderOutput {
 public:
  MRCBrakeDeciderOutput() = default;
  ~MRCBrakeDeciderOutput() = default;
  bool HasMRCVirtualObs() { return has_set_mrc_obs_; }
  void SetMRCVirtualObsFlag(bool has_mrc_obs) {
    has_set_mrc_obs_ = has_mrc_obs;
  }

 private:
  bool has_set_mrc_obs_ = false;
};

}  // namespace planning
