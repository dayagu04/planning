#pragma once

#include "ego_planning_config.h"
#include "target.h"

namespace planning {

class CautionTarget : public Target {
  struct UpperBoundInfo {
    double s = 0.0;
    double t = 0.0;
    double v = 0.0;
    TargetType target_type = TargetType::kNotSet;
    int32_t agent_id = -1;
  };

 public:
  CautionTarget(const SpeedPlannerConfig& config, framework::Session* session);
  ~CautionTarget() = default;

 private:
  void GenerateUpperBoundInfo();

  void GenerateCautionTarget();

 private:
  std::vector<UpperBoundInfo> upper_bound_infos_;
};

}  // namespace planning
