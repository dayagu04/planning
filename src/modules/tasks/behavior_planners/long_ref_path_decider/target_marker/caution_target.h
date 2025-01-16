#pragma once

#include "ego_planning_config.h"
#include "lon_target_maker.pb.h"
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

  void AddCautionTargetDataToProto();

 private:
  std::vector<UpperBoundInfo> upper_bound_infos_;
  planning::common::CautionTarget caution_target_pb_;

};

}  // namespace planning
