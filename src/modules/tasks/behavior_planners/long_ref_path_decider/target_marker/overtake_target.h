#pragma once

#include "ego_planning_config.h"
#include "lon_target_maker.pb.h"
#include "follow_target.h"
#include "target.h"

namespace planning {

class OvertakeTarget : public Target {
 public:
  OvertakeTarget(const SpeedPlannerConfig& config, framework::Session* session,
                 const FollowTarget& follow_target);
  ~OvertakeTarget() = default;

  struct OvertakeBound {
    double s = 0.0;
    double v = 0.0;
    double t = 0.0;
    TargetType type = TargetType::kNotSet;
    int32_t overtake_agent_id = -1;
  };

 private:
  void MakeOvertakeBoundsWithStCorridor();

  void MakeOvertakeTarget(const FollowTarget& follow_target);

  void AddOvertakeTargetDataToProto();

 private:
  std::vector<OvertakeBound> overtake_bounds_;
  planning::common::OvertakeTarget overtake_target_pb_;
};

}  // namespace planning
