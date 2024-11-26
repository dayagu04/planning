#pragma once
#include "ego_planning_config.h"
#include "lon_target_maker.pb.h"
#include "session.h"
#include "src/modules/common/status/status.h"
#include "target.h"

namespace planning {

class TargetMaker {
 public:
  TargetMaker(const SpeedPlannerConfig& speed_planning_config,
              framework::Session* session);
  ~TargetMaker() = default;

  common::Status Run();

  void Reset();

  void AddFinalTargetDataToProto();

  double s_target(const double t) const;

  double v_target(const double t) const;

  const TargetValue& target_value(const double t) const;

 private:
  framework::Session *session_;
  const SpeedPlannerConfig& speed_planning_config_;
  double dt_ = 0.0;
  double plan_time_ = 0.0;
  int32_t plan_points_num_ = 0.0;

  std::vector<TargetValue> target_values_;
  planning::common::FinalTarget final_target_pb_;
};

}  // namespace planning
