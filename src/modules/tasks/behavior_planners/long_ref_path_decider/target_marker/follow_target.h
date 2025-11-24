#pragma once

#include <cstdint>
#include <memory>
#include "ego_planning_config.h"
#include "lon_target_maker.pb.h"
#include "session.h"
#include "target.h"

namespace planning {

class FollowTarget : public Target {
  struct UpperBoundInfo {
    double s = 0.0;
    double t = 0.0;
    double v = 0.0;
    TargetType target_type = TargetType::kNotSet;
    int32_t agent_id = -1;
    int64_t st_boundary_id = -1;
  };

 public:
  FollowTarget(const SpeedPlannerConfig config, framework::Session* session);
  ~FollowTarget() = default;

  void Update();

  double MakeSlowerFollowSTarget(const double speed, const double upper_bound_s,
                                 const double time_gap) const;

 private:
  void GenerateUpperBoundInfo();

  void GenerateFollowTarget();

  void GenerateRadsFollowTarget();

  void MakeMinFollowDistance();

  bool MakeSValueWithTargetFollowCurve(const int32_t index,
                                       const bool has_valid_s_value,
                                       double* const target_s_value) const;

  void AddFollowTargetDataToProto();

 private:
  std::vector<UpperBoundInfo> upper_bound_infos_;
  planning::common::FollowTarget follow_target_pb_;
  double min_follow_distance_m_ = 3.0;
};

}  // namespace planning