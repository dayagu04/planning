#pragma once

#include <cstdint>
#include <map>
#include <string>
#include <vector>

#include "../speed_limit_decider/speed_limit_decider_output.h"
#include "planning_plan_c.h"

namespace planning {

struct HPPSpeedLimitAgent {
  int32_t id;
  double min_s;
  double v_limit;
  bool is_collison;
  bool is_need_v_hold;
  double v_follow_desired;
};

class HPPSpeedLimitDeciderOutput {
 public:
  HPPSpeedLimitDeciderOutput() = default;
  ~HPPSpeedLimitDeciderOutput() = default;

  void SetSpeedLimit(const double limited_speed,
                     const SpeedLimitType& speed_limit_type);

  bool GetSpeedLimit(double* const limited_speed,
                     SpeedLimitType* const speed_limit_type) const;

  void SetSpeedLimitIntoMap(const double limited_speed,
                            const SpeedLimitType& speed_limit_type);

  bool GetSpeedLimitByType(const SpeedLimitType& speed_limit_type,
                           double* const limited_speed) const;

 private:
  std::map<SpeedLimitType, double>
      hpp_speed_limit_map_;  //(type, speedlimit) for all scenes one by one
  std::pair<SpeedLimitType, double>
      hpp_speed_limit_type_final_;  // final speed limit and type
};
}  // namespace planning