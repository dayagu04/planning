#pragma once

#include <vector>

#include "planning_plan.pb.h"

#include "common/apa_speed_limit.h"
#include "math/line_segment2d.h"

namespace planning {

class ApaSpeedLimitDecider {
 public:
  ApaSpeedLimitDecider() = default;

  ~ApaSpeedLimitDecider() = default;

  bool GetSpeedLimits(const ::PlanningOutput::PlanningOutput& planning_output,
                      const std::vector<planning_math::LineSegment2d>& obstacles,
                      ApaSpeedLimit* const speed_limit_data) const;

 private:
  double CalSpeedLimitFromObstacleDistance(const double dis_from_obstacle) const;
};

}  // namespace planning
