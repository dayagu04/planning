#pragma once

#include "apa_speed_smoother_config.pb.h"
#include "planning_plan.pb.h"

#include "math/line_segment2d.h"
#include "speed_smoother/apa_speed_limit_decider.h"

namespace planning {

class ApaSpeedSmoother {
 public:
  ApaSpeedSmoother();

  virtual ~ApaSpeedSmoother() = default;

  bool Smooth(const std::vector<planning_math::LineSegment2d>& obstacles,
              ::PlanningOutput::PlanningOutput* const planning_output);

 private:
  bool Optimize();

 private:
  ApaSpeedSmootherConfig config_;
  ApaSpeedLimitDecider speed_limit_decider_;
  double delta_s_ = 0.0;
};

}  // namespace planning
