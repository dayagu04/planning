#pragma once

#include "frame.h"

namespace planning {
namespace apa_planner {

class ApaPlannerBase {
 public:
  ApaPlannerBase() = default;
  virtual ~ApaPlannerBase() = default;

  virtual bool Update(framework::Frame* const frame) = 0;

 protected:
  void SetFailedPlanningOutput(framework::Frame* const frame) const;
};

} // namespace apa_planner
} // namespace planning