#pragma once

#include <memory>

#include "apa_planner/apa_planner_base.h"

#include "frame.h"

namespace planning {
namespace apa_planner {

class ApaPlannerDispatcher {
 public:
  ApaPlannerDispatcher();
  virtual ~ApaPlannerDispatcher() = default;

  bool Update(framework::Frame* const frame);

 private:
  void RegisterPlanners();

 private:
  std::vector<std::unique_ptr<ApaPlannerBase>> planner_list_;
};

}  // namespace apa_planner
}  // namespace planning