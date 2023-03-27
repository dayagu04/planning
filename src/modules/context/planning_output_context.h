#ifndef MODULES_PLANNING_OUTPUT_CONTEXT_
#define MODULES_PLANNING_OUTPUT_CONTEXT_

#include "modules/common/define/planning_status.h"

namespace planning {

class PlanningOutputContext {
public:
  void reset() {

  }
  struct FallBackInfo {
    std::string last_successful_path_label;
  };

  const FallBackInfo &fallback_info() const { return fallback_info_; }
  FallBackInfo *mutable_fallback_info() { return &fallback_info_; }

  const common::PlanningStatus &planning_status() const { return planning_status_; }
  common::PlanningStatus *mutable_planning_status() { return &planning_status_; }

  const common::PlanningStatus &prev_planning_status() const {
    return prev_planning_status_;
  }
  common::PlanningStatus *mutable_prev_planning_status() {
    return &prev_planning_status_;
  }

private:
  FallBackInfo fallback_info_;
  common::PlanningStatus planning_status_;
  common::PlanningStatus prev_planning_status_;
};

} // namespace planning

#endif