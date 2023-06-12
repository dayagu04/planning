#ifndef MODULES_PLANNING_OUTPUT_CONTEXT_
#define MODULES_PLANNING_OUTPUT_CONTEXT_

#include "define/planning_status.h"
#include "planning_hmi.pb.h"

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
  const PlanningHMI::PlanningHMIOutputInfoStr planning_hmi_Info() const { return planning_hmi_Info_; }
  PlanningHMI::PlanningHMIOutputInfoStr* mutable_planning_hmi_Info()  { return &planning_hmi_Info_; }
private:
  FallBackInfo fallback_info_;
  common::PlanningStatus planning_status_;
  PlanningHMI::PlanningHMIOutputInfoStr planning_hmi_Info_;
  common::PlanningStatus prev_planning_status_;
};

} // namespace planning

#endif