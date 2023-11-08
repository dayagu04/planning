#ifndef __GENERAL_PLANNING_CONTEXT_H__
#define __GENERAL_PLANNING_CONTEXT_H__

#include <cstdint>
#include <memory>

#ifndef g_context
#define g_context planning::context::GeneralPlanningContext::GetInstance()
#endif

namespace planning {
namespace context {

enum PlannerType {
  REALTIME_PLANNER = 0,
  REALTIME_PLANNER_WITH_MOTION = 1,
  LONGTIME_PLANNER = 2,
};

struct PlanningPram {
  uint8_t planner_type = 0;
};

struct PlanningStatemachine {
  bool dbw_status = false;
  bool apa_reset_flag = false;
  uint8_t scene_type = 0;
  uint8_t current_state = 0;
};

class GeneralPlanningContext {
 public:
  static GeneralPlanningContext &GetInstance() {
    static GeneralPlanningContext instance;
    return instance;
  }

  // planning parameters
  const PlanningPram &GetParam() const { return param_; }
  PlanningPram &MutablePram() { return param_; }

  // planning statemachine
  const PlanningStatemachine &GetStatemachine() const { return statemachine_; }
  PlanningStatemachine &MutableStatemachine() { return statemachine_; }

 private:
  GeneralPlanningContext(){};

  // planning parameters
  PlanningPram param_;

  // planning statemachine
  PlanningStatemachine statemachine_;
};

}  // namespace context
}  // namespace planning

#endif
