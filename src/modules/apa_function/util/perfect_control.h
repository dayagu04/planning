#ifndef __PERFECT_CONTROL_H__
#define __PERFECT_CONTROL_H__

#include "Eigen/Core"
#include "planning_plan_c.h"
namespace planning {

namespace apa_planner {
class PerfectControl {
 public:
  struct DynamicState {
    DynamicState() {}
    DynamicState(const Eigen::Vector2d& pos_r, const double heading_r) {
      pos = pos_r;
      heading = heading_r;
    }

    double vel = 0.0;
    Eigen::Vector2d pos = Eigen::Vector2d::Zero();
    double heading = 0.0;
    double static_time = 0.0;
    bool static_flag = false;

    void Reset() {
      vel = 0.0;
      pos = Eigen::Vector2d::Zero();
      heading = 0.0;
    }
  };

 public:
  void Init();
  void SetState(const DynamicState& state) { state_ = state; }

  const bool Update(const iflyauto::PlanningOutput& planning_output,
                    const double dt);

  const DynamicState& GetState() const { return state_; }

 private:
  DynamicState state_;
};
}  // namespace apa_planner
}  // namespace planning
#endif
