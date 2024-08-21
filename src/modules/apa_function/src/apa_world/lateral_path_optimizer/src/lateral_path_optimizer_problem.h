#ifndef __LATERAL_PATH_OPTIMIZER_PROBLEM_H__
#define __LATERAL_PATH_OPTIMIZER_PROBLEM_H__

#include <cstddef>
#include <memory>

#include "lateral_path_optimizer.pb.h"
#include "lateral_path_optimizer_core.h"
#include "lateral_path_optimizer_cost.h"
#include "lateral_path_optimizer_model.h"

namespace planning {
namespace apa_planner {
class LateralPathOptimizerProblem {
 public:
  void Init(const bool c_ilqr_enable);
  uint8_t Update(planning::common::LateralPathOptimizerInput &planning_input,
                 const uint8_t gear_cmd);

  const planning::common::LateralPathOptimizerOutput &GetOutput() {
    return planning_output_;
  }

  void Reset();
  const std::shared_ptr<LateralPathOptimizerCore> GetiLqrCorePtr() const {
    return ilqr_core_ptr_;
  }

 private:
  std::shared_ptr<LateralPathOptimizerCore> ilqr_core_ptr_;
  planning::common::LateralPathOptimizerOutput planning_output_;
  ilqr_solver::State init_state_;
};
}  // namespace apa_planner
}  // namespace planning

#endif