#ifndef __LATERAL_PATH_OPTIMIZER_CORE_H__
#define __LATERAL_PATH_OPTIMIZER_CORE_H__
#include "ilqr_core.h"

namespace planning {
namespace apa_planner {

class LateralPathOptimizerCore : public ilqr_solver::iLqr {
 public:
  void AliLqrIteration() override;
  double MaxConstraintViolation() override;
  void UpdateAugmentedLagragian() override;
  double MaxDerivationValue() override;

  void PrintAlParamInfo() override;
  void PrintAlParamInfoAfter() override;

 protected:
  bool ForwardPass(double &new_cost, double &expected,
                   const size_t &iter) override;

  bool BackwardPass() override;
};
}  // namespace apa_planner
}  // namespace planning

#endif