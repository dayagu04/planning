#pragma once

#include "ilqr_core.h"

namespace pnc {
namespace lateral_planning {

class ALiLQR : public ilqr_solver::iLqr {
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
}  // namespace lateral_planning
}  // namespace pnc