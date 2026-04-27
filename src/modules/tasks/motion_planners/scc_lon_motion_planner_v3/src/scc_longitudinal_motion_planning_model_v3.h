#ifndef __SCC_LONGITUDINAL_MOTION_PLANNING_MODEL_H__
#define __SCC_LONGITUDINAL_MOTION_PLANNING_MODEL_H__

#include "al_ilqr_model.h"
namespace pnc {
namespace scc_longitudinal_planning_v3 {

class SccLongitudinalMotionPlanningModelV3
    : public al_ilqr_solver::AliLqrModel {
 public:
  al_ilqr_solver::State UpdateDynamicsOneStep(
      const al_ilqr_solver::State &x, const al_ilqr_solver::Control &u,
      const size_t &step) const override;

  void GetDynamicsDerivatives(const al_ilqr_solver::State &x,
                              const al_ilqr_solver::Control & /*u*/,
                              al_ilqr_solver::FxMT &f_x,
                              al_ilqr_solver::FuMT &f_u,
                              const size_t &step) const override;
};
}  // namespace scc_longitudinal_planning_v3
}  // namespace pnc
#endif
