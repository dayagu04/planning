#ifndef __SCC_LONGITUDINAL_MOTION_PLANNING_MODEL_H__
#define __SCC_LONGITUDINAL_MOTION_PLANNING_MODEL_H__

#include "ilqr_model.h"
namespace pnc {
namespace scc_longitudinal_planning {
class SccLongitudinalMotionPlanningModel : public ilqr_solver::iLqrModel {
 public:
  ilqr_solver::State UpdateDynamicsOneStep(const ilqr_solver::State &x,
                                           const ilqr_solver::Control &u,
                                           const size_t &step) const override;

  void GetDynamicsDerivatives(const ilqr_solver::State &x,
                              const ilqr_solver::Control & /*u*/,
                              ilqr_solver::FxMT &f_x, ilqr_solver::FuMT &f_u,
                              const size_t &step) const override;
};
}  // namespace scc_longitudinal_planning
}  // namespace pnc
#endif
