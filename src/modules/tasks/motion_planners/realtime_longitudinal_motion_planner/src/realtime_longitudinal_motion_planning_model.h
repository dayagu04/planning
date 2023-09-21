#ifndef __REALTIME_LONGITUDINAL_MOTION_PLANNING_MODEL_H__
#define __REALTIME_LONGITUDINAL_MOTION_PLANNING_MODEL_H__

#include "ilqr_model.h"
namespace pnc {
namespace realtime_longitudinal_planning {
class RealtimeLongitudinalMotionPlanningModel : public ilqr_solver::iLqrModel {
 public:
  State UpdateDynamicsOneStep(const State &x, const Control &u,
                              const size_t &step) const override;

  void GetDynamicsDerivatives(const State &x, const Control & /*u*/, FxMT &f_x,
                              FuMT &f_u, const size_t &step) const override;
};
}  // namespace realtime_longitudinal_planning
}  // namespace pnc
#endif
