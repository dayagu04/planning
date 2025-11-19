#pragma once
#include "ilqr_model.h"
#define EGO_STATE_SIZE 6
#define EGO_CONTROL_SIZE 2
#define OBS_STATE_SIZE 6
#define OBS_CONTROL_SIZE 2
namespace pnc {
namespace joint_motion_planning {
class JointMotionPlanningModel : public ilqr_solver::iLqrModel {
 public:
  ilqr_solver::State UpdateDynamicsOneStep(const ilqr_solver::State &x0,
                                           const ilqr_solver::Control &u,
                                           const size_t &step) const override;
  void GetDynamicsDerivatives(const ilqr_solver::State &x0,
                              const ilqr_solver::Control &u,
                              ilqr_solver::FxMT &f_x, ilqr_solver::FuMT &f_u,
                              const size_t &step) const override;
};
}  // namespace joint_motion_planning
}  // namespace pnc