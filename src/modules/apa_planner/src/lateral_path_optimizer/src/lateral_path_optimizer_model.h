#ifndef __LATERAL_PATH_OPTIMIZER_MODEL_H__
#define __LATERAL_PATH_OPTIMIZER_MODEL_H__

#include "ilqr_model.h"

namespace planning {
namespace apa_planner {
class LateralPathOptimizerModel : public ilqr_solver::iLqrModel {
 public:
  ilqr_solver::State UpdateDynamicsOneStep(const ilqr_solver::State &x,
                                           const ilqr_solver::Control &u,
                                           const size_t &step) const override;
  void GetDynamicsDerivatives(const ilqr_solver::State &x,
                              const ilqr_solver::Control & /*u*/,
                              ilqr_solver::FxMT &f_x, ilqr_solver::FuMT &f_u,
                              const size_t &step) const override;
};

}  // namespace apa_planner
}  // namespace planning

#endif