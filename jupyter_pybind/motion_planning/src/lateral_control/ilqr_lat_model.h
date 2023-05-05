#ifndef __ILQR_LAT_MODEL_H_
#define __ILQR_LAT_MODEL_H_

#include "ilqr_model.h"

class iLqrLatModel : public ilqr_solver::iLqrModel {
public:
  State UpdateDynamicsOneStep(const State &x, const Control &u,
                              const size_t &step) const override;

  void GetDynamicsDerivatives(const State &x, const Control & /*u*/, FxMT &f_x,
                              FuMT &f_u, const size_t &step) const override;

};

#endif