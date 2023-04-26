#ifndef ZNQC_MODULES_CONTEXT_RAMP_H_
#define ZNQC_MODULES_CONTEXT_RAMP_H_
#include "limits.h"

namespace planning {

class Ramp {
 public:
  double dis_to_ramp() const {
    return NL_NMAX;
  }
};
}

#endif