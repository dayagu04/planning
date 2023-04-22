#ifndef ZNQC_MODULES_CONTEXT_INTERSECTION_H_
#define ZNQC_MODULES_CONTEXT_INTERSECTION_H_
#include "limits.h"

namespace planning {

class Intersection {
 public:

  bool is_in_intersection() {
    return false;
  }
  double dist_to_intsect() {
    return NL_NMAX;
  }

  double dist_to_last_intsect() {
    return NL_NMAX;
  }
  
  double intsect_length() {
    return NL_NMAX;
  }

  double dis_to_ramp() {
    return NL_NMAX;
  }
};
}

#endif