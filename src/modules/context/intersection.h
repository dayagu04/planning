#ifndef ZNQC_MODULES_CONTEXT_INTERSECTION_H_
#define ZNQC_MODULES_CONTEXT_INTERSECTION_H_
#include "limits.h"

namespace planning {

class Intersection {
 public:

  bool is_in_intersection() const {
    return false;
  }
  double dist_to_intsect() const{
    return NL_NMAX;
  }

  double dist_to_last_intsect() const{
    return NL_NMAX;
  }
  
  double intsect_length() const {
    return NL_NMAX;
  }

  double dis_to_ramp() const {
    return NL_NMAX;
  }
};
}

#endif