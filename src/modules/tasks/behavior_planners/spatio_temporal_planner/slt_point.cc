
#include <assert.h>
#include "config/basic_type.h"
#include "define/geometry.h"
#include "session.h"
#include "task_basic_types.h"
#include "slt_point.h"

namespace planning {
using namespace planning_math;

SLTPoint::SLTPoint(const double s, const double l, const double t) {
  s_ = s;
  l_ = l;
  t_ = t;
}

double SLTPoint::s() const { return s_; }

double SLTPoint::l() const { return l_; }

double SLTPoint::t() const { return t_; }

void SLTPoint::set_s(const double s) { s_ = s; }

void SLTPoint::set_l(const double l) { l_ = l; }

void SLTPoint::set_t(const double t) { t_ = t; }

}  // namespace planning