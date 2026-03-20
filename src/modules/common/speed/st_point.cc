#include "speed/st_point.h"
#include "vec2d.h"

namespace planning {

STPoint::STPoint(const double s, const double t)
    : t_(t), s_(s), planning_math::Vec2d(t, s) {}

STPoint::STPoint(const planning_math::Vec2d& point)
    : s_(point.y()), t_(point.x()), planning_math::Vec2d(point) {}

STPoint::STPoint(const STPoint& point)
    : s_(point.s()),
      t_(point.t()),
      planning_math::Vec2d(point.t(), point.s()) {}

double STPoint::s() const { return s_; }

double STPoint::t() const { return t_; }

void STPoint::set_s(const double s) { s_ = s; }

void STPoint::set_t(const double t) { t_ = t; }

}  // namespace planning
