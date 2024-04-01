#include "speed/st_point.h"

namespace planning {

STPoint::STPoint(const double s, const double t) : Vec2d(t, s) {}

STPoint::STPoint(const planning_math::Vec2d& vec2d_point)
    : Vec2d(vec2d_point) {}

STPoint::STPoint(const STPoint& point) : Vec2d(point.t(), point.s()) {}

double STPoint::s() const { return s_; }

double STPoint::t() const { return t_; }

double STPoint::x() const { return x_; }

double STPoint::y() const { return y_; }

void STPoint::set_s(const double s) { s_ = s; }

void STPoint::set_t(const double t) { t_ = t; }

void STPoint::set_x(const double x) { x_ = x; }

void STPoint::set_y(const double y) { y_ = y; };
}  // namespace planning
