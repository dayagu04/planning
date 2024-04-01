#ifndef MODULES_PLANNING_OPTIMIZERS_ST_POINT_H_
#define MODULES_PLANNING_OPTIMIZERS_ST_POINT_H_

#include <string>

#include "vec2d.h"

namespace planning {

typedef std::pair<double, double> Point;

class STPoint : public planning_math::Vec2d {
 public:
  STPoint() = default;
  STPoint(const double s, const double t);
  explicit STPoint(const planning_math::Vec2d& vec2d_point);
  STPoint(const STPoint& point);

  double x() const;
  double y() const;

  double s() const;
  double t() const;
  void set_s(const double s);
  void set_t(const double t);

  void set_x(const double x);
  void set_y(const double y);

 private:
  double s_;
  double t_;
  double x_;
  double y_;
};

}  // namespace planning

#endif /* MODULES_PLANNING_OPTIMIZERS_ST_POINT_H_ */