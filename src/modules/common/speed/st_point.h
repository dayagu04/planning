#ifndef MODULES_PLANNING_OPTIMIZERS_ST_POINT_H_
#define MODULES_PLANNING_OPTIMIZERS_ST_POINT_H_

#include <string>

#include "vec2d.h"

namespace planning {

class STPoint : public planning_math::Vec2d {
 public:
  STPoint() = default;
  STPoint(const double s, const double t);
  explicit STPoint(const planning_math::Vec2d& vec2d_point);
  STPoint(const STPoint& point);

  double s() const;
  double t() const;
  void set_s(const double s);
  void set_t(const double t);

 private:
  double s_;
  double t_;
};

}  // namespace planning

#endif /* MODULES_PLANNING_OPTIMIZERS_ST_POINT_H_ */