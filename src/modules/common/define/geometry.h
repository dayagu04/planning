#ifndef __UTILS__GEOMETRY_H__
#define __UTILS__GEOMETRY_H__

#include "src/modules/common/common.h"
#include "src/common/quaternion.h"

namespace planning {

struct Point2D {
  double x = 0.0;
  double y = 0.0;

  Point2D() = default;
  Point2D(double xx, double yy) : x(xx), y(yy) {}
};

// TODO: this name need to retire. Too much same name for pose or point.
struct QuaternionPose {
  Point3d position{};
  planning_math::IflyQuaternion orientation{};
};
}  // namespace planning

#endif
