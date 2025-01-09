#ifndef MODULES_PLANNING_COMMON_DISCRETIZED_PATH_H_
#define MODULES_PLANNING_COMMON_DISCRETIZED_PATH_H_

#include <vector>

#include "config/message_type.h"
#include "math/linear_interpolation.h"

namespace planning {

class DiscretizedPath : public std::vector<planning_math::PathPoint> {
 public:
  DiscretizedPath() = default;

  explicit DiscretizedPath(
      const std::vector<planning_math::PathPoint>& path_points);

  double Length() const;

  planning_math::PathPoint Evaluate(const double path_s) const;

  planning_math::PathPoint EvaluateReverse(const double path_s) const;

  double QueryMatchedS(const planning_math::PathPoint& path_point) const;

 protected:
  std::vector<planning_math::PathPoint>::const_iterator QueryLowerBound(
      const double path_s) const;
  std::vector<planning_math::PathPoint>::const_iterator QueryUpperBound(
      const double path_s) const;
};

}  // namespace planning

#endif /* MODULES_PLANNING_COMMON_DISCRETIZED_PATH_H_ */
