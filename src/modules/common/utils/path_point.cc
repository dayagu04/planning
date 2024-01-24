#include "utils/path_point.h"

#include <sstream>

using namespace planning;
using namespace planning::planning_math;
namespace planning {
namespace planning_math {

PathPoint PathPoint::GetInterpolateByLinearApproximation(const PathPoint& p,
                                                         const double s) const {
  double s0 = s_;
  double s1 = p.s();
  double l = 0.0;

  double r = (s - s0) / (s1 - s0);
  double x = lerp(x_, p.x(), r);
  double y = lerp(y_, p.y(), r);
  double z = lerp(z_, p.z(), r);
  double theta = slerp(theta_, s_, p.theta(), p.s(), s);
  double kappa = lerp(kappa_, p.kappa(), r);
  double dkappa = lerp(dkappa_, p.dkappa(), r);
  double ddkappa = lerp(ddkappa_, p.ddkappa(), r);
  PathPoint res_pt(x, y, z, s, l, theta, kappa, dkappa, ddkappa);
  return res_pt;
}

}  // namespace planning_math
}  // namespace planning
