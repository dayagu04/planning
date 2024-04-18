#include "path_point.h"
#include <sstream>
#include "math/linear_interpolation.h"

namespace planning {
namespace trajectory {

PathPoint PathPoint::GetInterpolateByLinearApproximation(const PathPoint& p,
                                                         const double s) const {
  double s0 = s_;
  double s1 = p.s();
  double l = 0.0;

  double r = (s - s0) / (s1 - s0);
  double x = planning_math::lerp(x_, p.x(), r);
  double y = planning_math::lerp(y_, p.y(), r);
  double z = planning_math::lerp(z_, p.z(), r);
  double theta = planning_math::slerp(theta_, s_, p.theta(), p.s(), s);
  double kappa = planning_math::lerp(kappa_, p.kappa(), r);
  double dkappa = planning_math::lerp(dkappa_, p.dkappa(), r);
  double ddkappa = planning_math::lerp(ddkappa_, p.ddkappa(), r);
  PathPoint res_pt(x, y, z, s, l, theta, kappa, dkappa, ddkappa);
  return res_pt;
}

std::string PathPoint::DebugString() const {
  std::stringstream ss;
  ss << "PathPoint ( x = " << x_ << ", y = " << y_ << ", z = " << z_
     << ", heading = " << theta_ << ", s = " << s_ << ", kappa = " << kappa_
     << ", dkappa = " << dkappa_ << ", ddkappa = " << ddkappa_ << " )";

  return ss.str();
}

}  // namespace trajectory
}  // namespace planning