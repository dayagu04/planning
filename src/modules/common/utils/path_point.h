#pragma once

#include <limits>
#include <string>
#include <vector>

#include "math/linear_interpolation.h"
#include "vec2d.h"
using namespace planning;
using namespace planning::planning_math;
namespace planning {
namespace planning_math {
class PathPoint : public Vec2d {
 public:
  PathPoint() = default;

  ~PathPoint() = default;

  PathPoint(double x, double y) : Vec2d(x, y) {}

  PathPoint(double x, double y, double s, double l, double theta, double kappa,
            double dkappa, double ddkappa) noexcept
      : s_(s),
        l_(l),
        theta_(theta),
        kappa_(kappa),
        dkappa_(dkappa),
        ddkappa_(ddkappa),
        Vec2d(x, y) {}

  PathPoint(double x, double y, double z, double s, double l, double theta,
            double kappa, double dkappa, double ddkappa) noexcept
      : z_(z),
        s_(s),
        l_(l),
        theta_(theta),
        kappa_(kappa),
        dkappa_(dkappa),
        ddkappa_(ddkappa),
        Vec2d(x, y) {}

  double x() const { return x_; }

  double y() const { return y_; }

  double z() const { return z_; }

  double s() const { return s_; }

  double l() const { return l_; }

  double theta() const { return theta_; }

  double kappa() const { return kappa_; }

  double dkappa() const { return dkappa_; }

  double ddkappa() const { return ddkappa_; }

  void set_x(const double x) { x_ = x; }

  void set_y(const double y) { y_ = y; }

  void set_z(const double z) { z_ = z; }

  void set_s(const double s) { s_ = s; }

  void set_l(const double l) { l_ = l; }

  void set_theta(const double theta) { theta_ = theta; }

  void set_kappa(const double kappa) { kappa_ = kappa; }

  void set_dkappa(const double dkappa) { dkappa_ = dkappa; }

  void set_ddkappa(const double ddkappa) { ddkappa_ = ddkappa; }

  PathPoint GetInterpolateByLinearApproximation(const PathPoint& p,
                                                const double s) const;

  std::string DebugString() const;

 protected:
  double z_ = 0.0;
  double s_ = -1.0;
  double l_ = 0.0;
  double theta_ = 0.0;
  double kappa_ = 0.0;
  double dkappa_ = 0.0;
  double ddkappa_ = 0.0;
};
}  // namespace planning_math

}  // namespace planning
