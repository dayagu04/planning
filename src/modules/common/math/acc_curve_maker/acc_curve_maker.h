#pragma once

#include "trajectory/trajectory_point.h"

namespace planning {

class AccCurveMaker {
 public:
  AccCurveMaker(const std::array<double, 3>& ego_state, const double cipv_s,
                const double cipv_vel);

  ~AccCurveMaker() = default;

  bool Run();

  double ego_dv() const;

  double dv_safe() const;

  double dv_comfortalble() const;

 private:
  double MakeDvSafe();

  double MakeFollowDist();

  bool MakeDvComfortable(double* const dv_curve);

  template <typename T>
  static inline T interpolation(T x, const std::vector<T>& xp,
                                const std::vector<T>& fp) {
    const size_t N = xp.size();
    size_t hi;
    for (hi = 0; hi < N && x > xp[hi]; hi++)
      ;

    if (hi == 0) return fp[0];

    if (hi == N && x > xp[N - 1]) return fp[N - 1];

    const size_t low = hi - 1;
    const T xp_diff = xp[hi] - xp[low];
    if (xp_diff < static_cast<T>(1e-5f) && xp_diff > static_cast<T>(-1e-5f))
      return fp[low];

    return (x - xp[low]) * (fp[hi] - fp[low]) / xp_diff + fp[low];
  }

  // The derivative of sqrt at zero is inf, which makes derivative eval fail.
  // Add an offset to get smooth sqrt.
  template <typename T>
  T smooth_sqrt(const T& in) {
    return sqrt(in + 1e-10) - sqrt(1e-10);
  }

 private:
  const std::array<double, 3>& ego_state_;
  double cipv_s_ = 0.0;
  double cipv_vel_ = 0.0;
  double dv_safe_ = 0.0;
  double dv_curve_ = 0.0;
  double ego_dv_ = 0.0;
};

}  // namespace planning