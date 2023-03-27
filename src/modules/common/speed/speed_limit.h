#ifndef MODULES_PLANNING_OPTIMIZERS_SPEED_LIMIT_H_
#define MODULES_PLANNING_OPTIMIZERS_SPEED_LIMIT_H_

#include <utility>
#include <vector>

namespace planning {

class SpeedLimit {
 public:
  SpeedLimit() = default;

  void AppendSpeedLimit(const double t, const double v);

  const std::vector<std::pair<double, double>>& speed_limit_points() const;

  double GetSpeedLimitByT(const double t) const;

  void Clear();

 private:
  // use a vector to represent speed limit
  // the first number is t, the second number is v
  // It means at time t from the start point, the speed limit is v.
  std::vector<std::pair<double, double>> speed_limit_points_;
};

}  // namespace planning

#endif /* MODULES_PLANNING_OPTIMIZERS_SPEED_LIMIT_H_ */
