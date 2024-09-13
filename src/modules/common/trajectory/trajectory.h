#pragma once

#include <cstdint>
#include <vector>

#include "trajectory_point.h"

namespace planning {
namespace trajectory {

class Trajectory : public std::vector<TrajectoryPoint> {
 public:
  Trajectory() = default;
  explicit Trajectory(std::vector<TrajectoryPoint>&& trajectory_points);
  virtual ~Trajectory() = default;

  TrajectoryPoint Evaluate(const double query_time) const;
  const bool is_elements_vec_ready() const {
    return traj_elements_vec_ready_flag_;
  }

  const double GetTimeLength() const;

  size_t QueryLowerBoundPoint(const double absolute_time) const;
  size_t QueryLowerBoundPointByS(const double s) const;
  size_t QueryNearestPoint(const planning_math::Vec2d& position) const;
  size_t QueryNearestPointWithBuffer(const planning_math::Vec2d& position,
                                     const double buffer) const;

  bool traj_elements_vec_ready_flag_ = false;
  std::vector<double> x_vec_{};
  std::vector<double> y_vec_{};
  std::vector<double> theta_vec_{};
};

}  // namespace trajectory
}  // namespace planning