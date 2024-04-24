#pragma once

#include "trajectory_point.h"

#include <cstdint>
#include <vector>

namespace planning {
namespace trajectory {

class Trajectory : public std::vector<TrajectoryPoint> {
 public:
  Trajectory() = default;
  explicit Trajectory(std::vector<TrajectoryPoint>&& trajectory_points);
  virtual ~Trajectory() = default;

  TrajectoryPoint Evaluate(const double query_time) const;

  const double GetTimeLength() const;

  size_t QueryLowerBoundPoint(const double absolute_time) const;
  size_t QueryLowerBoundPointByS(const double s) const;
  size_t QueryNearestPoint(const planning_math::Vec2d& position) const;
  size_t QueryNearestPointWithBuffer(const planning_math::Vec2d& position,
                                     const double buffer) const;
};

}  // namespace trajectory
}  // namespace planning