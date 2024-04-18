#include "trajectory.h"
#include "trajectory_point.h"

#include <cstdint>
#include <iterator>
#include <vector>

namespace planning {
namespace trajectory {

Trajectory::Trajectory(std::vector<TrajectoryPoint>&& trajectory_points)
    : std::vector<TrajectoryPoint>(std::move(trajectory_points)) {}

TrajectoryPoint Trajectory::Evaluate(const double query_time) const {
  auto func = [](const TrajectoryPoint& tp, const double absolute_time) {
    return tp.absolute_time() < absolute_time;
  };
  auto iter = std::lower_bound(begin(), end(), query_time, func);
  if (iter == begin()) {
    return front();
  }

  if (iter == end()) {
    return back();
  }
  return (iter - 1)->InterpolateTrajectoryPoint(*iter, query_time);
}

const double Trajectory::GetTimeLength() const {
  return back().absolute_time() - front().absolute_time();
}

size_t Trajectory::QueryLowerBoundPoint(const double absolute_time) const {
  assert(!empty());

  if (absolute_time >= back().absolute_time()) {
    return size() - 1;
  }

  auto compare = [](const TrajectoryPoint& pt, const double absolute_time) {
    return pt.absolute_time() < absolute_time;
  };

  auto it_lower = std::lower_bound(begin(), end(), absolute_time, compare);

  return std::distance(begin(), it_lower);
}

size_t Trajectory::QueryLowerBoundPointByS(const double s) const {
  if (s > back().s()) {
    return size() - 1;
  }

  auto compare = [](const TrajectoryPoint& pt, const double s) {
    return pt.s() < s;
  };

  auto it_lower = std::lower_bound(begin(), end(), s, compare);
  return std::distance(begin(), it_lower);
}

size_t Trajectory::QueryNearestPoint(
    const planning_math::Vec2d& position) const {
  double min_sqr_dist = std::numeric_limits<double>::max();
  size_t min_index = 0;
  for (size_t i = 0; i < size(); ++i) {
    const planning_math::Vec2d curr_point(data()[i].x(), data()[i].y());

    const double curr_sqr_dist_m = curr_point.DistanceSquareTo(position);
    if (curr_sqr_dist_m < min_sqr_dist) {
      min_sqr_dist = curr_sqr_dist_m;
      min_index = i;
    }
  }
  return min_index;
}

size_t Trajectory::QueryNearestPointWithBuffer(
    const planning_math::Vec2d& position, const double buffer) const {
  double min_sqr_dist = std::numeric_limits<double>::max();
  size_t min_index = 0;
  for (size_t i = 0; i < size(); ++i) {
    const planning_math::Vec2d curr_point(data()[i].x(), data()[i].y());

    const double curr_sqr_dist_m = curr_point.DistanceSquareTo(position);
    if (curr_sqr_dist_m < min_sqr_dist + buffer) {
      min_sqr_dist = curr_sqr_dist_m;
      min_index = i;
    }
  }
  return min_index;
}

}  // namespace trajectory
}  // namespace planning