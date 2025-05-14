#include "trajectory.h"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <iterator>
#include <vector>

#include "log_glog.h"

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

const bool Trajectory::QueryNearestPointWithBuffer(
    const planning_math::Vec2d& position, const double buffer,
    TrajectoryPoint* point) const {
  if (empty()) {
    ILOG_INFO << "empty";
    return false;
  }

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

  if (min_sqr_dist > 1.0) {
    ILOG_INFO << "dist = " << min_sqr_dist;
    return false;
  }

  if (min_index >= size() - 1) {
    *point = data()[min_index];
    return true;
  }

  size_t predecessor_id;
  if (min_index < size() - 1) {
    predecessor_id = min_index;
  } else {
    predecessor_id = min_index - 1;
  }

  planning_math::Vec2d predecessor;
  planning_math::Vec2d successor;
  predecessor = data()[predecessor_id];
  successor = data()[predecessor_id + 1];
  double predecessor_s = data()[predecessor_id].s();

  planning_math::Vec2d base_vector = successor - predecessor;
  if (base_vector.LengthSquare() < 1e-4) {
    *point = data()[predecessor_id];
    return true;
  }

  base_vector.Normalize();
  planning_math::Vec2d predecessor_to_ego(position - predecessor);
  double dot = predecessor_to_ego.InnerProd(base_vector);
  double projection_s = predecessor_s + dot;
  projection_s = std::max(projection_s, 0.0);

  if (projection_s < 1e-3) {
    *point = data()[0];
    return true;
  }

  // ILOG_INFO << "predecessor s = " << predecessor_s
  //           << ",project s = " << projection_s;

  *point = InterpolateUsingLinearApproximation(
      data()[predecessor_id], data()[predecessor_id + 1], projection_s);

  // ILOG_INFO << "lon stitch v = " << point->vel() << ", a = " << point->acc()
  //           << ", s = " << point->s();

  return true;
}

void Trajectory::Clear() {
  clear();
  speed_type_ = common::SpeedProfileType::NONE;
  gear_ = 0;
  return;
}

void Trajectory::SetGear(const int gear) {
  gear_ = gear;
  return;
}

}  // namespace trajectory
}  // namespace planning