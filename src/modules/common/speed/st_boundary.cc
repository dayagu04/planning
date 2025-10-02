#include "speed/st_boundary.h"

#include <iostream>

#include "assert.h"
#include "common.h"
#include "math/math_utils.h"
#include "log_glog.h"

namespace planning {

STBoundary::STBoundary(
    const std::vector<std::pair<STPoint, STPoint>>& point_pairs) {
  std::vector<std::pair<STPoint, STPoint>> reduced_pairs(point_pairs);

  for (const auto& item : reduced_pairs) {
    // use same t for both points
    const double t = item.first.t();
    lower_points_.emplace_back(item.first);
    upper_points_.emplace_back(item.second);
  }

  for (const auto& point : lower_points_) {
    min_s_ = std::min(min_s_, point.s());
  }
  for (const auto& point : upper_points_) {
    max_s_ = std::max(max_s_, point.s());
  }
  min_t_ = lower_points_.front().t();
  max_t_ = lower_points_.back().t();
}

bool STBoundary::IsPointNear(const planning_math::LineSegment2d& seg,
                             const planning_math::Vec2d& point,
                             const double max_dist) {
  return seg.DistanceSquareTo(point) < max_dist * max_dist;
}

bool STBoundary::IsValid(
    const std::vector<std::pair<STPoint, STPoint>>& point_pairs) const {
  if (point_pairs.size() < 2) {
    return false;
  }

  constexpr double kStBoundaryEpsilon = 1e-9;
  constexpr double kMinDeltaT = 1e-6;
  for (size_t i = 0; i < point_pairs.size(); ++i) {
    const auto& curr_lower = point_pairs[i].first;
    const auto& curr_upper = point_pairs[i].second;
    if (curr_upper.s() < curr_lower.s()) {
      return false;
    }

    if (abs(curr_lower.t() - curr_upper.t()) > kStBoundaryEpsilon) {
      return false;
    }

    if (i + 1 != point_pairs.size()) {
      const auto& next_lower = point_pairs[i + 1].first;
      const auto& next_upper = point_pairs[i + 1].second;
      if (std::max(curr_lower.t(), curr_upper.t()) + kMinDeltaT >=
          std::min(next_lower.t(), next_upper.t())) {
        return false;
      }
    }
  }
  return true;
}

bool STBoundary::IsPointInBoundary(const STPoint& st_point) const {
  if (st_point.t() <= min_t_ || st_point.t() >= max_t_) {
    return false;
  }
  size_t left = 0;
  size_t right = 0;
  if (!GetIndexRange(lower_points_, st_point.t(), &left, &right)) {
    return false;
  }
  const double check_upper = planning_math::CrossProd(
      st_point, upper_points_[left], upper_points_[right]);
  const double check_lower = planning_math::CrossProd(
      st_point, lower_points_[left], lower_points_[right]);

  return (check_upper * check_lower < 0);
}

STPoint STBoundary::upper_left_point() const {
  assert(!upper_points_.empty());
  return upper_points_.front();
}

STPoint STBoundary::upper_right_point() const {
  assert(!upper_points_.empty());
  return upper_points_.back();
}

STPoint STBoundary::bottom_left_point() const {
  assert(!lower_points_.empty());
  return lower_points_.front();
}

STPoint STBoundary::bottom_right_point() const {
  assert(!lower_points_.empty());
  return lower_points_.back();
}

STBoundary STBoundary::ExpandByS(const double s) const {
  if (lower_points_.empty()) {
    return STBoundary();
  }
  std::vector<std::pair<STPoint, STPoint>> point_pairs;
  for (size_t i = 0; i < lower_points_.size(); ++i) {
    point_pairs.emplace_back(
        STPoint(lower_points_[i].s() - s, lower_points_[i].t()),
        STPoint(upper_points_[i].s() + s, upper_points_[i].t()));
  }
  return STBoundary(std::move(point_pairs));
}

STBoundary STBoundary::ShiftByS(const double s) const {
  if (lower_points_.empty()) {
    return STBoundary();
  }
  std::vector<std::pair<STPoint, STPoint>> point_pairs;
  for (size_t i = 0; i < lower_points_.size(); ++i) {
    point_pairs.emplace_back(
        STPoint(lower_points_[i].s() + s, lower_points_[i].t()),
        STPoint(upper_points_[i].s() + s, upper_points_[i].t()));
  }
  return STBoundary(std::move(point_pairs));
}

STBoundary STBoundary::ShrinkByDs(const double ds, const double begin_t) const {
  if (lower_points_.empty()) {
    return STBoundary();
  }
  // attention(@all) keep first timestep not shrinked
  // double start_t = std::max(begin_t, lower_points_.front().t());
  double start_t = begin_t;
  std::vector<std::pair<STPoint, STPoint>> point_pairs;
  for (size_t i = 0; i < lower_points_.size(); ++i) {
    if (lower_points_[i].t() <= start_t) {
      point_pairs.emplace_back(
          STPoint(lower_points_[i].s(), lower_points_[i].t()),
          STPoint(upper_points_[i].s(), upper_points_[i].t()));
    } else {
      double lower_s =
          lower_points_[i].s() + ds * (lower_points_[i].t() - start_t);
      double upper_s =
          upper_points_[i].s() - ds * (upper_points_[i].t() - start_t);
      if (lower_s >= upper_s) {
        break;
      }
      point_pairs.emplace_back(STPoint(lower_s, lower_points_[i].t()),
                               STPoint(upper_s, upper_points_[i].t()));
    }
  }
  if (point_pairs.empty()) {
    return STBoundary();
  }
  return STBoundary(std::move(point_pairs));
}

STBoundary STBoundary::ExpandByT(const double t) const {
  if (lower_points_.empty()) {
    ILOG_DEBUG << "The current st_boundary has NO points";
    return STBoundary();
  }

  std::vector<std::pair<STPoint, STPoint>> point_pairs;

  const double left_delta_t = lower_points_[1].t() - lower_points_[0].t();
  const double lower_left_delta_s = lower_points_[1].s() - lower_points_[0].s();
  const double upper_left_delta_s = upper_points_[1].s() - upper_points_[0].s();

  point_pairs.emplace_back(
      STPoint(lower_points_[0].s() - t * lower_left_delta_s / left_delta_t,
              lower_points_[0].t() - t),
      STPoint(upper_points_[0].s() - t * upper_left_delta_s / left_delta_t,
              upper_points_.front().t() - t));

  const double kMinSEpsilon = 1e-3;
  point_pairs.front().first.set_s(
      std::min(point_pairs.front().second.s() - kMinSEpsilon,
               point_pairs.front().first.s()));

  for (size_t i = 0; i < lower_points_.size(); ++i) {
    point_pairs.emplace_back(lower_points_[i], upper_points_[i]);
  }

  size_t length = lower_points_.size();
  assert(length > 2);

  const double right_delta_t =
      lower_points_[length - 1].t() - lower_points_[length - 2].t();
  const double lower_right_delta_s =
      lower_points_[length - 1].s() - lower_points_[length - 2].s();
  const double upper_right_delta_s =
      upper_points_[length - 1].s() - upper_points_[length - 2].s();

  point_pairs.emplace_back(STPoint(lower_points_.back().s() +
                                       t * lower_right_delta_s / right_delta_t,
                                   lower_points_.back().t() + t),
                           STPoint(upper_points_.back().s() +
                                       t * upper_right_delta_s / right_delta_t,
                                   upper_points_.back().t() + t));
  point_pairs.back().second.set_s(
      std::max(point_pairs.back().second.s(),
               point_pairs.back().first.s() + kMinSEpsilon));

  return STBoundary(std::move(point_pairs));
}

size_t STBoundary::id() const { return id_; }

void STBoundary::set_id(const size_t id) { id_ = id; }

double STBoundary::characteristic_length() const {
  return characteristic_length_;
}

void STBoundary::SetCharacteristicLength(const double characteristic_length) {
  characteristic_length_ = characteristic_length;
}

double STBoundary::min_s() const { return min_s_; }
double STBoundary::min_t() const { return min_t_; }
double STBoundary::max_s() const { return max_s_; }
double STBoundary::max_t() const { return max_t_; }

bool STBoundary::GetIndexRange(const std::vector<STPoint>& points,
                               const double t, size_t* left,
                               size_t* right) const {
  assert(left);
  assert(right);
  if (t < points.front().t() || t > points.back().t()) {
    ILOG_DEBUG << "t is out of range. t =" << t;
    return false;
  }
  auto comp = [](const STPoint& p, const double t) { return p.t() < t; };
  auto first_ge = std::lower_bound(points.begin(), points.end(), t, comp);
  size_t index = std::distance(points.begin(), first_ge);
  if (index == 0) {
    *left = *right = 0;
  } else if (first_ge == points.end()) {
    *left = *right = points.size() - 1;
  } else {
    *left = index - 1;
    *right = index;
  }
  return true;
}

STBoundary STBoundary::CreateInstance(
    const std::vector<STPoint>& lower_points,
    const std::vector<STPoint>& upper_points) {
  if (lower_points.size() != upper_points.size() || lower_points.size() < 2) {
    return STBoundary();
  }

  std::vector<std::pair<STPoint, STPoint>> point_pairs;
  for (size_t i = 0; i < lower_points.size(); ++i) {
    point_pairs.emplace_back(
        STPoint(lower_points.at(i).s(), lower_points.at(i).t()),
        STPoint(upper_points.at(i).s(), upper_points.at(i).t()));
  }
  return STBoundary(point_pairs);
}

void STBoundary::set_upper_left_point(STPoint st_point) {
  upper_left_point_ = std::move(st_point);
}

void STBoundary::set_upper_right_point(STPoint st_point) {
  upper_right_point_ = std::move(st_point);
}

void STBoundary::set_bottom_left_point(STPoint st_point) {
  bottom_left_point_ = std::move(st_point);
}

void STBoundary::set_bottom_right_point(STPoint st_point) {
  bottom_right_point_ = std::move(st_point);
}

bool STBoundary::GetTimestampBeginWithS(double lower_s, double& t) const {
  if (IsEmpty()) {
    return false;
  }
  auto comp = [](const STPoint& p, const double s) { return p.s() < s; };
  auto first_ge = std::lower_bound(lower_points_.begin(), lower_points_.end(),
                                   lower_s, comp);
  size_t index = std::distance(lower_points_.begin(), first_ge);
  if (index == 0) {
    t = lower_points_[index].t();
    return true;
  } else if (first_ge == lower_points_.end()) {
    t = std::numeric_limits<double>::max();
    return true;
  } else {
    double left_s = lower_points_[index - 1].s();
    double left_t = lower_points_[index - 1].t();
    double right_s = lower_points_[index].s();
    double right_t = lower_points_[index].t();
    t = left_t + std::max(0.0, (lower_s - left_s) / (right_s - left_s)) *
                     (right_t - left_t);
    return true;
  }
  return false;
}

void STBoundary::DebugString() {
  ILOG_INFO << "s = " << min_s_ << ", " << max_s_ << ", t = " << min_t_ << ", "
            << max_t_;
  return;
}

}  // namespace planning
