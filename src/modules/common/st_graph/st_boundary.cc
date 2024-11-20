#include "st_boundary.h"

#include <algorithm>
#include <utility>

#include "st_point.h"
#include "vec2d.h"

namespace planning {
namespace speed {

namespace {
constexpr double kTimeResolution = 0.1;
};

STBoundary::STBoundary(
    const std::vector<std::pair<STPoint, STPoint>>& st_point_pairs) {
  // if (IsValid(st_point_pairs)) {
  //   return;
  // }

  lower_points_.reserve(st_point_pairs.size());
  upper_points_.reserve(st_point_pairs.size());
  for (const auto& st_point_pair : st_point_pairs) {
    lower_points_.emplace_back(st_point_pair.first);
    upper_points_.emplace_back(st_point_pair.second);
    max_delta_s_ = std::fmax(
        max_delta_s_, st_point_pair.second.s() - st_point_pair.first.s());
  }
  min_t_ = lower_points_.front().t();
  max_t_ = lower_points_.back().t();
}

STBoundary STBoundary::CreateInstance(
    const std::vector<STPoint>& lower_points,
    const std::vector<STPoint>& upper_points) {
  if (lower_points.size() != upper_points.size() || lower_points.size() < 2) {
    return STBoundary();
  }

  std::vector<std::pair<STPoint, STPoint>> st_point_pairs;
  st_point_pairs.reserve(upper_points.size());
  for (size_t i = 0; i < lower_points.size(); ++i) {
    st_point_pairs.emplace_back(
        STPoint(lower_points.at(i).s(), lower_points.at(i).t()),
        STPoint(upper_points.at(i).s(), upper_points.at(i).t()));
  }
  return STBoundary(st_point_pairs);
}

bool STBoundary::IsEmpty() const { return lower_points_.empty(); }

bool STBoundary::GetBoundarySRange(const double curr_time,
                                   double* const s_lower,
                                   double* const s_upper) const {
  STPoint lower_point;
  STPoint upper_point;
  if (!GetBoundaryBounds(curr_time, &lower_point, &upper_point)) {
    return false;
  }
  *s_lower = lower_point.s();
  *s_upper = upper_point.s();
  return true;
}

bool STBoundary::GetBoundaryBounds(const double curr_time,
                                   STPoint* const lower_point,
                                   STPoint* const upper_point) const {
  size_t left = 0;
  size_t right = 0;
  if (!GetIndexRange(curr_time, &left, &right)) {
    return false;
  }
  const double r =
      (left == right
           ? 0.0
           : (curr_time - upper_points_[left].t()) /
                 (upper_points_[right].t() - upper_points_[left].t()));
  upper_point->set_s(upper_points_[left].s() +
                     r * (upper_points_[right].s() - upper_points_[left].s()));
  upper_point->set_t(curr_time);
  upper_point->set_acceleration(upper_points_[left].acceleration() +
                                r * (upper_points_[right].acceleration() -
                                     upper_points_[left].acceleration()));
  upper_point->set_velocity(
      upper_points_[left].velocity() +
      r * (upper_points_[right].velocity() - upper_points_[left].velocity()));
  upper_point->set_agent_id(id_ >> 8);
  upper_point->set_boundary_id(id_);
  upper_point->set_valid(true);

  lower_point->set_s(lower_points_[left].s() +
                     r * (lower_points_[right].s() - lower_points_[left].s()));
  lower_point->set_t(curr_time);
  lower_point->set_acceleration(lower_points_[left].acceleration() +
                                r * (lower_points_[right].acceleration() -
                                     lower_points_[left].acceleration()));
  lower_point->set_velocity(
      lower_points_[left].velocity() +
      r * (lower_points_[right].velocity() - lower_points_[left].velocity()));
  lower_point->set_agent_id(id_ >> 8);
  lower_point->set_boundary_id(id_);
  lower_point->set_valid(true);
  return true;
}

STBoundary::DecisionType STBoundary::decision_type() const {
  return decision_type_;
}

int64_t STBoundary::id() const { return id_; }

void STBoundary::set_id(const int64_t id) { id_ = id; }

void STBoundary::set_decision_type(const DecisionType& decision_type) {
  decision_type_ = decision_type;
}

double STBoundary::min_s() const { return min_s_; }

double STBoundary::min_t() const { return min_t_; }

double STBoundary::max_s() const { return max_s_; }

double STBoundary::max_t() const { return max_t_; }

const std::vector<STPoint>& STBoundary::upper_points() const {
  return upper_points_;
}

const std::vector<STPoint>& STBoundary::lower_points() const {
  return lower_points_;
}

void STBoundary::clear() {
  // DecisionType boundary_type_ = DecisionType::UNKNOWN;

  upper_points_.clear();
  lower_points_.clear();

  id_ = -1;
  characteristic_length_ = 1.0;
  min_s_ = std::numeric_limits<double>::max();
  max_s_ = std::numeric_limits<double>::lowest();
  min_t_ = std::numeric_limits<double>::max();
  max_t_ = std::numeric_limits<double>::lowest();
}

bool STBoundary::IsPointInBoundary(const STPoint& st_point) const {
  double s_upper = 0.0;
  double s_lower = 0.0;
  if (GetBoundarySRange(st_point.t(), &s_lower, &s_upper)) {
    return st_point.s() > s_lower - planning_math::kMathEpsilon &&
           st_point.s() < s_upper + planning_math::kMathEpsilon;
  }
  return false;
}

STBoundary STBoundary::ExpandByDs(const double ds) const {
  if (lower_points_.empty()) {
    return STBoundary();
  }
  std::vector<std::pair<STPoint, STPoint>> point_pairs;
  point_pairs.reserve((lower_points_.size()));
  for (size_t i = 0; i < lower_points_.size(); ++i) {
    point_pairs.emplace_back(
        STPoint(lower_points_[i].s() - ds, lower_points_[i].t()),
        STPoint(upper_points_[i].s() + ds, upper_points_[i].t()));
  }
  return STBoundary(std::move(point_pairs));
}

STBoundary STBoundary::ExpandByDt(const double dt) const {
  if (lower_points_.empty()) {
    return STBoundary();
  }

  std::vector<std::pair<STPoint, STPoint>> point_pairs;
  point_pairs.reserve(lower_points_.size() + 2);

  const double left_delta_t = lower_points_[1].t() - lower_points_[0].t();
  const double lower_left_delta_s = lower_points_[1].s() - lower_points_[0].s();
  const double upper_left_delta_s = upper_points_[1].s() - upper_points_[0].s();

  point_pairs.emplace_back(
      STPoint(lower_points_[0].s() - dt * lower_left_delta_s / left_delta_t,
              lower_points_[0].t() - dt),
      STPoint(upper_points_[0].s() - dt * upper_left_delta_s / left_delta_t,
              upper_points_[0].t() - dt));

  const double kMinSEpsilon = 1e-3;
  point_pairs.front().first.set_s(
      std::fmin(point_pairs.front().second.s() - kMinSEpsilon,
                point_pairs.front().first.s()));

  for (size_t i = 0; i < lower_points_.size(); ++i) {
    point_pairs.emplace_back(lower_points_[i], upper_points_[i]);
  }

  size_t length = lower_points_.size();

  const double right_delta_t =
      lower_points_[length - 1].t() - lower_points_[length - 2].t();
  const double lower_right_delta_s =
      lower_points_[length - 1].s() - lower_points_[length - 2].s();
  const double upper_right_delta_s =
      upper_points_[length - 1].s() - upper_points_[length - 2].s();

  point_pairs.emplace_back(STPoint(lower_points_.back().s() +
                                       dt * lower_right_delta_s / right_delta_t,
                                   lower_points_.back().t() + dt),
                           STPoint(upper_points_.back().s() +
                                       dt * upper_right_delta_s / right_delta_t,
                                   upper_points_.back().t() + dt));
  point_pairs.back().second.set_s(
      std::fmax(point_pairs.back().second.s(),
                point_pairs.back().first.s() + kMinSEpsilon));

  return STBoundary(std::move(point_pairs));
}

bool STBoundary::CutOffByTimeRange(const double start_time,
                                   const double end_time) {
  const int32_t start_index =
      int32_t((start_time - lower_points_.front().t()) / kTimeResolution);
  const int32_t end_index =
      int32_t((end_time - lower_points_.front().t()) / kTimeResolution);
  for (int32_t i = start_index; i <= end_index; ++i) {
    lower_points_[i].set_valid(false);
    upper_points_[i].set_valid(false);
  }
  invalid_time_sections_.emplace_back(start_time, end_time);
  return true;
}

const STPoint& STBoundary::upper_left_point() const {
  return upper_left_point_;
}

const STPoint& STBoundary::upper_right_point() const {
  return upper_right_point_;
}

const STPoint& STBoundary::bottom_left_point() const {
  return bottom_left_point_;
}

const STPoint& STBoundary::bottom_right_point() const {
  return bottom_right_point_;
}

const std::vector<std::pair<double, double>>&
STBoundary::invalid_time_sections() const {
  return invalid_time_sections_;
}

void STBoundary::set_upper_left_point(const STPoint& st_point) {
  upper_left_point_ = st_point;
}

void STBoundary::set_upper_right_point(const STPoint& st_point) {
  upper_right_point_ = st_point;
}

void STBoundary::set_bottom_left_point(const STPoint& st_point) {
  bottom_left_point_ = st_point;
}

void STBoundary::set_bottom_right_point(const STPoint& st_point) {
  bottom_right_point_ = st_point;
}

void STBoundary::InsertInvalidTimeSection(
    const std::pair<double, double>& invalid_time_section) {
  invalid_time_sections_.emplace_back(invalid_time_section);
}

bool STBoundary::IsValid(
    const std::vector<std::pair<STPoint, STPoint>>& point_pairs) const {
  if (point_pairs.size() < 2) {
    return false;
  }

  static constexpr double kStBoundaryEpsilon = 1e-9;
  static constexpr double kMinDeltaT = 1e-6;
  for (size_t i = 0; i < point_pairs.size(); ++i) {
    const auto& curr_lower = point_pairs[i].first;
    const auto& curr_upper = point_pairs[i].second;
    if (curr_upper.s() < curr_lower.s()) {
      return false;
    }
    if (std::fabs(curr_lower.t() - curr_upper.t()) > kStBoundaryEpsilon) {
      return false;
    }
    if (i + 1 != point_pairs.size()) {
      const auto& next_lower = point_pairs[i + 1].first;
      const auto& next_upper = point_pairs[i + 1].second;
      if (std::fmax(curr_lower.t(), curr_upper.t()) + kMinDeltaT >=
          std::fmin(next_lower.t(), next_upper.t())) {
        return false;
      }
    }
  }
  return true;
}

bool STBoundary::IsPointNearToLinesegment(
    const planning_math::LineSegment2d& seg, const planning_math::Vec2d& point,
    const double max_dist) {
  return seg.DistanceSquareTo(point) <
         max_dist * max_dist + planning_math::kMathEpsilon;
}

bool STBoundary::GetIndexRange(const double t, size_t* left,
                               size_t* right) const {
  if (IsTimeInvalid(t)) {
    return false;
  }

  if (t < upper_points_.front().t() || t > upper_points_.back().t()) {
    return false;
  }

  auto comp = [](const STPoint& p, const double t) { return p.t() < t; };
  auto first_ge =
      std::lower_bound(upper_points_.begin(), upper_points_.end(), t, comp);
  size_t index = std::distance(upper_points_.begin(), first_ge);
  if (index == 0) {
    *left = *right = 0;
  } else if (first_ge == upper_points_.end()) {
    *left = *right = upper_points_.size() - 1;
  } else {
    *left = index - 1;
    *right = index;
  }
  return true;
}

bool STBoundary::IsTimeInvalid(const double t) const {
  for (const auto& ele : invalid_time_sections_) {
    if (t > ele.first - planning_math::kMathEpsilon &&
        t < ele.second + planning_math::kMathEpsilon) {
      return true;
    }
  }
  return false;
}

double STBoundary::max_delta_s() const { return max_delta_s_; }

}  // namespace speed
}  // namespace planning