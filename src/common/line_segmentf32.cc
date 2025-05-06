#include "line_segmentf32.h"

#include <algorithm>
#include <cmath>
#include <utility>

#include "assert.h"
#include "vecf32.h"

namespace planning {

bool IsWithin(float val, float bound1, float bound2) {
  if (bound1 > bound2) {
    std::swap(bound1, bound2);
  }
  return val >= bound1 - kMathEpsilon32 && val <= bound2 + kMathEpsilon32;
}


LineSegmentf32::LineSegmentf32() {
  unit_direction_ = Vec2df32(1, 0);
  InitMaxMin();
}

LineSegmentf32::LineSegmentf32(const Vec2df32 &start, const Vec2df32 &end)
    : start_(start), end_(end) {
  const float dx = end_.x() - start_.x();
  const float dy = end_.y() - start_.y();
  length_ = hypot(dx, dy);
  unit_direction_ =
      (length_ <= kMathEpsilon32 ? Vec2df32(0, 0)
                               : Vec2df32(dx / length_, dy / length_));
  heading_ = unit_direction_.Angle();

  line_a_ = (end.y() - start.y());
  line_b_ = (start.x() - end.x());
  line_c_ = (end.x() * start.y() - start.x() * end.y());

  unit_a_ = line_a_;
  unit_b_ = line_b_;
  unit_c_ = line_c_;
  if (length_ > 1e-5) {
    unit_a_ /= length_;
    unit_b_ /= length_;
    unit_c_ /= length_;
  }

  InitMaxMin();
}

float LineSegmentf32::length() const { return length_; }

float LineSegmentf32::length_sqr() const { return length_ * length_; }

float LineSegmentf32::RawDistanceTo(const Vec2df32 &point) const {
  if (length_ <= kMathEpsilon32) {
    return point.DistanceTo(start_);
  }
  const float x0 = point.x() - start_.x();
  const float y0 = point.y() - start_.y();

  return x0 * unit_direction_.y() - y0 * unit_direction_.x();
}

float LineSegmentf32::DistanceTo(const Vec2df32 &point) const {
  if (length_ <= kMathEpsilon32) {
    return point.DistanceTo(start_);
  }
  const float x0 = point.x() - start_.x();
  const float y0 = point.y() - start_.y();
  const float proj = x0 * unit_direction_.x() + y0 * unit_direction_.y();
  if (proj <= 0.0) {
    return hypot(x0, y0);
  }
  if (proj >= length_) {
    return point.DistanceTo(end_);
  }
  return std::abs(x0 * unit_direction_.y() - y0 * unit_direction_.x());
}

float LineSegmentf32::DistanceTo(const Vec2df32 &point,
                                 Vec2df32 *const nearest_pt) const {
  assert(nearest_pt != nullptr);
  if (length_ <= kMathEpsilon32) {
    *nearest_pt = start_;
    return point.DistanceTo(start_);
  }
  const float x0 = point.x() - start_.x();
  const float y0 = point.y() - start_.y();
  const float proj = x0 * unit_direction_.x() + y0 * unit_direction_.y();
  if (proj < 0.0) {
    *nearest_pt = start_;
    return hypot(x0, y0);
  }
  if (proj > length_) {
    *nearest_pt = end_;
    return point.DistanceTo(end_);
  }
  *nearest_pt = start_ + unit_direction_ * proj;
  return std::abs(x0 * unit_direction_.y() - y0 * unit_direction_.x());
}

float LineSegmentf32::DistanceSquareTo(const Vec2df32 &point) const {
  if (length_ <= kMathEpsilon32) {
    return point.DistanceSquareTo(start_);
  }
  const float x0 = point.x() - start_.x();
  const float y0 = point.y() - start_.y();
  const float proj = x0 * unit_direction_.x() + y0 * unit_direction_.y();
  if (proj <= 0.0) {
    return x0 * x0 + y0 * y0;
  }
  if (proj >= length_) {
    return point.DistanceSquareTo(end_);
  }
  return std::pow(x0 * unit_direction_.y() - y0 * unit_direction_.x(), 2);
}

float LineSegmentf32::DistanceSquareTo(const Vec2df32 &point,
                                       Vec2df32 *const nearest_pt) const {
  assert(nearest_pt != nullptr);
  if (length_ <= kMathEpsilon32) {
    *nearest_pt = start_;
    return point.DistanceSquareTo(start_);
  }
  const float x0 = point.x() - start_.x();
  const float y0 = point.y() - start_.y();
  const float proj = x0 * unit_direction_.x() + y0 * unit_direction_.y();
  if (proj <= 0.0) {
    *nearest_pt = start_;
    return x0 * x0 + y0 * y0;
  }
  if (proj >= length_) {
    *nearest_pt = end_;
    return point.DistanceSquareTo(end_);
  }
  *nearest_pt = start_ + unit_direction_ * proj;
  return std::pow(x0 * unit_direction_.y() - y0 * unit_direction_.x(), 2);
}

bool LineSegmentf32::IsPointIn(const Vec2df32 &point) const {
  if (length_ <= kMathEpsilon32) {
    return std::abs(point.x() - start_.x()) <= kMathEpsilon32 &&
           std::abs(point.y() - start_.y()) <= kMathEpsilon32;
  }
  const float prod = CrossProdf32(point, start_, end_);
  if (std::abs(prod) > kMathEpsilon32) {
    return false;
  }
  return IsWithin(point.x(), start_.x(), end_.x()) &&
         IsWithin(point.y(), start_.y(), end_.y());
}

float LineSegmentf32::ProjectOntoUnit(const Vec2df32 &point) const {
  return unit_direction_.InnerProd(point - start_);
}

float LineSegmentf32::ProductOntoUnit(const Vec2df32 &point) const {
  return unit_direction_.CrossProd(point - start_);
}

bool LineSegmentf32::HasIntersect(const LineSegmentf32 &other_segment) const {
  Vec2df32 point;
  return GetIntersect(other_segment, &point);
}

bool LineSegmentf32::GetIntersect(const LineSegmentf32 &other_segment,
                                 Vec2df32 *const point) const {
  assert(point != nullptr);
  if (IsPointIn(other_segment.start())) {
    *point = other_segment.start();
    return true;
  }
  if (IsPointIn(other_segment.end())) {
    *point = other_segment.end();
    return true;
  }
  if (other_segment.IsPointIn(start_)) {
    *point = start_;
    return true;
  }
  if (other_segment.IsPointIn(end_)) {
    *point = end_;
    return true;
  }
  if (length_ <= kMathEpsilon32 || other_segment.length() <= kMathEpsilon32) {
    return false;
  }
  const float cc1 = CrossProdf32(start_, end_, other_segment.start());
  const float cc2 = CrossProdf32(start_, end_, other_segment.end());
  if (cc1 * cc2 >= -kMathEpsilon32) {
    return false;
  }
  const float cc3 =
      CrossProdf32(other_segment.start(), other_segment.end(), start_);
  const float cc4 =
      CrossProdf32(other_segment.start(), other_segment.end(), end_);
  if (cc3 * cc4 >= -kMathEpsilon32) {
    return false;
  }
  const float ratio = cc4 / (cc4 - cc3);
  *point = Vec2df32(start_.x() * ratio + end_.x() * (1.0 - ratio),
                 start_.y() * ratio + end_.y() * (1.0 - ratio));
  return true;
}

// return distance with perpendicular foot point.
float LineSegmentf32::GetPerpendicularFoot(const Vec2df32 &point,
                                           Vec2df32 *const foot_point) const {
  assert(foot_point != nullptr);
  if (length_ <= kMathEpsilon32) {
    *foot_point = start_;
    return point.DistanceTo(start_);
  }
  const float x0 = point.x() - start_.x();
  const float y0 = point.y() - start_.y();
  const float proj = x0 * unit_direction_.x() + y0 * unit_direction_.y();
  *foot_point = start_ + unit_direction_ * proj;
  return std::abs(x0 * unit_direction_.y() - y0 * unit_direction_.x());
}

void LineSegmentf32::InitMaxMin() {
  min_x_ = std::min(start_.x(), end_.x());
  max_x_ = std::max(start_.x(), end_.x());
  min_y_ = std::min(start_.y(), end_.y());
  max_y_ = std::max(start_.y(), end_.y());
}

Vec2df32 LineSegmentf32::GetPoint(float s) const {
  float ratio = s / length_;
  return start_ * (1 - ratio) + end_ * ratio;
}
}
// namespace planning
