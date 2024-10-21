#include "path_border_segment.h"

namespace planning {
namespace speed {

PathBorderSegment::PathBorderSegment(
    const int32_t index, const double back_center_s, const double start_s,
    const double end_s, const planning_math::LineSegment2d& left_border,
    const planning_math::LineSegment2d& right_border)
    : index_(index),
      start_s_(start_s),
      end_s_(end_s),
      back_center_s_(back_center_s),
      left_border_(left_border),
      right_border_(right_border) {}

int32_t PathBorderSegment::index() const { return index_; }

double PathBorderSegment::start_s() const { return start_s_; }

double PathBorderSegment::end_s() const { return end_s_; }

double PathBorderSegment::back_center_s() const { return back_center_s_; }

planning_math::LineSegment2d PathBorderSegment::left_border() const {
  return left_border_;
}

planning_math::LineSegment2d PathBorderSegment::right_border() const {
  return right_border_;
}

}  // namespace speed
}  // namespace planning