#pragma once

#include <cstdint>

#include "math/line_segment2d.h"
#include "utils/path_point.h"

namespace planning {
namespace speed {

class PathBorderSegment {
 public:
  PathBorderSegment(const int32_t index, const double back_center_s_,
                    const double start_s, const double end_s,
                    const planning_math::LineSegment2d& left_border,
                    const planning_math::LineSegment2d& right_border);

  ~PathBorderSegment(){};

  int32_t index() const;

  double start_s() const;

  double end_s() const;

  double back_center_s() const;

  planning_math::LineSegment2d left_border() const;

  planning_math::LineSegment2d right_border() const;

 private:
  int32_t index_ = -1;
  double start_s_ = 0.0;
  double end_s_ = 0.0;
  double back_center_s_ = 0.0;
  planning_math::LineSegment2d left_border_;
  planning_math::LineSegment2d right_border_;
};

}  // namespace speed
}  // namespace planning