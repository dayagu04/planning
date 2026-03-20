#include "sdmap_utils.h"

namespace ad_common {
namespace sdmap {
using ad_common::math::Vec2d;

void RoadSegInfo::Init() {
  segments_.clear();
  accumulated_s_.clear();
  headings_.clear();
  if (seg_.enu_points_size() > 0) {
    segments_.reserve(seg_.enu_points_size() - 1);
    accumulated_s_.reserve(seg_.enu_points_size());
    headings_.reserve(seg_.enu_points_size());
  }

  double s = 0.0;
  const auto &enu_points = seg_.enu_points();
  for (int i = 1; i < seg_.enu_points_size(); ++i) {
    accumulated_s_.push_back(s);
    segments_.emplace_back(Vec2d{enu_points[i - 1].x(), enu_points[i - 1].y()},
                           Vec2d{enu_points[i].x(), enu_points[i].y()});
    s += segments_.back().length();
  }
  if (!accumulated_s_.empty()) {
    accumulated_s_.push_back(s);
  }

  for (const auto &line_seg : segments_) {
    headings_.push_back(line_seg.unit_direction().Angle());
  }
}
}  // namespace sdmap
}  // namespace ad_common