#include "path_border_querier.h"

#include <algorithm>
#include <iostream>

#include "path_border_segment.h"

namespace planning {
namespace speed {

using namespace planning_math;

PathBorderQuerier::PathBorderQuerier(
    const std::vector<PathBorderSegment>& path_border_segments)
    : path_border_segments_(path_border_segments) {}

bool PathBorderQuerier::GetObjects(const double min_s, const double max_s,
                                   int32_t* const start_index,
                                   int32_t* const end_index) const {
  if (path_border_segments_.empty()) {
    return false;
  }
  QueryLowerBound(min_s, start_index);

  QueryUpperBound(max_s, end_index);

  return *start_index > -1 && *end_index >= *start_index;
}

void PathBorderQuerier::QueryLowerBound(const double value,
                                        int32_t* const index) const {
  auto comp = [](const PathBorderSegment& segment, const double value) {
    return segment.back_center_s() < value;
  };
  auto it = std::lower_bound(path_border_segments_.begin(),
                             path_border_segments_.end(), value, comp);
  if (it == path_border_segments_.end()) {
    *index = -1;
  } else {
    *index = it - path_border_segments_.begin();
  }
}

void PathBorderQuerier::QueryUpperBound(const double value,
                                        int32_t* const index) const {
  auto comp = [](const double value, const PathBorderSegment& segment) {
    return segment.back_center_s() > value;
  };
  auto it = std::upper_bound(path_border_segments_.begin(),
                             path_border_segments_.end(), value, comp);
  if (it == path_border_segments_.end()) {
    *index = -1;
  } else {
    *index = it - path_border_segments_.begin();
  }
}

PathBorderSegment PathBorderQuerier::GetPathBorderSegmentByIndex(
    const int index) const {
  return path_border_segments_[index];
}

}  // namespace speed
}  // namespace planning