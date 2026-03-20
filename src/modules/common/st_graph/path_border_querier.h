#pragma once

#include <memory>

#include "math/aaboxkdtree2d.h"
#include "math/geometry_object.h"
#include "math/line_segment2d.h"
#include "math/vec2d.h"
#include "path_border_segment.h"

namespace planning {
namespace speed {

class PathBorderQuerier {
 public:
  PathBorderQuerier(const std::vector<PathBorderSegment>& path_border_segments);

  ~PathBorderQuerier(){};

  bool GetObjects(const double min_s, const double max_s,
                  int32_t* const start_index, int32_t* const end_index) const;

  void QueryLowerBound(const double value, int32_t* const index) const;

  void QueryUpperBound(const double value, int32_t* const index) const;

  PathBorderSegment GetPathBorderSegmentByIndex(const int index) const;

 private:
  std::vector<PathBorderSegment> path_border_segments_;
};

}  // namespace speed
}  // namespace planning