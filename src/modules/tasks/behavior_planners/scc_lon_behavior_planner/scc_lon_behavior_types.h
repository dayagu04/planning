#pragma once

#include "task_basic_types.h"

namespace planning {
namespace scc {
enum BoundaryType {
  UNKNOWN,
  STOP,
  FOLLOW,
  YIELD,
  OVERTAKE,
  KEEP_CLEAR,
};

struct LonBound {
  double lower{std::numeric_limits<double>::min()};
  double upper{std::numeric_limits<double>::max()};
  double vel;
  double acc;
  double id;
};
using LonBounds = std::vector<LonBound>;

struct STBoundary {
  int id = 0;
  BoundaryType boundary_type;
  std::vector<LonBound> soft_bound;
  std::vector<LonBound> hard_bound;
};
using STboundaries = std::vector<STBoundary>;

}  // namespace scc
}  // namespace planning
