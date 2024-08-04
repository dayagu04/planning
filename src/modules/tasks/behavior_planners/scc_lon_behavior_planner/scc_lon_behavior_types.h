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

struct NarrowLead {
  int id;
  double min_s;
  double desire_distance;
  double safe_distance;
  double v_limit;
  bool is_collison;
};

}  // namespace scc
}  // namespace planning
