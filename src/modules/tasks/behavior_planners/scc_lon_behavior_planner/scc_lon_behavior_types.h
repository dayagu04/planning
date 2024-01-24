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

struct STBoundary {
  int id = 0;
  BoundaryType boundary_type;
  std::vector<Bound> soft_bound;
  std::vector<Bound> hard_bound;
};
using STboundaries = std::vector<STBoundary>;

}  // namespace scc
}  // namespace planning
