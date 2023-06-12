#pragma once

// 等common proto出来，
// #include "common.pb.h"
#include "common.h"
#include "worldmodel/lane.h"

namespace planning {

enum PerceptionFusionLanesAvailableFlag {
  PERCEPTION_FUSION_LANES_DATA = 1 << 0,
};

struct PerceptionFusionLaneData {
  std::vector<double> polycoeff;
  double head;
  double tail;
  int8_t relative_id;
  uint64_t track_id;

  std::vector<LaneBoundarySegment> lane_boundary_segments;

  std::string reserved_info;
};

struct PerceptionFusionLanes {
  size_t available;
  std::vector<PerceptionFusionLaneData> perception_fusion_lanes_data;
};

} // namespace planning
