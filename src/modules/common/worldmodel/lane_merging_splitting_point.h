#pragma once

#include "common.h"

namespace planning {

enum MSDLaneMergingSplittingPointAvailableFlag {
  MSD_LANE_MERGING_SPLITTING_POINT_LANE_MERGING_SPLITTING_POINTS_DATA = 1 << 0,
};

// enum MSDLaneMergingSplittingPointType {
//   MSD_LANE_MERGING_SPLITTING_POINT_TYPE_UNKNOWN = 0,
//   MSD_LANE_MERGING_SPLITTING_POINT_TYPE_UNRELATED = 1,
//   MSD_LANE_MERGING_SPLITTING_POINT_TYPE_MERGE_FROM_RIGHT = 2,
//   MSD_LANE_MERGING_SPLITTING_POINT_TYPE_MERGE_FROM_LEFT = 3,
//   MSD_LANE_MERGING_SPLITTING_POINT_TYPE_MERGE_FROM_BOTH = 4,
//   MSD_LANE_MERGING_SPLITTING_POINT_TYPE_MERGE_TO_RIGHT = 5,
//   MSD_LANE_MERGING_SPLITTING_POINT_TYPE_MERGE_TO_LEFT = 6,
//   MSD_LANE_MERGING_SPLITTING_POINT_TYPE_MERGE_TO_BOTH = 7,
//   MSD_LANE_MERGING_SPLITTING_POINT_TYPE_SPLIT_TO_LEFT = 8,
//   MSD_LANE_MERGING_SPLITTING_POINT_TYPE_SPLIT_TO_RIGHT = 9,
// };

enum MSDLaneOrientation {
  MSD_LANE_MERGING_SPLITTING_POINT_TYPE_UNKNOWN = 0,
  MSD_LANE_MERGING_SPLITTING_POINT_TYPE_LEFT = 1,
  MSD_LANE_MERGING_SPLITTING_POINT_TYPE_RIGHT = 2,
  MSD_LANE_MERGING_SPLITTING_POINT_TYPE_INTERSECT = 3,
  MSD_LANE_MERGING_SPLITTING_POINT_TYPE_IGNORED = 4,
};

struct MSDLaneMergingSplittingPointData {
  double distance;
  bool is_split{false};
  bool is_continue{true};
  MSDLaneOrientation orientation;
  double length;
  // MSDLaneMergingSplittingPointType type;
};

struct MSDLaneMergingSplittingPoint {
  size_t available;
  std::vector<MSDLaneMergingSplittingPointData>
      lane_merging_splitting_points_data;
};
} // namespace planning
