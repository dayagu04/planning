#pragma once

#include <map>
#include <vector>

#include "common.h"
#include "common_c.h"
#include "fusion_road_c.h"
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

/******************************************************
All merges can be abstracted as following lane topology:

  *
 * *
*   *
*   *
*   *

*******************************************************/
struct MergeSplitPoints {
  enum MergeSplitOrientation {
    UNKNOWN = 0,
    LEFT = 1,
    RIGHT = 2,
  };
  enum PointType {
    MERGE = 0,
    SPLIT = 1,
  };
  struct MergeSplitPointInfo {
    double merge_split_point_distance_to_ego_rear_axle{0.0};
    PointType is_split_or_merge{SPLIT};  // 0: merge, 1: split(from perception)
    MergeSplitOrientation split_merge_orientation{
        UNKNOWN};  // 1:left, 2:right(from perception)
    iflyauto::Point3d split_merge_point{0.0, 0.0, 0.0};
  };

  void Reset() {
    merge_split_existence = false;
    merge_split_points_size = 0;
    merge_split_points_in_dist_order.clear();
    closet_merge_split_point = MergeSplitPointInfo();
  }

  bool merge_split_existence{false};
  size_t merge_split_points_size{0};
  std::map<double, MergeSplitPointInfo> merge_split_points_in_dist_order;
  MergeSplitPointInfo closet_merge_split_point;
};

struct MergeAgentsInfo {
  enum class AgentOrientationToEgo {
    UNKNOWN = 0,
    LEFT_FRONT = 1,
    RIGHT_FRONT = 2,
    LEFT = 3,
    RIGHT = 4,
    LEFT_REAR = 5,
    RIGHT_REAR = 6
  };
  enum class MergeTargetName {
    UNKNOWN = 0,
    MERGE_TARGET_ONE = 1,
    MERGE_TARGET_TWO = 2
  };
};

}  // namespace scc
}  // namespace planning
