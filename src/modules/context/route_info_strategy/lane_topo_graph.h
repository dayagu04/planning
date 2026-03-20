#pragma once
#include "hdmap/hdmap.h"
#include "ad_common/sdmap/sdmap.h"
#include "ad_common/sdpromap/sdpromap.h"
#include "map_data.pb.h"
#include "modules/context/virtual_lane.h"
namespace planning {
enum class TopoLaneTransitionType {
  CONTINUE = 0,  // 继续
  MERGE_LEFT = 1,
  MERGE_RIGHT = 2,
  SPLIT_LEFT = 3,
  SPLIT_RIGHT = 4
};

struct TopoLane {
  uint64 id;
  uint64 link_id;
  uint32 order_id;
  iflymapdata::sdpro::Direction direction;            // 行驶方向
  iflymapdata::sdpro::TurnDirection turn_direction;   // 转向方向
  iflymapdata::sdpro::LaneType lane_type;             // 车道类型
  float32 max_speed_limit;                            // 车道限速
  float32 min_speed_limit;                            // 车道限速
  float32 length;                                     // 车道长度
  std::unordered_set<uint64> successor_lane_ids;             // 后继车道ID
  std::unordered_set<uint64> predecessor_lane_ids;           // 前驱车道ID
  iflymapdata::sdpro::LaneBoundary left_boundary;     // 左侧边界类型
  iflymapdata::sdpro::LaneBoundary right_boundary;  // 右侧边界类型
  TopoLaneTransitionType transition_type;             // 车道转换类型
  double front_feasible_distance;
};

struct LaneTopoGroup {
  uint64 link_id;
  std::vector<TopoLane> topo_lanes;  // 车道拓扑信息
  int lane_nums = 0;                 // 车道数量
};

struct TopoLinkGraph {
  std::vector<LaneTopoGroup> lane_topo_groups;  // 车道拓扑组
};
}  // namespace planning
