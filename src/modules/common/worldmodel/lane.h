#pragma once

#include <vector>
#include "common/common.h"

namespace planning {

enum LaneAvailableFlag {
  LANE_DATA = 1 << 0,
};

enum LaneBoundaryType {
  LANE_BOUNDARY_TYPE_UNKNOWN = 0,
  LANE_BOUNDARY_TYPE_SOLID = 1,
  LANE_BOUNDARY_TYPE_VIRTUAL = 2,
  LANE_BOUNDARY_TYPE_DASH = 3,
  LANE_BOUNDARY_TYPE_PHYSICAL = 4
};

enum LaneConnectDirection {
  LANE_CONNECT_DIRECTION_NORMAL = 0,
  LANE_CONNECT_DIRECTION_SPLIT_TO_LEFT = 1,
  LANE_CONNECT_DIRECTION_SPLIT_TO_RIGHT = 2,
  LANE_CONNECT_DIRECTION_MERGE_FROM_LEFT = 3,
  LANE_CONNECT_DIRECTION_MERGE_FROM_RIGHT = 4,
};

enum LaneType {
  LANE_TYPE_UNKNOWN = 0,
  LANE_TYPE_NORMAL = 1,
  LANE_TYPE_VIRTUAL = 2,
  LANE_TYPE_PARKING = 3,
  LANE_TYPE_ACCELERATE = 4,
  LANE_TYPE_DECELERATE = 5,
  LANE_TYPE_BUS = 6,
  LANE_TYPE_EMERGENCY = 7,
  LANE_TYPE_ACCELERATE_DECELERATE = 8,
  LANE_TYPE_LEFT_TURN_WAITTING_AREA = 9,
  LANE_TYPE_NON_MOTOR = 10,
  LANE_TYPE_RAMP = 11,
};

enum LaneBias{
  LANE_BIAS_NONE = 0,
  LANE_BIAS_LEFT = 1,
  LANE_BIAS_RIGHT = 2,
};

enum LaneSource {
  LANE_SOURCE_UNKNOWN = 0,
  LANE_SOURCE_MAP = 1,
  LANE_SOURCE_FUSION = 2,
};

struct LaneBoundaryPolyline {
  uint64_t track_id;
  std::vector<double> poly_coefficient;
  Interval available_interval;
};

struct LaneBoundarySegment {
  double length;
  LaneBoundaryType type;
};

struct LaneBoundary {
  bool existence;
  std::vector<LaneBoundarySegment> segments;
  LaneBoundaryPolyline polyline;
};

struct ReferenceLineSegment {
  uint16_t begin_index;
  uint16_t end_index;
  LaneConnectDirection direction;
  LaneType lane_type;
  bool is_in_intersection;
  bool is_in_route;
};

struct ReferenceLinePoint {
  Point3d car_point;
  Point3d enu_point;
  double curvature;
  double yaw;
  double distance_to_left_road_border;
  double distance_to_right_road_border;
  double distance_to_left_lane_border;
  double distance_to_right_lane_border;
  double distance_to_left_obstacle;
  double distance_to_right_obstacle;
  double lane_width;
  double max_velocity;
  std::string track_id;
  LaneBoundaryType left_road_border_type;
  LaneBoundaryType right_road_border_type;
};

struct ReferenceLine {
  bool available;
  std::vector<ReferenceLinePoint> reference_line_points;
  std::vector<ReferenceLineSegment> reference_line_segments;
};

enum MergeType {
  MERGE_TYPE_UNKNOWN = 0,
  MERGE_TYPE_UNRELATED = 1,
  MERGE_TYPE_MERGE_FROM_RIGHT = 2,
  MERGE_TYPE_MERGE_FROM_LEFT = 3,
  MERGE_TYPE_MERGE_FROM_BOTH = 4,
  MERGE_TYPE_MERGE_LANE_END = 5,
};

struct MergePoint {
  double distance;
  MergeType merge_type;
};


struct LaneData {
  int32_t relative_id;

  Direction lane_marks;
  LaneType lane_type;

  LaneBoundary left_lane_boundary;
  LaneBoundary right_lane_boundary;

  ReferenceLine reference_line;

  MergePoint merge_point;
  MergePoint Y_point;
  LaneBias lane_bias;
};

struct Lane {
  size_t available;
  std::vector<LaneData> lanes_data;
  LaneSource lane_source;
};
} // namespace planning
