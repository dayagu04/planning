#pragma once
#include <string>
#include <utility>
#include <vector>

#include "config/basic_type.h"
#include "define/geometry.h"

enum class RoadRightLevel {
  LOW_RIGHT = 0,
  HIGH_RIGHT,
  MID_RIGHT,
};

struct EgoLaneRoadRightDeciderOutput {
  bool is_merge_region = false;
  bool is_split_region = false;
  int merge_lane_virtual_id;
  int split_lane_virtual_id;
  planning::Point2D boundary_merge_point;
  bool cur_lane_is_continue = true;
  bool is_sharp_curve = false;
  bool boundary_merge_point_valid = false;
  double merge_point_distance = NL_NMAX;
  bool is_overlap_left = false;
  bool is_overlap_right = false;
  RoadRightLevel road_right_level = RoadRightLevel::HIGH_RIGHT;
};