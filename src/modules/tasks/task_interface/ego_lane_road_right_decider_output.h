#pragma once
#include <string>
#include <utility>
#include <vector>

#include "config/basic_type.h"
#include "define/geometry.h"

struct EgoLaneRoadRightDeciderOutput {
  bool is_merge_region = false;
  bool is_split_region = false;
  int merge_lane_virtual_id;
  int split_lane_virtual_id;
  planning::Point2D boundary_merge_point;
  bool cur_lane_is_continue = true;
  bool boundary_merge_point_valid = false;
};