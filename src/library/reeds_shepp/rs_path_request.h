#pragma once

#include "pose2d.h"
#include <string>

namespace planning {

enum class RSPathRequestType {
  none,
  all_path_forbid_reverse,
  all_path_forbid_forward,
  first_path_forbid_reverse,
  first_path_forbid_forward,
  last_path_forbid_forward,
  last_path_forbid_reverse,
  forbid_gear_change,
  // gear switch < 2,
  gear_switch_less_than_twice,
  max_num
};

#define RS_REQUST_MAX_NUM (8)
struct RSPathRequest {
  size_t request_size;
  RSPathRequestType request[RS_REQUST_MAX_NUM];

  // all path segment must bigger than segment_min_length.
  bool enable_segment_length_check;
  double segment_min_length;

  // first path must bigger than first_segment_min_length;
  bool enable_first_segment_len_check;
  double first_segment_min_length;

  Pose2D start;
  Pose2D end;

  double min_radius;
};

std::string GetRSRequestType(const RSPathRequestType type);

}  // namespace planning