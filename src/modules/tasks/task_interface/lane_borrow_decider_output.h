#pragma once
#include <vector>
enum LaneBorrowStatus {
  kNoLaneBorrow = 0,
  kLaneBorrowDriving,
  kLaneBorrowPassSide,
  kLaneBorrowBackDriving,
};

enum LaneBorrowFailedReason {
  NONE_FAILED_REASON = 0,
  SOLID_LINE_BORROW_DISABLED,
  NO_PASSABLE_OBSTACLE,
  SELF_LANE_ENOUGH,
  LANE_TYPE_CHECK_FAILED,
  OBSERVE_TIME_CHECK_FAILED,
  BOUNDS_TOO_NARROW,
  STATIC_OBSTACLE_BLOCKED,
  BACKWARD_OBSTACLE_TOO_CLOSE
};

struct LaneBorrowDeciderOutput {
  LaneBorrowFailedReason lane_borrow_failed_reason;
  int failed_obs_id;
  double target_l;
  double left_bounds_l;
  double right_bounds_l;
  int borrow_direction;  // 0--None, 1--left, 2--right
  bool is_in_lane_borrow_status = false;
  std::vector<int> blocked_obs_id;
};