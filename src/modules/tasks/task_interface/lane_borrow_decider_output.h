#pragma once
#include <vector>

#include "config/basic_type.h"
#include "lane_borrow_decider.pb.h"
#include "utils/kd_path.h"

namespace planning {

enum LaneBorrowStatus {
  kNoLaneBorrow = 0,
  kLaneBorrowDriving,
  kLaneBorrowCrossing,
  kLaneBorrowBackOriginLane,
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
  BACKWARD_OBSTACLE_TOO_CLOSE,
  LANE_CHANGE_STATE,
  CLOSE_TO_JUNCTION,
  NEARBY_OBSTACLE_TOO_CLOSE,
  STATIC_AREA_TOO_CLOSE,
  CENTER_OBSTACLE,
  TRIGGER_BUT_DP_SEARCH_FAILED,
  CURRENT_LANE_LOSS,
  DP_NO_DIRECTION,
  BORROWDIRECTION_DIFFERENT,
  TMP_DP_SEARCH_FAILED,
  CUTINOUT_RISK,
  AGENT_MGR_FAILED,
  CHANGE_TARGET_LANE,
  FRONT_OBS_NOT_BORROWING,
  NOT_DBW_STATUS,
  SPEED_TOO_HIGH
};
enum BorrowDirection { NO_BORROW = 0, LEFT_BORROW, RIGHT_BORROW };

struct LaneBorrowDeciderOutput {
  LaneBorrowFailedReason lane_borrow_failed_reason = NONE_FAILED_REASON;
  int failed_obs_id;  // failed obs
  double target_l;
  double left_bounds_l;
  double right_bounds_l;
  BorrowDirection borrow_direction = NO_BORROW;  // 0--None, 1--left, 2--right
  bool is_in_lane_borrow_status = false;
  bool is_change_target_lane = false;
  std::vector<int> blocked_obs_id;  // block objs and failed
  bool can_left_borrow = false;
  bool can_right_borrow = false;
  CarReferenceInfo dp_path_ref;
  LaneBorrowStatus lane_borrow_state;
  std::shared_ptr<planning_math::KDPath> dp_path_coord;
};

}  // namespace planning