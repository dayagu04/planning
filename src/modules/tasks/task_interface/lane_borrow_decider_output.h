#pragma once
#include <vector>
enum LaneBorrowStatus {
  kNoLaneBorrow = 0,
  kLaneBorrowDriving,
  kLaneBorrowBackOriginLane,
};

enum LaneBorrowFailedReason {
  NONE_FAILED_REASON = 0,
  SOLID_LINE_BORROW_DISABLED,   //实线不借道 1
  NO_PASSABLE_OBSTACLE,         //没有可通过的障碍物？ 2
  SELF_LANE_ENOUGH,             //自车车道足够车道内避让 3
  LANE_TYPE_CHECK_FAILED,       //车道类型不对，机动车非机动车? 4
  OBSERVE_TIME_CHECK_FAILED,    //观测时间？ 5
  BOUNDS_TOO_NARROW,            //空间不足
  STATIC_OBSTACLE_BLOCKED,      //(目标车道)有阻塞障碍物？
  BACKWARD_OBSTACLE_TOO_CLOSE,  //后方车太近
  LANE_CHANGE_STATE,            //借道
  CLOSE_TO_JUNCTION             //路口
};

struct LaneBorrowDeciderOutput {
  LaneBorrowFailedReason lane_borrow_failed_reason;
  int failed_obs_id;  // failed obs
  double target_l;    //目标l？
  double left_bounds_l;
  double right_bounds_l;
  int borrow_direction = 0;  // 0--None, 1--left, 2--right
  bool is_in_lane_borrow_status = false;
  std::vector<int> blocked_obs_id;  // block objs and failed？
};