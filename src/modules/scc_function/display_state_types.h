#pragma once

namespace planning {
/**
 * @brief confirm mode
 */
typedef enum { CLOSE, OPEN } ConfirmMode;

/**
 * @brief confirm signal
 */
typedef enum { EMPTY, APPROVED, REFUSED } ConfirmSignal;

/**
 * @brief confirm mode change type
 */
typedef enum { MUTE, MODE, SIGNAL } ConfirmModeChangeType;

/**
 * @brief status given to mff for display differ from inner scenario status
 */
enum DisplayStatus { WAIT, PROCESS, RETURN, CANCEL, FINISH, READY, OFF };

/**
 * @brief lane change condition given to mff for display
 */
enum LaneChangeCondition {
  DEFAULT_CONDITION,
  LEFT_CHANGE_SATISFIED,
  LEFT_CHANGE_UNSATISFIED,
  RIGHT_CHANGE_SATISFIED,
  RIGHT_CHANGE_UNSATISFIED
};

/**
 * @brief reasons why int request canceled
 */
enum IntCancelReasonType {
  NO_CANCEL,
  SOLID_LC,
  MANUAL_CANCEL,
  TIMEOUT_LC,
  UNSUITABLE_VEL,
  ENV_ERROR,
  LOW_CHANGE_LOGIT
};

/**
 * @brief reasons why going lane change
 */
enum TriggerReasonType {
  UNCERTAIN = 0,
  INTO_RAMP = 1,
  OFF_RAMP = 2,
  GETIN_FASTER_LANE = 3,
  AVOID_TRUCK = 4,
  AVOID_SPLIT_MERGE = 5,
  NO_TRIGGER_REASON = 32,
};

/**
 * @brief display state with time
 */
struct DisplayStateWithTime {
  DisplayStatus state;
  double start_time;
  double end_time;
};

/**
 * @brief display state config
 */
struct DisplayStateConfig {
  static constexpr int ready_remain_time{2};
  static constexpr int wait_remain_time{100};
  static constexpr int finish_remain_time{3};
  static constexpr double allow_int_vel_limit{0};
  static constexpr double into_ramp_threshold{2000.0};
  static constexpr double close_to_split_merge_threshold{100.0};
  static constexpr double off_ramp_threshold{100.0};
  static constexpr double avoid_truck_time_distance_threshold{1.5};
  static constexpr int map_int_cancel_freeze_cnt{50};
  static constexpr int model_int_cancel_freeze_cnt{100};
  static constexpr int map_confirm_cancel_freeze_cnt{50};
  static constexpr int model_confirm_cancel_freeze_cnt{100};
  static constexpr int DefaultCancelFreezeCnt{100};
  static constexpr double kMaxDistanceValue{1e6};
  static constexpr double kTlaneFasterBuffer{5.0};
};

/**
 * @brief confirm mode status
 */
struct ConfirmModeStatus {
  ConfirmMode confirm_mode;
  ConfirmSignal confirm_signal;
};

/**
 * @brief lane change freeze for serval seconds because driver cancel it
 */
struct LaneChangeFreezeCnt {
  int map_int_cancel_freeze_cnt;
  int model_int_cancel_freeze_cnt;
  int map_confirm_cancel_freeze_cnt;
  int model_confirm_cancel_freeze_cnt;
};

/**
 * @brief current lane change reason for mff
 */
enum LaneChangeType { MAP, ACTIVE, INT, NOTYPE };

/**
 * @brief reasons why int request canceled for mff
 */
enum LaneChangeFailReason { UNSUITABLE, SOLIDLINE, TIMEOUT, NONE };

/**
 * @brief split merge point type
 */
enum SplitMergeType { NO_SPLIT_MERGE, SPLIT, MERGE };

}  // namespace planning