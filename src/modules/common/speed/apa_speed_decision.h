#pragma once

#include "pose2d.h"
namespace planning {

enum class StopDecisionReason {
  NONE = 0,
  USS_POINT_CLOUD_COLLISION = 1,
  PATH_FINISH = 2,
  VIRTUAL_WALL_COLLISION = 3,
  OCC_COLLISION = 4,
  GROUD_LINE_COLLISION = 5,
  // If object is human, need add a stop desion.
  STATIC_POLYGON_OBJECT_COLLISION = 6,
  DYNAMIC_POLYGON_OBJECT = 7,
  LIMITER_COLLISION = 8,
  USS_DISTANCE_SMALL = 9,
  // slot pose change too much, add stop decision for gear switch?
  SLOT_POSE_CHANGE = 10,
  // If predicted tracking path collision with obstacle, add stop decision.
  PREDICTED_TRACKING_PATH_COLLISION = 11,
};

enum class SpeedLimitReason {
  NONE = 0,
  // If distance < 0.3 meter, add a speed limit decision.
  CLOSE_TO_OBSTACLE = 1,
  // path kappa change too much
  PATH_KAPPA_CHANGE = 2,
  // 方向盘和路径曲率，差距大要限速.
  KAPPA_GAP_BETTWEN_PATH_WITH_WHEEL = 3,
  PATH_KAPPA = 4,
};

struct StopDecision {
  StopDecisionReason reason_code;
  // If caused by obs, add this id.
  int32_t perception_id;

  // s from path start. ego is nearby path start.
  double path_s;
  Pose2D stop_point;

  double lateral_safe_buffer;
  double lon_safe_buffer;
};

struct SpeedLimitDecision {
  SpeedLimitReason reason_code;

  // If speed limit is caused by obs, add this id.
  int32_t perception_id;

  // s from path start. ego is nearby path start.
  double path_s;
  Pose2D path_point;

  double advised_speed;
  double dist_to_obs;
};

struct SpeedDecisions {
  std::vector<SpeedLimitDecision> speed_limit_decisions;
  std::vector<StopDecision> stop_decisions;
};

}  // namespace planning