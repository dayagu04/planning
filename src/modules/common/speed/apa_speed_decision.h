#pragma once

#include "pose2d.h"
namespace planning {

enum class LonDecisionReason {
  NONE = 0,
  USS_POINT_CLOUD_COLLISION = 1,
  PATH_FINISH = 2,
  STATIC_OCC_COLLISION = 3,
  DYNAMIC_OCC_COLLISION = 4,
  GROUD_LINE_COLLISION = 5,
  // If object is human, need add a stop desion.
  STATIC_POLYGON_OBJECT = 6,
  DYNAMIC_POLYGON_OBJECT = 7,
  LIMITER_COLLISION = 8,
  // slot pose change too much, add stop decision.
  SLOT_POSE_CHANGE = 10,
  // If control path is collision with obstacle, add stop decision.
  CONTROL_PATH_COLLISION = 11,
  // If point distance is small with obstacles, add a speed limit decision.
  CLOSE_TO_OBSTACLE = 12,
  // path kappa change too much
  PATH_KAPPA_SWITCH = 13,
  // lateral error is big
  PATH_CONTROL_ERROR = 14,
  PATH_KAPPA = 15,
  // will be retired
  REMAIN_DIST = 16,
  SPEED_LIMIT_BY_TERMINAL = 17,
};

enum class LonDecisionType {
  NONE = 0,
  IGNORE = 1,
  OVERTAKE = 2,
  STOP = 3,
  CAUTION = 4,
};

enum class LateralDecisionType {
  NONE = 0,
  IGNORE = 1,
  // If decision is side pass, need nudge an obstacle.
  SIDE_PASS = 2,
};

struct ParkLonDecision {
  LonDecisionReason reason_code;
  LonDecisionType decision_type;

  // If is caused by obs, add this id.
  int32_t perception_id;

  // s from path start. ego is nearby path start.
  double path_s;
  Pose2D path_interaction_point;
  double lat_dist_to_obs;

  double decision_speed;
  // todo: add hard constraint, soft constraint.
  double lon_decision_buffer;

  void Clear() {
    reason_code = LonDecisionReason::NONE;
    decision_type = LonDecisionType::NONE;
    perception_id = 0;
    path_s = 100.0;
    lat_dist_to_obs = 10.0;
    decision_speed = 0;
    lon_decision_buffer = 0;
    return;
  }
};

struct SpeedDecisions {
  std::vector<ParkLonDecision> decisions;
};

enum class RoadScenario {
  UPHILL,
  DOWNHILL,
  BUMP,
  CURBS,
  NORMAL,
  POTHOLE,
};

enum class AccelerationMode {
  // [0, 0.5]
  SOFT_ACCELERATE,
  HARD_ACCELERATE,
  // [0, -0.5]
  SOFT_BRAKE,
  // (-0.5, -2.0)
  HARD_BRAKE,
  // (-2.0, -inf)
  EMERGENCY_BRAKE,
  CRUISE,
};

enum class SpeedMode {
  // 5 km/h
  FAST,
  // 4 km/h
  MIDDLE,
  // 3 km/h
  SLOW
};

const ParkLonDecision* GetCloseStopDecision(
    const SpeedDecisions* speed_decisions);

}  // namespace planning