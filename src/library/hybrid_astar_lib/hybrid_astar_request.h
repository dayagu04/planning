#pragma once

#include <bits/stdint-uintn.h>
#include <cstddef>
#include "./../reeds_shepp/rs_path_request.h"
#include "hybrid_astar_common.h"
#include "node3d.h"
#include "pose2d.h"

namespace planning {

enum class CollisionDetectionMethod {
  EDT,
  GJK,
  SAT,
  AABB,
};

enum class PathGearRequest {
  NONE = 0,
  GEAR_REVERSE_ONLY,
  GEAR_DRIVE_ONLY,
  MAX_NUMBER
};

// request-response mode.
struct AstarRequest {
  double timestamp_ms;
  AstarPathGenerateType path_generate_method;

  // when ouput a path, the first action include: gear, drive distance et al.
  ParkFirstActionRequest first_action_request;
  AstarPathGear history_gear;

  ParkSpaceType space_type;
  size_t slot_id;

  ParkingVehDirection direction_request;

  RSPathRequestType rs_request;
  PlanningReason plan_reason;

  CollisionDetectionMethod path_safe_detect_method;

  // slot pose
  Pose2D base_pose_;
  // local frame in slot
  Pose2D start_;
  // local frame in slot, this goal is astar end point. Astar goal maybe
  // different with real goal in slot.
  Pose2D goal_;

  // real goal for park in, decide by slot limiter
  Pose2D real_goal;

  double slot_width;
  double slot_length;
};

void DebugAstarRequestString(const AstarRequest &request);

}  // namespace planning