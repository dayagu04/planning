#pragma once

#include "node3d.h"
#include "hybrid_astar_common.h"
#include "pose2d.h"
#include "./../reeds_shepp/rs_path_request.h"

namespace planning {

enum class CollisionDetectionMethod {
  edt,
  gjk,
};

// request-response mode.
struct AstarRequest {
  double timestamp_ms;
  AstarPathGenerateType path_generate_method;

  // when ouput a path, the first action include: gear, drive distance et al.
  ParkFirstActionRequest first_action_request;
  AstarPathGear history_gear;

  ParkSpaceType space_type;

  ParkingTask parking_task;

  ParkingVehDirectionRequest head_request;

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
  // notice: astar goal maybe different from slot limiter.
  double vertical_slot_target_adjust_dist_;

  double slot_width;
  double slot_length;
};

void DebugAstarRequestString(const AstarRequest &request);

}