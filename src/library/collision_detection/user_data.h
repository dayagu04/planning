#pragma once

#include "collision_box2d.h"
#include "collision_object2d.h"
#include "constants.h"
#include "convex2d.h"
#include "dynamic_tree.h"
#include "node_base2d.h"
#include "types.h"

namespace cdl {
struct UserData {
  /** id of collision object in the collision object array */
  int32 id;
  /** id of the trajecotry array in predicted trajectory set */
  int32 traj_id;
  /** id of the pose along a certain trajectory */
  int32 pose_id;

  void setUserData(const UserData &ud) {
    id = ud.id;
    traj_id = ud.traj_id;
    pose_id = ud.pose_id;
  }
};

}  // namespace cdl