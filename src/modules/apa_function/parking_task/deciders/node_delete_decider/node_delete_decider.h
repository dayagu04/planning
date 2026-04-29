#pragma once
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>

#include "aabb2d.h"
#include "apa_slot_manager.h"
#include "collision_detector_interface.h"
#include "common_math.h"
#include "curve_node.h"
#include "hybrid_astar_common.h"
#include "hybrid_astar_context.h"
#include "hybrid_astar_request.h"
#include "node3d.h"
#include "parking_task.h"
namespace planning {
namespace apa_planner {

enum NodeDeleteReason : uint8_t {
  NOT_DELETE = 0,
  UNKNOWN = 1,  // respresent not delete
  COLLISION = 2,
  OUT_OF_POSE_BOUND = 3,
  OUT_OF_GRID_BOUND = 4,
  UNEXPECTED_GEAR = 5,
  EXCEED_GEAR_SHIFT_NUMBER = 6,
  PARENT_NODE = 7,
  START_NODE = 8,
  LOOP_BACK_NODE = 9,
  SAME_GRID_NODE_CONTINUOUS = 10,
  ZIGZAG_PATH = 11,
  BACKTRACK_PATH = 12,
  EXCEED_SCURVE_NUMBER = 13,
  OUT_OF_POSE_STANDARD = 14,
};

const std::string GetNodeDeleteReasonString(const NodeDeleteReason reason);
void PrintNodeDeleteReason(const NodeDeleteReason reason,
                           const bool enable_log);

struct NodePoseBound {
  float heading_up_bound;
  float heading_down_bound;
  float x_up_bound;
  float x_down_bound;
  float curve_x_down_bound = -26.8;
  float y_up_bound;
  float y_down_bound;
};

struct NodeDeleteInput {
  ParkingScenarioType scenario_type;
  EgoInfoUnderSlot ego_info_under_slot;
  size_t start_id;
  MapBound map_bound;
  NodeGridIndex map_grid_bound;
  PlannerOpenSpaceConfig config;
  size_t max_gear_shift_number;
  size_t max_scurve_number;
  PathColDetBuffer path_col_det_buffer;
  bool swap_start_goal = false;
  bool need_cal_obs_dist = false;
  bool enable_smart_fold_mirror = false;
  float sample_ds = 0.1;
};

struct NodeDeleteRequest {
  AstarPathGear gear_request = AstarPathGear::NONE;

  AstarPathGear cur_gear = AstarPathGear::NONE;

  int explored_node_num = 3;

  Node3d* current_node = nullptr;
  CurveNode* curve_node = nullptr;
  Node3d* old_node = nullptr;
  Node3d* parent_node = nullptr;
  NodeDeleteRequest() {}
  ~NodeDeleteRequest() {}
};

class NodeDeleteDecider final : public ParkingTask {
 public:
  NodeDeleteDecider() {}
  NodeDeleteDecider(
      const std::shared_ptr<CollisionDetectorInterface> col_det_interface_ptr) {
    SetCollisionDetectorIntefacePtr(col_det_interface_ptr);
  }
  ~NodeDeleteDecider() = default;

  void Process(const NodeDeleteInput input);

  const bool CheckShouldBeDeleted(NodeDeleteRequest request);

  const NodeDeleteReason GetNodeDeleteReason() { return reason_; }

  const bool UpdateObsDistRelativeSlot(NodeDeleteRequest request);

  static const float CalcObsDistCost(const float obs_dist,
                                     const float gear_change_penalty,
                                     const float length_cost,
                                     const float critical_dist,
                                     const float safe_dist,
                                     const float adsolute_safe_dist);

 private:
  const bool CheckShouldBeDeletedForPerpendicularIn();

  const bool CheckPtsCollision(
      const std::vector<common_math::PathPt<float>>& origin_pts,
      bool is_special_node, ObsToPathDistRelativeSlot* obs_dist,
      double* safe_remain_dist);

  void SplitPathPtsUsingGradeBuffer(
      const std::vector<common_math::PathPt<float>>& origin_pts,
      std::vector<GradeBufferPathPts>& grade_buffer_pts_vec);

  const GradeColDetBufferType GetGradeBufferType(
      const common_math::PathPt<float>& pt);

  const bool CheckCollision();

  const bool CheckOutOfPoseBound();

  const bool CheckOutOfPoseStandard();

  const bool CheckOutOfGridBound();

  const bool CheckUnexpectedGear();

  const bool CheckExceedGearShiftNumber();

  const bool CheckDelByParentNode();

  const bool CheckDelByStartNode();

  const bool CheckDelByLoopBackNode();

  const bool CheckDelBySameGridNodeContinuous();

  const bool CheckZigzagPath();

  const bool CheckBacktrackPath();

  const bool CheckExceedScurveNum();

 private:
  NodeDeleteRequest request_;
  NodeDeleteReason reason_;
  NodeDeleteInput input_;

  cdl::AABB2f slot_box_;

  cdl::AABB2f slot_entrance_box_;

  NodePoseBound pose_bound_;

  // using member variables to reduce construction costs
  // it can allow capacity reuse
  std::vector<common_math::PathPt<float>> search_pts_;
  std::vector<std::vector<common_math::PathPt<float>>> curve_ptss_;
  std::vector<AstarPathGear> curve_gears_;
  std::vector<common_math::PathPt<float>> grade_segment_pts_;
};
}  // namespace apa_planner
}  // namespace planning