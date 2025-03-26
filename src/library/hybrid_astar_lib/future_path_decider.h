#pragma once

#include <cstddef>
#include <string>
#include "astar_decider.h"
#include "euler_distance_transform.h"
#include "hybrid_astar_common.h"
#include "node3d.h"
#include "park_reference_line.h"
#include "pose2d.h"
#include "rs_path_interpolate.h"

namespace planning {

enum class PathGearSwitchNumber {
  NONE,
  ONCE,
  TWICE,
  MANY_TIMES,
};

// use history path 推理下一次换档信息，行驶距离信息等
struct HistoryPathDriveInfo {
  PathGearSwitchNumber gear_switch_number_;

  float start_point_s_;
  float end_point_s_;
  float dist_;
  size_t start_point_id_;
  size_t end_point_id_;

  AstarPathGear gear_;
};

// use line model to inference next drive info.
struct InferenceDriveDist {
  float dist_to_ref_line;

  bool gear_drive_has_obs;
  float gear_drive_dist_to_obs;

  bool gear_reverse_has_obs;
  float gear_reverse_dist_to_obs;

  float advised_drive_dist;
};

// decider: check path drive distance.
class FuturePathDecider : public AstarDecider {
 public:
  FuturePathDecider() = default;

  void Process(const HybridAStarResult *history_path,
               const PlanningReason plan_reason, const Pose2D &ego_pose,
               EulerDistanceTransform *edt, const ParkReferenceLine *ref_line,
               const float min_turn_radius, const bool swap_start_goal,
               const AstarPathGenerateType path_generate_type,
               ParkFirstActionRequest *future_path_request);

  void Process(const Pose2D &start, const Pose2D &end);

  const AstarPathGear GetNextPathGearByHistory();

  const float GetNextPathLenByHistory();

  const size_t GetNextPathStartPointId();

  const bool IsNextPathNoGearSwitchByHistory();

  // if left, radius is positive
  void GetPathByRadius(const Pose2D *start_pose, const float length,
                       const float radius, const bool is_forward,
                       std::vector<Pose2D> *path);

 private:
  std::string PathGearSwitchNumberString(
      const PathGearSwitchNumber &gear_number);

  void CalcDriveDistByLineModel(const Pose2D &ego_pose,
                                EulerDistanceTransform *edt,
                                const ParkReferenceLine *ref_line);

  void CalcDriveDistByHistoryPath(const HybridAStarResult *history_path,
                                  const PlanningReason plan_reason);

  void UpdateFuturePathRequest(ParkFirstActionRequest *future_path_request);

  // radius: if left turn, radius is positive
  void GetVehCircleByPose(const Pose2D *pose, const float radius,
                          const AstarPathGear gear, VehicleCircle *veh_circle);

  // if left, radius is positive
  void GetPathByCircle(const Pose2D *start_pose, const float arc,
                       const float radius, const bool is_forward,
                       std::vector<Pose2D> *path);

  void GetPathByLine(const Pose2D *start_pose, const float length,
                     const bool is_forward, std::vector<Pose2D> *path);

  void GetStraightLinePoint(const Pose2D *start_state,
                            const float dist_to_start,
                            const Pose2D *unit_vector, Pose2D *goal_state);
  // arc is positive.
  // inverse_radius is positive
  void InterpolateByArcOffset(const VehicleCircle *veh_circle,
                              const Pose2D *start_pose, const float arc,
                              const float inverse_radius, Pose2D *pose);

  void CalcDriveDistByCircleModel(const Pose2D &ego_pose,
                                  EulerDistanceTransform *edt,
                                  const ParkReferenceLine *ref_line);

 private:
  // use history path info as an heuristic info
  HistoryPathDriveInfo history_path_info_;

  // use raycast path safe distance as an heuristic info
  InferenceDriveDist future_drive_dist_info_;

  float min_turn_radius_;

  bool swap_start_goal_;

  AstarPathGenerateType path_generate_type_;
  // todo: move to config
  float astar_step_;
};

}  // namespace planning