#pragma once

#include <cstddef>
#include <string>
#include "astar_decider.h"
#include "hybrid_astar_common.h"
#include "node3d.h"
#include "pose2d.h"
#include "euler_distance_transform.h"
#include "park_reference_line.h"


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

  double start_point_s_;
  double end_point_s_;
  double dist_;
  size_t start_point_id_;
  size_t end_point_id_;

  AstarPathGear gear_;
};

// use line model to inference next drive info.
struct InferenceDriveDist {
  double dist_to_ref_line;

  bool gear_drive_has_obs;
  double gear_drive_dist_to_obs;

  bool gear_reverse_has_obs;
  double gear_reverse_dist_to_obs;

  double advised_drive_dist;
};

// decider: check path drive distance.
class FuturePathDecider : public AstarDecider {
 public:
  FuturePathDecider() = default;

  void Process(const HybridAStarResult *history_path,
               const PlanningReason plan_reason, const Pose2D &ego_pose,
               EulerDistanceTransform *edt, const ParkReferenceLine *ref_line,
               ParkFirstActionRequest *future_path_request);

  void Process(const Pose2D &start, const Pose2D &end);

  const AstarPathGear GetNextPathGearByHistory();

  const double GetNextPathLenByHistory();

  const size_t GetNextPathStartPointId();

  const bool IsNextPathNoGearSwitchByHistory();

 private:
  std::string PathGearSwitchNumberString(
      const PathGearSwitchNumber &gear_number);

  void CalcDriveDistByLineModel(const Pose2D &ego_pose,
                                EulerDistanceTransform *edt,
                                const ParkReferenceLine *ref_line);

  void CalcDriveDistByHistoryPath(const HybridAStarResult *history_path,
                                  const PlanningReason plan_reason);

  void UpdateFuturePathRequest(ParkFirstActionRequest *future_path_request);

  // use history path info as an heuristic info
  HistoryPathDriveInfo history_path_info_;

  // use raycast path safe distance as an heuristic info
  InferenceDriveDist future_drive_dist_info_;
};

}  // namespace planning