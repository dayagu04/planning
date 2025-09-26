#pragma once

#include <cstddef>
#include <string>
#include <vector>

#include "astar_decider.h"
#include "euler_distance_transform.h"
#include "hybrid_astar_common.h"
#include "hybrid_astar_request.h"
#include "node3d.h"
#include "park_reference_line.h"
#include "pose2d.h"
#include "rs_path_interpolate.h"

namespace planning {

struct PoseRegulateCandidate {
  Pose2f pose;
  // 位姿对应的车辆真实外壳到障碍物的距离.
  float dist_to_obs;
  float lat_offset;

  PoseRegulateCandidate() = default;
  PoseRegulateCandidate(const Pose2f &point, const float dist,
                        const float offset) {
    pose = point;
    dist_to_obs = dist;
    lat_offset = offset;
  }

  PoseRegulateCandidate(const PoseRegulateCandidate &other)
      : pose(other.pose),
        dist_to_obs(other.dist_to_obs),
        lat_offset(other.lat_offset) {}
};

struct TerminalCheckBoundary {
  float upper;
  float lower;
  float step;
  int number;
};

struct GuessTerminal : public Pose2f {
  float dist_to_obs;
};

#define INITIAL_GUESS_PATH_MAX_POINT (50)

struct InitialGuessPath {
  GuessTerminal points[INITIAL_GUESS_PATH_MAX_POINT];
  int32_t size;
  float min_dist_to_obs;

  void AddPoint(const Pose2f &point, const float dist) {
    if (size < 0) {
      size = 0;
    }
    if (size >= INITIAL_GUESS_PATH_MAX_POINT) {
      return;
    }

    points[size].x = point.x;
    points[size].y = point.y;
    points[size].theta = point.theta;
    points[size].dist_to_obs = dist;
    size++;

    return;
  }

  void Clear() {
    size = 0;
    min_dist_to_obs = 0.0f;
    return;
  }

  const bool IsValid() const {
    if (min_dist_to_obs < 0.061f) {
      return false;
    }

    return true;
  }

  const bool IsPathAllPointsSafe(const float dist) {
    for (int32_t i = 0; i < size; i++) {
      if (points[i].dist_to_obs < dist) {
        return false;
      }
    }

    return true;
  }
};

// 目标pose调节器.
// Todo: all target pose decisions should be moved to here.
// If origin goal is collided, move goal to left/right/top/bottom.
class TargetPoseRegulator : public AstarDecider {
 public:
  TargetPoseRegulator() = default;

  void Process(EulerDistanceTransform *edt, const AstarRequest *request,
               const Pose2f &ego_pose, const Pose2f &target,
               const VehicleParam &veh_param,
               const ParkingVehDirection &direction_request);

  void Process(const Pose2f &start, const Pose2f &end);

  void Clear();

  // Get most safe target pose, 15 cm is ok.
  const PoseRegulateCandidate GetCandidatePose(
      const float lat_buffer = 0.15f) const;

  const float GetEgoObsDist() const { return ego_dist_to_obs_; }

 private:
  const bool IsParkingIn(const AstarRequest *request);

  void GenerateCandidatesForVerticalSlot(EulerDistanceTransform *edt,
                                         const AstarRequest *request,
                                         const VehicleParam &veh_param);

  void GenerateCandidatesForVerticalHeadOut(EulerDistanceTransform *edt,
                                            const AstarRequest *request,
                                            const VehicleParam &veh_param);

  void GenerateCandidatesForVerticalHeadOut(
      EulerDistanceTransform *edt, const ParkingVehDirection &direction_request,
      const VehicleParam &veh_param);

  // check min dist by x range
  const float GetCandidatePathByXRange(const Pose2f &global_pose,
                                       EulerDistanceTransform *edt);

  const float GetDistToObsHeadOut(const Pose2f &global_pose,
                                  const ParkingVehDirection &direction_request,
                                  EulerDistanceTransform *edt);

  void DebugString();

  void UpdateReferenceLinePath(const AstarRequest *request,
                               const VehicleParam &veh_param,
                               const ParkingVehDirection &direction_request,
                               EulerDistanceTransform *edt);

  bool IsReferenceLineSafeEnough();

  void GenerateXboundary(const AstarRequest *request,
                         const VehicleParam &veh_param);

  void GenerateYboundary(const AstarRequest *request,
                         const VehicleParam &veh_param);

  void GetMaxDeepthPoint(const InitialGuessPath &path, const float buffer);

 private:
  Pose2f target_;
  // used by park out
  std::vector<PoseRegulateCandidate> candidate_info_;
  const AstarRequest *request_;

  // rear axis center
  TerminalCheckBoundary x_check_bounday_;
  TerminalCheckBoundary y_check_bounday_;

  // max dist to cross over slot inside line
  float cross_the_slot_line_max_dist_;

  float ego_dist_to_obs_;
  float max_lat_buffer_;

  std::vector<InitialGuessPath> paths_;


};

}  // namespace planning