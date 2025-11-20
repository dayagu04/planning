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

#define TERMINAL_GUESS_PATH_MAX_POINT (50)

struct TerminalCandidatePoint {
  Pose2f pose;
  // 位姿对应的车辆真实外壳到障碍物的距离.
  float dist_to_obs;
  float lat_offset;

  TerminalCandidatePoint() = default;
  TerminalCandidatePoint(const Pose2f &point, const float dist,
                         const float offset) {
    pose = point;
    dist_to_obs = dist;
    lat_offset = offset;
  }

  TerminalCandidatePoint(const TerminalCandidatePoint &other)
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

struct TerminalGuessPoint : public Pose2f {
  float dist_to_obs;
};

struct TerminalGuessPath {
  TerminalGuessPoint points[TERMINAL_GUESS_PATH_MAX_POINT];
  int32_t size;

  // integral strategy
  float safe_width_integral;
  // analyze all points minimum dist to obs, but if one path point dist is
  // collision, do not consider this point dist.
  float min_dist_to_obs;

  void AddPoint(const Pose2f &point, const float dist);

  void Clear();

  const bool IsPathAllPointsSafe(const float dist);
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

  const TerminalCandidatePoint GetCandidatePose(
      const float lat_buffer = 0.15f, const float min_lat_buffer = 0.08f);

  const float GetEgoObsDist() const { return ego_dist_to_obs_; }

 private:
  const bool IsParkingIn(const AstarRequest *request);

  void GenerateCandidates(EulerDistanceTransform *edt,
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

  const void GetSafeIntegralForPath(TerminalGuessPath &path, const float buffer,
                                    const float min_lateral_buffer);

  const TerminalCandidatePoint GetCandidatePoseForParkOut(
      const float lat_buffer = 0.15f, const float min_lat_buffer = 0.08f);

  // Get most safe target pose, 15 cm is ok, 8 cm is not safe. So buffer range
  // [8, 15].
  const TerminalCandidatePoint GetCandidatePoseForParkIn(
      const float lat_buffer = 0.15f, const float min_lateral_buffer = 0.08f);

  void DebugPath(const TerminalGuessPath &path) const;

  float GetLonPosition(const TerminalGuessPath *path, const float lon_lower,
                       const float lon_upper, const float lat_buffer);

 private:
  // real target, decided by limiter.
  Pose2f target_;
  // used by park out, will be retired
  std::vector<TerminalCandidatePoint> candidate_info_;
  const AstarRequest *request_;

  // rear axis center
  TerminalCheckBoundary x_check_bounday_;
  TerminalCheckBoundary y_check_bounday_;

  // max dist to cross over slot inside line
  // positve: distance to inside line, not cross line.
  // negative: distance to inside line, vehicle cross line.
  float cross_the_slot_line_max_dist_;

  float ego_dist_to_obs_;
  float max_lat_buffer_;

  std::vector<TerminalGuessPath> paths_;

  // obstacle is always jumping in slot bottom, so add a buffer.
  float low_confidence_zone_for_vertical_;
  float low_confidence_zone_for_slant_;
};

}  // namespace planning