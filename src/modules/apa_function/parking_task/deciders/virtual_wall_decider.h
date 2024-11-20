#pragma once

#include "library/hybrid_astar_lib/hybrid_astar_common.h"
#include "parking_task.h"

namespace planning {

// generate virtual wall by ego pose and slot, used by astar, need refact.
class VirtualWallDecider : public ParkingTask {
 public:
  VirtualWallDecider() = default;

  void Process(std::vector<Position2D>& points, const double channel_length,
              const double channel_width, const double slot_width,
              const double slot_length, const Pose2D& ego_pose,
              const Pose2D& end, const ParkSpaceType slot_type,
              const SlotRelativePosition slot_side);

  void SampleInLine(const Eigen::Vector2d& start, const Eigen::Vector2d& end,
                    std::vector<Position2D>* points);

  void GenerateCarRelativePosition(const Pose2D& ego_pose);

 private:
  void CalcVerticalVirtualWall(std::vector<Position2D>& points,
                               const double channel_length,
                               const double channel_width,
                               const double slot_width,
                               const double slot_length, const Pose2D& ego_pose,
                               const Pose2D& end);

  void RightSideParallelVirtualWall(std::vector<Position2D>& points,
                                   const double slot_width,
                                   const double slot_length,
                                   const Pose2D& ego_pose, const Pose2D& end);

  void LeftSideParallelVirtualWall(std::vector<Position2D>& points,
                                   const double slot_width,
                                   const double slot_length,
                                   const Pose2D& ego_pose, const Pose2D& end);

  std::string name_;
  Pose2D start_;
  Pose2D end_;

  VehRelativePosition relative_position_;
};

}  // namespace planning