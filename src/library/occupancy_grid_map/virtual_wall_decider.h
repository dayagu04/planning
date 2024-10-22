#pragma once

#include "point_cloud_obstacle.h"

namespace planning {

enum CarSlotRelativePosition { none, car_is_right, car_is_left, car_is_middle };

// generate virtual wall by ego pose and slot, need refact.
class VirtualWallDecider {
 public:
  VirtualWallDecider() = default;

  void Process(const Pose2D& start, const Pose2D& end);

  int GenerateVirtualWall(ParkObstacleList& obs_list,
                          const double channel_length,
                          const double channel_width, const double slot_width,
                          const double slot_length, const Pose2D& ego_pose);

  int Process(std::vector<Position2D>& points, const double channel_length,
              const double channel_width, const double slot_width,
              const double slot_length, const Pose2D& ego_pose,
              const Pose2D& end, const ParkSpaceType slot_type);

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

  std::string name_;
  Pose2D start_;
  Pose2D end_;

  CarSlotRelativePosition relative_position_;
};

}  // namespace planning