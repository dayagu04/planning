#pragma once

#include "astar_decider.h"
#include "hybrid_astar_common.h"
#include "./../occupancy_grid_map/point_cloud_obstacle.h"

namespace planning {

// generate virtual wall by ego pose and slot, need refact.
class VirtualWallDecider : public AstarDecider {
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
              const Pose2D& end);

  void SampleInLine(const Eigen::Vector2d& start, const Eigen::Vector2d& end,
                    std::vector<Position2D>* points);

  void GenerateCarRelativePosition(const Pose2D& ego_pose);

 private:
  CarSlotRelativePosition relative_position_;
};

}  // namespace planning