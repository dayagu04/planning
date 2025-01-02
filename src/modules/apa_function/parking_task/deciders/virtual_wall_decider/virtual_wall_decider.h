#pragma once

#include "gjk2d_interface.h"
#include "library/hybrid_astar_lib/hybrid_astar_common.h"
#include "parking_task.h"
#include "polygon_base.h"
#include "pose2d.h"

namespace planning {

struct VirtualWallBoundary {
  double x_upper;
  double x_lower;
  double y_upper;
  double y_lower;

  VirtualWallBoundary() = default;

  VirtualWallBoundary(const Position2D& center)
      : x_upper(center.x),
        x_lower(center.x),
        y_upper(center.y),
        y_lower(center.y) {}

  VirtualWallBoundary(const double x_upper_, const double x_lower_,
                      const double y_upper_, const double y_lower_)
      : x_upper(x_upper_),
        x_lower(x_lower_),
        y_upper(y_upper_),
        y_lower(y_lower_){};

  void Combine(const VirtualWallBoundary& a) {
    x_lower = std::min(x_lower, a.x_lower);
    y_lower = std::min(y_lower, a.y_lower);
    x_upper = std::max(x_upper, a.x_upper);
    y_upper = std::max(y_upper, a.y_upper);
  }
};

// generate virtual wall by ego pose and slot, used by astar, need refact.
class VirtualWallDecider : public ParkingTask {
 public:
  VirtualWallDecider() = default;

  void Init(const Pose2D& ego_pose);

  void Process(std::vector<Position2D>& points, const double slot_width,
               const double slot_length, const Pose2D& ego_pose,
               const Pose2D& end, const ParkSpaceType slot_type,
               const SlotRelativePosition slot_side);

  void Reset(const Pose2D& ego_pose) {
    channel_bound_ = VirtualWallBoundary(Position2D(ego_pose.x, ego_pose.y));
    return;
  }

 private:
  void CalcVerticalVirtualWall(std::vector<Position2D>& points,
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

  // If virtual wall is collision with ego, remove virtual wall point.
  const bool IsVirtualWallPointCollision(const Position2D& point);

  void GenerateVehPolygonInSlot(const Pose2D& ego);

  void SampleInLineSegment(const Eigen::Vector2d& start,
                           const Eigen::Vector2d& end,
                           std::vector<Position2D>* points);

  void GenerateCarRelativePosition(const Pose2D& ego_pose);

 private:
  std::string name_;
  Pose2D start_;
  Pose2D end_;

  VehRelativePosition relative_position_;
  Polygon2D ego_polygon_in_slot_;
  GJK2DInterface gjk_interface_;

  // 感知存在盲区，规划认为盲区外存在障碍物. 车辆坐标系.
  Polygon2D blind_local_box_;
  // 车位坐标系
  Polygon2D blind_global_box_;

  VirtualWallBoundary channel_bound_;
};

}  // namespace planning