#pragma once

#include "parking_task.h"
#include "pose2d.h"
#include "src/library/convex_collision_detection/gjk2d_interface.h"
#include "src/library/geometry_lib/include/geometry_math.h"
#include "src/library/hybrid_astar_lib/hybrid_astar_common.h"

namespace planning {
namespace apa_planner {
struct VirtualWallBoundary {
  float x_upper;
  float x_lower;
  float y_upper;
  float y_lower;

  VirtualWallBoundary() = default;

  VirtualWallBoundary(const Position2D& center)
      : x_upper(center.x),
        x_lower(center.x),
        y_upper(center.y),
        y_lower(center.y) {}

  VirtualWallBoundary(const float x_upper_, const float x_lower_,
                      const float y_upper_, const float y_lower_)
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

  bool Contain(const Position2D& p) const {
    if (x_lower > p.x || y_lower > p.y) {
      return false;
    }

    if (x_upper < p.x || y_upper < p.y) {
      return false;
    }

    return true;
  }
};

// generate virtual wall by ego pose and slot, used by astar, need refact.
class VirtualWallDecider : public ParkingTask {
 public:
  VirtualWallDecider() = default;

  void Init(const Pose2D& ego_pose);

  void Process(std::vector<Position2D>& points, const float slot_width,
               const float slot_length, const Pose2D& ego_pose,
               const Pose2D& end, const ParkSpaceType slot_type,
               const pnc::geometry_lib::SlotSide slot_side,
               const ParkingVehDirection parking_type,
               const float passage_height);

  void Reset(const Pose2D& ego_pose) {
    passage_bound_ = VirtualWallBoundary(Position2D(ego_pose.x, ego_pose.y));
    return;
  }

 private:
  void CalcVerticalVirtualWall(std::vector<Position2D>& points,
                               const float slot_width, const float slot_length,
                               const Pose2D& ego_pose, const Pose2D& end,
                               const float virtual_wall_x_offset,
                               const float virtual_wall_y_offset,
                               const float passage_half_length,
                               const float passage_height);

  void RightSideParallelVirtualWall(std::vector<Position2D>& points,
                                    const float slot_width,
                                    const float slot_length,
                                    const Pose2D& ego_pose, const Pose2D& end);

  void LeftSideParallelVirtualWall(std::vector<Position2D>& points,
                                   const float slot_width,
                                   const float slot_length,
                                   const Pose2D& ego_pose, const Pose2D& end);

  // If virtual wall is collision with ego, remove virtual wall point.
  const bool IsVirtualWallPointCollision(const Position2D& point);

  void GenerateVehPolygonInSlot(const Pose2D& ego);

  void SampleInLineSegment(const Eigen::Vector2d& start,
                           const Eigen::Vector2d& end,
                           const bool delete_blind_zone_point,
                           std::vector<Position2D>* points);

  void GetVehicleBound();

 private:
  std::string name_;
  Pose2D start_;
  Pose2D end_;

  Polygon2D ego_polygon_in_slot_;
  GJK2DInterface gjk_interface_;

  // 感知存在盲区，规划认为盲区外存在障碍物. 车辆坐标系.
  Polygon2D blind_local_box_;
  // 车位坐标系
  Polygon2D blind_global_box_;

  // 感知范围6x6meter，所以passage范围尽量设置小一些，否则path经常穿墙、穿车而过.
  VirtualWallBoundary passage_bound_;

  // vehicle boundary
  VirtualWallBoundary veh_boundary_;
};
}  // namespace apa_planner
}  // namespace planning