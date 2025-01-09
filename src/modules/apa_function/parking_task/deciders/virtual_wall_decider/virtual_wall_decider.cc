#include "virtual_wall_decider.h"

#include <algorithm>
#include <cmath>

#include "aabb2d.h"
#include "apa_param_config.h"
#include "collision_box2d.h"
#include "collision_detect_types.h"
#include "log_glog.h"
#include "polygon_base.h"
#include "pose2d.h"
#include "utils_math.h"

namespace planning {

#define SLOT_VIRTUAL_WALL_Y_OFFSET (0.5)
#define SLOT_VIRTUAL_WALL_X_OFFSET (3.0)

void VirtualWallDecider::GenerateVehPolygonInSlot(const Pose2D& ego) {
  const apa_planner::ApaParameters& config = apa_param.GetParam();
  Polygon2D ego_local_polygon;
  double veh_x_buffer = 0.3;
  double veh_y_buffer = 0.4;

  GenerateUpLeftFrameBox(
      &ego_local_polygon, -config.rear_overhanging - veh_x_buffer,
      -config.max_car_width / 2 - veh_y_buffer,
      config.car_length - config.rear_overhanging + veh_x_buffer,
      config.max_car_width / 2 + veh_y_buffer);
  ULFLocalPolygonToGlobal(&ego_polygon_in_slot_, &ego_local_polygon, ego);

  return;
}

const bool VirtualWallDecider::IsVirtualWallPointCollision(
    const Position2D& point) {
  bool is_collision;

  gjk_interface_.PolygonPointCollisionDetect(&is_collision,
                                             &ego_polygon_in_slot_, point);
  if (is_collision) {
    return true;
  }

  return false;
}

void VirtualWallDecider::Process(std::vector<Position2D>& points,
                                 const double slot_width,
                                 const double slot_length,
                                 const Pose2D& ego_pose, const Pose2D& end,
                                 const ParkSpaceType slot_type,
                                 const pnc::geometry_lib::SlotSide slot_side) {
  start_ = ego_pose;
  end_ = end;

  GenerateVehPolygonInSlot(ego_pose);
  // vehicle boundary
  GetVehicleBound();
  ULFLocalPolygonToGlobal(&blind_global_box_, &blind_local_box_, ego_pose);

  points.clear();

  if (slot_type == ParkSpaceType::VERTICAL ||
      slot_type == ParkSpaceType::SLANTING) {
    CalcVerticalVirtualWall(points, slot_width, slot_length, ego_pose, end);

    ILOG_INFO << "vertical slot virtual wall";
  } else {
    if (slot_side == pnc::geometry_lib::SLOT_SIDE_RIGHT) {
      RightSideParallelVirtualWall(points, slot_width, slot_length, ego_pose,
                                   end);
    } else if (slot_side == pnc::geometry_lib::SLOT_SIDE_LEFT) {
      LeftSideParallelVirtualWall(points, slot_width, slot_length, ego_pose,
                                  end);
    }

    ILOG_INFO << "parallel slot virtual wall";
  }

  return;
}

void VirtualWallDecider::SampleInLineSegment(const Eigen::Vector2d& start,
                                             const Eigen::Vector2d& end,
                                             const bool delete_blind_zone_point,
                                             std::vector<Position2D>* points) {
  const Eigen::Vector2d line = end - start;
  const Eigen::Vector2d unit_line_vec = line.normalized();
  double len = line.norm();

  double s = 0.0;
  double ds = 0.4;

  Eigen::Vector2d point;
  while (s < len) {
    point = start + s * unit_line_vec;
    s += ds;

    if (IsVirtualWallPointCollision(Position2D(point.x(), point.y()))) {
      continue;
    }

    // 如果盲区点在通道外面，不需要考虑盲区点
    if (delete_blind_zone_point &&
        !passage_bound_.Contain(Position2D(point.x(), point.y()))) {
      continue;
    }

    points->emplace_back(Position2D(point.x(), point.y()));
  }

  do {
    if (IsVirtualWallPointCollision(Position2D(point.x(), point.y()))) {
      break;
    }

    // 如果盲区点在通道外面，不需要考虑盲区点
    if (delete_blind_zone_point &&
        !passage_bound_.Contain(Position2D(point.x(), point.y()))) {
      break;
    }

    points->emplace_back(Position2D(end.x(), end.y()));
  } while (0);

  return;
}

void VirtualWallDecider::CalcVerticalVirtualWall(
    std::vector<Position2D>& points, const double slot_width,
    const double slot_length, const Pose2D& ego_pose, const Pose2D& end) {
  // slot virtual wall
  VirtualWallBoundary slot_boundary;
  slot_boundary.y_lower = -slot_width / 2.0 - SLOT_VIRTUAL_WALL_Y_OFFSET;
  slot_boundary.x_upper = slot_length - SLOT_VIRTUAL_WALL_X_OFFSET;
  slot_boundary.y_upper = slot_width / 2.0 + SLOT_VIRTUAL_WALL_Y_OFFSET;
  double lower_bound_x = -1.0;
  slot_boundary.x_lower = lower_bound_x;

  // passage virtual wall
  // lower
  VirtualWallBoundary tmp_passage_boundary;
  tmp_passage_boundary.x_lower = slot_length - SLOT_VIRTUAL_WALL_X_OFFSET;
  // passage up bound
  double passage_height = 8.0;
  double passage_up_bound_x = slot_length + passage_height;
  tmp_passage_boundary.x_upper = passage_up_bound_x;
  // passage left/right bound
  const double passage_half_length = 8.0;
  tmp_passage_boundary.y_lower = -passage_half_length;
  tmp_passage_boundary.y_upper = passage_half_length;
  tmp_passage_boundary.Combine(veh_boundary_);

  // boundary只能增长，不能缩减.
  // 否则车辆移动后，缩减后的boundary无法再生成一致的path.
  passage_bound_.Combine(tmp_passage_boundary);

  // sampling point
  // slot right bound
  SampleInLineSegment(
      Eigen::Vector2d(slot_boundary.x_lower, slot_boundary.y_lower),
      Eigen::Vector2d(slot_boundary.x_upper, slot_boundary.y_lower), false,
      &points);

  // slot left bound
  SampleInLineSegment(
      Eigen::Vector2d(slot_boundary.x_upper, slot_boundary.y_upper),
      Eigen::Vector2d(slot_boundary.x_lower, slot_boundary.y_upper), false,
      &points);

  // passage lower right boundary
  SampleInLineSegment(
      Eigen::Vector2d(slot_boundary.x_upper, slot_boundary.y_lower),
      Eigen::Vector2d(passage_bound_.x_lower, passage_bound_.y_lower), false,
      &points);

  // passage lower left boundary
  SampleInLineSegment(
      Eigen::Vector2d(slot_boundary.x_upper, slot_boundary.y_upper),
      Eigen::Vector2d(passage_bound_.x_lower, passage_bound_.y_upper), false,
      &points);

  // upper boundary
  SampleInLineSegment(
      Eigen::Vector2d(passage_bound_.x_upper, passage_bound_.y_upper),
      Eigen::Vector2d(passage_bound_.x_upper, passage_bound_.y_lower), false,
      &points);

  // right boundary
  SampleInLineSegment(
      Eigen::Vector2d(passage_bound_.x_upper, passage_bound_.y_lower),
      Eigen::Vector2d(passage_bound_.x_lower, passage_bound_.y_lower), false,
      &points);

  // left boundary
  SampleInLineSegment(
      Eigen::Vector2d(passage_bound_.x_upper, passage_bound_.y_upper),
      Eigen::Vector2d(passage_bound_.x_lower, passage_bound_.y_upper), false,
      &points);

  // slot lower bound
  SampleInLineSegment(
      Eigen::Vector2d(slot_boundary.x_lower, slot_boundary.y_upper),
      Eigen::Vector2d(slot_boundary.x_lower, slot_boundary.y_lower), false,
      &points);

  // perception blind zone
  int point_a_idx;
  int point_b_idx;
  for (int i = 0; i < blind_global_box_.vertex_num; i++) {
    if (i == blind_global_box_.vertex_num - 1) {
      point_a_idx = i;
      point_b_idx = 0;

    } else {
      point_a_idx = i;
      point_b_idx = i + 1;
    }

    SampleInLineSegment(
        Eigen::Vector2d(blind_global_box_.vertexes[point_a_idx].x,
                        blind_global_box_.vertexes[point_a_idx].y),
        Eigen::Vector2d(blind_global_box_.vertexes[point_b_idx].x,
                        blind_global_box_.vertexes[point_b_idx].y),

        true, &points);
  }

  return;
}

#define PARALLEL_SLOT_X_OFFSET (7.0)
#define PARALLEL_SLOT_Y_OFFSET (1.0)
void VirtualWallDecider::RightSideParallelVirtualWall(
    std::vector<Position2D>& points, const double slot_width,
    const double slot_length, const Pose2D& ego_pose, const Pose2D& end) {
  // width is the slot upper edge to a virtual wall
  const double passage_width = 8;
  const double passage_top_buffer = 8.0;
  const double passage_bottom_buffer = 8.0;

  // slot right wall
  Eigen::Vector2d right_wall_upper =
      Eigen::Vector2d(slot_length + PARALLEL_SLOT_X_OFFSET,
                      -slot_width / 2.0 - PARALLEL_SLOT_Y_OFFSET);

  Eigen::Vector2d right_wall_lower = Eigen::Vector2d(
      -PARALLEL_SLOT_X_OFFSET, -slot_width / 2.0 - PARALLEL_SLOT_Y_OFFSET);

  // slot bottom wall
  Eigen::Vector2d slot_bottom_wall_left =
      Eigen::Vector2d(-PARALLEL_SLOT_X_OFFSET, slot_width / 2.0 - 1);

  Eigen::Vector2d slot_bottom_wall_right =
      Eigen::Vector2d(-PARALLEL_SLOT_X_OFFSET, -slot_width / 2.0 - 1);

  // slot top wall
  Eigen::Vector2d slot_top_wall_left = Eigen::Vector2d(
      slot_length + PARALLEL_SLOT_X_OFFSET, slot_width / 2.0 - 1);

  Eigen::Vector2d slot_top_wall_right = Eigen::Vector2d(
      slot_length + PARALLEL_SLOT_X_OFFSET, -slot_width / 2.0 - 1);

  // ego aabb
  cdl::AABB veh_aabb;
  double veh_x_buffer = 5.0;
  double veh_y_buffer = 4.0;

  veh_aabb.max_[0] = ego_pose.x + slot_length + veh_x_buffer;
  veh_aabb.max_[1] = ego_pose.y + slot_width / 2 +
                     slot_length * std::fabs(std::sin(ego_pose.theta)) +
                     veh_y_buffer;

  veh_aabb.min_[0] = ego_pose.x - slot_length - veh_x_buffer;
  veh_aabb.min_[1] = ego_pose.y - slot_width / 2 -
                     slot_length * std::fabs(std::sin(ego_pose.theta)) -
                     veh_y_buffer;

  // passage up bound
  Eigen::Vector2d passage_up_bound_left = Eigen::Vector2d(
      slot_length + passage_top_buffer, slot_width / 2.0 + passage_width);
  Eigen::Vector2d passage_up_bound_right =
      Eigen::Vector2d(slot_length + passage_top_buffer, -slot_width / 2.0);

  passage_up_bound_left[0] =
      std::max(passage_up_bound_left[0], veh_aabb.max_[0]);
  passage_up_bound_right[0] =
      std::max(passage_up_bound_right[0], veh_aabb.max_[0]);

  passage_up_bound_left[1] =
      std::max(passage_up_bound_left[1], veh_aabb.max_[1]);

  // passage bottom bound
  Eigen::Vector2d passage_bottom_bound_left = Eigen::Vector2d(
      -slot_length - passage_bottom_buffer, slot_width / 2.0 + passage_width);
  Eigen::Vector2d passage_bottom_bound_right =
      Eigen::Vector2d(-slot_length - passage_bottom_buffer, -slot_width / 2.0);

  passage_bottom_bound_left[0] =
      std::min(passage_bottom_bound_left[0], veh_aabb.min_[0]);
  passage_bottom_bound_right[0] =
      std::min(passage_bottom_bound_right[0], veh_aabb.min_[0]);

  passage_bottom_bound_left[1] =
      std::max(passage_bottom_bound_left[1], veh_aabb.max_[1]);

  // passage left bound
  Eigen::Vector2d passage_left_bound_upper = Eigen::Vector2d(
      slot_length + passage_top_buffer, slot_width / 2.0 + passage_width);
  Eigen::Vector2d passage_left_bound_lower = Eigen::Vector2d(
      -slot_length - passage_bottom_buffer, slot_width / 2.0 + passage_width);

  passage_left_bound_upper[0] =
      std::max(passage_left_bound_upper[0], veh_aabb.max_[0]);
  passage_left_bound_upper[1] =
      std::max(passage_left_bound_upper[1], veh_aabb.max_[1]);
  passage_left_bound_lower[0] =
      std::min(passage_left_bound_lower[0], veh_aabb.min_[0]);
  passage_left_bound_lower[1] =
      std::max(passage_left_bound_lower[1], veh_aabb.max_[1]);

  // passage right bound
  Eigen::Vector2d passage_right_bound_pt1 =
      Eigen::Vector2d(passage_up_bound_right[0], slot_top_wall_left[1]);
  Eigen::Vector2d passage_right_bound_pt2 = slot_top_wall_left;

  Eigen::Vector2d passage_right_bound_pt3 = slot_bottom_wall_left;
  Eigen::Vector2d passage_right_bound_pt4 =
      Eigen::Vector2d(passage_bottom_bound_left[0], slot_bottom_wall_left[1]);

  // add points
  SampleInLineSegment(right_wall_upper, right_wall_lower, false, &points);

  SampleInLineSegment(slot_bottom_wall_left, slot_bottom_wall_right, false,
                      &points);

  SampleInLineSegment(slot_top_wall_left, slot_top_wall_right, false, &points);

  SampleInLineSegment(passage_up_bound_left, passage_up_bound_right, false,
                      &points);

  SampleInLineSegment(passage_bottom_bound_left, passage_bottom_bound_right,
                      false, &points);

  SampleInLineSegment(passage_left_bound_upper, passage_left_bound_lower, false,
                      &points);

  SampleInLineSegment(passage_right_bound_pt1, passage_right_bound_pt2, false,
                      &points);
  SampleInLineSegment(passage_right_bound_pt3, passage_right_bound_pt4, false,
                      &points);

  return;
}

void VirtualWallDecider::LeftSideParallelVirtualWall(
    std::vector<Position2D>& points, const double slot_width,
    const double slot_length, const Pose2D& ego_pose, const Pose2D& end) {
  // width is the slot upper edge to a virtual wall
  const double passage_width = 8;
  const double passage_length_buffer = 15;

  // slot left wall
  Eigen::Vector2d slot_left_wall_upper =
      Eigen::Vector2d(slot_length + PARALLEL_SLOT_X_OFFSET,
                      slot_width / 2.0 + PARALLEL_SLOT_Y_OFFSET);

  Eigen::Vector2d slot_left_wall_lower = Eigen::Vector2d(
      -PARALLEL_SLOT_X_OFFSET, slot_width / 2.0 + PARALLEL_SLOT_Y_OFFSET);

  // slot bottom wall
  Eigen::Vector2d slot_bottom_wall_left =
      Eigen::Vector2d(-PARALLEL_SLOT_X_OFFSET, slot_width / 2.0 + 1);

  Eigen::Vector2d slot_bottom_wall_right =
      Eigen::Vector2d(-PARALLEL_SLOT_X_OFFSET, -slot_width / 2.0 + 1);

  // slot top wall
  Eigen::Vector2d slot_top_wall_left = Eigen::Vector2d(
      slot_length + PARALLEL_SLOT_X_OFFSET, slot_width / 2.0 + 1);

  Eigen::Vector2d slot_top_wall_right = Eigen::Vector2d(
      slot_length + PARALLEL_SLOT_X_OFFSET, -slot_width / 2.0 + 1);

  // ego aabb
  cdl::AABB veh_aabb;
  double veh_x_buffer = 5.0;
  double veh_y_buffer = 4.0;

  veh_aabb.max_[0] = ego_pose.x + slot_length + veh_x_buffer;
  veh_aabb.max_[1] = ego_pose.y + slot_width / 2 +
                     slot_length * std::fabs(std::sin(ego_pose.theta)) +
                     veh_y_buffer;

  veh_aabb.min_[0] = ego_pose.x - slot_length - veh_x_buffer;
  veh_aabb.min_[1] = ego_pose.y - slot_width / 2 -
                     slot_length * std::fabs(std::sin(ego_pose.theta)) -
                     veh_y_buffer;

  // passage up bound
  Eigen::Vector2d passage_up_bound_left =
      Eigen::Vector2d(slot_length + passage_length_buffer, slot_width / 2.0);
  Eigen::Vector2d passage_up_bound_right = Eigen::Vector2d(
      slot_length + passage_length_buffer, -slot_width / 2.0 - passage_width);

  passage_up_bound_left[0] =
      std::max(passage_up_bound_left[0], veh_aabb.max_[0]);
  passage_up_bound_right[0] =
      std::max(passage_up_bound_right[0], veh_aabb.max_[0]);

  passage_up_bound_left[1] =
      std::max(passage_up_bound_left[1], veh_aabb.max_[1]);
  passage_up_bound_right[1] =
      std::min(passage_up_bound_right[1], veh_aabb.min_[1]);

  // passage bottom bound
  double passage_bottom_buffer = 6.0;
  Eigen::Vector2d passage_bottom_bound_left =
      Eigen::Vector2d(-slot_length - passage_bottom_buffer, slot_width / 2.0);
  Eigen::Vector2d passage_bottom_bound_right = Eigen::Vector2d(
      -slot_length - passage_bottom_buffer, -slot_width / 2.0 - passage_width);

  passage_bottom_bound_left[0] =
      std::min(passage_bottom_bound_left[0], veh_aabb.min_[0]);
  passage_bottom_bound_right[0] =
      std::min(passage_bottom_bound_right[0], veh_aabb.min_[0]);

  passage_bottom_bound_left[1] =
      std::max(passage_bottom_bound_left[1], veh_aabb.max_[1]);
  passage_bottom_bound_right[1] =
      std::min(passage_bottom_bound_right[1], veh_aabb.min_[1]);

  // passage right bound
  Eigen::Vector2d passage_right_bound_upper = Eigen::Vector2d(
      slot_length + passage_length_buffer, -slot_width / 2.0 - passage_width);
  Eigen::Vector2d passage_right_bound_lower = Eigen::Vector2d(
      -slot_length - passage_length_buffer, -slot_width / 2.0 - passage_width);

  passage_right_bound_upper[0] =
      std::max(passage_right_bound_upper[0], veh_aabb.max_[0]);
  passage_right_bound_upper[1] =
      std::min(passage_right_bound_upper[1], veh_aabb.min_[1]);

  passage_right_bound_lower[0] =
      std::min(passage_right_bound_lower[0], veh_aabb.min_[0]);
  passage_right_bound_lower[1] =
      std::min(passage_right_bound_lower[1], veh_aabb.min_[1]);

  // passage left bound
  Eigen::Vector2d passage_left_bound_pt1 =
      Eigen::Vector2d(passage_up_bound_left[0], slot_top_wall_right[1]);
  Eigen::Vector2d passage_left_bound_pt2 = slot_top_wall_right;

  Eigen::Vector2d passage_left_bound_pt3 = slot_bottom_wall_right;
  Eigen::Vector2d passage_left_bound_pt4 =
      Eigen::Vector2d(passage_bottom_bound_left[0], slot_bottom_wall_right[1]);

  // add points
  SampleInLineSegment(slot_left_wall_upper, slot_left_wall_lower, false,
                      &points);

  SampleInLineSegment(slot_bottom_wall_left, slot_bottom_wall_right, false,
                      &points);

  SampleInLineSegment(slot_top_wall_left, slot_top_wall_right, false, &points);

  SampleInLineSegment(passage_up_bound_left, passage_up_bound_right, false,
                      &points);

  SampleInLineSegment(passage_bottom_bound_left, passage_bottom_bound_right,
                      false, &points);

  SampleInLineSegment(passage_right_bound_upper, passage_right_bound_lower,
                      false, &points);
  SampleInLineSegment(passage_left_bound_pt1, passage_left_bound_pt2, false,
                      &points);
  SampleInLineSegment(passage_left_bound_pt3, passage_left_bound_pt4, false,
                      &points);

  return;
}

void VirtualWallDecider::Init(const Pose2D& ego_pose) {
  passage_bound_ = VirtualWallBoundary(Position2D(ego_pose.x, ego_pose.y));
  GetUpLeftCoordinatePolygonByParam(&blind_local_box_, 15, 15, 15);

  return;
}

void VirtualWallDecider::GetVehicleBound() {
  double veh_upper_x = start_.x;
  double veh_lower_x = start_.x;
  double veh_left_y = start_.y;
  double veh_right_y = start_.y;
  for (int i = 0; i < ego_polygon_in_slot_.vertex_num; i++) {
    veh_upper_x = std::max(veh_upper_x, ego_polygon_in_slot_.vertexes[i].x);
    veh_lower_x = std::min(veh_lower_x, ego_polygon_in_slot_.vertexes[i].x);

    veh_left_y = std::max(veh_left_y, ego_polygon_in_slot_.vertexes[i].y);
    veh_right_y = std::min(veh_right_y, ego_polygon_in_slot_.vertexes[i].y);
  }

  double veh_buffer = 0.5;
  veh_boundary_.x_upper = veh_upper_x + veh_buffer;
  veh_boundary_.x_lower = veh_lower_x - veh_buffer;
  veh_boundary_.y_upper = veh_left_y + veh_buffer;
  veh_boundary_.y_lower = veh_right_y - veh_buffer;

  return;
}

}  // namespace planning