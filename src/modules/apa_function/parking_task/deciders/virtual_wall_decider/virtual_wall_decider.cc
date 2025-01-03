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
                                 const SlotRelativePosition slot_side) {
  start_ = ego_pose;
  end_ = end;

  GenerateVehPolygonInSlot(ego_pose);
  GenerateCarRelativePosition(ego_pose);
  ULFLocalPolygonToGlobal(&blind_global_box_, &blind_local_box_,
                          ego_pose);

  points.clear();

  if (slot_type == ParkSpaceType::VERTICAL ||
      slot_type == ParkSpaceType::SLANTING) {
    CalcVerticalVirtualWall(points, slot_width, slot_length, ego_pose, end);

    ILOG_INFO << "vertical slot virtual wall";
  } else {
    if (slot_side == SlotRelativePosition::RIGHT) {
      RightSideParallelVirtualWall(points, slot_width, slot_length, ego_pose,
                                   end);
    } else if (slot_side == SlotRelativePosition::LEFT) {
      LeftSideParallelVirtualWall(points, slot_width, slot_length, ego_pose,
                                  end);
    }

    ILOG_INFO << "parallel slot virtual wall";
  }

  return;
}

void VirtualWallDecider::SampleInLineSegment(const Eigen::Vector2d& start,
                                             const Eigen::Vector2d& end,
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

    points->emplace_back(Position2D(point.x(), point.y()));
  }

  if (!IsVirtualWallPointCollision(Position2D(point.x(), point.y()))) {
    points->emplace_back(Position2D(end.x(), end.y()));
  }

  return;
}

void VirtualWallDecider::GenerateCarRelativePosition(const Pose2D& ego_pose) {
  if (ego_pose.y > 0.2) {
    relative_position_ = VehRelativePosition::LEFT;
  } else if (ego_pose.y < -0.2) {
    relative_position_ = VehRelativePosition::RIGHT;
  } else {
    relative_position_ = VehRelativePosition::MIDDLE;
  }
  return;
}

void VirtualWallDecider::CalcVerticalVirtualWall(
    std::vector<Position2D>& points, const double slot_width,
    const double slot_length, const Pose2D& ego_pose, const Pose2D& end) {
  // width is the slot upper edge to a virtual wall
  const double channel_length = 40;
  const double lower_channel_length = (channel_length - slot_width) * 0.5;

  // right slot wall
  VirtualWallBoundary slot_boundary;
  slot_boundary.y_lower = -slot_width / 2.0 - SLOT_VIRTUAL_WALL_Y_OFFSET;
  slot_boundary.x_upper = slot_length - SLOT_VIRTUAL_WALL_X_OFFSET;

  // left slot wall
  slot_boundary.y_upper = slot_width / 2.0 + SLOT_VIRTUAL_WALL_Y_OFFSET;

  // slot back wall
  double lower_bound_x = -1.0;
  slot_boundary.x_lower = lower_bound_x;

  // lower
  VirtualWallBoundary tmp_channel_boundary;
  tmp_channel_boundary.x_lower = slot_length - SLOT_VIRTUAL_WALL_X_OFFSET;

  // channel up bound
  double veh_up_x = ego_pose.x;
  for (int i = 0; i < ego_polygon_in_slot_.vertex_num; i++) {
    veh_up_x = std::max(veh_up_x, ego_polygon_in_slot_.vertexes[i].x);
  }
  double car_buffer = 0.5;
  veh_up_x += car_buffer;

  double channel_up_bound_x = slot_length + 10.0;
  channel_up_bound_x = std::max(veh_up_x, channel_up_bound_x);
  tmp_channel_boundary.x_upper = channel_up_bound_x;

  // channel left/right bound
  double theta = IflyUnifyTheta(ego_pose.theta, M_PI);
  double channel_right_bound_y;
  double channel_left_bound_y;

  double tail_buffer = 5.5;
  double head_buffer = 7.0;

  if (relative_position_ == VehRelativePosition::RIGHT) {
    if (theta < 0.0) {
      channel_right_bound_y = ego_pose.y - head_buffer;
      channel_left_bound_y = slot_width / 2.0 + 5.0;
    } else if (theta > 0.0) {
      channel_right_bound_y = ego_pose.y - tail_buffer;
      channel_left_bound_y = slot_width / 2.0 + head_buffer;
    } else {
      channel_right_bound_y = -slot_width / 2.0 - lower_channel_length;
      channel_left_bound_y = slot_width / 2.0 + 10.0;
    }
  } else if (relative_position_ == VehRelativePosition::LEFT) {
    if (theta < 0.0) {
      channel_right_bound_y = -slot_width / 2 - head_buffer;
      channel_left_bound_y = ego_pose.y + tail_buffer;
    } else if (theta > 0.0) {
      channel_right_bound_y = -slot_width / 2 - tail_buffer;

      channel_left_bound_y = ego_pose.y + head_buffer;
    } else {
      channel_right_bound_y = -slot_width / 2.0 - 12.0;

      channel_left_bound_y = slot_width / 2 + 12.0;
    }
  } else {
    channel_right_bound_y = -slot_width / 2.0 - 12.0;
    channel_left_bound_y = slot_width / 2 + 12.0;
  }

  tmp_channel_boundary.y_lower = channel_right_bound_y;
  tmp_channel_boundary.y_upper = channel_left_bound_y;

  channel_bound_.Combine(tmp_channel_boundary);

  // add points
  // slot right bound
  SampleInLineSegment(
      Eigen::Vector2d(slot_boundary.x_lower, slot_boundary.y_lower),
      Eigen::Vector2d(slot_boundary.x_upper, slot_boundary.y_lower), &points);

  // slot left bound
  SampleInLineSegment(
      Eigen::Vector2d(slot_boundary.x_upper, slot_boundary.y_upper),
      Eigen::Vector2d(slot_boundary.x_lower, slot_boundary.y_upper), &points);

  // channel lower right boundary
  SampleInLineSegment(
      Eigen::Vector2d(slot_boundary.x_upper, slot_boundary.y_lower),
      Eigen::Vector2d(channel_bound_.x_lower, channel_bound_.y_lower), &points);

  // channel lower left boundary
  SampleInLineSegment(
      Eigen::Vector2d(slot_boundary.x_upper, slot_boundary.y_upper),
      Eigen::Vector2d(channel_bound_.x_lower, channel_bound_.y_upper), &points);

  // upper boundary
  SampleInLineSegment(
      Eigen::Vector2d(channel_bound_.x_upper, channel_bound_.y_upper),
      Eigen::Vector2d(channel_bound_.x_upper, channel_bound_.y_lower), &points);

  // right boundary
  SampleInLineSegment(
      Eigen::Vector2d(channel_bound_.x_upper, channel_bound_.y_lower),
      Eigen::Vector2d(channel_bound_.x_lower, channel_bound_.y_lower), &points);

  // left boundary
  SampleInLineSegment(
      Eigen::Vector2d(channel_bound_.x_upper, channel_bound_.y_upper),
      Eigen::Vector2d(channel_bound_.x_lower, channel_bound_.y_upper), &points);

  // slot lower bound
  SampleInLineSegment(
      Eigen::Vector2d(slot_boundary.x_lower, slot_boundary.y_upper),
      Eigen::Vector2d(slot_boundary.x_lower, slot_boundary.y_lower), &points);

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
        &points);
  }

  return;
}

#define PARALLEL_SLOT_X_OFFSET (7.0)
#define PARALLEL_SLOT_Y_OFFSET (1.0)
void VirtualWallDecider::RightSideParallelVirtualWall(
    std::vector<Position2D>& points, const double slot_width,
    const double slot_length, const Pose2D& ego_pose, const Pose2D& end) {
  // width is the slot upper edge to a virtual wall
  const double channel_width = 8;
  const double channel_top_buffer = 8.0;
  const double channel_bottom_buffer = 8.0;

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

  // channel up bound
  Eigen::Vector2d channel_up_bound_left = Eigen::Vector2d(
      slot_length + channel_top_buffer, slot_width / 2.0 + channel_width);
  Eigen::Vector2d channel_up_bound_right =
      Eigen::Vector2d(slot_length + channel_top_buffer, -slot_width / 2.0);

  channel_up_bound_left[0] =
      std::max(channel_up_bound_left[0], veh_aabb.max_[0]);
  channel_up_bound_right[0] =
      std::max(channel_up_bound_right[0], veh_aabb.max_[0]);

  channel_up_bound_left[1] =
      std::max(channel_up_bound_left[1], veh_aabb.max_[1]);

  // channel bottom bound
  Eigen::Vector2d channel_bottom_bound_left = Eigen::Vector2d(
      -slot_length - channel_bottom_buffer, slot_width / 2.0 + channel_width);
  Eigen::Vector2d channel_bottom_bound_right =
      Eigen::Vector2d(-slot_length - channel_bottom_buffer, -slot_width / 2.0);

  channel_bottom_bound_left[0] =
      std::min(channel_bottom_bound_left[0], veh_aabb.min_[0]);
  channel_bottom_bound_right[0] =
      std::min(channel_bottom_bound_right[0], veh_aabb.min_[0]);

  channel_bottom_bound_left[1] =
      std::max(channel_bottom_bound_left[1], veh_aabb.max_[1]);

  // channel left bound
  Eigen::Vector2d channel_left_bound_upper = Eigen::Vector2d(
      slot_length + channel_top_buffer, slot_width / 2.0 + channel_width);
  Eigen::Vector2d channel_left_bound_lower = Eigen::Vector2d(
      -slot_length - channel_bottom_buffer, slot_width / 2.0 + channel_width);

  channel_left_bound_upper[0] =
      std::max(channel_left_bound_upper[0], veh_aabb.max_[0]);
  channel_left_bound_upper[1] =
      std::max(channel_left_bound_upper[1], veh_aabb.max_[1]);
  channel_left_bound_lower[0] =
      std::min(channel_left_bound_lower[0], veh_aabb.min_[0]);
  channel_left_bound_lower[1] =
      std::max(channel_left_bound_lower[1], veh_aabb.max_[1]);

  // channel right bound
  Eigen::Vector2d channel_right_bound_pt1 =
      Eigen::Vector2d(channel_up_bound_right[0], slot_top_wall_left[1]);
  Eigen::Vector2d channel_right_bound_pt2 = slot_top_wall_left;

  Eigen::Vector2d channel_right_bound_pt3 = slot_bottom_wall_left;
  Eigen::Vector2d channel_right_bound_pt4 =
      Eigen::Vector2d(channel_bottom_bound_left[0], slot_bottom_wall_left[1]);

  // add points
  SampleInLineSegment(right_wall_upper, right_wall_lower, &points);

  SampleInLineSegment(slot_bottom_wall_left, slot_bottom_wall_right, &points);

  SampleInLineSegment(slot_top_wall_left, slot_top_wall_right, &points);

  SampleInLineSegment(channel_up_bound_left, channel_up_bound_right, &points);

  SampleInLineSegment(channel_bottom_bound_left, channel_bottom_bound_right,
                      &points);

  SampleInLineSegment(channel_left_bound_upper, channel_left_bound_lower,
                      &points);

  SampleInLineSegment(channel_right_bound_pt1, channel_right_bound_pt2,
                      &points);
  SampleInLineSegment(channel_right_bound_pt3, channel_right_bound_pt4,
                      &points);

  return;
}

void VirtualWallDecider::LeftSideParallelVirtualWall(
    std::vector<Position2D>& points, const double slot_width,
    const double slot_length, const Pose2D& ego_pose, const Pose2D& end) {
  // width is the slot upper edge to a virtual wall
  const double channel_width = 8;
  const double channel_length_buffer = 15;

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

  // channel up bound
  Eigen::Vector2d channel_up_bound_left =
      Eigen::Vector2d(slot_length + channel_length_buffer, slot_width / 2.0);
  Eigen::Vector2d channel_up_bound_right = Eigen::Vector2d(
      slot_length + channel_length_buffer, -slot_width / 2.0 - channel_width);

  channel_up_bound_left[0] =
      std::max(channel_up_bound_left[0], veh_aabb.max_[0]);
  channel_up_bound_right[0] =
      std::max(channel_up_bound_right[0], veh_aabb.max_[0]);

  channel_up_bound_left[1] =
      std::max(channel_up_bound_left[1], veh_aabb.max_[1]);
  channel_up_bound_right[1] =
      std::min(channel_up_bound_right[1], veh_aabb.min_[1]);

  // channel bottom bound
  double channel_bottom_buffer = 6.0;
  Eigen::Vector2d channel_bottom_bound_left =
      Eigen::Vector2d(-slot_length - channel_bottom_buffer, slot_width / 2.0);
  Eigen::Vector2d channel_bottom_bound_right = Eigen::Vector2d(
      -slot_length - channel_bottom_buffer, -slot_width / 2.0 - channel_width);

  channel_bottom_bound_left[0] =
      std::min(channel_bottom_bound_left[0], veh_aabb.min_[0]);
  channel_bottom_bound_right[0] =
      std::min(channel_bottom_bound_right[0], veh_aabb.min_[0]);

  channel_bottom_bound_left[1] =
      std::max(channel_bottom_bound_left[1], veh_aabb.max_[1]);
  channel_bottom_bound_right[1] =
      std::min(channel_bottom_bound_right[1], veh_aabb.min_[1]);

  // channel right bound
  Eigen::Vector2d channel_right_bound_upper = Eigen::Vector2d(
      slot_length + channel_length_buffer, -slot_width / 2.0 - channel_width);
  Eigen::Vector2d channel_right_bound_lower = Eigen::Vector2d(
      -slot_length - channel_length_buffer, -slot_width / 2.0 - channel_width);

  channel_right_bound_upper[0] =
      std::max(channel_right_bound_upper[0], veh_aabb.max_[0]);
  channel_right_bound_upper[1] =
      std::min(channel_right_bound_upper[1], veh_aabb.min_[1]);

  channel_right_bound_lower[0] =
      std::min(channel_right_bound_lower[0], veh_aabb.min_[0]);
  channel_right_bound_lower[1] =
      std::min(channel_right_bound_lower[1], veh_aabb.min_[1]);

  // channel left bound
  Eigen::Vector2d channel_left_bound_pt1 =
      Eigen::Vector2d(channel_up_bound_left[0], slot_top_wall_right[1]);
  Eigen::Vector2d channel_left_bound_pt2 = slot_top_wall_right;

  Eigen::Vector2d channel_left_bound_pt3 = slot_bottom_wall_right;
  Eigen::Vector2d channel_left_bound_pt4 =
      Eigen::Vector2d(channel_bottom_bound_left[0], slot_bottom_wall_right[1]);

  // add points
  SampleInLineSegment(slot_left_wall_upper, slot_left_wall_lower, &points);

  SampleInLineSegment(slot_bottom_wall_left, slot_bottom_wall_right, &points);

  SampleInLineSegment(slot_top_wall_left, slot_top_wall_right, &points);

  SampleInLineSegment(channel_up_bound_left, channel_up_bound_right, &points);

  SampleInLineSegment(channel_bottom_bound_left, channel_bottom_bound_right,
                      &points);

  SampleInLineSegment(channel_right_bound_upper, channel_right_bound_lower,
                      &points);
  SampleInLineSegment(channel_left_bound_pt1, channel_left_bound_pt2, &points);
  SampleInLineSegment(channel_left_bound_pt3, channel_left_bound_pt4, &points);

  return;
}

void VirtualWallDecider::Init(const Pose2D& ego_pose) {
  channel_bound_ = VirtualWallBoundary(Position2D(ego_pose.x, ego_pose.y));
  GetUpLeftCoordinatePolygonByParam(&blind_local_box_, 15, 15, 15);

  return;
}

}  // namespace planning