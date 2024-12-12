#include "virtual_wall_decider.h"

#include <algorithm>
#include <cmath>

#include "aabb2d.h"
#include "collision_box2d.h"
#include "collision_detect_types.h"
#include "log_glog.h"
#include "polygon_base.h"
#include "pose2d.h"
#include "utils_math.h"
#include "apa_param_config.h"

namespace planning {

#define SLOT_VIRTUAL_WALL_Y_OFFSET (0.5)
#define SLOT_VIRTUAL_WALL_X_OFFSET (3.0)

void VirtualWallDecider::Process(std::vector<Position2D>& points,
                                const double channel_length,
                                const double channel_width,
                                const double slot_width,
                                const double slot_length,
                                const Pose2D& ego_pose, const Pose2D& end,
                                const ParkSpaceType slot_type,
                                const SlotRelativePosition slot_side) {
  start_ = ego_pose;
  end_ = end;

  GenerateCarRelativePosition(ego_pose);
  points.clear();

  if (slot_type == ParkSpaceType::VERTICAL ||
      slot_type == ParkSpaceType::SLANTING) {
    CalcVerticalVirtualWall(points, channel_length, channel_width, slot_width,
                            slot_length, ego_pose, end);

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

void VirtualWallDecider::SampleInLine(const Eigen::Vector2d& start,
                                      const Eigen::Vector2d& end,
                                      std::vector<Position2D>* points) {
  const Eigen::Vector2d line = end - start;
  const Eigen::Vector2d unit_line_vec = line.normalized();
  double len = line.norm();

  double s = 0.0;
  double ds = 0.2;

  Eigen::Vector2d point;
  while (s < len) {
    point = start + s * unit_line_vec;

    points->emplace_back(Position2D(point.x(), point.y()));

    s += ds;
  }

  points->emplace_back(Position2D(end.x(), end.y()));

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
    std::vector<Position2D>& points, const double channel_length,
    const double channel_width, const double slot_width,
    const double slot_length, const Pose2D& ego_pose, const Pose2D& end) {
  // width is the slot upper edge to a virtual wall
  const double lower_channel_length = (channel_length - slot_width) * 0.5;

  // right slot wall
  Eigen::Vector2d right_wall_start =
      Eigen::Vector2d(0, -slot_width / 2.0 - SLOT_VIRTUAL_WALL_Y_OFFSET);

  Eigen::Vector2d right_wall_end =
      Eigen::Vector2d(slot_length - SLOT_VIRTUAL_WALL_X_OFFSET,
                      -slot_width / 2.0 - SLOT_VIRTUAL_WALL_Y_OFFSET);

  // left slot wall
  Eigen::Vector2d left_wall_start =
      Eigen::Vector2d(0, slot_width / 2.0 + SLOT_VIRTUAL_WALL_Y_OFFSET);

  Eigen::Vector2d left_wall_end =
      Eigen::Vector2d(slot_length - SLOT_VIRTUAL_WALL_X_OFFSET,
                      slot_width / 2.0 + SLOT_VIRTUAL_WALL_Y_OFFSET);

  // right channel
  const Eigen::Vector2d right_channel_end =
      Eigen::Vector2d(slot_length - SLOT_VIRTUAL_WALL_X_OFFSET,
                      -slot_width / 2.0 - lower_channel_length);

  // left channel
  const Eigen::Vector2d left_channel_end =
      Eigen::Vector2d(slot_length - SLOT_VIRTUAL_WALL_X_OFFSET,
                      slot_width / 2.0 + lower_channel_length);

  // channel up bound
  double channel_up_bound_x;
  double veh_up_x = ego_pose.x + 5.0 * std::fabs(std::cos(ego_pose.theta));
  channel_up_bound_x = veh_up_x + 8.0;

  Eigen::Vector2d upper_channel_left = Eigen::Vector2d(
      channel_up_bound_x, slot_width / 2.0 + lower_channel_length);
  Eigen::Vector2d upper_channel_right = Eigen::Vector2d(
      channel_up_bound_x, -slot_width / 2.0 - lower_channel_length);

  // channel right virtual wall

  Eigen::Vector2d right_channel_lower;
  Eigen::Vector2d right_channel_upper;

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

  right_channel_lower = Eigen::Vector2d(
      slot_length - SLOT_VIRTUAL_WALL_X_OFFSET, channel_right_bound_y);
  right_channel_upper =
      Eigen::Vector2d(slot_length + channel_width, channel_right_bound_y);

  // channel left virtual wall
  Eigen::Vector2d left_channel_lower;
  Eigen::Vector2d left_channel_upper;

  left_channel_lower = Eigen::Vector2d(slot_length - SLOT_VIRTUAL_WALL_X_OFFSET,
                                       channel_left_bound_y);
  left_channel_upper =
      Eigen::Vector2d(slot_length + channel_width, channel_left_bound_y);

  // slot back wall
  double lower_bound_x = -1.0;
  Eigen::Vector2d back_wall_left = Eigen::Vector2d(
      lower_bound_x, -slot_width / 2.0 - SLOT_VIRTUAL_WALL_Y_OFFSET);

  Eigen::Vector2d back_wall_right = Eigen::Vector2d(
      lower_bound_x, slot_width / 2.0 + SLOT_VIRTUAL_WALL_Y_OFFSET);

  // add points
  SampleInLine(right_wall_start, right_wall_end, &points);

  SampleInLine(left_wall_start, left_wall_end, &points);

  SampleInLine(right_wall_end, right_channel_end, &points);

  SampleInLine(left_wall_end, left_channel_end, &points);

  SampleInLine(upper_channel_left, upper_channel_right, &points);

  SampleInLine(right_channel_lower, right_channel_upper, &points);

  SampleInLine(left_channel_lower, left_channel_upper, &points);

  SampleInLine(back_wall_left, back_wall_right, &points);

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
  SampleInLine(right_wall_upper, right_wall_lower, &points);

  SampleInLine(slot_bottom_wall_left, slot_bottom_wall_right, &points);

  SampleInLine(slot_top_wall_left, slot_top_wall_right, &points);

  SampleInLine(channel_up_bound_left, channel_up_bound_right, &points);

  SampleInLine(channel_bottom_bound_left, channel_bottom_bound_right, &points);

  SampleInLine(channel_left_bound_upper, channel_left_bound_lower, &points);

  SampleInLine(channel_right_bound_pt1, channel_right_bound_pt2, &points);
  SampleInLine(channel_right_bound_pt3, channel_right_bound_pt4, &points);

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
  SampleInLine(slot_left_wall_upper, slot_left_wall_lower, &points);

  SampleInLine(slot_bottom_wall_left, slot_bottom_wall_right, &points);

  SampleInLine(slot_top_wall_left, slot_top_wall_right, &points);

  SampleInLine(channel_up_bound_left, channel_up_bound_right, &points);

  SampleInLine(channel_bottom_bound_left, channel_bottom_bound_right, &points);

  SampleInLine(channel_right_bound_upper, channel_right_bound_lower, &points);
  SampleInLine(channel_left_bound_pt1, channel_left_bound_pt2, &points);
  SampleInLine(channel_left_bound_pt3, channel_left_bound_pt4, &points);

  return;
}

}  // namespace planning