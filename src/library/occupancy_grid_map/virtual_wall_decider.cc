#include "virtual_wall_decider.h"
#include "pose2d.h"
#include "polygon_base.h"
#include "utils_math.h"

namespace planning {

void VirtualWallDecider::Process(const Pose2D& start, const Pose2D& end) {
  start_ = start;
  end_ = end;

  return;
}

#define SLOT_VIRTUAL_WALL_Y_OFFSET (0.5)
#define SLOT_VIRTUAL_WALL_X_OFFSET (2.0)
int VirtualWallDecider::GenerateVirtualWall(ParkObstacleList& obs_list,
                                            const double channel_length,
                                            const double channel_width,
                                            const double slot_width,
                                            const double slot_length,
                                            const Pose2D& ego_pose) {
  GenerateCarRelativePosition(ego_pose);

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
  channel_up_bound_x =
      ego_pose.x + 5.0 * std::fabs(std::cos(ego_pose.theta)) + 8.0;

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

  if (relative_position_ == CarSlotRelativePosition::car_is_right) {
    if (theta < 0.0) {
      channel_right_bound_y = ego_pose.y - 4.0 - 10.0;
      channel_left_bound_y = slot_width / 2.0 + 5.0;
    } else if (theta > 0.0) {
      channel_right_bound_y = ego_pose.y - 0.9 - 6.5;
      channel_left_bound_y = slot_width / 2.0 + 10.0;
    } else {
      channel_right_bound_y = -slot_width / 2.0 - lower_channel_length;
      channel_left_bound_y = slot_width / 2.0 + 10.0;
    }
  } else if (relative_position_ == CarSlotRelativePosition::car_is_left) {
    if (theta < 0.0) {
      channel_right_bound_y = -slot_width/2 - 4.0 - 10.0;
      channel_left_bound_y = ego_pose.y + 1.0 + 5.0;
    } else if (theta > 0.0) {
      channel_right_bound_y = -slot_width/2 - 0.9 - 10.0;

      channel_left_bound_y = ego_pose.y + 4.0 + 10.0;
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

  left_channel_lower =
      Eigen::Vector2d(slot_length - SLOT_VIRTUAL_WALL_X_OFFSET,
                      channel_left_bound_y);
  left_channel_upper = Eigen::Vector2d(
      slot_length + channel_width, channel_left_bound_y);

  // slot back wall
  double lower_bound_x = -0.8;
  Eigen::Vector2d back_wall_left = Eigen::Vector2d(
      lower_bound_x, -slot_width / 2.0 - SLOT_VIRTUAL_WALL_Y_OFFSET);

  Eigen::Vector2d back_wall_right = Eigen::Vector2d(
      lower_bound_x, slot_width / 2.0 + SLOT_VIRTUAL_WALL_Y_OFFSET);

  //
  Polygon2D polygon;
  std::vector<Polygon2D> convex_obs_list;

  //
  GenerateLineSegmentPolygon(&polygon, right_wall_start, right_wall_end);

  convex_obs_list.emplace_back(polygon);

  //
  GenerateLineSegmentPolygon(&polygon, left_wall_start, left_wall_end);

  convex_obs_list.emplace_back(polygon);

  //
  GenerateLineSegmentPolygon(&polygon, right_wall_end, right_channel_end);

  convex_obs_list.emplace_back(polygon);

  //
  GenerateLineSegmentPolygon(&polygon, left_wall_end, left_channel_end);

  convex_obs_list.emplace_back(polygon);

  //
  GenerateLineSegmentPolygon(&polygon, upper_channel_left,
                                upper_channel_right);

  convex_obs_list.emplace_back(polygon);

  //
  GenerateLineSegmentPolygon(&polygon, right_channel_lower,
                                right_channel_upper);

  convex_obs_list.emplace_back(polygon);

  //
  GenerateLineSegmentPolygon(&polygon, left_channel_lower,
                                left_channel_upper);

  convex_obs_list.emplace_back(polygon);

  //
  GenerateLineSegmentPolygon(&polygon, back_wall_left, back_wall_right);

  convex_obs_list.emplace_back(polygon);

  return 0;
}

int VirtualWallDecider::Process(std::vector<Position2D>& points,
                                const double channel_length,
                                const double channel_width,
                                const double slot_width,
                                const double slot_length,
                                const Pose2D& ego_pose, const Pose2D& end) {
  start_ = ego_pose;
  end_ = end;

  GenerateCarRelativePosition(ego_pose);

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

  // limit upper
  if (channel_up_bound_x > slot_length + 9.0) {
    channel_up_bound_x = slot_length+ 9.0;
  }

  // limit lower
  if (channel_up_bound_x < veh_up_x + 3.0) {
    channel_up_bound_x = veh_up_x + 3.0;
  }

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

  if (relative_position_ == CarSlotRelativePosition::car_is_right) {
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
  } else if (relative_position_ == CarSlotRelativePosition::car_is_left) {
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

  left_channel_lower =
      Eigen::Vector2d(slot_length - SLOT_VIRTUAL_WALL_X_OFFSET,
                      channel_left_bound_y);
  left_channel_upper = Eigen::Vector2d(
      slot_length + channel_width, channel_left_bound_y);

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

  return 0;
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
    relative_position_ = CarSlotRelativePosition::car_is_left;
  } else if (ego_pose.y < -0.2) {
    relative_position_ = CarSlotRelativePosition::car_is_right;
  } else {
    relative_position_ = CarSlotRelativePosition::car_is_middle;
  }
  return;
}

}  // namespace planning