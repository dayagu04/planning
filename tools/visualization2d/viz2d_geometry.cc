#include "viz2d_geometry.h"

#include "log_glog.h"
#include "pose2d.h"
#include "transform2d.h"

namespace planning {

int viz2d_draw_local_dash_line(viz2d_image *viz2d, const Position2D *start,
                               const Position2D *end, viz2d_color color_index,
                               int width) {
  CvPoint pt1, pt2;
  CvScalar color;
  int ret;
  Position2D next;

  ret = viz2d_get_color(&color, color_index);

  // 虚线段长度6米
  // 虚线间隔6米
  double dist = CalcPointDist(start, end);

  // 2个端点距离小，画一半
  if (dist <= 12.0) {
    next.x = (start->x + end->x) / 2;
    next.y = (start->y + end->y) / 2;

    viz2d_get_index(viz2d, &pt1, start->x, start->y);
    viz2d_get_index(viz2d, &pt2, next.x, next.y);
    cvLine(viz2d->image, pt1, pt2, color, width, CV_AA, 0);
  } else {
    int n = std::floor(dist / 12.0);

    ad_common::math::Vec2d dir(end->x - start->x, end->y - start->y);
    dir.Normalize();

    Position2D cur;
    for (size_t i = 0; i < n; i++) {
      cur.x = start->x + 12 * i * dir.x();
      cur.y = start->y + 12 * i * dir.y();

      next.x = start->x + (12 * i + 6) * dir.x();
      next.y = start->y + (12 * i + 6) * dir.y();

      viz2d_get_index(viz2d, &pt1, start->x, start->y);
      viz2d_get_index(viz2d, &pt2, next.x, next.y);
      cvLine(viz2d->image, pt1, pt2, color, width, CV_AA, 0);
    }
  }

  return 0;
}

int viz2d_draw_polygon(viz2d_image *viz2d, const Polygon2D *poly,
                       const Pose2D *ref_pose, viz2d_color color_index) {
  int i;
  CvPoint pt1, pt2;
  CvScalar color;
  Position2D poly_pts[MAX_POLYGON_VERTEX_NUM];
  Position2D poly_pts_local[MAX_POLYGON_VERTEX_NUM];
  int ret;

  if (poly->vertex_num < 2) return 1;

  ret = viz2d_get_color(&color, color_index);

  for (i = 0; i < poly->vertex_num; i++) {
    poly_pts[i].x = poly->vertexes[i].x;
    poly_pts[i].y = poly->vertexes[i].y;

    CvtPosGlobalToLocal(&poly_pts_local[i], &poly_pts[i], ref_pose);
  }

  for (i = 0; i < poly->vertex_num; i++) {
    viz2d_get_index(viz2d, &pt1, poly_pts_local[i].x, poly_pts_local[i].y);
    viz2d_get_index(viz2d, &pt2,
                    poly_pts_local[(i == poly->vertex_num - 1) ? 0 : i + 1].x,
                    poly_pts_local[(i == poly->vertex_num - 1) ? 0 : i + 1].y);
    cvLine(viz2d->image, pt1, pt2, color, 1, CV_AA, 0);
  }

  return 1;
}

int cv_draw_polygon(viz2d_image *viz2d, const Polygon2D *poly,
                    const Pose2D *ref_pose, viz2d_color color_index, int width,
                    const bool ref_pose_is_map_ref_frame) {
  int i;
  CvPoint pt1, pt2;
  CvScalar color;
  Position2D poly_pts[MAX_POLYGON_VERTEX_NUM];
  Position2D poly_pts_local[MAX_POLYGON_VERTEX_NUM];
  int ret;

  if (poly->vertex_num < 2) return 1;

  ret = viz2d_get_color(&color, color_index);

  for (i = 0; i < poly->vertex_num; i++) {
    poly_pts[i].x = poly->vertexes[i].x;
    poly_pts[i].y = poly->vertexes[i].y;

    CvtPosGlobalToLocal(&poly_pts_local[i], &poly_pts[i], ref_pose);

    // ILOG_INFO << "global " << poly_pts[i].x << " " << poly_pts[i].y;
    // ILOG_INFO << "local " << poly_pts_local[i].x << " " <<
    // poly_pts_local[i].y;
  }

  for (i = 0; i < poly->vertex_num; i++) {
    if (!ref_pose_is_map_ref_frame) {
      viz2d_get_index(viz2d, &pt1, poly_pts_local[i].x, poly_pts_local[i].y);
      viz2d_get_index(
          viz2d, &pt2,
          poly_pts_local[(i == poly->vertex_num - 1) ? 0 : i + 1].x,
          poly_pts_local[(i == poly->vertex_num - 1) ? 0 : i + 1].y);
    } else {
      viz2d_get_index(viz2d, &pt1, poly_pts[i].x, poly_pts[i].y);
      viz2d_get_index(viz2d, &pt2,
                      poly_pts[(i == poly->vertex_num - 1) ? 0 : i + 1].x,
                      poly_pts[(i == poly->vertex_num - 1) ? 0 : i + 1].y);
    }
    cvLine(viz2d->image, pt1, pt2, color, width, CV_AA, 0);
  }

  return 1;
}

int viz2d_draw_line(viz2d_image *viz2d, const Position2D *start,
                    const Position2D *end, const Pose2D *ref_pose,
                    viz2d_color color_index, int width) {
  int i;
  CvPoint pt1, pt2;
  CvScalar color;
  Position2D poly_pts_local[2];
  int ret;

  ret = viz2d_get_color(&color, color_index);

  CvtPosGlobalToLocal(&poly_pts_local[0], start, ref_pose);
  CvtPosGlobalToLocal(&poly_pts_local[1], end, ref_pose);

  viz2d_get_index(viz2d, &pt1, poly_pts_local[0].x, poly_pts_local[0].y);
  viz2d_get_index(viz2d, &pt2, poly_pts_local[1].x, poly_pts_local[1].y);
  cvLine(viz2d->image, pt1, pt2, color, width, CV_AA, 0);

  return 0;
}

int viz2d_draw_line(viz2d_image *viz2d, const Pose2D *start, const Pose2D *end,
                    const Pose2D *ref_pose, viz2d_color color_index,
                    int width) {
  int i;
  CvPoint pt1, pt2;
  CvScalar color;
  Pose2D poly_pts_local[2];
  int ret;

  ret = viz2d_get_color(&color, color_index);

  CvtPoseGlobalToLocal(&poly_pts_local[0], start, ref_pose);
  CvtPoseGlobalToLocal(&poly_pts_local[1], end, ref_pose);

  viz2d_get_index(viz2d, &pt1, poly_pts_local[0].x, poly_pts_local[0].y);
  viz2d_get_index(viz2d, &pt2, poly_pts_local[1].x, poly_pts_local[1].y);
  cvLine(viz2d->image, pt1, pt2, color, width, CV_AA, 0);

  return 0;
}

int viz2d_draw_local_line(viz2d_image *viz2d, const Position2D *start,
                          const Position2D *end, viz2d_color color_index,
                          int width) {
  int i;
  CvPoint pt1, pt2;
  CvScalar color;
  int ret;

  ret = viz2d_get_color(&color, color_index);

  double dist;
  dist = CalcPointDist(end, start);

  if (dist < 6) {
    viz2d_get_index(viz2d, &pt1, start->x, start->y);
    viz2d_get_index(viz2d, &pt2, end->x, end->y);
    cvLine(viz2d->image, pt1, pt2, color, width, CV_AA, 0);
    return 0;
  }

  // 2米插值
  int n = std::floor(dist / 2.0);
  if (n <= 0) {
    return 0;
  }

  ad_common::math::Vec2d dir(end->x - start->x, end->y - start->y);
  dir.Normalize();

  Position2D cur;
  cur = *start;
  Position2D next;

  double accumalte_dist = 0;

  for (size_t i = 1; i <= n; i++) {
    next.x = cur.x + 2 * dir.x();
    next.y = cur.y + 2 * dir.y();

    viz2d_get_index(viz2d, &pt1, cur.x, cur.y);
    viz2d_get_index(viz2d, &pt2, next.x, next.y);
    cvLine(viz2d->image, pt1, pt2, color, width, CV_AA, 0);

    cur = next;
    accumalte_dist += 2.0;
  }

  if (accumalte_dist < dist) {
    viz2d_get_index(viz2d, &pt1, cur.x, cur.y);
    viz2d_get_index(viz2d, &pt2, end->x, end->y);
    cvLine(viz2d->image, pt1, pt2, color, width, CV_AA, 0);
  }

  return 0;
}

int viz2d_draw_direction(viz2d_image *viz2d, const Pose2D *start, double len,
                         const Pose2D *ref_pose, viz2d_color color_index,
                         int width) {
  Pose2D end;
  end.x = start->x + len * std::cos(start->theta);
  end.y = start->y + len * std::sin(start->theta);

  end.theta = start->theta;

  viz2d_draw_line(viz2d, start, &end, ref_pose, color_index, width);

  // left
  Pose2D local;
  Pose2D global;

  local.x = 1.0;
  local.y = -1.0;

  Transform2d tf;
  tf.SetBasePose(end);

  tf.RUFLocalPoseToGlobal(&global, local);

  viz2d_draw_line(viz2d, &end, &global, ref_pose, color_index, width);

  // right

  local.x = -1.0;
  local.y = -1.0;

  tf.RUFLocalPoseToGlobal(&global, local);

  viz2d_draw_line(viz2d, &end, &global, ref_pose, color_index, width);

  return 0;
}

int viz2d_draw_circle_wrapper(viz2d_image *viz2d,
                              const Position2D *circle_center,
                              const Pose2D *base_pose, viz2d_color color_index,
                              int radius, bool filled,
                              const bool ref_pose_is_map_ref_frame) {
  int ret;
  CvPoint cv_center;
  CvScalar color;

  if (circle_center == nullptr || base_pose == nullptr || viz2d == nullptr) {
    return -1;
  }

  Position2D local;

  ret = viz2d_get_color(&color, color_index);

  if (ref_pose_is_map_ref_frame) {
    local = *circle_center;
  } else {
    CvtPosGlobalToLocal(&local, circle_center, base_pose);
  }

  ret = viz2d_get_index(viz2d, &cv_center, local.x, local.y);

  int thickness = 1;
  if (filled) {
    thickness = -1;
  }
  cvCircle(viz2d->image, cv_center, radius, color, thickness, CV_AA, 0);
  return 0;
}

int viz2d_draw_grid(viz2d_image *viz2d, double left, double right, double front,
                    double back, const Pose2D *base_pose,
                    viz2d_color color_index, int line_width) {
  int ret;
  CvPoint point1, point2;

  CvScalar color;

  if (base_pose == nullptr || viz2d == nullptr) {
    return -1;
  }

  ret = viz2d_get_color(&color, color_index);

  Position2D left_point;
  left_point.x = -left;
  left_point.y = -back;

  Position2D right_point;
  right_point.x = right;
  right_point.y = -back;

  double delta_y = 10;
  double delta_x = 10;

  while (left_point.y < front) {
    ret = viz2d_get_index(viz2d, &point1, left_point.x, left_point.y);

    ret = viz2d_get_index(viz2d, &point2, right_point.x, right_point.y);

    cvLine(viz2d->image, point1, point2, color, line_width, CV_AA, 0);

    left_point.y += delta_y;
    right_point.y += delta_y;
  }

  Position2D front_point;
  front_point.x = -left;
  front_point.y = front;

  Position2D back_point;
  back_point.x = -left;
  back_point.y = -back;

  while (front_point.x < right) {
    ret = viz2d_get_index(viz2d, &point1, front_point.x, front_point.y);

    ret = viz2d_get_index(viz2d, &point2, back_point.x, back_point.y);

    cvLine(viz2d->image, point1, point2, color, line_width, CV_AA, 0);

    front_point.x += delta_x;
    back_point.x += delta_x;
  }

  return 1;
}

int viz_draw_virtual_wall(Pose2D *wall_pose, viz2d_image *viz2d,
                          const Pose2D *veh_pose, int perception_id,
                          viz2d_color wall_color) {
  Polygon2D stop_wall_local;
  Polygon2D stop_wall_global;

  /**
   * @brief
   *
   *
   *
   *                      y
   *                           |
   *                           |
   *            2              |             1
   *                           |
   *                           |
   *                           |             0
   *            3              ----------------->  x
   * @note
   * @retval None
   */

  double wall_len = 5.0;
  double wall_width = 0.4;

  stop_wall_local.vertexes[0].x = wall_len / 2;
  stop_wall_local.vertexes[0].y = 0;

  stop_wall_local.vertexes[1].x = wall_len / 2;
  stop_wall_local.vertexes[1].y = wall_width;

  stop_wall_local.vertexes[2].x = -wall_len / 2;
  stop_wall_local.vertexes[2].y = wall_width;

  stop_wall_local.vertexes[3].x = -wall_len / 2;
  stop_wall_local.vertexes[3].y = 0;

  stop_wall_local.vertex_num = 4;

  RULocalPolygonToGlobal(&stop_wall_global, &stop_wall_local, wall_pose);

  viz2d_draw_filled_polygon(viz2d, &stop_wall_global, wall_color, veh_pose,
                            false);

  return 0;
}

int viz_draw_box_in_cv_frame(viz2d_image *viz2d, const CvPoint *center,
                             double height, double length, viz2d_color color,
                             bool fill) {
  CvScalar line_color;
  CvScalar box_fill_color;
  CvPoint points[4];
  CvPoint point;
  uint32_t i;
  int size;
  int ret;

  /**
   * @brief
   * @note
   * @retval None
   *
   *
   *
   *                                        x
   *              ------------------------>
   *              |
   *              |
   *              |
   *              |         -----------   1
   *              |        |          \
   *              |        |    *     \
   *              |        |--------- \   0
   *              |
   *        y     V
   */

  point.x = center->x + length / 2;
  point.y = center->y + height / 2;
  points[0] = point;

  point.x = center->x + length / 2;
  point.y = center->y - height / 2;
  points[1] = point;

  point.x = center->x - length / 2;
  point.y = center->y - height / 2;
  points[2] = point;

  point.x = center->x - length / 2;
  point.y = center->y + height / 2;
  points[3] = point;

  viz2d_get_color(&line_color, color);

  if (fill) {
    viz2d_get_color(&box_fill_color, viz2d_colors_black_gray);
    cvFillConvexPoly(viz2d->image, points, 4, box_fill_color, CV_AA, 0);
  }

  for (size_t i = 0; i < 4; i++) {
    if (i == 3) {
      cvLine(viz2d->image, points[i], points[0], line_color, 1, CV_AA, 0);
    } else {
      cvLine(viz2d->image, points[i], points[i + 1], line_color, 1, CV_AA, 0);
    }
  }

  return 0;
}

#define SLOT_VIRTUAL_WALL_LATERAL_OFFSET (1.0)
#define SLOT_VIRTUAL_WALL_LON_OFFSET (2.0)
const int GenerateVirtualWall(std::vector<Polygon2D> &obs_list,
                              const double channel_length,
                              const double channel_width,
                              const double slot_width,
                              const double slot_length) {
  // width is the slot upper edge to a virtual wall
  const double lower_channel_length = (channel_length - slot_width) * 0.5;

  // right slot wall
  Eigen::Vector2d right_wall_start =
      Eigen::Vector2d(slot_width / 2.0 + SLOT_VIRTUAL_WALL_LATERAL_OFFSET, 0);

  Eigen::Vector2d right_wall_end =
      Eigen::Vector2d(slot_width / 2.0 + SLOT_VIRTUAL_WALL_LATERAL_OFFSET,
                      slot_length - SLOT_VIRTUAL_WALL_LON_OFFSET);

  // left slot wall
  Eigen::Vector2d left_wall_start =
      Eigen::Vector2d(-slot_width / 2.0 - SLOT_VIRTUAL_WALL_LATERAL_OFFSET, 0);

  Eigen::Vector2d left_wall_end =
      Eigen::Vector2d(-slot_width / 2.0 - SLOT_VIRTUAL_WALL_LATERAL_OFFSET,
                      slot_length - SLOT_VIRTUAL_WALL_LON_OFFSET);

  // right channel
  const Eigen::Vector2d right_channel_end =
      Eigen::Vector2d(slot_width / 2.0 + lower_channel_length,
                      slot_length - SLOT_VIRTUAL_WALL_LON_OFFSET);

  // left channel
  const Eigen::Vector2d left_channel_end =
      Eigen::Vector2d(-slot_width / 2.0 - lower_channel_length,
                      slot_length - SLOT_VIRTUAL_WALL_LON_OFFSET);

  // upper channel
  const Eigen::Vector2d upper_channel_left = Eigen::Vector2d(
      -slot_width / 2.0 - lower_channel_length, slot_length + channel_width);
  const Eigen::Vector2d upper_channel_right = Eigen::Vector2d(
      slot_width / 2.0 + lower_channel_length, slot_length + channel_width);

  // channel right virtual wall
  const Eigen::Vector2d right_channel_lower =
      Eigen::Vector2d(slot_width / 2.0 + lower_channel_length,
                      slot_length - SLOT_VIRTUAL_WALL_LON_OFFSET);
  const Eigen::Vector2d right_channel_upper = Eigen::Vector2d(
      slot_width / 2.0 + lower_channel_length, slot_length + channel_width);

  // channel left virtual wall
  const Eigen::Vector2d left_channel_lower =
      Eigen::Vector2d(-slot_width / 2.0 - lower_channel_length,
                      slot_length - SLOT_VIRTUAL_WALL_LON_OFFSET);
  const Eigen::Vector2d left_channel_upper = Eigen::Vector2d(
      -slot_width / 2.0 - lower_channel_length, slot_length + channel_width);

  // slot back wall
  Eigen::Vector2d back_wall_left =
      Eigen::Vector2d(slot_width / 2.0 + SLOT_VIRTUAL_WALL_LATERAL_OFFSET, 0);

  Eigen::Vector2d back_wall_right =
      Eigen::Vector2d(-slot_width / 2.0 - SLOT_VIRTUAL_WALL_LATERAL_OFFSET, 0);

  Polygon2D polygon;
  GenerateLineSegmentPolygon(&polygon, right_wall_start, right_wall_end);

  obs_list.emplace_back(polygon);

  GenerateLineSegmentPolygon(&polygon, left_wall_start, left_wall_end);

  obs_list.emplace_back(polygon);

  GenerateLineSegmentPolygon(&polygon, right_wall_end, right_channel_end);

  obs_list.emplace_back(polygon);

  GenerateLineSegmentPolygon(&polygon, left_wall_end, left_channel_end);

  obs_list.emplace_back(polygon);

  GenerateLineSegmentPolygon(&polygon, upper_channel_left, upper_channel_right);

  obs_list.emplace_back(polygon);

  GenerateLineSegmentPolygon(&polygon, right_channel_lower,
                             right_channel_upper);

  obs_list.emplace_back(polygon);

  GenerateLineSegmentPolygon(&polygon, left_channel_lower, left_channel_upper);

  obs_list.emplace_back(polygon);

  GenerateLineSegmentPolygon(&polygon, back_wall_left, back_wall_right);

  obs_list.emplace_back(polygon);

  return 0;
}

}  // namespace planning
