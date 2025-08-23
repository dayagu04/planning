#include "polygon_base.h"

#include <algorithm>

#include "log_glog.h"
#include "src/modules/apa_function/apa_param_config.h"
#include "transform2d.h"
#include <array>

namespace planning {

int InitPolygon(Polygon2D *polygon) {
  int i;

  if (polygon == nullptr) {
    return -1;
  }

  polygon->shape = PolygonShape::multi_edge;
  polygon->vertex_num = 0;
  polygon->radius = 0.0;
  polygon->center_pt.x = 0.0;
  polygon->center_pt.y = 0.0;

  for (i = 0; i < MAX_POLYGON_VERTEX_NUM; i++) {
    polygon->vertexes[i].x = 0.0;
    polygon->vertexes[i].y = 0.0;
  }

  return 0;
}

int PolygonCopy(Polygon2D *des_poly, const Polygon2D *src_poly) {
  int i;

  if (des_poly == nullptr || src_poly == nullptr) {
    return -1;
  }

  des_poly->shape = src_poly->shape;
  des_poly->vertex_num = src_poly->vertex_num;
  des_poly->radius = src_poly->radius;
  des_poly->center_pt = src_poly->center_pt;

  for (i = 0; i < des_poly->vertex_num; i++) {
    des_poly->vertexes[i] = src_poly->vertexes[i];
  }

  return 1;
}

void UpdatePolygonValue(Polygon2D *polygon, const Pose2D *center_pose,
                        bool use_center_pose, bool radius_known, double radius) {
  int i;
  double tmp_dist;

  if (polygon == nullptr) {
    return;
  }

  switch (polygon->shape) {
    case PolygonShape::point:
      polygon->center_pt = polygon->vertexes[0];

      polygon->radius = 0;
      break;
    case PolygonShape::line_segment:
      polygon->center_pt.x = polygon->vertexes[0].x;
      polygon->center_pt.x += polygon->vertexes[1].x;
      polygon->center_pt.x /= 2;

      polygon->center_pt.y = polygon->vertexes[0].y;
      polygon->center_pt.y += polygon->vertexes[1].y;
      polygon->center_pt.y /= 2;

      if (!radius_known) {
        polygon->radius =
            CalcPointDist(&polygon->vertexes[0], &polygon->vertexes[1]) / 2.0;
      } else {
        polygon->radius = radius;
      }
      break;
    case PolygonShape::box:
      polygon->center_pt.x =
          (polygon->vertexes[0].x + polygon->vertexes[2].x) * 0.5;

      polygon->center_pt.y =
          (polygon->vertexes[0].y + polygon->vertexes[2].y) * 0.5;

      if (!radius_known) {
        polygon->radius =
            CalcPointDist(&polygon->vertexes[0], &polygon->vertexes[2]) / 2.0;
      } else {
        polygon->radius = radius;
      }
      break;
    case PolygonShape::multi_edge:
      if (center_pose != nullptr && use_center_pose) {
        polygon->center_pt.x = center_pose->x;
        polygon->center_pt.y = center_pose->y;
      } else {
        double x = 0.0;
        double y = 0.0;
        for (i = 0; i < polygon->vertex_num; i++) {
          x += polygon->vertexes[i].x;
          y += polygon->vertexes[i].y;
        }

        polygon->center_pt.x = x / polygon->vertex_num;
        polygon->center_pt.y = y / polygon->vertex_num;
      }

      if (!radius_known) {
        polygon->radius = 0.0;
        for (i = 0; i < polygon->vertex_num; i++) {
          tmp_dist = CalcPointDist(&polygon->center_pt, &polygon->vertexes[i]);
          polygon->radius = std::fmax(polygon->radius, tmp_dist);
        }
      } else {
        polygon->radius = radius;
      }
      break;

    default:
      break;
  }

  return;
}

int GenerateLineSegmentPolygon(Polygon2D *polygon, const Position2D &start,
                               const Position2D &end) {
  if (polygon == nullptr) {
    return 0;
  }

  polygon->vertex_num = 2;
  polygon->shape = PolygonShape::line_segment;

  polygon->vertexes[0] = start;
  polygon->vertexes[1] = end;

  polygon->center_pt.x = (start.x + end.x) * 0.5;
  polygon->center_pt.y = (start.y + end.y) * 0.5;

  polygon->radius = CalcPointDist(&start, &end) / 2.0;
  polygon->min_tangent_radius = 0.0;

  return 0;
}

int GenerateLineSegmentPolygon(Polygon2D *polygon, const Eigen::Vector2d &start,
                               const Eigen::Vector2d &end) {
  if (polygon == nullptr) {
    return 0;
  }

  polygon->vertex_num = 2;
  polygon->shape = PolygonShape::line_segment;

  polygon->vertexes[0].x = start[0];
  polygon->vertexes[0].y = start[1];
  polygon->vertexes[1].x = end[0];
  polygon->vertexes[1].y = end[1];

  polygon->center_pt.x = (start[0] + end[0]) * 0.5;
  polygon->center_pt.y = (start[1] + end[1]) * 0.5;

  polygon->radius =
      CalcPointDist(&polygon->vertexes[0], &polygon->vertexes[1]) / 2.0;
  polygon->min_tangent_radius = 0.0;

  return 0;
}

int GeneratePolygonByAABB(Polygon2D *polygon, const cdl::AABB &box) {
  if (polygon == nullptr) {
    return 0;
  }

  double x_diff = box.width();
  double y_diff = box.height();
  double point_error = 0.001;

  // point
  if (x_diff < point_error && y_diff < point_error) {
    polygon->vertex_num = 1;
    polygon->shape = PolygonShape::point;

    polygon->vertexes[0].x = box.min_[0];
    polygon->vertexes[0].y = box.min_[1];

    polygon->center_pt = polygon->vertexes[0];

    polygon->radius = 0;
    polygon->min_tangent_radius = 0;
  } else if (x_diff < point_error || y_diff < point_error) {
    polygon->vertex_num = 2;
    polygon->shape = PolygonShape::line_segment;

    polygon->vertexes[0].x = box.min_[0];
    polygon->vertexes[0].y = box.min_[1];
    polygon->vertexes[1].x = box.max_[0];
    polygon->vertexes[1].y = box.max_[1];

    polygon->center_pt.x =
        (polygon->vertexes[0].x + polygon->vertexes[1].x) * 0.5;
    polygon->center_pt.y =
        (polygon->vertexes[0].y + polygon->vertexes[1].y) * 0.5;

    polygon->radius =
        CalcPointDist(&polygon->vertexes[0], &polygon->vertexes[1]) / 2.0;
    polygon->min_tangent_radius = 0.0;
  } else {
    polygon->vertex_num = 4;
    polygon->shape = PolygonShape::box;

    // right up
    polygon->vertexes[0].x = box.max_(0);
    polygon->vertexes[0].y = box.min_(1);

    // left up
    polygon->vertexes[1].x = box.max_(0);
    polygon->vertexes[1].y = box.max_(1);

    // left down
    polygon->vertexes[2].x = box.min_(0);
    polygon->vertexes[2].y = box.max_(1);

    // right down
    polygon->vertexes[3].x = box.min_(0);
    polygon->vertexes[3].y = box.min_(1);

    UpdatePolygonValue(polygon, nullptr, false, false, POLYGON_MAX_RADIUS);

    double height = box.height();
    double width = box.width();
    polygon->min_tangent_radius = std::min(height / 2, width / 2);
  }

  return 0;
}

int GeneratePolygonByPoint(Polygon2D *polygon, const Position2D &point) {
  if (polygon == nullptr) {
    return 0;
  }

  polygon->vertex_num = 1;
  polygon->shape = PolygonShape::point;

  polygon->vertexes[0] = point;

  polygon->center_pt = point;

  polygon->radius = 0;
  polygon->min_tangent_radius = 0;

  return 0;
}

int GeneratePolygonByPoint(Polygon2D *polygon, const Eigen::Vector2d &point) {
  if (polygon == nullptr) {
    return 0;
  }

  polygon->vertex_num = 1;
  polygon->shape = PolygonShape::point;

  polygon->vertexes[0].x = point[0];
  polygon->vertexes[0].y = point[1];

  polygon->center_pt = polygon->vertexes[0];

  polygon->radius = 0;
  polygon->min_tangent_radius = 0;

  return 0;
}

int GenerateRectPolygon(Polygon2D *polygon, double min_x, double min_y,
                        double max_x, double max_y) {
  if (polygon == nullptr) {
    return 0;
  }

  polygon->vertex_num = 4;
  polygon->shape = PolygonShape::box;

  // right up
  polygon->vertexes[0].x = max_x;
  polygon->vertexes[0].y = max_y;

  // left up
  polygon->vertexes[1].x = min_x;
  polygon->vertexes[1].y = max_y;

  // left down
  polygon->vertexes[2].x = min_x;
  polygon->vertexes[2].y = min_y;

  // right down
  polygon->vertexes[3].x = max_x;
  polygon->vertexes[3].y = min_y;

  UpdatePolygonValue(polygon, nullptr, false, false, POLYGON_MAX_RADIUS);

  double dy = max_y - min_y;
  double dx = max_x - min_x;
  polygon->min_tangent_radius = std::min(dy / 2, dx / 2);

  return 1;
}

int GenerateUpLeftFrameBox(Polygon2D *polygon, double min_x, double min_y,
                           double max_x, double max_y) {
  if (polygon == nullptr) {
    return 0;
  }

  polygon->vertex_num = 4;
  polygon->shape = PolygonShape::box;

  // right up
  polygon->vertexes[0].x = max_x;
  polygon->vertexes[0].y = min_y;

  // left up
  polygon->vertexes[1].x = max_x;
  polygon->vertexes[1].y = max_y;

  // left down
  polygon->vertexes[2].x = min_x;
  polygon->vertexes[2].y = max_y;

  // right down
  polygon->vertexes[3].x = min_x;
  polygon->vertexes[3].y = min_y;

  UpdatePolygonValue(polygon, nullptr, false, false, POLYGON_MAX_RADIUS);

  double dy = max_y - min_y;
  double dx = max_x - min_x;
  polygon->min_tangent_radius = std::min(dy / 2, dx / 2);

  return 1;
}

void GeneratePolygonByPoints(Polygon2D *polygon,
                             const std::vector<Position2D> &points) {
  if (polygon == nullptr) {
    return;
  }

  polygon->vertex_num = static_cast<int>(points.size());
  polygon->vertex_num = std::min(MAX_POLYGON_VERTEX_NUM, polygon->vertex_num);

  switch (polygon->vertex_num) {
    case 1:
      polygon->shape = PolygonShape::point;
      break;
    case 2:
      polygon->shape = PolygonShape::line_segment;
      break;
    case 4:
      polygon->shape = PolygonShape::box;
      break;
    default:
      polygon->shape = PolygonShape::multi_edge;
      break;
  }

  for (int i = 0; i < polygon->vertex_num; i++) {
    polygon->vertexes[i] = points[i];
  }

  UpdatePolygonValue(polygon, nullptr, false, false, POLYGON_MAX_RADIUS);

  // if not kown it, fill 0.
  polygon->min_tangent_radius = 0;

  return;
}

void GeneratePolygonByPoints(
    Polygon2D *polygon, const std::vector<ad_common::math::Vec2d> &points) {
  if (polygon == nullptr) {
    return;
  }

  polygon->vertex_num = static_cast<int>(points.size());
  polygon->vertex_num = std::min(MAX_POLYGON_VERTEX_NUM, polygon->vertex_num);

  switch (polygon->vertex_num) {
    case 1:
      polygon->shape = PolygonShape::point;
      break;
    case 2:
      polygon->shape = PolygonShape::line_segment;
      break;
    case 4:
      polygon->shape = PolygonShape::box;
      break;
    default:
      polygon->shape = PolygonShape::multi_edge;
      break;
  }

  for (int i = 0; i < polygon->vertex_num; i++) {
    polygon->vertexes[i].x = points[i].x();
    polygon->vertexes[i].y = points[i].y();
  }

  UpdatePolygonValue(polygon, nullptr, false, false, POLYGON_MAX_RADIUS);
  polygon->min_tangent_radius = 0;

  return;
}

int GetRightUpCoordinatePolygonByParam(Polygon2D *box,
                                       const double back_overhanging,
                                       const double front_edge_to_rear_axis,
                                       const double half_width) {
  box->vertexes[0].x = half_width;
  box->vertexes[0].y = front_edge_to_rear_axis;

  box->vertexes[1].x = -half_width;
  box->vertexes[1].y = front_edge_to_rear_axis;

  box->vertexes[2].x = -half_width;
  box->vertexes[2].y = -back_overhanging;

  box->vertexes[3].x = half_width;
  box->vertexes[3].y = -back_overhanging;

  box->vertex_num = 4;

  box->shape = PolygonShape::box;
  UpdatePolygonValue(box, NULL, 0, false, POLYGON_MAX_RADIUS);

  box->min_tangent_radius = half_width;

  return 0;
}

int RULocalPolygonToGlobal(Polygon2D *poly_global, const Polygon2D *poly_local,
                           const Pose2D *global_pose) {
  int32_t i;
  double sin_theta, cos_theta;

  sin_theta = ifly_sin(global_pose->theta);
  cos_theta = ifly_cos(global_pose->theta);
  for (i = 0; i < poly_local->vertex_num; i++) {
    CvtPosLocalToGlobalFast(&poly_global->vertexes[i], &poly_local->vertexes[i],
                            global_pose, sin_theta, cos_theta);
  }
  poly_global->vertex_num = poly_local->vertex_num;
  poly_global->radius = poly_local->radius;
  poly_global->min_tangent_radius = poly_local->min_tangent_radius;
  poly_global->shape = poly_local->shape;

  CvtPosLocalToGlobalFast(&poly_global->center_pt, &poly_local->center_pt,
                          global_pose, sin_theta, cos_theta);

  return 1;
}

void ULFLocalPolygonToGlobal(Polygon2D *poly_global,
                             const Polygon2D *poly_local,
                             const Pose2D &global_pose) {
  int i;

  Transform2d tf;
  tf.SetBasePose(global_pose);

  for (i = 0; i < poly_local->vertex_num; i++) {
    tf.ULFLocalPointToGlobal(&poly_global->vertexes[i],
                             poly_local->vertexes[i]);
  }

  poly_global->vertex_num = poly_local->vertex_num;
  poly_global->radius = poly_local->radius;
  poly_global->min_tangent_radius = poly_local->min_tangent_radius;
  poly_global->shape = poly_local->shape;

  tf.ULFLocalPointToGlobal(&poly_global->center_pt, poly_local->center_pt);

  return;
}

void RULocalPolygonToGlobalFast(Polygon2D *poly_global,
                               const Polygon2D *poly_local,
                               const Transform2d &tf) {
  for (int i = 0; i < poly_local->vertex_num; i++) {
    tf.RUFLocalPoseToGlobal(&poly_global->vertexes[i],
                             poly_local->vertexes[i]);
  }

  poly_global->vertex_num = poly_local->vertex_num;
  poly_global->radius = poly_local->radius;
  poly_global->min_tangent_radius = poly_local->min_tangent_radius;
  poly_global->shape = poly_local->shape;
  tf.RUFLocalPoseToGlobal(&poly_global->center_pt, poly_local->center_pt);

  return;
}

int GlobalPolygonToRULocal(Polygon2D *poly_local, const Polygon2D *poly_global,
                           const Pose2D *global_pose) {
  int32_t i;

  for (i = 0; i < poly_global->vertex_num; i++) {
    CvtPosGlobalToLocal(&poly_local->vertexes[i], &poly_global->vertexes[i],
                        global_pose);
  }

  poly_local->vertex_num = poly_global->vertex_num;
  poly_local->radius = poly_global->radius;
  poly_local->min_tangent_radius = poly_global->min_tangent_radius;
  poly_local->shape = poly_global->shape;

  CvtPosGlobalToLocal(&poly_local->center_pt, &poly_global->center_pt,
                      global_pose);

  return 1;
}

#define VEH_FRONT_RIGHT (0)
#define VEH_FRONT_LEFT (1)
#define VEH_BACK_LEFT (2)
#define VEH_BACK_RIGHT (3)
int ExtendVehBoxByWidth(Polygon2D *poly, double w,
                        const Polygon2D *adc_local_polygon) {
  if (poly == NULL) {
    return 0;
  }

  *poly = *adc_local_polygon;
  poly->vertexes[VEH_FRONT_RIGHT].x += w;
  poly->vertexes[VEH_FRONT_LEFT].x -= w;
  poly->vertexes[VEH_BACK_LEFT].x -= w;
  poly->vertexes[VEH_BACK_RIGHT].x += w;
  UpdatePolygonValue(poly, NULL, false, false, 100000);
  poly->min_tangent_radius = adc_local_polygon->min_tangent_radius;

  return 0;
}

int ExtendVehBoxByLength(Polygon2D *poly, double h,
                         const Polygon2D *adc_local_polygon) {
  if (poly == NULL) {
    return 0;
  }

  *poly = *adc_local_polygon;
  poly->vertexes[VEH_FRONT_RIGHT].y += h;
  poly->vertexes[VEH_FRONT_LEFT].y += h;
  poly->vertexes[VEH_BACK_LEFT].y -= h;
  poly->vertexes[VEH_BACK_RIGHT].y -= h;
  UpdatePolygonValue(poly, NULL, false, false, 100000);
  poly->min_tangent_radius = adc_local_polygon->min_tangent_radius;

  return 0;
}

int ExtendVehBoxByWidthLength(Polygon2D *poly, double left_w, double right_w,
                              double l, const Polygon2D *adc_local_polygon) {
  if (poly == NULL) {
    return 0;
  }

  *poly = *adc_local_polygon;
  poly->vertexes[VEH_FRONT_RIGHT].x += right_w;
  poly->vertexes[VEH_FRONT_LEFT].x -= left_w;
  poly->vertexes[VEH_BACK_LEFT].x -= left_w;
  poly->vertexes[VEH_BACK_RIGHT].x += right_w;

  poly->vertexes[VEH_FRONT_RIGHT].y += l;
  poly->vertexes[VEH_FRONT_LEFT].y += l;
  poly->vertexes[VEH_BACK_LEFT].y -= l;
  poly->vertexes[VEH_BACK_RIGHT].y -= l;
  UpdatePolygonValue(poly, NULL, false, false, 1000000);
  poly->min_tangent_radius = adc_local_polygon->min_tangent_radius;

  return 0;
}

int GetBoundingBoxByPolygon(cdl::AABB *box, const Polygon2D *polygon) {
  double min_x;
  double min_y;
  double max_x;
  double max_y;

  min_x = polygon->vertexes[0].x;
  max_x = polygon->vertexes[0].x;
  min_y = polygon->vertexes[0].y;
  max_y = polygon->vertexes[0].y;
  for (int i = 1; i < polygon->vertex_num; i++) {
    min_x = std::min(min_x, polygon->vertexes[i].x);
    min_y = std::min(min_y, polygon->vertexes[i].y);

    max_x = std::max(max_x, polygon->vertexes[i].x);
    max_y = std::max(max_y, polygon->vertexes[i].y);
  }

  box->min_[0] = min_x;
  box->min_[1] = min_y;

  box->max_[0] = max_x;
  box->max_[1] = max_y;

  return 0;
}

void PolygonDebugString(const Polygon2D *polygon, const std::string &name) {
  if (nullptr == polygon) {
    return;
  }

  ILOG_INFO << "polygon name = " << name;

  ILOG_INFO << "poly_type= " << static_cast<int>(polygon->shape)
            << " poly_vertex_num= " << polygon->vertex_num
            << " polygon_radius= " << polygon->radius;

  for (int i = 0; i < polygon->vertex_num; i++) {
    ILOG_INFO << "x, y " << polygon->vertexes[i].x << " ,"
              << polygon->vertexes[i].y;
  }

  return;
}

int GetUpLeftCoordinatePolygonByParam(Polygon2D *box,
                                      const double back_overhanging,
                                      const double front_edge_to_rear_axis,
                                      const double half_width) {
  box->vertexes[0].x = front_edge_to_rear_axis;
  box->vertexes[0].y = -half_width;

  box->vertexes[1].x = front_edge_to_rear_axis;
  box->vertexes[1].y = half_width;

  box->vertexes[2].x = -back_overhanging;
  box->vertexes[2].y = half_width;

  box->vertexes[3].x = -back_overhanging;
  box->vertexes[3].y = -half_width;

  box->vertex_num = 4;

  box->shape = PolygonShape::box;
  UpdatePolygonValue(box, NULL, 0, false, POLYGON_MAX_RADIUS);

  box->min_tangent_radius = half_width;

  return 0;
}

void ULFLocalPolygonToGlobal(Polygon2D *poly_global,
                             const Polygon2D *poly_local,
                             const Transform2d &tf) {
  for (int i = 0; i < std::min(poly_local->vertex_num, MAX_POLYGON_VERTEX_NUM);
       i++) {
    tf.ULFLocalPointToGlobal(&poly_global->vertexes[i],
                             poly_local->vertexes[i]);
  }

  poly_global->vertex_num = poly_local->vertex_num;
  poly_global->radius = poly_local->radius;
  poly_global->min_tangent_radius = poly_local->min_tangent_radius;
  poly_global->shape = poly_local->shape;

  tf.ULFLocalPointToGlobal(&poly_global->center_pt, poly_local->center_pt);

  return;
}

void GlobalPolygonToULFLocal(const Polygon2D *poly_global,
                             const Transform2d &tf, Polygon2D *poly_local) {
  for (int i = 0; i < poly_global->vertex_num; i++) {
    tf.GlobalPointToULFLocal(poly_global->vertexes[i],
                             &poly_local->vertexes[i]);
  }

  poly_local->vertex_num = poly_global->vertex_num;
  poly_local->radius = poly_global->radius;
  poly_local->min_tangent_radius = poly_global->min_tangent_radius;
  poly_local->shape = poly_global->shape;

  tf.GlobalPointToULFLocal(poly_global->center_pt, &poly_local->center_pt);

  return;
}

void GetCompactPolygonByParam(const double lat_buffer, const double lon_buffer,
                              Polygon2D *polygon) {
  const apa_planner::ApaParameters &config = apa_param.GetParam();
  if (config.car_vertex_x_vec.size() != 20 ||
      config.car_vertex_y_vec.size() != 20) {
    polygon->vertex_num = 0;
    return;
  }

  polygon->vertexes[0].x = config.car_vertex_x_vec[2] + lon_buffer;
  polygon->vertexes[0].y = config.car_vertex_y_vec[1] + lat_buffer;

  polygon->vertexes[1].x = config.car_vertex_x_vec[0] + lon_buffer;
  polygon->vertexes[1].y = config.car_vertex_y_vec[0] + lat_buffer;

  polygon->vertexes[2].x = config.car_vertex_x_vec[15] - lon_buffer;
  polygon->vertexes[2].y = config.car_vertex_y_vec[15] + lat_buffer;

  polygon->vertexes[3].x = config.car_vertex_x_vec[13] - lon_buffer;
  polygon->vertexes[3].y = config.car_vertex_y_vec[14] + lat_buffer;

  polygon->vertexes[4].x = config.car_vertex_x_vec[12] - lon_buffer;
  polygon->vertexes[4].y = config.car_vertex_y_vec[11] - lat_buffer;

  polygon->vertexes[5].x = config.car_vertex_x_vec[10] - lon_buffer;
  polygon->vertexes[5].y = config.car_vertex_y_vec[10] - lat_buffer;

  polygon->vertexes[6].x = config.car_vertex_x_vec[5] + lon_buffer;
  polygon->vertexes[6].y = config.car_vertex_y_vec[5] - lat_buffer;

  polygon->vertexes[7].x = config.car_vertex_x_vec[3] + lon_buffer;
  polygon->vertexes[7].y = config.car_vertex_y_vec[4] - lat_buffer;

  polygon->vertex_num = 8;

  polygon->shape = PolygonShape::multi_edge;
  UpdatePolygonValue(polygon, NULL, 0, false, POLYGON_MAX_RADIUS);

  polygon->min_tangent_radius = config.car_width / 2 + lat_buffer;

  return;
}

void GenerateVehCompactPolygon(const double lateral_safe_buffer,
                               const double lon_safe_buffer,
                               const double aabb_buffer,
                               PolygonFootPrint *foot_print) {
  GetCompactPolygonByParam(lateral_safe_buffer, lon_safe_buffer,
                           &foot_print->body);

  // left mirror
  const apa_planner::ApaParameters &config = apa_param.GetParam();
  Position2D center;
  center.x = config.footprint_circle_x[6];
  center.y = config.footprint_circle_y[6] + lateral_safe_buffer;
  double radius = std::fabs(config.footprint_circle_r[6]);
  GenerateMirrorPolygon(0.3, radius * 2, center, &foot_print->mirror_left);

  // right mirror
  center.x = config.footprint_circle_x[3];
  center.y = config.footprint_circle_y[3] - lateral_safe_buffer;
  GenerateMirrorPolygon(0.3, radius * 2, center, &foot_print->mirror_right);

  GetUpLeftCoordinatePolygonByParam(
      &foot_print->max_polygon, config.rear_overhanging + lon_safe_buffer,
      config.wheel_base + config.front_overhanging + lon_safe_buffer,
      config.max_car_width / 2.0 + aabb_buffer);

  return;
}

void GenerateMirrorPolygon(const double x_length, const double y_length,
                           const Position2D &center, Polygon2D *box) {
  box->vertexes[0].x = center.x + x_length / 2;
  box->vertexes[0].y = center.y - y_length / 2;

  box->vertexes[1].x = center.x + x_length / 2;
  box->vertexes[1].y = center.y + y_length / 2;

  box->vertexes[2].x = center.x - x_length / 2;
  box->vertexes[2].y = center.y + y_length / 2;

  box->vertexes[3].x = center.x - x_length / 2;
  box->vertexes[3].y = center.y - y_length / 2;

  box->vertex_num = 4;

  box->shape = PolygonShape::box;
  UpdatePolygonValue(box, NULL, 0, false, POLYGON_MAX_RADIUS);

  box->min_tangent_radius = std::min(y_length / 2, x_length / 2);

  return;
}

void GenerateBoundingBox(const double x_length, const double y_length,
                         const Eigen::Vector2d &center,
                         std::vector<Eigen::Vector2d> &box) {
  Eigen::Vector2d pt;
  pt[0] = center.x() + x_length / 2;
  pt[1] = center.y() - y_length / 2;
  box.emplace_back(pt);

  pt[0] = center.x() + x_length / 2;
  pt[1] = center.y() + y_length / 2;
  box.emplace_back(pt);

  pt[0] = center.x() - x_length / 2;
  pt[1] = center.y() + y_length / 2;
  box.emplace_back(pt);

  pt[0] = center.x() - x_length / 2;
  pt[1] = center.y() - y_length / 2;
  box.emplace_back(pt);

  return;
}

void LocalPolygonToGlobal(const std::vector<Eigen::Vector2d> &poly_local,
                          const Pose2D &global_pose,
                          std::vector<Eigen::Vector2d> &poly_global) {
  Transform2d tf;
  tf.SetBasePose(global_pose);

  Eigen::Vector2d point;
  for (int i = 0; i < poly_local.size(); i++) {
    tf.ULFLocalPointToGlobal(point, poly_local[i]);
    poly_global.emplace_back(point);
  }

  return;
}

void GeneratePolygonByPoints(const std::vector<Eigen::Vector2d> &points,
                             Polygon2D *polygon) {
  if (polygon == nullptr) {
    return;
  }

  polygon->vertex_num = static_cast<int>(points.size());
  polygon->vertex_num = std::min(MAX_POLYGON_VERTEX_NUM, polygon->vertex_num);

  switch (polygon->vertex_num) {
    case 1:
      polygon->shape = PolygonShape::point;
      break;
    case 2:
      polygon->shape = PolygonShape::line_segment;
      break;
    case 4:
      polygon->shape = PolygonShape::box;
      break;
    default:
      polygon->shape = PolygonShape::multi_edge;
      break;
  }

  for (int i = 0; i < polygon->vertex_num; i++) {
    polygon->vertexes[i].x = points[i].x();
    polygon->vertexes[i].y = points[i].y();
  }

  UpdatePolygonValue(polygon, nullptr, false, false, POLYGON_MAX_RADIUS);
  polygon->min_tangent_radius = 0;

  return;
}

void FootPrintLocalToGlobal(const Transform2d &tf,
                            const PolygonFootPrint *local_foot_print,
                            PolygonFootPrint *global_foot_print) {
  ULFLocalPolygonToGlobal(&global_foot_print->max_polygon,
                          &local_foot_print->max_polygon, tf);

  ULFLocalPolygonToGlobal(&global_foot_print->body, &local_foot_print->body,
                          tf);

  ULFLocalPolygonToGlobal(&global_foot_print->mirror_left,
                          &local_foot_print->mirror_left, tf);

  ULFLocalPolygonToGlobal(&global_foot_print->mirror_right,
                          &local_foot_print->mirror_right, tf);
  return;
}

void GetUpLeftCoordinatePolygonByParam(std::array<Position2f, 4> &box,
                                       const float back_overhanging,
                                       const float front_edge_to_rear_axis,
                                       const float half_width) {
  box[0].x = front_edge_to_rear_axis;
  box[0].y = -half_width;

  box[1].x = front_edge_to_rear_axis;
  box[1].y = half_width;

  box[2].x = -back_overhanging;
  box[2].y = half_width;

  box[3].x = -back_overhanging;
  box[3].y = -half_width;

  return;
}

void GetBoundingBoxByPolygon(cdl::AABB2f *box,
                             const std::array<Position2f, 4> &polygon) {
  float min_x;
  float min_y;
  float max_x;
  float max_y;

  min_x = polygon[0].x;
  max_x = polygon[0].x;
  min_y = polygon[0].y;
  max_y = polygon[0].y;
  for (int i = 1; i < 4; i++) {
    min_x = std::min(min_x, polygon[i].x);
    min_y = std::min(min_y, polygon[i].y);

    max_x = std::max(max_x, polygon[i].x);
    max_y = std::max(max_y, polygon[i].y);
  }

  box->min_[0] = min_x;
  box->min_[1] = min_y;

  box->max_[0] = max_x;
  box->max_[1] = max_y;

  return;
}

}  // namespace planning