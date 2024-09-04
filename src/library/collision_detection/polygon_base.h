
#pragma once

#include <Eigen/Core>
#include <iostream>

#include "aabb2d.h"
#include "ad_common/math/vec2d.h"
#include "gjk2d.h"
#include "pose2d.h"
#include "transform2d.h"

namespace planning {

#define POLYGON_MAX_RADIUS (1000000.0)

enum class PolygonShape {
  multi_edge,
  box,
  point,
  line_segment,
};

struct Polygon2D {
  PolygonShape shape;
  int vertex_num;
  // counterclockwise
  Position2D vertexes[MAX_POLYGON_VERTEX_NUM];

  // must fill
  Position2D center_pt;
  // must fill
  double radius;
  // must fill
  double min_tangent_radius;

  const bool IsValid() {
    if (vertex_num > MAX_POLYGON_VERTEX_NUM || vertex_num < 1) {
      return false;
    }
    return true;
  }
};

int InitPolygon(Polygon2D *polygon);

int PolygonCopy(Polygon2D *des_poly, const Polygon2D *src_poly);

int UpdatePolygonValue(Polygon2D *polygon, const Pose2D *center_pose,
                       bool use_center_pose, bool radius_known, double radius);

int GenerateRectPolygon(Polygon2D *polygon, double min_x, double min_y,
                        double max_x, double max_y);

int GenerateUpLeftFrameBox(Polygon2D *polygon, double min_x, double min_y,
                           double max_x, double max_y);

int GenerateLineSegmentPolygon(Polygon2D *polygon, const Position2D &start,
                               const Position2D &end);

int GenerateLineSegmentPolygon(Polygon2D *polygon, const Eigen::Vector2d &start,
                               const Eigen::Vector2d &end);

int GeneratePolygonByPoint(Polygon2D *polygon, const Position2D &point);

int GeneratePolygonByPoint(Polygon2D *polygon, const Eigen::Vector2d &point);

int GetRightUpCoordinatePolygonByParam(Polygon2D *box,
                                       const double back_overhanging,
                                       const double front_edge_to_rear_axis,
                                       const double half_width);

int RULocalPolygonToGlobal(Polygon2D *poly_global, const Polygon2D *poly_local,
                           const Pose2D *global_pose);

int RULocalPolygonToGlobalFast(Polygon2D *poly_global,
                               const Polygon2D *poly_local,
                               const Pose2D *global_pose,
                               const double cos_theta, const double sin_theta);

int GlobalPolygonToRULocal(Polygon2D *poly_local, const Polygon2D *poly_global,
                           const Pose2D *global_pose);

int ExtendVehBoxByWidth(Polygon2D *poly, double w,
                        const Polygon2D *adc_local_polygon);

int ExtendVehBoxByLength(Polygon2D *poly, double h,
                         const Polygon2D *adc_local_polygon);

int ExtendVehBoxByWidthLength(Polygon2D *poly, double left_w, double right_w,
                              double l, const Polygon2D *adc_local_polygon);

int GeneratePolygonByPoints(Polygon2D *polygon, std::vector<Position2D> points);

int GeneratePolygonByPoints(Polygon2D *polygon,
                            std::vector<ad_common::math::Vec2d> points);

int GeneratePolygonByAABB(Polygon2D *polygon, const cdl::AABB &box);

int GetBoundingBoxByPolygon(cdl::AABB *box, const Polygon2D *polygon);

int PolygonDebugString(const Polygon2D *polygon);

// up left frame
int ULFLocalPolygonToGlobal(Polygon2D *poly_global, const Polygon2D *poly_local,
                            const Pose2D &global_pose);

int GetUpLeftCoordinatePolygonByParam(Polygon2D *box,
                                      const double back_overhanging,
                                      const double front_edge_to_rear_axis,
                                      const double half_width);

int ULFLocalPolygonToGlobal(Polygon2D *poly_global, const Polygon2D *poly_local,
                            const Transform2d &tf);

}  // namespace planning
