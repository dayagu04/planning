
#pragma once

#include <Eigen/Core>
#include <iostream>
#include <string>

#include "aabb2d.h"
#include "ad_common/math/vec2d.h"
#include "gjk2d.h"
#include "log_glog.h"
#include "pose2d.h"
#include "transform2d.h"

namespace planning {

#define POLYGON_MAX_RADIUS (1000000.0)

enum class PolygonShape {
  point,
  line_segment,
  triangle,
  box,
  multi_edge,
};

struct Polygon2D {
  PolygonShape shape;
  int vertex_num;
  // counterclockwise
  Position2D vertexes[MAX_POLYGON_VERTEX_NUM];

  // must fill 内切圆和外切圆圆心
  Position2D center_pt;
  // must fill 外切圆半径
  float radius;
  // must fill 内切圆半径
  float min_tangent_radius;

  void PrintInfo(const bool enable_log = true) const {
    ILOG_INFO_IF(enable_log)
        << "shape = " << static_cast<int>(shape)
        << "  vertex_num = " << vertex_num << "  center_pt = " << center_pt.x
        << "  " << center_pt.y << "  radius = " << radius
        << "  min_tangent_radius = " << min_tangent_radius;

    for (int i = 0; i < std::min(vertex_num, MAX_POLYGON_VERTEX_NUM); ++i) {
      vertexes[i].PrintInfo();
    }
  }

  const bool IsValid() {
    if (vertex_num > MAX_POLYGON_VERTEX_NUM || vertex_num < 1) {
      return false;
    }
    return true;
  }

  void FillTangentCircleParams(const std::vector<Eigen::Vector2d> &vertex_vec) {
    vertex_num = vertex_vec.size();

    if (vertex_num < 1 || vertex_num > MAX_POLYGON_VERTEX_NUM) {
      return;
    }

    switch (vertex_num) {
      case 1:
        shape = PolygonShape::point;
        break;
      case 2:
        shape = PolygonShape::line_segment;
        break;
      case 4:
        shape = PolygonShape::box;
        break;
      default:
        shape = PolygonShape::multi_edge;
        break;
    }

    for (size_t i = 0; i < vertex_num; ++i) {
      vertexes[i] = Position2D(static_cast<float>(vertex_vec[i].x()),
                               static_cast<float>(vertex_vec[i].y()));
    }

    switch (shape) {
      case PolygonShape::point:
        center_pt = vertexes[0];
        radius = 0.0;
        min_tangent_radius = 0.0;
        break;

      case PolygonShape::line_segment:
        center_pt.x = 0.5 * (vertexes[0].x + vertexes[1].x);
        center_pt.y = 0.5 * (vertexes[0].y + vertexes[1].y);
        radius = CalcPointDist(&vertexes[0], &vertexes[1]) * 0.5;
        min_tangent_radius = 0.0;
        break;

      case PolygonShape::box:
        center_pt.x = (vertexes[0].x + vertexes[2].x) * 0.5;
        center_pt.y = (vertexes[0].y + vertexes[2].y) * 0.5;
        radius = CalcPointDist(&vertexes[0], &vertexes[2]) * 0.5;
        min_tangent_radius =
            std::min(CalcPointDist(&vertexes[0], &vertexes[1]) * 0.5,
                     CalcPointDist(&vertexes[0], &vertexes[3]) * 0.5);
        break;

      case PolygonShape::multi_edge:
        center_pt.x = 0.0;
        center_pt.y = 0.0;
        for (size_t i = 0; i < vertex_num; ++i) {
          center_pt.x += vertexes[i].x;
          center_pt.y += vertexes[i].y;
        }
        center_pt.x /= vertex_num;
        center_pt.y /= vertex_num;

        radius = 0.0;
        for (size_t i = 0; i < vertex_num; ++i) {
          radius = std::max(radius, CalcPointDist(&center_pt, &vertexes[i]));
        }
        min_tangent_radius = 0.0;

        break;
      default:
        break;
    }
  }
};

// in system, you can use polygon foot_print or circle foot print.
struct PolygonFootPrint {
  Polygon2D chassis;
  Polygon2D mirror_to_front_overhang_expand_front;
  Polygon2D mirror_to_rear_overhang;
  Polygon2D body;
  Polygon2D mirror_left;
  Polygon2D mirror_right;

  // 这里引入了分层碰撞检测方案BVH.
  // 二叉/八叉树的方案在游戏领域常用，这里不使用树， 而是最大polygon.
  // 将来如果有时间，可以引入二叉树的方案. for collision check:
  // 如果最外层的polygon不存在碰撞，那么不必检测内层； for distance check:
  // 如果最外层polygon不存在碰撞,
  // 那么内层的距离值不必检测，因为内层的距离往往较大，不用担心安全问题.
  // 可以将距离值默认成2米.
  Polygon2D max_polygon;
};

int InitPolygon(Polygon2D *polygon);

int PolygonCopy(Polygon2D *des_poly, const Polygon2D *src_poly);

void UpdatePolygonValue(Polygon2D *polygon, const Pose2D *center_pose,
                       bool use_center_pose, bool radius_known, float radius);

int GenerateRectPolygon(Polygon2D *polygon, float min_x, float min_y,
                        float max_x, float max_y);

int GenerateUpLeftFrameBox(Polygon2D *polygon, float min_x, float min_y,
                           float max_x, float max_y);

int GenerateLineSegmentPolygon(Polygon2D *polygon, const Position2D &start,
                               const Position2D &end);

int GenerateLineSegmentPolygon(Polygon2D *polygon, const Eigen::Vector2f &start,
                               const Eigen::Vector2f &end);

int GeneratePolygonByPoint(Polygon2D *polygon, const Position2D &point);

int GeneratePolygonByPoint(Polygon2D *polygon, const Eigen::Vector2f &point);

int GetRightUpCoordinatePolygonByParam(Polygon2D *box,
                                       const float back_overhanging,
                                       const float front_edge_to_rear_axis,
                                       const float half_width);

int RULocalPolygonToGlobal(Polygon2D *poly_global, const Polygon2D *poly_local,
                           const Pose2D *global_pose);

int RULocalPolygonToGlobalFast(Polygon2D *poly_global,
                               const Polygon2D *poly_local,
                               const Pose2D *global_pose, const float cos_theta,
                               const float sin_theta);

int GlobalPolygonToRULocal(Polygon2D *poly_local, const Polygon2D *poly_global,
                           const Pose2D *global_pose);

int ExtendVehBoxByWidth(Polygon2D *poly, float w,
                        const Polygon2D *adc_local_polygon);

int ExtendVehBoxByLength(Polygon2D *poly, float h,
                         const Polygon2D *adc_local_polygon);

int ExtendVehBoxByWidthLength(Polygon2D *poly, float left_w, float right_w,
                              float l, const Polygon2D *adc_local_polygon);

void GeneratePolygonByPoints(Polygon2D *polygon,
                             const std::vector<Position2D> &points);

void GeneratePolygonByPoints(Polygon2D *polygon,
                             const std::vector<ad_common::math::Vec2d> &points);

int GeneratePolygonByAABB(Polygon2D *polygon, const cdl::AABB &box);

int GetBoundingBoxByPolygon(cdl::AABB *box, const Polygon2D *polygon);

void PolygonDebugString(const Polygon2D *polygon, const std::string &name);

// up left frame
void ULFLocalPolygonToGlobal(Polygon2D *poly_global,
                             const Polygon2D *poly_local,
                             const Pose2D &global_pose);

int GetUpLeftCoordinatePolygonByParam(Polygon2D *box,
                                      const float back_overhanging,
                                      const float front_edge_to_rear_axis,
                                      const float half_width);

void ULFLocalPolygonToGlobal(Polygon2D *poly_global,
                             const Polygon2D *poly_local,
                             const Transform2d &tf);

void GlobalPolygonToULFLocal(const Polygon2D *poly_global,
                             const Transform2d &tf, Polygon2D *poly_local);

void GetCompactPolygonByParam(const float lat_buffer, const float lon_buffer,
                              Polygon2D *polygon);

// Compact car body for accurate safe check.
void GenerateVehCompactPolygon(const float lateral_safe_buffer,
                               const float lon_safe_buffer,
                               const float aabb_buffer,
                               PolygonFootPrint *foot_print);

void GenerateMirrorPolygon(const float x_length, const float y_length,
                           const Position2D &center, Polygon2D *box);

void GenerateBoundingBox(const float x_length, const float y_length,
                         const Eigen::Vector2f &center,
                         std::vector<Eigen::Vector2f> &box);

void LocalPolygonToGlobal(const std::vector<Eigen::Vector2f> &poly_local,
                          const Pose2D &global_pose,
                          std::vector<Eigen::Vector2f> &poly_global);

void GeneratePolygonByPoints(const std::vector<Eigen::Vector2f> &points,
                             Polygon2D *polygon);

void FootPrintLocalToGlobal(const Transform2d &tf,
                            const PolygonFootPrint *local_foot_print,
                            PolygonFootPrint *global_foot_print);

}  // namespace planning
