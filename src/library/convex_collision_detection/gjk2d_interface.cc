#include "gjk2d_interface.h"

#include "log_glog.h"
#include "pose2d.h"

namespace planning {

void GJK2DInterface::PolygonCollision(bool *is_collision,
                                      const Polygon2D *polygon_p,
                                      const Polygon2D *polygon_q) {
  int32_t i;
  shape_p_.size = polygon_p->vertex_num;
  for (i = 0; i < polygon_p->vertex_num; ++i) {
    shape_p_.vertices[i](0) = polygon_p->vertexes[i].x;
    shape_p_.vertices[i](1) = polygon_p->vertexes[i].y;
  }

  shape_q_.size = polygon_q->vertex_num;
  for (i = 0; i < polygon_q->vertex_num; ++i) {
    shape_q_.vertices[i](0) = polygon_q->vertexes[i].x;
    shape_q_.vertices[i](1) = polygon_q->vertexes[i].y;
  }

  bool result = gjk_solver_.Collision(shape_p_, shape_q_);
  if (result) {
    *is_collision = true;
  } else {
    *is_collision = false;
  }
  return;
}

void GJK2DInterface::PolygonDistance(bool *is_collision, double *dist,
                                     const Polygon2D *polygon_p,
                                     const Polygon2D *polygon_q) {
  int32_t i;
  shape_p_.size = polygon_p->vertex_num;
  for (i = 0; i < polygon_p->vertex_num; ++i) {
    shape_p_.vertices[i](0) = polygon_p->vertexes[i].x;
    shape_p_.vertices[i](1) = polygon_p->vertexes[i].y;
  }

  shape_q_.size = polygon_q->vertex_num;
  for (i = 0; i < polygon_q->vertex_num; ++i) {
    shape_q_.vertices[i](0) = polygon_q->vertexes[i].x;
    shape_q_.vertices[i](1) = polygon_q->vertexes[i].y;
  }

  gjk_solver_.Distance2(shape_p_, shape_q_, &gjk2d_result_);
  if (gjk2d_result_.distance < cdl::constants::gjk_tolorance()) {
    *is_collision = true;
  } else {
    *is_collision = false;
  }
  *dist = gjk2d_result_.distance;

  return;
}

void GJK2DInterface::PolygonDistanceByThresh(bool *is_collision, double *dist,
                                             const Polygon2D *polygon_p,
                                             const Polygon2D *polygon_q,
                                             const double dist_thresh) {
  double point_dist, circle_dist;

  point_dist = CalcPointDist(&polygon_p->center_pt, &polygon_q->center_pt);
  circle_dist = point_dist - (polygon_p->radius + polygon_q->radius);
  if (!ifly_fless(circle_dist, dist_thresh)) {
    *is_collision = false;
    *dist = circle_dist;
  } else {
    PolygonDistance(is_collision, dist, polygon_p, polygon_q);
  }

  return;
}

void GJK2DInterface::PolygonDistanceByThresh(const Polygon2D *polygon,
                                             const Position2D &point,
                                             const double dist_thresh,
                                             bool *is_collision, double *dist) {
  double d, delta_d;

  d = CalcPointDist(&polygon->center_pt, &point);
  delta_d = d - polygon->radius;

  if (!ifly_fless(delta_d, dist_thresh)) {
    *is_collision = false;
    *dist = delta_d;
  } else if (d <= polygon->min_tangent_radius) {
    *is_collision = true;
    *dist = 0;

    // ILOG_INFO << "d" << d << ",min_tangent_radius"
    //           << polygon->min_tangent_radius;
  } else {
    int32_t i;

    shape_p_.size = polygon->vertex_num;
    for (i = 0; i < polygon->vertex_num; ++i) {
      shape_p_.vertices[i][0] = polygon->vertexes[i].x;
      shape_p_.vertices[i][1] = polygon->vertexes[i].y;
    }

    shape_q_.size = 1;
    shape_q_.vertices[0][0] = point.x;
    shape_q_.vertices[0][1] = point.y;

    gjk_solver_.Distance2(shape_p_, shape_q_, &gjk2d_result_);
    if (gjk2d_result_.distance < cdl::constants::gjk_tolorance()) {
      *is_collision = true;
    } else {
      *is_collision = false;
    }
    *dist = gjk2d_result_.distance;
  }

  return;
}

void GJK2DInterface::PolygonCollisionByCircleCheck(bool *is_collision,
                                                   const Polygon2D *polygon_p,
                                                   const Polygon2D *polygon_q,
                                                   const double dist_thresh) {
  double d, delta_d;

  d = CalcPointDist(&polygon_p->center_pt, &polygon_q->center_pt);
  delta_d = d - (polygon_p->radius + polygon_q->radius);
  if (!ifly_fless(delta_d, dist_thresh)) {
    *is_collision = false;
  } else {
    PolygonCollision(is_collision, polygon_p, polygon_q);
  }

  return;
}

void GJK2DInterface::PolygonPointCollisionDetect(bool *is_collision,
                                                 const Polygon2D *polygon,
                                                 const Position2D &point) {
  double d, delta_d;

  d = CalcPointDist(&polygon->center_pt, &point);
  delta_d = d - polygon->radius;

  if (!ifly_fless(delta_d, 0.01)) {
    *is_collision = false;
  } else if (d <= polygon->min_tangent_radius) {
    *is_collision = true;
  } else {
    int32_t i;

    shape_p_.size = polygon->vertex_num;
    for (i = 0; i < polygon->vertex_num; ++i) {
      shape_p_.vertices[i][0] = polygon->vertexes[i].x;
      shape_p_.vertices[i][1] = polygon->vertexes[i].y;
    }

    shape_q_.size = 1;
    shape_q_.vertices[0][0] = point.x;
    shape_q_.vertices[0][1] = point.y;

    bool result = gjk_solver_.Collision(shape_p_, shape_q_);
    if (result) {
      *is_collision = true;
    } else {
      *is_collision = false;
    }
  }

  return;
}

void GJK2DInterface::PolygonDistanceCheck(bool *is_collision, double *dist,
                                          Position2D *pos_obj1,
                                          Position2D *pos_obj2,
                                          const Polygon2D *polygon_obj1,
                                          const Polygon2D *polygon_obj2) {
  int32_t i;
  cdl::GJK2D gjk_solver;
  cdl::DistResult2D gjk2d_result;
  cdl::ShapeProxy2D P, Q;

  P.size = polygon_obj1->vertex_num;
  for (i = 0; i < polygon_obj1->vertex_num; ++i) {
    P.vertices[i](0) = polygon_obj1->vertexes[i].x;
    P.vertices[i](1) = polygon_obj1->vertexes[i].y;
  }
  Q.size = polygon_obj2->vertex_num;
  for (i = 0; i < polygon_obj2->vertex_num; ++i) {
    Q.vertices[i](0) = polygon_obj2->vertexes[i].x;
    Q.vertices[i](1) = polygon_obj2->vertexes[i].y;
  }

  gjk_solver.Distance2(P, Q, &gjk2d_result);
  if (gjk2d_result.distance < cdl::constants::gjk_tolorance()) {
    *is_collision = true;
  } else {
    *is_collision = false;
  }
  *dist = (double)(gjk2d_result.distance);
  pos_obj1->x = gjk2d_result.p(0);
  pos_obj1->y = gjk2d_result.p(1);
  pos_obj2->x = gjk2d_result.q(0);
  pos_obj2->y = gjk2d_result.q(1);

  return;
}

void GJK2DInterface::RaycastCollisoinCheck(
    Position2D *source, Position2D *direction, float max_lambda,
    const Polygon2D *polygon_veh, bool *is_collision,
    Position2D *collision_point, int32_t *hit_edge_id, float *ratio,
    RaycastCollisionInfo *raycast_label) {
  int32_t i;
  cdl::RayCastRequest input;
  cdl::GJK2D gjk_solver;
  cdl::RayCastResult raycast_result;
  const Position2D *points;
  int32_t point_b_id;

  float d;
  float d_a;

  input.source(0) = source->x;
  input.source(1) = source->y;
  input.direction(0) = direction->x;
  input.direction(1) = direction->y;
  input.max_lambda = max_lambda;

  input.shape.size = polygon_veh->vertex_num;

  for (i = 0; i < polygon_veh->vertex_num; ++i) {
    input.shape.vertices[i](0) = polygon_veh->vertexes[i].x;
    input.shape.vertices[i](1) = polygon_veh->vertexes[i].y;
  }
  gjk_solver.traversalRayCast(&input, &raycast_result);

  if (raycast_result.status == cdl::RayCastStatus::HIT ||
      raycast_result.status == cdl::RayCastStatus::SOURCE_IS_INSIDE) {
    *is_collision = true;
    collision_point->x = raycast_result.hit_spot(0);
    collision_point->y = raycast_result.hit_spot(1);
    *hit_edge_id = raycast_result.hit_edge_id;
    points = polygon_veh->vertexes;
    point_b_id = *hit_edge_id + 1;
    if (point_b_id == polygon_veh->vertex_num) {
      point_b_id = 0;
    }

    Position2D point1 = points[*hit_edge_id];
    Position2D point_b = points[point_b_id];

    d = CalcPointDist(&point1, &point_b);
    d_a = CalcPointDist(&point1, collision_point);
    if (ifly_fequal(d, 0.0)) {
      d = 0.1;
    }
    *ratio = d_a / d;
    if (raycast_label != NULL) {
      if (raycast_result.status == cdl::RayCastStatus::HIT) {
        *raycast_label = RAYCAST_HIT_POLYGON;
      } else {
        *raycast_label = RAYCAST_SOURCE_POINT_IN_POLYGON;
      }
    }
  } else {
    *is_collision = false;
    collision_point->x = 0;
    collision_point->y = 0;
    *hit_edge_id = -1;
    *ratio = -1.0;
    if (raycast_label != NULL) {
      *raycast_label = RAYCAST_NO_COLLISION;
    }
  }

  return;
}

void GJK2DInterface::get_cross_point_for_two_ray(cdl::Vector2r &ret,
                                                 const cdl::Vector2r &start0,
                                                 const cdl::Vector2r &dir0,
                                                 const cdl::Vector2r &start1,
                                                 const cdl::Vector2r &dir1) {
  cdl::real t0, tmp_value0, tmp_value1;
  cdl::Vector2r delta_vec;

  delta_vec = start1 - start0;
  tmp_value0 = cdl::GetVectorCross(delta_vec, dir1);
  tmp_value1 = cdl::GetVectorCross(dir0, dir1);
  t0 = tmp_value0 / tmp_value1;

  ret = start0 + t0 * dir0;

  return;
}

#define SHAPECAST_PARALLEL_ERR (0.1)
#define SHAPECAST_PARALLEL_EXPLORE_DIST (20.0)
void GJK2DInterface::ShapeCast(bool *is_collision, double *distA, double *distB,
                               Position2D *collision_pointA,
                               Position2D *collision_pointB,
                               const Polygon2D *polygonA_start,
                               const Polygon2D *polygonA_end,
                               const Polygon2D *polygonB_start,
                               const Polygon2D *polygonB_end) {
  int32_t i;
  cdl::ShapeCastRequest request;
  double cross_value, dot_value0, dot_value1;
  cdl::Vector2r cross_pt, centerA, centerB;
  Position2D dirA, dirB;
  bool intersected;
  cdl::GJK2D gjk_solver;
  cdl::ShapeCastResult result;

  request.proxyA.size = polygonA_start->vertex_num;
  for (i = 0; i < polygonA_start->vertex_num; ++i) {
    request.proxyA.vertices[i](0) = polygonA_start->vertexes[i].x;
    request.proxyA.vertices[i](1) = polygonA_start->vertexes[i].y;
  }
  request.proxyB.size = polygonB_start->vertex_num;
  for (i = 0; i < polygonB_start->vertex_num; ++i) {
    request.proxyB.vertices[i](0) = polygonB_start->vertexes[i].x;
    request.proxyB.vertices[i](1) = polygonB_start->vertexes[i].y;
  }

  request.translationA(0) =
      polygonA_end->center_pt.x - polygonA_start->center_pt.x;
  request.translationA(1) =
      polygonA_end->center_pt.y - polygonA_start->center_pt.y;
  request.translationB(0) =
      polygonB_end->center_pt.x - polygonB_start->center_pt.x;
  request.translationB(1) =
      polygonB_end->center_pt.y - polygonB_start->center_pt.y;

  dirA.x = request.translationA(0);
  dirA.y = request.translationA(1);
  dirB.x = request.translationB(0);
  dirB.y = request.translationB(1);

  /* Check zero vector */
  dot_value0 = CalcPointOriginDistSq(&dirA);
  dot_value1 = CalcPointOriginDistSq(&dirB);
  if (ifly_fgreater(dot_value0, ifly_float64_epsilon) &&
      ifly_fgreater(dot_value1, ifly_float64_epsilon)) {
    /* Check parallel shapecast direction */
    cross_value = ifly_fabs(dirA.x * dirB.y - dirA.y * dirB.x);
    if (ifly_fless(cross_value, SHAPECAST_PARALLEL_ERR)) /* almost parallel */
    {
      request.translationA.normalize();
      request.translationB.normalize();
      request.translationA =
          SHAPECAST_PARALLEL_EXPLORE_DIST * request.translationA;
      request.translationB =
          SHAPECAST_PARALLEL_EXPLORE_DIST * request.translationB;
    } else {
      IsLineSegmentIntersection(
          &intersected, &polygonA_start->center_pt, &polygonA_end->center_pt,
          &polygonB_start->center_pt, &polygonB_end->center_pt);

      if (intersected) {
        centerA(0) = polygonA_start->center_pt.x;
        centerA(1) = polygonA_start->center_pt.y;
        centerB(0) = polygonB_start->center_pt.x;
        centerB(1) = polygonB_start->center_pt.y;
        get_cross_point_for_two_ray(cross_pt, centerA, request.translationA,
                                    centerB, request.translationB);

        request.translationA = cross_pt - centerA;
        request.translationB = cross_pt - centerB;
      }
    }
  }

  gjk_solver.shapeCast(&result, &request);

  if (result.collided) {
    *is_collision = true;
    *distA = result.distanceA;
    *distB = result.distanceB;
    collision_pointA->x = result.collision_pointA(0);
    collision_pointA->y = result.collision_pointA(1);
    collision_pointB->x = result.collision_pointB(0);
    collision_pointB->y = result.collision_pointB(1);
  } else {
    *is_collision = false;
  }

  return;
}

void GJK2DInterface::ShapeCastByDirection(
    bool *is_collision, double *distA, double *distB,
    Position2D *collision_pointA, Position2D *collision_pointB,
    const Polygon2D *polygonA_start, const Position2D *dirA,
    const Polygon2D *polygonB_start, const Position2D *dirB) {
  int32_t i;
  cdl::ShapeCastRequest request_tmp;
  cdl::ShapeCastRequest *request = &request_tmp;
  cdl::GJK2D gjk_solver;
  cdl::ShapeCastResult result_tmp;
  cdl::ShapeCastResult *result = &result_tmp;
  double cross_value, dot_value0, dot_value1;
  cdl::Vector2r cross_pt;

  request->proxyA.size = polygonA_start->vertex_num;
  for (i = 0; i < polygonA_start->vertex_num; ++i) {
    request->proxyA.vertices[i](0) = polygonA_start->vertexes[i].x;
    request->proxyA.vertices[i](1) = polygonA_start->vertexes[i].y;
  }
  request->proxyB.size = polygonB_start->vertex_num;
  for (i = 0; i < polygonB_start->vertex_num; ++i) {
    request->proxyB.vertices[i](0) = polygonB_start->vertexes[i].x;
    request->proxyB.vertices[i](1) = polygonB_start->vertexes[i].y;
  }
  request->translationA(0) = dirA->x;
  request->translationA(1) = dirA->y;
  request->translationB(0) = dirB->x;
  request->translationB(1) = dirB->y;

  /* Check zero vector */
  dot_value0 = CalcPointOriginDistSq(dirA);
  dot_value1 = CalcPointOriginDistSq(dirB);
  if (ifly_fgreater(dot_value0, ifly_float64_epsilon) &&
      ifly_fgreater(dot_value1, ifly_float64_epsilon)) {
    /* Check parallel shapecast direction */
    cross_value = ifly_fabs(dirA->x * dirB->y - dirA->y * dirB->x);
    if (ifly_fless(cross_value, SHAPECAST_PARALLEL_ERR)) /* almost parallel */
    {
      request->translationA.normalize();
      request->translationB.normalize();
      request->translationA =
          SHAPECAST_PARALLEL_EXPLORE_DIST * request->translationA;
      request->translationB =
          SHAPECAST_PARALLEL_EXPLORE_DIST * request->translationB;
    } else {
      get_cross_point_for_two_ray(
          cross_pt, request->proxyA.vertices[0], request->translationA,
          request->proxyB.vertices[0], request->translationB);

      request->translationA = cross_pt - request->proxyA.vertices[0];
      request->translationB = cross_pt - request->proxyB.vertices[0];
    }
  }
  gjk_solver.shapeCast(result, request);

  if (result->collided) {
    *is_collision = true;
    *distA = result->distanceA;
    *distB = result->distanceB;
    collision_pointA->x = result->collision_pointA(0);
    collision_pointA->y = result->collision_pointA(1);
    collision_pointB->x = result->collision_pointB(0);
    collision_pointB->y = result->collision_pointB(1);
  } else {
    *is_collision = false;
  }

  return;
}

void GJK2DInterface::Raycast(RaycastCollisionInfo *info,
                             Position2D *collision_point,
                             double *collision_dist, Position2D *source,
                             Position2D *direction, double max_lambda,
                             const Polygon2D *polygon, bool get_collision_pt,
                             bool get_collision_dist, bool normalized_dir) {
  int32_t i;
  cdl::RayCastRequest input_tmp;
  cdl::RayCastRequest *input = &input_tmp;
  cdl::GJK2D gjk_solver;
  cdl::RayCastResult raycast_result_tmp;
  cdl::RayCastResult *raycast_result = &raycast_result_tmp;

  if (get_collision_pt && NULL == collision_point) {
    return;
  }
  if (get_collision_dist && NULL == collision_dist) {
    return;
  }
  input->source(0) = source->x;
  input->source(1) = source->y;
  input->direction(0) = direction->x;
  input->direction(1) = direction->y;
  input->max_lambda = max_lambda;
  input->shape.size = polygon->vertex_num;

  if (get_collision_dist && !normalized_dir) {
    input->direction.normalize();
  }

  for (i = 0; i < polygon->vertex_num; ++i) {
    input->shape.vertices[i](0) = polygon->vertexes[i].x;
    input->shape.vertices[i](1) = polygon->vertexes[i].y;
  }

  gjk_solver.traversalRayCast(input, raycast_result);

  if (cdl::RayCastStatus::HIT == raycast_result->status) {
    *info = RAYCAST_HIT_POLYGON;
  } else if (cdl::RayCastStatus::SOURCE_IS_INSIDE == raycast_result->status) {
    *info = RAYCAST_SOURCE_POINT_IN_POLYGON;
  } else {
    *info = RAYCAST_NO_COLLISION;
  }
  if (get_collision_dist) {
    *collision_dist = raycast_result->lambda_hit;
  }
  if (get_collision_pt) {
    collision_point->x = raycast_result->hit_spot(0);
    collision_point->y = raycast_result->hit_spot(1);
  }
  return;
}

void GJK2DInterface::PolygonPointCollisionDetect(const Polygon2D *polygon,
                                                 const Eigen::Vector2d &point,
                                                 bool *is_collision) {
  double d, delta_d;
  d = polygon->center_pt.DistanceTo(point);
  delta_d = d - polygon->radius;

  if (!ifly_fless(delta_d, 0.01)) {
    *is_collision = false;
  } else if (d <= polygon->min_tangent_radius) {
    *is_collision = true;
  } else {
    int32_t i;

    shape_p_.size = polygon->vertex_num;
    for (i = 0; i < polygon->vertex_num; ++i) {
      shape_p_.vertices[i][0] = polygon->vertexes[i].x;
      shape_p_.vertices[i][1] = polygon->vertexes[i].y;
    }

    shape_q_.size = 1;
    shape_q_.vertices[0][0] = point[0];
    shape_q_.vertices[0][1] = point[1];

    bool result = gjk_solver_.Collision(shape_p_, shape_q_);
    if (result) {
      *is_collision = true;
    } else {
      *is_collision = false;
    }
  }

  return;
}

}  // namespace planning
