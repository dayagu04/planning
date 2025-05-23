#include "node_collision_detect.h"

#include <cmath>

#include "hybrid_astar_request.h"
#include "log_glog.h"
#include "math/math_utils.h"
#include "obstacle.h"

namespace planning {

NodeCollisionDetect::NodeCollisionDetect(const ParkObstacleList* obstacles,
                                         EulerDistanceTransform* edt,
                                         const ObstacleClearZone* clear_zone,
                                         const MapBound* XYbounds,
                                         const AstarRequest* request)
    : obstacles_(obstacles),
      edt_(edt),
      clear_zone_(clear_zone),
      XYbounds_(XYbounds),
      request_(request) {}

const bool NodeCollisionDetect::IsPointBeyondBound(const float x,
                                                   const float y) const {
  if (x > XYbounds_->x_max || x < XYbounds_->x_min || y > XYbounds_->y_max ||
      y < XYbounds_->y_min) {
    return true;
  }
  return false;
}

const bool NodeCollisionDetect::IsFootPrintCollision(const Transform2d& tf) {
  Polygon2D veh_global_polygon;
  bool is_collision = false;
  ULFLocalPolygonToGlobal(&veh_global_polygon,
                          &cvx_hull_foot_print_.max_polygon, tf);

  is_collision = IsPolygonCollision(&veh_global_polygon);
  if (!is_collision) {
    return false;
  }

  ULFLocalPolygonToGlobal(&veh_global_polygon, &cvx_hull_foot_print_.body, tf);
  is_collision = IsPolygonCollision(&veh_global_polygon);
  if (is_collision) {
    return true;
  }

  ULFLocalPolygonToGlobal(&veh_global_polygon,
                          &cvx_hull_foot_print_.mirror_left, tf);
  // ILOG_INFO << "left mirror check";
  is_collision = IsPolygonCollision(&veh_global_polygon);
  if (is_collision) {
    return true;
  }

  ULFLocalPolygonToGlobal(&veh_global_polygon,
                          &cvx_hull_foot_print_.mirror_right, tf);

  is_collision = IsPolygonCollision(&veh_global_polygon);
  if (is_collision) {
    return true;
  }

  return false;
}

bool NodeCollisionDetect::ValidityCheckByConvex(Node3d* node) {
  if (node == nullptr) {
    return false;
  }

  if (node->GetStepSize() <= 0) {
    return false;
  }

  if (obstacles_->IsEmpty()) {
    return true;
  }

  size_t node_step_size = node->GetStepSize();
  const NodePath& path = node->GetNodePath();

  // The first {x, y, phi} is collision free unless they are start and end
  // configuration of search problem
  size_t check_start_index = 0;
  if (node_step_size == 1) {
    check_start_index = 0;
  } else {
    check_start_index = 1;
  }

#if LOG_TIME_PROFILE
  double check_start_time = IflyTime::Now_ms();
#endif

  Polygon2D global_polygon;
  Pose2D global_pose;
  bool is_collision = false;
  cdl::AABB path_point_aabb;

  Polygon2D* veh_local_polygon = GetVehPolygon(node->GetGearType());

  for (size_t i = check_start_index; i < node_step_size; ++i) {
    // check bound
    if (IsPointBeyondBound(path.points[i].x, path.points[i].y)) {
      node->SetCollisionType(NodeCollisionType::MAP_BOUND);
      return false;
    }

    global_pose = path.points[i];

    RULocalPolygonToGlobal(&global_polygon, veh_local_polygon, &global_pose);

    GetBoundingBoxByPolygon(&path_point_aabb, &global_polygon);
    if (clear_zone_->IsContain(path_point_aabb)) {
      // ILOG_INFO << "clear";
      continue;
    }

    for (const auto& obstacle : obstacles_->virtual_obs) {
      gjk_interface_.PolygonPointCollisionDetect(&is_collision, &global_polygon,
                                                 obstacle);

      if (is_collision) {
        node->SetCollisionType(NodeCollisionType::VIRTUAL_WALL);
        return false;
      }
    }

    for (const auto& obstacle : obstacles_->point_cloud_list) {
      // envelop box check
      gjk_interface_.PolygonCollisionByCircleCheck(
          &is_collision, &obstacle.envelop_polygon, &global_polygon, 0.01);

      if (!is_collision) {
        continue;
      }

      // internal points
      for (size_t j = 0; j < obstacle.points.size(); j++) {
        gjk_interface_.PolygonPointCollisionDetect(
            &is_collision, &global_polygon, obstacle.points[j]);

        if (is_collision) {
          node->SetCollisionType(NodeCollisionType::FUSION_OCC_OBS);
          return false;
        }
      }
    }
  }

#if LOG_TIME_PROFILE
  double check_end_time = IflyTime::Now_ms();
  collision_check_time_ms_ += check_end_time - check_start_time;

#endif

  return true;
}

const bool NodeCollisionDetect::IsPolygonCollision(const Polygon2D* polygon) {
  bool is_collision = false;
  for (const auto& obstacle : obstacles_->point_cloud_list) {
    // envelop box check
    gjk_interface_.PolygonCollisionByCircleCheck(
        &is_collision, &obstacle.envelop_polygon, polygon, 0.001);

    if (!is_collision) {
      continue;
    }

    // internal points
    for (size_t j = 0; j < obstacle.points.size(); j++) {
      gjk_interface_.PolygonPointCollisionDetect(&is_collision, polygon,
                                                 obstacle.points[j]);

      if (is_collision) {
        // ILOG_INFO << "size = " << obstacle.points.size() << " j =" << j;
        return true;
      }

      // ILOG_INFO << "point no collision";
    }
  }

  for (const auto& obstacle : obstacles_->virtual_obs) {
    gjk_interface_.PolygonPointCollisionDetect(&is_collision, polygon,
                                               obstacle);

    if (is_collision) {
      return true;
    }
  }

  return false;
}

bool NodeCollisionDetect::RSPathCollisionCheck(const RSPath* reeds_shepp_to_end,
                                               Node3d* rs_node_to_goal) {
  // length check
  if (reeds_shepp_to_end->size < 1) {
    return false;
  }

  // collision check
#if LOG_TIME_PROFILE
  double check_start_time = IflyTime::Now_ms();
#endif

  bool is_valid = IsRSPathSafeByEDT(reeds_shepp_to_end, rs_node_to_goal);
  // if (!is_valid) {
  // ILOG_INFO << "rs collision "
  //           << static_cast<int>(rs_end_node_.GetConstCollisionType());
  // }

#if LOG_TIME_PROFILE
  double check_end_time = IflyTime::Now_ms();
  collision_check_time_ms_ += check_end_time - check_start_time;
#endif

  return is_valid;
}

const bool NodeCollisionDetect::ValidityCheckByEDT(Node3d* node) {
  if (node == nullptr) {
    return false;
  }

  if (node->GetStepSize() <= 0) {
    return false;
  }

  node->SetDistToObs(0.0f);
  const NodePath& path = node->GetNodePath();

  // The first {x, y, phi} is collision free unless they are start and end
  // configuration of search problem
  size_t node_step_size = node->GetStepSize();
  size_t check_start_index = 0;
  if (node_step_size == 1) {
    check_start_index = 0;
  } else {
    check_start_index = 1;
  }

  Polygon2D global_polygon;
  Pose2D global_pose;
  cdl::AABB path_point_aabb;
  Transform2d tf;
  AstarPathGear node_gear = node->GetGearType();
  AstarPathGear point_gear;
  Polygon2D* veh_local_polygon = GetVehPolygon(node_gear);
  bool is_circle_path = IsCirclePathBySteeringWheel(node->GetSteer());
  FootPrintCircleModel* footprint_model =
      GetCircleFootPrintModel(path.points[0], is_circle_path);

  float dist = 100.0;
  float min_dist = 100.0;

  for (size_t i = check_start_index; i < node_step_size; ++i) {
    // check bound
    if (IsPointBeyondBound(path.points[i].x, path.points[i].y)) {
      node->SetCollisionType(NodeCollisionType::MAP_BOUND);
      return false;
    }

    global_pose = path.points[i];
    tf.SetBasePose(global_pose);

    RULocalPolygonToGlobalFast(&global_polygon, veh_local_polygon, &global_pose,
                               tf.GetCosTheta(), tf.GetSinTheta());

    GetBoundingBoxByPolygon(&path_point_aabb, &global_polygon);
    if (clear_zone_->IsContain(path_point_aabb)) {
      // ILOG_INFO << "clear";
      continue;
    }

    if (i == node_step_size - 1) {
      point_gear = node_gear;
    } else {
      point_gear = AstarPathGear::NONE;
    }

// for accelerate calculation, use macro
#if ENABLE_OBS_DIST_G_COST
    if (edt_->DistanceCheckForPoint(&dist, &tf, point_gear)) {
      node->SetCollisionType(NodeCollisionType::FUSION_OCC_OBS);
      node->SetDistToObs(dist);
      // node->SetCollisionID(i);

      return false;
    }

    if (dist < min_dist) {
      min_dist = dist;
    }
#else

    if (edt_->IsCollisionForPoint(&tf, point_gear, footprint_model)) {
      node->SetCollisionType(NodeCollisionType::FUSION_OCC_OBS);
      // node->SetCollisionID(i);

      return false;
    }

#endif

    // ILOG_INFO << "path size " << node_step_size << " ,pt id " << i
    //           << " , no collision ";
  }

  node->SetDistToObs(min_dist);

  return true;
}

bool NodeCollisionDetect::IsRSPathSafeByConvexHull(
    const RSPath* reeds_shepp_path, Node3d* node) {
  if (reeds_shepp_path == nullptr) {
    return false;
  }

  if (reeds_shepp_path->size <= 0) {
    return true;
  }

  if (obstacles_->IsEmpty()) {
    return true;
  }

  size_t point_size;
  // The first {x, y, phi} is collision free unless they are start and end
  // configuration of search problem
  size_t check_start_index = 0;

  Polygon2D global_polygon;
  Pose2D global_pose;
  bool is_collision;
  cdl::AABB path_point_aabb;
  Polygon2D* veh_local_polygon = nullptr;

  for (int seg_id = 0; seg_id < reeds_shepp_path->size; seg_id++) {
    const RSPathSegment* segment = &reeds_shepp_path->paths[seg_id];

    point_size = segment->size;
    if (point_size == 1) {
      check_start_index = 0;
    } else {
      check_start_index = 1;
    }

    for (size_t i = check_start_index; i < point_size; ++i) {
      // check bound
      if (IsPointBeyondBound(segment->points[i].x, segment->points[i].y)) {
        node->SetCollisionType(NodeCollisionType::MAP_BOUND);
        return false;
      }

      global_pose.x = segment->points[i].x;
      global_pose.y = segment->points[i].y;
      global_pose.theta = segment->points[i].theta;

      veh_local_polygon = GetVehPolygon(segment->gear);

      RULocalPolygonToGlobal(&global_polygon, veh_local_polygon, &global_pose);

      GetBoundingBoxByPolygon(&path_point_aabb, &global_polygon);
      if (clear_zone_->IsContain(path_point_aabb)) {
        // ILOG_INFO << "clear";
        continue;
      }

      for (const auto& obstacle : obstacles_->virtual_obs) {
        gjk_interface_.PolygonPointCollisionDetect(&is_collision,
                                                   &global_polygon, obstacle);

        if (is_collision) {
          node->SetCollisionType(NodeCollisionType::VIRTUAL_WALL);
          return false;
        }
      }

      for (const auto& obstacle : obstacles_->point_cloud_list) {
        // envelop box check
        gjk_interface_.PolygonCollisionByCircleCheck(
            &is_collision, &obstacle.envelop_polygon, &global_polygon, 0.01);

        if (!is_collision) {
          continue;
        }

        // internal points
        for (size_t j = 0; j < obstacle.points.size(); j++) {
          gjk_interface_.PolygonPointCollisionDetect(
              &is_collision, &global_polygon, obstacle.points[j]);

          if (is_collision) {
            node->SetCollisionType(NodeCollisionType::FUSION_OCC_OBS);
            return false;
          }
        }
      }
      //
    }
  }

  return true;
}

const bool NodeCollisionDetect::IsRSPathSafeByEDT(
    const RSPath* reeds_shepp_path, Node3d* node) {
  if (reeds_shepp_path == nullptr) {
    return false;
  }

  if (reeds_shepp_path->size <= 0) {
    return true;
  }

  size_t point_size;

  // The first {x, y, phi} is collision free unless they are start and end
  // configuration of search problem
  size_t check_start_index = 0;

  Polygon2D global_polygon;
  Pose2D global_pose;
  // bool is_collision;
  cdl::AABB path_point_aabb;
  Transform2d tf;

  Polygon2D* veh_local_polygon = nullptr;
  AstarPathGear point_gear;
  bool is_circle_path;

  for (int seg_id = 0; seg_id < reeds_shepp_path->size; seg_id++) {
    const RSPathSegment* segment = &reeds_shepp_path->paths[seg_id];

    point_size = segment->size;
    if (point_size == 1) {
      check_start_index = 0;
    } else {
      check_start_index = 1;
    }
    is_circle_path = IsCirclePathByKappa(segment->kappa);

    for (size_t i = check_start_index; i < point_size; ++i) {
      // check bound
      if (IsPointBeyondBound(segment->points[i].x, segment->points[i].y)) {
        node->SetCollisionType(NodeCollisionType::MAP_BOUND);
        return false;
      }

      global_pose.x = segment->points[i].x;
      global_pose.y = segment->points[i].y;
      global_pose.theta = segment->points[i].theta;

      tf.SetBasePose(global_pose);

      veh_local_polygon = GetVehPolygon(segment->gear);

      RULocalPolygonToGlobalFast(&global_polygon, veh_local_polygon,
                                 &global_pose, tf.GetCosTheta(),
                                 tf.GetSinTheta());

      GetBoundingBoxByPolygon(&path_point_aabb, &global_polygon);
      if (clear_zone_->IsContain(path_point_aabb)) {
        continue;
      }

      if (i == point_size - 1) {
        point_gear = segment->gear;
      } else {
        point_gear = AstarPathGear::NONE;
      }

      if (edt_->IsCollisionForPoint(
              &tf, point_gear,
              GetCircleFootPrintModel(global_pose, is_circle_path))) {
        node->SetCollisionType(NodeCollisionType::FUSION_OCC_OBS);
        return false;
      }
    }
  }

  return true;
}

const bool NodeCollisionDetect::IsPolynomialPathSafeByEDT(
    const std::vector<AStarPathPoint>& path, Node3d* node) {
  if (path.size() <= 0) {
    return true;
  }

  size_t point_size = path.size();

  // The first {x, y, phi} is collision free unless they are start and end
  // configuration of search problem
  size_t check_start_index = 0;

  Polygon2D global_polygon;
  Pose2D global_pose;
  // bool is_collision;
  cdl::AABB path_point_aabb;
  Transform2d tf;

  Polygon2D* veh_local_polygon = nullptr;
  AstarPathGear point_gear;

  for (size_t i = check_start_index; i < point_size; ++i) {
    // check bound
    if (IsPointBeyondBound(path[i].x, path[i].y)) {
      node->SetCollisionType(NodeCollisionType::MAP_BOUND);
      return false;
    }

    global_pose.x = path[i].x;
    global_pose.y = path[i].y;
    global_pose.theta = path[i].phi;

    tf.SetBasePose(global_pose);

    veh_local_polygon = GetVehPolygon(path[i].gear);
    // ILOG_INFO << "gear " << PathGearDebugString(segment->gear);

    RULocalPolygonToGlobalFast(&global_polygon, veh_local_polygon, &global_pose,
                               tf.GetCosTheta(), tf.GetSinTheta());

    GetBoundingBoxByPolygon(&path_point_aabb, &global_polygon);
    if (clear_zone_->IsContain(path_point_aabb)) {
      // ILOG_INFO << "clear";
      continue;
    }

    if (i == point_size - 1) {
      point_gear = path[i].gear;
    } else {
      point_gear = AstarPathGear::NONE;
    }

    if (edt_->IsCollisionForPoint(
            &tf, point_gear, GetCircleFootPrintModel(global_pose, false))) {
      node->SetCollisionType(NodeCollisionType::FUSION_OCC_OBS);
      return false;
    }
  }

  return true;
}

size_t NodeCollisionDetect::GetPathCollisionIndex(HybridAStarResult* result) {
  if (result == nullptr) {
    return 0;
  }

  if (result->x.size() <= 0) {
    return 0;
  }

  size_t path_end_id = result->x.size() - 1;

  if (obstacles_->point_cloud_list.empty() && obstacles_->virtual_obs.empty()) {
    return path_end_id;
  }

  Polygon2D polygon;
  Pose2D global_pose;
  bool is_collision;

  size_t collision_index = 100000;
  cdl::AABB path_point_aabb;

  Polygon2D* veh_local_polygon = nullptr;

  for (size_t i = 0; i <= path_end_id; ++i) {
    // check bound
    if (IsPointBeyondBound(result->x[i], result->y[i])) {
      collision_index = i;

#if DEBUG_GJK
      ILOG_INFO << "i=" << i << "beyond bound";
#endif
      break;
    }

    global_pose.x = result->x[i];
    global_pose.y = result->y[i];
    global_pose.theta = result->phi[i];

    veh_local_polygon = GetVehPolygon(result->gear[i]);
    RULocalPolygonToGlobal(&polygon, veh_local_polygon, &global_pose);

    GetBoundingBoxByPolygon(&path_point_aabb, &polygon);
    if (clear_zone_->IsContain(path_point_aabb)) {
#if DEBUG_GJK
      ILOG_INFO << "i=" << i << "clear";
#endif
      continue;
    }

    for (const auto& obstacle : obstacles_->point_cloud_list) {
      // envelop box check
      gjk_interface_.PolygonCollisionByCircleCheck(
          &is_collision, &obstacle.envelop_polygon, &polygon, 0.01);

      if (!is_collision) {
        continue;
      }

      // internal points
      for (size_t j = 0; j < obstacle.points.size(); j++) {
        gjk_interface_.PolygonPointCollisionDetect(&is_collision, &polygon,
                                                   obstacle.points[j]);

        if (is_collision) {
          collision_index = i;

#if DEBUG_GJK
          ILOG_INFO << "path id=" << i
                    << ",obs type=" << static_cast<int>(obstacle.obs_type)
                    << ",obs size=" << obstacle.points.size();
#endif
          return collision_index;
        }
      }
    }
  }

  return collision_index;
}

size_t NodeCollisionDetect::GetPathCollisionIDByEDT(HybridAStarResult* result) {
  if (result == nullptr) {
    return 0;
  }

  if (result->x.size() <= 0) {
    return 0;
  }

  size_t path_end_id = result->x.size() - 1;

  if (obstacles_->point_cloud_list.empty() && obstacles_->virtual_obs.empty()) {
    return path_end_id;
  }

  Polygon2D polygon;
  Pose2D global_pose;

  size_t collision_index = 100000;
  cdl::AABB path_point_aabb;
  Polygon2D* veh_local_polygon = nullptr;
  Transform2d tf;

  for (size_t i = 0; i <= path_end_id; ++i) {
    // check bound
    if (IsPointBeyondBound(result->x[i], result->y[i])) {
      collision_index = i;

      break;
    }

    global_pose.x = result->x[i];
    global_pose.y = result->y[i];
    global_pose.theta = result->phi[i];
    tf.SetBasePose(global_pose);

    veh_local_polygon = GetVehPolygon(result->gear[i]);
    RULocalPolygonToGlobalFast(&polygon, veh_local_polygon, &global_pose,
                               tf.GetCosTheta(), tf.GetSinTheta());

    GetBoundingBoxByPolygon(&path_point_aabb, &polygon);
    if (clear_zone_->IsContain(path_point_aabb)) {
      continue;
    }

    if (edt_->IsCollisionForPoint(
            &tf, result->gear[i],
            GetCircleFootPrintModel(global_pose, false))) {
      return i;
    }
  }

  return collision_index;
}

size_t NodeCollisionDetect::GetPathCollisionIDByEDT(
    const std::vector<AStarPathPoint>& poly_path) {
  if (poly_path.empty()) {
    return 0;
  }

  size_t path_end_id = poly_path.size() - 1;

  if (obstacles_->point_cloud_list.empty() && obstacles_->virtual_obs.empty()) {
    return path_end_id;
  }

  Polygon2D polygon;
  Pose2D global_pose;

  size_t collision_index = 100000;
  cdl::AABB path_point_aabb;
  Polygon2D* veh_local_polygon = nullptr;
  Transform2d tf;

  for (size_t i = 0; i <= path_end_id; ++i) {
    // check bound
    if (IsPointBeyondBound(poly_path[i].x, poly_path[i].y)) {
      collision_index = i;

      break;
    }

    global_pose.x = poly_path[i].x;
    global_pose.y = poly_path[i].y;
    global_pose.theta = poly_path[i].phi;
    tf.SetBasePose(global_pose);

    veh_local_polygon = GetVehPolygon(poly_path[i].gear);
    RULocalPolygonToGlobalFast(&polygon, veh_local_polygon, &global_pose,
                               tf.GetCosTheta(), tf.GetSinTheta());

    GetBoundingBoxByPolygon(&path_point_aabb, &polygon);
    if (clear_zone_->IsContain(path_point_aabb)) {
      continue;
    }

    if (edt_->IsCollisionForPoint(
            &tf, poly_path[i].gear,
            GetCircleFootPrintModel(global_pose,
                                    IsCirclePathByKappa(poly_path[i].kappa)))) {
      return i;
    }
  }

  return collision_index;
}

Polygon2D* NodeCollisionDetect::GetVehPolygon(const AstarPathGear& gear) {
  if (gear == AstarPathGear::DRIVE) {
    return &veh_box_gear_drive_;
  } else if (gear == AstarPathGear::REVERSE) {
    return &veh_box_gear_reverse_;
  }

  return &veh_box_gear_none_;
}

void NodeCollisionDetect::DebugEDTCheck(HybridAStarResult* path) {
  if (path == nullptr || path->x.size() < 1) {
    return;
  }

  Pose2D global_pose;
  // bool is_collision = false;
  Transform2d tf;
  AstarPathGear gear = AstarPathGear::NONE;

  float min_dist = 100;
  float dist;
  size_t point_size = path->x.size();
  for (size_t i = 0; i < point_size; ++i) {
    global_pose.x = path->x[i];
    global_pose.y = path->y[i];
    global_pose.theta = path->phi[i];
    tf.SetBasePose(global_pose);

    if (edt_->DistanceCheckForPoint(&dist, &tf, gear)) {
      ILOG_INFO << "collision";
    }

    min_dist = std::min(dist, min_dist);
    ILOG_INFO << "path size " << point_size << ", pt id " << i
              << ", dist= " << dist;
  }

  ILOG_INFO << "min_dist = " << min_dist;

  return;
}

FootPrintCircleModel* NodeCollisionDetect::GetCircleFootPrintModel(
    const Pose2D& pose, const bool is_circle_path) {
  // 60 degree
  if (slot_box_.contain(pose) &&
      std::fabs(IflyUnifyTheta(pose.theta - request_->goal_.theta, M_PI)) <
          1.05) {
    if (is_circle_path) {
      return &hierachy_circle_model_.footprint_model
                  [HierarchySafeBuffer::CIRCLE_PATH_INSIDE_SLOT_BUFFER];
    }

    return &hierachy_circle_model_
                .footprint_model[HierarchySafeBuffer::INSIDE_SLOT_BUFFER];
  }

  if (is_circle_path) {
    return &hierachy_circle_model_.footprint_model
                [HierarchySafeBuffer::CIRCLE_PATH_OUTSIDE_SLOT_BUFFER];
  }
  return &hierachy_circle_model_
              .footprint_model[HierarchySafeBuffer::OUTSIDE_SLOT_BUFFER];
}

FootPrintCircleModel* NodeCollisionDetect::GetSlotOutsideCircleFootPrint() {
  return &hierachy_circle_model_
              .footprint_model[HierarchySafeBuffer::OUTSIDE_SLOT_BUFFER];
}

void NodeCollisionDetect::UpdateFootPrintBySafeBuffer(
    const float lat_buffer_outside, const float lat_buffer_inside,
    const float lon_buffer, const VehicleParam& vehicle_param,
    const PlannerOpenSpaceConfig& config) {
  if (request_->direction_request == ParkingVehDirection::HEAD_IN) {
    slot_box_ = cdl::AABB(
        cdl::Vector2r(0.0f, -request_->slot_width / 2),
        cdl::Vector2r(request_->slot_length + (float)vehicle_param.length,
                      request_->slot_width / 2));
  } else {
    slot_box_ = cdl::AABB(
        cdl::Vector2r(0.0f, -request_->slot_width / 2),
        cdl::Vector2r(request_->slot_length + 1.5, request_->slot_width / 2));
  }

  slot_box_.DebugString();

  // gear d
  float safe_half_width =
      (vehicle_param.max_width + lat_buffer_outside * 2 + 0.1) * 0.5;

  GetRightUpCoordinatePolygonByParam(
      &veh_box_gear_drive_,
      vehicle_param.rear_edge_to_rear_axle +
          config.safe_buffer.lon_min_safe_buffer,
      vehicle_param.wheel_base + vehicle_param.front_overhanging + lon_buffer,
      safe_half_width);

  // gear r
  GetRightUpCoordinatePolygonByParam(
      &veh_box_gear_reverse_, vehicle_param.rear_edge_to_rear_axle + lon_buffer,
      vehicle_param.wheel_base + vehicle_param.front_overhanging +
          config.safe_buffer.lon_min_safe_buffer,
      safe_half_width);

  // gear none
  GetRightUpCoordinatePolygonByParam(&veh_box_gear_none_,
                                     vehicle_param.rear_edge_to_rear_axle +
                                         config.safe_buffer.lon_min_safe_buffer,
                                     vehicle_param.wheel_base +
                                         vehicle_param.front_overhanging +
                                         config.safe_buffer.lon_min_safe_buffer,
                                     safe_half_width);

  GenerateVehCompactPolygon(lat_buffer_inside,
                            config.safe_buffer.lon_min_safe_buffer,
                            &cvx_hull_foot_print_);

  // PolygonDebugString(&veh_box_gear_drive_, "drive");
  // PolygonDebugString(&veh_box_gear_reverse_, "reverse");
  // PolygonDebugString(&veh_box_gear_none_, "none gear");
  // PolygonDebugString(&cvx_hull_foot_print_.body, "body");
  // PolygonDebugString(&cvx_hull_foot_print_.mirror_left, "left mirror");
  // PolygonDebugString(&cvx_hull_foot_print_.mirror_right, "right mirror");

  hierachy_circle_model_
      .footprint_model[HierarchySafeBuffer::OUTSIDE_SLOT_BUFFER]
      .UpdateSafeBuffer(lat_buffer_outside, lon_buffer, lat_buffer_outside);
  hierachy_circle_model_
      .footprint_model[HierarchySafeBuffer::INSIDE_SLOT_BUFFER]
      .UpdateSafeBuffer(lat_buffer_inside, lon_buffer, lat_buffer_inside);

  float lat_buffer =
      lat_buffer_outside + config.safe_buffer.circle_path_extra_buffer_outside;
  hierachy_circle_model_
      .footprint_model[HierarchySafeBuffer::CIRCLE_PATH_OUTSIDE_SLOT_BUFFER]
      .UpdateSafeBuffer(lat_buffer, lon_buffer, lat_buffer);

  lat_buffer =
      lat_buffer_inside + config.safe_buffer.circle_path_extra_buffer_inside;
  hierachy_circle_model_
      .footprint_model[HierarchySafeBuffer::CIRCLE_PATH_INSIDE_SLOT_BUFFER]
      .UpdateSafeBuffer(lat_buffer, lon_buffer, lat_buffer);

  ILOG_INFO << "outside buffer = " << lat_buffer_outside
            << ", inside buffer = " << lat_buffer_inside;

  return;
}

}  // namespace planning