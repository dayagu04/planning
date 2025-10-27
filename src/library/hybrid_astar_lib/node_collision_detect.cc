#include "node_collision_detect.h"

#include <cmath>

#include "hybrid_astar_common.h"
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
      grid_map_bound_(XYbounds),
      request_(request) {}

const bool NodeCollisionDetect::IsPointOutOfGridMapBound(const float x,
                                                         const float y) const {
  if (x > grid_map_bound_->x_max || x < grid_map_bound_->x_min ||
      y > grid_map_bound_->y_max || y < grid_map_bound_->y_min) {
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

bool NodeCollisionDetect::IsValidByConvexHull(Node3d* node) {
  if (node == nullptr) {
    return false;
  }

  if (node->GetStepSize() <= 0) {
    return false;
  }

  if (obstacles_->IsEmpty()) {
    return true;
  }

  int node_step_size = node->GetStepSize();
  const NodePath& path = node->GetNodePath();

  // The first {x, y, phi} is collision free unless they are start and end
  // configuration of search problem
  int check_start_index = 0;
  if (node_step_size == 1) {
    check_start_index = 0;
  } else {
    check_start_index = 1;
  }

#if LOG_TIME_PROFILE
  double check_start_time = IflyTime::Now_ms();
#endif

  Pose2D global_pose;
  Transform2d tf;
  float sin_theta = 0.0f;
  float cos_theta = 0.0f;
  if (node->IsStraight()) {
    sin_theta = std::sin(node->GetPose().theta);
    cos_theta = std::cos(node->GetPose().theta);
  }

  for (int i = check_start_index; i < node_step_size; ++i) {
    // check bound
    // if (IsPointOutOfGridMapBound(path.points[i].x, path.points[i].y)) {
    //   node->SetCollisionType(NodeCollisionType::MAP_BOUND);
    //   return false;
    // }

    global_pose.x = path.points[i].x;
    global_pose.y = path.points[i].y;
    global_pose.theta = path.points[i].theta;

    if (node->IsStraight()) {
      tf.SetBasePose(global_pose, sin_theta, cos_theta);
    } else {
      tf.SetBasePose(global_pose);
    }

    if (IsFootPrintCollision(tf)) {
      return false;
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

const bool NodeCollisionDetect::IsValidByEDT(Node3d* node) {
  if (node == nullptr) {
    return false;
  }

  if (node->GetStepSize() <= 0) {
    return false;
  }

  const NodePath& path = node->GetNodePath();

  // The first {x, y, phi} is collision free unless they are start and end
  // configuration of search problem
  int node_step_size = node->GetStepSize();
  int check_start_index = 0;
  if (node_step_size == 1) {
    check_start_index = 0;
  } else {
    check_start_index = 1;
  }

  Pose2f global_pose;
  Transform2f tf;
  AstarPathGear point_gear = node->GetGearType();
  bool is_circle_path = !(node->IsStraight());
  FootPrintCircleModel* footprint_model =
      GetCircleFootPrintModel(path.points[0], is_circle_path);

  float sin_theta = 0.0f;
  float cos_theta = 0.0f;
  if (node->IsStraight()) {
    sin_theta = std::sin(node->GetPose().theta);
    cos_theta = std::cos(node->GetPose().theta);
  }

  float dist = 100.0f;
  float min_dist = 100.0f;

  for (int i = check_start_index; i < node_step_size; ++i) {
    // check bound, has checked this in node generator
    // if (IsPointOutOfGridMapBound(path.points[i].x, path.points[i].y)) {
    //   node->SetCollisionType(NodeCollisionType::MAP_BOUND);
    //   node->SetDistToObs(0.0f);
    //   return false;
    // }

    global_pose = path.points[i];

    if (node->IsStraight()) {
      tf.SetBasePose(global_pose, sin_theta, cos_theta);
    } else {
      tf.SetBasePose(global_pose);
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
      node->SetDistToObs(0.0f);

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

  int point_size;
  // The first {x, y, phi} is collision free unless they are start and end
  // configuration of search problem
  int check_start_index = 0;

  Pose2D global_pose;
  Transform2d tf;
  float sin_theta = 0.0f;
  float cos_theta = 0.0f;

  for (int seg_id = 0; seg_id < reeds_shepp_path->size; seg_id++) {
    const RSPathSegment* segment = &reeds_shepp_path->paths[seg_id];

    point_size = segment->size;
    if (point_size == 1) {
      check_start_index = 0;
    } else {
      check_start_index = 1;
    }

    if (segment->steer == RSPathSteer::RS_STRAIGHT) {
      sin_theta = std::sin(segment->points[0].theta);
      cos_theta = std::cos(segment->points[0].theta);
    }

    for (int i = check_start_index; i < point_size; ++i) {
      // check bound
      if (IsPointOutOfGridMapBound(segment->points[i].x,
                                   segment->points[i].y)) {
        node->SetCollisionType(NodeCollisionType::MAP_BOUND);
        return false;
      }

      global_pose.x = segment->points[i].x;
      global_pose.y = segment->points[i].y;
      global_pose.theta = segment->points[i].theta;

      if (segment->steer == RSPathSteer::RS_STRAIGHT) {
        tf.SetBasePose(global_pose, sin_theta, cos_theta);
      } else {
        tf.SetBasePose(global_pose);
      }

      if (IsFootPrintCollision(tf)) {
        return false;
      }
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

  int point_size;

  // The first {x, y, phi} is collision free unless they are start and end
  // configuration of search problem
  int check_start_index = 0;

  Pose2f global_pose;
  // bool is_collision;
  Transform2f tf;

  AstarPathGear point_gear;
  bool is_circle_path;
  float sin_theta = 0.0f;
  float cos_theta = 0.0f;

  for (int seg_id = 0; seg_id < reeds_shepp_path->size; seg_id++) {
    const RSPathSegment* segment = &reeds_shepp_path->paths[seg_id];

    point_size = segment->size;
    if (point_size == 1) {
      check_start_index = 0;
    } else {
      check_start_index = 1;
    }
    is_circle_path = segment->steer == RSPathSteer::RS_STRAIGHT ? false : true;
    if (!is_circle_path) {
      sin_theta = std::sin(segment->points[0].theta);
      cos_theta = std::cos(segment->points[0].theta);
    }

    point_gear = segment->gear;
    for (int i = check_start_index; i < point_size; ++i) {
      // check bound
      if (IsPointOutOfGridMapBound(segment->points[i].x,
                                   segment->points[i].y)) {
        node->SetCollisionType(NodeCollisionType::MAP_BOUND);
        return false;
      }

      global_pose.x = segment->points[i].x;
      global_pose.y = segment->points[i].y;
      global_pose.theta = segment->points[i].theta;

      if (!is_circle_path) {
        tf.SetBasePose(global_pose, sin_theta, cos_theta);
      } else {
        tf.SetBasePose(global_pose);
      }

      if (edt_->IsCollisionForPoint(
              &tf, point_gear,
              GetCircleFootPrintModel(global_pose, is_circle_path))) {
        // node->SetCollisionType(NodeCollisionType::FUSION_OCC_OBS);
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

  int point_size = path.size();

  // The first {x, y, phi} is collision free unless they are start and end
  // configuration of search problem
  int check_start_index = 0;
  Pose2f global_pose;
  // bool is_collision;
  Transform2f tf;
  AstarPathGear point_gear;

  for (int i = check_start_index; i < point_size; ++i) {
    // check bound
    if (IsPointOutOfGridMapBound(path[i].x, path[i].y)) {
      node->SetCollisionType(NodeCollisionType::MAP_BOUND);
      return false;
    }

    global_pose.x = path[i].x;
    global_pose.y = path[i].y;
    global_pose.theta = path[i].phi;

    tf.SetBasePose(global_pose);

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

int NodeCollisionDetect::GetPathCollisionIndex(HybridAStarResult* result) {
  if (result == nullptr) {
    return 0;
  }

  if (result->x.size() <= 0) {
    return 0;
  }

  int path_end_id = result->x.size() - 1;
  int collision_id = 10000;

  if (obstacles_->point_cloud_list.empty() && obstacles_->virtual_obs.empty()) {
    return collision_id;
  }

  Pose2D global_pose;
  Transform2d tf;

  for (int i = 0; i <= path_end_id; ++i) {
    // check bound
    if (IsPointOutOfGridMapBound(result->x[i], result->y[i])) {
      return i;
    }

    global_pose.x = result->x[i];
    global_pose.y = result->y[i];
    global_pose.theta = result->phi[i];
    tf.SetBasePose(global_pose);
    if (IsFootPrintCollision(tf)) {
      return i;
    }
  }

  return collision_id;
}

int NodeCollisionDetect::GetPathCollisionIDByEDT(HybridAStarResult* result) {
  if (result == nullptr) {
    return 0;
  }

  if (result->x.size() <= 0) {
    return 0;
  }

  int path_end_id = result->x.size() - 1;
  int collision_id = 10000;

  if (obstacles_->point_cloud_list.empty() && obstacles_->virtual_obs.empty()) {
    return collision_id;
  }

  Pose2f global_pose;
  Transform2f tf;

  for (int i = 0; i <= path_end_id; ++i) {
    // check bound
    if (IsPointOutOfGridMapBound(result->x[i], result->y[i])) {
      return i;
    }

    global_pose.x = result->x[i];
    global_pose.y = result->y[i];
    global_pose.theta = result->phi[i];
    tf.SetBasePose(global_pose);

    if (edt_->IsCollisionForPoint(
            &tf, result->gear[i],
            GetCircleFootPrintModel(global_pose, false))) {
      return i;
    }
  }

  return collision_id;
}

int NodeCollisionDetect::GetPathCollisionIDByEDT(
    const std::vector<AStarPathPoint>& poly_path) {
  if (poly_path.empty()) {
    return 0;
  }

  int path_end_id = poly_path.size() - 1;
  int collision_index = 10000;

  if (obstacles_->point_cloud_list.empty() && obstacles_->virtual_obs.empty()) {
    return collision_index;
  }

  Pose2f global_pose;
  Transform2f tf;

  for (int i = 0; i <= path_end_id; ++i) {
    // check bound
    if (IsPointOutOfGridMapBound(poly_path[i].x, poly_path[i].y)) {
      collision_index = i;

      break;
    }

    global_pose.x = poly_path[i].x;
    global_pose.y = poly_path[i].y;
    global_pose.theta = poly_path[i].phi;
    tf.SetBasePose(global_pose);

    if (edt_->IsCollisionForPoint(
            &tf, poly_path[i].gear,
            GetCircleFootPrintModel(global_pose,
                                    IsCirclePathByKappa(poly_path[i].kappa)))) {
      return i;
    }
  }

  return collision_index;
}

void NodeCollisionDetect::DebugEDTCheck(HybridAStarResult* path) {
  if (path == nullptr || path->x.size() < 1) {
    return;
  }

  Pose2f global_pose;
  // bool is_collision = false;
  Transform2f tf;
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
    const Pose2f& pose, const bool is_circle_path) {
  if (is_park_in_) {
    return GetCircleFootPrintModelForParkIn(pose, is_circle_path);
  }

  return GetCircleFootPrintModelForParkOut(pose, is_circle_path);
}

FootPrintCircleModel* NodeCollisionDetect::GetCircleFootPrintModelForParkIn(
    const Pose2f& pose, const bool is_circle_path) {
  bool inside_slot = slot_box_.contain(pose);
  // 40 degree
  bool theta_condition =
      std::fabs(IflyUnifyTheta(pose.theta - request_->goal.theta, M_PIf32)) <
      0.7f;

  if (inside_slot && theta_condition) {
    return &hierachy_circle_model_.footprint_model
                [is_circle_path
                     ? HierarchySafeBuffer::CIRCLE_PATH_INSIDE_SLOT_BUFFER
                     : HierarchySafeBuffer::INSIDE_SLOT_BUFFER];
  }

  return &hierachy_circle_model_.footprint_model
              [is_circle_path
                   ? HierarchySafeBuffer::CIRCLE_PATH_OUTSIDE_SLOT_BUFFER
                   : HierarchySafeBuffer::OUTSIDE_SLOT_BUFFER];
}

FootPrintCircleModel* NodeCollisionDetect::GetCircleFootPrintModelForParkOut(
    const Pose2f& pose, const bool is_circle_path) {
  bool inside_slot = slot_box_.contain(pose);
  if (inside_slot) {
    return &hierachy_circle_model_.footprint_model
                [is_circle_path
                     ? HierarchySafeBuffer::CIRCLE_PATH_INSIDE_SLOT_BUFFER
                     : HierarchySafeBuffer::INSIDE_SLOT_BUFFER];
  }

  return &hierachy_circle_model_.footprint_model
              [is_circle_path
                   ? HierarchySafeBuffer::CIRCLE_PATH_OUTSIDE_SLOT_BUFFER
                   : HierarchySafeBuffer::OUTSIDE_SLOT_BUFFER];
}

FootPrintCircleModel* NodeCollisionDetect::GetCircleFootPrint(
    const HierarchySafeBuffer buffer) {
  return &hierachy_circle_model_.footprint_model[buffer];
}

void NodeCollisionDetect::UpdateFootPrintBySafeBuffer(
    const float lat_buffer_outside, const float lat_buffer_inside,
    const float lon_buffer, const VehicleParam& vehicle_param,
    const PlannerOpenSpaceConfig& config) {
  if (request_->direction_request == ParkingVehDirection::HEAD_IN) {
    slot_box_ = cdl::AABB2f(
        Eigen::Vector2f(0.0f, -request_->slot_width / 2),
        Eigen::Vector2f(request_->slot_length + (float)vehicle_param.length,
                        request_->slot_width / 2));
  } else {
    slot_box_ = cdl::AABB2f(Eigen::Vector2f(0.0f, -request_->slot_width / 2),
                            Eigen::Vector2f(request_->slot_length + 2.5f,
                                            request_->slot_width / 2));
  }

  is_park_in_ = false;
  if (request_->direction_request == ParkingVehDirection::HEAD_IN ||
      request_->direction_request == ParkingVehDirection::TAIL_IN) {
    is_park_in_ = true;
  }

  // gear d
  float safe_half_width =
      (vehicle_param.max_width + lat_buffer_outside * 2) * 0.5f;

  GenerateVehCompactPolygon(lat_buffer_inside,
                            config.safe_buffer.lon_min_safe_buffer,
                            lat_buffer_inside, &cvx_hull_foot_print_);

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

  GetVehPolygonBy4Edge(veh_box_, vehicle_param.rear_edge_to_rear_axle,
                       vehicle_param.front_edge_to_rear_axle, safe_half_width);
  recommend_route_box_ = TransformMapBound(request_->recommend_route_bound);
  compact_route_box_ = recommend_route_box_;
  compact_route_box_.ShrinkX(vehicle_param.front_edge_to_rear_axle);
  compact_route_box_.ShrinkY(vehicle_param.front_edge_to_rear_axle);

  return;
}

const bool NodeCollisionDetect::IsContainByRecommendBox(
    const Pose2f& global_pose) {
  if (compact_route_box_.contain(global_pose)) {
    return true;
  }

  if (!recommend_route_box_.contain(global_pose)) {
    return false;
  }

  // check it again
  Transform2f tf;
  tf.SetBasePose(global_pose);
  std::array<Position2f, 4> global;
  for (int i = 0; i < 4; i++) {
    tf.ULFLocalPointToGlobal(&global[i], veh_box_[i]);
  }

  cdl::AABB2f aabb;
  GetBoundingBoxByPolygon(&aabb, global);
  if (recommend_route_box_.contain(aabb)) {
    return true;
  }

  return false;
}

}  // namespace planning