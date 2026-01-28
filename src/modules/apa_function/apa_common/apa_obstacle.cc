#include "apa_obstacle.h"

#include "speed/st_boundary.h"

namespace planning {
namespace apa_planner {

void ApaObstacle::Reset() {
  obs_height_type_ = ApaObsHeightType::UNKNOWN;
  obs_attribute_type_ = ApaObsAttributeType::UNKNOWN;
  obs_movement_type_ = ApaObsMovementType::STATIC;

  vel_ = 0.;
  acc_ = 0.;
  pose_global_.Reset();
  pose_local_.Reset();

  height_ = 0.;

  predict_traj_.clear();
  obstacle_history_positions_.clear();
  max_history_point_size_ = 10;
  lost_frame_count_ = 0;
  last_omega_mag_ = 0.0;
  last_omega_signed_ = 0.0;
  turn_dir_ = DynamicObsTurnDirection::STRAIGHT;

  pt_clout_2d_global_.clear();
  pt_clout_2d_local_.clear();

  pt_clout_3d_global_.clear();
  pt_clout_3d_local_.clear();

  obs_id_ = 0;
  st_boundary_ = STBoundary();
  lon_decision_.Clear();
  polygon_global_.Clear();
  polygon_local_.Clear();
  vel_heading_.setZero();
}

void ApaObstacle::TransformCoordFromGlobalToLocal(
    const pnc::geometry_lib::GlobalToLocalTf& g2l_tf) {
  TransformBoxFromGlobalToLocal(g2l_tf);
  TransformPtClout2dFromGlobalToLocal(g2l_tf);
  TransformPtClout3dFromGlobalToLocal(g2l_tf);
  TransformPolygonFromGlobalToLocal(g2l_tf);
}

void ApaObstacle::TransformBoxFromGlobalToLocal(
    const pnc::geometry_lib::GlobalToLocalTf& g2l_tf) {
  box_local_.min_ = g2l_tf.GetPos(box_global_.min_);
  box_local_.max_ = g2l_tf.GetPos(box_global_.max_);
}

void ApaObstacle::TransformPtClout2dFromGlobalToLocal(
    const pnc::geometry_lib::GlobalToLocalTf& g2l_tf) {
  pt_clout_2d_local_.clear();
  if (pt_clout_2d_global_.empty()) {
    return;
  }
  pt_clout_2d_local_.reserve(pt_clout_2d_global_.size());
  for (const Eigen::Vector2d& pt : pt_clout_2d_global_) {
    pt_clout_2d_local_.emplace_back(g2l_tf.GetPos(pt));
  }
}

void ApaObstacle::TransformPtClout3dFromGlobalToLocal(
    const pnc::geometry_lib::GlobalToLocalTf& g2l_tf) {
  if (pt_clout_3d_global_.empty()) {
    return;
  }
}

void ApaObstacle::TransformPolygonFromGlobalToLocal(
    const pnc::geometry_lib::GlobalToLocalTf& g2l_tf) {
  polygon_local_.vertex_num = polygon_global_.vertex_num;
  polygon_local_.radius = polygon_global_.radius;
  polygon_local_.min_tangent_radius = polygon_global_.min_tangent_radius;
  polygon_local_.shape = polygon_global_.shape;

  Eigen::Vector2d local;
  for (int i = 0; i < polygon_local_.vertex_num; i++) {
    local = g2l_tf.GetPos(Eigen::Vector2d(polygon_global_.vertexes[i].x,
                                          polygon_global_.vertexes[i].y));

    polygon_local_.vertexes[i].x = local[0];
    polygon_local_.vertexes[i].y = local[1];
  }

  local = g2l_tf.GetPos(Eigen::Vector2d(polygon_global_.center_pt.x,
                                        polygon_global_.center_pt.y));
  polygon_local_.center_pt.x = local[0];
  polygon_local_.center_pt.y = local[1];

  return;
}

void ApaObstacle::GenerateLocalBoundingbox(cdl::AABB* box) const {
  *box = cdl::AABB();

  for (const auto& pt : pt_clout_2d_local_) {
    box->MergePointfloat64(pt);
  }

  return;
}

const STBoundary& ApaObstacle::PathSTBoundary() const { return st_boundary_; }

void ApaObstacle::SetPathSTBoundary(const STBoundary& boundary) {
  st_boundary_ = boundary;
  return;
}

void ApaObstacle::EraseStBoundary() { st_boundary_ = STBoundary(); }

const ParkLonDecision& ApaObstacle::LongitudinalDecision() const {
  return lon_decision_;
}

void ApaObstacle::SetLonDecision(const ParkLonDecision& decision) {
  lon_decision_ = decision;
  return;
}

void ApaObstacle::ClearDecision() {
  EraseStBoundary();
  lon_decision_.Clear();
  return;
}

void ApaObstacle::SetObsScemanticType(const iflyauto::ObjectType obs_type) {
  obs_scemantic_type_ = ApaObsScemanticType::UNKNOWN;
  switch (obs_type) {
    case iflyauto::OBJECT_TYPE_OCC_COLUMN:
      obs_scemantic_type_ = ApaObsScemanticType::COLUMN;
      break;
    case iflyauto::OBJECT_TYPE_OCC_WALL:
      obs_scemantic_type_ = ApaObsScemanticType::WALL;
      break;
    case iflyauto::OBJECT_TYPE_OCC_CAR:
    case iflyauto::OBJECT_TYPE_PICKUP:
    case iflyauto::OBJECT_TYPE_SUV:
    case iflyauto::OBJECT_TYPE_MPV:
    case iflyauto::OBJECT_TYPE_ENGINEERING_VEHICLE:
    case iflyauto::OBJECT_TYPE_COUPE:
    case iflyauto::OBJECT_TYPE_MINIBUS:
    case iflyauto::OBJECT_TYPE_VAN:
    case iflyauto::OBJECT_TYPE_BUS:
    case iflyauto::OBJECT_TYPE_TRUCK:
    case iflyauto::OBJECT_TYPE_TRAILER:
      obs_scemantic_type_ = ApaObsScemanticType::CAR;
      break;
    case iflyauto::OBJECT_TYPE_OCC_GROUDING_WIRE:
      obs_scemantic_type_ = ApaObsScemanticType::CURB;
      break;
    case iflyauto::OBJECT_TYPE_OCC_CYCLIST:
    case iflyauto::OBJECT_TYPE_TRICYCLE_RIDING:
    case iflyauto::OBJECT_TYPE_MOTORCYCLE_RIDING:
    case iflyauto::OBJECT_TYPE_CYCLE_RIDING:
    case iflyauto::OBJECT_TYPE_TRICYCLE:
    case iflyauto::OBJECT_TYPE_MOTORCYCLE:
    case iflyauto::OBJECT_TYPE_BICYCLE:
      obs_scemantic_type_ = ApaObsScemanticType::CYCLIST;
      break;
    case iflyauto::OBJECT_TYPE_ADULT:
    case iflyauto::OBJECT_TYPE_TRAFFIC_POLICE:
    case iflyauto::OBJECT_TYPE_CHILD:
    case iflyauto::OBJECT_TYPE_ANIMAL:
    case iflyauto::OBJECT_TYPE_PEDESTRIAN:
      obs_scemantic_type_ = ApaObsScemanticType::PEOPLE;
      break;
    case iflyauto::OBJECT_TYPE_CTASH_BARREL:
    case iflyauto::OBJECT_TYPE_TRAFFIC_BARREL:
      obs_scemantic_type_ = ApaObsScemanticType::BARREL;
      break;
    case iflyauto::OBJECT_TYPE_WATER_SAFETY_BARRIER:
    case iflyauto::OBJECT_TYPE_TRAFFIC_CONE:
      obs_scemantic_type_ = ApaObsScemanticType::BARRIER;
      break;
    case iflyauto::OBJECT_TYPE_FENCE:
      obs_scemantic_type_ = ApaObsScemanticType::FENCE;
      break;
    default:
      obs_scemantic_type_ = ApaObsScemanticType::UNKNOWN;
      break;
  }

  return;
}

void ApaObstacle::SetObsScemanticType(const iflyauto::GroundLineType obs_type) {
  obs_scemantic_type_ = ApaObsScemanticType::UNKNOWN;
  switch (obs_type) {
    case iflyauto::GROUND_LINE_TYPE_COLUMN:
      obs_scemantic_type_ = ApaObsScemanticType::COLUMN;
      break;
    case iflyauto::GROUND_LINE_TYPE_WALL:
      obs_scemantic_type_ = ApaObsScemanticType::WALL;
      break;
    case iflyauto::GROUND_LINE_TYPE_FENCE:
      obs_scemantic_type_ = ApaObsScemanticType::FENCE;
      break;
    case iflyauto::GROUND_LINE_TYPE_STEP:
      obs_scemantic_type_ = ApaObsScemanticType::STEP;
      break;
    case iflyauto::GROUND_LINE_TYPE_CURB:
      obs_scemantic_type_ = ApaObsScemanticType::CURB;
      break;
    case iflyauto::GROUND_LINE_TYPE_SPECIAL:
      obs_scemantic_type_ = ApaObsScemanticType::SPECIAL;
      break;
    default:
      obs_scemantic_type_ = ApaObsScemanticType::UNKNOWN;
      break;
  }
  return;
}

const bool ApaObstacle::IsMovableStaticObs() const {
  if (obs_movement_type_ != ApaObsMovementType::STATIC) {
    return false;
  }

  if (obs_scemantic_type_ == ApaObsScemanticType::BARRIER ||
      obs_scemantic_type_ == ApaObsScemanticType::CYCLIST ||
      obs_scemantic_type_ == ApaObsScemanticType::BARREL ||
      obs_scemantic_type_ == ApaObsScemanticType::CAR) {
    return true;
  }

  return false;
}

void ApaObstacle::UpdateObstacleHistoryPositions(
    const Eigen::Vector2d& new_position) {
  // if (obstacle_history_positions_.empty()) {
  //   obstacle_history_positions_.push_back(new_position);
  //   return;
  // }

  // const Eigen::Vector2d& last = obstacle_history_positions_.back();

  // constexpr double kMinMoveDist = 0.05;  // 5cm
  // if ((new_position - last).norm() < kMinMoveDist) {
  //   return;
  // }

  // double max_step = std::max(0.05, vel_) * 0.2;  // v * dt
  // if ((new_position - last).norm() > max_step) {
  //   return;
  // }

  obstacle_history_positions_.push_back(new_position);

  while (obstacle_history_positions_.size() > max_history_point_size_) {
    obstacle_history_positions_.pop_front();
  }
}

}  // namespace apa_planner
}  // namespace planning