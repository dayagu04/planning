#include "apa_obstacle.h"

namespace planning {
namespace apa_planner {

void ApaObstacle::Reset() {
  obs_height_type_ = ApaObsHeightType::UNKNOWN;
  obs_attribute_type_ = ApaObsAttributeType::UNKNOWN;
  obs_movement_type_ = ApaObsMovementType::STATIC;

  obs_vel_ = 0.;
  obs_acc_ = 0.;
  obs_pose_global_.Reset();
  obs_pose_local_.Reset();

  height_ = 0.;

  obs_predict_traj_global_.clear();
  obs_predict_traj_local_.clear();

  pt_clout_2d_global_.clear();
  pt_clout_2d_local_.clear();
  pt_clout_2d_size_ = 0;

  pt_clout_3d_global_.clear();
  pt_clout_3d_local_.clear();
  pt_clout_3d_size_ = 0;

  obs_id_ = 0;
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
  if (pt_clout_2d_global_.empty()) {
    return;
  }
  pt_clout_2d_local_.clear();
  Eigen::Vector2d pt_local;
  for (const auto& pt : pt_clout_2d_global_) {
    pt_local = g2l_tf.GetPos(pt);
    pt_clout_2d_local_.emplace_back(pt_local);
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

}  // namespace apa_planner
}  // namespace planning