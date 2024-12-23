#pragma once

#include <cstddef>
#include <cstdint>
#include <list>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "ad_common/math/box2d.h"
#include "config/message_type.h"
#include "geometry_math.h"
#include "pose2d.h"
#include "src/library/convex_collision_detection/aabb2d.h"
#include "src/library/convex_collision_detection/polygon_base.h"
#include "utils/index_list.h"

namespace planning {
namespace apa_planner {

// 障碍物高度类型：高于后视镜 低于后视镜 低于底盘, 默认高于后视镜
enum class ApaObsHeightType : uint8_t {
  // unknown obstacles are treated as HIGH obstacle
  UNKNOWN = 0,
  MAY_RUN_OVER = 1,
  LOW = 2,
  MID = 3,
  HIGH = 4,
  COUNT,
};

// 障碍物属性类型：
enum class ApaObsAttributeType : uint8_t {
  UNKNOWN,
  FUSION_POINT_CLOUD,
  FUSION_POLYGON,
  GROUND_LINE_POINT_CLOUD,
  USS_POINT_CLOUD,
  VIRTUAL_POINT_CLOUD,
  MAP_BOUND,
  SLOT_LINE,
  SLOT_LIMITER,
  HOLE,
  COUNT,
};

// 障碍物运动类型：
enum class ApaObsMovementType : uint8_t {
  STATIC,
  MOTION,
  COUNT,
};

class ApaObstacle final {
 public:
  ApaObstacle() {}
  ~ApaObstacle() {}

  void SetId(const size_t id) { obs_id_ = id; }

  void SetPtClout2dGlobal(const std::vector<Eigen::Vector2d>& pt_clout_2d) {
    pt_clout_2d_global_ = pt_clout_2d;
  }

  void SetPtClout3dGlobal(const std::vector<Eigen::Vector3d>& pt_clout_3d) {
    pt_clout_3d_global_ = pt_clout_3d;
  }

  void SetPtClout2dLocal(const std::vector<Eigen::Vector2d>& pt_clout_2d) {
    pt_clout_2d_local_ = pt_clout_2d;
  }

  void SetPtClout3dLocal(const std::vector<Eigen::Vector3d>& pt_clout_3d) {
    pt_clout_3d_local_ = pt_clout_3d;
  }

  void SetObsAttributeType(const ApaObsAttributeType type) {
    obs_attribute_type_ = type;
  }

  void SetBoxGlobal(const cdl::AABB& box) { box_global_ = box; }
  void SetBoxLocal(const cdl::AABB& box) { box_local_ = box; }

  void SetPolygonGlobal(const Polygon2D& polygon) { polygon_global_ = polygon; }
  void SetPolygonLocal(const Polygon2D& polygon) { polygon_local_ = polygon; }

  void TransformCoordFromGlobalToLocal(
      const pnc::geometry_lib::GlobalToLocalTf& g2l_tf);

  void TransformPtClout2dFromGlobalToLocal(
      const pnc::geometry_lib::GlobalToLocalTf& g2l_tf);

  void TransformPtClout3dFromGlobalToLocal(
      const pnc::geometry_lib::GlobalToLocalTf& g2l_tf);

  void TransformBoxFromGlobalToLocal(
      const pnc::geometry_lib::GlobalToLocalTf& g2l_tf);

  void TransformPolygonFromGlobalToLocal(
      const pnc::geometry_lib::GlobalToLocalTf& g2l_tf);

  const std::vector<Eigen::Vector2d>& GetPtClout2dGlobal() const {
    return pt_clout_2d_global_;
  }

  const std::vector<Eigen::Vector2d>& GetPtClout2dLocal() const {
    return pt_clout_2d_local_;
  }

  const std::vector<Eigen::Vector3d>& GetPtClout3dGlobal() const {
    return pt_clout_3d_global_;
  }

  const std::vector<Eigen::Vector3d>& GetPtClout3dLocal() const {
    return pt_clout_3d_local_;
  }

  const ApaObsHeightType GetObsHeightType() const { return obs_height_type_; }

  const ApaObsAttributeType GetObsAttributeType() const {
    return obs_attribute_type_;
  }

  const ApaObsMovementType GetObsMovementType() const {
    return obs_movement_type_;
  }

  const cdl::AABB& GetBoxGlobal() const { return box_global_; }

  const cdl::AABB& GetBoxLocal() const { return box_local_; }

  const Polygon2D& GetPolygon2DGlobal() const { return polygon_global_; }

  const Polygon2D& GetPolygon2DLocal() const { return polygon_local_; }

  void Reset();

  void GenerateLocalBoundingbox(cdl::AABB *box) const;

 private:
  ApaObsHeightType obs_height_type_{ApaObsHeightType::UNKNOWN};
  ApaObsAttributeType obs_attribute_type_{ApaObsAttributeType::UNKNOWN};
  ApaObsMovementType obs_movement_type_{ApaObsMovementType::STATIC};

  double obs_vel_{0.};
  double obs_acc_{0.};
  pnc::geometry_lib::PathPoint obs_pose_global_;
  pnc::geometry_lib::PathPoint obs_pose_local_;

  double height_{0.};

  std::vector<pnc::geometry_lib::PathPoint> obs_predict_traj_global_;
  std::vector<pnc::geometry_lib::PathPoint> obs_predict_traj_local_;

  cdl::AABB box_local_;
  cdl::AABB box_global_;
  Polygon2D polygon_local_;
  Polygon2D polygon_global_;

  std::vector<Eigen::Vector2d> pt_clout_2d_global_;
  std::vector<Eigen::Vector2d> pt_clout_2d_local_;
  size_t pt_clout_2d_size_{0};

  std::vector<Eigen::Vector3d> pt_clout_3d_global_;
  std::vector<Eigen::Vector3d> pt_clout_3d_local_;
  size_t pt_clout_3d_size_{0};

  size_t obs_id_{0};
};
}  // namespace apa_planner
}  // namespace planning
