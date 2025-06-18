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
#include "speed/apa_speed_decision.h"
#include "speed/st_boundary.h"
#include "src/library/convex_collision_detection/aabb2d.h"
#include "src/library/convex_collision_detection/polygon_base.h"
#include "utils/index_list.h"
#include "trajectory/trajectory.h"

namespace planning {
namespace apa_planner {

// 障碍物高度类型：轮胎可以越过, 低于底盘但不可越过, 高于底盘但低于后视镜,
// 高于后视镜, 默认为高于后视镜
enum class ApaObsHeightType : uint8_t {
  // unknown obstacles are treated as HIGH obstacle
  UNKNOWN = 0,
  RUN_OVER = 1,
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

enum class ApaObsScemanticType : uint8_t {
  UNKNOWN,
  WALL,
  COLUMN,
  FENCE,
  STEP,
  CURB,
  SPECIAL,
  CAR,
  CYCLIST,
  PEOPLE,
  BARRIER,
  BARREL,
  LIMITER,
  SPECIFICATIONER,
  COUNT,
};

// 障碍物运动类型：
enum class ApaObsMovementType : uint8_t {
  STATIC,
  MOTION,
  ALL,
  COUNT,
};

class ApaObstacle final {
 public:
  ApaObstacle() {}
  ~ApaObstacle() {}

  void SetId(const size_t id) { obs_id_ = id; }

  const size_t GetId() const { return obs_id_; }

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

  void SetObsMovementType(const ApaObsMovementType type) {
    obs_movement_type_ = type;
  }

  void SetObsAttributeType(const ApaObsAttributeType type) {
    obs_attribute_type_ = type;
  }

  void SetObsScemanticType(const ApaObsScemanticType type) {
    obs_scemantic_type_ = type;
  }

  void SetObsHeightType(const ApaObsHeightType type) {
    obs_height_type_ = type;
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

  std::vector<Eigen::Vector2d>& GetMutablePtClout2dLocal() {
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

  const ApaObsScemanticType GetObsScemanticType() const {
    return obs_scemantic_type_;
  }

  const ApaObsMovementType GetObsMovementType() const {
    return obs_movement_type_;
  }

  const cdl::AABB& GetBoxGlobal() const { return box_global_; }

  const cdl::AABB& GetBoxLocal() const { return box_local_; }

  const Polygon2D& GetPolygon2DGlobal() const { return polygon_global_; }

  const Polygon2D& GetPolygon2DLocal() const { return polygon_local_; }

  void Reset();

  void GenerateLocalBoundingbox(cdl::AABB* box) const;

  const STBoundary& PathSTBoundary() const;

  void SetPathSTBoundary(const STBoundary& boundary);

  void EraseStBoundary();

  /**
   * return the merged longitudinal decision
   * Longitudinal decision is one of {Stop, Follow, Overtake, Ignore}
   **/
  const ParkLonDecision& LongitudinalDecision() const;

  void SetLonDecision(const ParkLonDecision& decision);

  void ClearDecision();

  void SetSpeed(const double v) {
    vel_ = v;
    return;
  }

  void SetSpeedHeading(const Eigen::Vector2d& heading) {
    vel_heading_ = heading;
    return;
  }

  const Eigen::Vector2d& GetSpeedHeading() const { return vel_heading_; }

  const double Speed() const { return vel_; }

  void SetPose(const pnc::geometry_lib::PathPoint& pose) {
    pose_global_ = pose;
    return;
  }

  const pnc::geometry_lib::PathPoint& GetCenterPose() const {
    return pose_global_;
  }

  void SetPredictTraj(const trajectory::Trajectory& traj) {
    predict_traj_ = traj;
    return;
  }

  const trajectory::Trajectory& GetPredictTraj() const { return predict_traj_; }

 private:
  ApaObsHeightType obs_height_type_{ApaObsHeightType::UNKNOWN};
  ApaObsAttributeType obs_attribute_type_{ApaObsAttributeType::UNKNOWN};
  ApaObsScemanticType obs_scemantic_type_{ApaObsScemanticType::UNKNOWN};
  ApaObsMovementType obs_movement_type_{ApaObsMovementType::STATIC};

  double vel_{0.};
  Eigen::Vector2d vel_heading_;
  double acc_{0.};
  pnc::geometry_lib::PathPoint pose_global_;
  pnc::geometry_lib::PathPoint pose_local_;

  double height_{0.};

  trajectory::Trajectory predict_traj_;

  cdl::AABB box_local_;
  cdl::AABB box_global_;
  Polygon2D polygon_local_;
  Polygon2D polygon_global_;

  std::vector<Eigen::Vector2d> pt_clout_2d_global_;
  std::vector<Eigen::Vector2d> pt_clout_2d_local_;

  std::vector<Eigen::Vector3d> pt_clout_3d_global_;
  std::vector<Eigen::Vector3d> pt_clout_3d_local_;

  size_t obs_id_{0};

  STBoundary st_boundary_;

  ParkLonDecision lon_decision_;
};
}  // namespace apa_planner
}  // namespace planning
