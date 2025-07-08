#pragma once
#include <cstdint>
#include <memory>
#include <utility>
#include <vector>

#include "apa_obstacle.h"
#include "apa_obstacle_manager.h"
#include "math_lib.h"
#include "obstacle_clear_zone_decider/obstacle_clear_zone_decider.h"

namespace planning {
namespace apa_planner {
using namespace pnc;

#define MAX_CAR_FOOTPRINT_CIRCLE_NUM (12)

struct ColResult {
  bool col_flag = false;
  // 纵向
  double remain_dist = 26.8;
  double remain_car_dist = 26.8;
  double remain_obs_dist = 36.8;
  // real-time brake
  double remain_dist_static = 16.8;
  double remain_dist_dynamic = 26.8;
  // 横向
  std::pair<double, geometry_lib::PathPoint> pt_closest2obs{
      26.8, geometry_lib::PathPoint()};
  geometry_lib::RectangleBound path_rectangle_bound;

  std::vector<geometry_lib::Pt2ObsDistInfo> pt_obs_dist_info_vec;

  void Reset() {
    col_flag = false;
    remain_dist = 26.8;
    remain_car_dist = 26.8;
    remain_obs_dist = 36.8;
    remain_dist_static = 16.8;
    remain_dist_dynamic = 26.8;
    pt_closest2obs = std::make_pair(26.8, geometry_lib::PathPoint());
    path_rectangle_bound.Reset();
    pt_obs_dist_info_vec.clear();
  }
};

struct CarFootPrintCircle {
  Eigen::Vector2d center_local = Eigen::Vector2d(0.0, 0.0);
  Eigen::Vector2d center_global = Eigen::Vector2d(0.0, 0.0);
  double radius{0.0};

  void Reset() {
    center_local.setZero();
    center_global.setZero();
    radius = 0.0;
  }

  void LocalToGlobal(const geometry_lib::PathPoint &pt) {
    geometry_lib::LocalToGlobalTf l2g_tf(pt.pos, pt.heading);
    center_global = l2g_tf.GetPos(center_local);
  }

  void LocalToGlobal(const geometry_lib::LocalToGlobalTf &l2g_tf) {
    center_global = l2g_tf.GetPos(center_local);
  }
};

struct CarFootPrintCircleList {
  uint8_t count = 0;
  CarFootPrintCircle circles[MAX_CAR_FOOTPRINT_CIRCLE_NUM];

  // if max circle no collision, then no need to check other circle
  CarFootPrintCircle max_circle;

  // applicable obstacle height type
  ApaObsHeightType height_type;

  void Reset() {
    count = 0;
    max_circle.Reset();
    height_type = ApaObsHeightType::UNKNOWN;
    memset(circles, 0,
           MAX_CAR_FOOTPRINT_CIRCLE_NUM * sizeof(CarFootPrintCircle));
  }

  void LocalToGlobal(const geometry_lib::PathPoint &pt) {
    geometry_lib::LocalToGlobalTf l2g_tf(pt.pos, pt.heading);
    max_circle.LocalToGlobal(l2g_tf);
    for (uint8_t i = 0; i < count; ++i) {
      circles[i].LocalToGlobal(l2g_tf);
    }
  }

  void LocalToGlobal(const geometry_lib::LocalToGlobalTf &l2g_tf) {
    max_circle.LocalToGlobal(l2g_tf);
    for (uint8_t i = 0; i < count; ++i) {
      circles[i].LocalToGlobal(l2g_tf);
    }
  }
};

class BaseCollisionDetector {
 public:
  BaseCollisionDetector() {}
  virtual ~BaseCollisionDetector() = default;
  void Init(const bool fold_mirror_flag);
  void SetObsManager(const std::shared_ptr<ApaObstacleManager> &obs_manager_ptr) {
    obs_manager_ptr_ = obs_manager_ptr;
  };
  void SetSampleDs(const double sample_ds) { sample_ds_ = sample_ds; }
  void UpdateSafeBuffer(const double lat_buffer, const double lon_buffer);

  void UpdateObsClearZone(const std::vector<Eigen::Vector2d> &pt_vec);
  const bool IsPoseInClearZone(const geometry_lib::PathPoint &pose);

  const geometry_lib::RectangleBound CalCarRectangleBound(
      const geometry_lib::PathPoint &current_pose);

  static const bool CheckObsMovementTypeFeasible(
      const ApaObsMovementType obs_type,
      const ApaObsMovementType obs_type_request);

  const std::vector<cdl::AABB> &GetBoxVec() const {
    return obs_clear_zone_decider_.GetBoxVec();
  }

 protected:
  // 需要的原始自车参数顶点坐标 基于自车坐标系 逆时针旋转
  // 包含左右后视镜的多边形
  std::vector<Eigen::Vector2d> car_with_mirror_polygon_vertex_;
  std::vector<Eigen::Vector2d> car_with_mirror_polygon_vertex_with_buffer_;
  // 不包含后视镜的多边形
  std::vector<Eigen::Vector2d> car_without_mirror_polygon_vertex_;
  std::vector<Eigen::Vector2d> car_without_mirror_polygon_vertex_with_buffer_;
  // 左后视镜
  std::vector<Eigen::Vector2d> left_mirror_rectangle_vertex_;
  std::vector<Eigen::Vector2d> left_mirror_rectangle_vertex_with_buffer_;
  // 右后视镜
  std::vector<Eigen::Vector2d> right_mirror_rectangle_vertex_;
  std::vector<Eigen::Vector2d> right_mirror_rectangle_vertex_with_buffer_;
  // 底盘矩形
  std::vector<Eigen::Vector2d> chassis_vertex_;
  std::vector<Eigen::Vector2d> chassis_vertex_with_buffer_;
  // 包含左右后视镜的矩形
  std::vector<Eigen::Vector2d> car_with_mirror_rectangle_vertex_;
  std::vector<Eigen::Vector2d> car_with_mirror_rectangle_vertex_with_buffer_;
  // 后视镜到前悬矩形
  std::vector<Eigen::Vector2d>
      mirror_to_front_overhanging_rectangle_vertex_expand_front_;
  std::vector<Eigen::Vector2d>
      mirror_to_front_overhanging_rectangle_vertex_expand_front_with_buffer_;
  // 后视镜到后悬矩形
  std::vector<Eigen::Vector2d> mirror_to_rear_overhanging_polygon_vertex_;
  std::vector<Eigen::Vector2d>
      mirror_to_rear_overhanging_polygon_vertex_with_buffer_;

  CarFootPrintCircleList car_with_mirror_circles_list_;
  CarFootPrintCircleList car_without_mirror_circles_list_;
  CarFootPrintCircleList car_chassis_circles_list_;

  double lat_buffer_{0.};
  double lon_buffer_{0.};
  std::vector<geometry_lib::PathPoint> path_pt_vec_;

  std::shared_ptr<ApaObstacleManager> obs_manager_ptr_;

  ObstacleClearZoneDecider obs_clear_zone_decider_;

  ColResult col_res_;

  double sample_ds_ = 0.1;

  bool need_update_buffer_ = false;
};

}  // namespace apa_planner
}  // namespace planning