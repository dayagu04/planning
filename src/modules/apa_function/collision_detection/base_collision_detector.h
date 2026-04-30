#pragma once
#include <cstdint>
#include <memory>
#include <utility>
#include <vector>

#include "aabb2d.h"
#include "apa_obstacle.h"
#include "apa_obstacle_manager.h"
#include "common_math.h"
#include "math_lib.h"
#include "obstacle_clear_zone_decider/obstacle_clear_zone_decider.h"

namespace planning {
namespace apa_planner {
using namespace pnc;

#define MAX_CAR_FOOTPRINT_CIRCLE_NUM (12)

struct CarShapeVertex {
  std::vector<Eigen::Vector2d> polygon_with_mirror;
  std::vector<Eigen::Vector2d> polygon_without_mirror;
  std::vector<Eigen::Vector2d> left_mirror_rectangle;
  std::vector<Eigen::Vector2d> right_mirror_rectangle;
  std::vector<Eigen::Vector2d> left_tyre_rectangle;
  std::vector<Eigen::Vector2d> right_tyre_rectangle;
  std::vector<Eigen::Vector2d> chassis_polygon;
  std::vector<Eigen::Vector2d> rectangle_with_mirror;
  std::vector<Eigen::Vector2f> rectangle_with_mirror_f;
  std::vector<Eigen::Vector2d> rectangle_without_mirror;
  std::vector<Eigen::Vector2d>
      mirror_to_front_overhanging_rectangle_expand_front;
  std::vector<Eigen::Vector2d> mirror_to_rear_overhanging_polygon;
  std::vector<Eigen::Vector2d> mirror_to_rear_overhanging_rectangle_expand_rear;
  std::vector<Eigen::Vector2d> mirror_to_front_overhanging_polygon;
};

struct ColResultF {
  bool col_flag = false;
  float remain_dist = 26.8f;

  float min_obs_dist = 26.8f;

  Eigen::Vector2f dangerous_path_pt;

  void Reset() {
    col_flag = false;
    remain_dist = 26.8f;
    min_obs_dist = 26.8f;

    dangerous_path_pt.setZero();
  }
};

struct ColResult {
  bool col_flag = false;
  double remain_dist = 26.8;
  double remain_car_dist = 26.8;
  double remain_obs_dist = 36.8;
  double remain_dist_static = 16.8;
  double remain_dist_dynamic = 26.8;
  std::pair<double, geometry_lib::PathPoint> pt_closest2obs{
      26.8, geometry_lib::PathPoint()};
  geometry_lib::RectangleBound path_rectangle_bound;

  std::vector<geometry_lib::Pt2ObsDistInfo> pt_obs_dist_info_vec;

  Eigen::Vector2d dangerous_obs_pt;
  Eigen::Vector2d dangerous_path_pt;

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
    dangerous_obs_pt.setZero();
    dangerous_path_pt.setZero();
  }
};

struct CarFootPrintCircle {
  common_math::Pos<float> center_local{0.0f, 0.0f};
  common_math::Pos<float> center_global{0.0f, 0.0f};
  float radius{0.0f};

  void Reset() {
    center_local.setZero();
    center_global.setZero();
    radius = 0.0f;
  }

  void LocalToGlobal(const geometry_lib::PathPoint &pt) {
    geometry_lib::LocalToGlobalTf l2g_tf(pt.pos, pt.heading);
    center_global = l2g_tf.GetPos(center_local);
  }

  void LocalToGlobal(const geometry_lib::LocalToGlobalTf &l2g_tf) {
    center_global = l2g_tf.GetPos(center_local);
  }

  void LocalToGlobal(const common_math::Local2GlobalTrans<float> &l2g_tf) {
    center_global = l2g_tf.GetPos(center_local);
  }

  void PrintInfo(const bool enable_log = true) const {
    ILOG_INFO_IF(enable_log)
        << "center_local: " << center_local.x() << " " << center_local.y()
        << "   center_global: " << center_global.x() << " " << center_global.y()
        << "  radius = " << radius;
  };
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
    geometry_lib::LocalToGlobalTf l2g_tf(Eigen::Vector2f(pt.GetX(), pt.GetY()),
                                         float(pt.GetTheta()));
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

  void LocalToGlobal(const common_math::PathPt<float> &pt) {
    common_math::Local2GlobalTrans<float> l2g_tf(pt.pos, pt.theta);
    max_circle.LocalToGlobal(l2g_tf);
    for (uint8_t i = 0; i < count; ++i) {
      circles[i].LocalToGlobal(l2g_tf);
    }
  }

  void PrintInfo(const bool enable_log = true) const {
    ILOG_INFO_IF(enable_log) << "max_circle: ";
    max_circle.PrintInfo(enable_log);
    for (uint8_t i = 0; i < count; ++i) {
      ILOG_INFO_IF(enable_log) << "circle[" << static_cast<int>(i) << "]: ";
      circles[i].PrintInfo(enable_log);
    }
    ILOG_INFO_IF(enable_log) << "height_type: " << int(height_type);
  }
};

struct MultiCarFootPrintCircleList {
  CarFootPrintCircleList circles_with_mirror;
  CarFootPrintCircleList circles_without_mirror;
  CarFootPrintCircleList chassis_circles;
};

class BaseCollisionDetector {
 public:
  BaseCollisionDetector() {}
  virtual ~BaseCollisionDetector() = default;
  void Init(const bool fold_mirror_flag);
  void SetObsManagerPtr(
      const std::shared_ptr<ApaObstacleManager> &obs_manager_ptr) {
    obs_manager_ptr_ = obs_manager_ptr;
  };
  void SetSampleDs(const float sample_ds) { sample_ds_ = sample_ds; }
  void UpdateSafeBuffer(const ColDetBuffer &col_det_buffer);

  void UpdateObsClearZone(const std::vector<Eigen::Vector2d> &pt_vec);
  const bool IsPoseInClearZone(const geometry_lib::PathPoint &pose);

  const geometry_lib::RectangleBound CalCarRectangleBound(
      const geometry_lib::PathPoint &current_pose);

  const bool IsPoseInClearZone(const common_math::PathPt<float> &pose);

  const cdl::AABB2f CalCarAABBBoxF(const common_math::PathPt<float> &pose);

  static const bool CheckObsMovementTypeFeasible(
      const ApaObsMovementType obs_type,
      const ApaObsMovementType obs_type_request);

  const std::vector<cdl::AABB> &GetBoxVec() const {
    return obs_clear_zone_decider_.GetBoxVec();
  }

  void GenRealTimeTyrePolygonAccordingToFrontWheelAngle(
      const double front_wheel_angle);

  static const std::vector<Eigen::Vector2d> GetCarBigBoxWithBuffer(
      const double lat_buf, const double lon_buf,
      const geometry_lib::PathPoint &pose);

 protected:
  CarShapeVertex car_shape_vertex_;
  CarShapeVertex car_shape_vertex_with_buffer_;

  MultiCarFootPrintCircleList multi_car_shape_circle_;
  MultiCarFootPrintCircleList multi_car_shape_circle_with_buffer_;

  ColDetBuffer col_det_buffer_;

  std::vector<geometry_lib::PathPoint> path_pt_vec_;
  std::vector<common_math::PathPt<float>> pts_;

  std::shared_ptr<ApaObstacleManager> obs_manager_ptr_;

  ObstacleClearZoneDecider obs_clear_zone_decider_;

  ColResult col_res_;

  ColResultF col_res_f_;

  float sample_ds_ = 0.1f;

  bool need_update_buffer_ = false;

  UseObsHeightMethod use_obs_height_method_ = UseObsHeightMethod::HIGH;
};

}  // namespace apa_planner
}  // namespace planning