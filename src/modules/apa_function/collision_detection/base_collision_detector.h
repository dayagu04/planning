#pragma once
#include <cstdint>
#include <memory>
#include <utility>
#include <vector>

#include "apa_obstacle.h"
#include "apa_obstacle_manager.h"
#include "math_lib.h"

namespace planning {
namespace apa_planner {
using namespace pnc;

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

  void Reset() {
    col_flag = false;
    remain_dist = 26.8;
    remain_car_dist = 26.8;
    remain_obs_dist = 36.8;
    remain_dist_static = 16.8;
    remain_dist_dynamic = 26.8;
    pt_closest2obs = std::make_pair(26.8, geometry_lib::PathPoint());
    path_rectangle_bound.Reset();
  }
};

class BaseCollisionDetector {
 public:
  BaseCollisionDetector() {}
  virtual ~BaseCollisionDetector() = default;
  void Init();
  void SetObsManager(const std::shared_ptr<ApaObstacleManager>& obs_manager) {
    obs_manager_ = obs_manager;
  };
  void SetSampleDs(const double sample_ds) { sample_ds_ = sample_ds; }
  void UpdateSafeBuffer(const double lat_buffer, const double lon_buffer);

  const geometry_lib::RectangleBound CalCarRectangleBound(
      const geometry_lib::PathPoint& current_pose);

  static const bool CheckObsMovementTypeFeasible(
      const ApaObsMovementType obs_type,
      const ApaObsMovementType obs_type_request);

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

  double lat_buffer_{0.};
  double lon_buffer_{0.};
  std::vector<geometry_lib::PathPoint> path_pt_vec_;

  std::shared_ptr<ApaObstacleManager> obs_manager_;

  ColResult col_res_;

  double sample_ds_ = 0.1;
};

}  // namespace apa_planner
}  // namespace planning