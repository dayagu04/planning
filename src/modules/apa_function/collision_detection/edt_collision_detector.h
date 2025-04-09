#pragma once
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <opencv2/core/mat.hpp>

#include "apa_obstacle.h"
#include "base_collision_detector.h"
#include "geometry_math.h"

namespace planning {
namespace apa_planner {

#define edt_ogm_resolution (0.05)
#define edt_ogm_grid_x_max (512)
#define edt_ogm_grid_y_max (800)
#define MAX_CAR_FOOTPRINT_CIRCLE_NUM (12)

// x -> row   y -> column
// slot coordinate system
//               ^  x
//               |
//               |
//               |
//               |
//               |
//  y <----------|

struct OGMIndex {
  size_t x{0};
  size_t y{0};

  OGMIndex() {}
  OGMIndex(const size_t _x, const size_t _y) : x(_x), y(_y) {}
  void Set(const size_t _x, const size_t _y) {
    x = _x;
    y = _y;
  }
  ~OGMIndex() {}
};

struct OGMObsData {
  // store the distance from each grid to the nearest obstacle
  float dist[edt_ogm_grid_x_max][edt_ogm_grid_y_max];
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
  uint8_t count{0};
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

class EDTCollisionDetector final : public BaseCollisionDetector {
 public:
  EDTCollisionDetector() {}
  EDTCollisionDetector(const std::shared_ptr<ApaObstacleManager> &obs_manager) {
    SetObsManager(obs_manager);
  }
  ~EDTCollisionDetector(){};

  void PreProcess();

  void PreProcess(const geometry_lib::RectangleBound &ogm_bound);

  void GenOccupancyGridMap(const geometry_lib::RectangleBound &ogm_bound);

  void GenOccupancyGridMap(const geometry_lib::RectangleBound &ogm_bound,
                           const double _ogm_resolution);

  void GenOccupancyGridMap(const Eigen::Vector2d &ogm_origin);

  void GenOccupancyGridMap(const Eigen::Vector2d &ogm_origin,
                           const double _ogm_resolution);

  const OGMIndex GetIndexFromOGMPose(const Eigen::Vector2d &pt);

  const OGMIndex GetIndexFromSlotPose(const Eigen::Vector2d &pt);

  const bool IsIndexValid(const OGMIndex &id) const;

  void TransformObsOGMToMatrix(cv::Mat *mat,
                               const bool (*obs_ogm)[edt_ogm_grid_y_max]) const;

  void AddObsToOGM();

  void Reset();

  void CalcObsDistArray();

  const double GetObsDistByIndex(const OGMIndex &id,
                                 const ApaObsHeightType &height_type);

  void UpdateSafeBuffer(const double lat_buffer, const double lon_buffer,
                        const double max_circle_buffer = 0.5);

  void UpdateCarWithMirrorSafeBuffer();
  void UpdateCarWithOutMirrorSafeBuffer();
  void UpdateCarChassisSafeBuffer();

  const bool IsCollisionForPoint(const geometry_lib::PathPoint &pt,
                                 CarFootPrintCircleList *car_circle_list);

  // return min dist between obs and car circle
  const bool IsCollisionForPoint(const geometry_lib::PathPoint &pt,
                                 CarFootPrintCircleList *car_circle_list,
                                 double *min_dist, int *circle_id,
                                 const double safe_dist = 0.5);

  const ColResult Update(const geometry_lib::PathSegment &path_seg,
                         const double lat_buffer, const double lon_buffer,
                         const bool need_cal_obs_dist = false,
                         const double max_circle_buffer = 0.5);

  const ColResult Update(const std::vector<geometry_lib::PathPoint> &pt_vec,
                         const double lat_buffer, const double lon_buffer,
                         const bool need_cal_obs_dist = false,
                         const double max_circle_buffer = 0.5);

 private:
  // origin and boundary of grid coordinate system
  Eigen::Vector2d ogm_origin_ = Eigen::Vector2d(0.0, 0.0);
  geometry_lib::RectangleBound ogm_bound_;

  // todo: x and y can have different resolution?
  double resolution_ = edt_ogm_resolution;
  double resolution_inv_ = 1.0 / edt_ogm_resolution;

  // If obstacles occupy this grid, true
  bool car_with_mirror_obs_ogm_[edt_ogm_grid_x_max][edt_ogm_grid_y_max] = {};
  bool car_without_mirror_obs_ogm_[edt_ogm_grid_x_max][edt_ogm_grid_y_max] = {};
  bool car_chassis_obs_ogm_[edt_ogm_grid_x_max][edt_ogm_grid_y_max] = {};

  // 用数组存储障碍物信息
  OGMObsData car_with_mirror_ogm_obs_data_;
  OGMObsData car_without_mirror_ogm_obs_data_;
  OGMObsData car_chassis_ogm_obs_data_;

  // 存储车包络圆信息
  // local->veh coord sys
  // global->slot coord sys
  CarFootPrintCircleList car_with_mirror_circles_list_;
  CarFootPrintCircleList car_without_mirror_circles_list_;
  CarFootPrintCircleList car_chassis_circles_list_;

  double lon_buffer_ = 0.3;
  double lat_buffer_ = 0.1;
  double max_circle_buffer_ = 0.5;
};

}  // namespace apa_planner
}  // namespace planning