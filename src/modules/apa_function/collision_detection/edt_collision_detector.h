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
#define edt_ogm_grid_x_max int(20 / edt_ogm_resolution)
#define edt_ogm_grid_y_max int(36 / edt_ogm_resolution)

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
  int x{0};
  int y{0};

  OGMIndex() {}
  OGMIndex(const int _x, const int _y) : x(_x), y(_y) {}
  void Set(const int _x, const int _y) {
    x = _x;
    y = _y;
  }
  ~OGMIndex() {}
};

struct OGMObsData {
  // store the distance from each grid to the nearest obstacle
  float dist[edt_ogm_grid_x_max][edt_ogm_grid_y_max];
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

  void UpdateSafeBuffer(const double body_lat_buffer, const double lon_buffer,
                        const double max_circle_buffer = 0.5,
                        const bool special_process_mirror = false,
                        const double mirror_lat_buffer = 0.08);

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
                         const double body_lat_buffer, const double lon_buffer,
                         const bool need_cal_obs_dist = false,
                         const double max_circle_buffer = 0.5,
                         const bool special_process_mirror = false,
                         const double mirror_lat_buffer = 0.08);

  const ColResult Update(const std::vector<geometry_lib::PathPoint> &pt_vec,
                         const double body_lat_buffer, const double lon_buffer,
                         const bool need_cal_obs_dist = false,
                         const double max_circle_buffer = 0.5,
                         const bool special_process_mirror = false,
                         const double mirror_lat_buffer = 0.08);

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
  CarFootPrintCircleList car_with_mirror_circles_list_buffer_;
  CarFootPrintCircleList car_without_mirror_circles_list_with_buffer_;
  CarFootPrintCircleList car_chassis_circles_list_with_buffer_;

  double max_circle_buffer_ = 0.5;
};

}  // namespace apa_planner
}  // namespace planning