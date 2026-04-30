#pragma once
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <opencv2/core/mat.hpp>

#include "apa_obstacle.h"
#include "base_collision_detector.h"
#include "geometry_math.h"
#include "hybrid_astar_common.h"

namespace planning {
namespace apa_planner {

#define edt_ogm_resolution (0.05f)
#define edt_ogm_grid_x_max int(20.0f / edt_ogm_resolution)
#define edt_ogm_grid_y_max int(36.0f / edt_ogm_resolution)

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
    SetObsManagerPtr(obs_manager);
  }
  ~EDTCollisionDetector(){};

  void PreProcess(const UseObsHeightMethod use_obs_height_method =
                      UseObsHeightMethod::HIGH);

  void PreProcess(const geometry_lib::RectangleBound &ogm_bound,
                  const UseObsHeightMethod use_obs_height_method =
                      UseObsHeightMethod::HIGH);

  void GenOccupancyGridMap(const geometry_lib::RectangleBound &ogm_bound);

  void GenOccupancyGridMap(const geometry_lib::RectangleBound &ogm_bound,
                           const double _ogm_resolution);

  void GenOccupancyGridMap(const Eigen::Vector2d &ogm_origin);

  void GenOccupancyGridMap(const Eigen::Vector2d &ogm_origin,
                           const double _ogm_resolution);

  const OGMIndex GetIndexFromOGMPose(const Eigen::Vector2f &pt);

  const OGMIndex GetIndexFromSlotPose(const Eigen::Vector2f &pt);

  const bool IsIndexValid(const OGMIndex &id) const;

  void TransformObsOGMToMatrix(cv::Mat *mat,
                               const bool (*obs_ogm)[edt_ogm_grid_y_max]) const;

  void AddObsToOGM();

  void Reset();

  void CalcObsDistArray();

  const double GetObsDistByIndex(const OGMIndex &id,
                                 const ApaObsHeightType &height_type);

  void UpdateSafeBuffer(const ColDetBuffer &col_det_buffer);

  void UpdateSingleCircleListSafeBuffer(const CarFootPrintCircleList &src,
                                        CarFootPrintCircleList &dst,
                                        ApaObsHeightType height_type,
                                        bool has_mirror);

  const bool IsCollisionForPoint(const geometry_lib::PathPoint &pt,
                                 CarFootPrintCircleList *car_circle_list);

  // return min dist between obs and car circle
  const bool IsCollisionForPoint(const geometry_lib::PathPoint &pt,
                                 CarFootPrintCircleList *car_circle_list,
                                 float *min_dist, int *circle_id,
                                 const float safe_dist = 0.5);

  const bool IsCollisionForPoint(const common_math::PathPt<float> &pt,
                                 CarFootPrintCircleList *car_circle_list);

  const bool IsCollisionForPoint(const common_math::PathPt<float> &pt,
                                 CarFootPrintCircleList *car_circle_list,
                                 float *min_dist, const float safe_dist = 0.5f);

  const ColResult Update(const geometry_lib::PathSegment &path_seg,
                         const ColDetBuffer &col_det_buffer,
                         const bool need_cal_obs_dist = false);

  const ColResult Update(const std::vector<geometry_lib::PathPoint> &pt_vec,
                         const ColDetBuffer &col_det_buffer,
                         const bool need_cal_obs_dist = false);

  const ColResultF Update(const std::vector<common_math::PathPt<float>> &pts,
                          const ColDetBuffer &col_det_buffer,
                          const bool need_cal_obs_dist = false);

  const geometry_lib::RectangleBound &GetOgmBound() const { return ogm_bound_; }

  const UseObsHeightMethod GetUseObsHeightMethod() const {
    return use_obs_height_method_;
  }

 private:
  // origin and boundary of grid coordinate system
  Eigen::Vector2f ogm_origin_ = Eigen::Vector2f(0.0f, 0.0f);
  geometry_lib::RectangleBound ogm_bound_;

  // todo: x and y can have different resolution?
  float resolution_ = edt_ogm_resolution;
  float resolution_inv_ = 1.0f / edt_ogm_resolution;

  // If obstacles occupy this grid, true
  bool car_with_mirror_obs_ogm_[edt_ogm_grid_x_max][edt_ogm_grid_y_max] = {};
  bool car_without_mirror_obs_ogm_[edt_ogm_grid_x_max][edt_ogm_grid_y_max] = {};
  bool car_chassis_obs_ogm_[edt_ogm_grid_x_max][edt_ogm_grid_y_max] = {};

  // 用数组存储障碍物信息
  OGMObsData car_with_mirror_ogm_obs_data_;
  OGMObsData car_without_mirror_ogm_obs_data_;
  OGMObsData car_chassis_ogm_obs_data_;
};

}  // namespace apa_planner
}  // namespace planning