#pragma once

#include <assert.h>

#include <algorithm>
#include <iostream>
#include <memory>
#include <mutex>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <set>
#include <string>
#include <thread>
#include <vector>

#include "grid_map.h"
#include "reference_path.h"
#include "src/library/spiral/spiral_typedefs.h"

namespace planning {

using ObstacleMapType = uint8_t;
using SscMapDataType = uint8_t;

struct Point2i {
  int x = 0;
  int y = 0;

  /**
   * @brief  Default constructor
   */
  Point2i() = default;

  /**
   * @brief Construct a new Point 2i object
   *
   * @param xx
   * @param yy
   */
  Point2i(int xx, int yy) : x(xx), y(yy) {}
};

class SscMap {
 public:
  using GridMap3D = GridMapND<ObstacleMapType, 3>;

  struct Config {
    std::array<int, 3> map_size = {{200, 100, 26}};            // s, d, t
    std::array<double, 3> map_resolution = {{1.0, 0.2, 0.2}};  // m, m, s
    std::array<std::string, 3> axis_name = {{"s", "d", "t"}};
    double s_back_len = 0.0;
  };

  SscMap() {}
  SscMap(const Config &config);
  ~SscMap() {}

  GridMap3D *p_3d_grid() const { return p_3d_grid_; }
  GridMap3D *p_3d_inflated_grid() const { return p_3d_inflated_grid_; }

  Config config() const { return config_; }

  void UpdateMapOrigin(const FrenetEgoState &ego_state, const double &time);

  void ConstructSscMap(
      const AgentFrenetSpatioTemporalInFo &surround_trajs_state_info);

  void ClearGridMap();

  void ClearDrivingCorridor();

  void GetFinalGlobalMetricCubesList();

  void ResetSscMap(const FrenetEgoState &ego_frenet_state, const double &time);

 private:
  // 填充静态障碍物
  // void FillStaticPart(
  // const std::vector<AgentFrenetSpatioTemporalInFo> &obs_grid_fs);

  void FillDynamicPart(
      const AgentFrenetSpatioTemporalInFo &sur_vehicle_trajs_fs);

  void FillMapWithFsVehicleTraj(
      const std::vector<std::vector<SLTPoint>> &traj_point);

  // void GetCvPoint2iVecUsingCommonPoint2iVec(
  //     const std::vector<Point2i>& pts_in, std::vector<cv::Point2i>* pts_out);

  // void GetCvPoint2iUsingCommonPoint2i(const Point2i& pt_in,
  //                                           cv::Point2i* pt_out);

  GridMapND<SscMapDataType, 3> *p_3d_grid_;
  GridMapND<SscMapDataType, 3> *p_3d_inflated_grid_;

  std::unordered_map<int, std::array<bool, 6>> inters_for_cube_;

  Config config_;

  common::FrenetState initial_fs_;

  bool map_valid_ = false;
};

}  // namespace planning
