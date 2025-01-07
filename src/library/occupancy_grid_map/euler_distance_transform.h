#pragma once

#include <opencv2/core/types.hpp>
#include <opencv2/opencv.hpp>

#include "footprint_circle_model.h"
#include "geometry_math.h"
#include "occupancy_grid_coordinate.h"
#include "occupancy_grid_map.h"
#include "ogm_common.h"

namespace planning {

class EulerDistanceTransform : public OccupancyGridCoordinate {
 public:
  EulerDistanceTransform(){};

  // use default ROI bound to generate ogm.
  void Process(const Pose2D &ogm_pose,
               const double _ogm_resolution = ogm_resolution) override;

  bool Excute(const OccupancyGridMap &map, const Pose2D &ogm_pose,
              const double _ogm_resolution = ogm_resolution);

  // use user ROI bound to generate ogm.
  void Process(const OccupancyGridBound &bound,
               const double _ogm_resolution = ogm_resolution) override;

  bool Excute(const OccupancyGridMap &map, const OccupancyGridBound &bound,
              const double _ogm_resolution = ogm_resolution);

  void CVMatrixToArray(cv::Mat *edt_matrix);

  const float GetDistanceByIndex(const OgmIndex &id);

  void CopyEDT(const EDTData &data);

  /** 到障碍物距离大于1.35米, 规划不再关心具体数值，所以此时距离往往不准确.
   * 距离小于1.35米，API返回的距离值误差在5厘米左右.
   * 返回值: 是障碍物到膨胀后车体+膨胀后镜子的最小距离，而不是障碍物到车辆真实边界的距离,
   * 返回值距离误差在5厘米左右.
   * gear设定为DRIVE，此时距离会考虑车头膨胀;
   * gear设定为REVERSE，此时距离会考虑车尾膨胀;
   * gear设定为NONE，距离不会考虑车头车尾膨胀,只会考虑车身两侧膨胀;
   */
  const bool DistanceCheckForPoint(float *min_dist, Transform2d *tf,
                                   const AstarPathGear gear);

  const bool DistanceCheckForPoint(float *min_dist,
                                   const pnc::geometry_lib::PathPoint &pose,
                                   const uint8_t gear);

  const bool IsCollisionForPoint(Transform2d *tf, const AstarPathGear gear);

  const bool IsCollisionForPoint(const pnc::geometry_lib::PathPoint &pose,
                                 const uint8_t gear);

  const bool IsCollisionForPath(
      const std::vector<pnc::geometry_lib::PathPoint> &path_pt_vec,
      const uint8_t gear);

  const double CalPathSafeDist(
      const std::vector<pnc::geometry_lib::PathPoint> &path_pt_vec,
      const double ds, const uint8_t gear);

  const std::pair<double, double> CalPathRemainDistAndObsDist(
      const std::vector<pnc::geometry_lib::PathPoint> &path_pt_vec,
      const double ds, const uint8_t gear);

  void Init(const float car_body_lat_safe_buffer, const float lon_safe_buffer,
            const float mirror_buffer);

  void UpdateSafeBuffer(const float car_body_lat_safe_buffer,
                        const float lon_safe_buffer, const float mirror_buffer);

  const EDTData &GetConstEDTData() const { return data_; }

  const FootPrintCircleList GetCircleFootPrint(const AstarPathGear gear) const;

  const double GetLatetalSafeBuffer() const { return latetal_safe_buffer_; }

 private:
  EDTData data_;

  FootPrintCircleList global_circles_;
  FootPrintCircleModel footprint_model_;
  float latetal_safe_buffer_;
  float mirror_safe_buffer_;
  float lon_safe_buffer_;
};

}  // namespace planning