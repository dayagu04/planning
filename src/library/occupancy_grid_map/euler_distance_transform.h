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
  EulerDistanceTransform() {
    map_matrix_ = cv::Mat(ogm_grid_x_max, ogm_grid_y_max, CV_8UC1);
  };

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

  // SlotCoordinate
  const bool DistanceCheckForPoint(float *min_dist, Transform2d *tf,
                                   const AstarPathGear gear);

  const bool IsCollisionForPoint(Transform2d *tf, const AstarPathGear gear);

  const bool IsCollisionForPoint(const pnc::geometry_lib::PathPoint &pose,
                                 const uint8_t gear);

  const bool IsCollisionForPath(
      const std::vector<pnc::geometry_lib::PathPoint> &path_pt_vec,
      const uint8_t gear);

  void Init(const float car_body_lat_safe_buffer, const float lon_safe_buffer,
            const float mirror_buffer);

  void UpdateSafeBuffer(const float car_body_lat_safe_buffer,
                        const float lon_safe_buffer, const float mirror_buffer);

  const EDTData &GetConstEDTData() const { return data_; }

  const FootPrintCircleList GetCircleFootPrint(const AstarPathGear gear) const;

 private:
  EDTData data_;

  cv::Mat map_matrix_;

  FootPrintCircleList global_circles_;
  FootPrintCircleModel footprint_model_;
  float latetal_safe_buffer_;
  float mirror_safe_buffer_;
  float lon_safe_buffer_;
};

}  // namespace planning