#pragma once

#include <array>
#include <opencv2/core/types.hpp>
#include <opencv2/opencv.hpp>

#include "euler_distance_transform.h"
#include "footprint_circle_model.h"
#include "geometry_math.h"
#include "hierarchy_occupancy_grid_map.h"
#include "occupancy_grid_map.h"
#include "ogm_common.h"

namespace planning {

class HierarchyEulerDistanceTransform {
 public:
  HierarchyEulerDistanceTransform() {};

  // use default ROI bound to generate ogm.
  void Process(const Pose2f &ogm_pose,
               const float _ogm_resolution = ogm_resolution);

  bool Excute(const HierarchyOccupancyGridMap &hierarchy_ogm,
              const Pose2f &ogm_pose,
              const float _ogm_resolution = ogm_resolution);

  // use user ROI bound to generate ogm.
  void Process(const OccupancyGridBound &bound,
               const float _ogm_resolution = ogm_resolution);

  bool Excute(const HierarchyOccupancyGridMap &hierarchy_ogm,
              const OccupancyGridBound &bound,
              const float _ogm_resolution = ogm_resolution);

  const float GetDistanceByIndex(const OgmIndex &id);

  /** 到障碍物距离大于1.35米, 规划不再关心具体数值，所以此时距离往往不准确.
   * 距离小于1.35米，API返回的距离值误差在5厘米左右.
   * 返回值:
   * 是障碍物到膨胀后车体+膨胀后镜子的最小距离，而不是障碍物到车辆真实边界的距离,
   * 返回值距离误差在5厘米左右.
   * gear设定为DRIVE，此时距离会考虑车头膨胀;
   * gear设定为REVERSE，此时距离会考虑车尾膨胀;
   * gear设定为NONE，距离不会考虑车头车尾膨胀,只会考虑车身两侧膨胀;
   */
  const bool DistanceCheckForPoint(float *min_dist, Transform2f *tf,
                                   const AstarPathGear gear);

  const bool DistanceCheckForPoint(float *min_dist,
                                   const pnc::geometry_lib::PathPoint &pose,
                                   const uint8_t gear);

  const float DistanceCheckForPoint(const Pose2f &pose, const float radius);

  const bool IsCollisionForPoint(Transform2f *tf, const AstarPathGear gear);

  const bool IsCollisionForPoint(const pnc::geometry_lib::PathPoint &pose,
                                 const uint8_t gear);

  void Init(const float car_body_lat_safe_buffer, const float lon_safe_buffer,
            const float mirror_buffer);

  void UpdateSafeBuffer(const bool fold_mirror,
                        const float car_body_lat_safe_buffer,
                        const float lon_safe_buffer, const float mirror_buffer,
                        const float big_circle_safe_buffer = 0.35);

  const EulerDistanceTransform &GetSingleEDTData(size_t i) const;

  const std::array<EulerDistanceTransform, 2> &GetHierarchyEDTData() const {
    return hierarchy_edt_;
  }

  EulerDistanceTransform &GetMutableSingleEDTData(size_t i) const;

  std::array<EulerDistanceTransform, 2> *GetMutableHierarchyEDTData() {
    return &hierarchy_edt_;
  }

  EulerDistanceTransform *GetMutableSingleEDTData(size_t i);

  const size_t GetHierarchyEDTSize() const { return hierarchy_edt_size_; };

  const bool IsCollisionForPoint(
      Transform2f *tf, const AstarPathGear gear,
      MultiHeightFootPrintView *multi_height_footprint_model);

  void SetHierarchyEDTSize(const size_t i);

 private:
  std::array<EulerDistanceTransform, 2> hierarchy_edt_;
  size_t hierarchy_edt_size_ = 2;
};

}  // namespace planning