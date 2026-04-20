
#include "hierarchy_euler_distance_transform.h"

#include <opencv2/imgproc/types_c.h>

#include <algorithm>
#include <limits>
#include <opencv2/imgproc.hpp>

#include "log_glog.h"
#include "occupancy_grid_map.h"
#include "ogm_common.h"
#include "transform2d.h"

namespace planning {

#define write_debug_file (0)

#define DEBUG_EDT (0)

// a1000, opencv version:4.0.0
// x86, opencv version:3.2.0
// todo

void HierarchyEulerDistanceTransform::Process(const Pose2f &ogm_pose,
                                              const float _ogm_resolution) {
  for (EulerDistanceTransform &edt : hierarchy_edt_) {
    edt.Process(ogm_pose, _ogm_resolution);
  }

  return;
}

void HierarchyEulerDistanceTransform::Process(const OccupancyGridBound &bound,
                                              const float _ogm_resolution) {
  for (EulerDistanceTransform &edt : hierarchy_edt_) {
    edt.Process(bound, _ogm_resolution);
  }

  return;
}

bool HierarchyEulerDistanceTransform::Excute(
    const HierarchyOccupancyGridMap &hierarchy_ogm, const Pose2f &ogm_pose,
    const float _ogm_resolution) {
  if (hierarchy_ogm.GetHierarchyOGM().size() != hierarchy_edt_.size()) {
    return false;
  }

  hierarchy_edt_size_ = hierarchy_ogm.GetHierarchyOGMSize();

  for (size_t i = 0; i < hierarchy_ogm.GetHierarchyOGMSize(); i++) {
    const OccupancyGridMap &ogm = hierarchy_ogm.GetHierarchyOGM()[i];
    EulerDistanceTransform &edt = hierarchy_edt_[i];
    edt.Excute(ogm, ogm_pose, _ogm_resolution);
  }

  return true;
}

bool HierarchyEulerDistanceTransform::Excute(
    const HierarchyOccupancyGridMap &hierarchy_ogm,
    const OccupancyGridBound &bound, const float _ogm_resolution) {
  if (hierarchy_ogm.GetHierarchyOGM().size() != hierarchy_edt_.size()) {
    return false;
  }

  hierarchy_edt_size_ = hierarchy_ogm.GetHierarchyOGMSize();

  for (size_t i = 0; i < hierarchy_ogm.GetHierarchyOGMSize(); i++) {
    const OccupancyGridMap &ogm = hierarchy_ogm.GetHierarchyOGM()[i];
    EulerDistanceTransform &edt = hierarchy_edt_[i];
    edt.Excute(ogm, bound, _ogm_resolution);
  }
  return true;
}

void HierarchyEulerDistanceTransform::SetHierarchyEDTSize(const size_t i) {
  hierarchy_edt_size_ = i;
}

const bool HierarchyEulerDistanceTransform::DistanceCheckForPoint(
    float *min_dist, Transform2f *tf, const AstarPathGear gear) {
  float closest_dist = 100.0f;

  for (size_t i = 0; i < hierarchy_edt_size_; i++) {
    EulerDistanceTransform &edt = hierarchy_edt_[i];
    float layer_dist;
    if (edt.DistanceCheckForPoint(&layer_dist, tf, gear)) {
      // Collision detected - return immediately with the collision distance
      *min_dist = layer_dist;
      return true;
    }
    // No collision - track minimum distance across layers
    closest_dist = std::min(closest_dist, layer_dist);
  }

  *min_dist = closest_dist;
  return false;
}

const bool HierarchyEulerDistanceTransform::DistanceCheckForPoint(
    float *min_dist, const pnc::geometry_lib::PathPoint &pose,
    const uint8_t gear) {
  AstarPathGear path_gear;
  switch (gear) {
    case pnc::geometry_lib::SEG_GEAR_DRIVE:
      path_gear = AstarPathGear::DRIVE;
      break;
    case pnc::geometry_lib::SEG_GEAR_REVERSE:
      path_gear = AstarPathGear::REVERSE;
      break;
    default:
      path_gear = AstarPathGear::NONE;
      break;
  }

  const Pose2f pose_2d(pose.pos.x(), pose.pos.y(), pose.heading);
  Transform2f tf(pose_2d);

  return DistanceCheckForPoint(min_dist, &tf, path_gear);
}

const float HierarchyEulerDistanceTransform::DistanceCheckForPoint(
    const Pose2f &pose, const float radius) {
  float min_dist = 100.0f;
  for (size_t i = 0; i < hierarchy_edt_size_; i++) {
    EulerDistanceTransform &edt = hierarchy_edt_[i];
    min_dist = std::min(min_dist, edt.DistanceCheckForPoint(pose, radius));
  }

  return min_dist;
}

const bool HierarchyEulerDistanceTransform::IsCollisionForPoint(
    Transform2f *tf, const AstarPathGear gear) {
  for (size_t i = 0; i < hierarchy_edt_size_; i++) {
    EulerDistanceTransform &edt = hierarchy_edt_[i];
    if (edt.IsCollisionForPoint(tf, gear)) {
      return true;
    }
  }
  return false;
}

const bool HierarchyEulerDistanceTransform::IsCollisionForPoint(
    const pnc::geometry_lib::PathPoint &pose, const uint8_t gear) {
  const AstarPathGear path_gear = (gear == pnc::geometry_lib::SEG_GEAR_DRIVE)
                                      ? AstarPathGear::DRIVE
                                      : AstarPathGear::REVERSE;

  const Pose2f pose_2d(pose.pos.x(), pose.pos.y(), pose.heading);
  Transform2f tf(pose_2d);

  for (size_t i = 0; i < hierarchy_edt_size_; i++) {
    EulerDistanceTransform &edt = hierarchy_edt_[i];
    if (edt.IsCollisionForPoint(&tf, path_gear)) {
      return true;
    }
  }

  return false;
}

const bool HierarchyEulerDistanceTransform::IsCollisionForPoint(
    Transform2f *tf, const AstarPathGear gear,
    MultiHeightFootPrintView *multi_height_footprint_model) {
  if (multi_height_footprint_model == nullptr) {
    return true;
  }

  for (size_t i = 0; i < hierarchy_edt_size_; i++) {
    EulerDistanceTransform &edt = hierarchy_edt_[i];
    FootPrintCircleModel *footprint_model =
        &(multi_height_footprint_model->height_model[i]);
    if (edt.IsCollisionForPoint(tf, gear, footprint_model)) {
      return true;
    }
  }

  return false;
}

void HierarchyEulerDistanceTransform::Init(const float car_body_lat_safe_buffer,
                                           const float lon_safe_buffer,
                                           const float mirror_buffer) {
  for (size_t i = 0; i < hierarchy_edt_size_; i++) {
    EulerDistanceTransform &edt = hierarchy_edt_[i];
    if (i == hierarchy_edt_.size() - 1) {
      edt.Init(car_body_lat_safe_buffer, lon_safe_buffer, mirror_buffer, true);
    } else {
      edt.Init(car_body_lat_safe_buffer, lon_safe_buffer, mirror_buffer);
    }
  }

  return;
}

void HierarchyEulerDistanceTransform::UpdateSafeBuffer(
    const bool fold_mirror, const float car_body_lat_safe_buffer,
    const float lon_safe_buffer, const float mirror_buffer,
    const float big_circle_safe_buffer) {
  for (size_t i = 0; i < hierarchy_edt_size_; i++) {
    EulerDistanceTransform &edt = hierarchy_edt_[i];
    if (i == hierarchy_edt_.size() - 1) {
      edt.UpdateSafeBuffer(fold_mirror, car_body_lat_safe_buffer,
                           lon_safe_buffer, mirror_buffer,
                           big_circle_safe_buffer, true);
    } else {
      edt.UpdateSafeBuffer(fold_mirror, car_body_lat_safe_buffer,
                           lon_safe_buffer, mirror_buffer,
                           big_circle_safe_buffer);
    }
  }

  return;
}

const EulerDistanceTransform &HierarchyEulerDistanceTransform::GetSingleEDTData(
    size_t i) const {
  return hierarchy_edt_[i];
}

EulerDistanceTransform *
HierarchyEulerDistanceTransform::GetMutableSingleEDTData(size_t i) {
  return &hierarchy_edt_[i];
}

}  // namespace planning