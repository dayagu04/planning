
#include "euler_distance_transform.h"

#include <opencv2/imgproc/types_c.h>

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

void EulerDistanceTransform::Process(const Pose2D &ogm_pose,
                                     const double _ogm_resolution) {
  OccupancyGridCoordinate::Process(ogm_pose, _ogm_resolution);

  return;
}

void EulerDistanceTransform::Process(const OccupancyGridBound &bound,
                                     const double _ogm_resolution) {
  OccupancyGridCoordinate::Process(bound, _ogm_resolution);

  return;
}

bool EulerDistanceTransform::Excute(const OccupancyGridMap &map,
                                    const Pose2D &ogm_pose,
                                    const double _ogm_resolution) {
  OccupancyGridCoordinate::Process(ogm_pose, _ogm_resolution);

  cv::Mat map_matrix(ogm_grid_x_max, ogm_grid_y_max, CV_8UC1, cv::Scalar(200));

  map.TransformToMatrix(&map_matrix);

  // cv::Mat edt_matrix(ogm_grid_x_max, ogm_grid_y_max, CV_32FC1,
  // cv::Scalar(200));
  cv::Mat edt_matrix;

  // 计算每一个像素到其他零像素的最近距离
  cv::distanceTransform(map_matrix, edt_matrix, CV_DIST_L2,
                        CV_DIST_MASK_PRECISE);

  CVMatrixToArray(&edt_matrix);

#if write_debug_file
  cv::flip(map_matrix, map_matrix, 0);
  cv::flip(map_matrix, map_matrix, 1);
  cv::flip(edt_matrix, edt_matrix, 0);
  cv::flip(edt_matrix, edt_matrix, 1);
  cv::imwrite("/asw/planning/glog/ogm.png", map_matrix);
  cv::imwrite("/asw/planning/glog/edt.png", edt_matrix);
#endif

  return true;
}

bool EulerDistanceTransform::Excute(const OccupancyGridMap &map,
                                    const OccupancyGridBound &bound,
                                    const double _ogm_resolution) {
  OccupancyGridCoordinate::Process(bound, _ogm_resolution);

  const int max_bound_x =
      std::round((bound.max_x - bound.min_x) * ogm_resolution_inv_);
  const int max_bound_y =
      std::round((bound.max_y - bound.min_y) * ogm_resolution_inv_);

  cv::Mat map_matrix(max_bound_x, max_bound_y, CV_8UC1, cv::Scalar(200));

  map.TransformToMatrix(&map_matrix);

  // cv::Mat edt_matrix(ogm_grid_x_max, ogm_grid_y_max, CV_32FC1,
  // cv::Scalar(200));
  cv::Mat edt_matrix;

  // 计算每一个像素到其他零像素的最近距离
  cv::distanceTransform(map_matrix, edt_matrix, CV_DIST_L2,
                        CV_DIST_MASK_PRECISE);

  CVMatrixToArray(&edt_matrix);

#if write_debug_file
  cv::flip(map_matrix, map_matrix, 0);
  cv::flip(map_matrix, map_matrix, 1);
  cv::flip(edt_matrix, edt_matrix, 0);
  cv::flip(edt_matrix, edt_matrix, 1);
  cv::imwrite("/asw/planning/glog/ogm.png", map_matrix);
  cv::imwrite("/asw/planning/glog/edt.png", edt_matrix);
#endif

  return true;
}

void EulerDistanceTransform::CVMatrixToArray(cv::Mat *edt_matrix) {
  const int row_num = std::min(edt_matrix->rows, ogm_grid_x_max);
  const int column_num = std::min(edt_matrix->cols, ogm_grid_y_max);

  // ILOG_INFO << "r " << row_num << " max x " << ogm_grid_x_max;
  // ILOG_INFO << "c " << column_num << " max y " << ogm_grid_y_max;

  for (int j = 0; j < row_num; j++) {
    float *data = edt_matrix->ptr<float>(j);
    for (int i = 0; i < column_num; i++) {
      data_.dist[j][i] = data[i] * float(ogm_resolution_);
    }
  }

  return;
}

const float EulerDistanceTransform::GetDistanceByIndex(const OgmIndex &id) {
  if (IsIndexValid(id)) {
    return data_.dist[id.x][id.y];
  }

  return 10000.0;
}

void EulerDistanceTransform::CopyEDT(const EDTData &data) {
  data_ = data;
  return;
}

const bool EulerDistanceTransform::DistanceCheckForPoint(
    float *min_dist, Transform2d *tf, const AstarPathGear gear) {
  OgmIndex index;

  FootPrintCircle *circle;

  FootPrintCircleList *global_circles;
  global_circles = &global_circles_;

  footprint_model_.LocalToGlobalByGear(global_circles, tf, gear);

  float dist;

  // check max circle
  circle = &global_circles->max_circle;

  index.x = std::round((circle->pos.x - bound_.min_x) * ogm_resolution_inv_);
  index.y = std::round((circle->pos.y - bound_.min_y) * ogm_resolution_inv_);

  // out of bound
  if (!IsIndexValid(index)) {
#if DEBUG_EDT
    ILOG_INFO << "out of bound";
#endif

    return true;
  }

  dist = data_.dist[index.x][index.y] - circle->radius;
  if (dist > 1.0f) {
#if DEBUG_EDT
    ILOG_INFO << "dist " << dist << " r " << circle->radius << " ,safe "
              << circle->safe_buffer;
#endif

    *min_dist = dist;

    return false;
  }

  // check other
  float close_dist = 100.0f;
  for (int i = 0; i < global_circles->size; i++) {
    circle = &global_circles->circles[i];

    index.x = std::round((circle->pos.x - bound_.min_x) * ogm_resolution_inv_);
    index.y = std::round((circle->pos.y - bound_.min_y) * ogm_resolution_inv_);

    // out of bound
    if (!IsIndexValid(index)) {
#if DEBUG_EDT
      ILOG_INFO << "out of bound " << index.x << " y" << index.y
                << " x_diff_frame_ " << bound_.min_x << " y_diff_frame_ "
                << bound_.min_y << "  circle->pos.x " << circle->pos.x
                << " circle->pos.y " << circle->pos.y << " r "
                << circle->radius;
#endif
      *min_dist = 0.0f;
      return true;
    }

    dist = data_.dist[index.x][index.y] - circle->radius;
    if (dist < 0.0f) {
#if DEBUG_EDT
      ILOG_INFO << "collision " << i << " dist " << dist << " "
                << circle->radius;
#endif

      *min_dist = dist;

      return true;
    }

    if (dist  < close_dist) {
      close_dist = dist;

#if DEBUG_EDT
      ILOG_INFO << "dist " << dist << " r " << circle->radius << " ,safe "
                << circle->safe_buffer;
#endif
    }
  }

  *min_dist = close_dist;

  return false;
}

const bool EulerDistanceTransform::DistanceCheckForPoint(
    float *min_dist, const pnc::geometry_lib::PathPoint &pose,
    const uint8_t gear) {
  const AstarPathGear path_gear = (gear == pnc::geometry_lib::SEG_GEAR_DRIVE)
                                      ? AstarPathGear::DRIVE
                                      : AstarPathGear::REVERSE;

  const Pose2D pose_2d(pose.pos.x(), pose.pos.y(), pose.heading);

  Transform2d tf(pose_2d);

  return DistanceCheckForPoint(min_dist, &tf, path_gear);
}

const bool EulerDistanceTransform::IsCollisionForPoint(
    Transform2d *tf, const AstarPathGear gear) {
  FootPrintCircle *circle;
  FootPrintCircleList *global_circles;
  global_circles = &global_circles_;

  footprint_model_.LocalToGlobalByGear(global_circles, tf, gear);

  // check max circle
  circle = &global_circles->max_circle;

  OgmIndex index;
  index.x = std::round((circle->pos.x - bound_.min_x) * ogm_resolution_inv_);
  index.y = std::round((circle->pos.y - bound_.min_y) * ogm_resolution_inv_);

  // out of bound
  if (!IsIndexValid(index)) {
#if DEBUG_EDT
    ILOG_INFO << "out of bound";
#endif

    return true;
  }

  float dist;
  dist = data_.dist[index.x][index.y] - circle->radius;
  if (dist > 0.0f) {
#if DEBUG_EDT
    ILOG_INFO << "dist " << dist << " r " << circle->radius;
#endif

    return false;
  }

  // check other
  for (int i = 0; i < global_circles->size; i++) {
    circle = &global_circles->circles[i];

    index.x = std::round((circle->pos.x - bound_.min_x) * ogm_resolution_inv_);
    index.y = std::round((circle->pos.y - bound_.min_y) * ogm_resolution_inv_);

    // out of bound
    if (!IsIndexValid(index)) {
#if DEBUG_EDT
      ILOG_INFO << "out of bound " << index.x << " y" << index.y
                << " x_diff_frame_ " << bound_.min_x << " y_diff_frame_ "
                << bound_.min_y << "  circle->pos.x " << circle->pos.x
                << " circle->pos.y " << circle->pos.y << " r "
                << circle->radius;
#endif
      return true;
    }

    dist = data_.dist[index.x][index.y] - circle->radius;
    if (dist < 0.0f) {
#if DEBUG_EDT
      ILOG_INFO << "collision " << i << " dist " << dist << " "
                << circle->radius;
#endif

      return true;
    }
  }

  return false;
}

const bool EulerDistanceTransform::IsCollisionForPoint(
    const pnc::geometry_lib::PathPoint &pose, const uint8_t gear) {
  const AstarPathGear path_gear = (gear == pnc::geometry_lib::SEG_GEAR_DRIVE)
                                      ? AstarPathGear::DRIVE
                                      : AstarPathGear::REVERSE;

  const Pose2D pose_2d(pose.pos.x(), pose.pos.y(), pose.heading);

  Transform2d tf(pose_2d);

  return IsCollisionForPoint(&tf, path_gear);
}

void EulerDistanceTransform::Init(const float car_body_lat_safe_buffer,
                                  const float lon_safe_buffer,
                                  const float mirror_buffer) {
  UpdateSafeBuffer(car_body_lat_safe_buffer, lon_safe_buffer, mirror_buffer);

  return;
}

const double EulerDistanceTransform::CalPathSafeDist(
    const std::vector<pnc::geometry_lib::PathPoint> &path_pt_vec,
    const double ds, const uint8_t gear) {
  if (path_pt_vec.size() < 1) {
    return 0.0;
  }

  if (IsCollisionForPoint(path_pt_vec[0], gear)) {
    return 0.0;
  }

  double safe_dist = 0.0;
  for (size_t i = 1; i < path_pt_vec.size(); ++i) {
    if (IsCollisionForPoint(path_pt_vec[i], gear)) {
      break;
    } else {
      safe_dist += ds;
    }
  }

  return safe_dist;
}

const std::pair<double, double>
EulerDistanceTransform::CalPathRemainDistAndObsDist(
    const std::vector<pnc::geometry_lib::PathPoint> &path_pt_vec,
    const double ds, const uint8_t gear) {
  double remain_dist = 0.0;
  float obs_dist = 0.0;

  if (path_pt_vec.size() < 1) {
    return std::pair<double, double>{remain_dist, obs_dist};
  }

  if (DistanceCheckForPoint(&obs_dist, path_pt_vec[0], gear)) {
    return std::pair<double, double>{remain_dist, obs_dist};
  }

  float min_obs_dist = std::numeric_limits<float>::infinity();
  for (size_t i = 1; i < path_pt_vec.size(); ++i) {
    const bool col_flag =
        DistanceCheckForPoint(&obs_dist, path_pt_vec[i], gear);
    if (obs_dist < min_obs_dist) {
      min_obs_dist = obs_dist;
    }
    if (col_flag) {
      break;
    } else {
      remain_dist += ds;
    }
  }

  return std::pair<double, double>{remain_dist, min_obs_dist};
}

const bool EulerDistanceTransform::IsCollisionForPath(
    const std::vector<pnc::geometry_lib::PathPoint> &path_pt_vec,
    const uint8_t gear) {
  if (path_pt_vec.empty()) {
    return false;
  }

  // Prioritize checking the last point
  if (IsCollisionForPoint(path_pt_vec.back(), gear)) {
    return true;
  }

  for (size_t i = 0; i < path_pt_vec.size() - 1; ++i) {
    if (IsCollisionForPoint(path_pt_vec[i], gear)) {
      return true;
    }
  }

  return false;
}

void EulerDistanceTransform::UpdateSafeBuffer(
    const float car_body_lat_safe_buffer, const float lon_safe_buffer,
    const float mirror_buffer) {
  if (std::fabs(latetal_safe_buffer_ - car_body_lat_safe_buffer) < 0.001 &&
      std::fabs(lon_safe_buffer_ - lon_safe_buffer) < 0.001 &&
      std::fabs(mirror_safe_buffer_ - mirror_buffer) < 0.001) {
    return;
  }

  footprint_model_.UpdateSafeBuffer(car_body_lat_safe_buffer, lon_safe_buffer,
                                    mirror_buffer);

  global_circles_ = footprint_model_.GetLocalFootPrintCircle();

  latetal_safe_buffer_ = car_body_lat_safe_buffer;
  lon_safe_buffer_ = lon_safe_buffer;
  mirror_safe_buffer_ = mirror_buffer;

  return;
}

const FootPrintCircleList EulerDistanceTransform::GetCircleFootPrint(
    const AstarPathGear gear) const {
  return footprint_model_.GetLocalFootPrintCircleByGear(gear);
}

}  // namespace planning