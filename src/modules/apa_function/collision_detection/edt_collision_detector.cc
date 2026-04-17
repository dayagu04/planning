#include "edt_collision_detector.h"

#include <opencv2/imgproc/types_c.h>

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <utility>

#include "apa_obstacle.h"
#include "apa_param_config.h"
#include "collision_detection/base_collision_detector.h"
#include "geometry_math.h"
#include "log_glog.h"

namespace planning {
namespace apa_planner {

#define write_debug_file (0)

void EDTCollisionDetector::PreProcess(
    const UseObsHeightMethod use_obs_height_method) {
  use_obs_height_method_ = use_obs_height_method;
  AddObsToOGM();
  CalcObsDistArray();
}

void EDTCollisionDetector::PreProcess(
    const geometry_lib::RectangleBound &ogm_bound,
    const UseObsHeightMethod use_obs_height_method) {
  GenOccupancyGridMap(ogm_bound);
  PreProcess(use_obs_height_method);
}

void EDTCollisionDetector::GenOccupancyGridMap(
    const geometry_lib::RectangleBound &ogm_bound) {
  ogm_bound_ = ogm_bound;
  ogm_origin_ << ogm_bound.min_x, ogm_bound.min_y;
}

void EDTCollisionDetector::GenOccupancyGridMap(
    const geometry_lib::RectangleBound &ogm_bound, const double resolution) {
  resolution_ = resolution;
  resolution_inv_ = 1.0 / resolution;
  GenOccupancyGridMap(ogm_bound);
}

void EDTCollisionDetector::GenOccupancyGridMap(
    const Eigen::Vector2d &ogm_origin) {
  ogm_origin_ << ogm_origin.x(), ogm_origin.y();
  ogm_bound_.Set(ogm_origin_.x(), ogm_origin_.y(),
                 ogm_origin_.x() + edt_ogm_grid_x_max * resolution_ + 1.0,
                 ogm_origin_.y() + edt_ogm_grid_y_max * resolution_ + 1.0);
}

void EDTCollisionDetector::GenOccupancyGridMap(
    const Eigen::Vector2d &ogm_origin, const double resolution) {
  resolution_ = resolution;
  resolution_inv_ = 1.0 / resolution;
  GenOccupancyGridMap(ogm_origin);
}

const OGMIndex EDTCollisionDetector::GetIndexFromOGMPose(
    const Eigen::Vector2f &pt) {
  return OGMIndex(int(pt.x() * resolution_inv_), int(pt.y() * resolution_inv_));
}

const OGMIndex EDTCollisionDetector::GetIndexFromSlotPose(
    const Eigen::Vector2f &pt) {
  return GetIndexFromOGMPose(Eigen::Vector2f(pt.x() - ogm_origin_.x() + 1e-6f,
                                             pt.y() - ogm_origin_.y() + 1e-6f));
}

const bool EDTCollisionDetector::IsIndexValid(const OGMIndex &id) const {
  if (id.x >= edt_ogm_grid_x_max || id.y >= edt_ogm_grid_y_max || id.x < 0 ||
      id.y < 0) {
    return false;
  }

  return true;
}

void EDTCollisionDetector::AddObsToOGM() {
  Reset();
  // now only one default height is supported, which includes all car body
  const std::unordered_map<size_t, ApaObstacle> &obs_map =
      obs_manager_ptr_->GetObstacles();

  ILOG_INFO << "AddObsToOGM, obs_map size: " << obs_map.size();

  ogm_bound_.PrintInfo();

  OGMIndex index;
  // add pt cloud
  bool(*obs_ogm)[edt_ogm_grid_y_max] = nullptr;
  for (const auto &obs_pair : obs_map) {
    const ApaObstacle &obs = obs_pair.second;
    ApaObsHeightType obs_height_type = obs.GetObsHeightType();
    if (use_obs_height_method_ == UseObsHeightMethod::HIGH) {
      obs_height_type = ApaObsHeightType::HIGH;
    } else if (use_obs_height_method_ == UseObsHeightMethod::HIGH_LOW) {
      if (obs_height_type == ApaObsHeightType::MID) {
        obs_height_type = ApaObsHeightType::HIGH;
      }
    } else {
      // do nothing
    }
    switch (obs_height_type) {
      case ApaObsHeightType::RUN_OVER:
        continue;
      case ApaObsHeightType::LOW:
        obs_ogm = car_chassis_obs_ogm_;
        break;
      case ApaObsHeightType::MID:
        obs_ogm = car_without_mirror_obs_ogm_;
        break;
      default:
        obs_ogm = car_with_mirror_obs_ogm_;
        break;
    }

    const std::vector<Eigen::Vector2d> pt_clout_2d = obs.GetPtClout2dLocal();
    for (const Eigen::Vector2d &pt : pt_clout_2d) {
      index = GetIndexFromSlotPose(Eigen::Vector2f(pt.x(), pt.y()));
      if (IsIndexValid(index)) {
        obs_ogm[index.x][index.y] = true;
      }
    }
  }

  // todo: add line obs and more other
}

void EDTCollisionDetector::Reset() {
  memset(car_with_mirror_obs_ogm_, false,
         sizeof(bool) * edt_ogm_grid_x_max * edt_ogm_grid_y_max);
  memset(car_without_mirror_obs_ogm_, false,
         sizeof(bool) * edt_ogm_grid_x_max * edt_ogm_grid_y_max);
  memset(car_chassis_obs_ogm_, false,
         sizeof(bool) * edt_ogm_grid_x_max * edt_ogm_grid_y_max);
  return;
}

void EDTCollisionDetector::TransformObsOGMToMatrix(
    cv::Mat *mat, const bool (*obs_ogm)[edt_ogm_grid_y_max]) const {
  OGMIndex index;
  // set brightness value to represent whether it is occupied
  // 0 represent occupied
  for (int i = 0; i < mat->rows; ++i) {
    uchar *data = mat->ptr<uchar>(i);
    for (int j = 0; j < mat->cols; ++j) {
      index.Set(i, j);
      if (IsIndexValid(index) && obs_ogm[i][j]) {
        data[j] = 0;
      }
    }
  }
}

void EDTCollisionDetector::CalcObsDistArray() {
  const int max_bound_x = std::round(ogm_bound_.length * resolution_inv_);
  const int max_bound_y = std::round(ogm_bound_.width * resolution_inv_);

  ILOG_INFO << "max_bound_x = " << max_bound_x
            << " max_bound_y = " << max_bound_y;

  // 200 represent no obs, 0 represent obs occ
  // CV_8UC1 represent uint8 single channel matrix, which is a grayscale image
  cv::Mat car_with_mirror_map_matrix(max_bound_x, max_bound_y, CV_8UC1,
                                     cv::Scalar(200));
  cv::Mat car_without_mirror_map_matrix(max_bound_x, max_bound_y, CV_8UC1,
                                        cv::Scalar(200));
  cv::Mat car_chassis_map_matrix(max_bound_x, max_bound_y, CV_8UC1,
                                 cv::Scalar(200));

  // Calculate the nearest distance from each pixel to other zero pixels
  cv::Mat car_with_mirror_edt_matrix, car_without_mirror_edt_matrix,
      car_chassis_edt_matrix;

  TransformObsOGMToMatrix(&car_with_mirror_map_matrix,
                          car_with_mirror_obs_ogm_);
  cv::distanceTransform(car_with_mirror_map_matrix, car_with_mirror_edt_matrix,
                        CV_DIST_L2, CV_DIST_MASK_PRECISE);

  if (use_obs_height_method_ == UseObsHeightMethod::HIGH_LOW) {
    TransformObsOGMToMatrix(&car_chassis_map_matrix, car_chassis_obs_ogm_);
    cv::distanceTransform(car_chassis_map_matrix, car_chassis_edt_matrix,
                          CV_DIST_L2, CV_DIST_MASK_PRECISE);
  } else if (use_obs_height_method_ == UseObsHeightMethod::HIGH_MID_LOW) {
    TransformObsOGMToMatrix(&car_without_mirror_map_matrix,
                            car_without_mirror_obs_ogm_);
    cv::distanceTransform(car_without_mirror_map_matrix,
                          car_without_mirror_edt_matrix, CV_DIST_L2,
                          CV_DIST_MASK_PRECISE);

    TransformObsOGMToMatrix(&car_chassis_map_matrix, car_chassis_obs_ogm_);
    cv::distanceTransform(car_chassis_map_matrix, car_chassis_edt_matrix,
                          CV_DIST_L2, CV_DIST_MASK_PRECISE);
  }

  // only protect, avoid crash caused by excessive boundary values
  const int max_row_num = std::min(max_bound_x, edt_ogm_grid_x_max);
  const int max_col_num = std::min(max_bound_y, edt_ogm_grid_y_max);
  for (int row = 0; row < max_row_num; ++row) {
    float *car_with_mirror_data, *car_without_mirror_data, *car_chassis_data;
    car_with_mirror_data = car_with_mirror_edt_matrix.ptr<float>(row);
    if (use_obs_height_method_ == UseObsHeightMethod::HIGH_LOW) {
      car_chassis_data = car_chassis_edt_matrix.ptr<float>(row);
    } else if (use_obs_height_method_ == UseObsHeightMethod::HIGH_MID_LOW) {
      car_without_mirror_data = car_without_mirror_edt_matrix.ptr<float>(row);
      car_chassis_data = car_chassis_edt_matrix.ptr<float>(row);
    }

    for (int col = 0; col < max_col_num; ++col) {
      car_with_mirror_ogm_obs_data_.dist[row][col] =
          car_with_mirror_data[col] * resolution_;
      if (use_obs_height_method_ == UseObsHeightMethod::HIGH_LOW) {
        car_chassis_ogm_obs_data_.dist[row][col] =
            car_chassis_data[col] * resolution_;
      } else if (use_obs_height_method_ == UseObsHeightMethod::HIGH_MID_LOW) {
        car_without_mirror_ogm_obs_data_.dist[row][col] =
            car_without_mirror_data[col] * resolution_;
        car_chassis_ogm_obs_data_.dist[row][col] =
            car_chassis_data[col] * resolution_;
      }
    }
  }

#if write_debug_file
  cv::flip(car_with_mirror_map_matrix, car_with_mirror_map_matrix, 0);
  cv::flip(car_with_mirror_map_matrix, car_with_mirror_map_matrix, 1);
  cv::flip(car_with_mirror_edt_matrix, car_with_mirror_edt_matrix, 0);
  cv::flip(car_with_mirror_edt_matrix, car_with_mirror_edt_matrix, 1);
#if defined(X86) && !defined(X86_SIMULATION)
  std::string path_dir = "/asw/planning/glog/";
#else
  std::string path_dir = "/opt/usr/log/app_log/planning/";
#endif
  cv::imwrite(path_dir + "car_with_mirror_ogm.png", car_with_mirror_map_matrix);
  cv::imwrite(path_dir + "car_with_mirror_edt.png", car_with_mirror_edt_matrix);
  cv::flip(car_without_mirror_map_matrix, car_without_mirror_map_matrix, 0);
  cv::flip(car_without_mirror_map_matrix, car_without_mirror_map_matrix, 1);
  cv::flip(car_without_mirror_edt_matrix, car_without_mirror_edt_matrix, 0);
  cv::flip(car_without_mirror_edt_matrix, car_without_mirror_edt_matrix, 1);
  cv::imwrite(path_dir + "car_without_mirror_ogm.png",
              car_without_mirror_map_matrix);
  cv::imwrite(path_dir + "car_without_mirror_edt.png",
              car_without_mirror_edt_matrix);
  cv::flip(car_chassis_map_matrix, car_chassis_map_matrix, 0);
  cv::flip(car_chassis_map_matrix, car_chassis_map_matrix, 1);
  cv::flip(car_chassis_edt_matrix, car_chassis_edt_matrix, 0);
  cv::flip(car_chassis_edt_matrix, car_chassis_edt_matrix, 1);
  cv::imwrite(path_dir + "car_chassis_ogm.png", car_chassis_map_matrix);
  cv::imwrite(path_dir + "car_chassis_edt.png", car_chassis_edt_matrix);
#endif
}

const double EDTCollisionDetector::GetObsDistByIndex(
    const OGMIndex &id, const ApaObsHeightType &height_type) {
  if (IsIndexValid(id)) {
    switch (height_type) {
      case ApaObsHeightType::RUN_OVER:
        return 26.8f;
      case ApaObsHeightType::LOW:
        return car_chassis_ogm_obs_data_.dist[id.x][id.y];
      case ApaObsHeightType::MID:
        return car_without_mirror_ogm_obs_data_.dist[id.x][id.y];
      default:
        return car_with_mirror_ogm_obs_data_.dist[id.x][id.y];
    }
  }

  return 0.0;
}

void EDTCollisionDetector::UpdateSafeBuffer(
    const ColDetBuffer &col_det_buffer) {
  if (!need_update_buffer_ &&
      common_math::IsTwoNumerEqual(col_det_buffer_.body_lat_buffer,
                                   col_det_buffer.body_lat_buffer) &&
      common_math::IsTwoNumerEqual(col_det_buffer_.mirror_lat_buffer,
                                   col_det_buffer.mirror_lat_buffer) &&
      common_math::IsTwoNumerEqual(col_det_buffer_.max_circle_buffer,
                                   col_det_buffer.max_circle_buffer)) {
    col_det_buffer_.lon_buffer = col_det_buffer.lon_buffer;
    return;
  }

  need_update_buffer_ = false;
  col_det_buffer_ = col_det_buffer;

  UpdateSingleCircleListSafeBuffer(
      multi_car_shape_circle_.circles_with_mirror,
      multi_car_shape_circle_with_buffer_.circles_with_mirror,
      ApaObsHeightType::HIGH, true);
  UpdateSingleCircleListSafeBuffer(
      multi_car_shape_circle_.circles_without_mirror,
      multi_car_shape_circle_with_buffer_.circles_without_mirror,
      ApaObsHeightType::MID, false);
  UpdateSingleCircleListSafeBuffer(
      multi_car_shape_circle_.chassis_circles,
      multi_car_shape_circle_with_buffer_.chassis_circles,
      ApaObsHeightType::LOW, false);
}

void EDTCollisionDetector::UpdateSingleCircleListSafeBuffer(
    const CarFootPrintCircleList &src, CarFootPrintCircleList &dst,
    ApaObsHeightType height_type, bool has_mirror) {
  dst.count = src.count;
  dst.max_circle.center_local = src.max_circle.center_local;
  dst.max_circle.radius =
      src.max_circle.radius + col_det_buffer_.max_circle_buffer;
  dst.height_type = height_type;

  // Index mapping depends on whether mirror circles exist:
  //   with_mirror:    left={0,4} right={1,3} mirror={2,5} front=6 rear=9
  //   without_mirror: left={0,3} right={1,2}              front=4 rear=7
  const size_t front_id = has_mirror ? 6 : 4;
  const size_t rear_id = has_mirror ? 9 : 7;

  for (size_t i = 0; i < dst.count; ++i) {
    dst.circles[i].center_local = src.circles[i].center_local;
    dst.circles[i].radius = src.circles[i].radius;

    if (has_mirror && (i == 2 || i == 5)) {
      // mirror circles: expand radius by mirror buffer
      dst.circles[i].radius += col_det_buffer_.mirror_lat_buffer;
    } else if (has_mirror ? (i == 0 || i == 4) : (i == 0 || i == 3)) {
      // left body circles: shift toward left
      dst.circles[i].center_local.y() += col_det_buffer_.body_lat_buffer;
    } else if (has_mirror ? (i == 1 || i == 3) : (i == 1 || i == 2)) {
      // right body circles: shift toward right
      dst.circles[i].center_local.y() -= col_det_buffer_.body_lat_buffer;
    } else if (i == front_id) {
      // front circle: expand + still tangent to the front lane of the car
      dst.circles[i].radius += col_det_buffer_.body_lat_buffer;
      dst.circles[i].center_local.x() -= col_det_buffer_.body_lat_buffer;
    } else if (i == rear_id) {
      // rear circle: expand + still tangent to the rear lane of the car
      dst.circles[i].radius += col_det_buffer_.body_lat_buffer;
      dst.circles[i].center_local.x() += col_det_buffer_.body_lat_buffer;
    } else {
      // other circles in car body: expand radius
      dst.circles[i].radius += col_det_buffer_.body_lat_buffer;
    }
  }
}

const bool EDTCollisionDetector::IsCollisionForPoint(
    const geometry_lib::PathPoint &pt,
    CarFootPrintCircleList *car_circle_list) {
  car_circle_list->LocalToGlobal(pt);

  CarFootPrintCircle *circle = nullptr;
  // grid number where the circle center is located
  OGMIndex center_index;
  // the minimum distance between the obs and the circle boundary
  double dist{0.0};

  // first check max circle
  circle = &car_circle_list->max_circle;
  center_index = GetIndexFromSlotPose(circle->center_global);
  if (!IsIndexValid(center_index)) {
    return true;
  }
  dist = GetObsDistByIndex(center_index, car_circle_list->height_type) -
         circle->radius;
  if (dist > 0.0) {
    return false;
  }

  // obs is in max circle, need check every other circle
  for (uint8_t i = 0; i < car_circle_list->count; ++i) {
    circle = &car_circle_list->circles[i];
    center_index = GetIndexFromSlotPose(circle->center_global);
    if (!IsIndexValid(center_index)) {
      return true;
    }
    dist = GetObsDistByIndex(center_index, car_circle_list->height_type) -
           circle->radius;
    if (dist < 0.0f) {
      return true;
    }
  }

  return false;
}

const bool EDTCollisionDetector::IsCollisionForPoint(
    const geometry_lib::PathPoint &pt, CarFootPrintCircleList *car_circle_list,
    float *min_dist, int *circle_id, const float safe_dist) {
  car_circle_list->LocalToGlobal(pt);

  CarFootPrintCircle *circle = nullptr;
  // grid number where the circle center is located
  OGMIndex center_index;
  // the minimum distance between the obs and the circle boundary
  float dist{0.0f};

  // first check max circle
  circle = &car_circle_list->max_circle;
  center_index = GetIndexFromSlotPose(circle->center_global);
  if (!IsIndexValid(center_index)) {
    *min_dist = 0.0f;
    *circle_id = -1;
    return true;
  }
  dist = GetObsDistByIndex(center_index, car_circle_list->height_type) -
         circle->radius;
  if (dist > safe_dist) {
    *min_dist = dist;
    *circle_id = -1;
    return false;
  }

  // although obs may be outside the max circle, it is necessary to check
  // every circle in order to calculate the distance from the obs to the car
  *min_dist = std::numeric_limits<double>::infinity();
  for (uint8_t i = 0; i < car_circle_list->count; ++i) {
    circle = &car_circle_list->circles[i];
    center_index = GetIndexFromSlotPose(circle->center_global);
    if (!IsIndexValid(center_index)) {
      *min_dist = 0.0f;
      *circle_id = -1;
      return true;
    }
    dist = GetObsDistByIndex(center_index, car_circle_list->height_type) -
           circle->radius;
    if (dist < 0.0f) {
      *min_dist = dist;
      *circle_id = -1;
      return true;
    }
    if (dist < *min_dist) {
      *min_dist = dist;
      *circle_id = i;
    }
  }

  return false;
}

const bool EDTCollisionDetector::IsCollisionForPoint(
    const common_math::PathPt<float> &pt,
    CarFootPrintCircleList *car_circle_list) {
  car_circle_list->LocalToGlobal(pt);
  CarFootPrintCircle *circle = nullptr;
  // grid number where the circle center is located
  OGMIndex center_index;
  // the minimum distance between the obs and the circle boundary
  float dist{0.0f};
  // first check max circle
  circle = &car_circle_list->max_circle;
  center_index = GetIndexFromSlotPose(circle->center_global);
  if (!IsIndexValid(center_index)) {
    return true;
  }
  dist = GetObsDistByIndex(center_index, car_circle_list->height_type) -
         circle->radius;
  if (dist > 0.0f) {
    return false;
  }

  // obs is in max circle, need check every other circle
  for (uint8_t i = 0; i < car_circle_list->count; ++i) {
    circle = &car_circle_list->circles[i];
    center_index = GetIndexFromSlotPose(circle->center_global);
    if (!IsIndexValid(center_index)) {
      return true;
    }
    dist = GetObsDistByIndex(center_index, car_circle_list->height_type) -
           circle->radius;
    if (dist < 0.0f) {
      return true;
    }
  }

  return false;
}

const bool EDTCollisionDetector::IsCollisionForPoint(
    const common_math::PathPt<float> &pt,
    CarFootPrintCircleList *car_circle_list, float *min_dist,
    const float safe_dist) {
  car_circle_list->LocalToGlobal(pt);
  CarFootPrintCircle *circle = nullptr;
  // grid number where the circle center is located
  OGMIndex center_index;
  // the minimum distance between the obs and the circle boundary
  float dist{0.0f};
  // first check max circle
  circle = &car_circle_list->max_circle;
  center_index = GetIndexFromSlotPose(circle->center_global);
  if (!IsIndexValid(center_index)) {
    *min_dist = 0.0f;
    return true;
  }

  dist = GetObsDistByIndex(center_index, car_circle_list->height_type) -
         circle->radius;
  if (dist > safe_dist) {
    *min_dist = dist;
    return false;
  }

  // although obs may be outside the max circle, it is necessary to check
  // every circle in order to calculate the distance from the obs to the car
  *min_dist = std::numeric_limits<float>::infinity();
  for (uint8_t i = 0; i < car_circle_list->count; ++i) {
    circle = &car_circle_list->circles[i];
    center_index = GetIndexFromSlotPose(circle->center_global);
    if (!IsIndexValid(center_index)) {
      *min_dist = 0.0f;
      return true;
    }
    dist = GetObsDistByIndex(center_index, car_circle_list->height_type) -
           circle->radius;
    if (dist < 0.0f) {
      *min_dist = dist;
      return true;
    }
    if (dist < *min_dist) {
      *min_dist = dist;
    }
  }

  return false;
}

const ColResult EDTCollisionDetector::Update(
    const geometry_lib::PathSegment &path_seg,
    const ColDetBuffer &col_det_buffer, const bool need_cal_obs_dist) {
  std::vector<geometry_lib::PathPoint> pt_vec;
  geometry_lib::SamplePointSetInPathSeg(pt_vec, path_seg, sample_ds_);
  return Update(pt_vec, col_det_buffer, need_cal_obs_dist);
}

const ColResult EDTCollisionDetector::Update(
    const std::vector<geometry_lib::PathPoint> &pt_vec,
    const ColDetBuffer &col_det_buffer, const bool need_cal_obs_dist) {
  // The input PathPoint s must be assigned a value
  col_res_.Reset();
  const size_t N = pt_vec.size();
  if (obs_manager_ptr_ == nullptr || obs_manager_ptr_->GetObstacles().empty() ||
      N == 0) {
    return col_res_;
  }

  col_res_.remain_car_dist = pt_vec.back().s;
  col_res_.remain_dist = pt_vec.back().s;

  UpdateSafeBuffer(col_det_buffer);

  path_pt_vec_ = pt_vec;

  const uint8_t gear = path_pt_vec_.back().gear;

  if (gear == geometry_lib::SEG_GEAR_DRIVE ||
      gear == geometry_lib::SEG_GEAR_REVERSE) {
    Eigen::Vector2d heading_vec =
        geometry_lib::GenHeadingVec(path_pt_vec_.back().heading);
    if (gear == geometry_lib::SEG_GEAR_REVERSE) {
      heading_vec *= -1.0;
    }
    double s = 0.0, ds = 0.1;
    const geometry_lib::PathPoint pt = path_pt_vec_.back();
    geometry_lib::PathPoint temp_pt = pt;
    do {
      s += ds;
      if (s >= col_det_buffer_.lon_buffer) {
        s = col_det_buffer_.lon_buffer;
      }
      temp_pt.pos = pt.pos + heading_vec * s;
      temp_pt.s = pt.s + s;
      path_pt_vec_.emplace_back(temp_pt);
    } while (s < col_det_buffer_.lon_buffer - 0.001);
  } else if (N > 1 && col_det_buffer.lon_buffer > 0.01) {
    const Eigen::Vector2d start_point(path_pt_vec_[N - 2].pos.x(),
                                      path_pt_vec_[N - 2].pos.y());

    const Eigen::Vector2d end_point(path_pt_vec_[N - 1].pos.x(),
                                    path_pt_vec_[N - 1].pos.y());

    const double ds =
        std::max(path_pt_vec_[N - 1].s - path_pt_vec_[N - 2].s, 0.05);

    geometry_lib::PathPoint pt = path_pt_vec_.back();
    double s = 0.0;
    do {
      s += ds;
      if (s >= col_det_buffer_.lon_buffer) {
        s = col_det_buffer_.lon_buffer;
      }
      geometry_lib::CalExtendedPointByTwoPoints(start_point, end_point, pt.pos,
                                                s);
      pt.s = col_res_.remain_car_dist + s;
      path_pt_vec_.emplace_back(pt);
    } while (s < col_det_buffer_.lon_buffer);
  }

  CarFootPrintCircleList *high_circle_list =
      &multi_car_shape_circle_with_buffer_.circles_with_mirror;
  CarFootPrintCircleList *mid_circle_list =
      &multi_car_shape_circle_with_buffer_.circles_without_mirror;
  CarFootPrintCircleList *low_circle_list =
      &multi_car_shape_circle_with_buffer_.chassis_circles;

  bool col_flag = false;
  double lon_safe_dist = 0.0;
  float obs_dist_high = 26.8f, obs_dist_mid = 26.8f, obs_dist_low = 26.8f;
  int circle_id = -1;
  geometry_lib::CarSafePos car_safe_pos = geometry_lib::CarSafePos::ALL;
  for (const geometry_lib::PathPoint &pt : path_pt_vec_) {
    circle_id = -1;
    obs_dist_high = 26.8f, obs_dist_mid = 26.8f, obs_dist_low = 26.8f;
    if (!IsPoseInClearZone(pt)) {
      col_flag = need_cal_obs_dist
                     ? IsCollisionForPoint(pt, high_circle_list, &obs_dist_high,
                                           &circle_id)
                     : IsCollisionForPoint(pt, high_circle_list);
      if (col_flag) {
        break;
      }

      if (use_obs_height_method_ == UseObsHeightMethod::HIGH_LOW) {
        col_flag = need_cal_obs_dist
                       ? IsCollisionForPoint(pt, low_circle_list, &obs_dist_low,
                                             &circle_id)
                       : IsCollisionForPoint(pt, low_circle_list);
        if (col_flag) {
          break;
        }
      } else if (use_obs_height_method_ == UseObsHeightMethod::HIGH_MID_LOW) {
        col_flag = need_cal_obs_dist
                       ? IsCollisionForPoint(pt, mid_circle_list, &obs_dist_mid,
                                             &circle_id)
                       : IsCollisionForPoint(pt, mid_circle_list);

        if (col_flag) {
          break;
        }

        col_flag = need_cal_obs_dist
                       ? IsCollisionForPoint(pt, low_circle_list, &obs_dist_low,
                                             &circle_id)
                       : IsCollisionForPoint(pt, low_circle_list);

        if (col_flag) {
          break;
        }
      } else {
        // only detect high obs
      }
    }

    if (need_cal_obs_dist && pt.s < col_res_.remain_car_dist + 0.01) {
      if (circle_id == -1) {
        car_safe_pos = geometry_lib::CarSafePos::ALL;
      } else if (circle_id == 0 || circle_id == 1 || circle_id == 2 ||
                 circle_id == 5 || circle_id == 6) {
        car_safe_pos = geometry_lib::CarSafePos::CAR_REAR;
      } else {
        car_safe_pos = geometry_lib::CarSafePos::CAR_FRONT;
      }

      col_res_.pt_obs_dist_info_vec.emplace_back(geometry_lib::Pt2ObsDistInfo(
          std::pair<double, geometry_lib::PathPoint>{
              std::min({obs_dist_high, obs_dist_mid, obs_dist_low}) +
                  col_det_buffer_.body_lat_buffer,
              pt},
          circle_id, car_safe_pos));
    }

    lon_safe_dist = pt.s;
  }

  col_res_.col_flag = col_flag;
  col_res_.remain_dist = lon_safe_dist - col_det_buffer.lon_buffer;
  int safe_pt_number = static_cast<int>(col_res_.remain_dist / (sample_ds_));
  safe_pt_number = std::max(safe_pt_number, 0);
  if (col_res_.pt_obs_dist_info_vec.size() > safe_pt_number + 1) {
    col_res_.pt_obs_dist_info_vec.resize(safe_pt_number + 1);
  }

  double min_obs_dist = std::numeric_limits<double>::infinity();
  geometry_lib::PathPoint pt_closest_to_obs;
  for (auto &info : col_res_.pt_obs_dist_info_vec) {
    if (info.dist_pt.first < min_obs_dist) {
      min_obs_dist = info.dist_pt.first;
      pt_closest_to_obs = info.dist_pt.second;
    }
  }
  // min_obs_dist is the dist that obs to expanded car
  col_res_.pt_closest2obs = std::make_pair(min_obs_dist, pt_closest_to_obs);

  return col_res_;
}

const ColResultF EDTCollisionDetector::Update(
    const std::vector<common_math::PathPt<float>> &pt_vec,
    const ColDetBuffer &col_det_buffer, const bool need_cal_obs_dist) {
  col_res_f_.Reset();
  if (pt_vec.empty()) {
    return col_res_f_;
  }

  pts_ = std::move(pt_vec);

  UpdateSafeBuffer(col_det_buffer);

  const float origin_length = pts_.back().s;

  const AstarPathGear gear = pts_.back().gear;

  if (col_det_buffer_.lon_buffer > 0.01f &&
      (gear == AstarPathGear::DRIVE || gear == AstarPathGear::REVERSE)) {
    common_math::Pos<float> dir =
        common_math::CalDirFromTheta(pts_.back().theta);
    if (gear == AstarPathGear::REVERSE) {
      dir *= -1.0f;
    }
    float s = 0.0f, ds = 0.1f;
    const common_math::PathPt<float> pt = pts_.back();
    common_math::PathPt<float> temp_pt = pt;
    do {
      s += ds;
      if (s >= col_det_buffer_.lon_buffer + 0.001f) {
        s = col_det_buffer_.lon_buffer;
      }
      temp_pt.pos = pt.pos + dir * s;
      temp_pt.s = pt.s + s;
      pts_.emplace_back(temp_pt);
    } while (s < col_det_buffer_.lon_buffer - 0.001f);
  }

  CarFootPrintCircleList *high_circle_list =
      &multi_car_shape_circle_with_buffer_.circles_with_mirror;
  CarFootPrintCircleList *mid_circle_list =
      &multi_car_shape_circle_with_buffer_.circles_without_mirror;
  CarFootPrintCircleList *low_circle_list =
      &multi_car_shape_circle_with_buffer_.chassis_circles;

  float min_obs_dist = 26.8f, lon_safe_dist = 0.0f;
  float obs_dist_high = 26.8f, obs_dist_mid = 26.8f, obs_dist_low = 26.8f;
  bool col_flag = false;
  Eigen::Vector2f dangerous_pt;
  for (const common_math::PathPt<float> &pt : pts_) {
    obs_dist_high = 26.8f, obs_dist_mid = 26.8f, obs_dist_low = 26.8f;
    if (!IsPoseInClearZone(pt)) {
      col_flag = need_cal_obs_dist
                     ? IsCollisionForPoint(pt, high_circle_list, &obs_dist_high)
                     : IsCollisionForPoint(pt, high_circle_list);
      if (col_flag) {
        dangerous_pt = pt.GetPos();
        break;
      }
      if (use_obs_height_method_ == UseObsHeightMethod::HIGH_LOW) {
        col_flag = need_cal_obs_dist
                       ? IsCollisionForPoint(pt, low_circle_list, &obs_dist_low)
                       : IsCollisionForPoint(pt, low_circle_list);
        if (col_flag) {
          dangerous_pt = pt.GetPos();
          break;
        }
      } else if (use_obs_height_method_ == UseObsHeightMethod::HIGH_MID_LOW) {
        col_flag = need_cal_obs_dist
                       ? IsCollisionForPoint(pt, mid_circle_list, &obs_dist_mid)
                       : IsCollisionForPoint(pt, mid_circle_list);
        if (col_flag) {
          dangerous_pt = pt.GetPos();
          break;
        }

        col_flag = need_cal_obs_dist
                       ? IsCollisionForPoint(pt, low_circle_list, &obs_dist_low)
                       : IsCollisionForPoint(pt, low_circle_list);
        if (col_flag) {
          dangerous_pt = pt.GetPos();
          break;
        }
      } else {
        // only detect high obs
      }
    }

    if (need_cal_obs_dist && pt.s < origin_length + 1e-3f) {
      min_obs_dist =
          std::min({min_obs_dist, obs_dist_high, obs_dist_mid, obs_dist_low});
    }

    lon_safe_dist = pt.s;
  }

  col_res_f_.col_flag = col_flag;
  col_res_f_.remain_dist = lon_safe_dist - col_det_buffer_.lon_buffer;
  col_res_f_.min_obs_dist = min_obs_dist + col_det_buffer_.body_lat_buffer;
  col_res_f_.dangerous_path_pt = dangerous_pt;

  return col_res_f_;
}

}  // namespace apa_planner
}  // namespace planning