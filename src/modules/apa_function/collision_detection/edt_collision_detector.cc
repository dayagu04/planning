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

void EDTCollisionDetector::PreProcess() {
  AddObsToOGM();
  CalcObsDistArray();
}

void EDTCollisionDetector::PreProcess(
    const geometry_lib::RectangleBound &ogm_bound) {
  GenOccupancyGridMap(ogm_bound);
  PreProcess();
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
  ogm_origin_ = ogm_origin;
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
    const Eigen::Vector2d &pt) {
  return OGMIndex(int(pt.x() * resolution_inv_), int(pt.y() * resolution_inv_));
}

const OGMIndex EDTCollisionDetector::GetIndexFromSlotPose(
    const Eigen::Vector2d &pt) {
  return GetIndexFromOGMPose(Eigen::Vector2d(pt.x() - ogm_bound_.min_x + 1e-6,
                                             pt.y() - ogm_bound_.min_y + 1e-6));
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

  OGMIndex index;
  // add pt cloud
  bool(*obs_ogm)[edt_ogm_grid_y_max] = nullptr;
  for (const auto &obs_pair : obs_map) {
    const ApaObstacle obs = obs_pair.second;
    switch (obs.GetObsHeightType()) {
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
      index = GetIndexFromSlotPose(pt);
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

  if (apa_param.GetParam().enable_multi_height_col_det) {
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
    if (apa_param.GetParam().enable_multi_height_col_det) {
      car_without_mirror_data = car_without_mirror_edt_matrix.ptr<float>(row);
      car_chassis_data = car_chassis_edt_matrix.ptr<float>(row);
    }

    for (int col = 0; col < max_col_num; ++col) {
      car_with_mirror_ogm_obs_data_.dist[row][col] =
          car_with_mirror_data[col] * float(resolution_);
      if (apa_param.GetParam().enable_multi_height_col_det) {
        car_without_mirror_ogm_obs_data_.dist[row][col] =
            car_without_mirror_data[col] * float(resolution_);
        car_chassis_ogm_obs_data_.dist[row][col] =
            car_chassis_data[col] * float(resolution_);
      }
    }
  }

#if write_debug_file
  cv::flip(car_with_mirror_map_matrix, car_with_mirror_map_matrix, 0);
  cv::flip(car_with_mirror_map_matrix, car_with_mirror_map_matrix, 1);
  cv::flip(car_with_mirror_edt_matrix, car_with_mirror_edt_matrix, 0);
  cv::flip(car_with_mirror_edt_matrix, car_with_mirror_edt_matrix, 1);
  cv::imwrite("/asw/planning/glog/car_with_mirror_ogm.png",
              car_with_mirror_map_matrix);
  cv::imwrite("/asw/planning/glog/car_with_mirror_edt.png",
              car_with_mirror_edt_matrix);
  if (apa_param.GetParam().enable_multi_height_col_det) {
    cv::flip(car_without_mirror_map_matrix, car_without_mirror_map_matrix, 0);
    cv::flip(car_without_mirror_map_matrix, car_without_mirror_map_matrix, 1);
    cv::flip(car_without_mirror_edt_matrix, car_without_mirror_edt_matrix, 0);
    cv::flip(car_without_mirror_edt_matrix, car_without_mirror_edt_matrix, 1);
    cv::imwrite("/asw/planning/glog/car_without_mirror_ogm.png",
                car_without_mirror_map_matrix);
    cv::imwrite("/asw/planning/glog/car_without_mirror_edt.png",
                car_without_mirror_edt_matrix);
    cv::flip(car_chassis_map_matrix, car_chassis_map_matrix, 0);
    cv::flip(car_chassis_map_matrix, car_chassis_map_matrix, 1);
    cv::flip(car_chassis_edt_matrix, car_chassis_edt_matrix, 0);
    cv::flip(car_chassis_edt_matrix, car_chassis_edt_matrix, 1);
    cv::imwrite("/asw/planning/glog/car_chassis_ogm.png",
                car_chassis_map_matrix);
    cv::imwrite("/asw/planning/glog/car_chassis_edt.png",
                car_chassis_edt_matrix);
  }
#endif
}

const double EDTCollisionDetector::GetObsDistByIndex(
    const OGMIndex &id, const ApaObsHeightType &height_type) {
  if (IsIndexValid(id)) {
    switch (height_type) {
      case ApaObsHeightType::RUN_OVER:
        return 26.8;
      case ApaObsHeightType::LOW:
        return double(car_chassis_ogm_obs_data_.dist[id.x][id.y]);
      case ApaObsHeightType::MID:
        return double(car_without_mirror_ogm_obs_data_.dist[id.x][id.y]);
      default:
        return double(car_with_mirror_ogm_obs_data_.dist[id.x][id.y]);
    }
  }

  return 0.0;
}

void EDTCollisionDetector::UpdateSafeBuffer(const double body_lat_buffer,
                                            const double lon_buffer,
                                            const double max_circle_buffer,
                                            const bool special_process_mirror,
                                            const double mirror_lat_buffer) {
  lon_buffer_ = lon_buffer;
  const float real_mirror_lat_buffer =
      special_process_mirror ? mirror_lat_buffer : body_lat_buffer;

  if (!need_update_buffer_ &&
      std::fabs(body_lat_buffer_ - body_lat_buffer) < 0.001 &&
      std::fabs(max_circle_buffer_ - max_circle_buffer) < 0.001 &&
      std::fabs(mirror_lat_buffer_ - real_mirror_lat_buffer) < 0.001) {
    return;
  }

  need_update_buffer_ = false;
  body_lat_buffer_ = body_lat_buffer;
  mirror_lat_buffer_ = real_mirror_lat_buffer;
  max_circle_buffer_ = max_circle_buffer;

  UpdateCarWithMirrorSafeBuffer();
  if (apa_param.GetParam().enable_multi_height_col_det) {
    UpdateCarWithOutMirrorSafeBuffer();
    UpdateCarChassisSafeBuffer();
  }
}

void EDTCollisionDetector::UpdateCarWithMirrorSafeBuffer() {
  car_with_mirror_circles_list_buffer_.count =
      car_with_mirror_circles_list_.count;

  car_with_mirror_circles_list_buffer_.max_circle.center_local =
      car_with_mirror_circles_list_.max_circle.center_local;

  car_with_mirror_circles_list_buffer_.max_circle.radius =
      car_with_mirror_circles_list_.max_circle.radius + max_circle_buffer_;

  car_with_mirror_circles_list_buffer_.height_type = ApaObsHeightType::HIGH;

  CarFootPrintCircle *circles = car_with_mirror_circles_list_buffer_.circles;
  CarFootPrintCircle *origin_circles = car_with_mirror_circles_list_.circles;

  for (size_t i = 0; i < car_with_mirror_circles_list_buffer_.count; ++i) {
    circles[i].center_local = origin_circles[i].center_local;
    circles[i].radius = origin_circles[i].radius;
    if (i == 0 || i == 4) {
      // left front circle, left rear circle, move toward left
      circles[i].center_local.y() += body_lat_buffer_;
    } else if (i == 1 || i == 3) {
      // right front circle, right rear circle, move toward right
      circles[i].center_local.y() -= body_lat_buffer_;
    } else if (i == 2 || i == 5) {
      // left and right mirror, increase radius
      circles[i].radius += mirror_lat_buffer_;
    } else if (i == 6) {
      // front circle, increase radius
      circles[i].radius += body_lat_buffer_;
      // move toward down, still tangent to the front lane of the car
      circles[i].center_local.x() -= body_lat_buffer_;
    } else if (i == 9) {
      // rear circle, increase radius
      circles[i].radius += body_lat_buffer_;
      // move toward up, still tangent to the rear lane of the car
      circles[i].center_local.x() += body_lat_buffer_;
    } else {
      // other circle in car
      // generally it willn't exceed front and rear lane of the car
      circles[i].radius += body_lat_buffer_;
    }
  }
}

void EDTCollisionDetector::UpdateCarWithOutMirrorSafeBuffer() {
  car_without_mirror_circles_list_with_buffer_.count =
      car_without_mirror_circles_list_.count;

  car_without_mirror_circles_list_with_buffer_.max_circle.center_local =
      car_without_mirror_circles_list_.max_circle.center_local;

  car_without_mirror_circles_list_with_buffer_.max_circle.radius =
      car_without_mirror_circles_list_.max_circle.radius + max_circle_buffer_;

  car_without_mirror_circles_list_with_buffer_.height_type =
      ApaObsHeightType::MID;

  CarFootPrintCircle *circles = car_with_mirror_circles_list_buffer_.circles;
  CarFootPrintCircle *origin_circles = car_with_mirror_circles_list_.circles;

  for (size_t i = 0; i < car_without_mirror_circles_list_with_buffer_.count;
       ++i) {
    circles[i].center_local = origin_circles[i].center_local;
    circles[i].radius = origin_circles[i].radius;
    if (i == 0 || i == 3) {
      // left front circle, left rear circle, move toward left
      circles[i].center_local.y() += body_lat_buffer_;
    } else if (i == 1 || i == 2) {
      // right front circle, right rear circle, move toward right
      circles[i].center_local.y() -= body_lat_buffer_;
    } else if (i == 4) {
      // front circle, increase radius
      circles[i].radius += body_lat_buffer_;
      // move toward down, still tangent to the front lane of the car
      circles[i].center_local.x() -= body_lat_buffer_;
    } else if (i == 7) {
      // rear circle, increase radius
      circles[i].radius += body_lat_buffer_;
      // move toward up, still tangent to the rear lane of the car
      circles[i].center_local.x() += body_lat_buffer_;
    } else {
      // other circle in car
      // generally it willn't exceed front and rear lane of the car
      circles[i].radius += body_lat_buffer_;
    }
  }
}

void EDTCollisionDetector::UpdateCarChassisSafeBuffer() {
  car_chassis_circles_list_with_buffer_.count = car_chassis_circles_list_.count;

  car_chassis_circles_list_with_buffer_.max_circle.center_local =
      car_chassis_circles_list_.max_circle.center_local;

  car_chassis_circles_list_with_buffer_.max_circle.radius =
      car_chassis_circles_list_.max_circle.radius + max_circle_buffer_;

  car_chassis_circles_list_with_buffer_.height_type = ApaObsHeightType::LOW;

  CarFootPrintCircle *circles = car_with_mirror_circles_list_buffer_.circles;
  CarFootPrintCircle *origin_circles = car_with_mirror_circles_list_.circles;

  for (size_t i = 0; i < car_chassis_circles_list_with_buffer_.count; ++i) {
    circles[i].center_local = origin_circles[i].center_local;
    circles[i].radius = origin_circles[i].radius;
    if (i == 0 || i == 4) {
      // left front circle, left rear circle, move toward left
      circles[i].center_local.y() += body_lat_buffer_;
    } else if (i == 1 || i == 3) {
      // right front circle, right rear circle, move toward right
      circles[i].center_local.y() -= body_lat_buffer_;
    } else if (i == 2 || i == 5) {
      // left and right mirror, increase radius
      circles[i].radius += mirror_lat_buffer_;
    } else if (i == 6) {
      // front circle, increase radius
      circles[i].radius += body_lat_buffer_;
      // move toward down, still tangent to the front lane of the car
      circles[i].center_local.x() -= body_lat_buffer_;
    } else if (i == 9) {
      // rear circle, increase radius
      circles[i].radius += body_lat_buffer_;
      // move toward up, still tangent to the rear lane of the car
      circles[i].center_local.x() += body_lat_buffer_;
    } else {
      // other circle in car
      // generally it willn't exceed front and rear lane of the car
      circles[i].radius += body_lat_buffer_;
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
    double *min_dist, int *circle_id, const double safe_dist) {
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
    *min_dist = 0.0;
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
  *min_dist = 26.8;
  for (uint8_t i = 0; i < car_circle_list->count; ++i) {
    circle = &car_circle_list->circles[i];
    center_index = GetIndexFromSlotPose(circle->center_global);
    if (!IsIndexValid(center_index)) {
      *min_dist = 0.0;
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

const ColResult EDTCollisionDetector::Update(
    const geometry_lib::PathSegment &path_seg, const double body_lat_buffer,
    const double lon_buffer, const bool need_cal_obs_dist,
    const double max_circle_buffer, const bool special_process_mirror,
    const double mirror_lat_buffer) {
  std::vector<geometry_lib::PathPoint> pt_vec;
  geometry_lib::SamplePointSetInPathSeg(pt_vec, path_seg, sample_ds_);
  return Update(pt_vec, body_lat_buffer, lon_buffer, need_cal_obs_dist,
                max_circle_buffer, special_process_mirror, mirror_lat_buffer);
}

const ColResult EDTCollisionDetector::Update(
    const std::vector<geometry_lib::PathPoint> &pt_vec,
    const double body_lat_buffer, const double lon_buffer,
    const bool need_cal_obs_dist, const double max_circle_buffer,
    const bool special_process_mirror, const double mirror_lat_buffer) {
  // The input PathPoint s must be assigned a value
  col_res_.Reset();
  size_t N = pt_vec.size();
  if (obs_manager_ptr_ == nullptr || obs_manager_ptr_->GetObstacles().empty() ||
      N == 0) {
    return col_res_;
  }

  col_res_.remain_car_dist = pt_vec.back().s;
  col_res_.remain_dist = pt_vec.back().s;

  UpdateSafeBuffer(body_lat_buffer, lon_buffer, max_circle_buffer,
                   special_process_mirror, mirror_lat_buffer);

  path_pt_vec_ = pt_vec;
  if (N > 1 && lon_buffer > 0.01) {
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
      if (s >= lon_buffer_) {
        s = lon_buffer_;
      }
      geometry_lib::CalExtendedPointByTwoPoints(start_point, end_point, pt.pos,
                                                s);
      pt.s = col_res_.remain_car_dist + s;
      path_pt_vec_.emplace_back(pt);
    } while (s < lon_buffer_);
  }

  bool col_flag = false;
  double lon_safe_dist = 0.0;
  double obs_dist = 0.0;
  int circle_id = -1;
  geometry_lib::CarSafePos car_safe_pos = geometry_lib::CarSafePos::ALL;
  CarFootPrintCircleList *car_circle_list = nullptr;
  for (const geometry_lib::PathPoint &pt : path_pt_vec_) {
    circle_id = -1;
    obs_dist = 26.8;
    if (!IsPoseInClearZone(pt)) {
      col_flag =
          need_cal_obs_dist
              ? IsCollisionForPoint(pt, &car_with_mirror_circles_list_buffer_,
                                    &obs_dist, &circle_id)
              : IsCollisionForPoint(pt, &car_with_mirror_circles_list_buffer_);

      if (col_flag) {
        break;
      }

      if (apa_param.GetParam().enable_multi_height_col_det) {
        col_flag = need_cal_obs_dist
                       ? IsCollisionForPoint(
                             pt, &car_without_mirror_circles_list_with_buffer_,
                             &obs_dist, &circle_id)
                       : IsCollisionForPoint(
                             pt, &car_without_mirror_circles_list_with_buffer_);

        if (col_flag) {
          break;
        }

        col_flag = need_cal_obs_dist
                       ? IsCollisionForPoint(
                             pt, &car_chassis_circles_list_with_buffer_,
                             &obs_dist, &circle_id)
                       : IsCollisionForPoint(
                             pt, &car_chassis_circles_list_with_buffer_);

        if (col_flag) {
          break;
        }
      }
    }

    if (need_cal_obs_dist) {
      if (circle_id == -1) {
        car_safe_pos = geometry_lib::CarSafePos::ALL;
      } else if (circle_id == 0 || circle_id == 1 || circle_id == 2 ||
                 circle_id == 5 || circle_id == 6) {
        car_safe_pos = geometry_lib::CarSafePos::CAR_REAR;
      } else {
        car_safe_pos = geometry_lib::CarSafePos::CAR_FRONT;
      }

      col_res_.pt_obs_dist_info_vec.emplace_back(geometry_lib::Pt2ObsDistInfo(
          std::pair<double, geometry_lib::PathPoint>{obs_dist + body_lat_buffer,
                                                     pt},
          circle_id, car_safe_pos));
    }

    lon_safe_dist = pt.s;
  }

  col_res_.col_flag = col_flag;
  col_res_.remain_dist = lon_safe_dist - lon_buffer;
  const int safe_pt_number =
      static_cast<int>(col_res_.remain_dist / sample_ds_) + 1;
  if (col_res_.pt_obs_dist_info_vec.size() > safe_pt_number) {
    col_res_.pt_obs_dist_info_vec.resize(safe_pt_number);
  }
  double min_obs_dist = 26.8;
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

}  // namespace apa_planner
}  // namespace planning