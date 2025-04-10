#include "gjk_collision_detector.h"

#include <cstddef>
#include <vector>

#include "apa_obstacle.h"
#include "apa_param_config.h"
#include "collision_detection/base_collision_detector.h"
#include "geometry_math.h"
#include "log_glog.h"
#include "polygon_base.h"

namespace planning {
namespace apa_planner {

const ColResult GJKCollisionDetector::Update(
    const geometry_lib::PathSegment& path_seg, const double lat_buffer,
    const double lon_buffer, const GJKColDetRequest gjk_col_det_request) {
  std::vector<geometry_lib::PathPoint> pt_vec;
  geometry_lib::SamplePointSetInPathSeg(pt_vec, path_seg, sample_ds_);
  return Update(pt_vec, lat_buffer, lon_buffer, gjk_col_det_request);
}

const ColResult GJKCollisionDetector::Update(
    const std::vector<geometry_lib::PathPoint>& pt_vec, const double lat_buffer,
    const double lon_buffer, const GJKColDetRequest gjk_col_det_request) {
  // 输入PathPoint的s必须赋值
  col_res_.Reset();
  size_t N = pt_vec.size();
  if (obs_manager_ == nullptr || obs_manager_->GetObstacles().empty() ||
      N == 0) {
    return col_res_;
  }

  col_res_.remain_car_dist = pt_vec.back().s;
  col_res_.remain_dist = pt_vec.back().s;

  UpdateSafeBuffer(lat_buffer, lon_buffer);
  GenCarPolygon();

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
  const std::unordered_map<size_t, ApaObstacle>& obs_map =
      obs_manager_->GetObstacles();

  for (const geometry_lib::PathPoint& pt : path_pt_vec_) {
    TransformPolygonFootPrintLocalToGlobal(pt);

    for (const auto& obs_pair : obs_map) {
      const ApaObstacle& obs = obs_pair.second;
      std::vector<Polygon2D> polygon_vec{
          polygon_foot_print_global_.max_polygon};

      switch (obs.GetObsHeightType()) {
        case ApaObsHeightType::RUN_OVER:
          continue;
        case ApaObsHeightType::LOW:
          polygon_vec.emplace_back(polygon_foot_print_global_.chassis);
          break;
        case ApaObsHeightType::MID:
          polygon_vec.emplace_back(polygon_foot_print_global_.body);
          break;
        default:
          if (gjk_col_det_request.car_body_type == CarBodyType::NORMAL) {
            polygon_vec.emplace_back(polygon_foot_print_global_.body);
            polygon_vec.emplace_back(polygon_foot_print_global_.mirror_left);
            polygon_vec.emplace_back(polygon_foot_print_global_.mirror_right);
          } else if (gjk_col_det_request.car_body_type ==
                     CarBodyType::EXPAND_MIRROR_TO_FRONT) {
            polygon_vec.emplace_back(
                polygon_foot_print_global_
                    .mirror_to_front_overhang_expand_front);
            polygon_vec.emplace_back(
                polygon_foot_print_global_.mirror_to_rear_overhang);
          } else if (gjk_col_det_request.car_body_type ==
                     CarBodyType::EXPAND_MIRROR_TO_END) {
            // todo: should use right polygon for end expansion
            polygon_vec.emplace_back(
                polygon_foot_print_global_
                    .mirror_to_front_overhang_expand_front);
            polygon_vec.emplace_back(
                polygon_foot_print_global_.mirror_to_rear_overhang);
          }

          break;
      }

      if (!CheckObsMovementTypeFeasible(obs.GetObsMovementType(),
                                        gjk_col_det_request.movement_type)) {
        continue;
      }

      for (size_t i = 0; i < polygon_vec.size(); ++i) {
        col_flag = IsPolygonCollision(polygon_vec[i], obs, gjk_col_det_request);
        if (i == 0) {
          if (!col_flag) {
            // max polygan no col, safe, quit
            break;
          } else {
            if (gjk_col_det_request.only_check_max_car_polygon) {
              // if only check max polygon, consider is col
              break;
            }
            // max polygan col, should use other polygon to col det
            continue;
          }
        } else {
          // other polygon, if col, consider col
          if (col_flag) {
            break;
          }
        }
      }

      if (col_flag) {
        break;
      }
    }

    if (col_flag) {
      break;
    }
    lon_safe_dist = pt.s;
  }

  col_res_.col_flag = col_flag;
  col_res_.remain_dist = lon_safe_dist - lon_buffer;

  if (gjk_col_det_request.movement_type == ApaObsMovementType::MOTION) {
    col_res_.remain_dist_dynamic = col_res_.remain_dist;
  }

  if (gjk_col_det_request.movement_type == ApaObsMovementType::STATIC) {
    col_res_.remain_dist_static = col_res_.remain_dist;
  }

  return col_res_;
}

const bool GJKCollisionDetector::IsPolygonCollision(
    const Polygon2D& polygon, const GJKColDetRequest gjk_col_det_request) {
  const std::unordered_map<size_t, ApaObstacle>& obs_map =
      obs_manager_->GetObstacles();

  for (const auto& obs_pair : obs_map) {
    const ApaObstacle& obs = obs_pair.second;
    if (IsPolygonCollision(polygon, obs, gjk_col_det_request)) {
      return true;
    }
  }

  return false;
}

const bool GJKCollisionDetector::IsPolygonCollision(
    const Polygon2D& polygon, const ApaObstacle& obs,
    const GJKColDetRequest gjk_col_det_request) {
  bool col_flag = false;

  const Polygon2D* obs_polygon = &obs.GetPolygon2DLocal();
  const std::vector<Eigen::Vector2d>* pt_clout = &obs.GetPtClout2dLocal();
  if (!gjk_col_det_request.use_obs_base_slot) {
    obs_polygon = &obs.GetPolygon2DGlobal();
    pt_clout = &obs.GetPtClout2dGlobal();
  }

  gjk_interface_.PolygonCollisionByCircleCheck(&col_flag, obs_polygon, &polygon,
                                               0.01);

  if (col_flag) {
    for (const Eigen::Vector2d& pt : *pt_clout) {
      gjk_interface_.PolygonPointCollisionDetect(
          &polygon, Eigen::Vector2f(pt[0], pt[1]), &col_flag);
      if (col_flag) {
        return true;
      }
    }
  }

  return false;
}

const bool GJKCollisionDetector::IsObsInCar(const geometry_lib::PathPoint& pose,
                                            const double lat_buffer,
                                            const Eigen::Vector2d& obs) {
  UpdateSafeBuffer(lat_buffer, 0.0);
  GenCarPolygon();
  TransformPolygonFootPrintLocalToGlobal(pose);
  bool col_flag = false;
  gjk_interface_.PolygonPointCollisionDetect(
      &polygon_foot_print_global_.max_polygon,
      Eigen::Vector2f(obs.x(), obs.y()), &col_flag);
  return col_flag;
}

void GJKCollisionDetector::TransformPolygonFootPrintLocalToGlobal(
    const geometry_lib::PathPoint& pt) {
  Transform2d tf(Pose2D(pt.pos.x(), pt.pos.y(), pt.heading));

  ULFLocalPolygonToGlobal(&polygon_foot_print_global_.max_polygon,
                          &polygon_foot_print_local_.max_polygon, tf);

  ULFLocalPolygonToGlobal(&polygon_foot_print_global_.body,
                          &polygon_foot_print_local_.body, tf);

  ULFLocalPolygonToGlobal(&polygon_foot_print_global_.mirror_left,
                          &polygon_foot_print_local_.mirror_left, tf);

  ULFLocalPolygonToGlobal(&polygon_foot_print_global_.mirror_right,
                          &polygon_foot_print_local_.mirror_right, tf);

  ULFLocalPolygonToGlobal(&polygon_foot_print_global_.chassis,
                          &polygon_foot_print_local_.chassis, tf);

  ULFLocalPolygonToGlobal(
      &polygon_foot_print_global_.mirror_to_front_overhang_expand_front,
      &polygon_foot_print_local_.mirror_to_front_overhang_expand_front, tf);

  ULFLocalPolygonToGlobal(&polygon_foot_print_global_.mirror_to_rear_overhang,
                          &polygon_foot_print_local_.mirror_to_rear_overhang,
                          tf);
}

void GJKCollisionDetector::GenCarPolygon() {
  polygon_foot_print_local_.chassis.FillTangentCircleParams(
      chassis_vertex_with_buffer_);

  polygon_foot_print_local_.body.FillTangentCircleParams(
      car_without_mirror_polygon_vertex_with_buffer_);

  polygon_foot_print_local_.mirror_left.FillTangentCircleParams(
      left_mirror_rectangle_vertex_with_buffer_);

  polygon_foot_print_local_.mirror_right.FillTangentCircleParams(
      right_mirror_rectangle_vertex_with_buffer_);

  polygon_foot_print_local_.max_polygon.FillTangentCircleParams(
      car_with_mirror_rectangle_vertex_with_buffer_);

  polygon_foot_print_local_.mirror_to_front_overhang_expand_front
      .FillTangentCircleParams(
          mirror_to_front_overhanging_rectangle_vertex_expand_front_with_buffer_);

  polygon_foot_print_local_.mirror_to_rear_overhang.FillTangentCircleParams(
      mirror_to_rear_overhanging_polygon_vertex_with_buffer_);
}

void GJKCollisionDetector::Reset() {}

}  // namespace apa_planner
}  // namespace planning