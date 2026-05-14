#include "environmental_model.h"
#include <iostream>
#include "debug_info_log.h"
#include "ego_planning_config.h"
#include "ego_state_manager.h"
#include "frenet_ego_state.h"
#include "history_obstacle_manager.h"
#include "lateral_obstacle.h"
#include "obstacle_manager.h"
#include "reference_path_manager.h"
#include "traffic_light_decision_manager.h"
#include "trajectory/trajectory_stitcher.h"
#include "virtual_lane_manager.h"

namespace {
constexpr uint64_t kStaticMapOvertimeThreshold = 20000000;  // 20s
}  // namespace

namespace planning {
EnvironmentalModel::EnvironmentalModel() {}

// EnvironmentalModel::~EnvironmentalModel() {}

bool EnvironmentalModel::Init(common::SceneType scene_type) {
  point_dx_filter_.SetWindowSize(10);
  point_dy_filter_.SetWindowSize(10);
  point_dtheta_filter_.SetWindowSize(10);
  init_point_diff_mat_.resize(3,1);
  return true;
}

bool EnvironmentalModel::Update() { return true; }

void EnvironmentalModel::UpdateMembers() {}

// compensation during divergence
// bool EnvironmentalModel::UpdateInitPoint() {
//   init_point_diff_mat_.setZero(3,1);
//   const auto &reference_path =
//       reference_path_manager_->get_reference_path_by_current_lane();
//   if (last_refline_ == nullptr ||
//       reference_path == nullptr) {
//     last_refline_ = reference_path;
//     return false;
//   }
//   bool is_need_translation = false;
//   bool is_need_rotation = false;
//   double diff_l = 0.0;
//   double diff_heading = 0.0;
//   double mean_point_dx = 0.0;
//   double mean_point_dy = 0.0;
//   double mean_point_dtheta = 0.0;
//   // calculate pos and heading bias
//   auto &frenet_ego_state =
//       reference_path->mutable_frenet_ego_state();
//   auto &init_point =
//       frenet_ego_state.mutable_planning_init_point();
//   const auto& refline_frenet_coord =
//       reference_path->get_frenet_coord();
//   const auto& last_refline_frenet_coord =
//       last_refline_->get_frenet_coord();
//   if (last_refline_frenet_coord != nullptr &&
//       refline_frenet_coord != nullptr) {
//     const auto& init_points = ego_state_manager_->GetAdjacentInitPoint();
//     double mean_count = 0;
//     for (size_t i = 0; i < init_points.size(); ++i) {
//       Point2D last_frenet_init_tp;
//       Point2D frenet_init_tp;
//       if (last_refline_frenet_coord->XYToSL(
//           Point2D(init_points[i](0,0), init_points[i](1,0)), last_frenet_init_tp) &&
//           refline_frenet_coord->XYToSL(
//           Point2D(init_points[i](0,0), init_points[i](1,0)), frenet_init_tp)) {
//         diff_l +=
//             (std::fabs(frenet_init_tp.y) - std::fabs(last_frenet_init_tp.y));
//         double last_rel_heading =
//             planning_math::NormalizeAngle(
//                 init_points[i](2,0) -
//                 last_refline_frenet_coord->GetPathCurveHeading(last_frenet_init_tp.x));
//         double rel_heading =
//             planning_math::NormalizeAngle(
//                 init_points[i](2,0) -
//                 refline_frenet_coord->GetPathCurveHeading(frenet_init_tp.x));
//         diff_heading += (rel_heading - last_rel_heading);
//         mean_count += 1;
//       }
//     }
//     diff_l /= std::max(mean_count, 1.0);
//     diff_heading /= std::max(mean_count, 1.0);
//     // correct xy
//     if (diff_l > 1e-6) {
//       if (init_point.frenet_state.r < -1e-6) {
//         diff_l *= -1.0;
//       }
//       Point2D real_cart_init_tp;
//       if (refline_frenet_coord->SLToXY(
//           Point2D(init_point.frenet_state.s, (init_point.frenet_state.r - diff_l)), real_cart_init_tp)) {
//         is_need_translation = true;
//         double diff_pos_x = real_cart_init_tp.x - init_point.x;
//         double diff_pos_y = real_cart_init_tp.y - init_point.y;
//         if (std::fabs(diff_pos_x) <= 0.15 &&
//             std::fabs(diff_pos_y) <= 0.15 &&
//             (std::fabs(diff_pos_x) >= 0.002 ||
//              std::fabs(diff_pos_y) >= 0.002)) {
//           mean_point_dx = point_dx_filter_.Update(std::fabs(diff_pos_x));
//           mean_point_dy = point_dy_filter_.Update(std::fabs(diff_pos_y));
//         } else if (std::fabs(diff_pos_x) > 0.15 ||
//                    std::fabs(diff_pos_y) > 0.15) {
//           mean_point_dx = point_dx_filter_.GetMeanValue();
//           mean_point_dy = point_dy_filter_.GetMeanValue();
//         } else {
//           mean_point_dx = 0.0;
//           mean_point_dy = 0.0;
//         }
//         if (diff_pos_x < -1e-6) {
//           mean_point_dx *= -1.0;
//         }
//         if (diff_pos_y < -1e-6) {
//           mean_point_dy *= -1.0;
//         }
//       }
//       if (point_dx_filter_.GetValuesSize() < 10 ||
//           point_dy_filter_.GetValuesSize() < 10) {
//         is_need_translation = false;
//         mean_point_dx = 0.0;
//         mean_point_dy = 0.0;
//       }
//     }
//     // correct heading
//     if (init_point.frenet_state.r * (diff_heading) < -1e-6) {
//       is_need_rotation = true;
//       if (std::fabs(diff_heading) * 57.3 <= 1.5 &&
//           std::fabs(diff_heading) * 57.3 >= 0.002) {
//         mean_point_dtheta = point_dtheta_filter_.Update(std::fabs(diff_heading));
//       } else if (std::fabs(diff_heading) * 57.3 > 1.5) {
//         mean_point_dtheta = point_dtheta_filter_.GetMeanValue();
//       } else {
//         mean_point_dtheta = 0.0;
//       }
//       if (diff_heading < -1e-6) {
//         mean_point_dtheta *= -1.0;
//       }
//       if (point_dtheta_filter_.GetValuesSize() < 10) {
//         is_need_rotation = false;
//         mean_point_dtheta = 0.0;
//       }
//     }
//   }
//   // if reset filter
//   // todu: consider curvature
//   // not in function
//   const auto &function_mode =
//       function_info_.function_mode();
//   bool is_in_function =
//       function_mode == common::DrivingFunctionInfo::SCC ||
//       function_mode == common::DrivingFunctionInfo::NOA;
//   if (!is_in_function) {
//     mean_point_dx = 0.0;
//     mean_point_dy = 0.0;
//     mean_point_dtheta = 0.0;
//     point_dx_filter_.Reset();
//     point_dy_filter_.Reset();
//     point_dtheta_filter_.Reset();
//   }
//   //
//   // update init point
//   if (is_need_translation || is_need_rotation) {
//     auto &planning_init_point =
//         ego_state_manager_->mutable_planning_init_point();
//     planning_init_point.x += mean_point_dx;
//     planning_init_point.y += mean_point_dy;
//     planning_init_point.heading_angle -= mean_point_dtheta;
//     planning_init_point.lat_init_state.set_x(planning_init_point.x);
//     planning_init_point.lat_init_state.set_y(planning_init_point.y);
//     planning_init_point.lat_init_state.set_theta(planning_init_point.heading_angle);
//     if (refline_frenet_coord != nullptr) {
//       frenet_ego_state.update(
//           refline_frenet_coord, *ego_state_manager_);
//     } else {
//       init_point.x = planning_init_point.x;
//       init_point.y = planning_init_point.y;
//       init_point.heading_angle = planning_init_point.heading_angle;
//       init_point.frenet_state.r += diff_l;
//       init_point.lat_init_state.set_x(planning_init_point.x);
//       init_point.lat_init_state.set_y(planning_init_point.y);
//       init_point.lat_init_state.set_theta(planning_init_point.heading_angle);
//     }
//   }
//   // update matrix
//   init_point_diff_mat_ << mean_point_dx,
//                           mean_point_dy,
//                           mean_point_dtheta;
//   // save reference path
//   last_refline_ = reference_path;
//   return true;
// }

// compensation for any deviation
bool EnvironmentalModel::UpdateInitPoint(
    bool is_in_cur_lane) {
  init_point_diff_mat_.setZero(3,1);
  const auto &reference_path =
      reference_path_manager_->get_reference_path_by_current_lane();
  if (last_refline_ == nullptr ||
      reference_path == nullptr ||
      !is_in_cur_lane) {
    last_refline_ = reference_path;
    return false;
  }
  bool is_need_translation = false;
  bool is_need_rotation = false;
  double diff_l = 0.0;
  double diff_heading = 0.0;
  double mean_point_dx = 0.0;
  double mean_point_dy = 0.0;
  double mean_point_dtheta = 0.0;
  // calculate pos and heading bias
  auto &frenet_ego_state =
      reference_path->mutable_frenet_ego_state();
  auto &init_point =
      frenet_ego_state.mutable_planning_init_point();
  const auto& refline_frenet_coord =
      reference_path->get_frenet_coord();
  const auto& last_refline_frenet_coord =
      last_refline_->get_frenet_coord();
  if (last_refline_frenet_coord != nullptr &&
      refline_frenet_coord != nullptr) {
    const auto& init_points = ego_state_manager_->GetAdjacentInitPoint();
    double mean_count = 0;
    for (size_t i = 0; i < init_points.size(); ++i) {
      Point2D last_frenet_init_tp;
      Point2D frenet_init_tp;
      if (last_refline_frenet_coord->XYToSL(
          Point2D(init_points[i](0,0), init_points[i](1,0)), last_frenet_init_tp) &&
          refline_frenet_coord->XYToSL(
          Point2D(init_points[i](0,0), init_points[i](1,0)), frenet_init_tp)) {
        diff_l +=
            (frenet_init_tp.y - last_frenet_init_tp.y);
        double last_rel_heading =
            planning_math::NormalizeAngle(
                init_points[i](2,0) -
                last_refline_frenet_coord->GetPathCurveHeading(last_frenet_init_tp.x));
        double rel_heading =
            planning_math::NormalizeAngle(
                init_points[i](2,0) -
                refline_frenet_coord->GetPathCurveHeading(frenet_init_tp.x));
        diff_heading += (rel_heading - last_rel_heading);
        mean_count += 1;
      }
    }
    diff_l /= std::max(mean_count, 1.0);
    diff_heading /= std::max(mean_count, 1.0);
    // correct xy
    if (std::fabs(diff_l) > 1e-3) {
      Point2D real_cart_init_tp;
      if (refline_frenet_coord->SLToXY(
          Point2D(init_point.frenet_state.s, (init_point.frenet_state.r - diff_l)), real_cart_init_tp)) {
        is_need_translation = true;
        double diff_pos_x = real_cart_init_tp.x - init_point.x;
        double diff_pos_y = real_cart_init_tp.y - init_point.y;
        if (std::fabs(diff_pos_x) <= 0.15 &&
            std::fabs(diff_pos_y) <= 0.15 &&
            (std::fabs(diff_pos_x) >= 0.005 ||
             std::fabs(diff_pos_y) >= 0.005)) {
          mean_point_dx = point_dx_filter_.Update(std::fabs(diff_pos_x));
          mean_point_dy = point_dy_filter_.Update(std::fabs(diff_pos_y));
        } else if (std::fabs(diff_pos_x) > 0.15 ||
                   std::fabs(diff_pos_y) > 0.15) {
          mean_point_dx = point_dx_filter_.GetMeanValue();
          mean_point_dy = point_dy_filter_.GetMeanValue();
        } else {
          mean_point_dx = 0.0;
          mean_point_dy = 0.0;
        }
        if (diff_pos_x < -1e-6) {
          mean_point_dx *= -1.0;
        }
        if (diff_pos_y < -1e-6) {
          mean_point_dy *= -1.0;
        }
      }
      if (point_dx_filter_.GetValuesSize() < 10 ||
          point_dy_filter_.GetValuesSize() < 10) {
        is_need_translation = false;
        mean_point_dx = 0.0;
        mean_point_dy = 0.0;
      }
    }
    // correct heading
    if (std::fabs(diff_heading) * 57.3 > 1e-3) {
      is_need_rotation = true;
      if (std::fabs(diff_heading) * 57.3 <= 1.5 &&
          std::fabs(diff_heading) * 57.3 >= 0.005) {
        mean_point_dtheta = point_dtheta_filter_.Update(std::fabs(diff_heading));
      } else if (std::fabs(diff_heading) * 57.3 > 1.5) {
        mean_point_dtheta = point_dtheta_filter_.GetMeanValue();
      } else {
        mean_point_dtheta = 0.0;
      }
      if (diff_heading < -1e-6) {
        mean_point_dtheta *= -1.0;
      }
      if (point_dtheta_filter_.GetValuesSize() < 10) {
        is_need_rotation = false;
        mean_point_dtheta = 0.0;
      }
    }
  }
  // if reset filter
  // todu: consider curvature
  // not in function
  const auto &function_mode =
      function_info_.function_mode();
  bool is_in_function =
      function_mode == common::DrivingFunctionInfo::SCC ||
      function_mode == common::DrivingFunctionInfo::NOA;
  if (!is_in_function) {
    mean_point_dx = 0.0;
    mean_point_dy = 0.0;
    mean_point_dtheta = 0.0;
    point_dx_filter_.Reset();
    point_dy_filter_.Reset();
    point_dtheta_filter_.Reset();
  }
  //
  // update init point
  if (is_need_translation || is_need_rotation) {
    auto &planning_init_point =
        ego_state_manager_->mutable_planning_init_point();
    planning_init_point.x += mean_point_dx;
    planning_init_point.y += mean_point_dy;
    planning_init_point.heading_angle -= mean_point_dtheta;
    planning_init_point.lat_init_state.set_x(planning_init_point.x);
    planning_init_point.lat_init_state.set_y(planning_init_point.y);
    planning_init_point.lat_init_state.set_theta(planning_init_point.heading_angle);
    if (refline_frenet_coord != nullptr) {
      frenet_ego_state.update(
          refline_frenet_coord, *ego_state_manager_);
    } else {
      init_point.x = planning_init_point.x;
      init_point.y = planning_init_point.y;
      init_point.heading_angle = planning_init_point.heading_angle;
      init_point.frenet_state.r += diff_l;
      init_point.lat_init_state.set_x(planning_init_point.x);
      init_point.lat_init_state.set_y(planning_init_point.y);
      init_point.lat_init_state.set_theta(planning_init_point.heading_angle);
    }
  }
  // update matrix
  init_point_diff_mat_ << mean_point_dx,
                          mean_point_dy,
                          mean_point_dtheta;
  // save reference path
  last_refline_ = reference_path;
  return true;
}

}  // namespace planning