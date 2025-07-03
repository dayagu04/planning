// #include "../include/lateral_motion_simulator_pure_pursuit_model.h"

// #include <Eigen/src/Core/Matrix.h>

// #include <cstddef>
// #include <limits>
// #include <vector>

// #include "basic_pure_pursuit_model.h"
// #include "config/basic_type.h"
// #include "reference_path.h"
// #include "spline.h"
// #include "spline_projection.h"

// namespace planning {
// namespace {
// constexpr size_t scope_goal_points_num = 20;
// }
// namespace simulator {
// ErrorType LatMotionSimulatorPurePursuitModel::CalculateAlpha(
//     const std::shared_ptr<ReferencePath>& reference_path,
//     bool need_calculate_init_projection) {
//   const auto& ref_points = reference_path->get_points();
//   const auto& frenet_ego_state = reference_path->get_frenet_ego_state();
//   Eigen::Vector2d ego_init_pos(init_state_.x, init_state_.y);
//   size_t pre_goal_point_index = 0;
//   // cal projection point on reference path
//   if (need_calculate_init_projection) {
//     pnc::mathlib::spline ref_x_s_spline, ref_y_s_spline, ref_theta_s_spline;
//     std::vector<double> s_vec(ref_points.size());
//     std::vector<double> x_vec(ref_points.size());
//     std::vector<double> y_vec(ref_points.size());
//     std::vector<double> theta_vec(ref_points.size());
//     for (size_t i = 0; i < ref_points.size(); ++i) {
//       s_vec[i] = ref_points[i].path_point.s();
//       x_vec[i] = ref_points[i].path_point.x();
//       y_vec[i] = ref_points[i].path_point.y();
//       theta_vec[i] = ref_points[i].path_point.theta();
//     }
//     ref_x_s_spline.set_points(s_vec, x_vec);
//     ref_y_s_spline.set_points(s_vec, y_vec);
//     ref_theta_s_spline.set_points(s_vec, theta_vec);
//     pnc::spline::Projection projection_spline;
//     projection_spline.CalProjectionPoint(ref_x_s_spline, ref_y_s_spline,
//                                          s_vec.front(), s_vec.back(),
//                                          ego_init_pos);
//     const auto projection_result = projection_spline.GetOutput();
//     const auto proj_point = projection_result.point_proj;
//     const double s_proj = projection_result.s_proj;  // s of projection point
//     // find look ahead point
//     size_t init_lookup_index = 0;
//     auto it = std::upper_bound(s_vec.begin(), s_vec.end(), s_proj);
//     if (it != s_vec.begin()) {
//       init_lookup_index = std::distance(s_vec.begin(), it) - 1;
//     } else {
//       init_lookup_index = 0;
//     }
//     double distance2_to_proj_point = std::numeric_limits<double>::max();
//     const double ld2 = model_param_.ld * model_param_.ld;
//     size_t pre_goal_point_index = init_lookup_index;
//     for (size_t i = init_lookup_index; i < ref_points.size(); ++i) {
//       distance2_to_proj_point =
//           pow(ref_points[i].path_point.x() - proj_point.x(),
//               ref_points[i].path_point.y() - proj_point.y());
//       if (distance2_to_proj_point > ld2) {
//         pre_goal_point_index = i;
//         break;
//       }
//     }
//     --pre_goal_point_index;
//   }

//   double distance2_to_proj_point = std::numeric_limits<double>::max();
//   const double ld2 = model_param_.ld * model_param_.ld;
//   for (size_t i = 1; i < ref_points.size(); ++i) {
//     distance2_to_proj_point =
//         pow(ref_points[i].path_point.x() - ref_points.front().path_point.x(),
//             ref_points[i].path_point.y() - ref_points.front().path_point.y());
//     if (distance2_to_proj_point > ld2) {
//       pre_goal_point_index = i;
//       break;
//     }
//   }
//   --pre_goal_point_index;
//   const auto front_goal_point = ref_points[pre_goal_point_index + 1].path_point;
//   const auto behind_goal_point = ref_points[pre_goal_point_index].path_point;
//   Eigen::Vector2d front_goal_pnt(front_goal_point.x(), front_goal_point.y());
//   Eigen::Vector2d behind_goal_pnt(behind_goal_point.x(), behind_goal_point.y());
//   double ld_interp = 0.0;
//   Eigen::Vector2d scope_goal_point_candidate;
//   for (size_t i = 1; i <= scope_goal_points_num; ++i) {
//     double t = static_cast<double>(i) / (scope_goal_points_num + 1);
//     scope_goal_point_candidate = (1 - t) * behind_goal_pnt + t * front_goal_pnt;
//     ld_interp = (scope_goal_point_candidate - ego_init_pos).norm();
//     if (ld_interp > model_param_.ld) {
//       goal_poinit_ = scope_goal_point_candidate;
//       break;
//     }
//   }
//   const double init_theta = ref_points.front().path_point.theta();
//   Eigen::Vector2d init_theta_vec(cos(init_theta), sin(init_theta));
//   Eigen::Vector2d ld_vec = (goal_poinit_ - ego_init_pos).normalized();
//   alpha_ = acos(init_theta_vec.dot(ld_vec));

//   double cross_product_ld_to_init_theta_vec =
//       init_theta_vec.x() * ld_vec.y() - init_theta_vec.y() * ld_vec.x();
//   int rotation_direction = (cross_product_ld_to_init_theta_vec >= 0) ? 1 : -1;
//   alpha_ *= rotation_direction;
//   return ErrorType::kSuccess;
// }

// }  // namespace simulator

// }  // namespace planning