#include "apa_predict_path_manager.h"

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>

#include "apa_param_config.h"
#include "apa_utils.h"
#include "debug_info_log.h"
#include "geometry_math.h"
#include "local_view.h"
#include "log_glog.h"
#include "planning_plan_c.h"
#include "spline.h"

namespace planning {
namespace apa_planner {
namespace {

enum class PredictSpliceStatus : uint8_t {
  kInit = 0,
  kInvalidInput = 1,
  kApaNotInProgress = 2,
  kPlanProjectionFail = 3,
  kMpcOnly = 4,
  kSkipByControlOrGear = 5,
  kSpliceProjectionFail = 6,
  kSpliceInsufficientPlanPoints = 7,
  kSplineSplice = 8,
  kSteerAngleFallback = 9,
};

const char* ToString(const PredictSpliceStatus status) {
  switch (status) {
    case PredictSpliceStatus::kInit:
      return "init";
    case PredictSpliceStatus::kInvalidInput:
      return "invalid_input";
    case PredictSpliceStatus::kApaNotInProgress:
      return "apa_not_in_progress";
    case PredictSpliceStatus::kPlanProjectionFail:
      return "plan_projection_fail";
    case PredictSpliceStatus::kMpcOnly:
      return "mpc_only";
    case PredictSpliceStatus::kSkipByControlOrGear:
      return "skip_by_control_or_gear";
    case PredictSpliceStatus::kSpliceProjectionFail:
      return "splice_projection_fail";
    case PredictSpliceStatus::kSpliceInsufficientPlanPoints:
      return "splice_insufficient_plan_points";
    case PredictSpliceStatus::kSplineSplice:
      return "spline_splice";
    case PredictSpliceStatus::kSteerAngleFallback:
      return "steer_angle_fallback";
  }
  return "unknown";
}

std::vector<double> UnwrapHeadingSequence(const std::vector<double>& heading_vec) {
  std::vector<double> heading_vec_unwrapped;
  if (heading_vec.empty()) {
    return heading_vec_unwrapped;
  }

  heading_vec_unwrapped.reserve(heading_vec.size());
  double heading_unwrapped = heading_vec[0];
  heading_vec_unwrapped.emplace_back(heading_unwrapped);
  for (size_t i = 1; i < heading_vec.size(); ++i) {
    heading_unwrapped +=
        pnc::geometry_lib::NormalizeAngle(heading_vec[i] - heading_vec[i - 1]);
    heading_vec_unwrapped.emplace_back(heading_unwrapped);
  }

  return heading_vec_unwrapped;
}
}

void ApaPredictPathManager::Update(
    const LocalView* local_view,
    const iflyauto::PlanningOutput* planning_output,
    const std::shared_ptr<ApaMeasureDataManager>& measure_data_ptr) {
  PredictSpliceStatus splice_status = PredictSpliceStatus::kInit;
  Reset();

  if (planning_output == nullptr || local_view == nullptr ||
      measure_data_ptr == nullptr) {
    splice_status = PredictSpliceStatus::kInvalidInput;
    JSON_DEBUG_VALUE("predict_splice_status", int(splice_status));
    ILOG_ERROR << "Update ApaPredictPathManager, local_view_ptr is nullptr";
    return;
  }

  ILOG_INFO << "Update ApaPredictPathManager";

  if (planning_output->planning_status.apa_planning_status !=
      iflyauto::APA_IN_PROGRESS) {
    splice_status = PredictSpliceStatus::kApaNotInProgress;
    JSON_DEBUG_VALUE("predict_splice_status", int(splice_status));
    ILOG_INFO << "apa not in progress, no predict_path";
    return;
  }

  const iflyauto::Trajectory& plan_traj = planning_output->trajectory;

  const int plan_pt_size =
      std::min(plan_traj.trajectory_points_size,
               static_cast<uint8>(PLANNING_TRAJ_POINTS_MAX_NUM));

  const double ego_x = measure_data_ptr->GetPos().x();
  const double ego_y = measure_data_ptr->GetPos().y();
  const double ego_phi = measure_data_ptr->GetHeading();

  const int pro_index = CalProjIndexFromPlanningTraj(
      plan_traj.trajectory_points, plan_pt_size, ego_x, ego_y);

  if (pro_index == -1) {
    splice_status = PredictSpliceStatus::kPlanProjectionFail;
    JSON_DEBUG_VALUE("predict_splice_status", int(splice_status));
    return;
  }

  const double path_length =
      plan_traj.trajectory_points[plan_pt_size - 1].distance;
  const double proj_s = plan_traj.trajectory_points[pro_index].distance;
  const double predict_distance = std::min(4.0, path_length - proj_s + 1.86);

  const double match_pt_x =
      planning_output->trajectory.trajectory_points[pro_index].x;
  const double match_pt_y =
      planning_output->trajectory.trajectory_points[pro_index].y;
  const double match_pt_phi =
      planning_output->trajectory.trajectory_points[pro_index].heading_yaw;

  phi_err_ =
      pnc::geometry_lib::NormalizeAngle(ego_phi - match_pt_phi) * kRad2Deg;

  lat_err_ = Eigen::Vector2d(ego_x - match_pt_x, ego_y - match_pt_y)
                 .dot(Eigen::Vector2d(-std::sin(match_pt_phi),
                                      std::cos(match_pt_phi)));

  if (std::fabs(lat_err_) > apa_param.GetParam().max_lat_err ||
      std::fabs(phi_err_) > apa_param.GetParam().max_phi_err) {
    control_err_big_ = true;
  }

  ILOG_INFO << "lat_err: " << lat_err_ << ", phi_err: " << phi_err_
            << ", control_err_big_: " << control_err_big_;

  JSON_DEBUG_VALUE("control_err_big", control_err_big_);

  constexpr double kPredictStep = 0.1;
  constexpr double kMaxHeadingJumpRad = 6.8 / 57.3;
  constexpr double kSpliceProjMaxDist = 1.5;
  constexpr double kSpliceProjMaxHeadingDiffRad = 35.0 / 57.3;
  constexpr int kSpliceProjSearchWindow = 68;
  constexpr int kSplicePlanPointCount = 8;

  const iflyauto::ControlTrajectory& mpc_traj =
      local_view->control_output.control_trajectory;

  const int mpc_pt_size =
      std::min(mpc_traj.control_result_points_size,
               static_cast<uint8>(CONTROL_RESULT_POINTS_MAX_NUM));

  const bool splice_plan_traj =
      (planning_output->gear_command.gear_command_value ==
       local_view->control_output.gear_command_value);

  bool use_steer_angle_flag = mpc_pt_size < 1;

  if (!use_steer_angle_flag) {
    ILOG_INFO << "use mpc traj";
    predict_pt_vec_.emplace_back(pnc::geometry_lib::PathPoint(
        measure_data_ptr->GetPos(), measure_data_ptr->GetHeading()));
    pnc::geometry_lib::LocalToGlobalTf l2g_tf(measure_data_ptr->GetPos(),
                                              measure_data_ptr->GetHeading());
    pnc::geometry_lib::PathPoint car_predict_pt;
    for (int i = 0; i < mpc_pt_size; ++i) {
      const iflyauto::Point3d& pt = mpc_traj.control_result_points[i];
      car_predict_pt.pos = l2g_tf.GetPos(Eigen::Vector2d(pt.x, pt.y));
      car_predict_pt.heading =
          pnc::geometry_lib::NormalizeAngle(l2g_tf.GetHeading(pt.z));

      const auto& last_predict_pt = predict_pt_vec_.back();

      const double ds =
          std::hypot(car_predict_pt.GetX() - last_predict_pt.GetX(),
                     car_predict_pt.GetY() - last_predict_pt.GetY());

      const double dphi = pnc::geometry_lib::NormalizeAngle(
          car_predict_pt.GetTheta() - last_predict_pt.GetTheta());

      if (ds > 1.0 || std::fabs(dphi) > kMaxHeadingJumpRad) {
        ILOG_ERROR << "control output is err";
        predict_pt_vec_.clear();
        break;
      }

      if (ds < 1e-2) {
        continue;
      }

      car_predict_pt.s = ds + last_predict_pt.s;
      predict_pt_vec_.emplace_back(car_predict_pt);
    }

    while (predict_pt_vec_.size() > 2 &&
           predict_pt_vec_.back().s < predict_distance) {
      // mpc traj is vaild, extend to predict_distance
      const double step = kPredictStep;
      const size_t cur_pt_size = predict_pt_vec_.size();
      // Extrapolating along a straight line
      do {
        size_t n = predict_pt_vec_.size();
        pnc::geometry_lib::CalExtendedPointByTwoPoints(
            predict_pt_vec_[n - 2].pos, predict_pt_vec_[n - 1].pos,
            car_predict_pt.pos, step);
        car_predict_pt.heading = predict_pt_vec_[n - 1].heading;
        car_predict_pt.s = predict_pt_vec_[n - 1].s + step;
        predict_pt_vec_.emplace_back(car_predict_pt);
      } while (predict_pt_vec_.back().s < predict_distance);

      if (control_err_big_ || !splice_plan_traj) {
        splice_status = PredictSpliceStatus::kSkipByControlOrGear;
        ILOG_INFO << "skip splicing and keep mpc extension, control_err_big="
                  << control_err_big_
                  << ", splice_plan_traj=" << splice_plan_traj
                  << ", cur_pt_size=" << cur_pt_size
                  << ", predict_distance=" << predict_distance;
        // no splice plan traj, directly extend mpc traj
        break;
      }

      // need splice plan traj with smooth transition
      const pnc::geometry_lib::PathPoint last_mpc_pt = predict_pt_vec_.back();

      int best_pro_index = -1;
      double best_proj_dist = std::numeric_limits<double>::infinity();
      const int search_start = std::max(0, pro_index);
      const int search_end =
          std::min(plan_pt_size, pro_index + kSpliceProjSearchWindow);
      for (int i = search_start; i < search_end; ++i) {
        const auto& plan_pt = plan_traj.trajectory_points[i];
        const double proj_dist = std::hypot(plan_pt.x - last_mpc_pt.GetX(),
                                            plan_pt.y - last_mpc_pt.GetY());
        const double heading_diff_rad =
            std::fabs(pnc::geometry_lib::NormalizeAngle(
                plan_pt.heading_yaw - last_mpc_pt.GetTheta()));
        if (proj_dist > kSpliceProjMaxDist ||
            heading_diff_rad > kSpliceProjMaxHeadingDiffRad) {
          continue;
        }
        if (proj_dist < best_proj_dist) {
          best_proj_dist = proj_dist;
          best_pro_index = i;
        }
      }

      if (best_pro_index == -1) {
        splice_status = PredictSpliceStatus::kSpliceProjectionFail;
        ILOG_INFO
            << "skip splicing due to invalid projection, keep mpc extension"
            << ", last_mpc_pt=(" << last_mpc_pt.GetX() << ", "
            << last_mpc_pt.GetY() << ", " << last_mpc_pt.GetTheta() << ")"
            << ", search_range=[" << search_start << ", " << search_end
            << ")"
            << ", proj_dist_limit=" << kSpliceProjMaxDist
            << ", heading_diff_limit_rad=" << kSpliceProjMaxHeadingDiffRad;
        break;
      }

      const double proj_s =
          plan_traj.trajectory_points[best_pro_index].distance;
      ILOG_INFO << "splice projection found"
                << ", best_pro_index=" << best_pro_index
                << ", best_proj_dist=" << best_proj_dist
                << ", proj_s=" << proj_s
                << ", last_mpc_s=" << last_mpc_pt.s;

      pnc::mathlib::spline x_s_spline, y_s_spline, heading_s_spline;
      std::vector<double> x_vec, y_vec, s_vec, heading_vec;
      for (size_t i = 0; i < cur_pt_size; ++i) {
        const auto& pt = predict_pt_vec_[i];
        x_vec.emplace_back(pt.GetX());
        y_vec.emplace_back(pt.GetY());
        s_vec.emplace_back(pt.s);
        heading_vec.emplace_back(pt.GetTheta());
      }

      const double splice_start_s = predict_pt_vec_[cur_pt_size - 1].s;
      const int splice_end_index =
          std::min(plan_pt_size, best_pro_index + kSplicePlanPointCount);
      for (int i = best_pro_index; i < splice_end_index; ++i) {
        const auto& plan_pt = plan_traj.trajectory_points[i];
        const double local_s = splice_start_s + (plan_pt.distance - proj_s);
        if (local_s <= s_vec.back() + 1e-3) {
          continue;
        }
        x_vec.emplace_back(plan_pt.x);
        y_vec.emplace_back(plan_pt.y);
        s_vec.emplace_back(local_s);
        heading_vec.emplace_back(plan_pt.heading_yaw);
      }

      if (s_vec.size() < cur_pt_size + 2) {
        splice_status = PredictSpliceStatus::kSpliceInsufficientPlanPoints;
        ILOG_INFO << "skip splicing due to insufficient plan points, keep mpc "
                     "extension"
                  << ", cur_pt_size=" << cur_pt_size
                  << ", sampled_point_count=" << s_vec.size()
                  << ", required_min=" << (cur_pt_size + 2)
                  << ", best_pro_index=" << best_pro_index
                  << ", splice_end_index=" << splice_end_index;
        break;
      }

      predict_pt_vec_.resize(cur_pt_size);

      const std::vector<double> heading_vec_unwrapped =
          UnwrapHeadingSequence(heading_vec);

      splice_status = PredictSpliceStatus::kSplineSplice;
      ILOG_INFO << "splice spline rebuild path"
                << ", cur_pt_size=" << cur_pt_size
                << ", heading_point_count=" << heading_vec_unwrapped.size()
                << ", target_predict_distance=" << predict_distance;
      x_s_spline.set_points(s_vec, x_vec);
      y_s_spline.set_points(s_vec, y_vec);
      heading_s_spline.set_points(s_vec, heading_vec_unwrapped);
      for (double s = predict_pt_vec_.back().s + kPredictStep;
           s < predict_distance; s += kPredictStep) {
        const double x = x_s_spline(s);
        const double y = y_s_spline(s);
        const double heading_unwrapped = heading_s_spline(s);
        const double heading =
            pnc::geometry_lib::NormalizeAngle(heading_unwrapped);
        car_predict_pt.SetX(x);
        car_predict_pt.SetY(y);
        car_predict_pt.SetTheta(heading);
        car_predict_pt.s = s;
        predict_pt_vec_.emplace_back(car_predict_pt);
      }
      break;
    }
  }

  use_steer_angle_flag = predict_pt_vec_.size() < 2;

  if (use_steer_angle_flag) {
    splice_status = PredictSpliceStatus::kSteerAngleFallback;
    ILOG_INFO << "use steer angle traj";
    Reset();
    const double steer_angle = measure_data_ptr->GetSteerWheelAngle();
    const uint8_t gear = (planning_output->gear_command.gear_command_value ==
                          iflyauto::GEAR_COMMAND_VALUE_DRIVE)
                             ? pnc::geometry_lib::SEG_GEAR_DRIVE
                             : pnc::geometry_lib::SEG_GEAR_REVERSE;
    pnc::geometry_lib::PathSegment path_seg;
    if (std::fabs(steer_angle) < 2.68 * kDeg2Rad) {
      pnc::geometry_lib::CalLineFromPt(
          gear, predict_distance,
          pnc::geometry_lib::PathPoint(measure_data_ptr->GetPos(),
                                       measure_data_ptr->GetHeading()),
          path_seg);
    } else {
      const auto& front_wheel_angle =
          std::fabs(measure_data_ptr->GetFrontWheelAngle());
      const double rear_axle_center_turn_radius =
          1.0 / (apa_param.GetParam().c1 * std::tan(front_wheel_angle));
      const uint8_t steer = (steer_angle > 0.0)
                                ? pnc::geometry_lib::SEG_STEER_LEFT
                                : pnc::geometry_lib::SEG_STEER_RIGHT;
      pnc::geometry_lib::CalArcFromPt(
          gear, steer, predict_distance, rear_axle_center_turn_radius,
          pnc::geometry_lib::PathPoint(measure_data_ptr->GetPos(),
                                       measure_data_ptr->GetHeading()),
          path_seg);
    }

    pnc::geometry_lib::SamplePointSetInPathSeg(predict_pt_vec_, path_seg, 0.2);
  }

  if (!use_steer_angle_flag && splice_status == PredictSpliceStatus::kInit) {
    splice_status = PredictSpliceStatus::kMpcOnly;
  }

  JSON_DEBUG_VALUE("predict_splice_status", int(splice_status));
  ILOG_INFO << "predict splice status: " << ToString(splice_status)
            << " (" << int(splice_status) << ")";

  if (planning_output->gear_command.available) {
    gear_ = (planning_output->gear_command.gear_command_value ==
             iflyauto::GEAR_COMMAND_VALUE_DRIVE)
                ? pnc::geometry_lib::SEG_GEAR_DRIVE
                : pnc::geometry_lib::SEG_GEAR_REVERSE;
  }

  if (predict_pt_vec_.size() > 0) {
    predict_traj_s_ = predict_pt_vec_.back().s;
  } else {
    predict_traj_s_ = 0.0;
  }

  ConvertPathPointsToPredictTrajectory(predict_pt_vec_);

  RecordDebugTraj();

  return;
}

void ApaPredictPathManager::RecordDebugTraj() {
  auto& debug = DebugInfoManager::GetInstance().GetDebugInfoPb();
  common::ApaPathDebug* path_debug = debug->mutable_apa_path_debug();
  path_debug->clear_predict_traj();

  common::Pose2d proto_point;
  for (const auto& pt : predict_pt_vec_) {
    proto_point.set_x(pt.pos.x());
    proto_point.set_y(pt.pos.y());
    proto_point.set_theta(pt.heading);

    path_debug->mutable_predict_traj()->add_points()->CopyFrom(proto_point);
  }

  return;
}

const trajectory::Trajectory&
ApaPredictPathManager::ConvertPathPointsToPredictTrajectory(
    const std::vector<pnc::geometry_lib::PathPoint>& predict_pt_vec) {
  predict_traj_.clear();
  predict_traj_.reserve(predict_pt_vec.size());
  trajectory::TrajectoryPoint traj_point;
  double absolute_time = 0.0;
  constexpr double kTimeStep = 0.1;
  for (const auto& pt : predict_pt_vec) {
    traj_point.set_absolute_time(absolute_time);
    traj_point.set_s(pt.s);
    traj_point.set_x(pt.GetX());
    traj_point.set_y(pt.GetY());
    traj_point.set_theta(pt.GetTheta());
    traj_point.set_kappa(pt.GetKappa());
    predict_traj_.emplace_back(traj_point);
    absolute_time += kTimeStep;
  }

  return predict_traj_;
}

}  // namespace apa_planner
}  // namespace planning