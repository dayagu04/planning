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

enum class PredictPathType : uint8_t {
  INVALID = 0,
  ONLY_MPC = 1,
  EXTEND_MPC = 2,
  MPC_SPLICE_PLAN = 3,
  ONLY_STEER_ANGLE = 4,
};

const char* ToString(const PredictPathType type) {
  switch (type) {
    case PredictPathType::INVALID:
      return "invalid";
    case PredictPathType::ONLY_MPC:
      return "only_mpc";
    case PredictPathType::EXTEND_MPC:
      return "extend_mpc";
    case PredictPathType::MPC_SPLICE_PLAN:
      return "mpc_splice_plan";
    case PredictPathType::ONLY_STEER_ANGLE:
      return "only_steer_angle";
  }
  return "invalid";
}

std::vector<double> UnwrapHeadingSequence(
    const std::vector<double>& heading_vec) {
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
}  // namespace

void ApaPredictPathManager::Update(
    const LocalView* local_view,
    const iflyauto::PlanningOutput* planning_output,
    const std::shared_ptr<ApaMeasureDataManager>& measure_data_ptr) {
  PredictPathType predict_path_type = PredictPathType::INVALID;
  Reset();

  if (planning_output == nullptr || local_view == nullptr ||
      measure_data_ptr == nullptr) {
    JSON_DEBUG_VALUE("predict_path_type", int(predict_path_type));
    ILOG_ERROR << "Update ApaPredictPathManager, local_view_ptr is nullptr";
    return;
  }

  ILOG_INFO << "Update ApaPredictPathManager";

  if (planning_output->planning_status.apa_planning_status !=
      iflyauto::APA_IN_PROGRESS) {
    JSON_DEBUG_VALUE("predict_path_type", int(predict_path_type));
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

  constexpr int kSplicePlanPointCount = 4;
  constexpr double kPredictStep = 0.1;

  const iflyauto::ControlTrajectory& mpc_traj =
      local_view->control_output.control_trajectory;

  const int mpc_pt_size =
      std::min(mpc_traj.control_result_points_size,
               static_cast<uint8>(CONTROL_RESULT_POINTS_MAX_NUM));

  const bool splice_plan_traj =
      (planning_output->gear_command.gear_command_value ==
       local_view->control_output.gear_command_value);

  int cur_predict_size = 0;
  double cur_predict_s = 0.0;
  // try taking points in the MPC path as predict path
  while (mpc_pt_size > 1) {
    predict_pt_vec_.emplace_back(pnc::geometry_lib::PathPoint(
        measure_data_ptr->GetPos(), measure_data_ptr->GetHeading()));
    pnc::geometry_lib::LocalToGlobalTf l2g_tf(measure_data_ptr->GetPos(),
                                              measure_data_ptr->GetHeading());
    pnc::geometry_lib::PathPoint car_predict_pt;
    for (int i = 0; i < mpc_pt_size; ++i) {
      const auto& pt = mpc_traj.control_result_points[i];
      car_predict_pt.pos = l2g_tf.GetPos(Eigen::Vector2d(pt.x, pt.y));
      car_predict_pt.heading =
          pnc::geometry_lib::NormalizeAngle(l2g_tf.GetHeading(pt.z));

      const auto& last_predict_pt = predict_pt_vec_.back();

      const double ds =
          std::hypot(car_predict_pt.GetX() - last_predict_pt.GetX(),
                     car_predict_pt.GetY() - last_predict_pt.GetY());

      const double dphi = pnc::geometry_lib::NormalizeAngle(
          car_predict_pt.GetTheta() - last_predict_pt.GetTheta());

      if (ds > 1.0 || std::fabs(dphi) > 6.8 / 57.3) {
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

    cur_predict_size = predict_pt_vec_.size();
    if (cur_predict_size < 2) {
      predict_pt_vec_.clear();
      predict_path_type = PredictPathType::ONLY_STEER_ANGLE;
      break;
    }

    cur_predict_s = predict_pt_vec_.back().s;
    if (cur_predict_s > predict_distance) {
      predict_path_type = PredictPathType::ONLY_MPC;
      break;
    }

    const auto extend_predict_pts = [&]() {
      do {
        size_t n = predict_pt_vec_.size();
        pnc::geometry_lib::CalExtendedPointByTwoPoints(
            predict_pt_vec_[n - 2].pos, predict_pt_vec_[n - 1].pos,
            car_predict_pt.pos, kPredictStep);

        car_predict_pt.heading = predict_pt_vec_[n - 1].heading;
        car_predict_pt.s = predict_pt_vec_[n - 1].s + kPredictStep;
        predict_pt_vec_.emplace_back(car_predict_pt);

      } while (predict_pt_vec_.back().s < predict_distance);
    };

    const auto get_proj_index = [&]() -> int {
      const auto& last_predict_pt = predict_pt_vec_.back();
      return CalProjIndexFromPlanningTraj(plan_traj.trajectory_points,
                                          plan_pt_size, last_predict_pt.GetX(),
                                          last_predict_pt.GetY());
    };

    const int mpc_index = get_proj_index();
    const double mpc_s = plan_traj.trajectory_points[mpc_index].distance;

    if (!splice_plan_traj || cur_predict_s > predict_distance - 0.3 ||
        mpc_index >= plan_pt_size - kSplicePlanPointCount) {
      predict_path_type = PredictPathType::EXTEND_MPC;
      extend_predict_pts();
      break;
    }

    extend_predict_pts();
    const int extend_index = get_proj_index();
    if (extend_index <= mpc_index) {
      predict_path_type = PredictPathType::ONLY_STEER_ANGLE;
      predict_pt_vec_.clear();
      break;
    }

    predict_pt_vec_.resize(cur_predict_size);

    pnc::mathlib::spline x_s_spline, y_s_spline, heading_s_spline;
    std::vector<double> x_vec, y_vec, s_vec, heading_vec;
    for (const auto& pt : predict_pt_vec_) {
      x_vec.emplace_back(pt.GetX());
      y_vec.emplace_back(pt.GetY());
      s_vec.emplace_back(pt.s);
      heading_vec.emplace_back(pt.GetTheta());
    }

    for (int i = mpc_index + 1; i <= extend_index; ++i) {
      const auto& plan_pt = plan_traj.trajectory_points[i];
      const double local_s = cur_predict_s + (plan_pt.distance - mpc_s);
      if (local_s <= s_vec.back() + 1e-3) {
        continue;
      }
      x_vec.emplace_back(plan_pt.x);
      y_vec.emplace_back(plan_pt.y);
      s_vec.emplace_back(local_s);
      heading_vec.emplace_back(plan_pt.heading_yaw);
    }

    const std::vector<double> heading_vec_unwrapped =
        UnwrapHeadingSequence(heading_vec);

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

  if (predict_path_type == PredictPathType::ONLY_STEER_ANGLE) {
    predict_pt_vec_.clear();
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

  JSON_DEBUG_VALUE("predict_path_type", int(predict_path_type));
  ILOG_INFO << "predict_path_type: " << ToString(predict_path_type);

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