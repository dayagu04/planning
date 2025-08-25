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

void ApaPredictPathManager::Update(
    const LocalView* local_view,
    const iflyauto::PlanningOutput* planning_output,
    const std::shared_ptr<ApaMeasureDataManager>& measure_data_ptr) {
  // 填充车辆基于控制MPC或自车方向盘转角的预测轨迹，
  // 用于后续的碰撞检测和速度规划

  Reset();

  if (planning_output == nullptr || local_view == nullptr ||
      measure_data_ptr == nullptr) {
    ILOG_ERROR << "Update ApaPredictPathManager, local_view_ptr is nullptr";
    return;
  }

  ILOG_INFO << "Update ApaPredictPathManager";

  if (planning_output->planning_status.apa_planning_status !=
      iflyauto::APA_IN_PROGRESS) {
    ILOG_INFO << "apa not in progress, no predict_path";
    return;
  }

  // 计算预测距离  当前位置往后3米  或者
  // 当前位置投影点到规划轨迹的终点距离加上一个阈值
  // 首先计算当前位置投影点到规划轨迹终点的距离
  double min_dist = std::numeric_limits<double>::infinity();
  int min_index = -1;
  double path_length = 0.0;
  double proj_s = 0.0;
  int i = 0;
  const double ego_x = measure_data_ptr->GetPos().x();
  const double ego_y = measure_data_ptr->GetPos().y();
  const double ego_phi = measure_data_ptr->GetHeading();
  for (i = 0; i < std::min(planning_output->trajectory.trajectory_points_size,
                           static_cast<uint8>(PLANNING_TRAJ_POINTS_MAX_NUM));
       ++i) {
    const iflyauto::TrajectoryPoint& planning_pt =
        planning_output->trajectory.trajectory_points[i];

    if (i > 1) {
      path_length += (std::hypot(
          planning_pt.x -
              planning_output->trajectory.trajectory_points[i - 1].x,
          planning_pt.y -
              planning_output->trajectory.trajectory_points[i - 1].y));
    }

    const double dist =
        std::hypot(planning_pt.x - ego_x, planning_pt.y - ego_y);
    if (dist < min_dist) {
      min_index = i;
      min_dist = dist;
      proj_s = path_length;
    }
  }

  if (i == -1) {
    return;
  }

  const double match_pt_x =
      planning_output->trajectory.trajectory_points[min_index].x;
  const double match_pt_y =
      planning_output->trajectory.trajectory_points[min_index].y;
  const double match_pt_phi =
      planning_output->trajectory.trajectory_points[min_index].heading_yaw;

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

  const double predict_distance = std::min(4.0, path_length - proj_s + 1.86);

  bool use_steer_angle_flag = false;

  const auto& control_trajectory =
      local_view->control_output.control_trajectory;

  const bool splice_plan_traj =
      (planning_output->gear_command.gear_command_value ==
       local_view->control_output.gear_command_value);

  if (control_trajectory.control_result_points_size < 1) {
    use_steer_angle_flag = true;
  }

  if (!use_steer_angle_flag) {
    ILOG_INFO << "use mpc traj";
    predict_pt_vec_.emplace_back(pnc::geometry_lib::PathPoint(
        measure_data_ptr->GetPos(), measure_data_ptr->GetHeading()));
    pnc::geometry_lib::LocalToGlobalTf l2g_tf;
    l2g_tf.Init(measure_data_ptr->GetPos(), measure_data_ptr->GetHeading());
    for (i = 0; i < std::min(control_trajectory.control_result_points_size,
                             static_cast<uint8>(CONTROL_RESULT_POINTS_MAX_NUM));
         ++i) {
      const auto& pt = control_trajectory.control_result_points[i];
      pnc::geometry_lib::PathPoint car_predict_pt;
      car_predict_pt.pos = l2g_tf.GetPos(Eigen::Vector2d(pt.x, pt.y));
      car_predict_pt.heading =
          pnc::geometry_lib::NormalizeAngle(l2g_tf.GetHeading(pt.z));

      const double ds =
          std::hypot(car_predict_pt.pos.x() - predict_pt_vec_.back().pos.x(),
                     car_predict_pt.pos.y() - predict_pt_vec_.back().pos.y());

      const double dphi = pnc::geometry_lib::NormalizeAngle(
          car_predict_pt.heading - predict_pt_vec_.back().heading);

      if (ds > 1.0 || dphi * kRad2Deg > 6.8) {
        ILOG_ERROR << "control output is err";
        predict_pt_vec_.clear();
        break;
      }

      if (ds < 1e-2) {
        continue;
      }

      car_predict_pt.s = ds + predict_pt_vec_.back().s;
      predict_pt_vec_.emplace_back(std::move(car_predict_pt));
    }

    if (predict_pt_vec_.back().s < predict_distance &&
        predict_pt_vec_.size() > 2) {
      // 最后一个点的s小于predict_distance，需要补全到predict_distance
      pnc::geometry_lib::PathPoint car_predict_pt;
      const double step = 0.1;
      if (control_err_big_ || !splice_plan_traj) {
        // 直线延长
        do {
          size_t n = predict_pt_vec_.size();
          pnc::geometry_lib::CalExtendedPointByTwoPoints(
              predict_pt_vec_[n - 2].pos, predict_pt_vec_[n - 1].pos,
              car_predict_pt.pos, 0.1);
          car_predict_pt.heading = predict_pt_vec_[n - 1].heading;
          car_predict_pt.s = predict_pt_vec_[n - 1].s + step;
          predict_pt_vec_.emplace_back(car_predict_pt);
        } while (predict_pt_vec_.back().s < predict_distance);
      } else {
        // 拼接规划轨迹
        const pnc::geometry_lib::PathPoint& last_car_predict_pt =
            predict_pt_vec_.back();
        min_dist = std::numeric_limits<double>::infinity();
        min_index = -1;
        for (i = 0;
             i < std::min(planning_output->trajectory.trajectory_points_size,
                          static_cast<uint8>(PLANNING_TRAJ_POINTS_MAX_NUM));
             ++i) {
          const iflyauto::TrajectoryPoint& planning_pt =
              planning_output->trajectory.trajectory_points[i];
          const double dist =
              std::hypot(planning_pt.x - last_car_predict_pt.pos.x(),
                         planning_pt.y - last_car_predict_pt.pos.y());
          if (dist < min_dist) {
            min_index = i;
            min_dist = dist;
          }
        }

        // i indicates spling index from plan traj, can not be last index
        i = min_index + 1;
        bool use_planning_traj_flag = false;
        if (i < planning_output->trajectory.trajectory_points_size - 1) {
          ILOG_INFO << "splice mpc and planning traj";
          use_planning_traj_flag = true;
        } else {
          ILOG_INFO << "directly extend mpc traj";
        }

        for (; use_planning_traj_flag &&
               i < std::min(planning_output->trajectory.trajectory_points_size,
                            static_cast<uint8>(PLANNING_TRAJ_POINTS_MAX_NUM)) &&
               predict_pt_vec_.back().s < predict_distance;
             ++i) {
          car_predict_pt.pos
              << planning_output->trajectory.trajectory_points[i].x,
              planning_output->trajectory.trajectory_points[i].y;
          car_predict_pt.heading =
              planning_output->trajectory.trajectory_points[i].heading_yaw;

          const double ds = std::hypot(
              car_predict_pt.pos.x() - predict_pt_vec_.back().pos.x(),
              car_predict_pt.pos.y() - predict_pt_vec_.back().pos.y());

          car_predict_pt.s = ds + predict_pt_vec_.back().s;
          predict_pt_vec_.emplace_back(car_predict_pt);
        }

        if (predict_pt_vec_.back().s < predict_distance &&
            predict_pt_vec_.size() > 2) {
          do {
            size_t n = predict_pt_vec_.size();
            pnc::geometry_lib::CalExtendedPointByTwoPoints(
                predict_pt_vec_[n - 2].pos, predict_pt_vec_[n - 1].pos,
                car_predict_pt.pos, 0.1);
            car_predict_pt.heading = predict_pt_vec_[n - 1].heading;
            car_predict_pt.s = predict_pt_vec_[n - 1].s + step;
            predict_pt_vec_.emplace_back(car_predict_pt);
          } while (predict_pt_vec_.back().s < predict_distance);
        }
      }
    }
  }

  if (predict_pt_vec_.size() < 2) {
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

  if (planning_output->gear_command.available) {
    gear_ = (planning_output->gear_command.gear_command_value ==
             iflyauto::GEAR_COMMAND_VALUE_DRIVE)
                ? pnc::geometry_lib::SEG_GEAR_DRIVE
                : pnc::geometry_lib::SEG_GEAR_REVERSE;
  }

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

}  // namespace apa_planner
}  // namespace planning