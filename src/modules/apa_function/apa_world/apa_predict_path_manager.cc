#include "apa_predict_path_manager.h"

#include <cstdint>

#include "apa_param_config.h"
#include "geometry_math.h"
#include "local_view.h"
#include "log_glog.h"

namespace planning {
namespace apa_planner {

void ApaPredictPathManager::Update(
    const LocalView* local_view,
    const iflyauto::PlanningOutput* planning_output,
    const std::shared_ptr<ApaMeasureDataManager> measure_data_ptr) {
  // 填充车辆基于控制MPC或自车方向盘转角的预测轨迹，
  // 用于后续的碰撞检测和速度规划

  Reset();

  if (planning_output == nullptr || local_view == nullptr ||
      measure_data_ptr == nullptr) {
    return;
  }

  if (planning_output->planning_status.apa_planning_status !=
      iflyauto::APA_IN_PROGRESS) {
    return;
  }

  const double predict_distance = 3.0;

  bool use_steer_angle_flag = false;

  const auto& control_trajectory =
      local_view->control_output.control_trajectory;

  if (control_trajectory.control_result_points_size < 1) {
    use_steer_angle_flag = true;
  }

  if (!use_steer_angle_flag) {
    ILOG_INFO << "use mpc traj";
    predict_pt_vec_.emplace_back(pnc::geometry_lib::PathPoint(
        measure_data_ptr->GetPos(), measure_data_ptr->GetHeading()));
    pnc::geometry_lib::LocalToGlobalTf l2g_tf;
    l2g_tf.Init(measure_data_ptr->GetPos(), measure_data_ptr->GetHeading());
    for (size_t i = 0;
         i < std::min(control_trajectory.control_result_points_size,
                      static_cast<uint8>(CONTROL_RESULT_POINTS_NUM));
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

      if (ds > 0.5 || dphi * kRad2Deg > 5.1) {
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

    if (predict_pt_vec_.back().s < predict_distance) {
      const pnc::geometry_lib::PathPoint& last_car_predict_pt =
          predict_pt_vec_.back();
      double min_dist = std::numeric_limits<double>::infinity();
      int min_index = -1;
      for (int i = 0;
           i < std::min(planning_output->trajectory.trajectory_points_size,
                        static_cast<uint8>(PLANNING_TRAJ_POINTS_NUM));
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

      for (int i = min_index + 1;
           i < std::min(planning_output->trajectory.trajectory_points_size,
                        static_cast<uint8>(PLANNING_TRAJ_POINTS_NUM)) &&
           predict_pt_vec_.back().s < predict_distance;
           ++i) {
        pnc::geometry_lib::PathPoint car_predict_pt;
        car_predict_pt.pos
            << planning_output->trajectory.trajectory_points[i].x,
            planning_output->trajectory.trajectory_points[i].y;
        car_predict_pt.heading =
            planning_output->trajectory.trajectory_points[i].heading_yaw;

        const double ds =
            std::hypot(car_predict_pt.pos.x() - predict_pt_vec_.back().pos.x(),
                       car_predict_pt.pos.y() - predict_pt_vec_.back().pos.y());

        car_predict_pt.s = ds + predict_pt_vec_.back().s;
        predict_pt_vec_.emplace_back(std::move(car_predict_pt));
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
}

}  // namespace apa_planner
}  // namespace planning