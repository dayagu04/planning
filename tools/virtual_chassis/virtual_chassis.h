#pragma once

#include <Eigen/Core>
#include <Eigen/Dense>
#include <cmath>

#include "config/vehicle_param.h"
#include "localization.pb.h"
#include "pose2d.h"
#include "vehicle_service.pb.h"
#include "virtual_chassis_state.h"

namespace planning {

class VirtualChassis {
 public:
  VirtualChassis(/* args */);
  ~VirtualChassis();

  int Init(const VehicleParam &veh_param, const double interval_second);

  int SetCommandTimeInterval(const double interval_second) {
    cmd_interval_ = interval_second;
    return 0;
  }

  int SetVehicleParam(const VehicleParam &veh_param) {
    veh_params_ = veh_param;

    return 0;
  }

  int SetInitChassisState(const double x, const double y, const double z,
                          const double heading) {
    state_.x_ = x;
    state_.y_ = y;
    state_.z_ = z;

    state_.heading_ = heading;

    state_.v_ = 0.0;
    state_.acc_ = 0.0;
    state_.steer_ = 0.0;

    return 0;
  }

  int ResetCommand() {
    cmd_.v_ = 0;
    cmd_.acc_ = 0;
    cmd_.steer_ = 0;
    return 0;
  }

  int Process(const ChassisCommand &cmd);

  int GetState(ChassisState *state) const {
    *state = state_;

    return 0;
  }

  int SetState(const ChassisState state) {
    state_ = state;

    return 0;
  }

  int PerfectResponse();

  // steering_angle, left is positive
  // move_dist, forward is positive
  int UpdateChassisPose(Pose2D *veh_pose, const double steering_angle,
                        const double move_dist, const VehicleParam *veh_params);

  inline double CalcTurnRadius(const double lfr,
                               const double steering_angle) const;

  Eigen::Vector3d InverseQuaternionRotate(
      const LocalizationOutput::Quaternion &orientation,
      const Eigen::Vector3d &rotated) {
    Eigen::Quaterniond quaternion(orientation.qw(), orientation.qx(),
                                  orientation.qy(), orientation.qz());

    return static_cast<Eigen::Vector3d>(
        quaternion.toRotationMatrix().inverse() * rotated);
  }

  /**
   * map reference frame, vehicle reference frame
   */
  void TransformToVehRefFrame(const Common::Point3d &point_mrf,
                              const LocalizationOutput::Quaternion &orientation,
                              Common::Point3d *point_vrf) {
    Eigen::Vector3d v_mrf(point_mrf.x(), point_mrf.y(), point_mrf.z());
    auto v_vrf = InverseQuaternionRotate(orientation, v_mrf);

    point_vrf->set_x(v_vrf.x());
    point_vrf->set_y(v_vrf.y());
    point_vrf->set_z(v_vrf.z());

    return;
  }

  int ChassisResponseByDelay();

  int ChassisResponseByDelayAndOvershoot();

  int ChassisResponseBySpeed();

 private:
  // input
  ChassisCommand cmd_;

  // state
  ChassisState state_;

  // time: second
  double history_time;
  double current_time;
  double cmd_interval_;

  VehicleParam veh_params_;

  // history
  ChassisState history_state_;
};

}  // namespace planning