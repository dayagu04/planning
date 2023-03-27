#ifndef ZNQC_MODULES_CONTEXT_CART_EGO_STATE_H_
#define ZNQC_MODULES_CONTEXT_CART_EGO_STATE_H_

#include "modules/common/transform.h"
#include "modules/context/vehicle_config_context.h"
#include "modules/common/define/geometry.h"
#include "proto/generated_files/ego_state_info.pb.h"
#include "proto/generated_files/vehicle_status.pb.h"
#include "modules/common/config/basic_type.h"
// #include "framework/session.h"

namespace planning {

class CartEgoStateManager {
 public:
  CartEgoStateManager(const VehicleParam &vehicle_param);
  ~CartEgoStateManager() = default;

  void set_ego_carte(const Pose2D &ego_carte) {
    cart_ego_state_.mutable_ego_carte()->set_x(ego_carte.x);
    cart_ego_state_.mutable_ego_carte()->set_y(ego_carte.y);
  }
  // void compute_stitch_trajectory(bool dbw_status);
  const planning::common::CartEgoState &get_cart_ego_state() const {
    return cart_ego_state_;
  }
  planning::common::CartEgoState &get_mutable_cart_ego_state() {
    return cart_ego_state_;
  }
  bool get_flag_is_replan() const { return cart_ego_state_.flag_is_replan(); }
  void set_ego_pose_and_vel(
      const planning::common::VehicleStatus &vehicle_status) {
    cart_ego_state_.mutable_ego_pose()->set_x(
        vehicle_status.location().location_enu().x());
    cart_ego_state_.mutable_ego_pose()->set_y(
        vehicle_status.location().location_enu().y());
    cart_ego_state_.mutable_ego_pose()->set_theta(
        vehicle_status.heading_yaw().heading_yaw_data().value_rad());
    cart_ego_state_.set_latitude(
        vehicle_status.location().location_geographic().latitude_degree());
    cart_ego_state_.set_longitude(
        vehicle_status.location().location_geographic().longitude_degree());

    cart_ego_state_.mutable_ego_pose_raw()->CopyFrom(
        cart_ego_state_.ego_pose());
    cart_ego_state_.mutable_ego_pose()->set_x(
        cart_ego_state_.ego_pose().x() -
        std::cos(cart_ego_state_.ego_pose().theta()) * vehicle_param_.length /
            2.0);
    cart_ego_state_.mutable_ego_pose()->set_y(
        cart_ego_state_.ego_pose().y() -
        std::sin(cart_ego_state_.ego_pose().theta()) * vehicle_param_.length /
            2.0);
    cart_ego_state_.set_ego_vel(
        vehicle_status.velocity().heading_velocity().value_mps());
  }
  void set_ego_enu(
      const planning::common::VehicleStatus &vehicle_status) {
    const auto &location_enu = vehicle_status.location().location_enu();
    cart_ego_state_.mutable_ego_enu()->mutable_position()->set_x(
        location_enu.x());
    cart_ego_state_.mutable_ego_enu()->mutable_position()->set_y(
        location_enu.y());
    cart_ego_state_.mutable_ego_enu()->mutable_position()->set_z(
        location_enu.z());
    cart_ego_state_.mutable_ego_enu()->mutable_orientation()->set_x(
        location_enu.orientation().x());
    cart_ego_state_.mutable_ego_enu()->mutable_orientation()->set_y(
        location_enu.orientation().y());
    cart_ego_state_.mutable_ego_enu()->mutable_orientation()->set_z(
        location_enu.orientation().z());
    cart_ego_state_.mutable_ego_enu()->mutable_orientation()->set_w(
        location_enu.orientation().w());
    update_transform();
  }
  void set_ego_navi_timestamp(double egopose_timestamp) {
    cart_ego_state_.set_ego_navi_timestamp(egopose_timestamp);
  }
  void set_ego_steer_angle(
      const planning::common::VehicleStatus &vehicle_status) {
    cart_ego_state_.set_ego_steer_angle(vehicle_status.steering_wheel()
                                            .steering_wheel_data()
                                            .steering_wheel_rad());
  }
  void set_ego_acc(
      const planning::common::VehicleStatus &vehicle_status) {
    cart_ego_state_.set_ego_acc(vehicle_status.brake_info()
                                    .brake_info_data()
                                    .acceleration_on_vehicle_wheel());
  }
  void set_ego_v_cruise(
      const planning::common::VehicleStatus &vehicle_status) {
    cart_ego_state_.set_ego_v_cruise(
        vehicle_status.velocity().cruise_velocity().value_mps());
  }
  void set_ego_t_distance(
      const planning::common::VehicleStatus &vehicle_status) {
    cart_ego_state_.set_ego_t_distance(vehicle_status.navi_time_distance());
  }
  void set_ego_start_stop(
      const planning::common::VehicleStatus &vehicle_status) {
    cart_ego_state_.set_ego_start_stop(vehicle_status.start_stop_info());
  }

  void set_throttle_override(
      const planning::common::VehicleStatus &vehicle_status) {
    cart_ego_state_.set_throttle_override(
        vehicle_status.throttle().throttle_data().override());
  }
  void set_ego_blinker(
      const planning::common::VehicleStatus &vehicle_status) {
    cart_ego_state_.set_ego_blinker(vehicle_status.vehicle_light()
                                        .vehicle_light_data()
                                        .turn_signal()
                                        .value());
  }
  void set_ego_blinker(
      const planning::common::VehicleLight &vehicle_light) {
    cart_ego_state_.set_ego_blinker(
        vehicle_light.vehicle_light_data().turn_signal().value());
  }
  const define::Transform &get_car2enu() const { return car2enu_; }
  const define::Transform &get_enu2car() const { return enu2car_; }

  const planning::common::CartEgoState &get_proto() const { return cart_ego_state_; }

  const VehicleParam &vehicle_param() const { return vehicle_param_; }

  void set_vehicle_param(const VehicleParam &vehicle_param) {
    vehicle_param_ = vehicle_param;
  }

 private:
  void update_transform();

  planning::common::CartEgoState cart_ego_state_;
  define::Transform car2enu_;
  define::Transform enu2car_;
  VehicleParam vehicle_param_;
  // planning::framework::Session *session_ = nullptr;

};
}  // planning

#endif