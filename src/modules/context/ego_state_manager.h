#pragma once
#include <array>

#include "src/framework/session.h"
#include "modules/common/config/message_type.h"
#include "modules/common/math/polygon2d.h"
#include "modules/context/vehicle_config_context.h"
#include "modules/context/environmental_model.h"
#include "modules/common/transform.h"


namespace planning {

class EgoStateManager {
  public:
    EgoStateManager(framework::Session *session);
    ~EgoStateManager() = default;

    void update();

    double navi_timestamp() const { return navi_timestamp_; }
    double ego_v_cruise() { return ego_v_cruise_; }
    double ego_t_distance() { return ego_t_distance_; }
    double x() const { return x_; }
    double y() const { return y_; }
    double heading_angle() const { return heading_angle_; }
    double ego_steer_angle() const { return ego_steer_angle_; }
    double velocity() const { return velocity_; }
    int ego_start_stop() const { return ego_start_stop_; }
    double acc() const { return acc_; }
    double jerk() const { return jerk_; }
    const planning_math::Polygon2d polygon() const { return polygon_; }
    const PlanningInitPoint &planning_init_point() const {
      return planning_init_point_;
    }
    const VehicleParam &vehicle_param() const { return vehicle_param_; }
    const define::Transform &enu2car() { return enu2car_; }
    const define::Transform &car2enu() { return car2enu_; }
    const std::vector<PncTrajectoryPoint> &stitching_trajectory()
        const {
      return stitch_trajectory_;
    }
    bool ego_throttle_override() { return ego_throttle_override_; }

 private:
  void update_planning_init_point();
  std::vector<PncTrajectoryPoint> compute_stitching_trajectory();

 private:
  framework::Session *session_ = nullptr;
  VehicleParam vehicle_param_;

  double navi_timestamp_;
  double x_;
  double y_;
  double heading_angle_;
  double velocity_;
  double velocity_angle_;
  double acc_;
  double jerk_;
  double ego_steer_angle_;
  double ego_v_cruise_;
  double ego_t_distance_;
  int ego_start_stop_;
  bool ego_throttle_override_;
  planning_math::Polygon2d polygon_;

  std::vector<PncTrajectoryPoint> stitch_trajectory_;
  PlanningInitPoint planning_init_point_;

  define::Transform car2enu_;
  define::Transform enu2car_;
};
}