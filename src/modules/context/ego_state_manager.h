#ifndef ZNQC_MODULES_CONTEXT_EGO_STATE_MANAGER_H_
#define ZNQC_MODULES_CONTEXT_EGO_STATE_MANAGER_H_

#include <cstdint>
#include <unordered_set>

#include "config/basic_type.h"
#include "config/message_type.h"
#include "config/vehicle_param.h"
#include "define/geometry.h"
#include "ego_planning_config.h"
#include "filters.h"
#include "math/polygon2d.h"
#include "refline.h"
#include "session.h"
#include "transform.h"
#include "vehicle_config_context.h"
#include "vehicle_status.pb.h"

namespace planning {

class EgoStateManager {
 public:
  EgoStateManager(const EgoPlanningConfigBuilder *config_builder,
                  framework::Session *session);
  ~EgoStateManager() = default;

  // enum ReplanStatus {
  //   NONE,
  //   LAT_REPLAN,
  //   LON_REPLAN,
  // };

  enum ReplanType {
    LAT_POSITION_REPLAN = 1,
    LAT_ANGLE_REPLAN = 2,
    LON_POSITION_REPLAN = 4,
    LON_TINY_SPEED_REPLAN = 8,
    LAT_LON_REPLAN = 16,
    LAT_REPLAN = 32,
    LAT_lON_REST = 64,
  };

  bool update(const planning::common::VehicleStatus &vehicle_status);
  void set_ego_carte(const Point2D &ego_carte);

  void set_ego_position_llh(
      const planning::common::VehicleStatus &vehicle_status);
  void set_ego_enu(const planning::common::VehicleStatus &vehicle_status);
  void set_ego_pose_and_vel(
      const planning::common::VehicleStatus &vehicle_status);
  void set_ego_prediction_info(double ego_pose_timestamp);
  void set_ego_steer_angle(
      const planning::common::VehicleStatus &vehicle_status);
  void set_ego_acc(const planning::common::VehicleStatus &vehicle_status);
  void set_ego_v_cruise(const planning::common::VehicleStatus &vehicle_status);
  void set_ego_t_distance(
      const planning::common::VehicleStatus &vehicle_status);
  void set_ego_start_stop(
      const planning::common::VehicleStatus &vehicle_status);
  void set_throttle_override(
      const planning::common::VehicleStatus &vehicle_status);
  void set_ego_blinker(const planning::common::VehicleLight &vehicle_light);
  void set_ego_blinker(const planning::common::VehicleStatus &vehicle_status);
  void set_ego_auto_light_state(
      const planning::common::VehicleStatus &vehicle_status);
  void set_driver_hand_state(
      const planning::common::VehicleStatus &vehicle_status);
  void set_planning_init_point_valid(bool planning_init_point_valid) {
    planning_init_point_valid_ = planning_init_point_valid;
  };
  void set_ego_gear(const planning::common::VehicleStatus &vehicle_status);

  // const planning::VehicleParam &get_vehicle_param() const {
  //   return vehicle_param_;
  // };
  double navi_timestamp() const { return navi_timestamp_; }  // todo
  double location_timestamp() const { return timestamp_us_; }
  Pose location_enu() const { return location_enu_; };
  PointLLH position_llh() const { return position_llh_; };
  EulerAngle euler_angle() const { return euler_angle_; }
  Pose2D ego_pose() const { return ego_pose_; };
  Pose2D ego_pose_raw() const { return ego_pose_raw_; };
  Point2D ego_carte() const { return ego_carte_; };
  uint32_t ego_gear() const { return ego_gear_; };
  double heading_angle() const { return ego_pose_.theta; }
  double ego_v() const { return ego_v_; };
  double ego_v_angle() const { return ego_v_angle_; };
  double ego_yaw_rate() const { return ego_yaw_rate_; }
  double ego_v_cruise() const { return ego_v_cruise_; };
  double ego_acc() const { return ego_acc_; };
  double ego_hmi_v() const { return ego_hmi_v_; }
  double ego_steer_angle() const { return ego_steer_angle_; };
  uint ego_blinker() const { return ego_blinker_; };
  bool ego_auto_light_state() const { return ego_auto_light_state_; }
  float driver_hand_torque() const { return driver_hand_torque_; }
  bool driver_hands_off_state() const { return driver_hands_off_state_; }
  double jerk() const { return jerk_; };
  double t_distance() const { return ego_t_distance_; };
  int start_stop() const { return ego_start_stop_; };
  bool is_auto() const { return is_auto_; };
  bool flag_is_replan() const { return flag_is_replan_; };
  bool throttle_override() const { return throttle_override_; };
  const planning_math::Polygon2d polygon() const { return polygon_; }

  const PlanningInitPoint &planning_init_point() const {
    return planning_init_point_;
  };
  PlanningInitPoint &mutable_planning_init_point() {
    return planning_init_point_;
  };
  bool planning_init_point_valid() const { return planning_init_point_valid_; };

  const std::vector<PncTrajectoryPoint> &stitching_trajectory() const {
    return stitch_trajectory_;
  }

  const define::Transform &get_car2enu() const { return car2enu_; };
  const define::Transform &get_enu2car() const { return enu2car_; };

 private:
  void update_transform();
  void UpdatePlanningInitState();
  void RealtimeUpdatePlanningInitState();
  // uint8_t ReplanProcess(const bool &lat_reset_flag, const bool
  // &lon_reset_flag); new replan
  uint8_t ReplanProcess(const bool &lat_reset_flag, const bool &lon_reset_flag);

  void LateralReset();
  void LongitudinalReset();
  void MotionPlanningInfoReset();
  bool LateralStitch();
  bool LongitudinalStitch();

  void set_timestamp_us(const planning::common::VehicleStatus &vehicle_status);

 private:
  framework::Session *session_ = nullptr;
  EgoPlanningEgoStateManagerConfig config_;
  double parking_cruise_speed_;
  double max_replan_lat_err_;
  double max_replan_theta_err_;
  double max_replan_lon_err_;
  double max_replan_dist_err_;
  double hpp_max_replan_lat_err_;
  double hpp_max_replan_theta_err_;
  double hpp_max_replan_lon_err_;
  double hpp_max_replan_dist_err_;
  double navi_timestamp_;
  uint64_t timestamp_us_ = 0;
  uint64_t timestamp_us_last_ = 0;
  uint32_t ego_gear_;
  Pose location_enu_;
  PointLLH position_llh_;
  EulerAngle euler_angle_;  // 车身姿态yaw, pitch, roll
  Pose2D ego_pose_;
  Pose2D ego_pose_raw_;
  Point2D ego_carte_;
  double ego_v_ = 0;
  double ego_v_angle_ = 0;
  double ego_yaw_rate_ = 0.;
  double ego_v_cruise_ = 0;
  double ego_acc_ = 0;
  double ego_acc_last_ = 0;
  double ego_hmi_v_;
  double ego_steer_angle_ = 0;
  uint ego_blinker_ = 0;
  bool ego_auto_light_state_;
  float driver_hand_torque_;
  bool driver_hands_off_state_;
  double jerk_ = 0;
  double ego_t_distance_ = 0;
  int ego_start_stop_ = 0;
  bool is_auto_ = false;
  bool flag_is_replan_ = false;
  bool throttle_override_ = false;
  planning_math::Polygon2d polygon_;
  PlanningInitPoint planning_init_point_;
  bool planning_init_point_valid_ = false;
  std::unordered_set<ReplanType> replan_type_;
  pnc::filters::SlopeFilter v_cruise_filter_;  // 对巡航车速变化速率限制

  std::vector<PncTrajectoryPoint> stitch_trajectory_;

  define::Transform car2enu_;
  define::Transform enu2car_;
};
}  // namespace planning

#endif
