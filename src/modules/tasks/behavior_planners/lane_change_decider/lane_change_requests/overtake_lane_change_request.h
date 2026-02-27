#pragma once

#include "config/basic_type.h"
#include "lane_change_request.h"
#include "src/modules/common/math/filter/mean_filter.h"
#include "tracked_object.h"

namespace planning {

/// @brief 自主式(Active)换道请求
class OvertakeRequest : public LaneChangeRequest {
 public:
  OvertakeRequest(const EgoPlanningConfigBuilder* config_builder,
                  planning::framework::Session* session,
                  std::shared_ptr<VirtualLaneManager> virtual_lane_mgr,
                  std::shared_ptr<LaneChangeLaneManager> lane_change_lane_mgr);
  virtual ~OvertakeRequest() = default;

  void Update(int lc_status);

  void Reset();

  bool isCancelOverTakingLaneChange(int lc_state);

  int GetOvertakeCount() { return overtake_count_; }

  int GetOvertakeVehicleId() { return overtake_vehicle_id_; }
  virtual void SetLaneChangeCmd(std::uint8_t lane_change_cmd) {
    lane_change_cmd_ = lane_change_cmd;
  }
  virtual void SetLaneChangeCancelFromTrigger(bool trigger_lane_change_cancel) {
    trigger_lane_change_cancel_ = trigger_lane_change_cancel;
  }
  virtual IntCancelReasonType lc_request_cancel_reason() {
    return lc_request_cancel_reason_;
  }

 private:
  void setLaneChangeRequestByFrontSlowVehcile(int lc_status);

  bool isSatisfyOvertakeCountUpdateCondition(const agent::Agent* leading_agent,
                                             const double ego_speed,
                                             const double reference_speed,
                                             const double leading_vehicle_dist,
                                             const bool rain_mode);
  bool isSatisfyOvertakeCountMaintainCondition(
      const agent::Agent* leading_agent, const double reference_speed,
      const double leading_vehicle_dist, const bool rain_mode);
  void updateOvertakeCount(const agent::Agent* leading_agent,
                           const double ego_speed, const double reference_speed,
                           const int max_count_thres);

  void updateRouteTrafficSpeed(
      const bool is_left, double* route_traffic_speed,
      planning::planning_math::MeanFilter& traffic_speed_filter);

  bool isCouldOvertakeByRoute(
      const std::shared_ptr<ReferencePath>& base_ref_line,
      const std::shared_ptr<VirtualLane>& target_lane,
      const double& lane_traffic_speed, const agent::Agent* agent,
      const bool& is_left, const bool& left_and_right_both_on_navigation_route,
      const double& total_feasible_lane_remain_distance,
      double& left_overtake_speed_threshold,
      double& right_overtake_speed_threshold);

  bool isCouldOvertakeMaintainByRoute(
      const double lane_traffic_speed, const agent::Agent* agent,
      const bool is_left, const std::shared_ptr<VirtualLane>& target_lane,
      const double& total_feasible_lane_remain_distance,
      const bool& both_lane_is_on_navigation_route);

  bool FeasibleLaneDistanceEnoughJudgment(
      const double& lane_traffic_speed, const double& leading_speed,
      const std::shared_ptr<VirtualLane>& target_lane, bool is_left,
      const double need_s);

  void updateLaneChangeSafety(
      const std::shared_ptr<ReferencePath>& left_ref_line,
      const std::shared_ptr<ReferencePath>& right_ref_line);

  bool checkLeftLaneChangeValidByObjects(
      const std::shared_ptr<ReferencePath>& ref_line);

  bool checkRightLaneChangeValidByObjects(
      const std::shared_ptr<ReferencePath>& ref_line);

  bool checkLaneChangeValidBySuprsSignal(const bool is_left);

  bool checkLeftLaneChangeValid(const std::shared_ptr<ReferencePath>& ref_line,
                                const bool is_left);

  bool checkRightLaneChangeValid(const std::shared_ptr<ReferencePath>& ref_line,
                                 const bool is_left);

  bool checkLaneChangeSafety(
      const std::shared_ptr<ReferencePath>& target_ref_line,
      const double extra_front_distance_buffer,
      const double extra_rear_distance_buffer, const double safety_forward_time,
      const double safety_backward_time, const bool is_left,
      const double lon_safety_ratio, const double lat_safety_ratio,
      const bool use_dynamic_safety_distance,
      std::vector<int>* not_safe_agent_ids, double* front_required_space,
      double* rear_required_space);

  void selectTargetObstacleIds(
      const std::shared_ptr<planning_math::KDPath>& ref_line,
      const Point2D ego_cart_point,
      const std::vector<std::shared_ptr<FrenetObstacle>> candidate_obs_info,
      const double search_range, const int max_target_num,
      const double ego_half_width, const double l_buffer,
      const double max_l_buffer, const bool order_reverse,
      std::vector<int>* target_tracks_ids);

  double getSafetyDistance(const double low_speed_safety_distance,
                           const double high_speed_safety_distance,
                           const double vehicle_speed,
                           const bool use_dynamic_safety_distance);

  bool checkFrontSafetyAtPresent(
      const double long_dis, const double ego_v, const double veh_v,
      const double ego_a, const double veh_a, const double ego_front_edge,
      const double veh_half_length, const double base_distance_buffer,
      const double safety_ratio, double* front_required_space);

  bool checkFrontSafetyAtFuture(const double long_dis, const double ego_v,
                                const double veh_v, const double ego_a,
                                const double veh_a, const double ego_front_edge,
                                const double veh_half_length,
                                const double base_distance_buffer,
                                const double safety_ratio,
                                const double future_time);

  bool checkRearSafetyAtPresent(const double long_dis, const double ego_v,
                                const double veh_v, const double ego_a,
                                const double veh_a, const double ego_rear_edge,
                                const double veh_half_length,
                                const double base_distance_buffer,
                                const double safety_backward_time,
                                const double safety_ratio,
                                double* rear_required_space);

  bool checkRearSafetyAtFuture(const double long_dis, const double ego_v,
                               const double veh_v, const double ego_a,
                               const double veh_a, const double ego_rear_edge,
                               const double veh_half_length,
                               const double base_distance_buffer,
                               const double safety_ratio,
                               const double future_time);

  double getDrivingDistance(const double v, const double a, const double t,
                            double* v_out);

  double CalculateAttenuationCoefficient(const double& lc_duration);

  void IsTargetLaneExistTruck(const std::shared_ptr<agent::Agent>& agent,
                              const std::shared_ptr<VirtualLane>& target_lane,
                              bool is_left,
                              double& target_lane_exist_truck_speed);

  EgoPlanningConfig config_;
  std::shared_ptr<planning_math::KDPath> base_frenet_coord_;
  PlanningInitPoint planning_init_point_;
  std::shared_ptr<ReferencePath> left_reference_path_ = nullptr;
  std::shared_ptr<ReferencePath> right_reference_path_ = nullptr;
  bool enable_l_ = false;
  bool enable_r_ = false;
  int overtake_count_ = 0;
  int overtake_vehicle_id_ = -1;  //初始化为有关无效的障碍物track_id
  double overtake_vehicle_speed_ = 0.0;
  int target_lane_exist_slow_front_veh_frame_num_ = 0;
  double left_overtake_valid_timestamp_ = std::numeric_limits<double>::max();
  double right_overtake_valid_timestamp_ = std::numeric_limits<double>::max();
  double left_invalid_timestamp_ = 0.0;
  double right_invalid_timestamp_ = 0.0;
  bool is_left_lane_change_safe_ = false;
  bool is_right_lane_change_safe_ = false;
  std::shared_ptr<LateralObstacle> lateral_obstacle_ = nullptr;
  std::shared_ptr<LaneTracksManager> lane_tracks_manager_ = nullptr;
  planning::planning_math::MeanFilter left_traffic_speed_filter_;
  planning::planning_math::MeanFilter right_traffic_speed_filter_;
  planning::planning_math::MeanFilter leading_speed_filter_;
  bool left_lane_exist_truck_ = false;
  bool right_lane_exist_truck_ = false;
  RequestType last_request_type_ = NO_CHANGE;
  int right_lane_nums_ = 0;
  int left_lane_nums_ = 0;
  double left_feasible_lane_remain_distance_ = 1500.0;
  double right_feasible_lane_remain_distance_ = 1500.0;
  bool exist_cross_line_large_agent_ahead_ = false;
  int truck_confirm_frame_count_ = 0;
  std::unordered_map<int32_t, int> truck_confirm_frame_count_map_;
  const std::vector<double> lane_width_bp_{3.2, 3.5, 3.7, 3.9};
  const std::vector<double> lane_width_factor_{0.8, 1, 1, 1.2};
  bool left_lane_exist_cross_line_truck_ = false;
  bool right_lane_exist_cross_line_truck_ = false;
  double left_lane_exist_cross_line_truck_speed_ = 33.33;
  double right_lane_exist_cross_line_truck_speed_ = 33.33;
};

}  // namespace planning