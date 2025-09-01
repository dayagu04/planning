#pragma once

#include "lane_change_request.h"
#include "tracked_object.h"
#include "src/modules/common/math/filter/mean_filter.h"

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

  void updateRouteTrafficSpeed(const bool is_left, double* route_traffic_speed, planning::planning_math::MeanFilter &traffic_speed_filter);

  bool isCouldOvertakeByRoute(
      const std::shared_ptr<ReferencePath>& base_ref_line,
      const std::shared_ptr<VirtualLane>& target_lane,
      const double lane_traffic_speed, const agent::Agent* agent,
      const int left_lane_num, const int right_lane_num, const bool is_left);
      
  bool isCouldOvertakeMaintainByRoute(
    const double lane_traffic_speed, const agent::Agent* agent,
    const bool is_left);

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
};

}  // namespace planning