/**
 * @file gap_selector_decider.h
 **/

#pragma once

#include <array>
#include <cstdint>
#include <string>
#include <vector>

#include "config/basic_type.h"
#include "define/geometry.h"
#include "gap_selector.pb.h"
#include "quintic_poly_path.h"
#include "session.h"
#include "speed/st_point.h"
// #include "speed/st_point.h"
#include "agent_node_manager.h"
#include "environmental_model.h"
#include "frenet_ego_state.h"
#include "lateral_obstacle.h"
#include "math/linear_interpolation.h"
#include "obstacle_manager.h"
// // #include "scenario_state_machine.h"
#include <memory>

#include "ego_planning_config.h"
#include "gap_selector_interface.h"
// #include "gap_selector_common.h"
// #include "gap_selector_debug_info.h"
#include "config/vehicle_param_tmp.h"
#include "spline_projection.h"
#include "task_basic_types.h"
#include "tasks/task.h"
#include "tasks/task_interface/gap_selcector_decider_output.h"
#include "trajectory1d/second_order_time_optimal_trajectory.h"
#include "trajectory1d/variable_coordinate_time_optimal_trajectory.h"
#include "utils/frenet_coordinate_system.h"
#include "utils/kd_path.h"
#include "virtual_lane.h"
#include "virtual_lane_manager.h"
// #define DEBUG_GAP_SELECTOR FALSE
#define __COLLECT_GAP_SELECTOR_DEBUG_INFO__

#define __COLLECT_GAP_SELECTOR_REPLAY_INFO__

#define __FUNCTION_CONSUMPTION_COLLECT__
namespace planning {
using namespace planning_math;

class GapSelectorDecider : public Task {
 public:
  explicit GapSelectorDecider(const EgoPlanningConfigBuilder *config_builder,
                              framework::Session *session);
  virtual ~GapSelectorDecider() = default;

  bool Execute() override;
  GapSelectorStatus Update(
      GapSelectorDeciderOutput &gap_selector_decider_output);

  GapSelectorStatus Update();

  void Preprocessor();
  GapSelectorStatus EnvHandle(
      const int target_state, const Point2D &ego_cart_pose,
      GapSelectorDeciderOutput &gap_selector_decider_output);

  void UpdateEnvInfos(const Point2D &ego_cart_pose,
                      PlanningInitPoint &planning_init_point);
  void UpdateSequenceInfos();
  void ResetAllInfo();

  int64_t SelectInteractObj();
  bool SetBaseFrenetCoordAndUpdateAgentNode();
  void CalcLatOffset(double &expected_s, double &expected_l,
                     double &refine_lc_time);
  int CollisionCheck();
  void CheckLaneCrossed();
  void ConvertObsPredToSTBoundary(
      const int64_t &id,
      const std::unordered_map<int64_t, ObstaclePredicatedInfo> &agent_node_map,
      std::vector<std::pair<STPoint, STPoint>> &st_boundary,
      double &dynamic_dis);
  int SafetyCheck(const GapSelectorPathSpline &path_spline);
  bool PreSafetyCheck();
  bool BodyCheck();

  void RestoreTrajResult(TrajectoryPoints &traj_points);
  void GenerateEgoTrajectory(TrajectoryPoints &traj_points);
  void GenerateLCTrajectory(TrajectoryPoints &traj_points);
  void GenerateLHTrajectory(
      const SecondOrderTimeOptimalTrajectory &lane_hold_time_speed_profile,
      TrajectoryPoints &traj_points);
  void GenerateLBTrajectory(TrajectoryPoints &traj_points);

  void ConstructTimeOptimal(
      const SecondOrderTimeOptimalTrajectory &time_optimal_traj,
      const GapSelectorPathSpline &path_spline);

  SecondOrderTimeOptimalTrajectory DriveStyleTimeOptimalTrajEvaluation(
      const double ego_v, const double target_v);
  void DriveStyleTimeOptimalTrajEvaluation(
      const GapSelectorPathSpline &path_spline);
  double QueryLaneWidth(
      const double s0,
      const std::vector<std::pair<double, double>> &lane_s_width);
  void MakeDriveStyleDecision();
  bool MakeInteractiveDecision(
      const ObstaclePredicatedInfo &interactive_obj_pred_info,
      const GapSelectorPathSpline &path_spline,
      SecondOrderTimeOptimalTrajectory &traj_time_optimal);
  bool MakeInteractiveDecision(
      const ObstaclePredicatedInfo &front_obj_pred_info,
      const ObstaclePredicatedInfo &rear_obj_pred_info,
      const GapSelectorPathSpline &path_spline,
      SecondOrderTimeOptimalTrajectory &traj_time_optimal);

  bool PathSplineLengthCheck(GapSelectorPathSpline &path_spline);
  void StitchQuinticPath(const double &truancation_end_s,
                         const double expected_lc_time, const double expected_l,
                         pnc::spline::QuinticPolynominalPath &quintic_path,
                         GapSelectorPathSpline &path_spline);
  void DecoupleQuinticPathSpline(
      const double remaining_lc_duration,
      const pnc::spline::QuinticPolynominalPath &quintic_path,
      const std::shared_ptr<KDPath> coord, GapSelectorPathSpline &path_spline);
  void GenerateLinearRefTrajectory(TrajectoryPoints &traj_points);
  void RetentivePathPlan();
  void ResponsivePathPlan();
  void FixedTimeQuinticPathPlan(const double lat_avoid_offset,
                                const double lc_end_s,
                                const double remain_lc_duration,
                                TrajectoryPoints &traj_points);

  void RefineLCTime(double *lc_end_s, double *remain_lc_time,
                    const double lat_avoid_offset);

  void ObtainNearbyGap();
  void QuerySplineCrossLinePoint(GapSelectorPathSpline &path_spline);
  void ConstructGaps();

  MonotonicStatus MonotonicCheck(const std::vector<double> &x_vec);
  pnc::spline::QuinticPolynominalPath ConstructQuinticPath(
      double &expected_s, double &expected_l, double &truancation_end_s,
      const double expected_lc_time);
  pnc::spline::QuinticPolynominalPath ConstructQuinticPath(
      const double remaining_lc_duration, const shared_ptr<KDPath> coord);

  bool CheckLCFinish();

#ifdef __COLLECT_GAP_SELECTOR_DEBUG_INFO__
  GapSelectorInterface gap_selector_interface_;
#endif

 public:
  GapSelectorConfig config_;
  VehicleParam vehicle_param_;
  std::vector<Gap> gap_list_;
  Gap nearby_gap_;
  std::vector<double> lc_time_list_{5.0, 7., 9.};
  int target_state_{0};  // 0 --none; 1--left; 2--right

  int path_spline_selected_id_{-1};
  GapDriveStyle gap_drive_style_{NONESTYLE};

  bool retreat_signal_{false};
  double target_l_ = 0.;
  double cruise_vel_{0.};
  std::vector<GapSelectorPathSpline> gap_selector_path_spline_;

  int64_t front_careful_car_id_origin_lane_{-1};

  std::array<double, 3> ego_planning_init_s_;
  std::array<double, 2> ego_cart_point_;
  std::array<double, 2> planning_init_cart_point_;
  std::array<double, 2> planning_init_frenet_point_;
  double ego_l_{0.};                // ego current l
  double ego_l_cur_lane_{0.};       //// ego current lane l
  double target_lane_width_ = 3.5;  // near ego s lane width
  double origin_lane_width_ = 3.5;  // near ego s lane width

  double rear_car_dynamic_dis_ = 0.;
  double front_car_dynamic_dis_ = 0.;
  double ego_lane_car_dynamic_dis_ = 0.;

  std::vector<STPoint> st_time_optimal_;
  std::vector<STPoint> st_upper_boundaries_;
  std::vector<STPoint> st_lower_boundaries_;
  std::vector<LonInfo> loninfo_time_optimal_;

  bool use_query_lane_width_ = false;
  std::vector<std::pair<double, double>>
      target_lane_s_width_;  // <s, lane_width>
  std::vector<std::pair<double, double>> origin_lane_s_width_;

  std::vector<std::pair<STPoint, STPoint>>
      front_gap_car_st_boundaries_;  // <upper, lower>
  std::vector<std::pair<STPoint, STPoint>>
      rear_gap_car_st_boundaries_;  // <upper, lower>
  std::vector<std::pair<STPoint, STPoint>>
      front_careful_car_st_boundary_;  //<upper, lower>

  std::vector<int> ids_obstacle_in_origin_lane_;
  std::vector<int> ids_obstacle_in_target_lane_;
  std::unordered_map<int, Obstacle> gs_care_obstacles_;

  std::vector<int> lc_request_sequence_{
      NO_CHANGE, NO_CHANGE,
      NO_CHANGE};  // 0 ---- NO_CHANGE, 1----LEFT_CHANGE, 2----RIGHT_CHANGE
  CrossedLinePointInfo cross_line_point_info_;

  GapSelectorStateMachineInfo gap_selector_state_machine_info_{
      false, false, false, false, false,     0.,       0.,
      0.,    false, false, false, {0, 0, 0}, {0, 0, 0}};

  std::vector<double> ego_l_buffer_{0., 0., 0.};
  GapSelectorPathSpline::PathSplineStatus gs_status_{
      GapSelectorPathSpline::NO_VALID};

  TrajectoryPoints *traj_points_ptr_;
  PlanningInitPoint planning_init_point_;

  Transform2D transform_;
  Transform2D refline_transform_;
  GapSelectorPathSpline path_spline_;

  SecondOrderTimeOptimalTrajectory traj_time_optimal_;
  SecondOrderTimeOptimalTrajectory interact_speed_adjusted_time_optimal_;

  // virtual lane coord list
  std::shared_ptr<KDPath> current_lane_coord_ptr_;
  std::shared_ptr<KDPath> origin_lane_coord_ptr_;
  std::shared_ptr<KDPath> target_lane_coord_ptr_;

  std::shared_ptr<KDPath> base_frenet_coord_;
  std::shared_ptr<AgentNodeManager> agent_node_mgr_;

  ScenarioStateEnum last_target_state_{ROAD_NONE};
  double lc_timer_{0.};
  Point2D frenet_init_point_;
  bool use_ego_v_{false};
  bool use_ego_point_{false};
  double lc_total_time_{6.0};
  double lc_back_total_time_{3.5};
  double lc_back_timer_{0.};

  std::vector<double> _LB_T_;
  std::vector<double> _LB_HEADING_ERROR_;

};

}  // namespace planning