/**
 * @file lateral_decider.h
 **/

#pragma once

#include "environmental_model.h"
#include "frenet_ego_state.h"
#include "lateral_behavior_planner.pb.h"
#include "lateral_obstacle.h"
#include "math/linear_interpolation.h"
#include "obstacle_manager.h"
#include "planning_context.h"
#include "quintic_poly_path.h"
// #include "scenario_state_machine.h"
#include "spline_projection.h"
#include "task_basic_types.h"
#include "tasks/task.h"
#include "utils/hysteresis_decision.h"
#include "utils/kd_path.h"
#include "virtual_lane.h"
#include "virtual_lane_manager.h"

namespace planning {

using namespace planning_math;

struct RoadCurvatureInfo {
  // 弯道类型
  enum class CurveType {
    STRAIGHT = 0,      // 直道
    NORMAL_CURVE = 1,  // 弯道
    BIG_CURVE = 2,     // 大曲率弯道
    S_CURVE = 3,       // S弯
    SHARP_CURVE = 4,   // 急弯
  };

  CurveType curve_type = CurveType::STRAIGHT;
  bool is_left = false;                 // 是否大曲率左弯
  bool is_right = false;                // 是否大曲率右弯
  double start_s = 0.0;                 // 大曲率起始纵向位置
  double end_s = 0.0;                   // 大曲率结束纵向位置
  double min_radius = 1e4;              // 最小曲率半径
  double max_curve = 1e-4;              // 最大曲率值
  double max_curve_s = 0.0;             // 最大曲率对应的纵向位置
  std::vector<double> curve_vec;        // 曲率序列
  std::vector<double> s_vec;            // 对应的纵向距离序列

  void Clear() {
    curve_type = CurveType::STRAIGHT;
    is_left = false;
    is_right = false;
    start_s = 0.0;
    end_s = 0.0;
    min_radius = 1e4;
    max_curve = 1e-4;
    max_curve_s = 0.0;
    curve_vec.clear();
    s_vec.clear();
  }
};

class GeneralLateralDecider : public Task {
 public:
  explicit GeneralLateralDecider(const EgoPlanningConfigBuilder *config_builder,
                                 framework::Session *session);

  virtual ~GeneralLateralDecider() = default;

  bool Execute() override;

  bool ExecuteTest(bool pipeline_test);

  bool InitInfo();
  void UnitTest();

 private:
  bool CalCruiseVelByCurvature(const double ego_v,
                               const CoarsePlanningInfo &coars_planning_info,
                               double &cruise_v);

  bool HandleRoadCurvature(const double ego_v,
                           const CoarsePlanningInfo& coars_planning_info);

  void ConstructTrajPoints(TrajectoryPoints &traj_points);

  // // 1. construct the trajectory of reference and bind the obstacle info on
  bool ConstructReferencePathPoints(const TrajectoryPoints &traj_points);

  // 2. construct the lane and boundary bound
  void GenerateRoadAndLaneBoundary();
  void UpdateDistanceToRoadBorder();
  void GenerateRoadHardSoftBoundary();
  void GenerateLaneSoftBoundary();

  void GetDesireRoadExtraBuffer(double *const left_road_extra_buffer,
                                double *const right_road_extra_buffer);
  void GetLateralTTCToRoad(double *max_collision_t,
                           double *const left_collision_t,
                           double *const right_collision_t);

  // 3. construct the obstacle decisions
  void GenerateObstaclesBoundary();
  void GenerateStaticObstaclesBoundary(
      const std::vector<std::shared_ptr<FrenetObstacle>> obs_vec,
      ObstacleDecisions &obstacle_decisions);
  void GenerateStaticObstacleDecision(
      const std::shared_ptr<FrenetObstacle> obstacle,
      ObstacleDecision &obstacle_decision, bool is_update_hard_bound);
  void GenerateDynamicObstaclesBoundary(
      const std::vector<std::shared_ptr<FrenetObstacle>> obs_vec,
      ObstacleDecisions &obstacle_decisions);
  void GenerateDynamicObstacleDecision(
      const std::shared_ptr<FrenetObstacle> obstacle,
      ObstacleDecision &obstacle_decision, bool is_update_hard_bound);
  double CalculateExtraDecreaseBuffer(
      const std::shared_ptr<FrenetObstacle> obstacle, bool is_nudge_left);
  double CalculateExtraLaneTypeDecreaseBuffer(bool is_nudge_left,
                                              const double start_s,
                                              const double end_s);
  void CalculateExtraLaneWidthDecreaseBuffer();

  bool CheckObstacleNudgeDecision(
      const std::shared_ptr<FrenetObstacle> &obstacle);

  bool CheckObstacleCrossingCondition(
      const std::shared_ptr<FrenetObstacle> obstacle, bool &is_cross_obj);

  void RefineConflictLatDecisions(const double &ego_l,
                                  ObstacleDecision &obstacle_decision);
  bool HackYawSideObstacle(const std::shared_ptr<FrenetObstacle> obstacle,
                           bool is_nudge_left, double &limit_overlap_min_y,
                           double &limit_overlap_max_y);
  bool IsCutoutSideObstacle(const std::shared_ptr<FrenetObstacle> obstacle,
                            double &limit_overlap_min_y,
                            double &limit_overlap_max_y);
  double CalLaneWidth();
  iflyauto::LaneBoundaryType CalLaneBoundaryType(const LineDirection direction,
                                                 const double s) const;
  void PostProcessReferenceTrajBySoftBound(
      const std::vector<std::pair<double, double>> &frenet_soft_bounds,
      GeneralLateralDeciderOutput &general_lateral_decider_output);
  void ExtractBoundary(
      std::vector<std::pair<double, double>> &frenet_soft_bounds,
      std::vector<std::pair<double, double>> &frenet_hard_bounds,
      std::vector<std::pair<BoundInfo, BoundInfo>> &soft_bounds_info,
      std::vector<std::pair<BoundInfo, BoundInfo>> &hard_bounds_info);
  void ProtectBoundByInitPoint(std::pair<double, double> &bound,
                               std::pair<BoundInfo, BoundInfo> &bound_info);
  void ExtractDynamicObstacleBound(const ObstacleDecision &obstacle_decision);
  void ExtractStaticObstacleBound(const ObstacleDecision &obstacle_decision);

  void PostProcessBound(const double planning_init_point_l,
                        const std::vector<WeightedBound> &bounds_input,
                        std::pair<double, double> &bound_output,
                        std::pair<BoundInfo, BoundInfo> &bound_info);
  void SaveLatDebugInfo(
      const std::vector<std::pair<double, double>> &frenet_soft_bounds,
      const std::vector<std::pair<double, double>> &frenet_hard_bounds,
      const std::vector<std::pair<BoundInfo, BoundInfo>> &soft_bounds_info,
      const std::vector<std::pair<BoundInfo, BoundInfo>> &hard_bounds_info);

  void GenerateObstaclePreliminaryDecision(
      double ego_l, double distance_to_right_lane_border,
      double distance_to_left_lane_border, double overlap_min_y,
      double overlap_max_y, double lat_buf_dis, bool b_overlap_side,
      bool init_lon_no_overlap, bool is_nudge_left, bool is_cross_obj,
      LatObstacleDecisionType pre_lateral_decision,
      bool &reset_conflict_decision, ObstacleDecision &obstacle_decision,
      LatObstacleDecisionType &lat_decision,
      LonObstacleDecisionType &lon_decision);
  void AddObstacleDecisionBound(int id, double t, BoundType bound_type,
                                double overlap_min_y, double overlap_max_y,
                                double lat_buf_dis,
                                LatObstacleDecisionType lat_decision,
                                LonObstacleDecisionType lon_decision,
                                ObstacleDecision &obstacle_decision,
                                bool is_update_hard_bound = false,
                                bool is_avoid_side_ignore_obj = false,
                                bool is_high_dangerous = false);
  void GenerateLateralDeciderOutput(
      const std::vector<std::pair<double, double>> &frenet_soft_bounds,
      const std::vector<std::pair<double, double>> &frenet_hard_bounds,
      const std::vector<std::pair<BoundInfo, BoundInfo>> &soft_bounds_info,
      const std::vector<std::pair<BoundInfo, BoundInfo>> &hard_bounds_info,
      GeneralLateralDeciderOutput &general_lateral_decider_output);
  void GenerateEnuBoundaryPoints(
      const std::vector<std::pair<double, double>> &frenet_soft_bounds,
      const std::vector<std::pair<double, double>> &frenet_hard_bounds,
      GeneralLateralDeciderOutput &general_lateral_decider_output);

  void SampleRoadDistanceInfo(const double &s_target,
                              ReferencePathPoint &sample_path_point);

  void GenerateEnuReferenceTraj(
      GeneralLateralDeciderOutput &general_lateral_decider_output);

  void GenerateEnuReferenceTheta(
      GeneralLateralDeciderOutput &general_lateral_decider_output);

  void HandleLaneChangeScene(TrajectoryPoints &traj_points);
  void HandleRefPathOffset(TrajectoryPoints &traj_points, std::vector<std::pair<double, double>> &front_axis_ref_path,
                           double dynamic_ref_buffer);
  void CalcLateralBehaviorOutput();
  bool IsLonOverlap(const std::shared_ptr<FrenetObstacle> obstacle);
  bool IsFarObstacle(const std::shared_ptr<FrenetObstacle> obstacle);
  bool IsRearObstacle(const std::shared_ptr<FrenetObstacle> obstacle);
  bool IsBlockedObstacleInLaneBorrow(
      const std::shared_ptr<FrenetObstacle> obstacle);
  bool IsFilterForStaticObstacle(
      const std::shared_ptr<FrenetObstacle> obstacle);
  bool IsFilterForDynamicObstacle(
      const std::shared_ptr<FrenetObstacle> obstacle);
  bool IsAgentPredLonOverlapWithPlanPath(
      const std::shared_ptr<FrenetObstacle> obstacle);
  double CalculateSideObstacleExtraDecreaseBufferInIntersection(
      const std::shared_ptr<FrenetObstacle> obstacle, bool is_nudge_left,
      bool in_intersection);
  double AdjustBufferForSideObstacleInIntersection(
      const std::shared_ptr<FrenetObstacle> obstacle, double overlap_min_y,
      double overlap_max_y, double lat_buf_dis, bool is_nudge_left,
      double rear_lon_buf_dis, double front_lon_buf_dis,
      LatObstacleDecisionType lat_decision, int index);
  bool CheckPredLonOverlapStability(
      int id, bool is_agent_pred_lon_overlap_with_plan_path);
  void ResetIsExceedObstacleHysteresisMap(int id = -1);
  void CalculateAvoidObstacles(
      const std::vector<std::pair<double, double>> frenet_soft_bounds,
      std::vector<std::pair<BoundInfo, BoundInfo>> soft_bounds_info);
  double CalStaticNudgeLatBufDis(const std::shared_ptr<FrenetObstacle> obstacle,
                                 bool in_intersection, bool is_nudge_left,
                                 double overlap_min_y, double overlap_max_y,
                                 bool is_side_obstacle,
                                 double extra_lane_type_decrease_buffer,
                                 bool is_update_hard_bound, double lane_width,
                                 bool is_care_reverse_ignore_obj);
  double CalDynamicNudgeLatBufDis(
      const std::shared_ptr<FrenetObstacle> obstacle, bool in_intersection,
      bool is_nudge_left, double overlap_min_y, double overlap_max_y,
      double limit_overlap_min_y, double limit_overlap_max_y, double pred_ts,
      double extra_lane_type_decrease_buffer,
      bool is_same_side_obstacle_during_lane_change, bool is_update_hard_bound,
      double extra_reverse_obj_decrease_buffer, bool is_care_reverse_ignore_obj,
      double last_t_lat_buf_dis, double &updated_overlap_min_y,
      double &updated_overlap_max_y);
  bool IsSameSideObstacleDuringLaneChange(
      const std::shared_ptr<FrenetObstacle> obstacle);
  void PostProcessRoadSoftBoundary();
  void ExtendRoadSoftBound(int num_rear_extended_points,
                           int num_front_extended_points, bool is_lower,
                           int bound_size);
  void GenerateEmergencyObstacleDecision(
      const std::shared_ptr<FrenetObstacle> obstacle,
      ObstacleDecision &obstacle_decision);
  double CalEmergencyNudgeLatBufDis(
      const std::shared_ptr<FrenetObstacle> obstacle, bool in_intersection,
      bool is_nudge_left, double overlap_min_y, double overlap_max_y,
      double limit_overlap_min_y, double limit_overlap_max_y, double pred_ts,
      double extra_lane_type_decrease_buffer,
      bool is_same_side_obstacle_during_lane_change,
      double &updated_overlap_min_y, double &updated_overlap_max_y);
  bool CheckLateralEmergencyAvoidSpace(
      bool is_nudge_left, const std::shared_ptr<FrenetObstacle> obstacle);
  bool IsObstacleOutsideRoadBoundary(
      const std::shared_ptr<FrenetObstacle> obstacle);
  void LimitFrenetLateralSlope(
      std::vector<std::pair<double, double>> &frenet_bounds);
  void GenerateRecommendJerk(
      const std::shared_ptr<FrenetObstacle> obstacle,
      bool &is_high_dangerous);
  void CheckObstacleSideCutinNudgeCondition(
      const std::shared_ptr<FrenetObstacle> obstacle, bool &is_nudge_left,
      BoundType &bound_type, bool &is_avoid_side_ignore_obj, bool &is_side_obstacle);

 private:
  GeneralLateralDeciderConfig config_;

  // VelocityLimitInfo vel_limit_info_;
  // LatIgnoreType lat_ignore_type_;
  TrajectoryPoints ref_traj_points_;
  TrajectoryPoints plan_history_traj_;
  TrajectoryPoints uniform_plan_history_traj_;
  std::unordered_map<int, std::vector<int>> match_index_map_;
  std::unordered_map<int, HysteresisDecision>
      is_exceed_obstacle_hysteresis_map_;
  bool is_agent_current_pred_lonoverlap_ = false;

  ReferencePathPoints ref_path_points_;
  ObstacleDecisions static_obstacle_decisions_;
  ObstacleDecisions dynamic_obstacle_decisions_;

  std::vector<WeightedBounds> soft_bounds_;
  std::vector<WeightedBounds> hard_bounds_;

  std::vector<std::pair<double, double>> frenet_soft_bounds_;
  std::vector<std::pair<double, double>> frenet_hard_bounds_;
  std::vector<std::pair<BoundInfo, BoundInfo>> soft_bounds_info_;
  std::vector<std::pair<BoundInfo, BoundInfo>> hard_bounds_info_;

  FrenetEgoState ego_frenet_state_;
  std::shared_ptr<EgoStateManager> ego_cart_state_manager_;
  std::shared_ptr<ReferencePath> reference_path_ptr_;
  double min_road_radius_ = 10000.0;
  double last_overlap_min_y_ = 0.0;
  double last_overlap_max_y_ = 0.0;
  double cruise_vel_ = 0.0;
  double extra_lane_width_decrease_buffer_ = 0.0;
  double overlap_start_s_ = 0.0;
  double overlap_end_s_ = 0.0;
  double last_ref_length_ = 0;
  double last_lc_ref_offset_ = 0.0;
  bool is_lane_change_scene_ = false;
  bool is_blocked_obstacle_ = false;
  LatDeciderLaneChangeInfo lat_lane_change_info_ =
      LatDeciderLaneChangeInfo::NONE;
  planning::common::LateralBehaviorDebugInfo lat_debug_info_;
  double emergency_avoid_lateral_distance_thr_ = 0.0;
  bool is_potential_dangerous_obstacle_ = false;
  HysteresisDecision has_enough_speed_emergency_avoid_hysteresis_;
  bool enable_emergency_avoid_ = false;
  HysteresisDecision has_enough_speed_bound_recurrence_hysteresis_;
  bool is_use_recurrence_ = false;
  RoadCurvatureInfo ref_curve_info_;
  double last_compensation_buffer_ = 0.0;
  std::unordered_map<uint32_t, double> current_desire_final_nudge_l_map_;
  std::unordered_map<uint32_t, double> last_desire_final_nudge_l_map_;

};

}  // namespace planning
