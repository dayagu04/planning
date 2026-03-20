#pragma once

#include <memory>
#include <unordered_map>
#include <vector>
#include "frenet_obstacle.h"
#include "obstacle.h"
#include "session.h"
#include "tasks/behavior_planners/lateral_obstacle_decider/ARAStar/scc_hybrid_ara_star.h"
#include "tasks/behavior_planners/lateral_obstacle_decider/base_lateral_obstacle_decider.h"
#include "tasks/task.h"
#include "tasks/task_interface/lateral_obstacle_decider_output.h"
#include "utils/kd_path.h"
#include "utils/hysteresis_decision.h"

namespace planning {

class SccLateralObstacleDecider : public BaseLateralObstacleDecider {
  struct LcGapInfo {
    int gap_front_id = -1;
    int gap_rear_id = -1;
    double gap_front_s = 300;
    double gap_rear_s = -100;
    void Reset() {
      gap_front_id = -1;
      gap_rear_id = -1;
      gap_front_s = 300;
      gap_rear_s = -100;
    }
  };

  enum class LateralSpaceConstraintType {
    DEFAULT = 0,
    LANE_BOUNDARY,
    ROAD_BOUNDARY,
    DYNAMIC_OBSTACLE,
    STATIC_OBSTACLE,
  };
  using LateralConstraintFreeSpaceMap =
      std::unordered_map<LateralSpaceConstraintType, double>;

  struct ObstacleInfo {
    int id = -1;
    LatObstacleDecisionType lateral_avoid_direction =
        LatObstacleDecisionType::NOT_SET;
    LateralConstraintFreeSpaceMap lateral_constraint_free_space_map;
    bool has_static_safe_v_space = false;
    bool has_dynamic_safe_v_space = false;
    double desire_nudge_static_safe_v = 0;
    double desire_nudge_dynamic_safe_v = 0;

    double GetLateralConstraintDistance(
        LateralSpaceConstraintType type) const {
      auto it = lateral_constraint_free_space_map.find(type);
      if (it == lateral_constraint_free_space_map.end()) {
        return std::numeric_limits<double>::infinity();
      }
      return it->second;
    }

    void SetLateralConstraintFreeSpace(
        LateralSpaceConstraintType type, double free_space) {
      lateral_constraint_free_space_map[type] = free_space;
    }

    void Reset() {
      id = -1;
      lateral_avoid_direction = LatObstacleDecisionType::NOT_SET;
      lateral_constraint_free_space_map.clear();
      has_static_safe_v_space = false;
      has_dynamic_safe_v_space = false;
      desire_nudge_static_safe_v = 0;
      desire_nudge_dynamic_safe_v = 0;
    }
  };


 public:
  SccLateralObstacleDecider(const EgoPlanningConfigBuilder *config_builder,
                            framework::Session *session);
  virtual ~SccLateralObstacleDecider() = default;

  bool Execute() override;
  bool ExecuteTest(bool pipeline_test);

 private:
  bool Init();
  bool CheckIsRightestLane();
  bool IsChangeLanes();
  void UpdateAvdObstacles();
  void UpdateAvdObstacle(const FrenetObstacle &frenet_obs, double expand_vel,
                         double farthest_distance, bool rightest_lane,
                         bool is_in_lane_change_execution_scene);
  bool IsPotentialAvoidingCar(const FrenetObstacle &frenet_obstacle,
                              bool rightest_lane, double farthest_distance,
                              bool can_left_borrow, bool can_right_borrow,
                              bool is_in_lane_change_execution_scene);
  double CalDynamicLatBuffer(const FrenetObstacle &frenet_obstacle);
  bool IsInRange(const FrenetObstacle &frenet_obstacle);
  bool IsAboutToEnterRange(const FrenetObstacle &frenet_obstacle);
  double CalDistanceThre2LaneLine(const FrenetObstacle &frenet_obstacle);
  bool IsAvoidable(const FrenetObstacle &frenet_obstacle,
                   double lat_safety_buffer, bool is_in_lane_change_execution_scene);
  bool HasEnoughNudgeSpace(const FrenetObstacle &frenet_obstacle,
                           double lat_safety_buffer, bool is_in_lane_change_execution_scene,
                           bool is_filter = false);
  double GetAvoidCountThre(const FrenetObstacle &frenet_obstacle);
  bool UpdateObstacleAvoidCount(const FrenetObstacle &frenet_obstacle,
                                bool is_avoidable, bool is_in_range,
                                bool is_about_to_enter_range, double ncar_count,
                                bool rightest_lane, bool is_lane_change);
  void GetPositionRelation(const FrenetObstacle &frenet_obs,
                           LateralObstacleHistoryInfo &history);
  void GetSideCarNudge(const FrenetObstacle& frenet_obs,
                       LateralObstacleHistoryInfo& history,
                       int& side_2_front_count_thr);
  void UpdateLateralObstacleDecisions();
  void LateralObstacleDecision(const FrenetObstacle &frenet_obstacle,
                               bool is_in_lane_change_execution_scene);
  bool ARAStar();
  void LateralObstacleDeciderOutput();
  void Log();
  bool CalculateCutInAndCross(const FrenetObstacle &frenet_obstacle);
  void UpdateLaneBorrowDirection();
  void UpdateIntersection();
  void HoldLatOffset(const FrenetObstacle &frenet_obstacle);
  bool CheckSideObstacle(const FrenetObstacle &frenet_obstacle);
  void CheckObstaclesIsReverse();
  void ConstructUniformPlanHistoryTraj();
  void CheckLateralEmergencyAvoidObstacle(
      const FrenetObstacle &frenet_obstacle);
  bool CheckStaticObstacleAvoidSafety(const FrenetObstacle &frenet_obstacle);
  bool CheckEgoOvertakeObstacle(const FrenetObstacle &frenet_obstacle);
  bool IsTruck(const FrenetObstacle &frenet_obstacle);
  void IsPotentialFollowingObstacle(const FrenetObstacle &frenet_obstacle,
                                    bool is_in_lane_change_execution_scene);
  void CalLaneChangeGapInfo(LcGapInfo &lc_gap_info);
  void ResetObstaclesHistory(bool is_change_lanes);
  void ClearHistoryInfo();
  bool IsCutInIgnore(const FrenetObstacle& frenet_obstacle,
                     bool is_in_lane_change_execution_scene);
  bool CheckSpatioTemporalPlanner();
  void UpdateObstacleInteractionInfo();
  void UpdateObstacleInteractionInfoBaseStaticEnvironment();
  void UpdateObstacleInteractionInfoBaseDynamicEnvironment(
      std::shared_ptr<FrenetObstacle>& front_nearest_follow_obstacle);
  void UpdateLateralObstacleDecisionBaseNearestFollowObstacle(
      std::shared_ptr<FrenetObstacle>& front_nearest_follow_obstacle);
  void UpdateStaticObstacleLateralDecisionBaseStaticFreeSpace(
      std::shared_ptr<FrenetObstacle>& front_nearest_follow_obstacle);
  void UpdateStaticObstacleLateralDecisionBaseDynamicFreeSpace();
  void UpdateObstaclelateraFreeSpaceAndTypeBaseStaticEnvironment(
      const FrenetObstacle& frenet_obstacle,
      ObstacleInfo& obstacle_interaction_info);
  void CalLateralFreeSpaceBaseLaneBoundary(
      const FrenetObstacle& frenet_obstacle,
      ObstacleInfo& obstacle_interaction_info);
  void CalLateralFreeSpaceBaseRoadBoundary(
      const FrenetObstacle& frenet_obstacle,
      ObstacleInfo& obstacle_interaction_info);
  void CalLateralFreeSpaceBaseStaticObstacle(
      const FrenetObstacle& frenet_obstacle,
      ObstacleInfo& obstacle_interaction_info);
  void CalLateralFreeSpaceBaseDynamicObstacle(
      const FrenetObstacle& frenet_obstacle,
      ObstacleInfo& obstacle_interaction_info,
      std::shared_ptr<FrenetObstacle>& front_nearest_follow_obstacle);
  void CheckEgoOverlapDynamicObstacle(
      const FrenetObstacle& frenet_obstacle,
      const FrenetObstacle& target_static_obstacle,
      double& distance_to_centerline,
      std::shared_ptr<FrenetObstacle>& front_nearest_follow_obstacle);
  void UpdateStaticFreeSpaceBaseInteractionType(
      const FrenetObstacle& frenet_obstacle,
      double& free_space);
  void UpdateDynamicFreeSpaceBaseInteractionType(
      const FrenetObstacle& frenet_obstacle,
      double& free_space);
  void GenerateSpatioTemporalFollowDecision(
      const FrenetObstacle& frenet_obstacle);

 private:
  std::unordered_map<uint32_t, LateralObstacleHistoryInfo>
      lateral_obstacle_history_info_;
  std::unordered_map<uint32_t, FollowObstacleInfo> follow_obstacle_info_;
  std::unordered_map<uint32_t, SpatioTemporalFollowObstacleInfo>
      spatio_temporal_follow_obstacle_info_;
  std::unordered_map<uint32_t, LatObstacleDecisionType> last_output_;
  std::vector<int> obstacles_id_behind_ego_;
  // ego info
  double ego_rear_axis_to_front_edge_;
  double ego_rear_axle_to_center_;
  double ego_length_;
  double ego_width_;
  double ego_head_s_ = 0;
  double ego_head_l_ = 0;
  double ego_v_ = 0;
  double ego_v_s_ = 0;
  double ego_v_l_ = 0;
  double ego_rear_edge_to_rear_axle_ = 0;
  bool in_intersection_;
  int intersection_count_ = 0;
  bool left_borrow_;
  bool right_borrow_;
  double lane_width_ = 3.8;
  std::unordered_map<uint32_t, double> obstacle_intrusion_distance_thr_;
  LcGapInfo lc_gap_info_;
  HysteresisDecision side_nudge_release_hysteresis_;
  std::unordered_map<uint32_t, ObstacleInfo> obstacle_interaction_map_;// 用于多障碍之间的交互决策
  int spatio_temporal_planner_intersection_count_ = 0;
  std::unique_ptr<SccHybridARAStar> hybrid_ara_star_ = nullptr;
  SearchResult search_result_;
};

}  // namespace planning