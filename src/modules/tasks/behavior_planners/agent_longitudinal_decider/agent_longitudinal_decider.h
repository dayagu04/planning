#pragma once

#include <deque>
#include <limits>
#include <memory>
#include <unordered_map>
#include <unordered_set>

#include "crossing_agent_decider/crossing_agent_decider.h"
#include "dynamic_world/dynamic_world.h"
#include "ego_state_manager.h"
#include "ego_planning_config.h"
#include "session.h"
#include "tasks/task.h"
#include "utils/kd_path.h"
#include "virtual_lane_manager.h"

namespace planning {

struct AgentHistoryState {
  double timestamp;
  double x;
  double y;
  double vx;
  double vy;
  double speed;
  double theta;
  double s = 0.0;
  double l = 0.0;
  double s_start = 0.0;
  double s_end = 0.0;
  double l_start = 0.0;
  double l_end = 0.0;
  double s_dot = 0.0;
  double l_dot = 0.0;
};
class AgentLongitudinalDecider : public Task {
 public:
  explicit AgentLongitudinalDecider(
      const EgoPlanningConfigBuilder* config_builder,
      framework::Session* session);
  virtual ~AgentLongitudinalDecider() = default;

  bool Execute() override;

  bool Update();

  bool Reset();

  void Init();

  void DeciderCutInAndOutAgents();

  void AddCutInForLaneChange();

  /** Rule base cut in check condition:
   * 1.ego car is not stopped
   * 2.obj is getting closer in lat
   * 3.obj is ahead of ego
   * 4.obj is not too far(far obj doesn't need quick cut-in detection)
   * 5.lat ttc is small & obj vy is obvious & lat dist is short
   * 6.low speed large yaw obj **/
  void DeciderCutInAgent(const bool is_lane_change, const agent::Agent& agent,
                         const double ego_speed_mps, const double ego_theta,
                         const double ego_half_length,
                         const double ego_half_width,
                         const PlanningInitPoint init_point,
                         const double road_curvature_radius,
                         const double cut_in_distance_range_m,
                         const double cut_in_lateral_threshold_m,
                         agent::AgentManager* const mutable_agent_manager,
                         AgentLongitudinalDeciderOutput* output = nullptr);

  void DeciderCutOutAgent(agent::AgentManager* const mutable_agent_manager);

  bool CheckCutOutAgent(const agent::Agent& agent);

  bool HaveCutOutIntention(
      const agent::Agent& agent,
      const std::shared_ptr<planning_math::KDPath>& current_lane_coord,
      const planning_math::PathPoint& agent_matched_point_in_lane,
      const double agent_trajetory_end_point_l_in_lane);

  bool IsLargeAgentCutIn(
      const std::shared_ptr<VirtualLane> ego_lane,
      const std::shared_ptr<planning_math::KDPath>& planned_path,
      const agent::Agent& agent, const double agent_max_s,
      const double cut_in_distance_range_m,
      const double large_agent_lower_small_heading_diff,
      const double ego_half_length, const double ego_s, const double ego_theta,
      const double ego_speed_mps);

  bool CheckSlowLargeAgentCutIn(
      const agent::Agent& agent, const double ego_speed_mps,
      const AgentHistoryState& current_state,
      const std::shared_ptr<planning_math::KDPath>& planned_path,
      const std::shared_ptr<VirtualLane>& ego_lane);

  void CalculateAgentLateralDistance(
      const double object_l_speed_mps, const double min_l, const double max_l,
      const double max_s, const double ego_speed_mps, const double ego_s,
      const double ego_l, double* const ptr_small_lateral_distance,
      double* const ptr_small_lateral_distance_with_ego_l,
      double* const ptr_large_lateral_distance);

  bool IsLargeAgent(const agent::Agent& agent);
  double GetDynamicBoundaryBuffer(double ego_speed_mps, double agent_speed_mps);

  bool IsVruCutIn(const agent::Agent& agent, const double object_l_speed_mps,
                  const double small_lateral_distance, const double max_s,
                  const double ego_s, const double ego_half_length,
                  const std::shared_ptr<planning_math::KDPath>& planned_path);
  bool IsVruCutInWithHistory(
      const agent::Agent& agent,
      const std::shared_ptr<planning_math::KDPath>& planned_path);
  void UpdateAndGetAgentState(
      const agent::Agent& agent, const PlanningInitPoint& init_point,
      const std::shared_ptr<planning_math::KDPath>& ego_lane_coord,
      AgentHistoryState& current_state);

  void UpdateCutInAgentTable();

  // filter agent for st graph
  // 1.rear agent (lane keep)
  // 2.rear agent except target lane rear agent (lane change)
  // 3.ultradistant agent
  // 4.reverse agent
  void FilterRearAgents();

  void FilterUltradistantObs();

  void FilterReverseAgents();
  void FilterReverseAgentsHpp();

  bool IsConsiderBackObs(
      const std::shared_ptr<planning_math::KDPath> planned_path,
      const PlanningInitPoint& init_point, const agent::Agent* agent,
      const double ego_front_s, const double front_corner_s,
      const double ego_center_s, const double front_edge_s_diff,
      const double min_lat_l_from_ego) const;

  bool FilterRearNoCutInAgent(
      const std::shared_ptr<planning_math::KDPath> planned_path,
      const PlanningInitPoint& init_point, const double ego_front_s,
      const double ego_width, const double agent_front_s,
      const agent::Agent* agent, const double min_lat_l_from_ego) const;

  double GetFilterUltraDistanceWithEgoVel(const double ego_vel) const;

  bool IsReverseAgent(const agent::Agent* agent,
                      const std::shared_ptr<VirtualLane> ego_lane,
                      const double consider_distance,
                      bool* is_perception_reverse = nullptr,
                      bool* is_prediction_reverse = nullptr) const;

  bool IsIgnoredLowSpeedReverseAgent(const agent::Agent& agent,
                                     const double init_point_s,
                                     const double init_point_spd,
                                     const double agent_max_s,
                                     const double agent_min_s,
                                     const double agent_max_l,
                                     const double agent_min_l) const;

  double CalculateRoadCurvature(const double v_ego);
  double CalculateRoadCurvatureByKDpath(const double ego_s, const double v_ego);

  double CalculateIntersectionLength(const double start_1, const double end_1,
                                     const double start_2,
                                     const double end_2) const;
  double CalculateMean(const std::vector<double>& values) const;

  double CalculateVariance(const std::vector<double>& values,
                           double mean) const;

  std::deque<AgentHistoryState> GetHistoryInWindow(
      const std::deque<AgentHistoryState>& history,
      double window_duration) const;

 private:
  // HPP对向来车证据结果
  struct ReverseEvidenceResult {
    bool is_perception_reverse = false;        // 感知层对向判断（航向角差>阈值）
    bool is_prediction_reverse = false;        // 预测层对向判断（轨迹终点s<当前s）
    bool is_reverse_current = false;           // 当前帧对向判定（近距离需感知&&预测，远距离感知||预测）
    bool has_valid_lane_projection = false;    // 车道投影是否有效（失败时fail-safe保留障碍物）
    double agent_s_center = 0.0;               // 障碍物相对自车的纵向距离（用于近/中/远分区）
    double agent_min_l = 0.0;                  // 障碍物Frenet坐标系横向最小值
    double agent_max_l = 0.0;                  // 障碍物Frenet坐标系横向最大值
    double lat_gap_to_ego_lane = std::numeric_limits<double>::infinity();  // 障碍物与自车道的最小横向间隙
  };

  ReverseEvidenceResult ComputeHppReverseAgentInfo(
      const agent::Agent& agent, const std::shared_ptr<VirtualLane>& ego_lane,
      double consider_distance, double ego_s) const;
  double CalculateLatGapToEgoLane(double agent_min_l, double agent_max_l,
                                  double half_lane_width) const;
  bool ShouldIgnoreReverseAgentInHpp(
      const ReverseEvidenceResult& reverse_evidence) const;
  void CleanupHppReverseHysteresis(
      const std::unordered_set<int32_t>& active_agent_ids);
  // framework::Session *session_ = nullptr;
  FrenetBoundary ego_frenet_boundary_;
  std::shared_ptr<planning_data::DynamicWorld> dynamic_world_;
  std::shared_ptr<VirtualLaneManager> virtual_lane_manager_;
  std::shared_ptr<EgoStateManager> ego_state_manager_;
  AgentLongitudinalDeciderConfig reverse_filter_config_;

  // cut-in data
  std::unordered_map<int32_t, int32_t> cut_in_agent_count_;
  std::unordered_map<int32_t, int32_t> rule_based_cut_in_agent_count_;
  std::unordered_set<int32_t> current_agent_ids_;
  // cut-out data
  std::unordered_map<int32_t, int32_t> steady_cut_out_agent_count_;
  std::unordered_map<int32_t, int32_t> cut_out_agent_count_;

  std::shared_ptr<CrossingAgentDecider> crossing_agent_decider_;
  std::unordered_map<int32_t, std::deque<AgentHistoryState>> agent_history_map_;

  // HPP reverse ignore hysteresis data
  std::unordered_map<int32_t, int32_t> hpp_reverse_want_ignore_count_;
  std::unordered_map<int32_t, int32_t> hpp_reverse_want_keep_count_;
  std::unordered_map<int32_t, bool> hpp_reverse_ignore_state_;
};
}  // namespace planning
