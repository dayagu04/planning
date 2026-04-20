#pragma once

#include <cstdint>
#include <vector>
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
  double l = 0.0;
  double l_dot = 0.0;
};


struct BayesFeatures {
  std::vector<double> norm_l_dot;
  std::vector<double> hist_lateral_dist;
  std::vector<double> pred_norm_vy;
  std::vector<double> pred_lateral_dist;
  bool valid = false;
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

  void ProcessCutInCutOutAgent(const bool is_in_lane_change,
                               const agent::Agent& agent,
                               const double ego_speed_mps,
                               const double ego_half_length,
                               const double ego_half_width,
                               const PlanningInitPoint init_point,
                               const double lateral_distance_range,
                               agent::AgentManager* const mutable_agent_manager,
                               AgentLongitudinalDeciderOutput* output = nullptr);

  void CalculateAgentLateralDistance(
      const double object_l_speed_mps, const double min_l, const double max_l,
      const double max_s, const double ego_speed_mps, const double ego_s,
      const double ego_l, double* const ptr_small_lateral_distance,
      double* const ptr_small_lateral_distance_with_ego_l,
      double* const ptr_large_lateral_distance);

  void UpdateAndGetAgentState(
      const agent::Agent& agent, const PlanningInitPoint& init_point,
      const std::shared_ptr<planning_math::KDPath>& ego_lane_coord,
      AgentHistoryState& current_state);

  void UpdateAgentTable();

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

  double CalculateIntersectionLength(const double start_1, const double end_1,
                                     const double start_2,
                                     const double end_2) const;

  BayesFeatures ExtractBayesFeatures(
      int32_t agent_id, const agent::Agent& agent,
      const std::shared_ptr<planning_math::KDPath>& ego_lane_coord) const;

  void UpdateBayesianPosteriors(int32_t agent_id, double log_L_hist_cutin,
                                double log_L_hist_cutout,
                                double log_L_hist_normal,
                                double log_L_pred_cutin,
                                double log_L_pred_cutout,
                                double log_L_pred_normal,
                                double* cutin_posterior,
                                double* cutout_posterior);

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

  std::unordered_map<int32_t, int32_t> cut_in_agent_count_;
  std::unordered_map<int32_t, int32_t> cut_out_agent_count_;
  std::unordered_set<int32_t> current_agent_ids_;
  std::unordered_set<int32_t> processed_cut_in_agent_ids_;
  std::unordered_set<int32_t> processed_cut_out_agent_ids_;

  std::shared_ptr<CrossingAgentDecider> crossing_agent_decider_;
  std::unordered_map<int32_t, std::deque<AgentHistoryState>> agent_history_map_;
  std::unordered_map<int32_t, double> bayes_cutin_posterior_;
  std::unordered_map<int32_t, double> bayes_cutout_posterior_;

  // HPP reverse ignore hysteresis data
  std::unordered_map<int32_t, int32_t> hpp_reverse_want_ignore_count_;
  std::unordered_map<int32_t, int32_t> hpp_reverse_want_keep_count_;
  std::unordered_map<int32_t, bool> hpp_reverse_ignore_state_;
};
}  // namespace planning
