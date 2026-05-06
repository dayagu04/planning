#pragma once

#include <cstdint>
#include <deque>
#include <map>
#include <memory>
#include <set>
#include <vector>

#include "agent/agent.h"
#include "agent/agent_manager.h"
#include "ego_planning_config.h"
#include "ego_state_manager.h"
#include "session.h"
#include "tasks/task.h"
#include "utils/kd_path.h"
#include "virtual_lane_manager.h"
namespace planning {
namespace longitudinal_intention {

struct AgentHistoryState {
  double s = 0.0;
  double l = 0.0;
  double x = 0.0;
  double y = 0.0;
  double timestamp = 0.0;
  //新增状态
  double prediction_length = 0.0;
  double vel_prediction = 0.0; // 预测长度/时间 1帧出
  double acc_prediction = 0.0; // 需要两帧
  double speed = 0.0;
  double accel_fusion = 0.0;  // 感知融合加速度 1帧出
  double accel_sp = 0.0;// diff speed 计算需要两帧
};

struct LongitudinalBayesFeatures {
  bool valid = false;
  // 三种加速度特征: accel_fusion, accel_vel(by差速), accel_pred(by预测长度)
  std::vector<double> accel_fusion_vec;
  std::vector<double> accel_vel_vec;
  std::vector<double> accel_pred_vec;
};

enum class LongitudinalIntent {
  DECEL = 0,
  CRUISE = 1,
  ACCEL = 2,
  UNKNOWN = 3
};

struct LongitudinalIntentResult {
  LongitudinalIntent intent = LongitudinalIntent::UNKNOWN;
  double decel_prob = 0.0;
  double cruise_prob = 0.0;
  double accel_prob = 0.0;
  double confidence = 0.0;
  double unknown_prob = 1.0;
};

class IntentionDecider : public Task {
 public:
  explicit IntentionDecider(const EgoPlanningConfigBuilder* config_builder,
                            framework::Session* session);
  ~IntentionDecider() = default;

  bool Execute() override;

  void DecideLongitudinalIntent(
      const agent::AgentManager& agent_manager,
      const PlanningInitPoint& init_point,
      const std::shared_ptr<planning_math::KDPath>& ego_lane_coord,
      agent::AgentManager* const mutable_agent_manager,
      double ego_s,
      double ego_l);

  void UpdateAgentTable(const agent::AgentManager& agent_manager, double ego_s);

 private:
  void UpdateAndGetAgentState(
      const agent::Agent& agent,
      const PlanningInitPoint& init_point,
      const std::shared_ptr<planning_math::KDPath>& ego_lane_coord,
      AgentHistoryState& current_state);

  LongitudinalBayesFeatures ExtractLongitudinalBayesFeatures(
      const int32_t agent_id,
      const agent::Agent& agent,
      const std::shared_ptr<planning_math::KDPath>& ego_lane_coord);

  LongitudinalIntentResult CalculateLongitudinalIntent(
      const int32_t agent_id,
      const agent::Agent& agent,
      const LongitudinalBayesFeatures& features);

  // Session and managers
  std::shared_ptr<VirtualLaneManager> virtual_lane_manager_;
  std::shared_ptr<EgoStateManager> ego_state_manager_;

  // Configuration
  LongitudinalIntentionConfig config_;

  // Agent history tracking
  std::map<int32_t, std::deque<AgentHistoryState>> agent_history_map_;
  std::set<int32_t> current_agent_ids_;
  std::set<int32_t> processed_lon_intent_agent_ids_;

  // Bayesian posterior tracking
  std::map<int32_t, double> lon_decel_posterior_;
  std::map<int32_t, double> lon_cruise_posterior_;
  std::map<int32_t, double> lon_accel_posterior_;
  std::map<int32_t, int> lon_intent_map_;
};

}  // namespace longitudinal_intention
}  // namespace planning
