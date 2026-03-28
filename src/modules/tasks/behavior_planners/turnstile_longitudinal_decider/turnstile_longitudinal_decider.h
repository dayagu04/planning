#pragma once

#include <cstdint>
#include <memory>

#include "agent/agent.h"
#include "ego_planning_config.h"
#include "reference_path.h"
#include "tasks/task.h"

namespace planning {

class TurnstileLongitudinalDecider : public Task {
 public:
  // 道闸纵向状态机阶段。
  enum class TurnstileStage {
    IDLE = 0,                     // 无有效道闸目标
    APPROACHING = 1,              // 头车接近道闸
    FOLLOW_WAIT = 2,              // 非头车常规跟停
    FOLLOW_WAIT_GATE_CLOSE = 3,   // 非头车等待道闸先完成一次关闭
    HEAD_WAIT_CLOSED = 4,         // 头车等待闸杆完全落下
    HEAD_WAIT_REOPEN = 5,         // 头车等待重新抬杆
    HEAD_WAIT_FULLY_OPEN = 6,     // 头车等待开闸稳定
    PASSABLE_RELEASE = 7,         // 满足放行条件
    PASSING = 8,                  // 自车已进入道闸区
    PASSED = 9,                   // 自车已通过道闸区
    FOLLOW_WAIT_OPEN_TIMEOUT = 10,  // 非头车按 open-timeout 兜底等待
    EMERGENCY_BLOCK = 11,         // 通过中出现落杆风险，触发紧急阻挡
  };

  TurnstileLongitudinalDecider(const EgoPlanningConfigBuilder* config_builder,
                               framework::Session* session);
  ~TurnstileLongitudinalDecider() override = default;

  bool Execute() override;

 private:
  // 单帧内用于状态机决策的事件快照（按决策流程顺序组织）。
  struct TurnstileEventFlags {
    // 目标有效性。
    bool has_target_turnstile = false;  // 是否存在有效道闸目标
    bool target_lost_timeout = false;   // 目标丢失是否达到超时阈值

    // 自车相对道闸位置。
    bool ego_passed = false;  // 自车是否已通过道闸
    bool ego_in_gate = false;  // 自车是否已进入道闸区域

    // 应急相关。
    bool emergency_active = false;  // 是否触发落杆应急阻挡
    bool emergency_opening_status_stable = false;  // 应急后抬杆状态是否达到稳定阈值

    // reopen 周期相关。
    bool wait_reopen_required = false;  // 是否需要等待“先关后开”
    bool has_seen_gate_closed_status_after_front_car_pass = false;  // 前车通过后是否已观测到关闸
    bool reopen_completed = false;  // reopen 流程是否完成
    bool reopen_timeout_halfway = false;  // open-timeout 是否超过 50%

    // 头车/放行相关。
    bool is_head_car = true;  // 是否为队列头车
    bool turnstile_passable_status = false;  // 道闸当前帧是否可通行
    bool passable_status_stable = false;  // 可通行状态是否达到稳定阈值

    // 闸杆状态相关。
    bool gate_opening_status = false;  // 闸杆是否处于抬杆状态
    bool gate_closed_status = false;  // 闸杆是否处于关闭状态
    bool gate_closing_status = false;  // 闸杆是否处于落杆/接近关闭状态
  };

  // 道闸状态快照，用于减少重复状态判定分支。
  struct GateSnapshot {
    iflyauto::GateBarrierStatus status;
    double open_ratio;
    bool is_unknown_or_close = false;
    bool is_static = false;
    bool is_opening = false;
  };

  // 当前帧上下文（每帧都会重置）。
  struct FrameContext {
    bool has_target_turnstile = false;  // 用于标记当前帧是否有有效主道闸目标
    int32_t target_turnstile_obs_id = agent::AgentDefaultInfo::kNoAgentId;  // 用于记录主道闸 obs id
    int32_t side_turnstile_obs_id = agent::AgentDefaultInfo::kNoAgentId;  // 用于记录侧道闸 obs id（调试）
    TurnstileSceneType turnstile_scene_type = TurnstileSceneType::TURNSTILE_SCENE_NONE;  // 用于记录参考线给出的道闸场景类型
    bool is_head_car = true;  // 用于标记当前是否判定为头车
    int32_t front_car_id = agent::AgentDefaultInfo::kNoAgentId;  // 用于记录当前前车 id
    bool turnstile_passable_status = false;  // 用于记录当前帧道闸是否可通行
    bool stop_required = false;  // 用于记录当前帧是否需要生成停车虚拟障碍物
    int32_t stop_virtual_agent_id = agent::AgentDefaultInfo::kNoAgentId;  // 用于记录停车虚拟障碍物 id
    double turnstile_stop_s = 0.0;  // 用于记录当前帧计算得到的停车 s
    double turnstile_s = 0.0;  // 用于记录当前帧道闸前沿 s

    const FrenetObstacle* target_turnstile_frenet_obs = nullptr;  // 用于缓存主道闸 frenet obstacle
    const Obstacle* target_turnstile_obs = nullptr;  // 用于缓存主道闸 obstacle
    const FrenetObstacle* front_vehicle_frenet_obs = nullptr;  // 用于缓存当前前车 frenet obstacle
  };

  // 周期状态（跨帧保留）。
  struct CycleState {
    int32_t target_turnstile_lost_frame_count = 0;  // 用于累计连续丢失主道闸目标帧数
    int32_t turnstile_passable_status_stable_frame_count = 0;  // 用于累计可通行状态稳定帧数
    int32_t cycle_closing_status_stable_frame_count = 0;  // 用于累计“先关后开”中的关闸稳定帧数
    int32_t reopen_open_status_continuous_frame_count = 0;  // 用于累计 reopen 阶段连续开闸帧数

    bool front_car_passed_in_current_cycle = false;  // 用于标记当前周期是否已判定前车通过
    bool wait_reopen_required = false;  // 用于标记当前周期是否需要等待 reopen
    bool has_seen_gate_closed_status_after_front_car_pass = false;  // 用于标记前车通过后是否已观测到关闸
    bool release_by_open_timeout = false;  // 用于标记是否由 open-timeout 触发放行
  };

  // 应急状态（跨帧保留）。
  struct EmergencyState {
    int32_t closing_status_drop_consecutive_frame_count = 0;  // 用于累计落杆风险连续帧数
    int32_t emergency_opening_status_stable_frame_count = 0;  // 用于累计应急后开闸稳定帧数
    bool closing_status_drop_emergency_active = false;  // 用于标记落杆应急阻挡是否激活
  };

  // 历史状态（跨帧保留）。
  struct HistoryState {
    int32_t previous_front_vehicle_id = agent::AgentDefaultInfo::kNoAgentId;  // 用于记录上一帧前车 id
    double previous_front_vehicle_s = 0.0;  // 用于记录上一帧前车尾部 s_end
    bool had_valid_front_vehicle_in_prev_frame = false;  // 用于标记上一帧是否存在有效前车
    bool was_turnstile_passable_status_in_prev_frame = false;  // 用于标记上一帧道闸是否可通行
  };

  // 单帧主流程：感知输入更新 + 状态机更新。
  bool IsCurrentReferencePathValid();
  void ResetStateWhenDeciderInactive();
  void InitFrameContextFromReferencePath();
  void UpdateTargetTurnstile();
  void UpdateFrontVehicle();
  void UpdateTurnstilePassability();
  void UpdateTurnstileCycleState(const FrameContext& frame_ctx,
                                 TurnstileStage current_stage,
                                 CycleState& cycle_state,
                                 EmergencyState& emergency_state,
                                 HistoryState& history_state);
  void UpdateReopenCycleState(const FrameContext& frame_ctx,
                              const HistoryState& history_state,
                              CycleState& cycle_state);
  void UpdateEmergencyState(const FrameContext& frame_ctx,
                            TurnstileStage current_stage,
                            EmergencyState& emergency_state);
  void UpdateHistoryFromCurrentFrame(const FrameContext& frame_ctx,
                                     HistoryState& history_state);
  void UpdateTurnstileStage();
  void DumpTurnstileDebug() const;

  // 状态机：构造事件、求解下一状态、处理进状态副作用。
  TurnstileEventFlags BuildTurnstileEventFlags(
      const FrameContext& frame_ctx, const CycleState& cycle_state,
      const EmergencyState& emergency_state,
      const ReferencePath* reference_path) const;
  TurnstileStage ResolveWaitReopenStage(const TurnstileEventFlags& flags) const;
  TurnstileStage ResolveHeadCarStage(const TurnstileEventFlags& flags) const;
  TurnstileStage ResolveEmergencyExitStage(const TurnstileEventFlags& flags) const;
  TurnstileStage ResolveNextStage(const TurnstileEventFlags& flags,
                                  TurnstileStage current_stage) const;
  void OnEnterStage(TurnstileStage prev_stage, TurnstileStage new_stage,
                    bool wait_reopen_required);
  void ResetReopenCycleFlags();
  void ResetEmergencyFlags();

  // 虚拟障碍物输出：决定是否停闸以及构造停点。
  bool ShouldCreateVirtualObstacle() const;
  bool AddVirtualObstacle();

  // 道闸/前车目标检索。
  bool IsFrontVehicleNearTurnstile(const FrenetObstacle& front_vehicle_frenet_obs) const;

  // 道闸状态判定。
  GateSnapshot GetGateSnapshot(const Obstacle& turnstile_obs) const;
  bool IsTurnstileInPassableStatus(const Obstacle& turnstile_obs) const;
  bool IsTurnstileInCycleClosingStatus(const Obstacle& turnstile_obs) const;
  bool IsTurnstileDroppedForEmergency(const Obstacle& turnstile_obs) const;
  bool HasCompletedReopenCycle(const CycleState& cycle_state) const;
  double GetEffectiveTurnstileDtSec() const;
  double ComputeTurnstileStopS(const FrenetObstacle& turnstile_obs) const;

 private:
  const LongitudinalDeciderV3Config lon_config_;  // 用于配置道闸相关阈值与开关
  std::shared_ptr<ReferencePath> reference_path_;  // 用于保存当前执行的参考线

  TurnstileStage stage_ = TurnstileStage::IDLE;  // 用于记录当前状态机阶段
  FrameContext frame_ctx_;
  CycleState cycle_state_;
  EmergencyState emergency_state_;
  HistoryState history_state_;
};

}  // namespace planning
