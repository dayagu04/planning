#pragma once

#include <cstdint>
#include <memory>

#include "agent/agent.h"
#include "ego_planning_config.h"
#include "frenet_obstacle.h"
#include "reference_path.h"
#include "tasks/task.h"

namespace planning {

class TurnstileLongitudinalDecider : public Task {
 public:
  // 道闸纵向状态机阶段。
  enum class TurnstileStage {
    IDLE = 0,                    // 无有效道闸目标
    APPROACHING = 1,             // 接近道闸（头车入口态）
    FOLLOW_WAIT = 2,             // 非头车常规跟停
    FOLLOW_WAIT_GATE_CLOSE = 3,  // 非头车等待前车通过后的重开周期
    HEAD_WAIT_CLOSED = 4,        // 头车等待关闸完成
    HEAD_WAIT_REOPEN = 5,        // 头车等待重开
    HEAD_WAIT_FULLY_OPEN = 6,    // 头车等待开闸稳定
    PASSABLE_RELEASE = 7,        // 满足放行条件
    PASSING = 8,                 // 自车已进入道闸区
    PASSED = 9,                  // 自车已通过道闸区
    EMERGENCY_BLOCK = 10,        // 通过中出现风险，触发紧急阻挡
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
    bool emergency_stop_stable = false;  // 应急解除所需的稳定停车条件是否满足

    // reopen 周期相关。
    bool wait_reopen_required = false;  // 当前状态转移是否需要进入 reopen 等待
    bool open_timeout_release_ready = false;  // 是否满足开闸超时放行条件

    // 头车/放行相关。
    bool is_head_car = true;  // 是否为队列头车
    bool head_release_opened_stable = false;  // 头车放行条件（与可通行稳定判定合并）是否满足

    // 闸杆状态相关。
    bool gate_opening_status = false;  // 闸杆是否处于抬杆状态
    bool gate_closed_status = false;  // 闸杆是否处于关闭状态
    bool gate_closing_status = false;  // 闸杆是否处于落杆/接近关闭状态
  };

  // 道闸状态快照（按用户定义语义）。
  struct GateSnapshot {
    iflyauto::GateBarrierStatus status;
    double open_ratio = 0.0;
    bool is_static = false;
    bool is_opening = false;
    bool is_closing = false;
    bool is_closed = false;
    bool is_opened = false;
    bool is_passable = false;
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
    bool turnstile_passable_status_stable = false;  // 用于记录当前帧道闸是否稳定可通行
    bool stop_required = false;  // 用于记录当前帧是否需要生成停车虚拟障碍物
    int32_t stop_virtual_agent_id = agent::AgentDefaultInfo::kNoAgentId;  // 用于记录停车虚拟障碍物 id
    double turnstile_stop_s = 0.0;  // 用于记录当前帧计算得到的停车 s
    double turnstile_s = 0.0;  // 用于记录当前帧道闸前沿 s

    ConstFrenetObstaclePtr target_turnstile_frenet_obs;  // 用于缓存主道闸 frenet obstacle
    const Obstacle* target_turnstile_obs = nullptr;  // 用于缓存主道闸 obstacle
    ConstFrenetObstaclePtr front_vehicle_frenet_obs;  // 用于缓存当前前车 frenet obstacle
    GateSnapshot target_turnstile_gate_snapshot;
  };

  // 周期状态（跨帧保留）。
  struct CycleState {
    int32_t target_turnstile_lost_frame_count = 0;  // 用于累计连续丢失主道闸目标帧数
    bool target_lost_timeout = false;  // 用于标记主道闸目标丢失是否达到超时阈值
    int32_t turnstile_passable_status_stable_frame_count = 0;  // 用于累计可通行状态稳定帧数
    int32_t reopen_open_status_continuous_frame_count = 0;  // 用于累计 reopen 阶段连续开闸帧数

    bool front_car_passed_in_current_cycle = false;  // 用于标记当前周期是否已判定前车通过
    bool wait_reopen_after_front_car_passed = false;  // 用于标记前车通过后当前周期是否需要等待 reopen
    bool release_by_open_timeout = false;  // 用于标记是否由 open-timeout 触发放行
  };

  // 应急状态（跨帧保留）。
  struct EmergencyState {
    int32_t closing_status_drop_consecutive_frame_count = 0;  // 用于累计落杆风险连续帧数
    int32_t emergency_stop_stable_frame_count = 0;  // 用于累计应急解除稳定停车帧数
    bool closing_status_drop_emergency_active = false;  // 用于标记落杆应急阻挡是否激活
    bool emergency_stop_stable = false;  // 用于标记应急解除稳定停车条件是否满足
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
  void UpdateGateSnapshot();
  void UpdateFrontVehicle();
  void UpdateTurnstilePassability();
  void UpdateTurnstileCrossFrameStates();
  void UpdateTurnstileStage();
  void DumpTurnstileDebug() const;

  // 状态机：构造事件、按当前状态求解下一状态、处理进状态副作用。
  TurnstileEventFlags BuildTurnstileEventFlags(
      const FrameContext& frame_ctx, const CycleState& cycle_state,
      const EmergencyState& emergency_state,
      const ReferencePath* reference_path) const;
  TurnstileStage ResolveWaitReopenStage(const TurnstileEventFlags& flags) const;
  TurnstileStage ResolveFollowWaitGateCloseStage(
      const TurnstileEventFlags& flags) const;
  TurnstileStage ResolveHeadCarStage(const TurnstileEventFlags& flags) const;
  TurnstileStage ResolveEmergencyExitStage(const TurnstileEventFlags& flags) const;
  TurnstileStage ResolveNextStageByGraph(const TurnstileEventFlags& flags,
                                         TurnstileStage current_stage) const;
  void ResetStatesOnStageTransition(
      TurnstileStage prev_stage, TurnstileStage new_stage,
      bool wait_reopen_after_front_car_passed);
  void ResetReopenCycleFlags();
  void ResetEmergencyFlags();

  // 虚拟障碍物输出：决定是否停闸以及构造停点。
  bool ShouldCreateVirtualObstacle() const;
  bool AddVirtualObstacle();

  // 道闸/前车目标检索。
  bool IsFrontVehicleNearTurnstile(const FrenetObstacle& front_vehicle_frenet_obs) const;

  // 道闸状态判定。
  GateSnapshot GetGateSnapshot(const Obstacle& turnstile_obs) const;
  bool IsOpenTimeoutReleaseReady(const CycleState& cycle_state) const;
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
