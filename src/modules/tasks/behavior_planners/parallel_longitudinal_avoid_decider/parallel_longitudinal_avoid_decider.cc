#include "parallel_longitudinal_avoid_decider.h"

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>

#include "dynamic_world/dynamic_agent_node.h"
#include "environmental_model.h"
#include "log.h"
#include "planning_context.h"
#include "vehicle_config_context.h"

namespace planning {
namespace {

constexpr double kMpsToKmh = 3.6;                   // 米/秒转公里/小时
constexpr double kTruckAvoidVelocityMinKmh = 30.0;  // 并行避让最小速度阈值
constexpr double kTruckAvoidVelocityMaxKmh = 120.0;  // 并行避让最大速度阈值
constexpr double kTruckYieldAvoidVelocityMinKmh = 18.0;  // 让行避让最小速度阈值
constexpr double kVelocityEgoHysteresisThresholdKmh = 5.0;  // 速度滞后阈值
constexpr double kReverseHeadingThresholdRad = 2.09;  // 逆行航向角阈值
constexpr int32_t kExitConditionSatisfiedFrameThreshold =
    10;  // 退出条件满足帧数阈值(1秒)
constexpr int32_t kMaxRunningFrameThreshold = 50;  // 最大运行帧数阈值（5秒）
constexpr int32_t kCooldownFrameThreshold = 20;  // 冷却帧数阈值（2秒）
constexpr double kMaxSpeedForOvertakeTruck = 35.0;  // 超车最大速度
constexpr double kMinSpeedForYieldTruck = 2.0;      // 让行最小速度
constexpr double kOvertakeStartSpeedDiffThresholdKmh =
    7.2;  // 超车启动速度差阈值
constexpr double kOvertakeStopSpeedDiffThresholdKmh =
    10.8;  // 超车退出速度差阈值
constexpr double kStartYieldSpeedDiffThresholdKmh = 7.2;  // 让行启动速度差阈值
constexpr double kStopYieldSpeedDiffThresholdKmh = 0.5;  // 让行退出速度差阈值
constexpr double kOvertakeDistanceM = 10.0;              // 超车距离
constexpr double kYieldDistanceM = 8.0;                  // 让行距离
constexpr double kMaxDistanceM = 40.0;                   // 最大距离
constexpr double kExitMaxDistanceM = 42.0;               // 最大退出距离
constexpr double kOvertakeDistanceThresholdM = 20.0;  // 超车距离阈值
constexpr double kOvertakeExitDistanceThresholdM = 0.5;  // 超车退出距离阈值
constexpr double kLeadTargetTrucksBufferM = 4.5;  // 让行最小缓冲距离阈值
constexpr double kLongitudinalTTCStartThresholdS =
    2.5;  // 纵向TTC启动阈值（秒）
constexpr double kLongitudinalTTCStopThresholdS = 3.0;  // 纵向TTC退出阈值（秒）
constexpr double kStartYieldLateralTTCThresholdS =
    2.5;  // 让行启动横向TTC阈值（秒）
constexpr double kStopYieldLateralTTCThresholdS =
    3.0;  // 让行退出横向TTC阈值（秒）
constexpr double kLeadTargetTrucksStartTimeHeadwayS =
    0.8;  // 前车大车与侧方大车启动时距
constexpr double kLeadTargetTrucksExitTimeHeadwayS =
    1.0;  // 前车大车与侧方大车退出时距
constexpr double kLeadTargetStartTimeHeadwayS = 1.2;  // 前车与自车启动时距阈值
constexpr double kLeadTargetExitTimeHeadwayS = 1.5;  // 前车与自车退出时距阈值
constexpr double kStartTargetFrontAgentStartTimeHeadwayS =
    1.5;  // 超前前方障碍物启动时距阈值（秒）
constexpr double kStopTargetFrontAgentExitTimeHeadwayS =
    1.8;  // 超前前方障碍物退出时距阈值（秒）
constexpr double kAgentAppearanceTimeThresholdS =
    2.5;                                     // 障碍物出现ST图时间阈值
constexpr double kVehicleLengthRatio = 0.5;  // 车辆长度比例
constexpr double kStartYieldLateralConflictRatio =
    0.15;  // 让行启动横向冲突比例
constexpr double kStopYieldLateralConflictRatio = 0.20;  // 让行退出横向冲突比例
constexpr double kStartLateralConflictRatio =
    0.25;  // 超车和前车让行启动横向冲突比例
constexpr double kStopLateralConflictRatio =
    0.30;  // 超车和前车让行退出横向冲突比例
constexpr double kCloseDistanceTimeHeadwayS = 1.5;  // 近距离考虑时距
constexpr double kDefaultCipvDistanceM = 200.0;  // 默认前车距离（米）
constexpr double kLargeAgentLengthM = 8.0;       // 大车长度（米）
}  // namespace

ParallelLongitudinalAvoidDecider::ParallelLongitudinalAvoidDecider(
    const EgoPlanningConfigBuilder* config_builder, framework::Session* session)
    : config_builder_(config_builder), session_(session) {
  name_ = "ParallelLongitudinalAvoidDecider";
  const auto& ego_vehicle_param =
      VehicleConfigurationContext::Instance()->get_vehicle_param();
  front_edge_to_rear_axle_ = ego_vehicle_param.front_edge_to_rear_axle;
  rear_edge_to_rear_axle_ = ego_vehicle_param.rear_edge_to_rear_axle;
  current_state_ = DeciderState::IDLE;
  running_frame_count_ = 0;
  cooldown_frame_count_ = 0;
  exit_condition_frame_count_ = 0;
  is_need_overtake_ = false;
  is_need_yield_ = false;
  is_lead_and_target_is_truck_ = false;
  is_running_longitudinal_avoid_ = false;
  is_right_agent_ = false;
  parallel_target_agent_id_ = planning_data::kInvalidId;
  start_running_agent_id_ = planning_data::kInvalidId;
  lateral_distance_ = -1.0;
  is_satisfied_speed_range_ = false;
}

bool ParallelLongitudinalAvoidDecider::Execute() {
  ProcessStateMachine();
  return true;
}

void ParallelLongitudinalAvoidDecider::ResetInnerParam() {
  parallel_target_agent_id_ = planning_data::kInvalidId;
  start_running_agent_id_ = planning_data::kInvalidId;
  is_need_overtake_ = false;
  is_need_yield_ = false;
  is_lead_and_target_is_truck_ = false;
  is_running_longitudinal_avoid_ = false;
  lateral_distance_ = -1.0;
  exit_condition_frame_count_ = 0;
  running_frame_count_ = 0;
  cooldown_frame_count_ = 0;
  is_right_agent_ = false;
  is_satisfied_speed_range_ = false;
}

void ParallelLongitudinalAvoidDecider::TransitionToState(
    DeciderState new_state) {
  switch (new_state) {
    case DeciderState::IDLE:
      exit_condition_frame_count_ = 0;
      running_frame_count_ = 0;
      cooldown_frame_count_ = 0;
      parallel_target_agent_id_ = planning_data::kInvalidId;
      start_running_agent_id_ = planning_data::kInvalidId;
      lateral_distance_ = -1.0;
      break;
    case DeciderState::RUNNING:
      running_frame_count_ = 0;
      break;
    case DeciderState::EXITING:
      running_frame_count_ = 0;
      exit_condition_frame_count_ = 0;
      break;
    case DeciderState::COOLDOWN:
      cooldown_frame_count_ = 0;
      parallel_target_agent_id_ = planning_data::kInvalidId;
      start_running_agent_id_ = planning_data::kInvalidId;
      break;
    default:
      return;
  }

  current_state_ = new_state;
}

void ParallelLongitudinalAvoidDecider::ProcessStateMachine() {
  if (!CheckIfTheTruckIsParallel()) {
    if (current_state_ == DeciderState::IDLE) {
      parallel_target_agent_id_ = planning_data::kInvalidId;
      OutputRunning();
      return;
    } else if (current_state_ == DeciderState::COOLDOWN) {
      if (cooldown_frame_count_ < kCooldownFrameThreshold) {
        ++cooldown_frame_count_;
      } else {
        TransitionToState(DeciderState::IDLE);
      }
      OutputRunning();
      return;
    } else {
      TransitionToState(DeciderState::COOLDOWN);
      ResetInnerParam();
      OutputRunning();
      return;
    }
  }

  switch (current_state_) {
    case DeciderState::IDLE:
      if (IsStartRunning()) {
        TransitionToState(DeciderState::RUNNING);
      } else {
        parallel_target_agent_id_ = planning_data::kInvalidId;
      }
      break;

    case DeciderState::RUNNING:
      if (IsStopRunning()) {
        TransitionToState(DeciderState::EXITING);
      } else {
        ++running_frame_count_;
      }
      break;

    case DeciderState::EXITING:
      ResetInnerParam();
      TransitionToState(DeciderState::COOLDOWN);
      break;

    case DeciderState::COOLDOWN:
      if (cooldown_frame_count_ < kCooldownFrameThreshold) {
        ++cooldown_frame_count_;
      } else {
        TransitionToState(DeciderState::IDLE);
      }
      break;

    default:
      TransitionToState(DeciderState::IDLE);
      break;
  }

  OutputRunning();
}

bool ParallelLongitudinalAvoidDecider::IsStartRunning() {
  const auto& environmental_model = session_->environmental_model();
  const agent::Agent* target_agent =
      environmental_model.get_dynamic_world()->agent_manager()->GetAgent(
          parallel_target_agent_id_);

  is_need_overtake_ = CheckIfNeedOvertake(target_agent);
  if (!is_need_overtake_) {
    is_need_yield_ = CheckIfNeedYield(target_agent);
  }

  if (!is_need_overtake_ && !is_need_yield_) {
    is_lead_and_target_is_truck_ = CheckLeadAndTargetIsTruck(target_agent);
  }

  if (!is_need_overtake_ && !is_need_yield_ && !is_lead_and_target_is_truck_) {
    return false;
  }

  start_running_agent_id_ = parallel_target_agent_id_;
  is_running_longitudinal_avoid_ = true;
  return true;
}

bool ParallelLongitudinalAvoidDecider::IsStopRunning() {
  if (running_frame_count_ >= kMaxRunningFrameThreshold) {
    return true;
  }

  if (parallel_target_agent_id_ != start_running_agent_id_) {
    return true;
  }

  const auto* target_agent = session_->environmental_model()
                                 .get_dynamic_world()
                                 ->agent_manager()
                                 ->GetAgent(parallel_target_agent_id_);
  if (target_agent == nullptr) return true;

  const double ego_speed_kmh = session_->environmental_model()
                                   .get_ego_state_manager()
                                   ->planning_init_point()
                                   .v *
                               kMpsToKmh;
  const auto* st_graph_helper = session_->planning_context().st_graph_helper();

  bool exit_condition_satisfied = false;

  if (is_need_overtake_) {
    exit_condition_satisfied = IsSatisfiedOvertakeExitCondition(
        ego_speed_kmh, st_graph_helper, target_agent,
        parallel_target_agent_id_);
  } else if (is_need_yield_) {
    exit_condition_satisfied =
        IsSatisfiedYieldExitCondition(ego_speed_kmh, st_graph_helper,
                                      target_agent, parallel_target_agent_id_);
  } else if (is_lead_and_target_is_truck_) {
    exit_condition_satisfied =
        IsSatisfiedLeadAndTargetIsTruckExitCondition(target_agent);
  } else {
    return false;
  }

  if (exit_condition_satisfied) {
    return ++exit_condition_frame_count_ >=
           kExitConditionSatisfiedFrameThreshold;
  } else {
    exit_condition_frame_count_ = 0;
    return false;
  }
}

void ParallelLongitudinalAvoidDecider::SelectNearestNeighborAgent(
    const int64_t left_front_node_id, const int64_t left_node_id,
    const int64_t right_front_node_id, const int64_t right_node_id,
    int64_t* target_node_id, int64_t* alter_target_node_id) {
  const auto& dynamic_world =
      session_->environmental_model().get_dynamic_world();
  const auto* agent_manager = dynamic_world->agent_manager();

  // 1.获取自车位置信息
  const auto& ego_init_point = session_->environmental_model()
                                   .get_ego_state_manager()
                                   ->planning_init_point();
  const auto& ego_lane = session_->environmental_model()
                             .get_virtual_lane_manager()
                             ->get_current_lane();
  if (ego_lane == nullptr || ego_lane->get_lane_frenet_coord() == nullptr) {
    *target_node_id = planning_data::kInvalidId;
    *alter_target_node_id = planning_data::kInvalidId;
    return;
  }
  const auto& ego_lane_coord = ego_lane->get_lane_frenet_coord();
  double ego_s = 0.0, ego_l = 0.0;
  if (!ego_lane_coord->XYToSL(ego_init_point.x, ego_init_point.y, &ego_s,
                              &ego_l)) {
    *target_node_id = planning_data::kInvalidId;
    *alter_target_node_id = planning_data::kInvalidId;
    return;
  }

  // 2.收集所有候选障碍物及其信息
  struct CandidateInfo {
    int64_t node_id;
    double distance;
    bool is_front;
    bool is_right;
  };
  std::vector<CandidateInfo> candidates;

  // 3.检查左邻车道障碍物
  if (left_node_id != planning_data::kInvalidId) {
    const auto* left_node = dynamic_world->GetNode(left_node_id);
    if (left_node != nullptr) {
      const int64_t left_agent_id = left_node->node_agent_id();
      const auto* left_agent = agent_manager->GetAgent(left_agent_id);
      if (left_agent != nullptr) {
        double left_agent_s = 0.0, left_agent_l = 0.0;
        if (ego_lane_coord->XYToSL(left_agent->box().center().x(),
                                   left_agent->box().center().y(),
                                   &left_agent_s, &left_agent_l)) {
          double left_distance =
              std::max(left_agent_s - ego_s - front_edge_to_rear_axle_ -
                           left_agent->length() * kVehicleLengthRatio,
                       0.0);
          candidates.push_back({left_node_id, left_distance, false, false});
        }
      }
    }
  }

  // 4.检查左前邻车道障碍物
  if (left_front_node_id != planning_data::kInvalidId) {
    const auto* left_front_node = dynamic_world->GetNode(left_front_node_id);
    if (left_front_node != nullptr) {
      const int64_t left_front_agent_id = left_front_node->node_agent_id();
      const auto* left_front_agent =
          agent_manager->GetAgent(left_front_agent_id);
      if (left_front_agent != nullptr) {
        double left_front_agent_s = 0.0, left_front_agent_l = 0.0;
        if (ego_lane_coord->XYToSL(left_front_agent->box().center().x(),
                                   left_front_agent->box().center().y(),
                                   &left_front_agent_s, &left_front_agent_l)) {
          double left_front_distance =
              std::max(left_front_agent_s - ego_s - front_edge_to_rear_axle_ -
                           left_front_agent->length() * kVehicleLengthRatio,
                       0.0);
          candidates.push_back(
              {left_front_node_id, left_front_distance, true, false});
        }
      }
    }
  }

  // 5.检查右邻车道障碍物
  if (right_node_id != planning_data::kInvalidId) {
    const auto* right_node = dynamic_world->GetNode(right_node_id);
    if (right_node != nullptr) {
      const int64_t right_agent_id = right_node->node_agent_id();
      const auto* right_agent = agent_manager->GetAgent(right_agent_id);
      if (right_agent != nullptr) {
        double right_agent_s = 0.0, right_agent_l = 0.0;
        if (ego_lane_coord->XYToSL(right_agent->box().center().x(),
                                   right_agent->box().center().y(),
                                   &right_agent_s, &right_agent_l)) {
          double right_distance =
              std::max(right_agent_s - ego_s - front_edge_to_rear_axle_ -
                           right_agent->length() * kVehicleLengthRatio,
                       0.0);
          candidates.push_back({right_node_id, right_distance, false, true});
        }
      }
    }
  }

  // 6.检查右前邻车道障碍物
  if (right_front_node_id != planning_data::kInvalidId) {
    const auto* right_front_node = dynamic_world->GetNode(right_front_node_id);
    if (right_front_node != nullptr) {
      const int64_t right_front_agent_id = right_front_node->node_agent_id();
      const auto* right_front_agent =
          agent_manager->GetAgent(right_front_agent_id);
      if (right_front_agent != nullptr) {
        double right_front_agent_s = 0.0, right_front_agent_l = 0.0;
        if (ego_lane_coord->XYToSL(right_front_agent->box().center().x(),
                                   right_front_agent->box().center().y(),
                                   &right_front_agent_s,
                                   &right_front_agent_l)) {
          double right_front_distance =
              std::max(right_front_agent_s - ego_s - front_edge_to_rear_axle_ -
                           right_front_agent->length() * kVehicleLengthRatio,
                       0.0);
          candidates.push_back(
              {right_front_node_id, right_front_distance, true, true});
        }
      }
    }
  }

  if (candidates.empty()) {
    *target_node_id = planning_data::kInvalidId;
    *alter_target_node_id = planning_data::kInvalidId;
    return;
  }

  // 7.按优先级排序：纵向距离 > 邻车道 > 邻前车道
  std::sort(candidates.begin(), candidates.end(),
            [](const CandidateInfo& a, const CandidateInfo& b) {
              if (std::fabs(a.distance - b.distance) > 1e-6) {
                return a.distance < b.distance;
              }
              if (a.is_front != b.is_front) {
                return !a.is_front;
              }
              return a.is_right;
            });

  // 8.设置目标障碍物
  *target_node_id = candidates[0].node_id;
  is_right_agent_ = candidates[0].is_right;

  // 9.设置备选障碍物
  if (candidates.size() > 1) {
    *alter_target_node_id = candidates[1].node_id;
  } else {
    *alter_target_node_id = planning_data::kInvalidId;
  }
}

bool ParallelLongitudinalAvoidDecider::IsDynamicTruckOrBus(
    const agent::Agent* agent) {
  return agent &&
         (agent->type() == agent::AgentType::TRUCK ||
          agent->type() == agent::AgentType::BUS ||
          agent->length() > kLargeAgentLengthM) &&
         !agent->is_static();
}

bool ParallelLongitudinalAvoidDecider::ConstructNeighborLaneStGraph(
    const agent::Agent* const truck_agent) {
  if (nullptr == truck_agent) {
    return false;
  }
  auto* mutable_st_graph = session_->planning_context().st_graph();
  return mutable_st_graph->InsertAgent(*truck_agent,
                                       speed::StBoundaryType::NEIGHBOR);
}

bool ParallelLongitudinalAvoidDecider::IsSpeedInRange(
    const agent::Agent* agent, const double min_speed_kmh,
    const double max_speed_kmh) {
  if (agent == nullptr) return false;
  const double agent_speed_kmh = agent->speed() * kMpsToKmh;

  bool is_in_speed_range =
      agent_speed_kmh >= min_speed_kmh && agent_speed_kmh <= max_speed_kmh;

  if (is_in_speed_range) {
    is_satisfied_speed_range_ = true;
  } else if (is_satisfied_speed_range_) {
    bool is_in_speed_low_hysteresis_range =
        agent_speed_kmh < (min_speed_kmh - kVelocityEgoHysteresisThresholdKmh);
    bool is_in_speed_high_hysteresis_range =
        agent_speed_kmh > (max_speed_kmh + kVelocityEgoHysteresisThresholdKmh);
    if (is_in_speed_low_hysteresis_range || is_in_speed_high_hysteresis_range) {
      is_satisfied_speed_range_ = false;
    }
  } else {
    is_satisfied_speed_range_ = false;
  }

  return is_satisfied_speed_range_;
}

bool ParallelLongitudinalAvoidDecider::CheckIfTheTruckIsParallel() {
  // 1. 检查车道保持状态
  const auto& curr_state =
      session_->planning_context().lane_change_decider_output().curr_state;
  const bool is_in_lane_change_condition = curr_state == kLaneChangeExecution ||
                                           curr_state == kLaneChangeComplete ||
                                           curr_state == kLaneChangeHold;
  if (is_in_lane_change_condition) {
    return false;
  }

  // 2. 选择左右邻车道中纵向距离最近的障碍物
  const auto& dynamic_world =
      session_->environmental_model().get_dynamic_world();
  const auto ego_left_front_node_id = dynamic_world->ego_left_front_node_id();
  const auto ego_left_node_id = dynamic_world->ego_left_node_id();
  const auto ego_right_front_node_id = dynamic_world->ego_right_front_node_id();
  const auto ego_right_node_id = dynamic_world->ego_right_node_id();

  int64_t target_node_id = planning_data::kInvalidId,
          alter_target_node_id = planning_data::kInvalidId;
  SelectNearestNeighborAgent(ego_left_front_node_id, ego_left_node_id,
                             ego_right_front_node_id, ego_right_node_id,
                             &target_node_id, &alter_target_node_id);

  const auto* agent_manger = dynamic_world->agent_manager();
  const auto* target_node = dynamic_world->GetNode(target_node_id);
  const auto* alter_target_node = dynamic_world->GetNode(alter_target_node_id);

  const int64_t target_node_agent_id =
      target_node ? target_node->node_agent_id() : planning_data::kInvalidId;
  const int64_t alter_target_node_agent_id =
      alter_target_node ? alter_target_node->node_agent_id()
                        : planning_data::kInvalidId;
  const auto* target_agent = agent_manger->GetAgent(target_node_agent_id);
  const auto* alter_target_agent =
      agent_manger->GetAgent(alter_target_node_agent_id);

  if (target_agent != nullptr &&
      target_node_agent_id != planning_data::kInvalidId) {
    parallel_target_agent_id_ = target_node_agent_id;
  } else if (alter_target_agent != nullptr &&
             alter_target_node_agent_id != planning_data::kInvalidId) {
    parallel_target_agent_id_ = alter_target_node_agent_id;
  } else {
    return false;
  }

  // 3. 检查是否已在主ST图中且出现时间小于阈值
  auto* mutable_st_graph = session_->planning_context().st_graph();
  if (mutable_st_graph->agent_id_st_boundaries_map().count(
          parallel_target_agent_id_) > 0) {
    const auto& boundary_ids =
        mutable_st_graph->agent_id_st_boundaries_map().at(
            parallel_target_agent_id_);
    if (!boundary_ids.empty()) {
      const auto& boundary_id_st_boundaries_map =
          mutable_st_graph->boundary_id_st_boundaries_map();
      const auto boundary_iter =
          boundary_id_st_boundaries_map.find(boundary_ids.front());
      if (boundary_iter != boundary_id_st_boundaries_map.end() &&
          boundary_iter->second != nullptr) {
        const double current_agent_appearance_time =
            boundary_iter->second->min_t();
        if (current_agent_appearance_time < kAgentAppearanceTimeThresholdS) {
          return false;
        }
      }
    }
  }

  // 4. 检查是否在邻道ST图中，如果不在则插入
  const auto& environmental_model = session_->environmental_model();
  if (environmental_model.get_virtual_lane_manager() == nullptr ||
      environmental_model.get_dynamic_world() == nullptr ||
      environmental_model.get_dynamic_world()->agent_manager() == nullptr ||
      session_->planning_context().st_graph() == nullptr ||
      environmental_model.get_ego_state_manager() == nullptr) {
    return false;
  }

  auto neighbor_agent_id_st_boundraies_map =
      mutable_st_graph->neighbor_agent_id_st_boundaries_map();
  if (neighbor_agent_id_st_boundraies_map.count(parallel_target_agent_id_) ==
      0) {
    const agent::Agent* target_agent =
        environmental_model.get_dynamic_world()->agent_manager()->GetAgent(
            parallel_target_agent_id_);
    if (!ConstructNeighborLaneStGraph(target_agent)) return false;
  }

  // 5. 检查目标障碍物纵向位置是否比前车小，且在纵向阈值内
  const auto& ego_lane = session_->environmental_model()
                             .get_virtual_lane_manager()
                             ->get_current_lane();
  if (ego_lane == nullptr || ego_lane->get_lane_frenet_coord() == nullptr)
    return false;

  const auto& ego_lane_coord = ego_lane->get_lane_frenet_coord();
  double ego_s = 0.0, ego_l = 0.0;
  const auto& ego_init_point = session_->environmental_model()
                                   .get_ego_state_manager()
                                   ->planning_init_point();
  if (!ego_lane_coord->XYToSL(ego_init_point.x, ego_init_point.y, &ego_s,
                              &ego_l)) {
    return false;
  }

  double half_agent_length = target_agent->length() * kVehicleLengthRatio;
  const auto agent_box_conter = target_agent->box().center();
  double agent_s = 0.0, agent_l = 0.0;
  ego_lane_coord->XYToSL(agent_box_conter.x(), agent_box_conter.y(), &agent_s,
                         &agent_l);

  const double longitudinal_distance = std::max(
      agent_s - ego_s - front_edge_to_rear_axle_ - half_agent_length, 0.0);

  const auto& cipv_info = session_->planning_context().cipv_decider_output();
  double cipv_s = kDefaultCipvDistanceM;
  if (cipv_info.cipv_id() != planning_data::kInvalidId) {
    cipv_s = cipv_info.relative_s();
  }
  if (longitudinal_distance > cipv_s || longitudinal_distance > kMaxDistanceM) {
    return false;
  }

  // 6. 检查是否为逆行障碍物
  const auto matched_path_point = ego_lane_coord->GetPathPointByS(agent_s);
  const double heading_diff = std::fabs(planning_math::NormalizeAngle(
      matched_path_point.theta() - target_agent->theta()));
  const bool is_perception_reverse = heading_diff > kReverseHeadingThresholdRad;
  bool is_prediction_reverse = false;
  if (!target_agent->trajectories().empty() &&
      !target_agent->trajectories().front().empty()) {
    const auto& end_point = target_agent->trajectories().front().back();
    double end_s = 0.0, end_l = 0.0;
    ego_lane_coord->XYToSL(end_point.x(), end_point.y(), &end_s, &end_l);
    is_prediction_reverse = end_s < agent_s;
  }

  return !(is_perception_reverse || is_prediction_reverse);
}

int64_t ParallelLongitudinalAvoidDecider::FindMatchingNodeId(
    const agent::Agent* agent, const double target_agent_s) {
  const auto& dynamic_world =
      session_->environmental_model().get_dynamic_world();
  const auto& virtual_lane_mgr =
      session_->environmental_model().get_virtual_lane_manager();
  double nearest_s = 0.0;
  double nearest_l = 0.0;
  const auto& ptr_obj_lane = virtual_lane_mgr->GetNearestLane(
      {agent->x(), agent->y()}, &nearest_s, &nearest_l);
  if (ptr_obj_lane == nullptr) {
    return -1;
  }
  const auto& target_lane = virtual_lane_mgr->get_lane_with_virtual_id(
      ptr_obj_lane->get_virtual_id());
  if (target_lane == nullptr) {
    return -1;
  }
  const auto target_lane_nodes =
      dynamic_world->GetNodesByLaneId(ptr_obj_lane->get_virtual_id());
  for (auto it = target_lane_nodes.rbegin(); it != target_lane_nodes.rend();
       ++it) {
    const auto& target_lane_node = *it;
    if (target_lane_node->node_s() < target_agent_s) continue;
    if (target_lane_node->node_agent_id() == parallel_target_agent_id_)
      continue;
    double half_width =
        0.5 * target_lane->width_by_s(target_lane_node->node_s());
    if (std::fabs(target_lane_node->node_l_min_max().first) >
            half_width - 0.5 ||
        std::fabs(target_lane_node->node_l_min_max().second) <
            -(half_width - 0.5)) {
      continue;
    }
    return target_lane_node->node_agent_id();
  }
  return -1;
}

bool ParallelLongitudinalAvoidDecider::CheckIfNeedOvertake(
    const agent::Agent* agent) {
  if (agent == nullptr) return false;

  // 1.检查障碍物速度范围
  if (!IsSpeedInRange(agent, kTruckAvoidVelocityMinKmh,
                      kTruckAvoidVelocityMaxKmh)) {
    return false;
  }

  // 2.检查障碍物是否为卡车/巴士
  if (!IsDynamicTruckOrBus(agent)) return false;

  // 3.检查速度差条件
  const auto& ego_init_point = session_->environmental_model()
                                   .get_ego_state_manager()
                                   ->planning_init_point();
  const double ego_speed_kmh = ego_init_point.v * kMpsToKmh;
  const double target_agent_speed_kmh = agent->speed() * kMpsToKmh;
  const double speed_diff = ego_speed_kmh - target_agent_speed_kmh;
  if (speed_diff <= kOvertakeStartSpeedDiffThresholdKmh) return false;

  const auto& ego_lane = session_->environmental_model()
                             .get_virtual_lane_manager()
                             ->get_current_lane();
  if (ego_lane == nullptr || ego_lane->get_lane_frenet_coord() == nullptr) {
    return false;
  }

  const auto& ego_lane_coord = ego_lane->get_lane_frenet_coord();
  double ego_s = 0.0, ego_l = 0.0;
  if (!ego_lane_coord->XYToSL(ego_init_point.x, ego_init_point.y, &ego_s,
                              &ego_l)) {
    return false;
  }

  double agent_s = 0.0, agent_l = 0.0;
  double half_agent_length = agent->length() * 0.5;
  const auto agent_center = agent->box().center();
  if (!ego_lane_coord->XYToSL(agent_center.x(), agent_center.y(), &agent_s,
                              &agent_l)) {
    return false;
  }

  const double longitudinal_distance = std::max(
      agent_s - ego_s - front_edge_to_rear_axle_ - half_agent_length, 0.0);

  // 4.检查横向冲突
  if (CheckLateralConflict(agent, kStartLateralConflictRatio)) {
    return false;
  }

  // 5.检查纵向距离条件
  if (longitudinal_distance >= kOvertakeDistanceThresholdM) {
    return false;
  }

  // 6.检查超车轨迹
  bool is_overtake_trajectory = false;
  std::shared_ptr<VariableCoordinateTimeOptimalTrajectory>
      ptr_overtake_trajectory;
  is_overtake_trajectory = CalculateOvertakeReachingTrajectory(
      ego_init_point, agent, session_->planning_context().st_graph_helper(),
      ptr_overtake_trajectory);

  // 7.检查前前方障碍物的纵向距离和横向冲突
  const auto& dynamic_world =
      session_->environmental_model().get_dynamic_world();
  const auto matching_node_agent_id = FindMatchingNodeId(agent, agent_s);
  const auto* agent_manager = dynamic_world->agent_manager();
  const auto* target_front_agent =
      agent_manager->GetAgent(matching_node_agent_id);

  if (target_front_agent == nullptr) {
    return is_overtake_trajectory;
  }

  double target_agent_front_node_s = 0.0, target_agent_front_node_l = 0.0;
  if (!ego_lane_coord->XYToSL(target_front_agent->box().center().x(),
                              target_front_agent->box().center().y(),
                              &target_agent_front_node_s,
                              &target_agent_front_node_l)) {
    return is_overtake_trajectory;
  }

  const double longitudinal_target_agent_front_distance =
      std::max(target_agent_front_node_s - ego_s - front_edge_to_rear_axle_ -
                   target_front_agent->length() * kVehicleLengthRatio,
               0.0);

  const bool has_front_agent_conflict =
      CheckLateralConflict(target_front_agent, kStartLateralConflictRatio) &&
      longitudinal_target_agent_front_distance <=
          ego_init_point.v * kStartTargetFrontAgentStartTimeHeadwayS;

  return is_overtake_trajectory && !has_front_agent_conflict;
}

bool ParallelLongitudinalAvoidDecider::IsSatisfiedOvertakeExitCondition(
    const double ego_speed_kmh,
    const speed::StGraphHelper* const st_graph_helper,
    const agent::Agent* const agent, const int64_t agent_id) {
  if (agent == nullptr) return true;

  // 1.检查障碍物是否为卡车/巴士，如果不是卡车/巴士，退出超车
  if (!IsDynamicTruckOrBus(agent)) return true;

  // 2.检查横向冲突, 如果横向空间不够，退出超车
  if (CheckLateralConflict(agent, kStopLateralConflictRatio)) {
    return true;
  }

  const auto& ego_init_point = session_->environmental_model()
                                   .get_ego_state_manager()
                                   ->planning_init_point();
  const auto& ego_lane = session_->environmental_model()
                             .get_virtual_lane_manager()
                             ->get_current_lane();
  if (ego_lane == nullptr || ego_lane->get_lane_frenet_coord() == nullptr) {
    return true;
  }

  const auto& ego_lane_coord = ego_lane->get_lane_frenet_coord();
  double ego_s = 0.0, ego_l = 0.0;
  if (!ego_lane_coord->XYToSL(ego_init_point.x, ego_init_point.y, &ego_s,
                              &ego_l)) {
    return true;
  }

  double agent_front_s = 0.0, agent_front_l = 0.0;
  const auto agent_box_center = agent->box().center();
  if (!ego_lane_coord->XYToSL(agent_box_center.x(), agent_box_center.y(),
                              &agent_front_s, &agent_front_l)) {
    return true;
  }

  const double longitudinal_distance =
      std::max(agent_front_s - ego_s - front_edge_to_rear_axle_ -
                   agent->length() * kVehicleLengthRatio,
               0.0);

  // 3.检查纵向距离是否过大，如果过大，退出超车
  if (longitudinal_distance > kExitMaxDistanceM) {
    return true;
  }

  const double agent_front_position =
      agent_front_s + agent->length() * kVehicleLengthRatio;
  double back_edge_to_rear_axle = 0.0;
  const auto& ego_vehicle_param =
      VehicleConfigurationContext::Instance()->get_vehicle_param();
  back_edge_to_rear_axle = ego_vehicle_param.rear_edge_to_rear_axle;
  const double ego_back_position = ego_s - back_edge_to_rear_axle;

  // 4.检查速度差大于启动阈值且障碍物车头位置 - 自车车尾位置 < 阈值，
  // 如果满足超车条件，退出超车
  const double target_agent_speed_kmh = agent->speed() * kMpsToKmh;
  const double speed_diff = ego_speed_kmh - target_agent_speed_kmh;
  bool is_reached_distance = (speed_diff > kOvertakeStopSpeedDiffThresholdKmh &&
                              agent_front_position - ego_back_position <
                                  kOvertakeExitDistanceThresholdM);

  // 5.检查前前方障碍物的纵向距离和横向冲突
  const auto& dynamic_world =
      session_->environmental_model().get_dynamic_world();
  const auto matching_node_agent_id = FindMatchingNodeId(agent, agent_front_s);
  const auto* agent_manager = dynamic_world->agent_manager();
  const auto* target_front_agent =
      agent_manager->GetAgent(matching_node_agent_id);

  if (target_front_agent == nullptr) {
    return is_reached_distance;
  }

  double target_agent_front_node_s = 0.0, target_agent_front_node_l = 0.0;
  if (!ego_lane_coord->XYToSL(target_front_agent->box().center().x(),
                              target_front_agent->box().center().y(),
                              &target_agent_front_node_s,
                              &target_agent_front_node_l)) {
    return is_reached_distance;
  }

  const double longitudinal_target_agent_front_distance =
      std::max(target_agent_front_node_s - ego_s - front_edge_to_rear_axle_ -
                   target_front_agent->length() * kVehicleLengthRatio,
               0.0);

  const bool has_front_agent_conflict =
      CheckLateralConflict(target_front_agent, kStopLateralConflictRatio) &&
      longitudinal_target_agent_front_distance <=
          ego_init_point.v * kStopTargetFrontAgentExitTimeHeadwayS;

  return is_reached_distance || has_front_agent_conflict;
}

bool ParallelLongitudinalAvoidDecider::CheckIfNeedYield(
    const agent::Agent* agent) {
  if (agent == nullptr) return false;

  // 1.检查障碍物速度范围
  if (!IsSpeedInRange(agent, kTruckYieldAvoidVelocityMinKmh,
                      kTruckAvoidVelocityMaxKmh)) {
    return false;
  }

  const auto& ego_init_point = session_->environmental_model()
                                   .get_ego_state_manager()
                                   ->planning_init_point();
  const double ego_speed_kmh = ego_init_point.v * kMpsToKmh;
  const double target_agent_speed_kmh = agent->speed() * kMpsToKmh;
  const double speed_diff = ego_speed_kmh - target_agent_speed_kmh;

  // 2. 检查速度差条件
  if (speed_diff < kStartYieldSpeedDiffThresholdKmh) {
    return false;
  }

  // 3.检查是否是左方或者右方障碍物
  const auto& dynamic_world =
      session_->environmental_model().get_dynamic_world();
  const auto ego_left_node_id = dynamic_world->ego_left_node_id();
  const auto ego_right_node_id = dynamic_world->ego_right_node_id();
  const auto* ego_left_node = dynamic_world->GetNode(ego_left_node_id);
  const auto* ego_right_node = dynamic_world->GetNode(ego_right_node_id);

  if (ego_left_node != nullptr &&
      ego_left_node->node_agent_id() == parallel_target_agent_id_) {
    return false;
  }

  if (ego_right_node != nullptr &&
      ego_right_node->node_agent_id() == parallel_target_agent_id_) {
    return false;
  }

  // 4.检查横向冲突
  if (!CheckLateralConflict(agent, kStartYieldLateralConflictRatio)) {
    return false;
  }

  // 5.如果不是大车，检查横向TTC冲突
  if (!IsDynamicTruckOrBus(agent) &&
      !CheckLateralTTC(agent, kStartYieldLateralTTCThresholdS)) {
    return false;
  }

  // 6.检查纵向TTC
  const auto& ego_lane = session_->environmental_model()
                             .get_virtual_lane_manager()
                             ->get_current_lane();
  if (ego_lane == nullptr || ego_lane->get_lane_frenet_coord() == nullptr) {
    return false;
  }

  const auto& ego_lane_coord = ego_lane->get_lane_frenet_coord();
  double ego_s = 0.0, ego_l = 0.0;
  if (!ego_lane_coord->XYToSL(ego_init_point.x, ego_init_point.y, &ego_s,
                              &ego_l)) {
    return false;
  }

  double agent_s = 0.0, agent_l = 0.0;
  double half_agent_length = agent->length() * 0.5;
  const auto agent_center = agent->box().center();
  if (!ego_lane_coord->XYToSL(agent_center.x(), agent_center.y(), &agent_s,
                              &agent_l)) {
    return false;
  }

  const double longitudinal_distance = std::max(
      agent_s - ego_s - front_edge_to_rear_axle_ - half_agent_length, 0.0);
  const double longitudinal_distance_threshold =
      ego_init_point.v * kLongitudinalTTCStartThresholdS;
  if (longitudinal_distance > longitudinal_distance_threshold) {
    return false;
  }

  // 7.检查让行轨迹
  std::shared_ptr<VariableCoordinateTimeOptimalTrajectory> ptr_yield_trajectory;
  return CalculateYieldReachingTrajectory(
      ego_init_point, agent, session_->planning_context().st_graph_helper(),
      ptr_yield_trajectory);
}

bool ParallelLongitudinalAvoidDecider::IsSatisfiedYieldExitCondition(
    const double ego_speed_kmh,
    const speed::StGraphHelper* const st_graph_helper,
    const agent::Agent* const agent, const int64_t agent_id) {
  if (agent == nullptr) return true;

  const auto& ego_init_point = session_->environmental_model()
                                   .get_ego_state_manager()
                                   ->planning_init_point();
  const auto& ego_lane = session_->environmental_model()
                             .get_virtual_lane_manager()
                             ->get_current_lane();
  if (ego_lane == nullptr || ego_lane->get_lane_frenet_coord() == nullptr) {
    return true;
  }

  const auto& ego_lane_coord = ego_lane->get_lane_frenet_coord();
  double ego_s = 0.0, ego_l = 0.0;
  if (!ego_lane_coord->XYToSL(ego_init_point.x, ego_init_point.y, &ego_s,
                              &ego_l)) {
    return true;
  }

  double agent_s = 0.0, agent_l = 0.0;
  const auto agent_box_center = agent->box().center();
  if (!ego_lane_coord->XYToSL(agent_box_center.x(), agent_box_center.y(),
                              &agent_s, &agent_l)) {
    return true;
  }

  const double longitudinal_distance =
      std::max(agent_s - ego_s - front_edge_to_rear_axle_ -
                   agent->length() * kVehicleLengthRatio,
               0.0);

  // 1.检查横向冲突和横向TTC条件，如果不再有横向冲突，退出让行
  if (!CheckLateralConflict(agent, kStopYieldLateralConflictRatio)) {
    return true;
  }

  // 2.检查是否是左方或者右方障碍物, 如果是，退出让行
  const auto& dynamic_world =
      session_->environmental_model().get_dynamic_world();
  const auto ego_left_node_id = dynamic_world->ego_left_node_id();
  const auto ego_right_node_id = dynamic_world->ego_right_node_id();
  const auto* ego_left_node = dynamic_world->GetNode(ego_left_node_id);
  const auto* ego_right_node = dynamic_world->GetNode(ego_right_node_id);

  if (ego_left_node != nullptr &&
      ego_left_node->node_agent_id() == parallel_target_agent_id_) {
    return true;
  }

  if (ego_right_node != nullptr &&
      ego_right_node->node_agent_id() == parallel_target_agent_id_) {
    return true;
  }

  // 3.检查纵向TTC条件，如果纵向TTC大于退出阈值，退出让行
  const double distance_threshold =
      ego_init_point.v * kLongitudinalTTCStopThresholdS;
  if (longitudinal_distance > distance_threshold) {
    return true;
  }

  // 4.检查纵向距离是否过大，如果过大，退出让行
  if (longitudinal_distance > kExitMaxDistanceM) {
    return true;
  }

  // 5.如果不是大车，检查横向TTC冲突, 如果存在横向TTC冲突，退出让行
  if (!IsDynamicTruckOrBus(agent) &&
      CheckLateralTTC(agent, kStopYieldLateralTTCThresholdS)) {
    return true;
  }

  const double target_agent_speed_kmh = agent->speed() * kMpsToKmh;
  const double speed_diff = ego_speed_kmh - target_agent_speed_kmh;

  // 6.检查速度差小于负阈值且大车车尾位置 - 自车车头位置 > 阈值，
  // 如果满足让行条件，退出让行
  const double agent_back_position =
      agent_s - agent->length() * kVehicleLengthRatio;
  const double ego_front_position = ego_s + front_edge_to_rear_axle_;
  bool is_reached_distance =
      (speed_diff < -kStopYieldSpeedDiffThresholdKmh &&
       agent_back_position - ego_front_position > kLeadTargetTrucksBufferM);

  return is_reached_distance;
}

bool ParallelLongitudinalAvoidDecider::CheckLeadAndTargetIsTruck(
    const agent::Agent* agent) {
  if (agent == nullptr) return false;

  // 1.检查障碍物速度范围
  if (!IsSpeedInRange(agent, kTruckAvoidVelocityMinKmh,
                      kTruckAvoidVelocityMaxKmh)) {
    return false;
  }

  // 2.检查障碍物是否为卡车/巴士
  if (!IsDynamicTruckOrBus(agent)) return false;

  const auto& ego_init_point = session_->environmental_model()
                                   .get_ego_state_manager()
                                   ->planning_init_point();
  const double ego_speed_kmh = ego_init_point.v * kMpsToKmh;

  // 3.检查是否是左方或者右方障碍物
  const auto& dynamic_world =
      session_->environmental_model().get_dynamic_world();
  const auto ego_left_node_id = dynamic_world->ego_left_node_id();
  const auto ego_right_node_id = dynamic_world->ego_right_node_id();
  const auto* ego_left_node = dynamic_world->GetNode(ego_left_node_id);
  const auto* ego_right_node = dynamic_world->GetNode(ego_right_node_id);

  if (ego_left_node != nullptr &&
      ego_left_node->node_agent_id() == parallel_target_agent_id_) {
    return false;
  }

  if (ego_right_node != nullptr &&
      ego_right_node->node_agent_id() == parallel_target_agent_id_) {
    return false;
  }

  // 4.检查前车信息
  const auto& cipv_info = session_->planning_context().cipv_decider_output();
  if (cipv_info.cipv_id() == planning_data::kInvalidId ||
      cipv_info.is_virtual()) {
    return false;
  }

  const auto& ego_lane = session_->environmental_model()
                             .get_virtual_lane_manager()
                             ->get_current_lane();
  if (ego_lane == nullptr || ego_lane->get_lane_frenet_coord() == nullptr) {
    return false;
  }

  const auto& ego_lane_coord = ego_lane->get_lane_frenet_coord();
  double ego_s = 0.0, ego_l = 0.0;
  if (!ego_lane_coord->XYToSL(ego_init_point.x, ego_init_point.y, &ego_s,
                              &ego_l)) {
    return false;
  }

  double target_agent_s = 0.0, target_agent_l = 0.0;
  double half_target_agent_length = agent->length() * kVehicleLengthRatio;
  const auto target_agent_center = agent->box().center();
  if (!ego_lane_coord->XYToSL(target_agent_center.x(), target_agent_center.y(),
                              &target_agent_s, &target_agent_l)) {
    return false;
  }

  const double ego_target_distance =
      std::max(target_agent_s - ego_s - front_edge_to_rear_axle_ -
                   half_target_agent_length,
               0.0);

  // 5.检查前车与自车的纵向距离
  const double lead_target_distance = cipv_info.relative_s();
  if (lead_target_distance >= ego_init_point.v * kLeadTargetStartTimeHeadwayS +
                                  kLeadTargetTrucksBufferM ||
      lead_target_distance > kMaxDistanceM) {
    return false;
  }

  // 6.检查前车与大车距离条件
  const double target_to_lead_distance =
      std::max(lead_target_distance - ego_target_distance, 0.0);

  if (target_to_lead_distance >=
      ego_init_point.v * kLeadTargetTrucksStartTimeHeadwayS +
          kLeadTargetTrucksBufferM) {
    return false;
  }

  // 7.检查是否有横向冲突
  if (!CheckLateralConflict(agent, kStartLateralConflictRatio)) {
    return false;
  }

  // 8.检查能否生成可行的yield轨迹
  std::shared_ptr<VariableCoordinateTimeOptimalTrajectory> ptr_yield_trajectory;
  if (!CalculateYieldReachingTrajectory(
          ego_init_point, agent, session_->planning_context().st_graph_helper(),
          ptr_yield_trajectory)) {
    return false;
  }

  return true;
}

bool ParallelLongitudinalAvoidDecider::
    IsSatisfiedLeadAndTargetIsTruckExitCondition(
        const agent::Agent* const agent) {
  if (agent == nullptr) return true;

  // 1.检查障碍物是否为卡车/巴士，如果不是卡车/巴士，退出并排让行
  if (!IsDynamicTruckOrBus(agent)) return true;

  const auto& ego_init_point = session_->environmental_model()
                                   .get_ego_state_manager()
                                   ->planning_init_point();
  const double ego_speed_kmh = ego_init_point.v * kMpsToKmh;

  const auto& cipv_info = session_->planning_context().cipv_decider_output();
  if (cipv_info.cipv_id() == planning_data::kInvalidId ||
      cipv_info.is_virtual()) {
    return true;
  }

  const auto& ego_lane = session_->environmental_model()
                             .get_virtual_lane_manager()
                             ->get_current_lane();
  if (ego_lane == nullptr || ego_lane->get_lane_frenet_coord() == nullptr) {
    return true;
  }

  const auto& ego_lane_coord = ego_lane->get_lane_frenet_coord();
  double ego_s = 0.0, ego_l = 0.0;
  if (!ego_lane_coord->XYToSL(ego_init_point.x, ego_init_point.y, &ego_s,
                              &ego_l)) {
    return true;
  }

  // 2.检查是否是左方或者右方障碍物, 如果是，退出并排让行
  const auto& dynamic_world =
      session_->environmental_model().get_dynamic_world();
  const auto ego_left_node_id = dynamic_world->ego_left_node_id();
  const auto ego_right_node_id = dynamic_world->ego_right_node_id();
  const auto* ego_left_node = dynamic_world->GetNode(ego_left_node_id);
  const auto* ego_right_node = dynamic_world->GetNode(ego_right_node_id);

  if (ego_left_node != nullptr &&
      ego_left_node->node_agent_id() == parallel_target_agent_id_) {
    return true;
  }

  if (ego_right_node != nullptr &&
      ego_right_node->node_agent_id() == parallel_target_agent_id_) {
    return true;
  }

  // 3.检查目标障碍物纵向距离是否大于阈值，如果大于阈值，退出并排让行
  double target_agent_s = 0.0, target_agent_l = 0.0;
  double half_target_agent_length = agent->length() * kVehicleLengthRatio;
  const auto target_agent_center = agent->box().center();
  if (!ego_lane_coord->XYToSL(target_agent_center.x(), target_agent_center.y(),
                              &target_agent_s, &target_agent_l)) {
    return true;
  }

  const double ego_to_target_distance =
      std::max(target_agent_s - ego_s - half_target_agent_length -
                   front_edge_to_rear_axle_,
               0.0);
  if (ego_to_target_distance > kExitMaxDistanceM) {
    return true;
  }

  // 4.检查前车与自车的纵向距离,如果距离大于退出时距阈值，退出并排让行
  const double lead_target_distance = cipv_info.relative_s();
  if (lead_target_distance >= ego_init_point.v * kLeadTargetExitTimeHeadwayS +
                                  kLeadTargetTrucksBufferM ||
      lead_target_distance > kExitMaxDistanceM) {
    return true;
  }

  // 5.检查前车与大车的纵向距离, 如果距离大于退出时距阈值，退出并排让行
  const double target_to_lead_distance =
      std::max(lead_target_distance - ego_to_target_distance, 0.0);
  if (target_to_lead_distance >=
      ego_init_point.v * kLeadTargetTrucksExitTimeHeadwayS +
          kLeadTargetTrucksBufferM) {
    return true;
  }

  // 6.检查横向冲突和横向TTC条件, 如果不再有横向冲突，退出并排让行
  if (!CheckLateralConflict(agent, kStopLateralConflictRatio)) {
    return true;
  }

  const double target_agent_speed_kmh = agent->speed() * kMpsToKmh;
  const double speed_diff = ego_speed_kmh - target_agent_speed_kmh;

  // 7.检查速度差小于负阈值且大车车尾位置 - 自车车头位置 > 最小缓冲距离阈值,
  // 如果满足让行条件，退出并排让行
  const double agent_back_position =
      target_agent_s - agent->length() * kVehicleLengthRatio;
  const double ego_front_position = ego_s + front_edge_to_rear_axle_;
  bool is_reached_distance =
      (speed_diff < -kStopYieldSpeedDiffThresholdKmh &&
       agent_back_position - ego_front_position > kLeadTargetTrucksBufferM);

  return is_reached_distance;
}

void ParallelLongitudinalAvoidDecider::OutputRunning() {
  auto& output = session_->mutable_planning_context()
                     ->mutable_parallel_longitudinal_avoid_decider_output();

  output.set_is_parallel_overtake(is_need_overtake_);
  output.set_is_parallel_yield(is_need_yield_);
  output.set_is_need_parallel_longitudinal_avoid(
      is_running_longitudinal_avoid_);
  output.set_parallel_target_agent_id(parallel_target_agent_id_);
  output.set_is_lead_and_target_is_truck(is_lead_and_target_is_truck_);

  output.set_current_state(static_cast<int32_t>(current_state_));
  output.set_running_frame_count(running_frame_count_);
  output.set_cooldown_frame_count(cooldown_frame_count_);

  const auto* target_agent = session_->environmental_model()
                                 .get_dynamic_world()
                                 ->agent_manager()
                                 ->GetAgent(parallel_target_agent_id_);
  if (target_agent == nullptr) return;

  // 判断是否为让行场景
  bool is_yield = is_need_yield_ || is_lead_and_target_is_truck_;

  planning_math::Vec2d match_point = target_agent->box().center();

  if (is_need_overtake_) {
    // 超车场景：选择前角点
    match_point = is_right_agent_ ? target_agent->box().GetAllCorners().at(0)
                                  :  // 右侧障碍物使用左前角点（自车在左侧超车）
                      target_agent->box().GetAllCorners().at(
                          1);  // 左侧障碍物使用右前角点（自车在右侧超车）
  } else if (is_yield) {
    // 让行场景：选择后角点
    match_point = is_right_agent_ ? target_agent->box().GetAllCorners().at(3)
                                  :  // 右侧障碍物使用左后角点（自车在左侧让行）
                      target_agent->box().GetAllCorners().at(
                          2);  // 左侧障碍物使用右后角点（自车在右侧让行）
  }

  const auto& ego_init_point = session_->environmental_model()
                                   .get_ego_state_manager()
                                   ->planning_init_point();
  const auto& ego_lane = session_->environmental_model()
                             .get_virtual_lane_manager()
                             ->get_current_lane();

  if (ego_lane != nullptr && ego_lane->get_lane_frenet_coord() != nullptr) {
    const auto ego_lane_coord = ego_lane->get_lane_frenet_coord();
    double ego_s = 0.0, ego_l = 0.0;
    if (ego_lane_coord->XYToSL(ego_init_point.x, ego_init_point.y, &ego_s,
                               &ego_l)) {
      double match_point_s = 0.0, match_point_l = 0.0;
      if (ego_lane_coord->XYToSL(match_point.x(), match_point.y(),
                                 &match_point_s, &match_point_l)) {
        const double relative_s =
            match_point_s - ego_s - front_edge_to_rear_axle_;
        double agent_velocity = target_agent->speed();
        if (is_need_yield_ &&
            relative_s < ego_init_point.v * kCloseDistanceTimeHeadwayS) {
          agent_velocity = std::max(
              0.0, std::min(agent_velocity,
                            ego_init_point.v - kMinSpeedForYieldTruck));
        }
        output.set_trajectory_start_s(relative_s);
        output.set_trajectory_start_v(agent_velocity);
        output.set_lateral_distance(lateral_distance_);
      }
    }
  }
}

bool ParallelLongitudinalAvoidDecider::CheckLateralConflict(
    const agent::Agent* agent, const double lane_width_ratio) {
  if (agent == nullptr) return false;

  const auto& ego_init_point = session_->environmental_model()
                                   .get_ego_state_manager()
                                   ->planning_init_point();

  const auto& ego_lane = session_->environmental_model()
                             .get_virtual_lane_manager()
                             ->get_current_lane();

  if (ego_lane == nullptr || ego_lane->get_lane_frenet_coord() == nullptr) {
    return false;
  }

  const auto& ego_lane_coord = ego_lane->get_lane_frenet_coord();

  double ego_s = 0.0, ego_l = 0.0;
  if (!ego_lane_coord->XYToSL(ego_init_point.x, ego_init_point.y, &ego_s,
                              &ego_l))
    return false;

  const auto& ego_vehicle_param =
      VehicleConfigurationContext::Instance()->get_vehicle_param();
  const double half_ego_width = ego_vehicle_param.width * 0.5;

  const double ego_l_min = ego_l - half_ego_width;
  const double ego_l_max = ego_l + half_ego_width;

  double agent_s = 0.0, agent_l = 0.0;
  const auto agent_center = agent->box().center();
  if (!ego_lane_coord->XYToSL(agent_center.x(), agent_center.y(), &agent_s,
                              &agent_l)) {
    return false;
  }

  const auto& agent_corners = agent->box().GetAllCorners();
  double obs_min_l = std::numeric_limits<double>::max(),
         obs_max_l = -std::numeric_limits<double>::max();
  for (const auto& agent_corner : agent_corners) {
    double agent_corner_s = 0.0, agent_corner_l = 0.0;
    if (ego_lane_coord->XYToSL(agent_corner.x(), agent_corner.y(),
                               &agent_corner_s, &agent_corner_l)) {
      obs_min_l = std::min(obs_min_l, agent_corner_l);
      obs_max_l = std::max(obs_max_l, agent_corner_l);
    }
  }

  double lateral_dist = 0.0;
  if (obs_min_l > ego_l_max)
    lateral_dist = obs_min_l - ego_l_max;
  else if (obs_max_l < ego_l_min)
    lateral_dist = ego_l_min - obs_max_l;

  lateral_distance_ = lateral_dist;

  bool has_lateral_distance_conflict =
      lateral_dist <= ego_lane->width_by_s(ego_s) * lane_width_ratio;

  return has_lateral_distance_conflict;
}

bool ParallelLongitudinalAvoidDecider::CheckLateralTTC(
    const agent::Agent* agent, const double lateral_ttc_threshold_s) {
  if (agent == nullptr) return false;

  const auto& ego_init_point = session_->environmental_model()
                                   .get_ego_state_manager()
                                   ->planning_init_point();

  const auto& ego_lane = session_->environmental_model()
                             .get_virtual_lane_manager()
                             ->get_current_lane();

  if (ego_lane == nullptr || ego_lane->get_lane_frenet_coord() == nullptr) {
    return false;
  }

  const auto& ego_lane_coord = ego_lane->get_lane_frenet_coord();

  double ego_s = 0.0, ego_l = 0.0;
  if (!ego_lane_coord->XYToSL(ego_init_point.x, ego_init_point.y, &ego_s,
                              &ego_l))
    return false;

  const auto& ego_vehicle_param =
      VehicleConfigurationContext::Instance()->get_vehicle_param();
  const double half_ego_width = ego_vehicle_param.width * 0.5;

  const double ego_l_min = ego_l - half_ego_width;
  const double ego_l_max = ego_l + half_ego_width;

  double agent_s = 0.0, agent_l = 0.0;
  const auto agent_center = agent->box().center();
  if (!ego_lane_coord->XYToSL(agent_center.x(), agent_center.y(), &agent_s,
                              &agent_l)) {
    return false;
  }

  const auto& agent_corners = agent->box().GetAllCorners();
  double obs_min_l = std::numeric_limits<double>::max(),
         obs_max_l = -std::numeric_limits<double>::max();
  for (const auto& agent_corner : agent_corners) {
    double agent_corner_s = 0.0, agent_corner_l = 0.0;
    if (ego_lane_coord->XYToSL(agent_corner.x(), agent_corner.y(),
                               &agent_corner_s, &agent_corner_l)) {
      obs_min_l = std::min(obs_min_l, agent_corner_l);
      obs_max_l = std::max(obs_max_l, agent_corner_l);
    }
  }

  double lateral_dist = 0.0;
  if (obs_min_l > ego_l_max)
    lateral_dist = obs_min_l - ego_l_max;
  else if (obs_max_l < ego_l_min)
    lateral_dist = ego_l_min - obs_max_l;

  const double agent_speed = agent->speed();
  auto matched_point = ego_lane_coord->GetPathPointByS(agent_s);
  double heading_diff =
      planning_math::NormalizeAngle(agent->theta() - matched_point.theta());
  double agent_lateral_speed =
      std::max(1e-6, agent_speed * std::sin(heading_diff));

  double lateral_distance_threshold =
      agent_lateral_speed * lateral_ttc_threshold_s;

  bool has_lateral_ttc_conflict = lateral_dist <= lateral_distance_threshold;

  return has_lateral_ttc_conflict;
}

bool ParallelLongitudinalAvoidDecider::CalculateOvertakeReachingTrajectory(
    const PlanningInitPoint& init_point, const agent::Agent* const truck_agent,
    const speed::StGraphHelper* const st_graph_helper,
    std::shared_ptr<VariableCoordinateTimeOptimalTrajectory>&
        ptr_overtake_trajectory) {
  LonState init_state{init_point.v, init_point.a};
  const double speed_buffer = 5.2;
  const double v_upper_end = std::fmax(
      std::fmin(init_point.v + speed_buffer, kMaxSpeedForOvertakeTruck),
      init_point.v);

  StateLimit state_limit{0.0, init_point.v, v_upper_end, -2.0, 1.5, -2.0, 2.0};
  std::vector<int64_t> boundary_ids;
  if (!st_graph_helper->GetNeighborAgentStBoundaries(truck_agent->agent_id(),
                                                     &boundary_ids) ||
      boundary_ids.empty()) {
    return false;
  }

  speed::STBoundary st_boundary;
  if (!st_graph_helper->GetNeighborAgentStBoundary(boundary_ids.at(0),
                                                   &st_boundary) ||
      st_boundary.IsEmpty()) {
    return false;
  }

  const double target_s = st_boundary.upper_points().front().s() +
                          kOvertakeDistanceM;  // 超车位置 = 上边界 + 超车距离
  const double target_vel = truck_agent->speed();
  CoordinateParam relative_coordinate_param{target_s, target_vel};

  ptr_overtake_trajectory =
      std::make_shared<VariableCoordinateTimeOptimalTrajectory>(
          VariableCoordinateTimeOptimalTrajectory::ConstructInstance(
              init_state, state_limit, relative_coordinate_param, 0.1));

  if (nullptr == ptr_overtake_trajectory) return false;

  const double time_resolution = 0.2, collision_buff = 1.0,
               plan_time_length = 3.0;
  const double check_time_length =
      std::fmin(plan_time_length, ptr_overtake_trajectory->ParamLength());

  for (double t = 0; t < check_time_length + 1e-6; t += time_resolution) {
    const double s = ptr_overtake_trajectory->Evaluate(0, t);
    speed::STPoint lower_point, upper_point;
    if (!st_graph_helper->GetBorderByStPoint(s, t, &lower_point,
                                             &upper_point) ||
        std::fabs(s - lower_point.s()) < collision_buff ||
        std::fabs(s - upper_point.s()) < collision_buff) {
      return false;
    }
  }

  return true;
}

bool ParallelLongitudinalAvoidDecider::CalculateYieldReachingTrajectory(
    const PlanningInitPoint& init_point, const agent::Agent* const truck_agent,
    const speed::StGraphHelper* const st_graph_helper,
    std::shared_ptr<VariableCoordinateTimeOptimalTrajectory>&
        ptr_yield_trajectory) {
  LonState init_state{init_point.v, init_point.a};
  const double speed_buffer = 4.0;
  const double v_lower_end =
      std::fmin(std::fmax(init_point.v - speed_buffer, kMinSpeedForYieldTruck),
                init_point.v);

  StateLimit state_limit{0.0, v_lower_end, init_point.v, -2.0, 1.5, -2.0, 2.0};

  std::vector<int64_t> boundary_ids;
  if (!st_graph_helper->GetNeighborAgentStBoundaries(truck_agent->agent_id(),
                                                     &boundary_ids) ||
      boundary_ids.empty()) {
    return false;
  }

  speed::STBoundary st_boundary;
  if (!st_graph_helper->GetNeighborAgentStBoundary(boundary_ids.at(0),
                                                   &st_boundary) ||
      st_boundary.IsEmpty()) {
    return false;
  }

  const double target_s = st_boundary.lower_points().front().s() -
                          kYieldDistanceM;  // 让行位置 = 下边界 - 让行距离
  const double target_vel = truck_agent->speed();
  CoordinateParam relative_coordinate_param{target_s, target_vel};

  ptr_yield_trajectory =
      std::make_shared<VariableCoordinateTimeOptimalTrajectory>(
          VariableCoordinateTimeOptimalTrajectory::ConstructInstance(
              init_state, state_limit, relative_coordinate_param, 0.1));

  if (nullptr == ptr_yield_trajectory) return false;

  const double time_resolution = 0.2, collision_buff = 1.0,
               plan_time_length = 3.0;
  const double check_time_length =
      std::fmin(plan_time_length, ptr_yield_trajectory->ParamLength());

  for (double t = 0; t < check_time_length + 1e-6; t += time_resolution) {
    const double s = ptr_yield_trajectory->Evaluate(0, t);
    speed::STPoint lower_point, upper_point;
    if (!st_graph_helper->GetBorderByStPoint(s, t, &lower_point,
                                             &upper_point) ||
        std::fabs(s - lower_point.s()) < collision_buff ||
        std::fabs(s - upper_point.s()) < collision_buff) {
      return false;
    }
  }

  return true;
}

}  // namespace planning
