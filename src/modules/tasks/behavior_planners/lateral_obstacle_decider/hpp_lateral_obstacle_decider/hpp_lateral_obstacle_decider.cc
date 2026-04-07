#include "hpp_lateral_obstacle_decider.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <memory>
#include <vector>

#include "../../common/planning_gflags.h"
#include "debug_info_log.h"
#include "edt_manager.h"
#include "environment_model_debug_info.pb.h"
#include "environmental_model.h"
#include "math/math_utils.h"
#include "modules/tasks/behavior_planners/general_lateral_decider/general_lateral_decider_utils.h"
#include "obstacle_manager.h"
#include "planning_context.h"
#include "task_interface/lateral_obstacle_decider_output.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <memory>
#include <vector>

#include "../../common/planning_gflags.h"
#include "debug_info_log.h"
#include "edt_manager.h"
#include "environment_model_debug_info.pb.h"
#include "environmental_model.h"
#include "math/math_utils.h"

namespace planning {

HppLateralObstacleDecider::HppLateralObstacleDecider(
    const EgoPlanningConfigBuilder *config_builder, framework::Session *session)
    : BaseLateralObstacleDecider(config_builder, session) {
  const VehicleParam &vehicle_param =
      VehicleConfigurationContext::Instance()->get_vehicle_param();
  name_ = "HppLateralObstacleDecider";
  hybrid_ara_star_ = std::make_unique<HybridARAStar>(session);
  hpp_general_lateral_decider_config_ =
      session_->environmental_model()
          .hpp_config_builder()
          ->cast<HppLateralObstacleDeciderConfig>();
}

bool HppLateralObstacleDecider::Execute() {
  if (!PreCheck()) {
    ILOG_DEBUG << "PreCheck failed";
    return false;
  }

  auto &plan_history_traj = session_->mutable_planning_context()
                                ->mutable_lateral_obstacle_decider_output()
                                .plan_history_traj;
  auto &is_plan_history_traj_valid =
      session_->mutable_planning_context()
          ->mutable_lateral_obstacle_decider_output()
          .is_plan_history_traj_valid;
  auto &uniform_plan_history_traj =
      session_->mutable_planning_context()
          ->mutable_lateral_obstacle_decider_output()
          .uniform_plan_history_traj;
  plan_history_traj.clear();
  is_plan_history_traj_valid = false;
  uniform_plan_history_traj.clear();

  const auto target_reference_virtual_id = session_->planning_context()
                                               .lane_change_decider_output()
                                               .fix_lane_virtual_id;
  reference_path_ptr_ =
      session_->environmental_model()
          .get_reference_path_manager()
          ->get_reference_path_by_lane(target_reference_virtual_id, false);
  if (reference_path_ptr_ == nullptr) {
    return false;
  }
  ConstructPlanHistoryTraj(reference_path_ptr_);

  const auto &reference_path_ptr = session_->planning_context()
                                       .lane_change_decider_output()
                                       .coarse_planning_info.reference_path;
  DebugInfoManager::GetInstance()
      .GetDebugInfoPb()
      ->mutable_hybrid_ara_info()
      ->Clear();

  // 1. 获取障碍物预处理结果
  const auto &hpp_obstacle_lateral_preprocess_output =
      session_->planning_context().hpp_obstacle_lat_preprocess_output();
  const auto &obs_cluster_container =
      hpp_obstacle_lateral_preprocess_output.obs_cluster_container;
  const auto &obs_classification_result =
      hpp_obstacle_lateral_preprocess_output.obs_classification_result;

  // 2. 清理过期历史记录
  // double current_timestamp = IflyTime::Now_ms();
  // std::unordered_set<uint32_t> current_frame_ids;
  // for (const auto& pair : obs_item_map) current_frame_ids.insert(pair.first);
  // ClearOldConsistencyInfo(current_frame_ids, current_timestamp);

  // 2. 检查是否需要启动 A* 搜索
  bool enable_search = CheckEnableSearch(reference_path_ptr, search_result_);

  auto time1 = IflyTime::Now_ms();
  if (enable_search) {
    if (ARAStar()) {
      search_result_ = SearchResult::SUCCESS;
    } else {
      search_result_ = SearchResult::FAILED;
    };
  } else {
    search_result_ = SearchResult::NO_SEARCH;
  }
  auto time2 = IflyTime::Now_ms();
  JSON_DEBUG_VALUE("ARAStarTime", time2 - time1);

  if (search_result_ != SearchResult::SUCCESS) {
    // 3. 没有搜索结果，进行规则决策
    UpdateLatDecision(reference_path_ptr, obstacle_consistency_map_,
                      obs_cluster_container, obs_classification_result);
  } else {
    // 4. 存在搜索结果，进行规则后决策
    UpdateLatDecisionWithARAStar(reference_path_ptr);
  }

  // 5. 缓存决策结果，用于下一帧连续性保障
  const auto &lat_obstacle_decision = session_->mutable_planning_context()
                                    ->mutable_lateral_obstacle_decider_output()
                                    .lat_obstacle_decision;
  UpdateObstacleConsistencyMap(lat_obstacle_decision,
                               obstacle_consistency_map_);

  // 6. 将障碍物信息 & 决策结果存储到 protobuf debug
  SaveObstaleToEnvironmentModelDebug(reference_path_ptr, lat_obstacle_decision);
  return true;
}

void HppLateralObstacleDecider::UpdateLatDecision(
    const std::shared_ptr<ReferencePath> &reference_path_ptr,
    const ObstacleConsistencyMap &obstacle_consistency_map,
    const ObstacleClusterContainer &obs_cluster_container,
    const ObstacleClassificationResult &obs_classification_result) {
  auto &lat_obstacle_decision = session_->mutable_planning_context()
                                    ->mutable_lateral_obstacle_decider_output()
                                    .lat_obstacle_decision;
  lat_obstacle_decision.clear();
  for (const auto &cluster : obs_cluster_container.obstacle_clusters) {
    LatObstacleDecisionType decision;
    if (cluster.motion_types.empty() || cluster.rel_pos_types.empty()) continue;
    MakeDecisionForStaticCluster(cluster, obstacle_consistency_map,
                                 obs_classification_result, decision);
    for (const auto &obs_id : cluster.original_ids) {
      lat_obstacle_decision[obs_id] = decision;
      if (obstacle_consistency_map_.find(obs_id) ==
          obstacle_consistency_map_.end()) {
        obstacle_consistency_map_[obs_id].last_decision = decision;
      }
    }
  }
  for (const auto &obstacle : reference_path_ptr->get_obstacles()) {
    if (!obstacle->b_frenet_valid()) {
      continue;
    }
    int64_t obs_id = obstacle->id();
    if (obs_classification_result.id_to_rel_pos_type.find(obs_id)->second ==
        ObstacleRelPosType::FAR_AWAY)
      continue;
    if (lat_obstacle_decision.find(obs_id) == lat_obstacle_decision.end()) {
      MakeDecisionForSingleDynamicObs(reference_path_ptr, obstacle);
      if (obstacle_consistency_map_.find(obs_id) ==
          obstacle_consistency_map_.end()) {
        obstacle_consistency_map_[obs_id].last_decision = lat_obstacle_decision.at(obs_id);
      }
    }
  }

  UpdateObstacleConsistencyMap(lat_obstacle_decision,
                               obstacle_consistency_map_);
}
// TODO:动态障碍物暂时先使用旧版基于规则的决策，后续方案确定再更改
void HppLateralObstacleDecider::MakeDecisionForSingleDynamicObs(
    const std::shared_ptr<ReferencePath> &reference_path_ptr,
    const std::shared_ptr<FrenetObstacle> &obstacle) {
  const auto &reference_path = session_->planning_context()
                                   .lane_change_decider_output()
                                   .coarse_planning_info.reference_path;
  auto ego_l_max = reference_path->get_frenet_ego_state().boundary().l_end;
  auto ego_l_min = reference_path->get_frenet_ego_state().boundary().l_start;
  auto ego_head_l = reference_path_ptr->get_frenet_ego_state().head_l();
  auto ego_head_s_ = reference_path_ptr->get_frenet_ego_state().head_s();
  const auto &vehicle_param =
      VehicleConfigurationContext::Instance()->get_vehicle_param();
  const auto ego_width = vehicle_param.width;
  const auto half_ego_width = vehicle_param.width * 0.5;
  auto &lat_obstacle_decision = session_->mutable_planning_context()
                                    ->mutable_lateral_obstacle_decider_output()
                                    .lat_obstacle_decision;
  constexpr double kNearFrontThreshold = 7;
  constexpr double kHeadLBuffer = 0.5;
  if (obstacle->b_frenet_valid()) {
    const double obstacle_l_start =
        obstacle->frenet_obstacle_boundary().l_start;
    const double obstacle_l_end = obstacle->frenet_obstacle_boundary().l_end;
    const double obstacle_s_start =
        obstacle->frenet_obstacle_boundary().s_start;
    const double obstacle_s_end = obstacle->frenet_obstacle_boundary().s_end;
    if (EdtManager::FilterObstacleForAra(*obstacle)) {
      double l_buffer = config_.left_l_buffer_for_lat_decision;
      const double ego_head_l_start = ego_head_l - half_ego_width;
      const double ego_head_l_end = ego_head_l + half_ego_width;
      const double ego_s_start =
          reference_path_ptr->get_frenet_ego_state().boundary().s_start;
      const double ego_s_end =
          reference_path_ptr->get_frenet_ego_state().boundary().s_end;
      double start_s = std::max(ego_s_start, obstacle_s_start);
      double end_s = std::min(ego_s_end, obstacle_s_end);
      bool lon_overlap = start_s < end_s;

      // 平行车辆
      if (lon_overlap) {
        ego_head_l = reference_path_ptr->get_frenet_ego_state().head_l();
        double ego_l = reference_path_ptr->get_frenet_ego_state().l();
        const double ego_l_start = ego_l - half_ego_width;
        const double ego_l_end = ego_l + half_ego_width;
        double start_l = std::max(ego_l_start, obstacle_l_start);
        double end_l = std::min(ego_l_end, obstacle_l_end);
        double start_head_l = std::max(ego_head_l_start, obstacle_l_start);
        double end_head_l = std::min(ego_head_l_end, obstacle_l_end);
        constexpr double kLatOverlapBuffer = 0.25;
        bool lat_overlap = (start_l < end_l - kLatOverlapBuffer) &&
                           (start_head_l < end_head_l - kLatOverlapBuffer);

        if (ego_s_end > obstacle_s_end) {
          if (ego_l < obstacle->frenet_l()) {
            lat_obstacle_decision[obstacle->id()] =
                LatObstacleDecisionType::RIGHT;
          } else {
            lat_obstacle_decision[obstacle->id()] =
                LatObstacleDecisionType::LEFT;
          }
        } else {
          if (ego_head_l < obstacle->frenet_l()) {
            lat_obstacle_decision[obstacle->id()] =
                LatObstacleDecisionType::RIGHT;
          } else {
            lat_obstacle_decision[obstacle->id()] =
                LatObstacleDecisionType::LEFT;
          }
        }
        // 防止感知误检，同时有横向和纵向overlap
        if (lat_overlap) {
          lat_obstacle_decision[obstacle->id()] =
              LatObstacleDecisionType::IGNORE;
        }
      } else {
        if (obstacle->frenet_l() > 0) {
          if (obstacle_l_start > -l_buffer) {
            lat_obstacle_decision[obstacle->id()] =
                LatObstacleDecisionType::RIGHT;
          } else {
            lat_obstacle_decision[obstacle->id()] =
                LatObstacleDecisionType::IGNORE;
          }
        } else {
          if (obstacle_l_end < l_buffer) {
            lat_obstacle_decision[obstacle->id()] =
                LatObstacleDecisionType::LEFT;
          } else {
            lat_obstacle_decision[obstacle->id()] =
                LatObstacleDecisionType::IGNORE;
          }
        }
        if (obstacle_s_start > ego_head_s_ &&
            obstacle_s_start < ego_head_s_ + kNearFrontThreshold) {
          if (ego_head_l_start > obstacle_l_end - kHeadLBuffer) {
            lat_obstacle_decision[obstacle->id()] =
                LatObstacleDecisionType::LEFT;
          } else if (ego_head_l_end < obstacle_l_start + kHeadLBuffer) {
            lat_obstacle_decision[obstacle->id()] =
                LatObstacleDecisionType::RIGHT;
          }
        }
      }
    } else {
      lat_obstacle_decision[obstacle->id()] = LatObstacleDecisionType::IGNORE;
    }
  } else {
    lat_obstacle_decision[obstacle->id()] = LatObstacleDecisionType::IGNORE;
  }
  SerializeDynamicObsDecideResultToDebugInfo(obstacle,lat_obstacle_decision[obstacle->id()]);
}

LatObstacleDecisionType HppLateralObstacleDecider::MakeDecisionForSingleCluster(
    const ObstacleCluster &cluster) {
  LatObstacleDecisionType decision = LatObstacleDecisionType::IGNORE;

  // 迟滞参数配置
  const double kBaseSideLock = 0.6;
  const double kMaxHysteresisBuffer = 0.6;
  const double kMaxCenterBuffer = 0.6;

  bool is_static = cluster.motion_types.count(ObstacleMotionType::STATIC) > 0;
  bool is_opposite_moving =
      cluster.motion_types.count(ObstacleMotionType::OPPOSITE_DIR_MOVING) > 0;
  bool is_same_moving =
      cluster.motion_types.count(ObstacleMotionType::SAME_DIR_MOVING) > 0;

  bool is_front =
      cluster.rel_pos_types.count(ObstacleRelPosType::MID_FRONT) > 0 ||
      cluster.rel_pos_types.count(ObstacleRelPosType::LEFT_FRONT) > 0 ||
      cluster.rel_pos_types.count(ObstacleRelPosType::RIGHT_FRONT) > 0;

  bool is_side_rear =
      cluster.rel_pos_types.count(ObstacleRelPosType::LEFT_SIDE) > 0 ||
      cluster.rel_pos_types.count(ObstacleRelPosType::RIGHT_SIDE) > 0 ||
      cluster.rel_pos_types.count(ObstacleRelPosType::MID_BACK) > 0 ||
      cluster.rel_pos_types.count(ObstacleRelPosType::LEFT_BACK) > 0 ||
      cluster.rel_pos_types.count(ObstacleRelPosType::RIGHT_BACK) > 0;

  double current_l =
      (cluster.frenet_boundary.l_start + cluster.frenet_boundary.l_end) / 2.0;
  double right_side = cluster.frenet_boundary.l_start;
  double left_side = cluster.frenet_boundary.l_end;

  if (is_static && is_front) {
    ObstacleConsistencyInfo best_history;
    int max_count = -1;
    for (int obs_id : cluster.original_ids) {
      auto it = obstacle_consistency_map_.find(obs_id);
      if (it != obstacle_consistency_map_.end()) {
        if (it->second.count > max_count) {
          max_count = it->second.count;
          best_history = it->second;
        }
      }
    }

    double growth_factor = 0.0;
    if (best_history.last_decision == LatObstacleDecisionType::LEFT ||
        best_history.last_decision == LatObstacleDecisionType::RIGHT) {
      growth_factor = planning_math::ClampInterpolate(
          1.0, 3.0, 0.0, 1.0, static_cast<double>(best_history.count));
    }

    double dyn_side_threshold =
        kBaseSideLock + growth_factor * kMaxHysteresisBuffer;
    double dyn_center_buffer = kMaxCenterBuffer * growth_factor;

    bool treat_as_left = (current_l > 0.0);

    if (best_history.count >= 2) {
      if (best_history.last_decision == LatObstacleDecisionType::RIGHT) {
        if (current_l > -dyn_center_buffer) treat_as_left = true;
      } else if (best_history.last_decision == LatObstacleDecisionType::LEFT) {
        if (current_l < dyn_center_buffer) treat_as_left = false;
      }
    }

    if (treat_as_left) {
      if (right_side > -dyn_side_threshold)
        decision = LatObstacleDecisionType::RIGHT;
      else
        decision = LatObstacleDecisionType::IGNORE;
    } else {
      if (left_side < dyn_side_threshold)
        decision = LatObstacleDecisionType::LEFT;
      else
        decision = LatObstacleDecisionType::IGNORE;
    }
  } else if (is_static && is_side_rear) {
    if (current_l > 0)
      decision = LatObstacleDecisionType::RIGHT;
    else
      decision = LatObstacleDecisionType::LEFT;
  } else if (is_opposite_moving) {
    if (current_l > 0)
      decision = LatObstacleDecisionType::RIGHT;
    else
      decision = LatObstacleDecisionType::LEFT;
  } else if (is_same_moving) {
    if (current_l > 0)
      decision = LatObstacleDecisionType::RIGHT;
    else
      decision = LatObstacleDecisionType::LEFT;
  }

  return decision;
}

void HppLateralObstacleDecider::MakeDecisionForStaticCluster(
    const ObstacleCluster &cluster,
    const ObstacleConsistencyMap &obstacle_consistency_map,
    const ObstacleClassificationResult &obs_classification_result,
    LatObstacleDecisionType &decision) {
  LatObstacleDecisionInfo passage_width_info;
  LatObstacleDecisionInfo relative_pos_info;
  LatObstacleDecisionInfo last_path_info;
  std::vector<PathPoint> refer_path;

  const auto &frenet_coord = reference_path_ptr_->get_frenet_coord();
  const VehicleParam &vehicle_param =
      VehicleConfigurationContext::Instance()->get_vehicle_param();
  constexpr double kReferEndFrontIgnoreThr = 0.5;
  if (cluster.frenet_boundary.s_start >
      frenet_coord->Length() + vehicle_param.front_edge_to_rear_axle -
          kReferEndFrontIgnoreThr) {
    decision = LatObstacleDecisionType::IGNORE;
    return;
  }

  const bool cluster_is_front =
      cluster.rel_pos_types.count(ObstacleRelPosType::MID_FRONT) > 0 ||
      cluster.rel_pos_types.count(ObstacleRelPosType::LEFT_FRONT) > 0 ||
      cluster.rel_pos_types.count(ObstacleRelPosType::RIGHT_FRONT) > 0;
  const bool cluster_is_side =
      cluster.rel_pos_types.count(ObstacleRelPosType::LEFT_SIDE) > 0 ||
      cluster.rel_pos_types.count(ObstacleRelPosType::RIGHT_SIDE) > 0;
  if (!cluster_is_front && !cluster_is_side) {
    decision = LatObstacleDecisionType::IGNORE;
    return;
  }
  const bool use_relativepos_decision =
      JudgeObsAndEgoInSameStraightLane(reference_path_ptr_, cluster);
  if (cluster_is_side) {
    MakeDecisionBasedPassageWidth(cluster, passage_width_info);
    decision = passage_width_info.decision;
    ILOG_INFO << "passage_width_info.decision = "
              << static_cast<int>(passage_width_info.decision)
              << "  passage_width_info.left_nudge_level = "
              << static_cast<int>(passage_width_info.left_nudge_level)
              << "  passage_width_info.right_nudge_level = "
              << static_cast<int>(passage_width_info.right_nudge_level);
  } else if (!use_relativepos_decision) {
    MakeDecisionBasedPassageWidth(cluster, passage_width_info);
    MakeDecisionBasedReferPath(cluster, obstacle_consistency_map,
                               reference_path_ptr_, refer_path, last_path_info);
    // TODO:弯道需要综合考虑上一次的决策结果和通道宽度决策结果
    decision = passage_width_info.decision;
  } else {
    MakeDecisionBasedPassageWidth(cluster, passage_width_info);
    MakeDecisionBasedRelativePos(cluster, passage_width_info,
                                 relative_pos_info);
    MakeDecisionBasedReferPath(cluster, obstacle_consistency_map,
                               reference_path_ptr_, refer_path, last_path_info);
    MakeFinalDecision(cluster, obstacle_consistency_map, passage_width_info,
                      relative_pos_info, last_path_info, decision);
  }
  SerializeStaticObsDecideResultToDebugInfo(
      cluster, passage_width_info, relative_pos_info, last_path_info, obs_classification_result,decision
      );
}

void HppLateralObstacleDecider::MakeDecisionForDynamicCluster(
    const ObstacleCluster &cluster,
    const ObstacleConsistencyMap &obstacle_consistency_map,
    LatObstacleDecisionType &decision) {
  // MakeDecisionForStaticCluster(cluster, obstacle_consistency_map, decision);
}

void HppLateralObstacleDecider::MakeDecisionBasedPassageWidth(
    const ObstacleCluster &cluster, LatObstacleDecisionInfo &decision_info) {
  std::vector<EnvType> env_type;
  double left_scenario_obs_position_jump_threshold = 0.f,
         right_scenario_obs_position_jump_threshold = 0.f;
  const VehicleParam &vehicle_param =
      VehicleConfigurationContext::Instance()->get_vehicle_param();
  const auto last_decision =
      GetLastDecisionInfo(cluster, obstacle_consistency_map_);

  if (last_decision != LatObstacleDecisionType::IGNORE) {
    const double ego_v = reference_path_ptr_->get_frenet_ego_state().velocity();
    const double cluster_center_s =
        (cluster.frenet_boundary.s_start + cluster.frenet_boundary.s_end) * 0.5;
    QueryTypeInfo type_info = {CRoadType::Turn, CPassageType::Ignore,
                               CElemType::Ignore};
    std::pair<double, double> obs_locate_turn_lane =
        reference_path_ptr_->get_static_analysis_storage()->GetFrontSRange(
            type_info, cluster_center_s);
    if (obs_locate_turn_lane.first < obs_locate_turn_lane.second) {
      env_type.emplace_back(EnvType::TURN);
    }
    type_info = {CRoadType::Ignore, CPassageType::Ignore, CElemType::RampRoad};
    std::pair<double, double> obs_locate_ramp_lane =
        reference_path_ptr_->get_static_analysis_storage()->GetFrontSRange(
            type_info, cluster_center_s);
    if (obs_locate_ramp_lane.first < obs_locate_ramp_lane.second) {
      env_type.emplace_back(EnvType::RAMP);
    }
    if (last_decision == LatObstacleDecisionType::LEFT) {
      left_scenario_obs_position_jump_threshold =
          HPPParameterUtil::CalculateLatBuffer(
              BufferType::BUFFER_TYPE_UNMOVABLE_OBJ, ego_v, 0.f, env_type);
      right_scenario_obs_position_jump_threshold = 0.f;
    } else if (last_decision == LatObstacleDecisionType::RIGHT) {
      right_scenario_obs_position_jump_threshold =
          HPPParameterUtil::CalculateLatBuffer(
              BufferType::BUFFER_TYPE_UNMOVABLE_OBJ, ego_v, 0.f, env_type);
      left_scenario_obs_position_jump_threshold = 0.f;
    }
  } else {
    left_scenario_obs_position_jump_threshold = 0.f;
    right_scenario_obs_position_jump_threshold = 0.f;
  }
  const double ego_extend_width_relative =
      vehicle_param.width +
      hpp_general_lateral_decider_config_.relative_nudge_buffer;
  const double ego_extend_width_absolute =
      vehicle_param.width +
      hpp_general_lateral_decider_config_.absolute_nudge_buffer;
  // left decision
  if (cluster.frenet_boundary.obs_2left_road_boundary_mindis +
          left_scenario_obs_position_jump_threshold <
      ego_extend_width_relative) {
    decision_info.left_nudge_level = LatObstacleNudgeLevel::FORBIDDEN_NUDGE;
  } else if (cluster.frenet_boundary.obs_2left_road_boundary_mindis +
                 left_scenario_obs_position_jump_threshold <
             ego_extend_width_absolute) {
    decision_info.left_nudge_level = LatObstacleNudgeLevel::RELATIVE_NUDGE;
  } else {
    decision_info.left_nudge_level = LatObstacleNudgeLevel::ABSOLUTE_NUDGE;
  }
  // right decision
  if (cluster.frenet_boundary.obs_2right_road_boundary_mindis +
          right_scenario_obs_position_jump_threshold <
      ego_extend_width_relative) {
    decision_info.right_nudge_level = LatObstacleNudgeLevel::FORBIDDEN_NUDGE;
  } else if (cluster.frenet_boundary.obs_2right_road_boundary_mindis +
                 right_scenario_obs_position_jump_threshold <
             ego_extend_width_absolute) {
    decision_info.right_nudge_level = LatObstacleNudgeLevel::RELATIVE_NUDGE;
  } else {
    decision_info.right_nudge_level = LatObstacleNudgeLevel::ABSOLUTE_NUDGE;
  }
  // make final decision
  if (decision_info.right_nudge_level ==
      LatObstacleNudgeLevel::FORBIDDEN_NUDGE) {
    if (decision_info.left_nudge_level ==
            LatObstacleNudgeLevel::ABSOLUTE_NUDGE ||
        decision_info.left_nudge_level ==
            LatObstacleNudgeLevel::RELATIVE_NUDGE) {
      decision_info.decision = LatObstacleDecisionType::LEFT;
    } else {
      decision_info.decision = LatObstacleDecisionType::IGNORE;
    }

  } else if (decision_info.right_nudge_level ==
                 LatObstacleNudgeLevel::RELATIVE_NUDGE ||
             decision_info.right_nudge_level ==
                 LatObstacleNudgeLevel::ABSOLUTE_NUDGE) {
    if (decision_info.left_nudge_level ==
            LatObstacleNudgeLevel::ABSOLUTE_NUDGE ||
        decision_info.left_nudge_level ==
            LatObstacleNudgeLevel::RELATIVE_NUDGE) {
      if (cluster.frenet_boundary.obs_2left_road_boundary_mindis >
          cluster.frenet_boundary.obs_2right_road_boundary_mindis) {
        decision_info.decision = LatObstacleDecisionType::LEFT;
      } else {
        decision_info.decision = LatObstacleDecisionType::RIGHT;
      }
    } else {
      decision_info.decision = LatObstacleDecisionType::RIGHT;
    }
  }
}

void HppLateralObstacleDecider::MakeDecisionBasedRelativePos(
    const ObstacleCluster &cluster,
    const LatObstacleDecisionInfo &previous_decision_info,
    LatObstacleDecisionInfo &decision_info) {
  if (previous_decision_info.left_nudge_level ==
          LatObstacleNudgeLevel::FORBIDDEN_NUDGE &&
      previous_decision_info.right_nudge_level ==
          LatObstacleNudgeLevel::FORBIDDEN_NUDGE) {
    decision_info.left_nudge_level = LatObstacleNudgeLevel::FORBIDDEN_NUDGE;
    decision_info.right_nudge_level = LatObstacleNudgeLevel::FORBIDDEN_NUDGE;
    decision_info.decision = LatObstacleDecisionType::IGNORE;
    return;
  }
  const double cluster_s_start = cluster.frenet_boundary.s_start;
  const double cluster_s_end = cluster.frenet_boundary.s_end;
  const double cluster_l_start = cluster.frenet_boundary.l_start;
  const double cluster_l_end = cluster.frenet_boundary.l_end;
  const double cluster_center_l = (cluster_l_start + cluster_l_end) * 0.5;
  const auto &ego_state = reference_path_ptr_->get_frenet_ego_state();

  //直行轨迹
  if (cluster_center_l > ego_state.l()) {
    const double dis_left = cluster_l_start - ego_state.boundary().l_end;
    if (dis_left > hpp_general_lateral_decider_config_.ego_detour_safe_dis) {
      decision_info.right_nudge_level = LatObstacleNudgeLevel::ABSOLUTE_NUDGE;
      decision_info.left_nudge_level = LatObstacleNudgeLevel::FORBIDDEN_NUDGE;
    } else {
      //曲线轨迹
      AnalyzeNudgeLevelBaseCurve(cluster, previous_decision_info,
                                 decision_info);
    }
  } else if (cluster_center_l < ego_state.l()) {
    const double dis_right =
        ego_state.boundary().l_start - cluster_l_end;
    if (dis_right > hpp_general_lateral_decider_config_.ego_detour_safe_dis) {
      decision_info.left_nudge_level = LatObstacleNudgeLevel::ABSOLUTE_NUDGE;
      decision_info.right_nudge_level = LatObstacleNudgeLevel::FORBIDDEN_NUDGE;
    } else {
      AnalyzeNudgeLevelBaseCurve(cluster, previous_decision_info,
                                 decision_info);
    }
  }

  // make final decision
  if (decision_info.right_nudge_level ==
      LatObstacleNudgeLevel::FORBIDDEN_NUDGE) {
    if (decision_info.left_nudge_level ==
            LatObstacleNudgeLevel::ABSOLUTE_NUDGE ||
        decision_info.left_nudge_level ==
            LatObstacleNudgeLevel::RELATIVE_NUDGE) {
      decision_info.decision = LatObstacleDecisionType::LEFT;
    } else {
      decision_info.decision = LatObstacleDecisionType::IGNORE;
    }
  } else if (decision_info.right_nudge_level ==
                 LatObstacleNudgeLevel::RELATIVE_NUDGE ||
             decision_info.right_nudge_level ==
                 LatObstacleNudgeLevel::ABSOLUTE_NUDGE) {
    if (decision_info.left_nudge_level ==
            LatObstacleNudgeLevel::ABSOLUTE_NUDGE ||
        decision_info.left_nudge_level ==
            LatObstacleNudgeLevel::RELATIVE_NUDGE) {
      decision_info.decision = LatObstacleDecisionType::LEFT;
    } else {
      decision_info.decision = LatObstacleDecisionType::RIGHT;
    }
  }
}

void HppLateralObstacleDecider::AnalyzeNudgeLevelBaseCurve(
    const ObstacleCluster &cluster,
    const LatObstacleDecisionInfo &previous_decision_info,
    LatObstacleDecisionInfo &decision_info) {
  const VehicleParam &vehicle_param =
      VehicleConfigurationContext::Instance()->get_vehicle_param();
  Point2D ideal_target_point_frenet;
  Point2D ego_point_frenet;

  ideal_target_point_frenet.x =
      cluster.frenet_boundary.s_start - vehicle_param.front_edge_to_rear_axle;
  ego_point_frenet.x = reference_path_ptr_->get_frenet_ego_state().s();
  ego_point_frenet.y = reference_path_ptr_->get_frenet_ego_state().l();
  auto cal_kappa = [](const std::shared_ptr<ReferencePath> reference_path_ptr_,
                      const VehicleParam &vehicle_param,
                      const Point2D &ideal_target_point_frenet,
                      const Point2D &ego_point_frenet, const bool is_left,
                      LatObstacleDecisionInfo &decision_info) {
    const double min_radius = vehicle_param.min_turn_radius;
    const double kAbsoluteSafeKappa = 1 / (2.5 * min_radius);
    const double kRelativeSafeKappa = 1 / (1.8 * min_radius);
    double length_AH = 0.0;
    if (is_left) {
      length_AH = (ideal_target_point_frenet.y - ego_point_frenet.y) * 0.5;
    } else {
      length_AH = -(ideal_target_point_frenet.y - ego_point_frenet.y) * 0.5;
    }
    const double squared_AH = std::pow(length_AH, 2);
    const double squared_PH =
        std::pow((ideal_target_point_frenet.x - ego_point_frenet.x * 0.5), 2);
    const double kappa = std::fabs(2 * length_AH) / (squared_AH + squared_PH);
    if (is_left) {
      if (kappa <= kAbsoluteSafeKappa) {
        decision_info.left_nudge_level = LatObstacleNudgeLevel::ABSOLUTE_NUDGE;
      } else if (kappa <= kRelativeSafeKappa) {
        decision_info.left_nudge_level = LatObstacleNudgeLevel::RELATIVE_NUDGE;
      } else {
        decision_info.left_nudge_level = LatObstacleNudgeLevel::FORBIDDEN_NUDGE;
      }
    } else {
      if (kappa <= kAbsoluteSafeKappa) {
        decision_info.right_nudge_level = LatObstacleNudgeLevel::ABSOLUTE_NUDGE;
      } else if (kappa <= kRelativeSafeKappa) {
        decision_info.right_nudge_level = LatObstacleNudgeLevel::RELATIVE_NUDGE;
      } else {
        decision_info.right_nudge_level =
            LatObstacleNudgeLevel::FORBIDDEN_NUDGE;
      }
    }
  };
  // left curve path
  if (previous_decision_info.left_nudge_level !=
      LatObstacleNudgeLevel::FORBIDDEN_NUDGE) {
    ideal_target_point_frenet.y =
        cluster.frenet_boundary.l_end +
        hpp_general_lateral_decider_config_.ego_detour_safe_dis +
        vehicle_param.max_width * 0.5;
    cal_kappa(reference_path_ptr_, vehicle_param, ideal_target_point_frenet,
              ego_point_frenet, true, decision_info);
  } else {
    decision_info.left_nudge_level = LatObstacleNudgeLevel::FORBIDDEN_NUDGE;
  }
  // right curve path
  if (previous_decision_info.right_nudge_level !=
      LatObstacleNudgeLevel::FORBIDDEN_NUDGE) {
    ideal_target_point_frenet.y =
        cluster.frenet_boundary.l_start -
        hpp_general_lateral_decider_config_.ego_detour_safe_dis -
        vehicle_param.max_width * 0.5;
    //TODO:是否在这里加l保护，防止l超过道路边界。
    cal_kappa(reference_path_ptr_, vehicle_param, ideal_target_point_frenet,
              ego_point_frenet, false, decision_info);
  } else {
    decision_info.right_nudge_level = LatObstacleNudgeLevel::FORBIDDEN_NUDGE;
  }
}

bool HppLateralObstacleDecider::JudgeObsAndEgoInSameStraightLane(
    const std::shared_ptr<ReferencePath> &reference_path_ptr,
    const ObstacleCluster &cluster) {
  //判断障碍物和自车是否在同一段直行车道上,仅都在一个直道再考虑相对位置决策
  const auto &ego_state = reference_path_ptr->get_frenet_ego_state();
  const double cluster_s_start = cluster.frenet_boundary.s_start;
  const double cluster_s_end = cluster.frenet_boundary.s_end;
  const double cluster_center_s = (cluster_s_start + cluster_s_end) * 0.5;
  const QueryTypeInfo type_info = {CRoadType::NormalStraight,
                                   CPassageType::Ignore, CElemType::Ignore};
  const std::pair<double, double> ego_locate_straight_lane =
      reference_path_ptr->get_static_analysis_storage()->GetFrontSRange(
          type_info, ego_state.s());
  if (ego_locate_straight_lane.first > ego_locate_straight_lane.second) {
    return false;
  }
  const bool is_ego_locate_straight_lane =
      ego_locate_straight_lane.first < ego_state.s();
  const bool is_cluster_locate_straight_lane =
      cluster_center_s > ego_locate_straight_lane.first &&
      cluster_center_s < ego_locate_straight_lane.second;
  if (is_ego_locate_straight_lane && is_cluster_locate_straight_lane) {
    return true;
  } else {
    return false;
  }
}

void HppLateralObstacleDecider::SerializeStaticObsDecideResultToDebugInfo(
    const ObstacleCluster &cluster,
    const LatObstacleDecisionInfo &passage_width_info,
    const LatObstacleDecisionInfo &relative_pos_info,
    const LatObstacleDecisionInfo &last_path_info,
    const ObstacleClassificationResult &obs_classification_result,
    const LatObstacleDecisionType &decision) {
  auto *hpp_lateral_obstacle_decider_result =
      DebugInfoManager::GetInstance()
          .GetDebugInfoPb()
          ->mutable_hpp_lateral_obstacle_decider_result();
  auto *static_cluster_obstacle_info =
      hpp_lateral_obstacle_decider_result->add_static_obstacle_clusters();
  static_cluster_obstacle_info->set_final_decison_result(
      static_cast<common::LatObstacleDecisionType>(decision));
  for (size_t i = 0; i < cluster.original_ids.size(); i++) {
    auto *original_obs = static_cluster_obstacle_info->add_original_obstacle();
    original_obs->set_original_id(cluster.original_ids.at(i));
    original_obs->set_rel_pos_type(static_cast<common::ObstacleRelPosType>(
        obs_classification_result
            .id_to_rel_pos_type.at(cluster.original_ids.at(i))));
    original_obs->set_motion_type(static_cast<common::ObstacleMotionType>(
        obs_classification_result
            .id_to_motion_type.at(cluster.original_ids.at(i))));
  }
  auto *cluster_obs_boundary =
      static_cluster_obstacle_info->mutable_cluster_obstacle_frenet_boundary();
  double half_l =
      (cluster.frenet_boundary.l_end - cluster.frenet_boundary.l_start) * 0.5;
  cluster_obs_boundary->set_l_end(cluster.frenet_boundary.l_end);
  cluster_obs_boundary->set_l_start(cluster.frenet_boundary.l_start);
  cluster_obs_boundary->set_s_end(cluster.frenet_boundary.s_end);
  cluster_obs_boundary->set_s_start(cluster.frenet_boundary.s_start);
  cluster_obs_boundary->set_obs_2left_road_boundary_mindis(
      cluster.frenet_boundary.obs_2left_road_boundary_mindis - half_l);
  cluster_obs_boundary->set_obs_2right_road_boundary_mindis(
      cluster.frenet_boundary.obs_2right_road_boundary_mindis - half_l);
  if (static_cast<int>(passage_width_info.decision) <= 6 && static_cast<int>(passage_width_info.decision) >= 0) {
    auto *cluster_obs_passage_width_info =
        static_cluster_obstacle_info->mutable_passage_width_info();
    cluster_obs_passage_width_info->set_decision(
        static_cast<common::LatObstacleDecisionType>(
            passage_width_info.decision));
    if(static_cast<int>(passage_width_info.left_nudge_level) >=0 && static_cast<int>(passage_width_info.left_nudge_level) <= 2)
    cluster_obs_passage_width_info->set_left_nudge_level(
        static_cast<common::LatObstacleNudgeLevel>(
            passage_width_info.left_nudge_level));
    if(static_cast<int>(passage_width_info.right_nudge_level) >=0 && static_cast<int>(passage_width_info.right_nudge_level) <= 2)
    cluster_obs_passage_width_info->set_right_nudge_level(
        static_cast<common::LatObstacleNudgeLevel>(
            passage_width_info.right_nudge_level));
  }

  if (static_cast<int>(relative_pos_info.decision) <= 6 && static_cast<int>(relative_pos_info.decision) >= 0) {
    auto *cluster_obs_relative_pos_info =
        static_cluster_obstacle_info->mutable_relative_pos_info();
    cluster_obs_relative_pos_info->set_decision(
        static_cast<common::LatObstacleDecisionType>(
            relative_pos_info.decision));
    if(static_cast<int>(relative_pos_info.left_nudge_level) >=0 && static_cast<int>(relative_pos_info.left_nudge_level) <= 2)
    cluster_obs_relative_pos_info->set_left_nudge_level(
        static_cast<common::LatObstacleNudgeLevel>(
            relative_pos_info.left_nudge_level));
    if(static_cast<int>(relative_pos_info.right_nudge_level) >=0 && static_cast<int>(relative_pos_info.right_nudge_level) <= 2)
    cluster_obs_relative_pos_info->set_right_nudge_level(
        static_cast<common::LatObstacleNudgeLevel>(
            relative_pos_info.right_nudge_level));
  }

  if (static_cast<int>(last_path_info.decision) <= 6 && static_cast<int>(last_path_info.decision) >= 0) {
    auto *cluster_obs_last_path_info =
        static_cluster_obstacle_info->mutable_last_path_info();
    cluster_obs_last_path_info->set_decision(
        static_cast<common::LatObstacleDecisionType>(last_path_info.decision));
    if(static_cast<int>(last_path_info.left_nudge_level) >=0 && static_cast<int>(last_path_info.left_nudge_level) <= 2)
    cluster_obs_last_path_info->set_left_nudge_level(
        static_cast<common::LatObstacleNudgeLevel>(
            last_path_info.left_nudge_level));
    if(static_cast<int>(last_path_info.right_nudge_level) >=0 && static_cast<int>(last_path_info.right_nudge_level) <= 2)
    cluster_obs_last_path_info->set_right_nudge_level(
        static_cast<common::LatObstacleNudgeLevel>(
            last_path_info.right_nudge_level));
  }
}

void HppLateralObstacleDecider::SerializeDynamicObsDecideResultToDebugInfo(
    const std::shared_ptr<FrenetObstacle>& obstacle,
    const LatObstacleDecisionType &decision) {
  auto *hpp_lateral_obstacle_decider_result =
      DebugInfoManager::GetInstance()
          .GetDebugInfoPb()
          ->mutable_hpp_lateral_obstacle_decider_result();
  auto *dynamic_obstacle_info =
      hpp_lateral_obstacle_decider_result->add_dynamic_obstacle();
  dynamic_obstacle_info->set_original_id(obstacle->id());
  dynamic_obstacle_info->set_final_decison_result(static_cast<common::LatObstacleDecisionType>(decision));
}

void HppLateralObstacleDecider::MakeDecisionBasedReferPath(
    const ObstacleCluster &cluster,
    const ObstacleConsistencyMap &obstacle_consistency_map,
    const std::shared_ptr<ReferencePath> &reference_path_ptr,
    const std::vector<PathPoint> &refer_path,
    LatObstacleDecisionInfo &decision_info) {
  ObstacleConsistencyInfo best_history;
  best_history.last_decision = LatObstacleDecisionType::IGNORE;
  int max_count = -1;
  for (int obs_id : cluster.original_ids) {
    auto it = obstacle_consistency_map_.find(obs_id);
    if (it != obstacle_consistency_map_.end()) {
      if (it->second.count > max_count) {
        max_count = it->second.count;
        best_history = it->second;
      }
    }
  }
  decision_info.decision = best_history.last_decision;
}
LatObstacleDecisionType HppLateralObstacleDecider::GetLastDecisionInfo(
    const ObstacleCluster &cluster,
    const ObstacleConsistencyMap &obstacle_consistency_map) {
  ObstacleConsistencyInfo best_history;
  best_history.last_decision = LatObstacleDecisionType::IGNORE;
  int max_count = -1;
  for (int obs_id : cluster.original_ids) {
    auto it = obstacle_consistency_map_.find(obs_id);
    if (it != obstacle_consistency_map_.end()) {
      if (it->second.count > max_count) {
        max_count = it->second.count;
        best_history = it->second;
      }
    }
  }
  return best_history.last_decision;
}

void HppLateralObstacleDecider::MakeFinalDecision(
    const ObstacleCluster &cluster,
    const ObstacleConsistencyMap &obstacle_consistency_map,
    LatObstacleDecisionInfo &passage_width_info,
    LatObstacleDecisionInfo &relative_pos_info,
    LatObstacleDecisionInfo &last_path_info,
    LatObstacleDecisionType &decision) {
  ILOG_INFO << "passage_width_info.decision = "
            << static_cast<int>(passage_width_info.decision)
            << "  passage_width_info.left_nudge_level = "
            << static_cast<int>(passage_width_info.left_nudge_level)
            << "  passage_width_info.right_nudge_level = "
            << static_cast<int>(passage_width_info.right_nudge_level);
  ILOG_INFO << "relative_pos_info.decision = "
            << static_cast<int>(passage_width_info.decision)
            << "  relative_pos_info.left_nudge_level = "
            << static_cast<int>(passage_width_info.left_nudge_level)
            << "  relative_pos_info.right_nudge_level = "
            << static_cast<int>(passage_width_info.right_nudge_level);
  if (passage_width_info.decision == LatObstacleDecisionType::IGNORE ||
      relative_pos_info.decision == LatObstacleDecisionType::IGNORE) {
    decision = LatObstacleDecisionType::IGNORE;
    return;
  }
  if (passage_width_info.left_nudge_level ==
          LatObstacleNudgeLevel::FORBIDDEN_NUDGE ||
      passage_width_info.right_nudge_level ==
          LatObstacleNudgeLevel::FORBIDDEN_NUDGE ||
      relative_pos_info.left_nudge_level ==
          LatObstacleNudgeLevel::FORBIDDEN_NUDGE ||
      relative_pos_info.right_nudge_level ==
          LatObstacleNudgeLevel::FORBIDDEN_NUDGE) {
    if (passage_width_info.left_nudge_level ==
            LatObstacleNudgeLevel::FORBIDDEN_NUDGE ||
        relative_pos_info.left_nudge_level ==
            LatObstacleNudgeLevel::FORBIDDEN_NUDGE) {
      if (passage_width_info.left_nudge_level !=
          relative_pos_info.left_nudge_level) {
        decision = LatObstacleDecisionType::IGNORE;
      } else {
        decision = LatObstacleDecisionType::RIGHT;
      }
    }
    if (passage_width_info.right_nudge_level ==
            LatObstacleNudgeLevel::FORBIDDEN_NUDGE ||
        relative_pos_info.right_nudge_level ==
            LatObstacleNudgeLevel::FORBIDDEN_NUDGE) {
      if (passage_width_info.right_nudge_level !=
          relative_pos_info.right_nudge_level) {
        decision = LatObstacleDecisionType::IGNORE;
      } else {
        decision = LatObstacleDecisionType::LEFT;
      }
    }
  } else {
    if (passage_width_info.decision == relative_pos_info.decision) {
      decision = passage_width_info.decision;
    } else {
      if (passage_width_info.decision == LatObstacleDecisionType::LEFT) {
        if (passage_width_info.left_nudge_level ==
            LatObstacleNudgeLevel::ABSOLUTE_NUDGE) {
          if (relative_pos_info.right_nudge_level !=
              LatObstacleNudgeLevel::ABSOLUTE_NUDGE) {
            decision = passage_width_info.decision;
          } else {
            decision = last_path_info.decision;
          }
        } else {
          if (relative_pos_info.right_nudge_level ==
              LatObstacleNudgeLevel::ABSOLUTE_NUDGE) {
            decision = relative_pos_info.decision;
          } else {
            decision = last_path_info.decision;
          }
        }
      } else {
        if (passage_width_info.right_nudge_level ==
            LatObstacleNudgeLevel::ABSOLUTE_NUDGE) {
          if (relative_pos_info.left_nudge_level !=
              LatObstacleNudgeLevel::ABSOLUTE_NUDGE) {
            decision = passage_width_info.decision;
          } else {
            decision = last_path_info.decision;
          }
        } else {
          if (relative_pos_info.left_nudge_level ==
              LatObstacleNudgeLevel::ABSOLUTE_NUDGE) {
            decision = relative_pos_info.decision;
          } else {
            decision = last_path_info.decision;
          }
        }
      }
    }
  }
  ILOG_INFO << "final decision = " << static_cast<int>(decision);
}

void HppLateralObstacleDecider::UpdateObstacleConsistencyMap(
    const ObstacleLateralDecisionMap &lat_obstacle_decision,
    ObstacleConsistencyMap &obs_consistency_map) {
  constexpr double kMaxIdleTimeMs = 1000.0;
  double current_timestamp = IflyTime::Now_ms();

  for (auto iter = obs_consistency_map.begin();
       iter != obs_consistency_map.end();) {
    auto obs_id = iter->first;
    auto &consistency_info = iter->second;
    if (lat_obstacle_decision.find(obs_id) != lat_obstacle_decision.end()) {
      const auto curr_decision = lat_obstacle_decision.at(obs_id);

      consistency_info.last_seen_timestamp = current_timestamp;
      if (curr_decision == consistency_info.last_decision) {
        consistency_info.count++;
      } else {
        consistency_info.count = 1;
        consistency_info.last_decision = curr_decision;
      }
      ++iter;
    } else {
      double idle_time =
          current_timestamp - consistency_info.last_seen_timestamp;
      if (idle_time > kMaxIdleTimeMs) {
        iter = obs_consistency_map.erase(iter);
      } else {
        ++iter;
      }
    }
  }
}

bool HppLateralObstacleDecider::CheckEnableSearch(
    const std::shared_ptr<ReferencePath> &reference_path_ptr,
    const SearchResult search_result) {
  const VehicleParam &vehicle_param =
      VehicleConfigurationContext::Instance()->get_vehicle_param();

  const auto ego_s = reference_path_ptr->get_frenet_ego_state().s();
  if (config_.enable_hybrid_ara) {
    for (auto &obstacle : reference_path_ptr->get_obstacles()) {
      if (obstacle->b_frenet_valid() &&
          obstacle->frenet_obstacle_boundary().s_end >
              ego_s - vehicle_param.rear_edge_to_rear_axle &&
          obstacle->frenet_obstacle_boundary().s_start - ego_s <
              config_.hybrid_ara_s_range) {
        auto min_abs_l =
            std::min(std::fabs(obstacle->frenet_obstacle_boundary().l_start),
                     std::fabs(obstacle->frenet_obstacle_boundary().l_end));
        double l_threshold = 0.0;
        if (obstacle->type() ==
                iflyauto::ObjectType::OBJECT_TYPE_OCC_GROUDING_WIRE ||
            obstacle->type() == iflyauto::ObjectType::OBJECT_TYPE_COLUMN) {
          l_threshold = config_.column_static_buffer_for_search;
        } else {
          l_threshold = config_.static_buffer_for_search;
        }
        if (search_result == SearchResult::SUCCESS) {
          l_threshold += 0.3;
        }
        if (min_abs_l < l_threshold) {
          return true;
        }
      }
    }
  }
  return false;
}

bool HppLateralObstacleDecider::ARAStar() {
  auto &hybrid_ara_result = session_->mutable_planning_context()
                                ->mutable_lateral_obstacle_decider_output()
                                .hybrid_ara_result;
  hybrid_ara_result.Clear();
  bool find_path = hybrid_ara_star_->Plan(hybrid_ara_result, search_result_);

  // log
  auto &planning_debug_data = DebugInfoManager::GetInstance().GetDebugInfoPb();
  auto hybrid_ara_path =
      planning_debug_data->mutable_hybrid_ara_info()->mutable_hybrid_ara_path();
  auto hybrid_ara_path_cost = planning_debug_data->mutable_hybrid_ara_info()
                                  ->mutable_hybrid_ara_path_cost();
  hybrid_ara_path_cost->Clear();
  for (const auto x : hybrid_ara_result.x) {
    hybrid_ara_path->add_x(x);
  }
  for (const auto y : hybrid_ara_result.y) {
    hybrid_ara_path->add_y(y);
  }
  for (const auto phi : hybrid_ara_result.phi) {
    hybrid_ara_path->add_phi(phi);
  }
  for (const auto s : hybrid_ara_result.s) {
    hybrid_ara_path->add_s(s);
  }
  for (const auto l : hybrid_ara_result.l) {
    hybrid_ara_path->add_l(l);
  }

  return (find_path && hybrid_ara_result.Valid());
}

bool HppLateralObstacleDecider::CheckARAStarPath(
    const ara_star::HybridARAStarResult &result) {
  return false;
}

void HppLateralObstacleDecider::UpdateLatDecisionWithARAStar(
    const std::shared_ptr<ReferencePath> &reference_path_ptr) {
  auto &hybrid_ara_result = session_->mutable_planning_context()
                                ->mutable_lateral_obstacle_decider_output()
                                .hybrid_ara_result;
  auto traj_size = hybrid_ara_result.x.size();
  std::vector<double> s_vec(traj_size);
  std::vector<double> l_vec(traj_size);
  double angle_offset = 0.0;
  bool behind_equal_l_point = true;
  for (size_t i = 0; i < traj_size; ++i) {
    s_vec[i] = hybrid_ara_result.s[i];
    l_vec[i] = hybrid_ara_result.l[i];
  }
  pnc::mathlib::spline l_s_spline;
  l_s_spline.set_points(s_vec, l_vec, pnc::mathlib::spline::linear);

  auto &lat_obstacle_decision = session_->mutable_planning_context()
                                    ->mutable_lateral_obstacle_decider_output()
                                    .lat_obstacle_decision;
  lat_obstacle_decision.clear();
  for (auto &obstacle : reference_path_ptr->get_obstacles()) {
    if (obstacle->b_frenet_valid()) {
      if (EdtManager::FilterObstacleForAra(*obstacle)) {
        double l_ara = 0;
        if (obstacle->frenet_s() < s_vec.front()) {
          l_ara = l_vec.front();
        } else if (obstacle->frenet_s() > s_vec.back()) {
          l_ara = l_vec.back();
        } else {
          l_ara = l_s_spline(obstacle->frenet_s());
        }
        if (obstacle->frenet_l() > l_ara) {
          lat_obstacle_decision[obstacle->id()] =
              LatObstacleDecisionType::RIGHT;
        } else {
          lat_obstacle_decision[obstacle->id()] = LatObstacleDecisionType::LEFT;
        }
      } else {
        lat_obstacle_decision[obstacle->id()] = LatObstacleDecisionType::IGNORE;
      }
    } else {
      lat_obstacle_decision[obstacle->id()] = LatObstacleDecisionType::IGNORE;
    }
  }
}

void HppLateralObstacleDecider::SaveObstaleToEnvironmentModelDebug(
    const std::shared_ptr<ReferencePath> &reference_path_ptr,
    const ObstacleLateralDecisionMap &lat_obstacle_decision) {
#ifdef ENABLE_PROTO_LOG
  auto &planning_debug_data = DebugInfoManager::GetInstance().GetDebugInfoPb();
  auto environment_model_debug_info =
      planning_debug_data->mutable_environment_model_info();
  environment_model_debug_info->clear_obstacle();

  auto add_obstacles = [&](const std::vector<FrenetObstaclePtr> &obstacles) {
    for (const auto obstacle : obstacles) {
      // log
      planning::common::Obstacle *obstacle_log =
          environment_model_debug_info->add_obstacle();
      obstacle_log->set_id(obstacle->id());
      obstacle_log->set_type(obstacle->type());
      obstacle_log->set_is_static(obstacle->is_static());
      if (lat_obstacle_decision.find(obstacle->id()) ==
          lat_obstacle_decision.end()) {
        obstacle_log->set_lat_decision(
            static_cast<uint32_t>(LatObstacleDecisionType::NOT_SET));
      } else {
        obstacle_log->set_lat_decision(
            static_cast<uint32_t>(lat_obstacle_decision.at(obstacle->id())));
      }
      obstacle_log->set_vs_lat_relative(obstacle->frenet_velocity_l());
      obstacle_log->set_vs_lon_relative(obstacle->frenet_velocity_s());

      if (obstacle->source_type() == SourceType::GroundLine ||
          obstacle->source_type() == SourceType::OCC ||
          obstacle->source_type() == SourceType::OD ||
          obstacle->source_type() == SourceType::MAP) {
        for (const auto &polygon :
             obstacle->obstacle()->perception_polygon().points()) {
          planning::common::Point2d *obstacle_polygon =
              obstacle_log->add_polygon_points();
          obstacle_polygon->set_x(polygon.x());
          obstacle_polygon->set_y(polygon.y());
        }
      }
    }
  };
  add_obstacles(reference_path_ptr->get_obstacles());
  add_obstacles(reference_path_ptr->get_turnstile_obstacles());
  add_obstacles(reference_path_ptr->get_speed_bump_obstacles());
#endif
}

}  // namespace planning