#include "hpp_lateral_obstacle_decider.h"
#include "hpp_lateral_obstacle_utils.h"

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
#include "obstacle_manager.h"
#include "planning_context.h"
#include "modules/tasks/behavior_planners/general_lateral_decider/general_lateral_decider_utils.h"
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
}

bool HppLateralObstacleDecider::Execute() {
  if (!PreCheck()) {
    output_.clear();
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
    UpdateLatDecision(reference_path_ptr);
  } else {
    UpdateLatDecisionWithARAStar(reference_path_ptr);
  }

  Log(reference_path_ptr);

  return true;
}

void HppLateralObstacleDecider::UpdateLatDecision(
    const std::shared_ptr<ReferencePath> &reference_path_ptr) {

  const auto &vehicle_param = VehicleConfigurationContext::Instance()->get_vehicle_param();
  const auto& ego_state = reference_path_ptr->get_frenet_ego_state();
  double ego_head_l = ego_state.head_l();
  double ego_s = ego_state.s();
  double ego_v = ego_state.velocity_s();

  auto &lat_obstacle_decision = session_->mutable_planning_context()
                                    ->mutable_lateral_obstacle_decider_output()
                                    .lat_obstacle_decision;
  lat_obstacle_decision.clear();
  double current_timestamp = IflyTime::Now_ms();

  // 1. 数据准备
  ObstacleItemMap obs_item_map;
  bool has_obs = HppLateralObstacleUtils::GenerateObstaclesToBeConsidered( reference_path_ptr, obs_item_map);

  std::unordered_set<uint32_t> current_frame_ids;
  for (const auto& pair : obs_item_map) current_frame_ids.insert(pair.first);
  ClearOldConsistencyInfo(current_frame_ids, current_timestamp);

  if (!has_obs) return;

  // 2. 物理聚类
  MergedObstacleContainer merged_container;
  HppLateralObstacleUtils::MergeObstaclesBaseOnPos( obs_item_map, merged_container);

  // 3. 排序
  HppLateralObstacleUtils::SortMergedObstacles(merged_container);

  // 4. 分类
  ObstacleClassificationResult classification_result;
  HppLateralObstacleUtils::ClassifyObstacles( merged_container, ego_s, ego_v, classification_result);

  // 5. 静态障碍物一致性更新 (ROI 不足则合并 Group ID)
  HppLateralObstacleUtils::UpdateStaticObsConsistency( merged_container, classification_result, ego_head_l);

  // 6. 最终决策生成
  // 6.1 按 consistency_group_id 将 Cluster 分组
  std::unordered_map<int, std::vector<const MergedObstacleInfo*>> consistency_groups;
  for (const auto& cluster : merged_container.merged_obstacles) {
      consistency_groups[cluster.consistency_group_id].push_back(&cluster);
  }

  // 6.2 标记 Group 的属性 (根据包含的 Cluster 类型)
  // 优先级：前方静止 > 侧方静止 > 其他
  std::unordered_set<int> front_static_group_ids;
  std::unordered_set<int> side_static_group_ids;

  // 遍历前方静止列表，标记对应的 Group ID
  for (const auto* obs : classification_result.front_static_obs) {
      front_static_group_ids.insert(obs->consistency_group_id);
  }
  // 遍历侧方静止列表，标记对应的 Group ID
  for (const auto* obs : classification_result.side_static_obs) {
      side_static_group_ids.insert(obs->consistency_group_id);
  }

  // 参数配置
  const double kMaxHysteresisBuffer = 0.6;
  const double kMaxHysteresisCenteringBuffer = 0.6;
  const double kBaseSideLock = 0.6;

  for (const auto& entry : consistency_groups) {
      int group_id = entry.first;
      const auto& group_list = entry.second;
      if (group_list.empty()) continue;

      LatObstacleDecisionType decision = LatObstacleDecisionType::IGNORE;
      if (front_static_group_ids.count(group_id)) {
          const MergedObstacleInfo* dominant_cluster = nullptr;
          double max_abs_l = -1.0;
          for (const auto* c : group_list) {
              if (std::abs(c->center_l()) > max_abs_l) {
                  max_abs_l = std::abs(c->center_l());
                  dominant_cluster = c;
              }
          }
          if (!dominant_cluster) continue;

          double current_l = dominant_cluster->center_l();
          double right_side = dominant_cluster->l_start;
          double left_side = dominant_cluster->l_end;

          ObstacleConsistencyInfo best_history_info;
          int max_count = -1;
          for (const auto* c : group_list) {
              for (int obs_id : c->original_ids) {
                  if (obstacle_consistency_map_.find(obs_id) != obstacle_consistency_map_.end()) {
                      auto info = obstacle_consistency_map_[obs_id];
                      if (info.count > max_count) {
                          max_count = info.count;
                          best_history_info = info;
                      }
                  }
              }
          }

          double growth_factor = 0.0;
          if (best_history_info.last_decision == LatObstacleDecisionType::LEFT ||
              best_history_info.last_decision == LatObstacleDecisionType::RIGHT) {
              growth_factor = (best_history_info.count <= 1) ? 0.0 :
                              (best_history_info.count >= 3) ? 1.0 : 0.5;
          }

          double current_side_lock_threshold = kBaseSideLock + growth_factor * kMaxHysteresisBuffer;
          double current_center_buffer = kMaxHysteresisCenteringBuffer * growth_factor;
          bool treat_as_left_obstacle = (current_l > 0.0);

          if (best_history_info.count >= 2) {
              if (best_history_info.last_decision == LatObstacleDecisionType::RIGHT) {
                  if (current_l > -current_center_buffer) {
                    treat_as_left_obstacle = true;}
              } else if (best_history_info.last_decision == LatObstacleDecisionType::LEFT) {
                  if (current_l < current_center_buffer) {
                    treat_as_left_obstacle = false;}
              }
          }

          if (treat_as_left_obstacle) {
              if (right_side > -current_side_lock_threshold) {
                  decision = LatObstacleDecisionType::RIGHT;
              } else {
                  decision = LatObstacleDecisionType::IGNORE;
              }
          } else {
              if (left_side < current_side_lock_threshold) {
                  decision = LatObstacleDecisionType::LEFT;
              } else {
                  decision = LatObstacleDecisionType::IGNORE;
              }
          }
      }
      else if (side_static_group_ids.count(group_id)) {
          const MergedObstacleInfo* dominant_cluster = nullptr;
          double max_abs_l = -1.0;
          for (const auto* c : group_list) {
              if (std::abs(c->center_l()) > max_abs_l) {
                  max_abs_l = std::abs(c->center_l());
                  dominant_cluster = c;
              }
          }

          if (dominant_cluster) {
              if (dominant_cluster->center_l() > 0) {
                  decision = LatObstacleDecisionType::RIGHT;
              } else {
                  decision = LatObstacleDecisionType::LEFT;
              }
          }
      }
      // 动态三个逻辑待补充
      else {
          decision = LatObstacleDecisionType::IGNORE;
      }

      for (const auto* c : group_list) {
          for (int obs_id : c->original_ids) {
              auto& info = obstacle_consistency_map_[obs_id];
              info.last_seen_timestamp = current_timestamp;

              if (decision == info.last_decision) {
                  if (info.count < 100) info.count++;
              } else {
                  info.count = 1;
                  info.last_decision = decision;
              }
              lat_obstacle_decision[obs_id] = info.last_decision;
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

void HppLateralObstacleDecider::ClearOldConsistencyInfo(
    const std::unordered_set<uint32_t>& current_frame_ids,
    double current_timestamp) {

    const double kMaxIdleTimeMs = 1000.0;

    for (auto it = obstacle_consistency_map_.begin(); it != obstacle_consistency_map_.end(); ) {
        uint32_t obs_id = it->first;

        if (current_frame_ids.find(obs_id) != current_frame_ids.end()) {
            it->second.last_seen_timestamp = current_timestamp;
            ++it;
        } else {
            double idle_time = current_timestamp - it->second.last_seen_timestamp;
            if (idle_time > kMaxIdleTimeMs) {
                it = obstacle_consistency_map_.erase(it);
            } else {
                ++it;
            }
        }
    }
}

void HppLateralObstacleDecider::Log(
    const std::shared_ptr<ReferencePath> &reference_path_ptr) {
#ifdef ENABLE_PROTO_LOG
  auto &planning_debug_data = DebugInfoManager::GetInstance().GetDebugInfoPb();
  auto environment_model_debug_info =
      planning_debug_data->mutable_environment_model_info();
  environment_model_debug_info->clear_obstacle();
  auto &lat_obstacle_decision = session_->mutable_planning_context()
                                    ->mutable_lateral_obstacle_decider_output()
                                    .lat_obstacle_decision;
  for (auto &obstacle : reference_path_ptr->get_obstacles()) {
    // log
    planning::common::Obstacle *obstacle_log =
        environment_model_debug_info->add_obstacle();
    obstacle_log->set_id(obstacle->id());
    obstacle_log->set_type(obstacle->type());
    obstacle_log->set_is_static(obstacle->is_static());
    obstacle_log->set_lat_decision(
        static_cast<uint32_t>(lat_obstacle_decision[obstacle->id()]));
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
#endif
}

}  // namespace planning