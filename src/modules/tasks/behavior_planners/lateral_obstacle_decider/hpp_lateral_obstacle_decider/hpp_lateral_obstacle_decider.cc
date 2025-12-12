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
#include "obstacle_manager.h"
#include "planning_context.h"
#include "src/modules/tasks/behavior_planners/general_lateral_decider/general_lateral_decider_utils.h"
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

void HppLateralObstacleDecider::UpdateLatDecision(
    const std::shared_ptr<ReferencePath> &reference_path_ptr) {
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
  lat_obstacle_decision.clear();
  constexpr double kNearFrontThreshold = 7;
  constexpr double kHeadLBuffer = 0.5;
  for (auto &obstacle : reference_path_ptr->get_obstacles()) {
    if (obstacle->b_frenet_valid()) {
      const double obstacle_l_start =
          obstacle->frenet_obstacle_boundary().l_start;
      const double obstacle_l_end = obstacle->frenet_obstacle_boundary().l_end;
      const double obstacle_s_start =
          obstacle->frenet_obstacle_boundary().s_start;
      const double obstacle_s_end = obstacle->frenet_obstacle_boundary().s_end;
      if (EdtManager::FilterObstacleForAra(*obstacle)) {
        double l_buffer = 0;
        if (obstacle->obstacle()->type() ==
                iflyauto::ObjectType::OBJECT_TYPE_COLUMN ||
            obstacle->obstacle()->type() ==
                iflyauto::ObjectType::OBJECT_TYPE_OCC_GROUDING_WIRE) {
          l_buffer = config_.column_l_buffer_for_decision;
        } else {
          l_buffer = config_.l_buffer_for_lat_decision;
        }

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
  }
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