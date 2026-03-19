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
#include "src/library/reeds_shepp/reeds_shepp.h"

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
  //double current_timestamp = IflyTime::Now_ms();
  //std::unordered_set<uint32_t> current_frame_ids;
  //for (const auto& pair : obs_item_map) current_frame_ids.insert(pair.first);
  //ClearOldConsistencyInfo(current_frame_ids, current_timestamp);

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
    if (cluster.motion_types.empty() || cluster.rel_pos_types.empty()) continue;

    LatObstacleDecisionType decision = MakeDecisionForSingleCluster(cluster);
    for (const auto &obs_id : cluster.original_ids) {
      lat_obstacle_decision[obs_id] = decision;
    }
  }
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
    LatObstacleDecisionType &decision) {}

void HppLateralObstacleDecider::MakeDecisionForDynamicCluster(
    const ObstacleCluster &cluster,
    const ObstacleConsistencyMap &obstacle_consistency_map,
    LatObstacleDecisionType &decision) {
  MakeDecisionForStaticCluster(cluster, obstacle_consistency_map, decision);
}

void HppLateralObstacleDecider::MakeDecisionBasedPassageWidth(
     ObstacleCluster &cluster, LatObstacleDecisionInfo &decision_info) {

  //TODO:这部分是否移动到障碍物的预处理中计算
  const double kRelativeNudgeBuffer = 0.8;
  const double kAbsoluteNudgeBuffer = 2.0;
  const double kPositionJumpThreshold = 0.0;//TODO:阈值需要改为动态

  const auto &reference_path_ptr = session_->planning_context()
                                       .lane_change_decider_output()
                                       .coarse_planning_info.reference_path;
  const VehicleParam &vehicle_param =
      VehicleConfigurationContext::Instance()->get_vehicle_param();
  const double cluster_s_start = cluster.frenet_boundary.s_start;
  const double cluster_s_end = cluster.frenet_boundary.s_end;
  const double cluster_l_start = cluster.frenet_boundary.l_start;
  const double cluster_l_end = cluster.frenet_boundary.l_end;
  const double cluster_center_l = (cluster_l_start + cluster_l_end) * 0.5;

  ReferencePathPoint refpath_pt;

  reference_path_ptr->get_reference_point_by_lon(cluster_s_start, refpath_pt);
  const double obs_start2left_road_boundary_dis =
      refpath_pt.distance_to_left_road_border - cluster_l_end;
  const double obs_start2right_road_boundary_dis =
      refpath_pt.distance_to_right_road_border + cluster_l_start;

  reference_path_ptr->get_reference_point_by_lon(cluster_s_end, refpath_pt);
  const double obs_end2left_road_boundary_dis =
      refpath_pt.distance_to_left_road_border - cluster_l_end;
  const double obs_end2right_road_boundary_dis =
      refpath_pt.distance_to_right_road_border + cluster_l_start;

  const double obs_2left_road_boundary_mindis =
      std::max(obs_start2left_road_boundary_dis,
               obs_end2left_road_boundary_dis);  //距离带正负
  const double obs_2right_road_boundary_mindis = std::min(
      obs_start2right_road_boundary_dis, obs_end2right_road_boundary_dis);
  ILOG_INFO << "obs_2left_road_boundary_mindis = "
            << obs_2left_road_boundary_mindis
            << ", obs_2right_road_boundary_mindis = "
            << obs_2right_road_boundary_mindis;
  cluster.frenet_boundary.obs_2left_road_boundary_mindis = obs_2left_road_boundary_mindis;
  cluster.frenet_boundary.obs_2right_road_boundary_mindis = obs_2right_road_boundary_mindis;
  //TODO:

  //left decision
  if (obs_2left_road_boundary_mindis <
      vehicle_param.width + kRelativeNudgeBuffer + kPositionJumpThreshold) {
    decision_info.left_nudge_level = LatObstacleNudgeLevel::FORBIDDEN_NUDGE;
  } else if (obs_2left_road_boundary_mindis <
             vehicle_param.width + kAbsoluteNudgeBuffer + kPositionJumpThreshold) {
    decision_info.left_nudge_level = LatObstacleNudgeLevel::RELATIVE_NUDGE;
  } else {
    decision_info.left_nudge_level = LatObstacleNudgeLevel::ABSOLUTE_NUDGE;
  }
  //right decision
  if (obs_2right_road_boundary_mindis <=
      vehicle_param.width + kRelativeNudgeBuffer + kPositionJumpThreshold) {
    decision_info.right_nudge_level = LatObstacleNudgeLevel::FORBIDDEN_NUDGE;
  } else if (obs_2right_road_boundary_mindis <
             vehicle_param.width + kAbsoluteNudgeBuffer + kPositionJumpThreshold) {
    decision_info.right_nudge_level = LatObstacleNudgeLevel::RELATIVE_NUDGE;
  } else {
    decision_info.right_nudge_level = LatObstacleNudgeLevel::ABSOLUTE_NUDGE;
  }
  //make final decision
  if (decision_info.right_nudge_level ==
      LatObstacleNudgeLevel::FORBIDDEN_NUDGE) {
    if (decision_info.left_nudge_level ==
            LatObstacleNudgeLevel::ABSOLUTE_NUDGE ||
        decision_info.left_nudge_level ==
            LatObstacleNudgeLevel::RELATIVE_NUDGE) {
      decision_info.decision = LatObstacleDecisionType::LEFT;
    } else if (decision_info.left_nudge_level ==
               LatObstacleNudgeLevel::FORBIDDEN_NUDGE) {
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
      if (obs_2left_road_boundary_mindis == obs_2right_road_boundary_mindis){
        decision_info.decision = LatObstacleDecisionType::LEFT;//超车靠左
      }else if (obs_2left_road_boundary_mindis > obs_2right_road_boundary_mindis)
      {
        decision_info.decision = LatObstacleDecisionType::LEFT;
      }else{
        decision_info.decision = LatObstacleDecisionType::RIGHT;
      }
    } else if (decision_info.left_nudge_level ==
               LatObstacleNudgeLevel::FORBIDDEN_NUDGE) {
      decision_info.decision = LatObstacleDecisionType::RIGHT;
    }
  }
}

void HppLateralObstacleDecider::MakeDecisionBasedRelativePos(
    const ObstacleCluster &cluster,
    const LatObstacleDecisionInfo &passage_width_info,
    LatObstacleDecisionInfo &decision_info) {
  if (passage_width_info.left_nudge_level ==
          LatObstacleNudgeLevel::FORBIDDEN_NUDGE &&
      passage_width_info.right_nudge_level ==
          LatObstacleNudgeLevel::FORBIDDEN_NUDGE) {
    decision_info.left_nudge_level = LatObstacleNudgeLevel::FORBIDDEN_NUDGE;
    decision_info.right_nudge_level = LatObstacleNudgeLevel::FORBIDDEN_NUDGE;
    decision_info.decision = LatObstacleDecisionType::IGNORE;
    return;
  }

  const double kAvoidanceSafeDis = 0.4;

  const double cluster_s_start = cluster.frenet_boundary.s_start;
  const double cluster_s_end = cluster.frenet_boundary.s_end;
  const double cluster_l_start = cluster.frenet_boundary.l_start;
  const double cluster_l_end = cluster.frenet_boundary.l_end;
  const double cluster_center_l = (cluster_l_start + cluster_l_end) * 0.5;
  const auto &ego_state = reference_path_ptr_->get_frenet_ego_state();

  //直行轨迹
  if (cluster_center_l > ego_state.l()) {
    const double dis_left = cluster_l_start - ego_state.boundary().l_end;
    if (dis_left > kAvoidanceSafeDis + 0.6) {
      decision_info.left_nudge_level = LatObstacleNudgeLevel::ABSOLUTE_NUDGE;
    } else if (dis_left > kAvoidanceSafeDis) {
      decision_info.left_nudge_level = LatObstacleNudgeLevel::RELATIVE_NUDGE;
    } else {
      //曲线轨迹
      AnalyzeNudgeLevelBaseCurve(cluster,passage_width_info,decision_info);
    }
  } else if (cluster_center_l < ego_state.l()) {
    const double dis_right =
        ego_state.boundary().l_start - ego_state.boundary().l_end;
    if (dis_right > kAvoidanceSafeDis + 0.6) {
      decision_info.right_nudge_level = LatObstacleNudgeLevel::ABSOLUTE_NUDGE;
    } else if (dis_right > kAvoidanceSafeDis) {
      decision_info.right_nudge_level = LatObstacleNudgeLevel::RELATIVE_NUDGE;
    } else {
      //曲线轨迹
      AnalyzeNudgeLevelBaseCurve(cluster,passage_width_info,decision_info);
    }
  }

  //make final decision


}

void HppLateralObstacleDecider::MakeDecisionBasedLastPath(
    const ObstacleCluster &cluster, LatObstacleDecisionInfo &decision_info) {}

void HppLateralObstacleDecider::MakeFinalDecision(
    const ObstacleCluster &cluster,
    const ObstacleConsistencyMap &obstacle_consistency_map,
    LatObstacleDecisionInfo &passage_width_info,
    LatObstacleDecisionInfo &relative_pos_info,
    LatObstacleDecisionInfo &last_path_info,
    LatObstacleDecisionType &decision) {}

void HppLateralObstacleDecider::AnalyzeNudgeLevelBaseCurve(
    const ObstacleCluster &cluster,
    const LatObstacleDecisionInfo &passage_width_info,
    LatObstacleDecisionInfo &decision_info) {
  const double kAvoidanceSafeDis = 0.4;
  const double kSampleStep = 0.2;
  const double kSampleStepInv = 5;
  Pose2D start_point;  // TODO:起点也应该采样
  std::vector<Pose2D> end_points;
  double theta = 0;
  Point2D end_point_frenet;
  Point2D end_point_cartesian;
  Polygon2D ego_polygon;


  const VehicleParam &vehicle_param =
      VehicleConfigurationContext::Instance()->get_vehicle_param();
  const auto &frenet_coord = reference_path_ptr_->get_frenet_coord();

  start_point.x =
      session_->environmental_model().get_ego_state_manager()->ego_pose().x;
  start_point.y =
      session_->environmental_model().get_ego_state_manager()->ego_pose().y;
  start_point.theta =
      session_->environmental_model().get_ego_state_manager()->ego_pose().theta;
  const uint32_t num_s = static_cast<uint32_t>(
      (cluster.frenet_boundary.s_end - cluster.frenet_boundary.s_start) *
      kSampleStepInv);
  // right curve path left
  if (passage_width_info.left_nudge_level !=
      LatObstacleNudgeLevel::FORBIDDEN_NUDGE) {
    std::vector<Pose2D> left_path_points;
    const uint32_t num_l = static_cast<uint32_t>(
        (cluster.frenet_boundary.obs_2left_road_boundary_mindis -
         vehicle_param.width - kAvoidanceSafeDis * 2) *
        kSampleStepInv);
    for (uint32_t i = 0; i < num_s; i++) {
      end_point_frenet.x = cluster.frenet_boundary.s_start + kSampleStep * i;
      // TODO:确认航向是否正确
      theta = (frenet_coord->GetPathPointByS(end_point_frenet.x)).theta;
      for (uint32_t j = 0; j < num_l; j++) {
        end_point_frenet.y = cluster.frenet_boundary.l_end + kAvoidanceSafeDis +
                             vehicle_param.width * 0.5 + j * kSampleStep;
        frenet_coord->SLToXY(end_point_frenet, end_point_cartesian);
        end_points.emplace_back(
            Pose2D(end_point_cartesian.x, end_point_cartesian.y, theta));
      }
    }
  } else {
    decision_info.left_nudge_level = LatObstacleNudgeLevel::FORBIDDEN_NUDGE;
  }
  // right curve path
  if (passage_width_info.right_nudge_level !=
      LatObstacleNudgeLevel::FORBIDDEN_NUDGE) {
    std::vector<Pose2D> right_path_points;
    if (!end_points.empty()) std::vector<Pose2D>().swap(end_points);
    const uint32_t num_r = static_cast<uint32_t>(
        (cluster.frenet_boundary.obs_2right_road_boundary_mindis -
         vehicle_param.width - kAvoidanceSafeDis * 2) *
        kSampleStepInv);
    for (uint32_t i = 0; i < num_s; i++) {
      end_point_frenet.x = cluster.frenet_boundary.s_start + kSampleStep * i;
      theta = (frenet_coord->GetPathPointByS(end_point_frenet.x)).theta;
      for (uint32_t j = 0; j < num_r; j++) {
        end_point_frenet.y = cluster.frenet_boundary.l_start -
                             kAvoidanceSafeDis - vehicle_param.width * 0.5 -
                             j * kSampleStep;
        frenet_coord->SLToXY(end_point_frenet, end_point_cartesian);
        end_points.emplace_back(
            Pose2D(end_point_cartesian.x, end_point_cartesian.y, theta));
      }
    }
  } else {
    decision_info.right_nudge_level = LatObstacleNudgeLevel::FORBIDDEN_NUDGE;
  }
}


bool HppLateralObstacleDecider::LRSCurve(const double min_line_length, double x, double y, double theta, double &t, double &u, double &v) {
  const double cos_theta = std::cos(theta);
  const double sin_heta = std::sin(theta);
  const double xi = x + sin_heta;
  const double eta = y - 1.0 - cos_theta;
  const double u1 = (xi * sin_heta - eta * cos_theta) * 0.5;
  const double kZero = 1e-20;

  double vs1 = 0.0;
  double vs2 = 0.0;
  bool validity = false;

  theta = NormalizeAngle(theta);
  if (u1 >= -1 && u1 <= 1) {
    const double acos_u1 = std::acos(u1);
    const double ts1 = NormalizeAngle(acos_u1 + theta);
    const double us1 = NormalizeAngle(ts1 - theta);
    if (std::fabs(std::fabs(theta) - M_PI * 0.5) > M_PI * 0.25) {
      vs1 = (xi - 2 * std::sin(ts1)) / cos_theta;

    } else {
      vs1 = (eta + 2 * std::cos(ts1)) / sin_heta;
    }
    const double ts2 = NormalizeAngle(-acos_u1 + theta);
    const double us2 = NormalizeAngle(ts2 - theta);
    if (std::fabs(std::fabs(theta) - M_PI * 0.5) > M_PI * 0.25) {
      vs2 = (xi - 2 * std::sin(ts2)) / cos_theta;

    } else {
      vs2 = (eta + 2 * std::cos(ts2)) / sin_heta;
    }

    if (ts1 < -kZero && us1 <= -kZero && vs1 <= -min_line_length) {
      t = ts1;
      u = us1;
      v = vs1;
      validity = true;
    } else if (ts2 < -kZero && us2 <= -kZero && vs2 <= -min_line_length) {
      t = ts2;
      u = us2;
      v = vs2;
      validity = true;
    } else {
    }
  }
  return validity;
}

bool HppLateralObstacleDecider::CalSCurve(const Pose2D& start_point,
                                          const Pose2D& end_point,
                                          const double radius,const bool is_left_path,std::vector<Pose2D>& path_points) {

  const double dx = end_point.x - start_point.x;
  const double dy = end_point.y - start_point.y;
  const double theta_rel = end_point.theta - start_point.theta;
  const double x_rel = (std::cos(start_point.theta) * dx + std::sin(start_point.theta) * dy) / radius;
  const double y_rel = (-std::sin(start_point.theta) * dx + std::cos(start_point.theta) * dy) / radius;
  const double min_line_length = 0.3 / radius;
  const double l_min = 1.0e6;;
  const double step_size = 0.5 / radius;//采样步长
  double t = 0;
  double u = 0;
  double v = 0;
  double unit_path_length = 0;
  double path_segment_num = 0;
  bool is_curve_valid = false;
  RSPathParam path;
  if (is_left_path) {
    if (LRSCurve(min_line_length,x_rel,-y_rel,-theta_rel,t,u,v))  // reflect
    {
      unit_path_length = std::fabs(t) + std::fabs(u) + std::fabs(v);
      if (l_min > unit_path_length) {
        SetRSPathParam(&path,ReedsSheppPathype[21],t,u,v,0,0);
        is_curve_valid = true;
        l_min = unit_path_length;
      }
    }

    if (LRSCurve(min_line_length,-x_rel,-y_rel,theta_rel,t,u,v))  // timeflip+reflect
    {
      unit_path_length = std::fabs(t) + std::fabs(u) + std::fabs(v);
      if (l_min > unit_path_length) {
        SetRSPathParam(&path,ReedsSheppPathype[21],-t,-u,-v,0,0);
        is_curve_valid = true;
        l_min = unit_path_length;
      }
    }
  } else {
    if (LRSCurve(min_line_length, x_rel, y_rel, theta_rel, t, u, v)) {
      unit_path_length = std::fabs(t) + std::fabs(u) + std::fabs(v);
      if (l_min > unit_path_length) {
        is_curve_valid = true;
        SetRSPathParam(&path,ReedsSheppPathype[20],t,u,v,0,0);
        l_min = unit_path_length;
      }
    }
    if (LRSCurve(min_line_length,-x_rel,y_rel,-theta_rel,t,u,v))  // timeflip
    {
      unit_path_length = std::fabs(t) + std::fabs(u) + std::fabs(v);
      if (l_min > unit_path_length) {
        is_curve_valid = true;
        SetRSPathParam(&path,ReedsSheppPathype[20],-t,-u,-v,0,0);
        l_min = unit_path_length;
      }
    }
  }
  //插值
  auto RSCurveInterpolate = [](const double seg_length, const double radius,
                               const Pose2D &start_point,
                               const RSPathParam &path,
                               std::vector<Pose2D> &path_points) {
    Pose2D path_point;

    switch (path.type[i]) {
      case RSPathSteer::RS_LEFT:
        path_point.x = (std::sin(start_point.theta + seg_length) -
                        std::sin(start_point.theta));
        path_point.y = (-std::cos(start_point.theta + seg_length) +
                        std::cos(start_point.theta));
        path_point.theta = start_point.theta + seg_length;
        break;
      case RSPathSteer::RS_RIGHT:
        path_point.x = (-std::sin(start_point.theta - seg_length) +
                        std::sin(start_point.theta));
        path_point.y = (std::cos(start_point.theta - seg_length) -
                        std::cos(start_point.theta));
        path_point.theta = start_point.theta - seg_length;
        break;
      case RSPathSteer::RS_STRAIGHT:
        path_point.x = seg_length * std::cos(start_point.theta);
        path_point.y = seg_length * std::sin(start_point.theta);
        break;
    }
    path_point.x = path_point.x * radius + start_point.x;
    path_point.y = path_point.y * radius + start_point.y;
  };
  if (is_curve_valid) {
    path.length[0] = t;
    path.length[1] = u;
    path.length[2] = v;
    for (size_t i = 0; i < 5; i++) {
      if (path.type[i] != RS_NOP) {
        path_segment_num++
      };
    }
    for (size_t i = 0; i < path_segment_num; i++) {
      double seg_length = 0.0;
      for (seg_length = step_size; seg_length < path.length[i];
           seg_length += step_size) {
            RSCurveInterpolate(seg_length,radius,start_point,path,path_points)；
      }
    }
  }

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
    const ara_star::HybridARAStarResult& result) {
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
          lat_obstacle_decision[obstacle->id()] = LatObstacleDecisionType::RIGHT;
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