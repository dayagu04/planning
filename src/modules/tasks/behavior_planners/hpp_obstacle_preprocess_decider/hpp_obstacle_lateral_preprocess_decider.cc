#include <list>
#include <algorithm>
#include "hpp_obstacle_lateral_preprocess_decider.h"
#include "edt_manager.h"
#include "planning_context.h"
#include "utils/cartesian_coordinate_system.h"

namespace planning {

HppObstacleLateralPreprocessDecider::HppObstacleLateralPreprocessDecider(
    const EgoPlanningConfigBuilder* config_builder, framework::Session* session)
    : Task(config_builder, session) {
  name_ = "HppObstacleLateralPreprocessDecider";
  config_ = config_builder->cast<HppObstacleLateralPreprocessDeciderConfig>();
}
bool HppObstacleLateralPreprocessDecider::Execute() {
  if (!PreCheck()) {
    ILOG_DEBUG << "PreCheck failed";
    return false;
  }

  const auto fix_lane_virtual_id = session_->planning_context()
                                       .lane_change_decider_output()
                                       .fix_lane_virtual_id;
  auto reference_path_ptr =
      session_->environmental_model()
          .get_reference_path_manager()
          ->get_reference_path_by_lane(fix_lane_virtual_id, false);
  if (reference_path_ptr == nullptr) {
    return false;
  }

  auto& hpp_obs_lat_preprocess_output =
      session_->mutable_planning_context()
          ->mutable_hpp_obstacle_lat_preprocess_output();
  auto& obs_cluster_container =
      hpp_obs_lat_preprocess_output.obs_cluster_container;
  auto& obs_classification_result =
      hpp_obs_lat_preprocess_output.obs_classification_result;

  ObstacleItemMap obs_item_map;
  // 1. 障碍物过滤
  if (!GenerateObstaclesToBeConsidered(reference_path_ptr, obs_item_map)) {
    return false;
  }

  // 2. 障碍物分类
  const auto& ego_state = reference_path_ptr->get_frenet_ego_state();
  if (!ClassifyObstacles(obs_item_map, ego_state, obs_classification_result)) {
    return false;
  }

  // 3: 聚类 (动静分离 + 规则聚类 + 凸包生成)
  if (!ClusterObstacles(obs_item_map, obs_classification_result,
                        obs_cluster_container)) {
    return false;
  }

  // 对聚类结果进行排序
  std::sort(obs_cluster_container.obstacle_clusters.begin(),
            obs_cluster_container.obstacle_clusters.end(),
            [](const ObstacleCluster& a, const ObstacleCluster& b) {
              return a.frenet_boundary.s_start < b.frenet_boundary.s_start;
            });

  // 4: 过闸机场景识别、分类和预处理
  auto& turnstile_scene_info =
      hpp_obs_lat_preprocess_output.turnstile_scene_info;
  if (!JudgeTurnstileScene(reference_path_ptr, ego_state, turnstile_scene_info)) {
    return false;
  }
  reference_path_ptr->set_turnstile_scene_info(turnstile_scene_info);
  return true;
}

bool HppObstacleLateralPreprocessDecider::GenerateObstaclesToBeConsidered(
    ConstReferencePathPtr reference_path_ptr, ObstacleItemMap& obs_item_map) {
  if (!reference_path_ptr) {
    return false;
  }
  obs_item_map.clear();

  for (const auto& obstacle : reference_path_ptr->get_obstacles()) {
    if (!obstacle->b_frenet_valid()) {
      continue;
    }
    obs_item_map[obstacle->id()] = obstacle;
  }
  return true;
}

bool HppObstacleLateralPreprocessDecider::ClassifyObstacles(
    const ObstacleItemMap& obs_item_map, const FrenetEgoState& ego_state,
    ObstacleClassificationResult& classification_result) {
  classification_result.Clear();
  auto& rel_pos_type_to_ids = classification_result.rel_pos_type_to_ids;
  auto& motion_type_to_ids = classification_result.motion_type_to_ids;
  auto& id_to_rel_pos_type = classification_result.id_to_rel_pos_type;
  auto& id_to_motion_type = classification_result.id_to_motion_type;

  for (const auto& entry : obs_item_map) {
    const auto& obs = entry.second;
    const auto rel_pos_type = ClassifyObstaclesByRelPos(ego_state, obs);
    const auto motion_type = ClassifyObstaclesByMotion(obs);
    rel_pos_type_to_ids[rel_pos_type].push_back(obs->id());
    motion_type_to_ids[motion_type].push_back(obs->id());
    id_to_rel_pos_type[obs->id()] = rel_pos_type;
    id_to_motion_type[obs->id()] = motion_type;
  }
  return true;
}

bool HppObstacleLateralPreprocessDecider::ClusterObstacles(
    const ObstacleItemMap& obs_item_map,
    const ObstacleClassificationResult& classification_result,
    ObstacleClusterContainer& obstacle_cluster_container) {
  obstacle_cluster_container.obs_id_to_cluster_id.clear();
  obstacle_cluster_container.obstacle_clusters.clear();

  std::vector<ObstacleClusterCandicate> cluster_candidates;
  if (!GenerateClusterCandicates(obs_item_map, classification_result,
                                 cluster_candidates)) {
    return false;
  }

  ObstacleClusterGraph obstacle_cluster_graph;
  if (!CalculateCandidateClusterGraph(obs_item_map, cluster_candidates,
                                      obstacle_cluster_graph)) {
    return false;
  }

  std::vector<ObstacleCluster> obstacle_clusters;
  std::unordered_set<int> visited_candidate_idxs;
  for (int idx = 0; idx < cluster_candidates.size(); ++idx) {
    if (visited_candidate_idxs.count(idx) == 0) {
      ObstacleCluster obstacle_cluster;
      obstacle_cluster.cluster_id = cluster_candidates[idx].origin_id;
      DFSGenerateObstacleClusters(cluster_candidates, obstacle_cluster_graph,
                                  idx, ObstacleClusterType::NO_MERGE,
                                  visited_candidate_idxs, obstacle_cluster);
      obstacle_clusters.push_back(std::move(obstacle_cluster));
    }
  }

  for (auto& obstacle_cluster : obstacle_clusters) {
    if (!BuildObstacleClusterConvexHull(obs_item_map, obstacle_cluster)) {
      continue;
    }
    CalObstacleClusterInfo(obstacle_cluster);
    obstacle_cluster_container.obstacle_clusters.push_back(obstacle_cluster);
  }

  return true;
}

bool HppObstacleLateralPreprocessDecider::JudgeTurnstileScene(
    ConstReferencePathPtr reference_path_ptr, const FrenetEgoState& ego_state,
    TurnstileSceneInfo& turnstile_scene_info) {
  constexpr double kTurnstileLonThr1 = 15.0;
  constexpr double kTurnstileLonThr2 = 1.0;
  constexpr double kTurnstileLatThr = 5.0;
  constexpr double kTurnstileLatSafeThr = 0.2;
  if (!reference_path_ptr) {
    return false;
  }
  const auto& veh_param =
      VehicleConfigurationContext::Instance()->get_vehicle_param();
  const double vel_max_width = veh_param.max_width;
  const auto& ego_sl_boundary = ego_state.boundary();
  // S1: 判断前方是否存在闸机
  const auto& static_analysis_result = reference_path_ptr->get_static_analysis_storage();
  QueryTypeInfo turnstile_type_info = {CRoadType::Ignore, CPassageType::Ignore,
                                       CElemType::TurnStileRoad};
  const auto front_turnstile_range =
      static_analysis_result->GetFrontSRange(turnstile_type_info, ego_state.s());
  if (front_turnstile_range.first > front_turnstile_range.second ||
      front_turnstile_range.first > ego_sl_boundary.s_end + kTurnstileLonThr1) {
    return true;
  }

  // S2：获取不同位置的闸机
  const auto& turnstile_osbatcles =
      reference_path_ptr->get_turnstile_obstacles();
  std::vector<ConstFrenetObstaclePtr> l_turnstiles;
  std::vector<ConstFrenetObstaclePtr> r_turnstiles;
  std::vector<ConstFrenetObstaclePtr> m_turnstiles;
  for(const auto& turnstile_osbatcle : turnstile_osbatcles) {
    const auto& sl_boundary = turnstile_osbatcle->frenet_obstacle_boundary();
    if (std::min(std::fabs(sl_boundary.l_start), std::fabs(sl_boundary.l_end)) >
            kTurnstileLatThr ||
        sl_boundary.s_start > ego_sl_boundary.s_end + kTurnstileLonThr1 ||
        sl_boundary.s_end < ego_sl_boundary.s_start - kTurnstileLonThr2) {
      continue;
    }
    const auto width = reference_path_ptr->get_interpolated_point_width(
        turnstile_osbatcle->frenet_s(), true);
    const double left_border_l = width.first;
    const double right_border_l = -width.second;
    const double mid_l = (left_border_l + right_border_l) / 2.0;
    if (left_border_l - sl_boundary.l_end >
            vel_max_width + kTurnstileLatSafeThr &&
        sl_boundary.l_start - right_border_l >
            vel_max_width + kTurnstileLatSafeThr) {
      m_turnstiles.push_back(turnstile_osbatcle);
    } else if (sl_boundary.l_start - right_border_l >
               vel_max_width + kTurnstileLatSafeThr) {
      l_turnstiles.push_back(turnstile_osbatcle);
    } else if (left_border_l - sl_boundary.l_end >
               vel_max_width + kTurnstileLatSafeThr) {
      r_turnstiles.push_back(turnstile_osbatcle);
    } else {
      ILOG_ERROR << "block turnstile obsatcle id : "
                 << turnstile_osbatcle->id();
    }
  }
  const auto sort_fun = [](const ConstFrenetObstaclePtr& a,
                           const ConstFrenetObstaclePtr& b) {
    return a->frenet_obstacle_boundary().s_start <
           b->frenet_obstacle_boundary().s_start;
  };
  std::sort(l_turnstiles.begin(), l_turnstiles.end(), sort_fun);
  std::sort(r_turnstiles.begin(), r_turnstiles.end(), sort_fun);
  std::sort(m_turnstiles.begin(), m_turnstiles.end(), sort_fun);

  // S3: 判断闸机场景和目标闸机
  if(l_turnstiles.empty() && r_turnstiles.empty() && m_turnstiles.empty()) {
    turnstile_scene_info.type = TurnstileSceneType::TURNSTILE_SCENE_NONE;
  } else if(m_turnstiles.empty() == false) {
    if (l_turnstiles.empty() && r_turnstiles.empty()) {
      turnstile_scene_info.type =
          TurnstileSceneType::TURNSTILE_SCENE_MID_DOUBLE;
      turnstile_scene_info.target_id = m_turnstiles.front()->id();
    } else if (r_turnstiles.empty()) {
      turnstile_scene_info.type =
          TurnstileSceneType::TURNSTILE_SCENE_LEFT_SINGLE;
      turnstile_scene_info.target_id = m_turnstiles.front()->id();
    } else {
      turnstile_scene_info.type =
          TurnstileSceneType::TURNSTILE_SCENE_SIDE_DOUBLE;
      turnstile_scene_info.target_id = r_turnstiles.front()->id();
      turnstile_scene_info.side_id = m_turnstiles.front()->id();
    }
  } else {
    if(l_turnstiles.empty() == false && r_turnstiles.empty() == false) {
      turnstile_scene_info.type =
          TurnstileSceneType::TURNSTILE_SCENE_SIDE_DOUBLE;
      turnstile_scene_info.target_id = r_turnstiles.front()->id();
      turnstile_scene_info.side_id = l_turnstiles.front()->id();
    } else if(l_turnstiles.empty() == false) {
      turnstile_scene_info.type =
          TurnstileSceneType::TURNSTILE_SCENE_LEFT_SINGLE;
      turnstile_scene_info.target_id = l_turnstiles.front()->id();
    } else {
      turnstile_scene_info.type =
          TurnstileSceneType::TURNSTILE_SCENE_RIGHT_SINGLE;
      turnstile_scene_info.target_id = r_turnstiles.front()->id();
    }
  }
  return true;
}

void HppObstacleLateralPreprocessDecider::CalObstacleClusterInfo(
    ObstacleCluster& obstacle_cluster) {
  const auto& reference_path_ptr = session_->planning_context()
                                       .lane_change_decider_output()
                                       .coarse_planning_info.reference_path;
  const VehicleParam& vehicle_param =
      VehicleConfigurationContext::Instance()->get_vehicle_param();
  const double cluster_s_start = obstacle_cluster.frenet_boundary.s_start;
  const double cluster_s_end = obstacle_cluster.frenet_boundary.s_end;
  const double cluster_l_start = obstacle_cluster.frenet_boundary.l_start;
  const double cluster_l_end = obstacle_cluster.frenet_boundary.l_end;
  const double cluster_center_l = (cluster_l_start + cluster_l_end) * 0.5;

  ReferencePathPoint refpath_pt;
  reference_path_ptr->get_reference_point_by_lon(cluster_s_start, refpath_pt);
  if (cluster_l_start > -refpath_pt.distance_to_right_road_border &&
      cluster_l_end < refpath_pt.distance_to_left_road_border) {
    const double obs_start2left_road_boundary_dis =
        refpath_pt.left_drivable_width - cluster_l_end;
    const double obs_start2right_road_boundary_dis =
        refpath_pt.right_drivable_width + cluster_l_start;

    reference_path_ptr->get_reference_point_by_lon(cluster_s_end, refpath_pt);
    const double obs_end2left_road_boundary_dis =
        refpath_pt.left_drivable_width - cluster_l_end;
    const double obs_end2right_road_boundary_dis =
        refpath_pt.right_drivable_width + cluster_l_start;

    const double obs_2left_road_boundary_mindis =
        std::max(obs_start2left_road_boundary_dis,
                 obs_end2left_road_boundary_dis);  //距离带正负
    const double obs_2right_road_boundary_mindis = std::min(
        obs_start2right_road_boundary_dis, obs_end2right_road_boundary_dis);
    ILOG_INFO << "obs_2left_road_boundary_mindis = "
              << obs_2left_road_boundary_mindis
              << ", obs_2right_road_boundary_mindis = "
              << obs_2right_road_boundary_mindis;
    obstacle_cluster.frenet_boundary.obs_2left_road_boundary_mindis =
        obs_2left_road_boundary_mindis;
    obstacle_cluster.frenet_boundary.obs_2right_road_boundary_mindis =
        obs_2right_road_boundary_mindis;
  } else {
    if (cluster_l_end >= refpath_pt.distance_to_left_road_border &&
        cluster_l_start <= refpath_pt.distance_to_left_road_border) {
      obstacle_cluster.frenet_boundary.obs_2left_road_boundary_mindis = 0.f;
      obstacle_cluster.frenet_boundary.obs_2right_road_boundary_mindis =
          cluster_l_start + refpath_pt.right_drivable_width;
    }
    if (cluster_l_start <= -refpath_pt.distance_to_right_road_border &&
        cluster_l_end >= -refpath_pt.distance_to_right_road_border) {
      obstacle_cluster.frenet_boundary.obs_2left_road_boundary_mindis =
          refpath_pt.left_drivable_width - cluster_l_end;
      obstacle_cluster.frenet_boundary.obs_2right_road_boundary_mindis = 0.f;
    }
  }
}
ObstacleRelPosType
HppObstacleLateralPreprocessDecider::ClassifyObstaclesByRelPos(
    const FrenetEgoState& ego_state, const FrenetObstaclePtr& obs_ptr) {
  const double kFarAwayLonFrontThr = 50.0;  // 判断障碍物在自车前方远处
  const double kFarAwayLonBackThr = 5.0;  // 判断障碍物在自车后方远处
  const double kFarAwayLatAbsThr = 5.0;  // 判断障碍物在引导线侧向远处
  const double kFarAwayLatRelThr = 5.0;  // 判断障碍物在自车侧向远处
  const double kSideObsFrontThr = 0.0;  // 判断障碍物是否和自车并排的前方阈值
  const double kSideObsBackThr = 0.0;  // 判断障碍物是否和自车并排的后方阈值
  const double kMidObsAbsThr = 2.0;
  const double kMidObsRelThr = 1.0;

  const double ego_start_s = ego_state.boundary().s_start;
  const double ego_end_s = ego_state.boundary().s_end;
  const double ego_start_l = ego_state.boundary().l_start;
  const double ego_end_l = ego_state.boundary().l_end;
  const double ego_l = ego_state.l();
  const auto& obs_boundary = obs_ptr->frenet_obstacle_boundary();
  const double obs_start_s = obs_boundary.s_start;
  const double obs_end_s = obs_boundary.s_end;
  const double obs_start_l = obs_boundary.l_start;
  const double obs_end_l = obs_boundary.l_end;
  const double obs_l = obs_ptr->frenet_l();

  ObstacleRelPosType type = ObstacleRelPosType::FAR_AWAY;
  bool is_lon_far_away = (obs_start_s > ego_end_s + kFarAwayLonFrontThr ||
                          obs_end_s < ego_start_s - kFarAwayLonBackThr);
  bool is_lat_far_away = (obs_start_l > std::fmax(ego_end_l + kFarAwayLatRelThr,
                                                  kFarAwayLatAbsThr) ||
                          obs_end_l < std::fmin(ego_start_l - kFarAwayLatRelThr,
                                                -kFarAwayLatAbsThr));

  if (is_lon_far_away && is_lat_far_away) {
    type = ObstacleRelPosType::FAR_AWAY;
  } else if (obs_start_s > ego_end_s + kSideObsFrontThr) {
    if (obs_start_l > std::fmax(ego_end_l + kMidObsRelThr, kMidObsAbsThr)) {
      type = ObstacleRelPosType::LEFT_FRONT;
    } else if (obs_end_l <
               std::fmin(ego_start_l - kMidObsRelThr, -kMidObsAbsThr)) {
      type = ObstacleRelPosType::RIGHT_FRONT;
    } else {
      type = ObstacleRelPosType::MID_FRONT;
    }
  } else if (obs_end_s < ego_start_s - kSideObsBackThr) {
    if (obs_start_l > std::fmax(ego_end_l + kMidObsRelThr, kMidObsAbsThr)) {
      type = ObstacleRelPosType::LEFT_BACK;
    } else if (obs_end_l <
               std::fmin(ego_start_l - kMidObsRelThr, -kMidObsAbsThr)) {
      type = ObstacleRelPosType::RIGHT_BACK;
    } else {
      type = ObstacleRelPosType::MID_BACK;
    }
  } else {
    if (obs_l > ego_l) {
      type = ObstacleRelPosType::LEFT_SIDE;
    } else {
      type = ObstacleRelPosType::RIGHT_SIDE;
    }
  }
  return type;
}

ObstacleMotionType
HppObstacleLateralPreprocessDecider::ClassifyObstaclesByMotion(
    const FrenetObstaclePtr& obs_ptr) {
  const double obs_relative_v_angle =
      std::fabs(obs_ptr->frenet_relative_velocity_angle());

  ObstacleMotionType type = ObstacleMotionType::STATIC;
  if (obs_ptr->is_static()) {
    type = ObstacleMotionType::STATIC;
  } else {
    if (obs_relative_v_angle > 3 * PI / 4) {
      type = ObstacleMotionType::OPPOSITE_DIR_MOVING;
    } else if (obs_relative_v_angle < PI / 4) {
      type = ObstacleMotionType::SAME_DIR_MOVING;
    } else {
      type = ObstacleMotionType::CROSSING_MOVING;
    }
  }
  return type;
}

bool HppObstacleLateralPreprocessDecider::GenerateClusterCandicates(
    const ObstacleItemMap& obs_item_map,
    const ObstacleClassificationResult& classification_result,
    std::vector<ObstacleClusterCandicate>& cluster_candidates) {
  cluster_candidates.clear();
  constexpr double kLowSpeedPedestrainThr = 0.5;
  const auto& id_to_rel_pos_type = classification_result.id_to_rel_pos_type;
  const auto& id_to_motion_type = classification_result.id_to_motion_type;

  for (const auto& entry : obs_item_map) {
    const auto& obs_id = entry.first;
    const auto& obs = entry.second;

    if (id_to_motion_type.find(obs_id) == id_to_motion_type.end()) continue;
    if (id_to_rel_pos_type.find(obs_id) == id_to_rel_pos_type.end()) continue;

    const auto& rel_pos_type = id_to_rel_pos_type.at(obs_id);
    const auto& motion_type = id_to_motion_type.at(obs_id);
    if (motion_type == ObstacleMotionType::STATIC) {
      if (rel_pos_type == ObstacleRelPosType::FAR_AWAY) continue;
    } else {
      if (rel_pos_type == ObstacleRelPosType::FAR_AWAY) continue;
      if (!obs->obstacle()->is_pedestrain() ||
          obs->velocity() > kLowSpeedPedestrainThr) {
        continue;
      }
    }

    ObstacleClusterCandicate candidate;
    candidate.origin_id = obs_id;
    candidate.rel_pos_types = rel_pos_type;
    candidate.motion_type = motion_type;
    cluster_candidates.push_back(candidate);
  }
  return true;
}

bool HppObstacleLateralPreprocessDecider::CalculateCandidateClusterGraph(
    const ObstacleItemMap& obs_item_map,
    const std::vector<ObstacleClusterCandicate>& cluster_candidates,
    ObstacleClusterGraph& cluster_graph) {
  constexpr double kMergeLonLargeThr = 3.0;
  constexpr double kMergeLatLargeThr = 2.0;
  constexpr double kMergeLonSmallThr = 0.5;
  constexpr double kMergeLatSmallThr = 0.2;
  constexpr double kMergeAbsDisThr = 2.0;

  for (int i = 0; i < cluster_candidates.size(); ++i) {
    const auto& obs_i_id = cluster_candidates[i].origin_id;
    const auto& obs_i = obs_item_map.at(obs_i_id);
    const auto& obs_i_boundary = obs_i->frenet_obstacle_boundary();
    const double obs_i_start_s = obs_i_boundary.s_start;
    const double obs_i_end_s = obs_i_boundary.s_end;
    const double obs_i_start_l = obs_i_boundary.l_start;
    const double obs_i_end_l = obs_i_boundary.l_end;

    auto& cluster_map_i = cluster_graph[i];
    for (int j = i + 1; j < cluster_candidates.size(); ++j) {
      const auto& obs_j_id = cluster_candidates[j].origin_id;
      const auto& obs_j = obs_item_map.at(obs_j_id);
      auto& cluster_map_j = cluster_graph[j];
      //调用类型判断函数，不符合规定类型的不进行合并
      if (!IsObstacleTypeMergeable(obs_i->type(), obs_j->type())) {
        cluster_map_i[j] = ObstacleClusterType::NO_MERGE;
        cluster_map_j[i] = ObstacleClusterType::NO_MERGE;
        continue;
      }

      const auto& obs_j_boundary = obs_j->frenet_obstacle_boundary();
      const double obs_j_start_s = obs_j_boundary.s_start;
      const double obs_j_end_s = obs_j_boundary.s_end;
      const double obs_j_start_l = obs_j_boundary.l_start;
      const double obs_j_end_l = obs_j_boundary.l_end;

      double s_gap =
          std::fmax(obs_i_start_s - obs_j_end_s, obs_j_start_s - obs_i_end_s);
      double l_gap =
          std::fmax(obs_i_start_l - obs_j_end_l, obs_j_start_l - obs_i_end_l);

      bool lon_meet_cond1 = s_gap < kMergeLonLargeThr;
      bool lon_meet_cond2 = s_gap < kMergeLonSmallThr;
      bool lat_meet_cond1 = l_gap < kMergeLatLargeThr;
      bool lat_meet_cond2 = l_gap < kMergeLatSmallThr;

      if (lon_meet_cond1 && lat_meet_cond1) {
        double dist = obs_i->obstacle()->perception_polygon().DistanceTo(
            obs_j->obstacle()->perception_polygon());
        if (dist < kMergeAbsDisThr) {
          cluster_map_i[j] = ObstacleClusterType::ABS_MERGE;
          cluster_map_j[i] = ObstacleClusterType::ABS_MERGE;
          continue;
        }
      }
      if (lon_meet_cond1 && lat_meet_cond2) {
        cluster_map_i[j] = ObstacleClusterType::LON_MERGE;
        cluster_map_j[i] = ObstacleClusterType::LON_MERGE;
        continue;
      }
      if (lon_meet_cond2 && lat_meet_cond1) {
        cluster_map_i[j] = ObstacleClusterType::LAT_MERGE;
        cluster_map_j[i] = ObstacleClusterType::LAT_MERGE;
        continue;
      }
      cluster_map_i[j] = ObstacleClusterType::NO_MERGE;
      cluster_map_j[i] = ObstacleClusterType::NO_MERGE;
    }
  }
  return true;
}

bool HppObstacleLateralPreprocessDecider::IsObstacleTypeMergeable(
    const iflyauto::ObjectType& type_a,
    const iflyauto::ObjectType& type_b) {
  // 规则1：车辆只能和车辆合并
  bool is_a_vehicle = (type_a == iflyauto::ObjectType::OBJECT_TYPE_COUPE ||
                      type_a == iflyauto::ObjectType::OBJECT_TYPE_MINIBUS ||
                      type_a == iflyauto::ObjectType::OBJECT_TYPE_VAN ||
                      type_a == iflyauto::ObjectType::OBJECT_TYPE_BUS ||
                      type_a == iflyauto::ObjectType::OBJECT_TYPE_TRUCK ||
                      type_a == iflyauto::ObjectType::OBJECT_TYPE_TRAILER ||
                      type_a == iflyauto::ObjectType::OBJECT_TYPE_PICKUP ||
                      type_a == iflyauto::ObjectType::OBJECT_TYPE_SUV ||
                      type_a == iflyauto::ObjectType::OBJECT_TYPE_MPV ||
                      type_a == iflyauto::ObjectType::OBJECT_TYPE_ENGINEERING_VEHICLE ||
                      type_a == iflyauto::ObjectType::OBJECT_TYPE_OCC_CAR);
  bool is_b_vehicle = (type_b == iflyauto::ObjectType::OBJECT_TYPE_COUPE ||
                      type_b == iflyauto::ObjectType::OBJECT_TYPE_MINIBUS ||
                      type_b == iflyauto::ObjectType::OBJECT_TYPE_VAN ||
                      type_b == iflyauto::ObjectType::OBJECT_TYPE_BUS ||
                      type_b == iflyauto::ObjectType::OBJECT_TYPE_TRUCK ||
                      type_b == iflyauto::ObjectType::OBJECT_TYPE_TRAILER ||
                      type_b == iflyauto::ObjectType::OBJECT_TYPE_PICKUP ||
                      type_b == iflyauto::ObjectType::OBJECT_TYPE_SUV ||
                      type_b == iflyauto::ObjectType::OBJECT_TYPE_MPV ||
                      type_b == iflyauto::ObjectType::OBJECT_TYPE_ENGINEERING_VEHICLE ||
                      type_b == iflyauto::ObjectType::OBJECT_TYPE_OCC_CAR);

  // 如果一个是车，一个不是 → 不允许合并
  if (is_a_vehicle != is_b_vehicle) {
    return false;
  }
  // ===================== 新的类型扩展=====================
  // 例：规则2：接地线不和其他类型合并

  return true; // 类型匹配 → 允许继续判断距离
}

bool HppObstacleLateralPreprocessDecider::DFSGenerateObstacleClusters(
    const std::vector<ObstacleClusterCandicate>& cluster_candidates,
    const ObstacleClusterGraph& cluster_graph, const int curr_idx,
    const ObstacleClusterType curr_cluster_type,
    std::unordered_set<int>& visited_candidate_idxs,
    ObstacleCluster& obstacle_cluster) {
  visited_candidate_idxs.insert(curr_idx);
  obstacle_cluster.original_ids.push_back(
      cluster_candidates[curr_idx].origin_id);
  obstacle_cluster.motion_types.insert(
      cluster_candidates[curr_idx].motion_type);
  obstacle_cluster.rel_pos_types.insert(
      cluster_candidates[curr_idx].rel_pos_types);
  obstacle_cluster.cluster_types.insert(curr_cluster_type);

  static auto is_valid_cluster_type =
      [](const ObstacleClusterType& new_type,
         const std::unordered_set<ObstacleClusterType>& cluster_types) {
        if (new_type == ObstacleClusterType::NO_MERGE) {
          return false;
        } else if (new_type == ObstacleClusterType::LAT_MERGE) {
          return cluster_types.find(ObstacleClusterType::LON_MERGE) ==
                 cluster_types.end();
        } else if (new_type == ObstacleClusterType::LON_MERGE) {
          return cluster_types.find(ObstacleClusterType::LAT_MERGE) ==
                 cluster_types.end();
        } else {
          return true;
        }
      };

  auto it = cluster_graph.find(curr_idx);
  if (it != cluster_graph.end()) {
    const auto& neighbors_map = it->second;
    for (const auto& neighbor : neighbors_map) {
      const auto& cluster_idx = neighbor.first;
      const auto& cluster_type = neighbor.second;

      if (visited_candidate_idxs.find(cluster_idx) ==
          visited_candidate_idxs.end()) {
        if (is_valid_cluster_type(cluster_type,
                                  obstacle_cluster.cluster_types)) {
          DFSGenerateObstacleClusters(cluster_candidates, cluster_graph,
                                      cluster_idx, cluster_type,
                                      visited_candidate_idxs, obstacle_cluster);
        }
      }
    }
  }
  return true;
}

bool HppObstacleLateralPreprocessDecider::BuildObstacleClusterConvexHull(
    const ObstacleItemMap& obs_item_map, ObstacleCluster& obstacle_cluster) {
  auto& new_points = obstacle_cluster.perception_points;
  auto& new_sl_boundary = obstacle_cluster.frenet_boundary;

  for (const auto& origin_id : obstacle_cluster.original_ids) {
    if (obs_item_map.find(origin_id) == obs_item_map.end()) {
      continue;
    }
    const auto& obs_ptr = obs_item_map.at(origin_id);
    const auto& perception_points = obs_ptr->obstacle()->perception_points();
    const auto& frenet_boundary = obs_ptr->frenet_obstacle_boundary();
    new_points.insert(new_points.end(), perception_points.begin(),
                      perception_points.end());
    new_sl_boundary.l_end =
        std::fmax(new_sl_boundary.l_end, frenet_boundary.l_end);
    new_sl_boundary.l_start =
        std::fmin(new_sl_boundary.l_start, frenet_boundary.l_start);
    new_sl_boundary.s_end =
        std::fmax(new_sl_boundary.s_end, frenet_boundary.s_end);
    new_sl_boundary.s_start =
        std::fmin(new_sl_boundary.s_start, frenet_boundary.s_start);
  }

  double min_x = std::numeric_limits<double>::max();
  double max_x = std::numeric_limits<double>::lowest();
  double min_y = std::numeric_limits<double>::max();
  double max_y = std::numeric_limits<double>::lowest();
  std::for_each(new_points.begin(), new_points.end(), [&](const auto& point) {
    min_x = std::min(min_x, point.x());
    max_x = std::max(max_x, point.x());
    min_y = std::min(min_y, point.y());
    max_y = std::max(max_y, point.y());
  });

  auto& new_polygon = obstacle_cluster.polygon;
  if (new_points.size() < 3) {
    new_points.clear();
    new_points.emplace_back(min_x, min_y);
    new_points.emplace_back(min_x, max_y);
    new_points.emplace_back(max_x, min_y);
    new_points.emplace_back(max_x, max_y);
  }
  if (!planning_math::Polygon2d::ComputeConvexHull(new_points, &new_polygon)) {
    return false;
  }

  obstacle_cluster.bounding_box = new_polygon.MinAreaBoundingBox();
  return true;
}
}  // namespace planning