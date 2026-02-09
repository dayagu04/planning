#include "hpp_lateral_obstacle_utils.h"
#include "edt_manager.h"
#include "utils/cartesian_coordinate_system.h"

namespace planning {

bool HppLateralObstacleUtils::GenerateObstaclesToBeConsidered(
    ConstReferencePathPtr reference_path_ptr,
    ObstacleItemMap& obs_item_map) {

  if (!reference_path_ptr) return false;
  obs_item_map.clear();

  for (const auto& obstacle : reference_path_ptr->get_obstacles()) {
    if (!obstacle->b_frenet_valid()) continue;
    //TODO: 这里可以添加更多过滤条件
    obs_item_map[obstacle->id()] = obstacle;
  }
  return !obs_item_map.empty();
}

std::unordered_map<int, ObstacleTypeBeforeCluster> HppLateralObstacleUtils::ClassifyObstacles(
    const ObstacleItemMap& obs_item_map,
    const double ego_s,
    const double ego_v) {

    std::unordered_map<int, ObstacleTypeBeforeCluster> type_map;

    // 阈值配置
    const double kHighSpeedDiff = 3.0;  // 同向前方：如果比我快太多，忽略
    const double kLowSpeedThresh = 2.0; // 同向后方：如果比我慢太多(被甩开)，忽略

    for (const auto& entry : obs_item_map) {
        const auto& obs = entry.second;
        int id = entry.first;
        ObstacleTypeBeforeCluster type = ObstacleTypeBeforeCluster::IGNORE;

        if (obs->is_static()) {
            if (obs->frenet_s() > ego_s) {
                type = ObstacleTypeBeforeCluster::FRONT_STATIC;
            } else {
                type = ObstacleTypeBeforeCluster::SIDE_REAR_STATIC;
            }
        } else {
            double v_s = obs->frenet_velocity_s();
            double heading_angle = obs->frenet_relative_velocity_angle();   //这里默认angle是[-pi,pi]
            if ((heading_angle > - PI && heading_angle < -2*PI/3) || (heading_angle > 2*PI/3 && heading_angle < PI)) {
                if (obs->frenet_s() > ego_s) {
                    type = ObstacleTypeBeforeCluster::OPPOSITE_MOVING;
                } else {
                    type = ObstacleTypeBeforeCluster::IGNORE;
                }
            }
            else if(heading_angle > -PI/3 && heading_angle < PI/3) {
                if (obs->frenet_s() > ego_s) {
                    if (v_s > ego_v + kHighSpeedDiff) {
                         type = ObstacleTypeBeforeCluster::IGNORE;
                    } else {
                         type = ObstacleTypeBeforeCluster::FRONT_SAME_MOVING;
                    }
                } else {
                    if (v_s < ego_v - kLowSpeedThresh && v_s > 0.1) {
                         type = ObstacleTypeBeforeCluster::IGNORE;
                    } else {
                         type = ObstacleTypeBeforeCluster::BACK_SAME_MOVING;
                    }
                }
            }else{
                type = ObstacleTypeBeforeCluster::IGNORE;
                //TODO : 侧向移动的障碍物暂时不考虑，后续可以根据实际情况调整
            }
        }
        type_map[id] = type;
    }
    return type_map;
}

bool HppLateralObstacleUtils::MergeObstaclesBaseOnPos(
    const ObstacleItemMap& obs_item_map,
    const std::unordered_map<int, ObstacleTypeBeforeCluster>& classification_map,
    MergedObstacleContainer& merged_obs_container) {

    merged_obs_container.obs_id_to_merged_id.clear();
    merged_obs_container.merged_obstacles.clear();

    if (obs_item_map.empty()) return false;

    std::vector<MergedObstacleInfo> static_candidates;
    std::vector<MergedObstacleInfo> dynamic_candidates;

    int temp_idx = 0;

    for (const auto& entry : obs_item_map) {
        int id = entry.first;
        if (classification_map.find(id) == classification_map.end()) {
            continue;
        }
        ObstacleTypeBeforeCluster type = classification_map.at(id);
        if (type == ObstacleTypeBeforeCluster::IGNORE) {
            continue;
        }

        const auto& obs = entry.second;
        MergedObstacleInfo info;
        info.original_ids.push_back(id);
        info.type = type;
        const auto& b = obs->frenet_obstacle_boundary();
        info.s_start = b.s_start;
        info.s_end = b.s_end;
        info.l_start = std::min(b.l_start, b.l_end);
        info.l_end = std::max(b.l_start, b.l_end);

        if (obs->obstacle() && !obs->obstacle()->perception_polygon().points().empty()) {
             const auto& pts = obs->obstacle()->perception_polygon().points();
             info.raw_points.insert(info.raw_points.end(), pts.begin(), pts.end());
        } else {
             info.raw_points.emplace_back(b.s_start, b.l_start);
             info.raw_points.emplace_back(b.s_start, b.l_end);
             info.raw_points.emplace_back(b.s_end, b.l_end);
             info.raw_points.emplace_back(b.s_end, b.l_start);
        }

        if (type == ObstacleTypeBeforeCluster::FRONT_STATIC || type == ObstacleTypeBeforeCluster::SIDE_REAR_STATIC) {
            static_candidates.push_back(info);
        } else {
            dynamic_candidates.push_back(info); // 动态障碍物不参与聚类
        }
    }

    bool changed = true;
    while (changed) {
        changed = false;
        for (auto it = static_candidates.begin(); it != static_candidates.end(); ++it) {
            for (auto it2 = it + 1; it2 != static_candidates.end(); ) {

                double dist_sq = std::pow(it->center_s() - it2->center_s(), 2) +
                                 std::pow(it->center_l() - it2->center_l(), 2);
                double l_gap = std::max(0.0, std::abs(it->center_l() - it2->center_l()));
                double s_gap = std::max(0.0, std::abs(it->center_s() - it2->center_s()));
                bool cond1 = (dist_sq <= 1.0 * 1.0);
                bool cond2 = (l_gap <= 0.2 && s_gap <= 3.0);
                bool cond3 = (s_gap <= 2.0 && l_gap <= 2.0);

                if (cond1 || cond2 || cond3) {
                    it->original_ids.insert(it->original_ids.end(), it2->original_ids.begin(), it2->original_ids.end());
                    it->s_start = std::min(it->s_start, it2->s_start);
                    it->s_end = std::max(it->s_end, it2->s_end);
                    it->l_start = std::min(it->l_start, it2->l_start);
                    it->l_end = std::max(it->l_end, it2->l_end);

                    it->raw_points.insert(it->raw_points.end(), it2->raw_points.begin(), it2->raw_points.end());

                    if (it2->type == ObstacleTypeBeforeCluster::FRONT_STATIC) it->type = ObstacleTypeBeforeCluster::FRONT_STATIC;

                    it2 = static_candidates.erase(it2);
                    changed = true;
                } else {
                    ++it2;
                }
            }
        }
    }

    int final_id = 0;
    auto process_and_add = [&](std::vector<MergedObstacleInfo>& list) {
        for (auto& cluster : list) {
            cluster.merged_id = final_id++;
            BuildConvexHull(cluster);
            merged_obs_container.merged_obstacles.push_back(cluster);

            for (int orig_id : cluster.original_ids) {
                merged_obs_container.obs_id_to_merged_id[orig_id] = cluster.merged_id;
            }
        }
    };

    process_and_add(static_candidates);
    process_and_add(dynamic_candidates);

    return true;
}

void HppLateralObstacleUtils::BuildConvexHull(MergedObstacleInfo& cluster) {
    if (cluster.raw_points.size() >= 3) {
        planning_math::Polygon2d::ComputeConvexHull(cluster.raw_points, &cluster.polygon);
    } else {
        // 点不够，用 AABB 框代替
        std::vector<planning_math::Vec2d> pts;
        pts.emplace_back(cluster.s_start, cluster.l_start);
        pts.emplace_back(cluster.s_start, cluster.l_end);
        pts.emplace_back(cluster.s_end, cluster.l_end);
        pts.emplace_back(cluster.s_end, cluster.l_start);
        cluster.polygon = planning_math::Polygon2d(pts);
    }
    std::vector<planning_math::Vec2d>().swap(cluster.raw_points);
}

} // namespace planning