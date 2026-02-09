#include "hpp_lateral_obstacle_utils.h"
#include "edt_manager.h"

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

bool HppLateralObstacleUtils::MergeObstaclesBaseOnPos(
    const ObstacleItemMap& obs_item_map,
    MergedObstacleContainer& merged_obs_container) {

  merged_obs_container.obs_id_to_merged_id.clear();
  merged_obs_container.merged_obstacles.clear();

  if (obs_item_map.empty()) return false;

  std::vector<MergedObstacleInfo> temp_clusters;
  int temp_id_counter = 0; // 从 0 开始的迭代编号

  // 初始化
  for (const auto& entry : obs_item_map) {
      const auto& obs = entry.second;
      MergedObstacleInfo info;
      info.merged_id = temp_id_counter++;
      info.original_ids.push_back(obs->id());
      info.is_static = obs->is_static();
      info.v_s = obs->frenet_velocity_s();
      info.v_l = obs->frenet_velocity_l();

      const auto& b = obs->frenet_obstacle_boundary();
      info.s_start = b.s_start;
      info.s_end = b.s_end;
      info.l_start = std::min(b.l_start, b.l_end);
      info.l_end = std::max(b.l_start, b.l_end);

      temp_clusters.push_back(info);
  }

  // 聚类迭代
  bool changed = true;
  while (changed) {
    changed = false;
    for (auto it = temp_clusters.begin(); it != temp_clusters.end(); ++it) {
      for (auto it2 = it + 1; it2 != temp_clusters.end(); ) {

        double l_1 = it->center_l();
        double s_1 = it->center_s();
        double l_2 = it2->center_l();
        double s_2 = it2->center_s();

        double dist_sq = std::pow(l_1 - l_2 , 2) + std::pow(s_1 - s_2 , 2);
        double l_gap = abs(l_1 - l_2);
        double s_gap = abs(s_1 - s_2);

        // 规则
        bool cond1 = (dist_sq <= 1.0 * 1.0);
        bool cond_lat_cluster = (l_gap < 2.0 && s_gap < 0.5);
        bool cond_lon_cluster = (l_gap < 1.5 && s_gap < 3.0);

        if (cond1 || cond_lat_cluster || cond_lon_cluster) {
            // 合并
            it->original_ids.insert(it->original_ids.end(), it2->original_ids.begin(), it2->original_ids.end());
            it->s_start = std::min(it->s_start, it2->s_start);
            it->s_end = std::max(it->s_end, it2->s_end);
            it->l_start = std::min(it->l_start, it2->l_start);
            it->l_end = std::max(it->l_end, it2->l_end);
            if (!it2->is_static) it->is_static = false;

            it2 = temp_clusters.erase(it2);
            changed = true;
        } else {
            ++it2;
        }
      }
    }
  }

  int final_id = 0;
  for (auto& cluster : temp_clusters) {
      cluster.merged_id = final_id++;
      cluster.consistency_group_id = cluster.merged_id; // 初始状态：一致性组ID = 自身ID
      merged_obs_container.merged_obstacles.push_back(cluster);

      for (int orig_id : cluster.original_ids) {
          merged_obs_container.obs_id_to_merged_id[orig_id] = cluster.merged_id;
      }
  }

  return true;
}

// 按s排序
bool HppLateralObstacleUtils::SortMergedObstacles(
    MergedObstacleContainer& merged_obs_container) {
    auto& obs_list = merged_obs_container.merged_obstacles;
    std::sort(obs_list.begin(), obs_list.end(),
        [](const MergedObstacleInfo& a, const MergedObstacleInfo& b){
            return a.s_start < b.s_start;
        });
    return true;
}

bool HppLateralObstacleUtils::ClassifyObstacles(
    const MergedObstacleContainer& merged_obs_container,
    const double ego_s,
    const double ego_v,
    ObstacleClassificationResult& obs_classification_result) {

    obs_classification_result.Clear();

    for (const auto& obs : merged_obs_container.merged_obstacles) {

        if (obs.is_static) {
            if (obs.center_s() > ego_s) {
                obs_classification_result.front_static_obs.push_back(&obs);
            } else {
                obs_classification_result.side_static_obs.push_back(&obs);
            }
        } else {
            if (obs.v_s < -0.1) {
                if (obs.center_s() > ego_s) {
                    obs_classification_result.opposite_moving_obs.push_back(&obs);
                }
            } else {
                if (obs.center_s() > ego_s) {
                    if (obs.v_s <= ego_v) {
                        obs_classification_result.front_same_moving_obs.push_back(&obs);
                    }
                } else {
                    if (obs.v_s >= ego_v) {
                        obs_classification_result.back_same_moving_obs.push_back(&obs);
                    }
                }
            }
        }
    }
    return true;
}

void HppLateralObstacleUtils::UpdateStaticObsConsistency(
    MergedObstacleContainer& merged_obs_container,
    const ObstacleClassificationResult& classification_result,
    const double ego_head_l) {

    std::unordered_map<int, MergedObstacleInfo*> mutable_map;
    for (auto& obs : merged_obs_container.merged_obstacles) {
        mutable_map[obs.merged_id] = &obs;
    }

    if (!classification_result.front_static_obs.empty()) {

        // 1. 提取可变指针列表
        std::vector<MergedObstacleInfo*> target_clusters;
        target_clusters.reserve(classification_result.front_static_obs.size());

        for (const auto* const_ptr : classification_result.front_static_obs) {
            if (mutable_map.count(const_ptr->merged_id)) {
                target_clusters.push_back(mutable_map[const_ptr->merged_id]);
            }
        }

        // 2. 按横向位置 L 从小到大排序 (从右到左)
        // c1(右, 小L), c2(左, 大L)
        std::sort(target_clusters.begin(), target_clusters.end(),
            [](const MergedObstacleInfo* a, const MergedObstacleInfo* b){
                return a->center_l() < b->center_l();
            });

        // 3. 参数配置
        const auto &vehicle_param =
                VehicleConfigurationContext::Instance()->get_vehicle_param();
        const double kRoiHalfWidth = 4.0;
        const double kMinLatPass = vehicle_param.width + 1.0;
        const double kKinematicRatio = 3.5;

        // 4. 遍历相邻 Cluster 进行 Gap 判断
        for (size_t i = 0; i < target_clusters.size(); ++i) {
            if (i == target_clusters.size() - 1) break;

            MergedObstacleInfo* c1 = target_clusters[i];   // 右侧障碍物 (L较小)
            MergedObstacleInfo* c2 = target_clusters[i+1]; // 左侧障碍物 (L较大)

            // 4.1 ROI 检查: 只要有一个在 ROI 内，就需要处理
            bool in_roi = false;
            if ((c1->center_l() > ego_head_l - kRoiHalfWidth && c1->center_l() < ego_head_l + kRoiHalfWidth) ||
                (c2->center_l() > ego_head_l - kRoiHalfWidth && c2->center_l() < ego_head_l + kRoiHalfWidth)) {
                in_roi = true;
            }
            if (!in_roi) continue;

            double raw_gap = c2->l_start - c1->l_end;
            double l_gap = std::max(0.0, raw_gap);

            // 4.3 纵向绕行条件 (中心距)
            double s_dist = std::abs(c1->center_s() - c2->center_s());
            double needed_s = l_gap * kKinematicRatio;

            // 4.4 判断通行条件
            bool lat_ok = (l_gap >= kMinLatPass);
            bool lon_ok = (s_dist >= needed_s);

            if (!lat_ok || !lon_ok) {
                int root1 = c1->consistency_group_id;
                int root2 = c2->consistency_group_id;

                if (root1 != root2) {
                    for (auto& obs : merged_obs_container.merged_obstacles) {
                        if (obs.consistency_group_id == root2) {
                            obs.consistency_group_id = root1;
                        }
                    }
                }
            }
        }
    }

}

bool HppLateralObstacleUtils::MakeDecisionForMovingObstacles(
    const MergedObstacleContainer& merged_obs_container,
    const ObstacleClassificationResult& obs_classification_result) {

    // 1. 对向行驶障碍物
    if (!obs_classification_result.opposite_moving_obs.empty()) {
        // TODO: 处理对向会车逻辑
    }

    // 2. 同向前方障碍物
    if (!obs_classification_result.front_same_moving_obs.empty()) {
        // TODO: 处理同向超车/跟车逻辑
    }

    // 3. 同向后方障碍物
    if (!obs_classification_result.back_same_moving_obs.empty()) {
        // TODO: 处理被超车/避让逻辑
    }

    return true;
}

} // namespace planning