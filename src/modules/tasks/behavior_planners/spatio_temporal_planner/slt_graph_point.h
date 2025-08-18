#pragma once

#include <limits>

#include "slt_point.h"

namespace planning {

class SLTGraphPoint {
 public:
  std::uint32_t index_s() const;
  std::uint32_t index_l() const;
  std::uint32_t index_t() const;

  const SLTPoint& point() const;
  const SLTGraphPoint* pre_point() const;

  double path_cost() const;
  double obstacle_cost() const;
  double longitinal_cost() const;
  double min_obs_distance() const;
  double min_distance_agent_id() const;
  double spatial_potential_cost() const;
  double total_cost() const;

  void Init(const std::uint32_t index_t, const std::uint32_t index_s,
            const std::uint32_t index_l, const SLTPoint& slt_point);

  // given reference speed profile, reach the cost, including position
  // void SetReferenceCost(const double path_cost);

  // 规划路径引起的cost
  void SetPathCost(const double path_cost);

  // 节点与静态障碍物产生的cost
  void SetStaticObstacleCost(const double StaticObstacleCost);

  // 节点与动态障碍物之间的安全性cost
  void SetDynamicObstacleCost(const double dynamic_obstacle_cost);

  // 节点与动态障碍物之间的安全性cost
  void SetLongitinalCost(const double longitinal_relative_cost);

  // path_length与当前s之间的潜在cost
  void SetSpatialPotentialCost(const double spatial_potential_cost);

  // 上一节点至当前节点自车与障碍物的最近距离
  void SetMinObsDistance(const double min_obs_distance) {
    min_obs_distance_ = min_obs_distance;
  }

  // 上一节点至当前节点自车与障碍物的最近距离
  void SetMinDistanceAgentId(const double agent_id) {
    min_dis_agent_id_ = agent_id;
  }

  // total cost
  void SetTotalCost(const double total_cost);

  void SetPrePoint(const SLTGraphPoint& pre_point);

  double GetOptimalSpeed() const;

  void SetOptimalSpeed(const double optimal_speed);

  double GetAcc() const;

  void SetAcc(const double& acc);

 private:
  SLTPoint point_;
  const SLTGraphPoint* pre_point_ = nullptr;
  std::uint32_t index_s_ = 0;
  std::uint32_t index_t_ = 0;
  std::uint32_t index_l_ = 0;

  double optimal_speed_ = 0.0;
  double a_ = 0.0;
  double reference_cost_ = 0.0;
  double path_cost_ = 0.0;
  double static_obstacle_cost_ = 0.0;
  double dynamic_obstacle_cost_ = 0.0;
  double spatial_potential_cost_ = 0.0;
  double longitinal_relative_cost_ = 0.0;
  double min_obs_distance_ = 100.0;
  int min_dis_agent_id_ = -1;

  double total_cost_ = std::numeric_limits<double>::infinity();
};

}  // namespace planning