#include "slt_graph_point.h"

namespace planning {

std::uint32_t SLTGraphPoint::index_s() const { return index_s_; }

std::uint32_t SLTGraphPoint::index_l() const { return index_l_; }

std::uint32_t SLTGraphPoint::index_t() const { return index_t_; }

const SLTPoint& SLTGraphPoint::point() const { return point_; }

const SLTGraphPoint* SLTGraphPoint::pre_point() const { return pre_point_; }

double SLTGraphPoint::path_cost() const { return path_cost_; }

double SLTGraphPoint::obstacle_cost() const { return dynamic_obstacle_cost_; }

double SLTGraphPoint::longitinal_cost() const { return longitinal_relative_cost_; }

double SLTGraphPoint::min_obs_distance() const { return min_obs_distance_; }

double SLTGraphPoint::min_distance_agent_id() const { return min_dis_agent_id_; }


double SLTGraphPoint::spatial_potential_cost() const {
  return spatial_potential_cost_;
}

double SLTGraphPoint::total_cost() const { return total_cost_; }

void SLTGraphPoint::Init(
    const std::uint32_t index_t, const std::uint32_t index_s,
    const std::uint32_t index_l, const SLTPoint& slt_point) {
  index_t_ = index_t;
  index_s_ = index_s;
  index_l_ = index_l;
  point_ = slt_point;
}

void SLTGraphPoint::SetPathCost(const double path_cost) {
  path_cost_ = path_cost;
}

void SLTGraphPoint::SetDynamicObstacleCost(const double dynamic_obstacle_cost) {
  dynamic_obstacle_cost_ = dynamic_obstacle_cost;
}

void SLTGraphPoint::SetLongitinalCost(const double longitinal_relative_cost) {
  longitinal_relative_cost_ = longitinal_relative_cost;
}

void SLTGraphPoint::SetSpatialPotentialCost(
    const double spatial_potential_cost) {
  spatial_potential_cost_ = spatial_potential_cost;
}

void SLTGraphPoint::SetTotalCost(const double total_cost) {
  total_cost_ = total_cost;
}

void SLTGraphPoint::SetPrePoint(const SLTGraphPoint& pre_point) {
  pre_point_ = &pre_point;
}

double SLTGraphPoint::GetOptimalSpeed() const { return optimal_speed_; }

void SLTGraphPoint::SetOptimalSpeed(const double optimal_speed) {
  optimal_speed_ = optimal_speed;
}

void SLTGraphPoint::SetAcc(const double &acc) {
  a_ = acc;
}

}  // namespace planning