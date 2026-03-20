#pragma once

#include <unordered_map>

#include "st_graph/st_boundary.h"
namespace planning {

class StGraphSearcherOutput {
 public:
  StGraphSearcherOutput() = default;
  ~StGraphSearcherOutput() = default;

  const bool is_search_success() const;
  void set_is_search_success(const bool is_st_search_success);

  const bool is_search_yield_back_vehicle() const;
  void set_search_yield_back_vehicle(const bool is_search_yield_back_vehicle);

  const bool is_search_overtake_front_vehicle() const;
  void set_is_search_overtake_front_vehicle(
      const bool is_search_overtake_front_vehicle);

  const bool is_yield_front_vehicle_safe() const;
  void set_is_yield_front_vehicle_safe(const bool is_yield_front_vehicle_safe);

  const std::unordered_map<int32_t, speed::STBoundary::DecisionType>
  traffic_light_decision_map() const;
  void set_traffic_light_decision_map(
      const std::unordered_map<int32_t, speed::STBoundary::DecisionType>&
          traffic_light_decision_map);

 private:
  bool is_st_search_success_ = true;
  bool is_search_yield_back_vehicle_ = false;
  bool is_search_overtake_front_vehicle_ = false;
  bool is_yield_front_vehicle_safe_ = true;
  std::unordered_map<int32_t, speed::STBoundary::DecisionType>
      traffic_light_decision_map_;
};

}  // namespace planning
