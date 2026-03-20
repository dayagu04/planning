#include "st_graph_searcher_output.h"

namespace planning {
const bool StGraphSearcherOutput::is_search_success() const {
  return is_st_search_success_;
}

void StGraphSearcherOutput::set_is_search_success(
    const bool is_st_search_success) {
  is_st_search_success_ = is_st_search_success;
}

const bool StGraphSearcherOutput::is_search_yield_back_vehicle() const {
  return is_search_yield_back_vehicle_;
}

void StGraphSearcherOutput::set_search_yield_back_vehicle(
    const bool is_search_yield_back_vehicle) {
  is_search_yield_back_vehicle_ = is_search_yield_back_vehicle;
}

const bool StGraphSearcherOutput::is_search_overtake_front_vehicle() const {
  return is_search_overtake_front_vehicle_;
}
void StGraphSearcherOutput::set_is_search_overtake_front_vehicle(
    const bool is_search_overtake_front_vehicle) {
  is_search_overtake_front_vehicle_ = is_search_overtake_front_vehicle;
}

const std::unordered_map<int32_t, speed::STBoundary::DecisionType>
StGraphSearcherOutput::traffic_light_decision_map() const {
  return traffic_light_decision_map_;
}

void StGraphSearcherOutput::set_traffic_light_decision_map(
    const std::unordered_map<int32_t, speed::STBoundary::DecisionType>&
        traffic_light_decision_map) {
  traffic_light_decision_map_ = traffic_light_decision_map;
}

void StGraphSearcherOutput::set_is_yield_front_vehicle_safe(
    const bool is_yield_front_vehicle_safe) {
  is_yield_front_vehicle_safe_ = is_yield_front_vehicle_safe;
}

const bool StGraphSearcherOutput::is_yield_front_vehicle_safe() const {
  return is_yield_front_vehicle_safe_;
}
}  // namespace planning
