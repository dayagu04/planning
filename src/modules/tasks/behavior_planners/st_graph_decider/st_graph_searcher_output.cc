#include "st_graph_searcher_output.h"

#include "ifly_time.h"
#include "context/ego_planning_config.h"
#include "session.h"

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
  raw_is_search_yield_back_vehicle_ = is_search_yield_back_vehicle;
  UpdateYieldBackDebounce(is_search_yield_back_vehicle);
}

const bool StGraphSearcherOutput::is_search_overtake_front_vehicle() const {
  return is_search_overtake_front_vehicle_;
}
void StGraphSearcherOutput::set_is_search_overtake_front_vehicle(
    const bool is_search_overtake_front_vehicle) {
  raw_is_search_overtake_front_vehicle_ = is_search_overtake_front_vehicle;
  UpdateOvertakeFrontDebounce(is_search_overtake_front_vehicle);
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

void StGraphSearcherOutput::UpdateYieldBackDebounce(bool raw_flag) {
  const double now_ms = IflyTime::Now_ms();

  if (raw_flag == is_search_yield_back_vehicle_) {
    last_raw_yield_back_ = raw_flag;
    yield_back_consecutive_cnt_ = 0;
    return;
  }

  if (raw_flag == last_raw_yield_back_) {
    ++yield_back_consecutive_cnt_;
  } else {
    last_raw_yield_back_ = raw_flag;
    yield_back_consecutive_cnt_ = 1;
    yield_back_last_change_time_ms_ = now_ms;
  }

  const double hold_time = now_ms - yield_back_last_change_time_ms_;
  if (yield_back_consecutive_cnt_ >= debounce_min_consecutive_frames_ &&
      hold_time >= debounce_min_hold_time_ms_) {
    is_search_yield_back_vehicle_ = raw_flag;
    yield_back_last_change_time_ms_ = now_ms;
    yield_back_consecutive_cnt_ = 0;
  }
}

void StGraphSearcherOutput::UpdateOvertakeFrontDebounce(bool raw_flag) {
  const double now_ms = IflyTime::Now_ms();

  if (raw_flag == is_search_overtake_front_vehicle_) {
    last_raw_overtake_front_ = raw_flag;
    overtake_front_consecutive_cnt_ = 0;
    return;
  }

  if (raw_flag == last_raw_overtake_front_) {
    ++overtake_front_consecutive_cnt_;
  } else {
    last_raw_overtake_front_ = raw_flag;
    overtake_front_consecutive_cnt_ = 1;
    overtake_front_last_change_time_ms_ = now_ms;
  }

  const double hold_time = now_ms - overtake_front_last_change_time_ms_;
  if (overtake_front_consecutive_cnt_ >= debounce_min_consecutive_frames_ &&
      hold_time >= debounce_min_hold_time_ms_) {
    is_search_overtake_front_vehicle_ = raw_flag;
    overtake_front_last_change_time_ms_ = now_ms;
    overtake_front_consecutive_cnt_ = 0;
  }
}

}  // namespace planning
