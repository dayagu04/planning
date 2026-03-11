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

  const bool raw_is_search_yield_back_vehicle() const {
    return raw_is_search_yield_back_vehicle_;
  }
  const bool raw_is_search_overtake_front_vehicle() const {
    return raw_is_search_overtake_front_vehicle_;
  }

 private:
  void UpdateDebounceFlag(const bool raw_flag, const double now_ms,
                          const int min_consecutive_frames,
                          const double min_hold_time_ms, bool* debounced_flag,
                          bool* last_raw_flag, int* consecutive_cnt,
                          double* last_change_time_ms);

  bool is_st_search_success_ = true;
  bool is_search_yield_back_vehicle_ = false;
  bool is_search_overtake_front_vehicle_ = false;
  bool is_yield_front_vehicle_safe_ = true;
  std::unordered_map<int32_t, speed::STBoundary::DecisionType>
      traffic_light_decision_map_;

  bool raw_is_search_yield_back_vehicle_ = false;
  bool raw_is_search_overtake_front_vehicle_ = false;

  bool last_raw_yield_back_ = false;
  bool last_raw_overtake_front_ = false;

  int yield_back_consecutive_cnt_ = 0;
  int overtake_front_consecutive_cnt_ = 0;

  double yield_back_last_change_time_ms_ = 0.0;
  double overtake_front_last_change_time_ms_ = 0.0;

  int debounce_min_consecutive_frames_ = 3;
  double debounce_min_hold_time_ms_ = 500.0;

 public:
  void set_debounce_params(const int min_consecutive_frames,
                           const double min_hold_time_ms) {
    if (min_consecutive_frames > 0) {
      debounce_min_consecutive_frames_ = min_consecutive_frames;
    }
    if (min_hold_time_ms > 0.0) {
      debounce_min_hold_time_ms_ = min_hold_time_ms;
    }
  }
};

}  // namespace planning
