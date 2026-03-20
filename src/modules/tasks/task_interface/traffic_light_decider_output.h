#pragma once

struct TrafficLightDeciderOutput {
  bool can_pass = true;
  bool is_small_front_intersection = false;
  bool is_tfl_match_intersection = true;
};