#pragma once

struct GapSelectorDeciderOutput {
  bool gap_selector_trustworthy = false;
  int lc_direction = 0;  // 0 -- no change, 1-- left change, 2--right change
  bool lat_decider_ignore = 0;
  bool lon_decider_ignore = 0;
};