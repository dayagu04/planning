#pragma once

#include <vector>

namespace planning {

enum MSDLaneStrateryAvailableFlag {
  MSD_LANE_STRATERY_LANE_STRATERY_DATA = 1 << 0,
};

enum MSDLaneChangeState {
  MSD_LANE_CHANGE_STATE_FOLLOW = 0,
  MSD_LANE_CHANGE_STATE_TURN_LEFT = 1,
  MSD_LANE_CHANGE_STATE_TURN_RIGHT = 2,
};

struct MSDLaneStrateryData {
  std::vector<MSDLaneChangeState> stratery_start_with_current_lane;
  std::vector<MSDLaneChangeState> stratery_start_with_left_lane;
  std::vector<MSDLaneChangeState> stratery_start_with_right_lane;
  std::vector<MSDInterval> current_lane_change_available_interval;
};

struct MSDLaneStratery {
  size_t available;
  MSDLaneStrateryData lane_stratery_data;
};
} // namespace planning
