#include "start_stop_enable.h"

namespace planning {

void StartStopEnable::init() {
  enable_start_stop_function_ = config_.enable_start_stop_function;
  enable_hnp_functions_ = config_.enable_hnp_functions;
}

void StartStopEnable::go_trajectory(
    common::LonDecisionInfo& lon_decision_information,
    int& start_stop_information, common::StartStopInfo& start_stop_result) {}

}  // namespace planning