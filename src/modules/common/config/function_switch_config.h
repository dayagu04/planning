#pragma once
#include "modules/common/utils/file.h"
#include "common/debug_info_log.h"

namespace planning {
struct FunctionSwitchConfig {
  // true: disable lane borrow, otherwise enable lane borrow
  int disable_lane_borrow = 0;
  int disable_tlf_function = 0;

  void InitParameters(const std::string& config_path) {
    std::string config_file = planning::common::util::ReadFile(config_path);
    auto config = mjson::Reader(config_file);
    JSON_READ_VALUE(disable_lane_borrow, int, "function.LB");
    JSON_READ_VALUE(disable_tlf_function, int, "function.TLC");
  }
};

}  // namespace planning