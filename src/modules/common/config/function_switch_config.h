#pragma once
#include "modules/common/utils/file.h"
#include "common/debug_info_log.h"

namespace planning {
struct FunctionSwitchConfig {
  bool disable_lane_borrow =
      false;  // true: disable lane borrow, otherwise enable lane borrow
  bool disable_tlf_function = false;

  void InitParameters(const std::string& config_path) {
    std::string config_file = planning::common::util::ReadFile(config_path);
    auto config = mjson::Reader(config_file);
    JSON_READ_VALUE(disable_lane_borrow, bool, "function.LB");
    JSON_READ_VALUE(disable_tlf_function, bool, "function.TLC");
  }
};

}  // namespace planning