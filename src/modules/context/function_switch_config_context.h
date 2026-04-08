#pragma once

#include <string>

#include "common/config/function_switch_config.h"
#include "log_glog.h"
#include "common/macro.h"

namespace planning {

class FunctionSwitchConfigContext {
 private:
  // this is a singleton class
  IFLY_DECLARE_SINGLETON(FunctionSwitchConfigContext);

 public:
  void load_function_switch_config(const std::string& config_path) {
    ILOG_DEBUG << "load_function_switch config: config_path:"
               << config_path.c_str();
    function_switch_config_.InitParameters(config_path);
  }

  const FunctionSwitchConfig& get_function_switch_config() const {
    return function_switch_config_;
  }

  FunctionSwitchConfig* mutable_function_switch_config() {
    return &function_switch_config_;
  }

 private:
  FunctionSwitchConfig function_switch_config_;
};

}  // namespace planning
