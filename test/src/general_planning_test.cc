#include <array>
#include <cmath>

#include "common/config_context.h"
#include "define/debug_output.h"
#include "gtest/gtest.h"
#include "local_view.h"
#include "planning_scheduler.h"

namespace planning {

static void init_planning() {
  std::cout << "The planning component init!!!" << std::endl;
  const std::string CONFIG_PATH = "/asw/planning/res/conf";
  std::string engine_config_path = CONFIG_PATH + "/planning_engine_config.json";
  common::ConfigurationContext::Instance()->load_engine_config_from_json(
      engine_config_path);

  auto engine_config =
      common::ConfigurationContext::Instance()->engine_config();

  std::string log_file = engine_config.log_conf.log_file;
  std::cout << "log_file!!!" << log_file << std::endl;
  // Nanolog
  bst::LogLevel log_level;
  if (engine_config.log_conf.log_level == "FETAL") {
    log_level = bst::FETAL;
  } else if (engine_config.log_conf.log_level == "ERROR") {
    log_level = bst::ERROR;
  } else if (engine_config.log_conf.log_level == "WARNING") {
    log_level = bst::WARNING;
  } else if (engine_config.log_conf.log_level == "NOTICE") {
    log_level = bst::NOTICE;
  } else if (engine_config.log_conf.log_level == "DEBUG") {
    log_level = bst::DEBUG;
  } else {
    log_level = bst::ERROR;
  }

  std::cout << "log_level!!!" << engine_config.log_conf.log_level << std::endl;
  bst::Log::getInstance().setConfig("Planning_Log", log_file.c_str(),
                                    log_level);
  LOG_DEBUG("The planning component init!!! \n");
}

TEST(PlanningScheduler, RunOnce) {
  init_planning();

  LocalView local_view_;
  iflyauto::PlanningOutput planning_output;
  // DebugOutput debug_output;
  iflyauto::PlanningHMIOutputInfoStr planning_hmi_info;

  std::unique_ptr<PlanningScheduler> planning_scheduler =
      std::make_unique<PlanningScheduler>();
  std::cout << "==============The planning enters RunOnce============="
            << std::endl;
}
}  // namespace planning