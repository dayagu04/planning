#include <array>
#include <cmath>

#include "define/debug_output.h"
#include "general_planning.h"
#include "gtest/gtest.h"
#include "local_view.h"
#include "common/config_context.h"

namespace planning {

static void init_planning() {
  std::cout << "The planning component init!!!" << std::endl;
  std::string engine_config_path = std::string(CONFIG_PATH) + "/planning_engine_config.json";
  common::ConfigurationContext::Instance()->load_engine_config_from_json(engine_config_path);

  auto engine_config = common::ConfigurationContext::Instance()->engine_config();

  std::string log_file = engine_config.log_conf.log_file_dir + "/planning_log";
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
  bst::Log::getInstance().setConfig("Planning_Log", log_file.c_str(), log_level);
  LOG_DEBUG("The planning component init!!! \n");
}

TEST(GeneralPlanning, RunOnce) {
  init_planning();

  LocalView local_view_;
  PlanningOutput::PlanningOutput planning_output;
  DebugOutput debug_output;
  PlanningHMI::PlanningHMIOutputInfoStr planning_hmi_Info;

  std::unique_ptr<GeneralPlanning> planning_base =
      std::make_unique<GeneralPlanning>();
  std::cout << "==============The planning enters RunOnce============="
            << std::endl;
  planning_base->RunOnce(local_view_, &planning_output, debug_output, planning_hmi_Info);
}
}  // namespace planning