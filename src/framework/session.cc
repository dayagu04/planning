#include "session.h"

#include <algorithm>
#include <cstdint>
#include <string>

#include "config_context.h"
#include "ego_planning_config.h"
#include "environmental_model.h"
#include "planning_context.h"
#include "planning_output_context.h"
#include "utils.h"
#include "vehicle_config_context.h"
// #include "ego_planning_config.pb.h"

namespace planning {
namespace framework {

Session::Session() {}

Session::~Session() {}

void Session::Reset() {
  // environmental_model_->reset();
  planning_context_->reset();
  planning_output_context_->reset();
}

// static inline planning::common::SceneType parse_scene_type(const std::string &str) {
//   std::string scene_type = planning::common::trim(str);
//   if (scene_type == "HNP") {
//     return planning::common::SceneType::HIGHWAY;
//   } else if (scene_type == "PNP") {
//     return planning::common::SceneType::PARKING;
//   } else if (scene_type == "UNP") {
//     return planning::common::SceneType::URBAN;
//   } else if (scene_type == "RADS") {
//     return planning::common::SceneType::RADS;
//   } else {
//     LOG_DEBUG("unkown config scene type %s", scene_type.c_str());
//     return planning::common::SceneType::NOT_DEFINED;
//   }
// }

// static inline uint8_t parse_function_mode(const std::string &str) {
//   std::string function_mode = planning::common::trim(str);
// }

bool Session::Init() {
  auto engine_config = common::ConfigurationContext::Instance()->engine_config();
  auto scenario_config_file_dir = engine_config.scenario_cfg_dir;
  auto module_config_file_dir = engine_config.module_cfg_dir;
  LOG_DEBUG("ScenarioManager scenario_config_file_dir is: %s \n", scenario_config_file_dir.c_str());
  if (!common::ConfigurationContext::Instance()->load_params_from_json(scenario_config_file_dir)) {
    LOG_ERROR("ConfigurationContext load_params_from_json : %s \n", "ERROR");
    return false;
  }
  auto synthetic_config = common::ConfigurationContext::Instance()->synthetic_config();
  planning::common::SceneType init_scene_type = planning::common::SceneType::HIGHWAY;
  if (synthetic_config.scene_type == "apa") {
    init_scene_type = planning::common::SceneType::PARKING_APA;
  }
  // planning::common::SceneType init_scene_type =
  // static_cast<planning::common::SceneType>(synthetic_config.scene_type);
  if (init_scene_type == planning::common::SceneType::NOT_DEFINED) {
    init_scene_type = default_scene_type_;
  }
  LOG_DEBUG("init_scene_type %s\n", planning::common::SceneType_Name(init_scene_type).c_str());

  environmental_model_ = alloc<EnvironmentalModel>();
  (void)environmental_model_->Init(init_scene_type);
  planning_context_ = alloc<PlanningContext>();
  planning_output_context_ = alloc<PlanningOutputContext>();
  environmental_model_->set_module_config_file_dir(module_config_file_dir);
  vehicle_config_context_ = VehicleConfigurationContext::Instance();
  return true;
}

void Session::Update() { LOG_DEBUG("Session::update\n"); }

}  // namespace framework
}  // namespace planning