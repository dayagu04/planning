#include "session.h"

#include <algorithm>
#include <cstdint>
#include <string>

#include "config_context.h"
#include "ego_planning_config.h"
#include "environmental_model.h"
#include "planning_context.h"
#include "planning_output_context.h"
#include "scene_type_config.pb.h"
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

static common::SceneType parse_scene_type_str(const std::string& str) {
  if (str == "apa") {
    return planning::common::SceneType::PARKING_APA;
  } else if (str == "highway") {
    return planning::common::SceneType::HIGHWAY;
  } else {
    return planning::common::NOT_DEFINED;
  }
}

bool Session::Init() {
  auto engine_config =
      common::ConfigurationContext::Instance()->engine_config();
  auto module_config_file_dir = engine_config.module_cfg_dir;
  if (!common::ConfigurationContext::Instance()->load_params_from_json(
          engine_config.vehicle_cfg_dir)) {
    LOG_ERROR("ConfigurationContext load_params_from_json : %s \n", "ERROR");
    return false;
  }
  auto scene_type_config =
      common::ConfigurationContext::Instance()->scene_type_config();
  scene_type_ = parse_scene_type_str(scene_type_config.default_scene_type());
  LOG_DEBUG("init_scene_type %s\n",
            planning::common::SceneType_Name(scene_type_).c_str());
  for (auto it : scene_type_config.module_name_map()) {
    module_name_map_[parse_scene_type_str(it.first)] = it.second;
  }

  environmental_model_ = alloc<EnvironmentalModel>();
  (void)environmental_model_->Init(scene_type_);
  planning_context_ = alloc<PlanningContext>();
  planning_output_context_ = alloc<PlanningOutputContext>();
  environmental_model_->set_module_config_file_dir(module_config_file_dir);
  vehicle_config_context_ = VehicleConfigurationContext::Instance();
  return true;
}

void Session::Update() { LOG_DEBUG("Session::update\n"); }

}  // namespace framework
}  // namespace planning