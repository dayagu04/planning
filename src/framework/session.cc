#include "session.h"

#include <algorithm>
#include <cstdint>
#include <string>

#include "config_context.h"
#include "ego_planning_config.h"
#include "environmental_model.h"
#include "planning_context.h"
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
  // planning_output_context_->reset();
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

  environmental_model_ = alloc<EnvironmentalModel>();
  (void)environmental_model_->Init(scene_type_);
  planning_context_ = alloc<PlanningContext>();
  
  environmental_model_->set_module_config_file_dir(module_config_file_dir);
  simulation_context_ = SimulationContext::Instance();
  return true;
}

void Session::Update() { LOG_DEBUG("Session::update\n"); }

}  // namespace framework
}  // namespace planning