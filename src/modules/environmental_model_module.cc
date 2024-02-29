#include "environmental_model_module.h"

#include "common/vehicle_model/vehicle_model.h"
#include "log.h"
#include "planning_context.h"
// #include "context/vehicle_config_context.h"

namespace planning {
namespace modules {

EnvironmentalModelModule::EnvironmentalModelModule()
    : environmental_model_manager_() {
  LOG_DEBUG("%s constructed\n", name().c_str());
}

EnvironmentalModelModule::~EnvironmentalModelModule() {
  LOG_DEBUG("%s destructed\n", name().c_str());
}

planning::framework::BaseModule* EnvironmentalModelModule::clone() const {
  return nullptr;
}

EgoPlanningConfigBuilder* EnvironmentalModelModule::load_config_builder(
    planning::framework::Session* session, const char* file_name) {
  auto config_file_dir =
      session->environmental_model().get_module_config_file_dir();
  auto ego_planning_config_json_file = config_file_dir + "/" + file_name;
  LOG_DEBUG("%s\n", ego_planning_config_json_file.c_str());

  Json ego_planning_config_json;
  std::ifstream fin(ego_planning_config_json_file);
  fin >> ego_planning_config_json;
  fin.close();

  return session->alloc<EgoPlanningConfigBuilder>(ego_planning_config_json,
                                                  file_name);
}

bool EnvironmentalModelModule::init(const ::google::protobuf::Message* config,
                                    planning::framework::Session* session) {
  LOG_DEBUG("%s init\n", name().c_str());
  std::string config_file_dir =
      session->mutable_environmental_model()->get_module_config_file_dir();
  // planning::planner::ConfigurationContext::Instance()->load_vehicle_param();
  (void)planning::common::VehicleModel::LoadVehicleModelConfig(config_file_dir);

  auto parking_config_builder =
      load_config_builder(session, "general_planner_module_parking.json");
  session->mutable_environmental_model()->set_parking_config_builder(
      parking_config_builder);

  auto highway_config_builder =
      load_config_builder(session, "general_planner_module_highway.json");
  session->mutable_environmental_model()->set_highway_config_builder(
      highway_config_builder);

  auto hpp_config_builder =
      load_config_builder(session, "general_planner_module_hpp.json");
  session->mutable_environmental_model()->set_hpp_config_builder(
      hpp_config_builder);

  environmental_model_manager_.Init(session);
  return true;
}

bool EnvironmentalModelModule::reset(
    const ::google::protobuf::Message* config) {
  environmental_model_manager_.InitContext();
  return true;
}

bool EnvironmentalModelModule::compute(planning::framework::Frame* frame) {
  LOG_DEBUG("%s compute\n", name().c_str());
  if (!frame->session()->environmental_model().GetVehicleDbwStatus()) {
    LOG_WARNING("%s DBW_Disable, but continue\n", name().c_str());
  }

  bool success = false;

  success = environmental_model_manager_.Run(frame);

  if (!success) {
    LOG_ERROR("%s planning run failed\n", name().c_str());
    frame->mutable_session()->mutable_planning_context()->clear();
    return false;
  }
  return true;
}

}  // namespace modules
}  // namespace planning
