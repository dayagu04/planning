#include "general_planner_module.h"

#include "common/log.h"
#include "modules/common/vehicle_model/vehicle_model.h"
// #include "modules/context/vehicle_config_context.h"

namespace planning {
namespace modules {

GeneralPlannerModule::GeneralPlannerModule() : general_planner_() {
  LOG_DEBUG("%s constructed\n", name().c_str());
}

GeneralPlannerModule::~GeneralPlannerModule() {
  LOG_DEBUG("%s destructed\n", name().c_str());
}

planning::framework::BaseModule* GeneralPlannerModule::clone() const {
  return nullptr;
}

EgoPlanningConfigBuilder* GeneralPlannerModule::load_config_builder(
    planning::framework::Session* session, const char* file_name) {
  // auto config_file_dir = session->environmental_model().get_module_config_file_dir();
  // auto ego_planning_config_json_file = config_file_dir + "/" + file_name;
  // LOG_DEBUG("%s", ego_planning_config_json_file.c_str());

  // Json ego_planning_config_json;
  // std::ifstream fin(ego_planning_config_json_file);
  // fin >> ego_planning_config_json;
  // fin.close();

  // return session->alloc<EgoPlanningConfigBuilder>(ego_planning_config_json,
  //                                                   file_name);
}

bool GeneralPlannerModule::init(const ::google::protobuf::Message* config,
                                  planning::framework::Session* session) {
  LOG_DEBUG("%s init\n", name().c_str());
  // std::string config_file_dir =
  //     session->mutable_environmental_model()->get_module_config_file_dir();
  // // planning::planner::ConfigurationContext::Instance()->load_vehicle_param();
  // (void)planning::planner::VehicleModel::LoadVehicleModelConfig(config_file_dir);

  // auto urban_config_builder =
  //     load_config_builder(session, "general_planner_module_urban.json");
  // session->mutable_environmental_model()->set_urban_config_builder(
  //     urban_config_builder);

  // auto parking_config_builder =
  //     load_config_builder(session, "general_planner_module_parking.json");
  // session->mutable_environmental_model()->set_parking_config_builder(
  //     parking_config_builder);

  // auto highway_config_builder =
  //     load_config_builder(session, "general_planner_module_highway.json");
  // session->mutable_environmental_model()->set_highway_config_builder(
  //     highway_config_builder);

  general_planner_.Init(session);
  return true;
}

bool GeneralPlannerModule::reset(const ::google::protobuf::Message* config) {
  general_planner_.InitContext();
  return true;
}

void GeneralPlannerModule::compute(planning::framework::Frame* frame) {

  LOG_DEBUG("%s compute\n", name().c_str());
  if (!frame->session()->environmental_model().GetVehicleDbwStatus()) {
    LOG_WARNING("%s DBW_Disable, but continue", name().c_str());
  }

  bool success = false;

  success = general_planner_.Run(frame);

  if (!success) {
    LOG_ERROR("%s planning run failed", name().c_str());
    return;
  }

  frame->mutable_session()
      ->mutable_planning_context()
      ->mutable_last_frame_planning_result() =
      frame->session()->planning_output_context().planning_status().planning_result;
  frame->mutable_session()
      ->mutable_planning_context()
      ->mutable_last_planning_success() =
      frame->session()->planning_context().planning_success();
  LOG_DEBUG("%s planning_success = %d\n", name().c_str(),
        frame->session()->planning_context().planning_success());
}

}  // namespace modules
}  // namespace planning
