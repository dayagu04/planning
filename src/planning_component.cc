#include "planning_component.h"

#include "common/Platform_Types.h"
#include "general_planning.h"
#include "modules/common/config_context.h"

namespace planning {
bool PlanningComponent::Init() {
  std::cout << "The planning component init!!!" << std::endl;
  std::string engine_config_path = std::string(CONFIG_PATH) + "/engine_config.json";
  common::ConfigurationContext::Instance()->load_engine_config_from_json(engine_config_path);

  auto engine_config = common::ConfigurationContext::Instance()->engine_config();
  double time_stamp = IflyTime::Now_s();

  std::string log_file = engine_config.log_file_dir + "/planning_log_" + std::to_string(time_stamp);
  std::cout << "log_file!!!" <<log_file<< std::endl;
  // Nanolog
  bst::LogLevel log_level;
  if(engine_config.log_level == "FETAL"){
    log_level = bst::FETAL;
  }else if (engine_config.log_level == "ERROR"){
    log_level = bst::ERROR;
  }else if (engine_config.log_level == "WARNING"){
    log_level = bst::WARNING;
  }else if (engine_config.log_level == "NOTICE"){
    log_level = bst::NOTICE;
  }else if (engine_config.log_level == "DEBUG"){
    log_level = bst::DEBUG;
  }else{
    log_level = bst::ERROR;
  }

  std::cout << "log_level!!!" << engine_config.log_level << std::endl;
  bst::Log::getInstance().setConfig(
            "Planning_Log", log_file.c_str(), log_level);
  LOG_DEBUG("The planning component init!!! \n");

  //load algorithm and planner config params
  std::string algorithm_config_path =  engine_config.algorithm_param_cfg_dir + "/algorithm_param.json"; // 待优化
  common::ConfigurationContext::Instance()->load_algorithm_config_from_json(algorithm_config_path);
  LOG_DEBUG("The algorithm_config json file load well!!! \n");

  // 1.定义cyber node
  ADSNode::Init("planning_node");
  planning_node_ = std::make_shared<ADSNode>("planning_node");

  // session_.init();
  // scheduler_.init(&session_);

  // 2.定义收发topics
  // -------------- reader topics --------------
  auto fusion_objects_reader_ =
      planning_node_->CreateReader<FusionObjects::FusionObjectsInfo>(
          "/fusion/fusion_object",
          [this](
              const std::shared_ptr<FusionObjects::FusionObjectsInfo> &fusion_objects_info_msg) {
            std::cout << "receive fusion fusion_objects_info " << fusion_objects_info_msg->header().timestamp()
                      << std::endl;
            std::lock_guard<std::mutex> lock(msg_mutex_);
            fusion_objects_info_msg_.CopyFrom(*fusion_objects_info_msg);
          });

  auto fusion_road_reader_ =
      planning_node_->CreateReader<FusionRoad::RoadInfo>(
          "/fusion/fusion_road",
          [this](
              const std::shared_ptr<FusionRoad::RoadInfo> &road_info_msg) {
            std::cout << "receive fusion road_info " << road_info_msg->header().timestamp()
                      << std::endl;
            std::lock_guard<std::mutex> lock(msg_mutex_);
            road_info_msg_.CopyFrom(*road_info_msg);
          });

  auto localization_reader_ =
      planning_node_->CreateReader<LocalizationOutput::LocalizationEstimate>(
          "/localization",
          [this](
              const std::shared_ptr<LocalizationOutput::LocalizationEstimate> &localization_estimate_msg) {
            std::cout << "receive localization_estimate " << localization_estimate_msg->header().timestamp()
                      << std::endl;
            std::lock_guard<std::mutex> lock(msg_mutex_);
            localization_estimate_msg_.CopyFrom(*localization_estimate_msg);
          });

  auto prediction_reader_ =
      planning_node_->CreateReader<Prediction::PredictionResult>(
          "/prediction",
          [this](
              const std::shared_ptr<Prediction::PredictionResult> &prediction_result_msg) {
            std::cout << "receive prediction_result " << prediction_result_msg->header().timestamp()
                      << std::endl;
            std::lock_guard<std::mutex> lock(msg_mutex_);
            prediction_result_msg_.CopyFrom(*prediction_result_msg);
          });

  auto vehicel_service_reader_ =
      planning_node_->CreateReader<VehicleService::VehicleServiceOutputInfo>(
          "/vehicel_service_output_info",
          [this](
              const std::shared_ptr<VehicleService::VehicleServiceOutputInfo> &vehicel_service_output_info_msg) {
            std::cout << "receive vehicel_service_output_info " << vehicel_service_output_info_msg->header().timestamp()
                      << std::endl;
            std::lock_guard<std::mutex> lock(msg_mutex_);
            vehicel_service_output_info_msg_.CopyFrom(*vehicel_service_output_info_msg);
          });

  auto radar_perception_objects_reader_ =
      planning_node_->CreateReader<RadarPerceptionObjects::RadarPerceptionObjectsInfo>(
          "/radar_perception_objects_info",
          [this](
              const std::shared_ptr<RadarPerceptionObjects::RadarPerceptionObjectsInfo> &radar_perception_objects_info_msg) {
            std::cout << "receive radar_perception_objects_info " << radar_perception_objects_info_msg->header().timestamp()
                      << std::endl;
            std::lock_guard<std::mutex> lock(msg_mutex_);
            radar_perception_objects_info_msg_.CopyFrom(*radar_perception_objects_info_msg);
          });

  auto control_output_reader_ =
      planning_node_->CreateReader<ControlCommand::ControlOutput>(
          "/control_output",
          [this](
              const std::shared_ptr<ControlCommand::ControlOutput> &control_output_msg) {
            std::cout << "receive control_output " << control_output_msg->header().timestamp()
                      << std::endl;
            std::lock_guard<std::mutex> lock(msg_mutex_);
            control_output_msg_.CopyFrom(*control_output_msg);
          });

  // -------------- writter topics --------------
  planning_writer_ =
      planning_node_->CreateWriter<PlanningOutput::PlanningOutput>(
          "/planning");
  planning_debug_writer_ =
      planning_node_->CreateWriter<planning::common::PlanningDebugInfo>(
          "/planning_debug_info");

  // 3.planning初始化，使用general_planning
  planning_base_ = std::make_unique<GeneralPlanning>();

  return true;
}

bool PlanningComponent::Proc() {
  // system("clear");
  std::cout << "==============The planning component running=============" << std::endl;
  LOG_DEBUG("GeneralPlanning::RunOnce \n");
  double start_time = IflyTime::Now_ms();

  // 1.update all inputs to local_view
  {
    std::lock_guard<std::mutex> lock(msg_mutex_);
    local_view_.prediction_result = prediction_result_msg_;
    local_view_.road_info = road_info_msg_;
    local_view_.localization_estimate = localization_estimate_msg_;
    local_view_.fusion_objects_info = fusion_objects_info_msg_;
    local_view_.vehicel_service_output_info = vehicel_service_output_info_msg_;
    local_view_.radar_perception_objects_info = radar_perception_objects_info_msg_;
    local_view_.control_output = control_output_msg_;
  }

  // 2.planning run
  PlanningOutput::PlanningOutput planning_output;
  DebugOutput debug_output;

  std::cout << "==============The planning enters RunOnce=============" << std::endl;
  planning_base_->RunOnce(local_view_, &planning_output, debug_output);

  // 3.get output & publish
  // 重新刷新相对时间 - TBD
  // const double dt = start_time - adc_trajectory_pb.header().timestamp_sec();
  // for (auto& p : *adc_trajectory_pb.mutable_trajectory_point()) {
  //   p.set_relative_time(p.relative_time() + dt);
  // }

  planning::common::PlanningDebugInfo planning_debug_data;
  planning_debug_data.set_timestamp(IflyTime::Now_ms());
  planning_debug_data.set_data_json(debug_output.data.data_json());
  planning_debug_writer_->Write(planning_debug_data);

  // fill planning_debug_info —需要好好设计一下
  auto debug_info = mjson::Json(mjson::Json::object());

  debug_info["fix_lane_a"]=debug_output.fix_lane.a;
  debug_info["fix_lane_b"]=debug_output.fix_lane.b;
  debug_info["fix_lane_c"]=debug_output.fix_lane.c;
  debug_info["fix_lane_d"]=debug_output.fix_lane.d;
  if (debug_output.target_lane.a!=0&&debug_output.target_lane.b!=0&&
      debug_output.target_lane.c!=0&&debug_output.target_lane.d!=0) {
      debug_info["target_lane_a"]=debug_output.target_lane.a;
      debug_info["target_lane_b"]=debug_output.target_lane.b;
      debug_info["target_lane_c"]=debug_output.target_lane.c;
      debug_info["target_lane_d"]=debug_output.target_lane.d;
  }

  // planning_output.mutable_extra()->set_json(debug_info.dump());
  planning_writer_->Write(planning_output);

  double planning_cost_time = IflyTime::Now_ms() - start_time;
  if (planning_cost_time > 50.0) {
    LOG_ERROR("The cost time of proc() is: [%f] ms!\n", planning_cost_time);
  }
  LOG_WARNING("The cost time of proc() is: [%f] ms\n", planning_cost_time);

  return true;
}

DebugOutput PlanningComponent::GetDebugInfo() {
  return debug_info_;
}

}  // namespace planning
