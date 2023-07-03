#include "planning_adapter.h"

#include <sys/types.h>

#include <cstdint>

#include "common/config_context.h"
#include "debug_info_log.h"
#include "ifly_time.h"

namespace planning {

void PlanningAdapter::Init() {
  std::cout << "The planning component init!!!" << std::endl;
  const std::string CONFIG_PATH = "/asw/planning/res/conf";
  std::string engine_config_path = CONFIG_PATH + "/planning_engine_config.json";
  common::ConfigurationContext::Instance()->load_engine_config_from_json(
      engine_config_path);

  auto engine_config =
      common::ConfigurationContext::Instance()->engine_config();

  std::string log_file = engine_config.log_conf.log_file_dir + "/planning_log";
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

  planning_base_ = std::make_unique<GeneralPlanning>();
}

static uint64_t get_latency(double now, uint64_t input_time) {
  constexpr double US_PER_MS = 1000.0;
  if (input_time == 0) {
    return 0;
  }
  return (now - input_time) / US_PER_MS;
}

void PlanningAdapter::Proc() {
  LOG_DEBUG("GeneralPlanning::RunOnce \n");
  double start_time = IflyTime::Now_ms();

  auto &planning_debug_data = DebugInfoManager::GetInstance().GetDebugInfoPb();

  // 1.update all inputs to local_view
  {
    std::lock_guard<std::mutex> lock(msg_mutex_);
    auto input_topic_timestamp =
        planning_debug_data->mutable_input_topic_timestamp();
    auto input_topic_latency =
        planning_debug_data->mutable_input_topic_latency();

    local_view_.prediction_result = prediction_result_msg_;
    local_view_.prediction_result_recv_time = prediction_result_msg_recv_time_;
    input_topic_timestamp->set_prediction(
        prediction_result_msg_.header().timestamp());
    input_topic_latency->set_prediction(
        get_latency(start_time, prediction_result_msg_.header().timestamp()));
    prediction_result_msg_.Clear();

    local_view_.road_info = road_info_msg_;
    local_view_.road_info_recv_time = road_info_msg_recv_time_;
    input_topic_timestamp->set_fusion_road(road_info_msg_.header().timestamp());
    input_topic_latency->set_fusion_road(
        get_latency(start_time, road_info_msg_.header().timestamp()));
    road_info_msg_.Clear();

    local_view_.localization_estimate = localization_estimate_msg_;
    local_view_.localization_estimate_recv_time =
        localization_estimate_msg_recv_time_;
    input_topic_timestamp->set_localization(
        localization_estimate_msg_.header().timestamp());
    input_topic_latency->set_localization(get_latency(
        start_time, localization_estimate_msg_.header().timestamp()));
    localization_estimate_msg_.Clear();

    local_view_.fusion_objects_info = fusion_objects_info_msg_;
    local_view_.fusion_objects_info_recv_time =
        fusion_objects_info_msg_recv_time_;
    input_topic_timestamp->set_fusion_object(
        fusion_objects_info_msg_.header().timestamp());
    input_topic_latency->set_fusion_object(
        get_latency(start_time, fusion_objects_info_msg_.header().timestamp()));
    fusion_objects_info_msg_.Clear();

    local_view_.vehicel_service_output_info = vehicel_service_output_info_msg_;
    local_view_.vehicel_service_output_info_recv_time =
        vehicel_service_output_info_msg_recv_time_;
    input_topic_timestamp->set_vehicle_service(
        vehicel_service_output_info_msg_.header().timestamp());
    input_topic_latency->set_vehicle_service(get_latency(
        start_time, vehicel_service_output_info_msg_.header().timestamp()));
    vehicel_service_output_info_msg_.Clear();

    local_view_.control_output = control_output_msg_;
    local_view_.control_output_recv_time = control_output_msg_recv_time_;
    input_topic_timestamp->set_control_output(
        control_output_msg_.header().timestamp());
    input_topic_latency->set_control_output(
        get_latency(start_time, control_output_msg_.header().timestamp()));
    control_output_msg_.Clear();

    local_view_.hmi_mcu_inner_info = hmi_mcu_inner_info_msg_;
    local_view_.hmi_mcu_inner_info_recv_time =
        hmi_mcu_inner_info_msg_recv_time_;
    input_topic_timestamp->set_hmi(
        hmi_mcu_inner_info_msg_.header().timestamp());
    input_topic_latency->set_hmi(
        get_latency(start_time, hmi_mcu_inner_info_msg_.header().timestamp()));
    hmi_mcu_inner_info_msg_.Clear();

    local_view_.parking_fusion_info = parking_fusion_info_msg_;
    local_view_.parking_fusion_info_recv_time =
        parking_fusion_info_msg_recv_time_;
    input_topic_timestamp->set_parking_fusion(
        parking_fusion_info_msg_.header().timestamp());
    input_topic_latency->set_parking_fusion(
        get_latency(start_time, parking_fusion_info_msg_.header().timestamp()));
    parking_fusion_info_msg_.Clear();

    local_view_.function_state_machine_info = func_state_machine_msg_;
    input_topic_timestamp->set_function_state_machine(
        func_state_machine_msg_.header().timestamp());
    input_topic_latency->set_function_state_machine(
        get_latency(start_time, func_state_machine_msg_.header().timestamp()));
    func_state_machine_msg_.Clear();
  }

  // 2.planning run
  PlanningOutput::PlanningOutput planning_output;
  DebugOutput debug_output;
  PlanningHMI::PlanningHMIOutputInfoStr planning_hmi_info;
  std::cout << "==============The planning enters RunOnce============="
            << std::endl;
  bool run_success = planning_base_->RunOnce(
      local_view_, &planning_output, *planning_debug_data, &planning_hmi_info);

  // 3.get output & publish
  if (planning_debug_writer_) {
    planning_debug_data->set_timestamp(IflyTime::Now_us());
    auto debug_info_json = *DebugInfoManager::GetInstance().GetDebugJson();
    planning_debug_data->set_data_json(mjson::Json(debug_info_json).dump());
    planning_debug_writer_(*planning_debug_data);
  }

  if (planning_writer_) {
    if (run_success) {
      last_planning_output_ = planning_output;
    } else {
      // use last succ planning output when planning not succ
      // if never succeed, output will bi empty
      planning_output = last_planning_output_;
      LOG_WARNING("planning failed, use last planning output\n");
    }
    auto time_stamp_us = IflyTime::Now_us();
    auto header = planning_output.mutable_meta()->mutable_header();
    header->set_timestamp(time_stamp_us);
    header->set_version("TEST");
    planning_writer_(planning_output);
  }

  if (planning_hmi_info_writer_) {
    planning_hmi_info_writer_(planning_hmi_info);
  }
  double planning_cost_time = IflyTime::Now_ms() - start_time;
  LOG_WARNING("The cost time of proc() is: [%f] ms\n", planning_cost_time);
}

}  // namespace planning
