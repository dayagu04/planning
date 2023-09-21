#include "planning_adapter.h"

#include <sys/types.h>

#include <cstdint>

#include "common.pb.h"
#include "common/config_context.h"
#include "debug_info_log.h"
#include "func_state_machine.pb.h"
#include "general_planning_context.h"
#include "ifly_time.h"
#include "planning_debug_info.pb.h"
#include "version.h"

namespace planning {

void PlanningAdapter::Init() {
  std::cout << "The planning component init!!!" << std::endl;
  const std::string CONFIG_PATH = "/asw/planning/res/conf";
  std::string engine_config_path = CONFIG_PATH + "/planning_engine_config.json";
  common::ConfigurationContext::Instance()->load_engine_config_from_json(
      engine_config_path);

  auto engine_config =
      common::ConfigurationContext::Instance()->engine_config();

  std::string log_file = engine_config.log_conf.log_file;
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

static void calc_fusion_latency(
    uint64 planning_in_time_us,
    const google::protobuf::RepeatedPtrField<Common::InputHistoryTimestamp>
        &input_timestamp_list,
    planning::common::ImageLatency *latency,
    Common::InputHistoryTimestamp::InputHistoryTimestampSourceType
        fusion_type) {
  constexpr uint64_t US_PER_MS = 1000;

  uint64 image_expose_time_us = 0;
  uint64 perception_in_time_us = 0;
  uint64 perception_out_time_us = 0;
  uint64 fusion_in_time_us = 0;
  uint64 fusion_out_time_us = 0;

  for (auto &input : input_timestamp_list) {
    switch (input.input_type()) {
      case Common::InputHistoryTimestamp::
          INPUT_HISTORY_TIMESTAMP_SOURCE_TYPE_CAMERA: {
        perception_in_time_us = input.in_ts_us();
        break;
      }
      case Common::InputHistoryTimestamp::
          INPUT_HISTORY_TIMESTAMP_SOURCE_TYPE_PERCEPTION: {
        perception_in_time_us = input.in_ts_us();
        perception_out_time_us = input.out_ts_us();
        break;
      }
      case Common::InputHistoryTimestamp::
          INPUT_HISTORY_TIMESTAMP_SOURCE_TYPE_OBSTACLE_FUSION: {
        if (fusion_type ==
            Common::InputHistoryTimestamp::
                INPUT_HISTORY_TIMESTAMP_SOURCE_TYPE_OBSTACLE_FUSION) {
          fusion_in_time_us = input.in_ts_us();
          fusion_out_time_us = input.out_ts_us();
        }
        break;
      }
      case Common::InputHistoryTimestamp::
          INPUT_HISTORY_TIMESTAMP_SOURCE_TYPE_ROAD_FUSION: {
        if (fusion_type ==
            Common::InputHistoryTimestamp::
                INPUT_HISTORY_TIMESTAMP_SOURCE_TYPE_ROAD_FUSION) {
          fusion_in_time_us = input.in_ts_us();
          fusion_out_time_us = input.out_ts_us();
        }
        break;
      }
      default: { break; }
    }
  }

  latency->set_image_com_latency_ms(
      (perception_in_time_us - image_expose_time_us) / US_PER_MS);
  latency->set_perception_latency_ms(
      (perception_out_time_us - perception_in_time_us) / US_PER_MS);
  latency->set_perception_com_latency_ms(
      (fusion_in_time_us - perception_out_time_us) / US_PER_MS);
  latency->set_fusion_lantency_ms((fusion_out_time_us - fusion_in_time_us) /
                                  US_PER_MS);
  latency->set_fusion_com_lantency_ms(
      (planning_in_time_us - fusion_out_time_us) / US_PER_MS);
}

static void calc_location_latency(
    uint64 planning_in_time_us,
    const google::protobuf::RepeatedPtrField<Common::InputHistoryTimestamp>
        &input_timestamp_list,
    planning::common::LocationLatency *latency) {
  constexpr uint64_t US_PER_MS = 1000;

  uint64 sensor_time_us = 0;
  uint64 location_in_time_us = 0;
  uint64 location_out_time_us = 0;

  for (auto &input : input_timestamp_list) {
    switch (input.input_type()) {
      case Common::InputHistoryTimestamp::
          INPUT_HISTORY_TIMESTAMP_SOURCE_TYPE_IMU: {
        sensor_time_us = input.in_ts_us();
        break;
      }
      case Common::InputHistoryTimestamp::
          INPUT_HISTORY_TIMESTAMP_SOURCE_TYPE_LOCALIZATION: {
        location_in_time_us = input.in_ts_us();
        location_out_time_us = input.out_ts_us();
        break;
      }
      default: { break; }
    }
  }

  latency->set_sensor_mcu_latency_ms(0);
  latency->set_sensor_soc_latency_ms(0);
  latency->set_sensor_com_latency_ms((location_in_time_us - sensor_time_us) /
                                     US_PER_MS);
  latency->set_location_latency((location_out_time_us - location_in_time_us) /
                                US_PER_MS);
}

void PlanningAdapter::Proc() {
  LOG_DEBUG("GeneralPlanning::RunOnce \n");
  double start_time = IflyTime::Now_us();

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

    local_view_.road_info = road_info_msg_;
    local_view_.road_info_recv_time = road_info_msg_recv_time_;
    input_topic_timestamp->set_fusion_road(road_info_msg_.header().timestamp());
    input_topic_latency->set_fusion_road(
        get_latency(start_time, road_info_msg_.header().timestamp()));

    local_view_.localization_estimate = localization_estimate_msg_;
    local_view_.localization_estimate_recv_time =
        localization_estimate_msg_recv_time_;
    input_topic_timestamp->set_localization(
        localization_estimate_msg_.header().timestamp());
    input_topic_latency->set_localization(get_latency(
        start_time, localization_estimate_msg_.header().timestamp()));

    local_view_.fusion_objects_info = fusion_objects_info_msg_;
    local_view_.fusion_objects_info_recv_time =
        fusion_objects_info_msg_recv_time_;
    input_topic_timestamp->set_fusion_object(
        fusion_objects_info_msg_.header().timestamp());
    input_topic_latency->set_fusion_object(
        get_latency(start_time, fusion_objects_info_msg_.header().timestamp()));

    local_view_.vehicle_service_output_info = vehicle_service_output_info_msg_;
    local_view_.vehicle_service_output_info_recv_time =
        vehicle_service_output_info_msg_recv_time_;
    input_topic_timestamp->set_vehicle_service(
        vehicle_service_output_info_msg_.header().timestamp());
    input_topic_latency->set_vehicle_service(get_latency(
        start_time, vehicle_service_output_info_msg_.header().timestamp()));

    local_view_.control_output = control_output_msg_;
    local_view_.control_output_recv_time = control_output_msg_recv_time_;
    input_topic_timestamp->set_control_output(
        control_output_msg_.header().timestamp());
    input_topic_latency->set_control_output(
        get_latency(start_time, control_output_msg_.header().timestamp()));

    local_view_.hmi_mcu_inner_info = hmi_mcu_inner_info_msg_;
    local_view_.hmi_mcu_inner_info_recv_time =
        hmi_mcu_inner_info_msg_recv_time_;
    input_topic_timestamp->set_hmi(
        hmi_mcu_inner_info_msg_.header().timestamp());
    input_topic_latency->set_hmi(
        get_latency(start_time, hmi_mcu_inner_info_msg_.header().timestamp()));

    local_view_.parking_fusion_info = parking_fusion_info_msg_;
    local_view_.parking_fusion_info_recv_time =
        parking_fusion_info_msg_recv_time_;
    input_topic_timestamp->set_parking_fusion(
        parking_fusion_info_msg_.header().timestamp());
    input_topic_latency->set_parking_fusion(
        get_latency(start_time, parking_fusion_info_msg_.header().timestamp()));

    local_view_.function_state_machine_info = func_state_machine_msg_;
    input_topic_timestamp->set_function_state_machine(
        func_state_machine_msg_.header().timestamp());
    input_topic_latency->set_function_state_machine(
        get_latency(start_time, func_state_machine_msg_.header().timestamp()));

    local_view_.uss_wave_info = uss_wave_info_msg_;

    local_view_.static_map_info = map_info_msg_;
    local_view_.static_map_info_recv_time = map_info_msg_recv_time_;
    // input_topic_timestamp->set_map(map_info_msg_.header().timestamp());
    // input_topic_latency->set_map(
    //     get_latency(start_time, map_info_msg_.header().timestamp()));
    local_view_.hdmap_time = map_info_msg_recv_time_;
    // TODO: add input topic info
  }

  // update general context
  auto &state_machine_g = g_context.MutableStatemachine();

  const auto &current_state =
      local_view_.function_state_machine_info.current_state();

  const auto &last_state = g_context.GetStatemachine().current_state;

  if (last_state == FuncStateMachine::STANDBY &&
      (current_state >= FuncStateMachine::PARK_IN_APA_IN &&
       current_state <= FuncStateMachine::PARK_IN_COMPLETED)) {
    state_machine_g.apa_reset_flag = true;
  } else {
    state_machine_g.apa_reset_flag = false;
  }

  state_machine_g.current_state = current_state;

  // 2.planning run
  PlanningOutput::PlanningOutput planning_output;
  DebugOutput debug_output;
  PlanningHMI::PlanningHMIOutputInfoStr planning_hmi_info;
  std::cout << "==============The planning enters RunOnce============="
            << std::endl;
  bool run_success = planning_base_->RunOnce(
      &local_view_, &planning_output, *planning_debug_data, &planning_hmi_info);

  // 3.get output & publish
  uint64_t output_time_us = (uint64_t)IflyTime::Now_us();
  google::protobuf::RepeatedPtrField<::Common::InputHistoryTimestamp>
      input_timestamp_list{};
  input_timestamp_list.MergeFrom(
      fusion_objects_info_msg_.header().input_list());
  input_timestamp_list.MergeFrom(road_info_msg_.header().input_list());
  input_timestamp_list.MergeFrom(
      localization_estimate_msg_.header().input_list());

  if (planning_debug_writer_) {
    planning_debug_data->set_timestamp(output_time_us);
    planning_debug_data->mutable_frame_info()->set_version(__version_str__);
    auto debug_info_json = *DebugInfoManager::GetInstance().GetDebugJson();
    planning_debug_data->set_data_json(mjson::Json(debug_info_json).dump());
    calc_fusion_latency(start_time, input_timestamp_list,
                        planning_debug_data->mutable_road_fusion_latency(),
                        Common::InputHistoryTimestamp::
                            INPUT_HISTORY_TIMESTAMP_SOURCE_TYPE_ROAD_FUSION);
    calc_fusion_latency(
        start_time, input_timestamp_list,
        planning_debug_data->mutable_obstacle_fusion_latency(),
        Common::InputHistoryTimestamp::
            INPUT_HISTORY_TIMESTAMP_SOURCE_TYPE_OBSTACLE_FUSION);
    calc_location_latency(start_time, input_timestamp_list,
                          planning_debug_data->mutable_location_latency());
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
    auto header = planning_output.mutable_meta()->mutable_header();
    header->set_timestamp(output_time_us);
    header->set_version(__version_str__);
    header->mutable_input_list()->CopyFrom(input_timestamp_list);
    auto planning_latency = header->mutable_input_list()->Add();
    planning_latency->set_input_type(
        Common::InputHistoryTimestamp::
            INPUT_HISTORY_TIMESTAMP_SOURCE_TYPE_PLANNING);
    planning_latency->set_in_ts_us(start_time);
    planning_latency->set_out_ts_us(output_time_us);
    planning_writer_(planning_output);
  }

  if (planning_hmi_info_writer_) {
    planning_hmi_info.mutable_header()->set_timestamp(output_time_us);
    planning_hmi_info.mutable_header()->set_version(__version_str__);
    planning_hmi_info_writer_(planning_hmi_info);
  }
  double planning_cost_time = (IflyTime::Now_us() - start_time) / 1000;
  LOG_WARNING("The cost time of proc() is: [%f] ms\n", planning_cost_time);
}

}  // namespace planning
