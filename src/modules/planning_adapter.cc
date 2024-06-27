#include "planning_adapter.h"

#include <sys/types.h>

#include <cstdint>
#include <memory>

#include "common.pb.h"
#include "common/config_context.h"
#include "debug_info_log.h"
#include "general_planning_context.h"
#include "ifly_time.h"
#include "version.h"

namespace planning {

void PlanningAdapter::Init() {
  std::cout << "The planning component init!!!" << std::endl;
  std::string engine_config_path = PLANNING_ENGINE_CONFIG_PATH;
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

  local_view_ptr_ = std::make_shared<LocalView>();
  planning_scheduler_ =
      std::make_unique<PlanningScheduler>(local_view_ptr_.get());
}

static uint64_t get_latency(double now, uint64_t input_time) {
  constexpr double US_PER_MS = 1000.0;
  if (input_time == 0) {
    return 0;
  }
  return (now - input_time) / US_PER_MS;
}

static inline void calc_fusion_latency(
    uint64 planning_in_time_us, const iflyauto::Header &header,
    planning::common::ImageLatency *latency) {
  constexpr uint64_t US_PER_MS = 1000;

  u_int64_t image_expose_time_us = 0;
  u_int64_t perception_in_time_us = 0;
  u_int64_t perception_out_time_us = 0;
  u_int64_t fusion_in_time_us = 0;
  u_int64_t fusion_out_time_us = 0;

  for (int i = 0; i < header.input_list_size; i++) {
    const auto &input = header.input_list[i];
    switch (input.input_type) {
      case Common::InputHistoryTimestamp::
          INPUT_HISTORY_TIMESTAMP_SOURCE_TYPE_CAMERA: {
        perception_in_time_us = input.in_ts_us;
        break;
      }
      case Common::InputHistoryTimestamp::
          INPUT_HISTORY_TIMESTAMP_SOURCE_TYPE_PERCEPTION: {
        perception_in_time_us = input.in_ts_us;
        perception_out_time_us = input.out_ts_us;
        break;
      }
      case Common::InputHistoryTimestamp::
          INPUT_HISTORY_TIMESTAMP_SOURCE_TYPE_OBSTACLE_FUSION: {
        fusion_in_time_us = input.in_ts_us;
        fusion_out_time_us = input.out_ts_us;
        break;
      }
      case Common::InputHistoryTimestamp::
          INPUT_HISTORY_TIMESTAMP_SOURCE_TYPE_ROAD_FUSION: {
        fusion_in_time_us = input.in_ts_us;
        fusion_out_time_us = input.out_ts_us;
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

static void calc_location_latency(uint64 planning_in_time_us,
                                  const iflyauto::Header &header,
                                  planning::common::LocationLatency *latency) {
  constexpr uint64_t US_PER_MS = 1000;

  u_int64_t sensor_time_us = 0;
  u_int64_t location_in_time_us = 0;
  u_int64_t location_out_time_us = 0;

  for (int i = 0; i < header.input_list_size; i++) {
    const auto &input = header.input_list[i];
    switch (input.input_type) {
      case Common::InputHistoryTimestamp::
          INPUT_HISTORY_TIMESTAMP_SOURCE_TYPE_IMU: {
        sensor_time_us = input.in_ts_us;
        break;
      }
      case Common::InputHistoryTimestamp::
          INPUT_HISTORY_TIMESTAMP_SOURCE_TYPE_LOCALIZATION: {
        location_in_time_us = input.in_ts_us;
        location_out_time_us = input.out_ts_us;
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
  LOG_DEBUG("PlanningScheduler::RunOnce \n");
  double start_time = IflyTime::Now_us();

  auto &planning_debug_data = DebugInfoManager::GetInstance().GetDebugInfoPb();

  // 1.update all inputs to local_view
  auto input_topic_timestamp =
      planning_debug_data->mutable_input_topic_timestamp();
  auto input_topic_latency = planning_debug_data->mutable_input_topic_latency();

  if (is_prediction_result_msg_updated_) {
    std::lock_guard<std::mutex> lock(msg_mutex_);
    local_view_ptr_->prediction_result = prediction_result_msg_;
    local_view_ptr_->prediction_result_recv_time =
        prediction_result_msg_recv_time_;
    is_prediction_result_msg_updated_.store(false);
  }
  input_topic_timestamp->set_prediction(
      local_view_ptr_->prediction_result.header.timestamp);
  input_topic_latency->set_prediction(get_latency(
      start_time, local_view_ptr_->prediction_result.header.timestamp));

  if (is_road_info_msg_updated_) {
    std::lock_guard<std::mutex> lock(msg_mutex_);
    local_view_ptr_->road_info = road_info_msg_;
    local_view_ptr_->road_info_recv_time = road_info_msg_recv_time_;
    is_road_info_msg_updated_.store(false);
  }
  input_topic_timestamp->set_fusion_road(
      local_view_ptr_->road_info.header.timestamp);
  input_topic_latency->set_fusion_road(
      get_latency(start_time, local_view_ptr_->road_info.header.timestamp));

  if (is_ground_line_perception_msg_updated_) {
    std::lock_guard<std::mutex> lock(msg_mutex_);
    local_view_ptr_->ground_line_perception = ground_line_perception_msg_;
    local_view_ptr_->ground_line_perception_recv_time =
        ground_line_perception_msg_recv_time_;
    is_ground_line_perception_msg_updated_.store(false);
  }
  input_topic_timestamp->set_ground_line(
      local_view_ptr_->ground_line_perception.header.timestamp);
  input_topic_latency->set_ground_line(get_latency(
      start_time, local_view_ptr_->ground_line_perception.header.timestamp));

  if (is_localization_msg_updated_) {
    std::lock_guard<std::mutex> lock(msg_mutex_);
    local_view_ptr_->localization = localization_msg_;
    local_view_ptr_->localization_recv_time = localization_msg_recv_time_;
    is_localization_msg_updated_.store(false);
  }
  input_topic_timestamp->set_localization(
      local_view_ptr_->localization.header.timestamp);
  input_topic_latency->set_localization(
      get_latency(start_time, local_view_ptr_->localization.header.timestamp));

  if (is_ground_line_perception_msg_updated_) {
    std::lock_guard<std::mutex> lock(msg_mutex_);
    local_view_ptr_->ground_line_perception = ground_line_perception_msg_;
    local_view_ptr_->ground_line_perception_recv_time = ground_line_perception_msg_recv_time_;
    is_ground_line_perception_msg_updated_.store(false);
  }
  input_topic_timestamp->set_ground_line(
      local_view_ptr_->road_info.header.timestamp);
  input_topic_latency->set_ground_line(
      get_latency(start_time, local_view_ptr_->ground_line_perception.header.timestamp));

  if (is_localization_estimate_msg_updated_) {
    std::lock_guard<std::mutex> lock(msg_mutex_);
    local_view_ptr_->localization_estimate = localization_estimate_msg_;
    local_view_ptr_->localization_estimate_recv_time =
        localization_estimate_msg_recv_time_;
    is_localization_estimate_msg_updated_.store(false);
  }
  input_topic_timestamp->set_localization_estimate(
      local_view_ptr_->localization_estimate.header.timestamp);
  input_topic_latency->set_localization(get_latency(
      start_time, local_view_ptr_->localization_estimate.header.timestamp));

  if (is_localization_msg_updated_) {
    std::lock_guard<std::mutex> lock(msg_mutex_);
    local_view_ptr_->localization = localization_msg_;
    local_view_ptr_->localization_recv_time = localization_msg_recv_time_;
    is_localization_msg_updated_.store(false);
  }
  input_topic_timestamp->set_localization(
      local_view_ptr_->localization.header.timestamp);
  input_topic_latency->set_localization(
      get_latency(start_time, local_view_ptr_->localization.header.timestamp));

  if (is_fusion_objects_info_msg_updated_) {
    std::lock_guard<std::mutex> lock(msg_mutex_);
    local_view_ptr_->fusion_objects_info = fusion_objects_info_msg_;
    local_view_ptr_->fusion_objects_info_recv_time =
        fusion_objects_info_msg_recv_time_;
    is_fusion_objects_info_msg_updated_.store(false);
  }
  input_topic_timestamp->set_fusion_object(
      local_view_ptr_->fusion_objects_info.header.timestamp);
  input_topic_latency->set_fusion_object(get_latency(
      start_time, local_view_ptr_->fusion_objects_info.header.timestamp));

  if (is_vehicle_service_output_info_msg_updated_) {
    std::lock_guard<std::mutex> lock(msg_mutex_);
    local_view_ptr_->vehicle_service_output_info =
        vehicle_service_output_info_msg_;
    local_view_ptr_->vehicle_service_output_info_recv_time =
        vehicle_service_output_info_msg_recv_time_;
    is_vehicle_service_output_info_msg_updated_.store(false);
  }
  input_topic_timestamp->set_vehicle_service(
      local_view_ptr_->vehicle_service_output_info.header.timestamp);
  input_topic_latency->set_vehicle_service(get_latency(
      start_time,
      local_view_ptr_->vehicle_service_output_info.header.timestamp));

  if (is_control_output_msg_updated_) {
    std::lock_guard<std::mutex> lock(msg_mutex_);
    local_view_ptr_->control_output = control_output_msg_;
    local_view_ptr_->control_output_recv_time = control_output_msg_recv_time_;
    is_control_output_msg_updated_.store(false);
  }
  input_topic_timestamp->set_control_output(
      local_view_ptr_->control_output.header.timestamp);
  input_topic_latency->set_control_output(get_latency(
      start_time, local_view_ptr_->control_output.header.timestamp));

  if (is_hmi_mcu_inner_info_msg_updated_) {
    std::lock_guard<std::mutex> lock(msg_mutex_);
    local_view_ptr_->hmi_mcu_inner_info = hmi_mcu_inner_info_msg_;
    local_view_ptr_->hmi_mcu_inner_info_recv_time =
        hmi_mcu_inner_info_msg_recv_time_;
    is_hmi_mcu_inner_info_msg_updated_.store(false);
  }
  input_topic_timestamp->set_hmi(
      local_view_ptr_->hmi_mcu_inner_info.header.timestamp);
  input_topic_latency->set_hmi(get_latency(
      start_time, local_view_ptr_->hmi_mcu_inner_info.header.timestamp));

  if (is_parking_fusion_info_msg_updated_) {
    std::lock_guard<std::mutex> lock(msg_mutex_);
    local_view_ptr_->parking_fusion_info = parking_fusion_info_msg_;
    local_view_ptr_->parking_fusion_info_recv_time =
        parking_fusion_info_msg_recv_time_;
    is_parking_fusion_info_msg_updated_.store(false);
  }
  input_topic_timestamp->set_parking_fusion(
      local_view_ptr_->parking_fusion_info.header.timestamp);
  input_topic_latency->set_parking_fusion(get_latency(
      start_time, local_view_ptr_->parking_fusion_info.header.timestamp));

  if (is_func_state_machine_msg_updated_) {
    std::lock_guard<std::mutex> lock(msg_mutex_);
    local_view_ptr_->function_state_machine_info = func_state_machine_msg_;
    local_view_ptr_->function_state_machine_info_recv_time =
        func_state_machine_msg_recv_time_;
    is_func_state_machine_msg_updated_.store(false);
  }
  input_topic_timestamp->set_function_state_machine(
      local_view_ptr_->function_state_machine_info.header.timestamp);
  input_topic_latency->set_function_state_machine(get_latency(
      start_time,
      local_view_ptr_->function_state_machine_info.header.timestamp));

  if (is_uss_wave_info_msg_updated_) {
    std::lock_guard<std::mutex> lock(msg_mutex_);
    local_view_ptr_->uss_wave_info = uss_wave_info_msg_;
    is_uss_wave_info_msg_updated_.store(false);
  }

  if (is_uss_percept_info_msg_updated_) {
    std::lock_guard<std::mutex> lock(msg_mutex_);
    local_view_ptr_->uss_percept_info = uss_percept_info_msg_;
    is_uss_percept_info_msg_updated_.store(false);
  }

  if (is_map_info_msg_updated_) {
    std::lock_guard<std::mutex> lock(msg_mutex_);
    local_view_ptr_->static_map_info = map_info_msg_;
    local_view_ptr_->static_map_info_recv_time = map_info_msg_recv_time_;
    is_map_info_msg_updated_.store(false);
  }
  input_topic_timestamp->set_map(
      local_view_ptr_->static_map_info.header().timestamp());
  input_topic_latency->set_map(get_latency(
      start_time, local_view_ptr_->static_map_info.header().timestamp()));

  // update general context
  auto &state_machine_g = g_context.MutableStatemachine();

  const auto &current_state =
      local_view_ptr_->function_state_machine_info.current_state;

  const auto &last_state = g_context.GetStatemachine().current_state;

  if (last_state == iflyauto::FunctionalState_STANDBY &&
      (current_state >= iflyauto::FunctionalState_PARK_IN_APA_IN &&
       current_state <= iflyauto::FunctionalState_PARK_IN_COMPLETED)) {
    state_machine_g.apa_reset_flag = true;
  } else {
    state_machine_g.apa_reset_flag = false;
  }

  // APA plan once when state machine changes from no ready to other parking in
  // state
  if (last_state == iflyauto::FunctionalState_PARK_IN_NO_READY &&
      (current_state == iflyauto::FunctionalState_PARK_IN_READY ||
       current_state >= iflyauto::FunctionalState_PARK_IN_ACTIVATE_CONTROL)) {
    state_machine_g.apa_start_plan_once_flag = true;
  } else {
    state_machine_g.apa_start_plan_once_flag = false;
  }

  state_machine_g.current_state = current_state;

  // 2.planning run
  auto planning_output_container =
      std::make_shared<iflyauto::StructContainer>();
  auto &planning_output = *iflyauto::struct_cast<iflyauto::PlanningOutput>(
      planning_output_container);
  auto planning_hmi_info_container =
      std::make_shared<iflyauto::StructContainer>();
  auto &planning_hmi_info =
      *iflyauto::struct_cast<iflyauto::PlanningHMIOutputInfoStr>(
          planning_hmi_info_container);
  auto planning_debuginfo_container =
      std::make_shared<iflyauto::StructContainer>();

  std::cout << "==============The planning enters RunOnce============="
            << std::endl;

  const bool run_success =
      planning_scheduler_->RunOnce(&planning_output, &planning_hmi_info);

  // 3.get output & publish
  uint64_t output_time_us = (uint64_t)IflyTime::Now_us();
  frame_num_++;

  if (planning_debug_writer_) {
    planning_debug_data->set_timestamp(output_time_us);
    planning_debug_data->mutable_frame_info()->set_version(__version_str__);
    const auto &debug_info_json =
        *DebugInfoManager::GetInstance().GetDebugJson();
    planning_debug_data->set_data_json(mjson::Json(debug_info_json).dump());
    calc_fusion_latency(start_time, local_view_ptr_->road_info.header,
                        planning_debug_data->mutable_road_fusion_latency());
    calc_fusion_latency(start_time, local_view_ptr_->fusion_objects_info.header,
                        planning_debug_data->mutable_obstacle_fusion_latency());
    calc_location_latency(start_time,
                          local_view_ptr_->localization_estimate.header,
                          planning_debug_data->mutable_location_latency());

    auto frame_info = planning_debug_data->mutable_frame_info();
    frame_info->set_frame_num(frame_num_);
    const auto frame_duration = (output_time_us - start_time) * 1e-3;
    frame_info->set_frame_duration_ms(frame_duration);
    frame_info->set_planning_succ(run_success);

    auto payload = planning_debuginfo_container->mutable_payload();
    planning_debug_data->SerializeToString(payload);
    planning_debug_writer_(planning_debuginfo_container);
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
    auto &header = planning_output.meta.header;
    header.timestamp = output_time_us;
    iflyauto::strcpy_array(header.version, __version_str__);
    // TODO
    // header->input_list_size =
    // local_view_ptr_->fusion_objects_info.header.input_list_size; for (int i =
    // 0; i < local_view_ptr_->fusion_objects_info.header.input_list_size; i++)
    // {
    //   header->input_list[i] =
    //   local_view_ptr_->fusion_objects_info.header.input_list[i];
    // }
    // for (int i = ; i < local_view_ptr_->road_info.header.input_list_size;
    // i++) {
    //   header->input_list[header->input_list_size + i] =
    //   local_view_ptr_->road_info.header.input_list[i];
    // }
    // for (int i = ; i <
    // local_view_ptr_->localization_estimate.header.input_list_size; i++) {
    //   header->input_list[header->input_list_size + i] =
    //   local_view_ptr_->localization_estimate.header.input_list[i];
    // }
    // auto &planning_latency = header->mutable_input_list()->Add();
    // planning_latency->set_input_type(
    //     Common::InputHistoryTimestamp::
    //         INPUT_HISTORY_TIMESTAMP_SOURCE_TYPE_PLANNING);
    // planning_latency->set_in_ts_us(start_time);
    // planning_latency->set_out_ts_us(output_time_us);
    planning_writer_(planning_output_container);
  }

  if (planning_hmi_info_writer_) {
    planning_hmi_info.header.timestamp = output_time_us;
    iflyauto::strcpy_array(planning_hmi_info.header.version, __version_str__);
    planning_hmi_info_writer_(planning_hmi_info_container);
  }
  double planning_cost_time = (IflyTime::Now_us() - start_time) / 1000;
  LOG_WARNING("The cost time of proc() is: [%f] ms\n", planning_cost_time);
}

}  // namespace planning
