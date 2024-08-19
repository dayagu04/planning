#include "planning_adapter.h"

#include <sys/types.h>

#include <cstdint>
#include <memory>
#include <mutex>

#include "common.pb.h"
#include "common/config_context.h"
#include "debug_info_log.h"
#include "func_state_machine_c.h"
#include "general_planning_context.h"
#include "ifly_time.h"
#include "version.h"

namespace planning {

static uint64_t get_latency(double now, uint64_t input_time) {
  constexpr double US_PER_MS = 1000.0;
  if (input_time == 0) {
    return 0;
  }
  return (now - input_time) / US_PER_MS;
}

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
  planning_scheduler_ = std::make_unique<PlanningScheduler>(
      local_view_ptr_.get(), &engine_config);
}

void PlanningAdapter::ReportFmIfno(uint64 alarmId, uint64 alarmObj,
                                   bool fault_exist) {
  iflyauto::FmInfo fmInfo{};
  fmInfo.alarmId = alarmId;
  fmInfo.alarmObj = alarmObj;
  fmInfo.clss = 1;  // 告警类别，0：状态型，1：事件型
  fmInfo.level = 2;  // 告警级别，0：提示，1：次要，2：重要，3：紧急
  fmInfo.status = fault_exist ? 1 : 0;  // 告警状态，0：告警恢复，1：告警产生
  fmInfo.time = IflyTime::Now_ms();
  // fmInfo.desc; //告警描述，最长为64字节

  if (fm_info_writer_) {
    fm_info_writer_(fmInfo);
  }
}

void PlanningAdapter::Proc() {
  LOG_DEBUG("PlanningScheduler::RunOnce \n");
  double start_time = IflyTime::Now_us();

  auto &planning_debug_data = DebugInfoManager::GetInstance().GetDebugInfoPb();

  // 1.update all inputs to local_view
  auto input_topic_timestamp =
      planning_debug_data->mutable_input_topic_timestamp();
  auto input_topic_latency = planning_debug_data->mutable_input_topic_latency();

  // 1.1 receive prediction
  if (is_prediction_result_msg_updated_) {
    std::lock_guard<std::mutex> lock(msg_mutex_);
    local_view_ptr_->prediction_result = prediction_result_msg_;
    local_view_ptr_->prediction_result_recv_time =
        prediction_result_msg_recv_time_;
    is_prediction_result_msg_updated_.store(false);
  }
  input_topic_timestamp->set_prediction(
      local_view_ptr_->prediction_result.msg_header.stamp);
  input_topic_latency->set_prediction(get_latency(
      start_time, local_view_ptr_->prediction_result.msg_header.stamp));

  // 1.2 receive fusion_road
  if (is_road_info_msg_updated_) {
    std::lock_guard<std::mutex> lock(msg_mutex_);
    local_view_ptr_->road_info = road_info_msg_;
    local_view_ptr_->road_info_recv_time = road_info_msg_recv_time_;
    is_road_info_msg_updated_.store(false);
  }
  input_topic_timestamp->set_fusion_road(
      local_view_ptr_->road_info.msg_header.stamp);
  input_topic_latency->set_fusion_road(
      get_latency(start_time, local_view_ptr_->road_info.msg_header.stamp));

  // 1.3 receive localization
  if (is_localization_msg_updated_) {
    std::lock_guard<std::mutex> lock(msg_mutex_);
    local_view_ptr_->localization = localization_msg_;
    local_view_ptr_->localization_recv_time = localization_msg_recv_time_;
    is_localization_msg_updated_.store(false);
  }
  input_topic_timestamp->set_localization(
      local_view_ptr_->localization.msg_header.stamp);
  input_topic_latency->set_localization(
      get_latency(start_time, local_view_ptr_->localization.msg_header.stamp));
  // TBD: 新定位，2.4.9后应该启用
  // if (is_localization_estimate_msg_updated_) {
  //   std::lock_guard<std::mutex> lock(msg_mutex_);
  //   local_view_ptr_->localization_estimate = localization_estimate_msg_;
  //   local_view_ptr_->localization_estimate_recv_time =
  //       localization_estimate_msg_recv_time_;
  //   is_localization_estimate_msg_updated_.store(false);
  // }
  // input_topic_timestamp->set_localization_estimate(
  //     local_view_ptr_->localization_estimate.msg_header.stamp);
  // input_topic_latency->set_localization_estimate(get_latency(
  //     start_time, local_view_ptr_->localization_estimate.msg_header.stamp));

  // 1.4 receive ground_line
  if (is_ground_line_perception_msg_updated_) {
    std::lock_guard<std::mutex> lock(msg_mutex_);
    local_view_ptr_->ground_line_perception = ground_line_perception_msg_;
    local_view_ptr_->ground_line_perception_recv_time =
        ground_line_perception_msg_recv_time_;
    is_ground_line_perception_msg_updated_.store(false);
  }
  input_topic_timestamp->set_ground_line(
      local_view_ptr_->ground_line_perception.msg_header.stamp);
  input_topic_latency->set_ground_line(get_latency(
      start_time, local_view_ptr_->ground_line_perception.msg_header.stamp));

  // 1.5 receive fusion_object
  if (is_fusion_objects_info_msg_updated_) {
    std::lock_guard<std::mutex> lock(msg_mutex_);
    local_view_ptr_->fusion_objects_info = fusion_objects_info_msg_;
    local_view_ptr_->fusion_objects_info_recv_time =
        fusion_objects_info_msg_recv_time_;
    is_fusion_objects_info_msg_updated_.store(false);
  }
  input_topic_timestamp->set_fusion_object(
      local_view_ptr_->fusion_objects_info.msg_header.stamp);
  input_topic_latency->set_fusion_object(get_latency(
      start_time, local_view_ptr_->fusion_objects_info.msg_header.stamp));

  // 1.6 receive fusion_occupancy_object
  if (is_fusion_occupancy_objects_info_msg_updated_) {
    std::lock_guard<std::mutex> lock(msg_mutex_);
    local_view_ptr_->fusion_occupancy_objects_info =
        fusion_occupancy_objects_info_msg_;
    local_view_ptr_->fusion_occupancy_objects_info_recv_time =
        fusion_occupancy_objects_info_msg_recv_time_;
    is_fusion_occupancy_objects_info_msg_updated_.store(false);
  }
  input_topic_timestamp->set_fusion_occupancy_object(
      local_view_ptr_->fusion_occupancy_objects_info.msg_header.stamp);
  input_topic_latency->set_fusion_occupancy_object(get_latency(
      start_time,
      local_view_ptr_->fusion_occupancy_objects_info.msg_header.stamp));

  // 1.7 receive vehicle_service
  if (is_vehicle_service_output_info_msg_updated_) {
    std::lock_guard<std::mutex> lock(msg_mutex_);
    local_view_ptr_->vehicle_service_output_info =
        vehicle_service_output_info_msg_;
    local_view_ptr_->vehicle_service_output_info_recv_time =
        vehicle_service_output_info_msg_recv_time_;
    is_vehicle_service_output_info_msg_updated_.store(false);
  }
  input_topic_timestamp->set_vehicle_service(
      local_view_ptr_->vehicle_service_output_info.msg_header.stamp);
  input_topic_latency->set_vehicle_service(get_latency(
      start_time,
      local_view_ptr_->vehicle_service_output_info.msg_header.stamp));

  // 1.7 receive control_output
  if (is_control_output_msg_updated_) {
    std::lock_guard<std::mutex> lock(msg_mutex_);
    local_view_ptr_->control_output = control_output_msg_;
    local_view_ptr_->control_output_recv_time = control_output_msg_recv_time_;
    is_control_output_msg_updated_.store(false);
  }
  input_topic_timestamp->set_control_output(
      local_view_ptr_->control_output.msg_header.stamp);
  input_topic_latency->set_control_output(get_latency(
      start_time, local_view_ptr_->control_output.msg_header.stamp));

  // 不再接收hmi消息，统一从状态机获取
  // if (is_hmi_inner_info_msg_updated_) {
  //   std::lock_guard<std::mutex> lock(msg_mutex_);
  //   local_view_ptr_->hmi_inner_info = hmi_inner_info_msg_;
  //   local_view_ptr_->hmi_inner_info_recv_time =
  //   hmi_inner_info_msg_recv_time_;
  //   is_hmi_inner_info_msg_updated_.store(false);
  // }
  // input_topic_timestamp->set_hmi(
  //     local_view_ptr_->hmi_inner_info.msg_header.stamp);
  // input_topic_latency->set_hmi(get_latency(
  //     start_time, local_view_ptr_->hmi_inner_info.msg_header.stamp));

  // 1.7 receive parking_fusion
  if (is_parking_fusion_info_msg_updated_) {
    std::lock_guard<std::mutex> lock(msg_mutex_);
    local_view_ptr_->parking_fusion_info = parking_fusion_info_msg_;
    local_view_ptr_->parking_fusion_info_recv_time =
        parking_fusion_info_msg_recv_time_;
    is_parking_fusion_info_msg_updated_.store(false);
  }
  input_topic_timestamp->set_parking_fusion(
      local_view_ptr_->parking_fusion_info.msg_header.stamp);
  input_topic_latency->set_parking_fusion(get_latency(
      start_time, local_view_ptr_->parking_fusion_info.msg_header.stamp));

  // 1.8 receive function_state_machine
  if (is_func_state_machine_msg_updated_) {
    std::lock_guard<std::mutex> lock(msg_mutex_);
    local_view_ptr_->function_state_machine_info = func_state_machine_msg_;
    local_view_ptr_->function_state_machine_info_recv_time =
        func_state_machine_msg_recv_time_;
    is_func_state_machine_msg_updated_.store(false);
  }
  input_topic_timestamp->set_function_state_machine(
      local_view_ptr_->function_state_machine_info.msg_header.stamp);
  input_topic_latency->set_function_state_machine(get_latency(
      start_time,
      local_view_ptr_->function_state_machine_info.msg_header.stamp));

  // 1.9 receive uss_wave
  if (is_uss_wave_info_msg_updated_) {
    std::lock_guard<std::mutex> lock(msg_mutex_);
    local_view_ptr_->uss_wave_info = uss_wave_info_msg_;
    local_view_ptr_->uss_wave_info_recv_time = uss_wave_info_msg_recv_time_;
    is_uss_wave_info_msg_updated_.store(false);
  }
  input_topic_timestamp->set_uss_wave(
      local_view_ptr_->uss_wave_info.msg_header.stamp);
  input_topic_latency->set_uss_wave(
      get_latency(start_time, local_view_ptr_->uss_wave_info.msg_header.stamp));

  // 1.10 receive uss_perception
  if (is_uss_percept_info_msg_updated_) {
    std::lock_guard<std::mutex> lock(msg_mutex_);
    local_view_ptr_->uss_percept_info = uss_percept_info_msg_;
    local_view_ptr_->uss_percept_info_recv_time =
        uss_percept_info_msg_recv_time_;
    is_uss_percept_info_msg_updated_.store(false);
  }
  input_topic_timestamp->set_uss_perception(
      local_view_ptr_->uss_percept_info.msg_header.stamp);
  input_topic_latency->set_uss_perception(get_latency(
      start_time, local_view_ptr_->uss_percept_info.msg_header.stamp));

  // 1.11 receive static_map
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

  // 1.12 receive sd_map
  if (is_sd_map_info_msg_updated_) {
    std::lock_guard<std::mutex> lock(msg_mutex_);
    local_view_ptr_->sd_map_info = sd_map_info_msg_;
    local_view_ptr_->sd_map_info_recv_time = sd_map_info_msg_recv_time_;
    is_sd_map_info_msg_updated_.store(false);
  }
  input_topic_timestamp->set_map(
      local_view_ptr_->sd_map_info.header().timestamp());
  input_topic_latency->set_map(get_latency(
      start_time, local_view_ptr_->sd_map_info.header().timestamp()));

  // update general context
  auto &state_machine_g = g_context.MutableStatemachine();

  const auto &current_state =
      local_view_ptr_->function_state_machine_info.current_state;

  const auto &last_state = g_context.GetStatemachine().current_state;

  if ((last_state == iflyauto::FunctionalState_MANUAL ||
       last_state == iflyauto::FunctionalState_PARK_STANDBY) &&
      (current_state >= iflyauto::FunctionalState_PARK_IN_SEARCHING &&
       current_state <= iflyauto::FunctionalState_PARK_OUT_SEARCHING)) {
    state_machine_g.apa_reset_flag = true;
  } else {
    state_machine_g.apa_reset_flag = false;
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
    auto &msg_header = planning_output.msg_header;
    auto &msg_meta = planning_output.msg_meta;
    msg_header.stamp = output_time_us;
    // iflyauto::strcpy_array(header.version, __version_str__);
    msg_header.seq = frame_num_;
    UpdateInputListInfo(msg_meta);

    planning_writer_(planning_output_container);
  }

  if (planning_hmi_info_writer_) {
    planning_hmi_info.msg_header.stamp = output_time_us;
    planning_hmi_info.msg_header.seq = frame_num_;
    // iflyauto::strcpy_array(planning_hmi_info.msg_header.version, __version_str__);
    planning_hmi_info_writer_(planning_hmi_info_container);
  }

  // Trigger: write fault info where error occured
  if (planning_scheduler_->FaultCode() >= 39000 &&
      planning_scheduler_->FaultCode() <= 39999) {
    ReportFmIfno(planning_scheduler_->FaultCode(), 1, true);
  }

  double planning_cost_time = (IflyTime::Now_us() - start_time) / 1000;
  LOG_WARNING("The cost time of proc() is: [%f] ms\n", planning_cost_time);
}

void PlanningAdapter::UpdateInputListInfo(iflyauto::MsgMeta &msg_meta) {
  int input_list_count = 0;
  // 更新input_list
  msg_meta.input_list[input_list_count].input_type =
      iflyauto::INPUT_HISTORY_TIMESTAMP_SOURCE_TYPE_PREDICTION;
  msg_meta.input_list[input_list_count].seq =
      local_view_ptr_->prediction_result.msg_header.seq;
  input_list_count += 1;

  msg_meta.input_list[input_list_count].input_type =
      iflyauto::INPUT_HISTORY_TIMESTAMP_SOURCE_TYPE_STATIC_FUSION;
  msg_meta.input_list[input_list_count].seq =
      local_view_ptr_->road_info.msg_header.seq;
  input_list_count += 1;

  msg_meta.input_list[input_list_count].input_type =
      iflyauto::INPUT_HISTORY_TIMESTAMP_SOURCE_TYPE_LOCALIZATION;
  msg_meta.input_list[input_list_count].seq =
      local_view_ptr_->localization.msg_header.seq;
  input_list_count += 1;

  msg_meta.input_list[input_list_count].input_type =
      iflyauto::INPUT_HISTORY_TIMESTAMP_SOURCE_TYPE_OBSTACLE_FUSION;
  msg_meta.input_list[input_list_count].seq =
      local_view_ptr_->fusion_objects_info.msg_header.seq;
  input_list_count += 1;

  msg_meta.input_list[input_list_count].input_type =
      iflyauto::INPUT_HISTORY_TIMESTAMP_SOURCE_TYPE_VIHECLE_SERVICES;
  msg_meta.input_list[input_list_count].seq =
      local_view_ptr_->vehicle_service_output_info.msg_header.seq;
  input_list_count += 1;

  msg_meta.input_list[input_list_count].input_type =
      iflyauto::INPUT_HISTORY_TIMESTAMP_SOURCE_TYPE_CONTROL;
  msg_meta.input_list[input_list_count].seq =
      local_view_ptr_->control_output.msg_header.seq;
  input_list_count += 1;

  msg_meta.input_list[input_list_count].input_type =
      iflyauto::INPUT_HISTORY_TIMESTAMP_SOURCE_TYPE_HMI_SERVICE_MCU_INNER;
  msg_meta.input_list[input_list_count].seq =
      local_view_ptr_->hmi_inner_info.msg_header.seq;
  input_list_count += 1;

  msg_meta.input_list[input_list_count].input_type =
      iflyauto::INPUT_HISTORY_TIMESTAMP_SOURCE_TYPE_STATE_MACHINE;
  msg_meta.input_list[input_list_count].seq =
      local_view_ptr_->function_state_machine_info.msg_header.seq;
  input_list_count += 1;

  msg_meta.input_list[input_list_count].input_type =
      iflyauto::INPUT_HISTORY_TIMESTAMP_SOURCE_TYPE_USS_WAVE;
  msg_meta.input_list[input_list_count].seq =
      local_view_ptr_->uss_wave_info.msg_header.seq;
  input_list_count += 1;

  msg_meta.input_list[input_list_count].input_type =
      iflyauto::INPUT_HISTORY_TIMESTAMP_SOURCE_TYPE_USS_PERCEPTION;
  msg_meta.input_list[input_list_count].seq =
      local_view_ptr_->uss_percept_info.msg_header.seq;
  input_list_count += 1;

  msg_meta.input_list[input_list_count].input_type =
      iflyauto::INPUT_HISTORY_TIMESTAMP_SOURCE_TYPE_MAP;
  msg_meta.input_list[input_list_count].seq =
      local_view_ptr_->static_map_info.header().seq();
  input_list_count += 1;

  msg_meta.input_list[input_list_count].input_type =
      iflyauto::INPUT_HISTORY_TIMESTAMP_SOURCE_TYPE_EHR;
  msg_meta.input_list[input_list_count].seq =
      local_view_ptr_->sd_map_info.header().seq();
  input_list_count += 1;

  msg_meta.input_list_size = input_list_count;
}

}  // namespace planning
