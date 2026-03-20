#pragma once

#include "base_convert.h"
//#include "c/sensor_sinpro_radar.h"
#include "legacy/4d_radar_for_data_collection/sensor_sinpro_radar.h"
using namespace iflyauto;

template <typename T2>//using
void convert(iflyauto::Sinpro_Radar_Header &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  for (size_t i0 = 0; i0 < ros_v.sync_word.size(); i0++) {
    convert(static_cast<uint16>(struct_v.sync_word[i0]), ros_v.sync_word[i0], type);
  }
  convert(static_cast<uint16>(struct_v.version), ros_v.version, type);
  convert(static_cast<uint8>(struct_v.data_type), ros_v.data_type, type);
  convert(static_cast<uint64>(struct_v.frame_cnt), ros_v.frame_cnt, type);
  convert(static_cast<uint64>(struct_v.frame_ts_ns), ros_v.frame_ts_ns, type);
  convert(static_cast<uint64>(struct_v.publish_ts_ns), ros_v.publish_ts_ns, type);
}

template <typename T2>//using
void convert(iflyauto::Vehicle_Input_Info &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(static_cast<uint64>(struct_v.vehicle_info_ts_ns), ros_v.vehicle_info_ts_ns, type);
  convert(static_cast<float32>(struct_v.vehicle_speed_kmph), ros_v.vehicle_speed_kmph, type);
  convert(static_cast<float32>(struct_v.yaw_rate_degps), ros_v.yaw_rate_degps, type);
  convert(static_cast<uint8>(struct_v.gear_pos), ros_v.gear_pos, type);
  convert(static_cast<float32>(struct_v.steer_whl_ang_deg), ros_v.steer_whl_ang_deg, type);
  for (size_t i0 = 0; i0 < ros_v.whl_spd_pulse.size(); i0++) {
    convert(static_cast<uint8>(struct_v.whl_spd_pulse[i0]), ros_v.whl_spd_pulse[i0], type);
  }
  for (size_t i1 = 0; i1 < ros_v.whl_spd_dir.size(); i1++) {
    convert(static_cast<uint8>(struct_v.whl_spd_dir[i1]), ros_v.whl_spd_dir[i1], type);
  }
  convert(static_cast<uint8>(struct_v.wiper_sts), ros_v.wiper_sts, type);
  convert(static_cast<float32>(struct_v.vehicle_acceleration_g), ros_v.vehicle_acceleration_g, type);
  convert(static_cast<uint64>(struct_v.vehicle_speed_ts_ns), ros_v.vehicle_speed_ts_ns, type);
  convert(static_cast<uint64>(struct_v.yaw_rate_ts_ns), ros_v.yaw_rate_ts_ns, type);
  convert(static_cast<uint64>(struct_v.exposure_mid_ts_ns), ros_v.exposure_mid_ts_ns, type);
  convert(static_cast<float32>(struct_v.vehicle_speed_raw_kmph), ros_v.vehicle_speed_raw_kmph, type);
  convert(static_cast<float32>(struct_v.yaw_rate_raw_degps), ros_v.yaw_rate_raw_degps, type);
  convert(static_cast<float32>(struct_v.vehicle_acceleration_raw_g), ros_v.vehicle_acceleration_raw_g, type);
}

template <typename T2>//using
void convert(iflyauto::Vehicle_Suspension_Info &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  for (size_t i0 = 0; i0 < ros_v.susp_level_height_mm.size(); i0++) {
    convert(static_cast<int32>(struct_v.susp_level_height_mm[i0]), ros_v.susp_level_height_mm[i0], type);
  }
  for (size_t i1 = 0; i1 < ros_v.susp_valve_state.size(); i1++) {
    convert(static_cast<uint8>(struct_v.susp_valve_state[i1]), ros_v.susp_valve_state[i1], type);
  }
  convert(static_cast<uint8>(struct_v.susp_cur_level), ros_v.susp_cur_level, type);
  convert(static_cast<int32>(struct_v.susp_comp_z), ros_v.susp_comp_z, type);
}

template <typename T2>//using
void convert(iflyauto::Vehicle_Type_Info &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(static_cast<uint16>(struct_v.brand_info), ros_v.brand_info, type);
  convert(static_cast<uint16>(struct_v.vehicle_length_mm), ros_v.vehicle_length_mm, type);
  convert(static_cast<uint16>(struct_v.vehicle_width_mm), ros_v.vehicle_width_mm, type);
  convert(static_cast<uint16>(struct_v.wheel_base_mm), ros_v.wheel_base_mm, type);
  convert(static_cast<int16>(struct_v.radar_rear_x_mm), ros_v.radar_rear_x_mm, type);
  convert(static_cast<int16>(struct_v.radar_rear_y_mm), ros_v.radar_rear_y_mm, type);
  convert(static_cast<int16>(struct_v.radar_ground_z_mm), ros_v.radar_ground_z_mm, type);
  convert(static_cast<int16>(struct_v.head_front_x_mm), ros_v.head_front_x_mm, type);
  convert(static_cast<uint8>(struct_v.mount_direction), ros_v.mount_direction, type);
}

template <typename T2>//using
void convert(iflyauto::Sinpro_Radar_Cali_Info &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(static_cast<uint8>(struct_v.cali_state), ros_v.cali_state, type);
  convert(static_cast<float32>(struct_v.yaw_cali_deg), ros_v.yaw_cali_deg, type);
  convert(static_cast<float32>(struct_v.pitch_cali_deg), ros_v.pitch_cali_deg, type);
  convert(static_cast<float32>(struct_v.roll_cali_deg), ros_v.roll_cali_deg, type);
}

template <typename T2>//using
void convert(iflyauto::Pointcloud_Unit &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(static_cast<uint16>(struct_v.matched_obj_id), ros_v.matched_obj_id, type);
  convert(static_cast<float32>(struct_v.rng_m), ros_v.rng_m, type);
  convert(static_cast<float32>(struct_v.azi_deg), ros_v.azi_deg, type);
  convert(static_cast<float32>(struct_v.ele_deg), ros_v.ele_deg, type);
  convert(static_cast<uint8>(struct_v.motion_status), ros_v.motion_status, type);
  convert(static_cast<float32>(struct_v.dpl_raw_mps), ros_v.dpl_raw_mps, type);
  convert(static_cast<float32>(struct_v.dpl_unambi_mps), ros_v.dpl_unambi_mps, type);
  convert(static_cast<uint8>(struct_v.bandwidth_mode), ros_v.bandwidth_mode, type);
  convert(static_cast<float32>(struct_v.SNR_dB), ros_v.SNR_dB, type);
  convert(static_cast<float32>(struct_v.RCS_dBsm), ros_v.RCS_dBsm, type);
  convert(static_cast<uint8>(struct_v.IsPeak), ros_v.IsPeak, type);
  convert(static_cast<uint8>(struct_v.IsGhost), ros_v.IsGhost, type);
  convert(static_cast<uint8>(struct_v.azimuth_order), ros_v.azimuth_order, type);
}

template <typename T2>//using
void convert(iflyauto::Im_Info &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(static_cast<uint8>(struct_v.im_flag_sr), ros_v.im_flag_sr, type);
  convert(static_cast<uint8>(struct_v.im_flag_lr), ros_v.im_flag_lr, type);
  convert(static_cast<uint16>(struct_v.im_chirp_num_sr), ros_v.im_chirp_num_sr, type);
  convert(static_cast<uint32>(struct_v.im_sample_num_sr), ros_v.im_sample_num_sr, type);
  convert(static_cast<uint32>(struct_v.im_pow_level_sr), ros_v.im_pow_level_sr, type);
  convert(static_cast<float32>(struct_v.im_snr_loss_sr), ros_v.im_snr_loss_sr, type);
  convert(static_cast<uint16>(struct_v.im_chirp_num_lr), ros_v.im_chirp_num_lr, type);
  convert(static_cast<uint32>(struct_v.im_sample_num_lr), ros_v.im_sample_num_lr, type);
  convert(static_cast<uint32>(struct_v.im_pow_level_lr), ros_v.im_pow_level_lr, type);
  convert(static_cast<float32>(struct_v.im_snr_loss_lr), ros_v.im_snr_loss_lr, type);
}

template <typename T2>//using
void convert(iflyauto::Ch_Pow &struct_v, T2 &ros_v, ConvertTypeInfo type) {
  for (size_t i0 = 0; i0 < ros_v.Ch_Powx.size(); i0++) {
    convert(static_cast<uint8>(struct_v.ch_pow_tx[i0]), ros_v.ch_pow_tx[i0], type);
  }
  for (size_t i1 = 0; i1 < ros_v.ch_pow_rx.size(); i1++) {
    convert(static_cast<uint8>(struct_v.ch_pow_rx[i1]), ros_v.ch_pow_rx[i1], type);
  }
}

template <typename T2>//using
void convert(iflyauto::Sinpro_Radar_Pointcloud &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(static_cast<uint16>(struct_v.pointcloud_num), ros_v.pointcloud_num, type);
  convert(static_cast<float32>(struct_v.short_dpl_unambi_scale_mps), ros_v.short_dpl_unambi_scale_mps, type);
  convert(static_cast<float32>(struct_v.long_dpl_unambi_scale_mps), ros_v.long_dpl_unambi_scale_mps, type);
  convert(struct_v.im_info, ros_v.im_info, type);
  convert(struct_v.ch_pow, ros_v.ch_pow, type);
  for (size_t i0 = 0; i0 < ros_v.pointcloud_unit.size(); i0++) {
    convert(struct_v.pointcloud_unit[i0], ros_v.pointcloud_unit[i0], type);
  }
}
/*
template <typename T2>
void convert(iflyauto::Target_Info &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(static_cast<uint16>(struct_v.id), ros_v.id, type);
  convert(static_cast<uint16>(struct_v.life_counter), ros_v.life_counter, type);
  convert(static_cast<uint16>(struct_v.observed_counter), ros_v.observed_counter, type);
  convert(static_cast<uint8>(struct_v.observed_status), ros_v.observed_status, type);
  convert(static_cast<float32>(struct_v.pos_x_m), ros_v.pos_x_m, type);
  convert(static_cast<float32>(struct_v.pos_y_m), ros_v.pos_y_m, type);
  convert(static_cast<float32>(struct_v.pos_z_m), ros_v.pos_z_m, type);
  convert(static_cast<float32>(struct_v.vel_x_mps), ros_v.vel_x_mps, type);
  convert(static_cast<float32>(struct_v.vel_y_mps), ros_v.vel_y_mps, type);
  convert(static_cast<float32>(struct_v.acc_x_mps2), ros_v.acc_x_mps2, type);
  convert(static_cast<float32>(struct_v.acc_y_mps2), ros_v.acc_y_mps2, type);
  convert(static_cast<float32>(struct_v.std_pos_x_m), ros_v.std_pos_x_m, type);
  convert(static_cast<float32>(struct_v.std_pos_y_m), ros_v.std_pos_y_m, type);
  convert(static_cast<float32>(struct_v.std_vel_x_mps), ros_v.std_vel_x_mps, type);
  convert(static_cast<float32>(struct_v.std_vel_y_mps), ros_v.std_vel_y_mps, type);
  convert(static_cast<float32>(struct_v.std_acc_x_mps2), ros_v.std_acc_x_mps2, type);
  convert(static_cast<float32>(struct_v.std_acc_y_mps2), ros_v.std_acc_y_mps2, type);
  convert(static_cast<float32>(struct_v.box_length_m), ros_v.box_length_m, type);
  convert(static_cast<float32>(struct_v.box_width_m), ros_v.box_width_m, type);
  convert(static_cast<float32>(struct_v.box_height_m), ros_v.box_height_m, type);
  convert(static_cast<float32>(struct_v.heading_deg), ros_v.heading_deg, type);
  convert(static_cast<float32>(struct_v.yawrate_degps), ros_v.yawrate_degps, type);
  convert(static_cast<uint8>(struct_v.existence_prob), ros_v.existence_prob, type);
  convert(static_cast<uint8>(struct_v.obstacle_prob), ros_v.obstacle_prob, type);
  convert(static_cast<uint8>(struct_v.moving_status), ros_v.moving_status, type);
  convert(static_cast<uint8>(struct_v.object_classification), ros_v.object_classification, type);
  convert(static_cast<uint8>(struct_v.classification_prob), ros_v.classification_prob, type);
  convert(static_cast<int8>(struct_v.object_rcs), ros_v.object_rcs, type);
  convert(static_cast<uint16>(struct_v.pcld_obs_num), ros_v.pcld_obs_num, type);
  convert(static_cast<uint8>(struct_v.ghost_estm), ros_v.ghost_estm, type);
  for (size_t i0 = 0; i0 < ros_v.pcld_obs_edge_pos_x_m.size(); i0++) {
    convert(static_cast<float32>(struct_v.pcld_obs_edge_pos_x_m[i0]), ros_v.pcld_obs_edge_pos_x_m[i0], type);
  }
  for (size_t i1 = 0; i1 < ros_v.pcld_obs_edge_pos_y_m.size(); i1++) {
    convert(static_cast<float32>(struct_v.pcld_obs_edge_pos_y_m[i1]), ros_v.pcld_obs_edge_pos_y_m[i1], type);
  }
}

template <typename T2>
void convert(iflyauto::Sinpro_Radar_Target_Opt &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(static_cast<uint16>(struct_v.target_num), ros_v.target_num, type);
  // flex array
  if (type == ConvertTypeInfo::TO_ROS) {
    if (struct_v.target_num >= 0 && struct_v.target_num <= TARGET_OUTPUT_BUFFER_LENGTH) {
      ros_v.target.resize(struct_v.target_num);
    } else {
      std::cout << "convert/sinpro_radar_data_c.h:" << __LINE__
                << " [convert][TO_ROS] target_num=" << struct_v.target_num
                << " not in range TARGET_OUTPUT_BUFFER_LENGTH=" << TARGET_OUTPUT_BUFFER_LENGTH
                << std::endl;
      ros_v.target_num = TARGET_OUTPUT_BUFFER_LENGTH;
      ros_v.target.resize(TARGET_OUTPUT_BUFFER_LENGTH);
    }
    for (size_t i0 = 0; i0 < ros_v.target.size(); i0++) {
      convert(struct_v.target[i0], ros_v.target[i0], type);
    }
  }
  if (type == ConvertTypeInfo::TO_STRUCT) {
    if (ros_v.target_num > TARGET_OUTPUT_BUFFER_LENGTH || ros_v.target_num < 0 || ros_v.target.size() > TARGET_OUTPUT_BUFFER_LENGTH) {
      std::cout << "convert/sinpro_radar_data_c.h:" << __LINE__
                << "[convert][TO_STRUCT] target_num=" << ros_v.target_num
                << " ros_v.target.size()=" << ros_v.target.size()
                << " not in range TARGET_OUTPUT_BUFFER_LENGTH=" << TARGET_OUTPUT_BUFFER_LENGTH
                << std::endl;
    }
    if (ros_v.target.size() > TARGET_OUTPUT_BUFFER_LENGTH) {
      for (size_t i0 = 0; i0 < TARGET_OUTPUT_BUFFER_LENGTH; i0++) {
        convert(struct_v.target[i0], ros_v.target[i0], type);
      }
    } else {
      for (size_t i0 = 0; i0 < ros_v.target.size(); i0++) {
        convert(struct_v.target[i0], ros_v.target[i0], type);
      }
    }
  }
  //
}

template <typename T2>
void convert(iflyauto::Obstacle_Info &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(static_cast<uint16>(struct_v.id), ros_v.id, type);
  convert(static_cast<uint16>(struct_v.life_counter), ros_v.life_counter, type);
  convert(static_cast<uint8>(struct_v.existence_prob), ros_v.existence_prob, type);
  for (size_t i0 = 0; i0 < ros_v.box_edge_x_m.size(); i0++) {
    convert(static_cast<uint16>(struct_v.box_edge_x_m[i0]), ros_v.box_edge_x_m[i0], type);
  }
  for (size_t i1 = 0; i1 < ros_v.box_edge_y_m.size(); i1++) {
    convert(static_cast<uint16>(struct_v.box_edge_y_m[i1]), ros_v.box_edge_y_m[i1], type);
  }
  convert(static_cast<uint8>(struct_v.total_edge_num), ros_v.total_edge_num, type);
  for (size_t i2 = 0; i2 < ros_v.vertex_x_m.size(); i2++) {
    convert(static_cast<float32>(struct_v.vertex_x_m[i2]), ros_v.vertex_x_m[i2], type);
  }
  for (size_t i3 = 0; i3 < ros_v.vertex_y_m.size(); i3++) {
    convert(static_cast<float32>(struct_v.vertex_y_m[i3]), ros_v.vertex_y_m[i3], type);
  }
  convert(static_cast<float32>(struct_v.pos_z_m), ros_v.pos_z_m, type);
  convert(static_cast<float32>(struct_v.box_height_m), ros_v.box_height_m, type);
  convert(static_cast<uint8>(struct_v.is_passable), ros_v.is_passable, type);
  convert(static_cast<uint16>(struct_v.pcld_obs_num), ros_v.pcld_obs_num, type);
  convert(static_cast<uint8>(struct_v.obstacle_classification), ros_v.obstacle_classification, type);
  convert(static_cast<uint8>(struct_v.moving_status), ros_v.moving_status, type);
  convert(static_cast<float32>(struct_v.radial_absolute_velocity), ros_v.radial_absolute_velocity, type);
}

template <typename T2>
void convert(iflyauto::Sinpro_Radar_Obstacle_Opt &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(static_cast<uint16>(struct_v.obstacle_num), ros_v.obstacle_num, type);
  // flex array
  if (type == ConvertTypeInfo::TO_ROS) {
    if (struct_v.obstacle_num >= 0 && struct_v.obstacle_num <= OBSTACLE_OUTPUT_BUFFER_LENGTH) {
      ros_v.obstacle.resize(struct_v.obstacle_num);
    } else {
      std::cout << "convert/sinpro_radar_data_c.h:" << __LINE__
                << " [convert][TO_ROS] obstacle_num=" << struct_v.obstacle_num
                << " not in range OBSTACLE_OUTPUT_BUFFER_LENGTH=" << OBSTACLE_OUTPUT_BUFFER_LENGTH
                << std::endl;
      ros_v.obstacle_num = OBSTACLE_OUTPUT_BUFFER_LENGTH;
      ros_v.obstacle.resize(OBSTACLE_OUTPUT_BUFFER_LENGTH);
    }
    for (size_t i0 = 0; i0 < ros_v.obstacle.size(); i0++) {
      convert(struct_v.obstacle[i0], ros_v.obstacle[i0], type);
    }
  }
  if (type == ConvertTypeInfo::TO_STRUCT) {
    if (ros_v.obstacle_num > OBSTACLE_OUTPUT_BUFFER_LENGTH || ros_v.obstacle_num < 0 || ros_v.obstacle.size() > OBSTACLE_OUTPUT_BUFFER_LENGTH) {
      std::cout << "convert/sinpro_radar_data_c.h:" << __LINE__
                << "[convert][TO_STRUCT] obstacle_num=" << ros_v.obstacle_num
                << " ros_v.obstacle.size()=" << ros_v.obstacle.size()
                << " not in range OBSTACLE_OUTPUT_BUFFER_LENGTH=" << OBSTACLE_OUTPUT_BUFFER_LENGTH
                << std::endl;
    }
    if (ros_v.obstacle.size() > OBSTACLE_OUTPUT_BUFFER_LENGTH) {
      for (size_t i0 = 0; i0 < OBSTACLE_OUTPUT_BUFFER_LENGTH; i0++) {
        convert(struct_v.obstacle[i0], ros_v.obstacle[i0], type);
      }
    } else {
      for (size_t i0 = 0; i0 < ros_v.obstacle.size(); i0++) {
        convert(struct_v.obstacle[i0], ros_v.obstacle[i0], type);
      }
    }
  }
  //
}

template <typename T2>
void convert(iflyauto::Tag_Guardrail_Info &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(static_cast<uint8>(struct_v.valid), ros_v.valid, type);
  convert(static_cast<float32>(struct_v.coef_3), ros_v.coef_3, type);
  convert(static_cast<float32>(struct_v.coef_2), ros_v.coef_2, type);
  convert(static_cast<float32>(struct_v.coef_1), ros_v.coef_1, type);
  convert(static_cast<float32>(struct_v.coef_0), ros_v.coef_0, type);
  convert(static_cast<float32>(struct_v.start_x_m), ros_v.start_x_m, type);
  convert(static_cast<float32>(struct_v.end_x_m), ros_v.end_x_m, type);
}

template <typename T2>
void convert(iflyauto::Sinpro_Radar_Guardrail &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.left, ros_v.left, type);
  convert(struct_v.right, ros_v.right, type);
}

template <typename T2>
void convert(iflyauto::Sinpro_Radar_Active_Error &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  for (size_t i0 = 0; i0 < ros_v.err.size(); i0++) {
    convert(static_cast<uint16>(struct_v.err[i0]), ros_v.err[i0], type);
  }
}

template <typename T2>
void convert(iflyauto::Sinpro_Summary_Err_Reason &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  for (size_t i0 = 0; i0 < ros_v.val.size(); i0++) {
    convert(static_cast<uint16>(struct_v.val[i0]), ros_v.val[i0], type);
  }
}

template <typename T2>
void convert(iflyauto::Sinpro_Radar_Summary_Error &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(static_cast<uint16>(struct_v.blindness), ros_v.blindness, type);
  convert(static_cast<uint16>(struct_v.internal_fault_0), ros_v.internal_fault_0, type);
  convert(static_cast<uint16>(struct_v.alignment_incomplete), ros_v.alignment_incomplete, type);
  convert(static_cast<uint16>(struct_v.alignment_out_of_range), ros_v.alignment_out_of_range, type);
  convert(static_cast<uint16>(struct_v.voltage_above_threshold), ros_v.voltage_above_threshold, type);
  convert(static_cast<uint16>(struct_v.voltage_below_threshold), ros_v.voltage_below_threshold, type);
  convert(static_cast<uint16>(struct_v.temp_high_fault), ros_v.temp_high_fault, type);
  convert(static_cast<uint16>(struct_v.loss_comm_with_adc), ros_v.loss_comm_with_adc, type);
  convert(static_cast<uint16>(struct_v.invalid_data_from_adc), ros_v.invalid_data_from_adc, type);
  convert(static_cast<uint16>(struct_v.loss_comm_with_zone), ros_v.loss_comm_with_zone, type);
  convert(static_cast<uint16>(struct_v.invalid_data_from_zone), ros_v.invalid_data_from_zone, type);
  convert(static_cast<uint16>(struct_v.internal_fault_1), ros_v.internal_fault_1, type);
  convert(static_cast<uint16>(struct_v.internal_fault_2), ros_v.internal_fault_2, type);
  convert(static_cast<uint16>(struct_v.radar_data_invalid), ros_v.radar_data_invalid, type);
  convert(static_cast<uint16>(struct_v.radar_version_mismatch), ros_v.radar_version_mismatch, type);
  for (size_t i0 = 0; i0 < ros_v.rsv.size(); i0++) {
    convert(static_cast<uint16>(struct_v.rsv[i0]), ros_v.rsv[i0], type);
  }
}

template <typename T2>
void convert(iflyauto::Sinpro_Sw_Version &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  for (size_t i0 = 0; i0 < ros_v.radar_sw_version.size(); i0++) {
    convert(static_cast<char>(struct_v.radar_sw_version[i0]), ros_v.radar_sw_version[i0], type);
  }
  for (size_t i1 = 0; i1 < ros_v.sdk_sw_version.size(); i1++) {
    convert(static_cast<char>(struct_v.sdk_sw_version[i1]), ros_v.sdk_sw_version[i1], type);
  }
}
*/
template <typename T2>//using
void convert(iflyauto::Sinpro_Radar_Rd_Map &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  for (size_t i0 = 0; i0 < ros_v.rdmap_short.size(); i0++) {
    convert(static_cast<uint8>(struct_v.rdmap_short[i0]), ros_v.rdmap_short[i0], type);
  }
  for (size_t i1 = 0; i1 < ros_v.rdmap_long.size(); i1++) {
    convert(static_cast<uint8>(struct_v.rdmap_long[i1]), ros_v.rdmap_long[i1], type);
  }
}
/*
template <typename T2>
void convert(iflyauto::Sinpro_Radar_Blindness_Data &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(static_cast<uint16>(struct_v.blindness_prob), ros_v.blindness_prob, type);
}

template <typename T2>
void convert(iflyauto::Sinpro_Radar_Dump_Data &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(static_cast<uint16>(struct_v.is_useful), ros_v.is_useful, type);
  for (size_t i0 = 0; i0 < ros_v.dump_reg.size(); i0++) {
    convert(static_cast<uint8>(struct_v.dump_reg[i0]), ros_v.dump_reg[i0], type);
  }
}

template <typename T2>
void convert(iflyauto::Sinpro_Radar_Sync_Data &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(static_cast<uint8>(struct_v.ptp_sync_status), ros_v.ptp_sync_status, type);
  convert(static_cast<uint8>(struct_v.frame_sync_status), ros_v.frame_sync_status, type);
  convert(static_cast<uint8>(struct_v.chirp_sync_mode), ros_v.chirp_sync_mode, type);
  convert(static_cast<uint32>(struct_v.target_time_us), ros_v.target_time_us, type);
  convert(static_cast<uint32>(struct_v.short_chirp_center_us), ros_v.short_chirp_center_us, type);
  convert(static_cast<uint32>(struct_v.long_chirp_center_us), ros_v.long_chirp_center_us, type);
  convert(static_cast<uint32>(struct_v.sync_step_us), ros_v.sync_step_us, type);
  convert(static_cast<uint32>(struct_v.boot_to_sync_time_us), ros_v.boot_to_sync_time_us, type);
  convert(static_cast<uint32>(struct_v.sync_cost_time_us), ros_v.sync_cost_time_us, type);
  convert(static_cast<uint64>(struct_v.sync_start_time_us), ros_v.sync_start_time_us, type);
}

template <typename T2>
void convert(iflyauto::Sinpro_Radar_Log &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(static_cast<uint16>(struct_v.is_useful), ros_v.is_useful, type);
  for (size_t i0 = 0; i0 < ros_v.data.size(); i0++) {
    convert(static_cast<char>(struct_v.data[i0]), ros_v.data[i0], type);
  }
}
*/
template <typename T2>//using
void convert(iflyauto::Sinpro_Radar_Append_Info &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(static_cast<uint64>(struct_v.radar_local_time_us), ros_v.radar_local_time_us, type);
  convert(static_cast<uint8>(struct_v.frame_type), ros_v.frame_type, type);
  convert(static_cast<uint16>(struct_v.useful_len), ros_v.useful_len, type);
  for (size_t i0 = 0; i0 < ros_v.rsv.size(); i0++) {
    convert(static_cast<uint8>(struct_v.rsv[i0]), ros_v.rsv[i0], type);
  }
}
/*
template <typename T2>
void convert(iflyauto::Sinpro_Drop_Detect &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(static_cast<uint16>(struct_v.rsv_data), ros_v.rsv_data, type);
}

template <typename T2>
void convert(iflyauto::Sinpro_Hw_Info &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  for (size_t i0 = 0; i0 < ros_v.formal_hw_ver.size(); i0++) {
    convert(static_cast<uint8>(struct_v.formal_hw_ver[i0]), ros_v.formal_hw_ver[i0], type);
  }
  for (size_t i1 = 0; i1 < ros_v.local_hw_ver.size(); i1++) {
    convert(static_cast<uint8>(struct_v.local_hw_ver[i1]), ros_v.local_hw_ver[i1], type);
  }
  for (size_t i2 = 0; i2 < ros_v.radar_hw_pn.size(); i2++) {
    convert(static_cast<uint8>(struct_v.radar_hw_pn[i2]), ros_v.radar_hw_pn[i2], type);
  }
}

template <typename T2>
void convert(iflyauto::Sinpro_Vol_Info &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(static_cast<uint16>(struct_v.vol_input), ros_v.vol_input, type);
  convert(static_cast<uint16>(struct_v.vol_SBC_4V1), ros_v.vol_SBC_4V1, type);
  convert(static_cast<uint16>(struct_v.vol_SBC_3V3), ros_v.vol_SBC_3V3, type);
  convert(static_cast<uint16>(struct_v.vol_SBC_1V8), ros_v.vol_SBC_1V8, type);
  for (size_t i0 = 0; i0 < ros_v.MMIC_2V3.size(); i0++) {
    convert(static_cast<uint16>(struct_v.MMIC_2V3[i0]), ros_v.MMIC_2V3[i0], type);
  }
}

template <typename T2>
void convert(iflyauto::Sinpro_Temp_Info &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(static_cast<uint8>(struct_v.temp_SOC), ros_v.temp_SOC, type);
  convert(static_cast<uint8>(struct_v.temp_SBC), ros_v.temp_SBC, type);
  convert(static_cast<uint8>(struct_v.temp_PMIC), ros_v.temp_PMIC, type);
  convert(static_cast<uint8>(struct_v.temp_MMIC), ros_v.temp_MMIC, type);
}

template <typename T2>
void convert(iflyauto::Sinpro_Resource_Info &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(static_cast<uint8>(struct_v.CPU_load_M_core), ros_v.CPU_load_M_core, type);
  convert(static_cast<uint8>(struct_v.CPU_load_A_core), ros_v.CPU_load_A_core, type);
  convert(static_cast<uint8>(struct_v.stack_used_max), ros_v.stack_used_max, type);
  convert(static_cast<uint8>(struct_v.stack_ID), ros_v.stack_ID, type);
}

template <typename T2>
void convert(iflyauto::Sinpro_Key_Monitor_Info &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(static_cast<uint8>(struct_v.is_useful), ros_v.is_useful, type);
  convert(static_cast<Sinpro_Vol_Info>(struct_v.vol_info), ros_v.vol_info, type);
  convert(static_cast<Sinpro_Temp_Info>(struct_v.temp_info), ros_v.temp_info, type);
  convert(static_cast<Sinpro_Resource_Info>(struct_v.resource_info), ros_v.resource_info, type);
}

template <typename T2>
void convert(iflyauto::Sinpro_Key_Module_Status &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(static_cast<sint64>(struct_v.gPTP_TS_deviation_ns), ros_v.gPTP_TS_deviation_ns, type);
  convert(static_cast<uint8>(struct_v.power_off_act_status), ros_v.power_off_act_status, type);
}

template <typename T2>
void convert(iflyauto::Sinpro_Perc_Debug_Data &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(static_cast<uint16>(struct_v.useful_len), ros_v.useful_len, type);
  for (size_t i0 = 0; i0 < ros_v.data.size(); i0++) {
    convert(static_cast<uint8>(struct_v.data[i0]), ros_v.data[i0], type);
  }
}
*/
template <typename T2>//using
void convert(iflyauto::Sinpro_Invalid_Reason &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  for (size_t i0 = 0; i0 < ros_v.err.size(); i0++) {
    convert(static_cast<uint16>(struct_v.err[i0]), ros_v.err[i0], type);
  }
}
/*
template <typename T2>
void convert(iflyauto::Sinpro_Fls_Cnt_Unit &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(static_cast<uint32>(struct_v.cnt_and_flag), ros_v.cnt_and_flag, type);
}

template <typename T2>
void convert(iflyauto::Sinpro_Fls_Cnt &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  for (size_t i0 = 0; i0 < ros_v.fls_count.size(); i0++) {
    convert(struct_v.fls_count[i0], ros_v.fls_count[i0], type);
  }
}

template <typename T2>
void convert(iflyauto::Sinpro_Misa_Alert_Mem &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  for (size_t i0 = 0; i0 < ros_v.yaw_array.size(); i0++) {
    convert(static_cast<float32>(struct_v.yaw_array[i0]), ros_v.yaw_array[i0], type);
  }
  for (size_t i1 = 0; i1 < ros_v.pitch_array.size(); i1++) {
    convert(static_cast<float32>(struct_v.pitch_array[i1]), ros_v.pitch_array[i1], type);
  }
  convert(static_cast<uint8>(struct_v.yaw_final_judge_flag), ros_v.yaw_final_judge_flag, type);
  convert(static_cast<uint8>(struct_v.pitch_final_judge_flag), ros_v.pitch_final_judge_flag, type);
}
*/
template <typename T2>
void convert(iflyauto::Sinpro_Radar_Pointcloud_Data &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  /*convert(struct_v.radar_header, ros_v.radar_header, type);
  convert(struct_v.pointcloud, ros_v.pointcloud, type);
  convert(struct_v.vehicle_input_info, ros_v.vehicle_input_info, type);
  convert(struct_v.append_info, ros_v.append_info, type);
  convert(struct_v.suspension_info, ros_v.suspension_info, type);
  convert(struct_v.vehicle_type_info, ros_v.vehicle_type_info, type);
  convert(struct_v.radar_cali_info, ros_v.radar_cali_info, type);
  convert(struct_v.invalid_reason, ros_v.invalid_reason, type);
  convert(static_cast<uint8>(struct_v.is_valid), ros_v.is_valid, type);
  convert(static_cast<uint8>(struct_v.rsv), ros_v.rsv, type);*/
  ros_v.payload.resize(sizeof(iflyauto::Sinpro_Radar_Pointcloud_Data));
  memcpy(ros_v.payload.data(), &struct_v, sizeof(iflyauto::Sinpro_Radar_Pointcloud_Data));
}

template <typename T2>
void convert(iflyauto::Sinpro_Radar_Perception_Data &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  /*convert(struct_v.radar_header, ros_v.radar_header, type);
  convert(struct_v.target_opt, ros_v.target_opt, type);
  convert(struct_v.obstacle_opt, ros_v.obstacle_opt, type);
  convert(struct_v.guardrail, ros_v.guardrail, type);
  convert(struct_v.vehicle_input_info, ros_v.vehicle_input_info, type);
  convert(struct_v.perc_debug_data, ros_v.perc_debug_data, type);
  convert(struct_v.invalid_reason, ros_v.invalid_reason, type);
  convert(struct_v.is_valid, ros_v.is_valid, type);*/
  ros_v.payload.resize(sizeof(iflyauto::Sinpro_Radar_Perception_Data));
  memcpy(ros_v.payload.data(), &struct_v, sizeof(iflyauto::Sinpro_Radar_Perception_Data));
}

template <typename T2>
void convert(iflyauto::Sinpro_Radar_System_Data &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  /*convert(struct_v.radar_header, ros_v.radar_header, type);
  convert(struct_v.state, ros_v.state, type);
  convert(struct_v.active_error, ros_v.active_error, type);
  convert(struct_v.vehicle_type_info, ros_v.vehicle_type_info, type);
  convert(struct_v.radar_cali_info, ros_v.radar_cali_info, type);
  convert(struct_v.sw_version, ros_v.sw_version, type);
  convert(struct_v.dump_data, ros_v.dump_data, type);
  convert(struct_v.blindness_opt, ros_v.blindness_opt, type);
  convert(struct_v.sync_data, ros_v.sync_data, type);
  convert(struct_v.active_error_summary, ros_v.active_error_summary, type);
  convert(struct_v.radar_log, ros_v.radar_log, type);
  convert(struct_v.drop_detect_data, ros_v.drop_detect_data, type);
  convert(struct_v.hw_version, ros_v.hw_version, type);
  convert(struct_v.key_monitor_info, ros_v.key_monitor_info, type);
  convert(struct_v.key_module_status, ros_v.key_module_status, type);
  convert(struct_v.fls_cnt, ros_v.fls_cnt, type);
  convert(struct_v.misa_alert_mem, ros_v.misa_alert_mem, type);
  convert(struct_v.err_reason, ros_v.err_reason, type);
  convert(struct_v.invalid_reason, ros_v.invalid_reason, type);
  convert(struct_v.is_valid, ros_v.is_valid, type);*/
  ros_v.payload.resize(sizeof(iflyauto::Sinpro_Radar_System_Data));
  memcpy(ros_v.payload.data(), &struct_v, sizeof(iflyauto::Sinpro_Radar_System_Data));
}

template <typename T2>
void convert(iflyauto::Sinpro_Radar_Debug_Data &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  /*convert(struct_v.radar_header, ros_v.radar_header, type);
  convert(struct_v.rd_map, ros_v.rd_map, type);
  convert(struct_v.invalid_reason, ros_v.invalid_reason, type);
  convert(static_cast<uint8>(struct_v.is_valid), ros_v.is_valid, type);*/
  ros_v.payload.resize(sizeof(iflyauto::Sinpro_Radar_Debug_Data));
  memcpy(ros_v.payload.data(), &struct_v, sizeof(iflyauto::Sinpro_Radar_Debug_Data));
}

