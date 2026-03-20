#pragma once

#include "base_convert.h"
#include "c/uss_wave_debug_info_c.h"
#include "struct_convert/common_c.h"

using namespace iflyauto;

template <typename T2>
void convert(iflyauto::EchoInfo &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.u8s_uss_echo_type, ros_v.u8s_uss_echo_type, type);
  convert(struct_v.u16s_uss_echo_timestamp, ros_v.u16s_uss_echo_timestamp, type);
}

template <typename T2>
void convert(iflyauto::USSenDataOneChannel &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.u8s_uss_sfl_burst_mode, ros_v.u8s_uss_sfl_burst_mode, type);
  convert(struct_v.u8s_uss_sfl_meas_mode, ros_v.u8s_uss_sfl_meas_mode, type);
  for (size_t i0 = 0; i0 < ros_v.echo_info.size(); i0++) {
	  convert(struct_v.echo_info[i0], ros_v.echo_info[i0], type);
  }
}

template <typename T2>
void convert(iflyauto::SnsCmdEepromData &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.data_valid_flag, ros_v.data_valid_flag, type);
  convert(struct_v.void0, ros_v.void0, type);
  convert(struct_v.osc_trim, ros_v.osc_trim, type);
  convert(struct_v.void1, ros_v.void1, type);
  convert(struct_v.customer_bits, ros_v.customer_bits, type);
  convert(struct_v.void2, ros_v.void2, type);
  convert(struct_v.gain_digital, ros_v.gain_digital, type);
  convert(struct_v.void3, ros_v.void3, type);
  convert(struct_v.gain_analog, ros_v.gain_analog, type);
  convert(struct_v.void4, ros_v.void4, type);
  convert(struct_v.drive_current, ros_v.drive_current, type);
  convert(struct_v.void5, ros_v.void5, type);
  convert(struct_v.drive_frequence, ros_v.drive_frequence, type);
}

template <typename T2>
void convert(iflyauto::RcDataTypeDebug &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  for (size_t i0 = 0; i0 < ros_v.rcv_data_value.size(); i0++) {
	  convert(struct_v.rcv_data_value[i0], ros_v.rcv_data_value[i0], type);
  }
}

template <typename T2>
void convert(iflyauto::SnsOpInfoType &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  for (size_t i0 = 0; i0 < ros_v.sns_data.size(); i0++) {
	  convert(struct_v.sns_data[i0], ros_v.sns_data[i0], type);
  }
  convert(struct_v.cur_bit_index, ros_v.cur_bit_index, type);
  convert(struct_v.sns_op_cmd_tx_bit_cnt, ros_v.sns_op_cmd_tx_bit_cnt, type);
  convert(struct_v.data_bit_index, ros_v.data_bit_index, type);
  convert(struct_v.data_cnt, ros_v.data_cnt, type);
  convert(struct_v.data_addr, ros_v.data_addr, type);
  convert(struct_v.data_rx_try_cnt, ros_v.data_rx_try_cnt, type);
  for (size_t i1 = 0; i1 < ros_v.check_sum.size(); i1++) {
	  convert(struct_v.check_sum[i1], ros_v.check_sum[i1], type);
  }
  for (size_t i2 = 0; i2 < ros_v.prev_sns_bkwv_time.size(); i2++) {
	  convert(struct_v.prev_sns_bkwv_time[i2], ros_v.prev_sns_bkwv_time[i2], type);
  }
  for (size_t i3 = 0; i3 < ros_v.rcv_data_completed_flag.size(); i3++) {
	  convert(struct_v.rcv_data_completed_flag[i3], ros_v.rcv_data_completed_flag[i3], type);
  }
  for (size_t i4 = 0; i4 < ros_v.need_rcv_data_bit_length.size(); i4++) {
	  convert(struct_v.need_rcv_data_bit_length[i4], ros_v.need_rcv_data_bit_length[i4], type);
  }
  for (size_t i5 = 0; i5 < ros_v.rcv_data_bit_index.size(); i5++) {
	  convert(struct_v.rcv_data_bit_index[i5], ros_v.rcv_data_bit_index[i5], type);
  }
  for (size_t i6 = 0; i6 < ros_v.cmd_processed_state.size(); i6++) {
	  convert(struct_v.cmd_processed_state[i6], ros_v.cmd_processed_state[i6], type);
  }
  for (size_t i7 = 0; i7 < ros_v.rcv_data_cmd_type.size(); i7++) {
	  convert(struct_v.rcv_data_cmd_type[i7], ros_v.rcv_data_cmd_type[i7], type);
  }
  for (size_t i8 = 0; i8 < ros_v.rcv_data_buf.size(); i8++) {
	  convert(struct_v.rcv_data_buf[i8], ros_v.rcv_data_buf[i8], type);
  }
}

template <typename T2>
void convert(iflyauto::SnsBkWvStartTimeBuf &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  for (size_t i0 = 0; i0 < ros_v.sns_bk_wv_start_time_value.size(); i0++) {
	  convert(struct_v.sns_bk_wv_start_time_value[i0], ros_v.sns_bk_wv_start_time_value[i0], type);
  }
}

template <typename T2>
void convert(iflyauto::SnsBkWvWidthBuf &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  for (size_t i0 = 0; i0 < ros_v.sns_bk_wv_width_value.size(); i0++) {
	  convert(struct_v.sns_bk_wv_width_value[i0], ros_v.sns_bk_wv_width_value[i0], type);
  }
}

template <typename T2>
void convert(iflyauto::SnsBkWvHighBuf &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  for (size_t i0 = 0; i0 < ros_v.sns_bk_wv_high_value.size(); i0++) {
	  convert(struct_v.sns_bk_wv_high_value[i0], ros_v.sns_bk_wv_high_value[i0], type);
  }
}

template <typename T2>
void convert(iflyauto::SnsBkWvObjTypeBuf &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  for (size_t i0 = 0; i0 < ros_v.sns_bk_wv_obj_type_value.size(); i0++) {
	  convert(struct_v.sns_bk_wv_obj_type_value[i0], ros_v.sns_bk_wv_obj_type_value[i0], type);
  }
}

template <typename T2>
void convert(iflyauto::EchoTypeBuf &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  for (size_t i0 = 0; i0 < ros_v.echo_type_value.size(); i0++) {
	  convert(struct_v.echo_type_value[i0], ros_v.echo_type_value[i0], type);
  }
}

template <typename T2>
void convert(iflyauto::SnsDtBufType &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  for (size_t i0 = 0; i0 < ros_v.sns_bk_wv_start_time.size(); i0++) {
	  convert(struct_v.sns_bk_wv_start_time[i0], ros_v.sns_bk_wv_start_time[i0], type);
  }
  for (size_t i1 = 0; i1 < ros_v.sns_wv_number.size(); i1++) {
	  convert(struct_v.sns_wv_number[i1], ros_v.sns_wv_number[i1], type);
  }
  for (size_t i2 = 0; i2 < ros_v.sns_bk_wv_width.size(); i2++) {
	  convert(struct_v.sns_bk_wv_width[i2], ros_v.sns_bk_wv_width[i2], type);
  }
  for (size_t i3 = 0; i3 < ros_v.sns_bk_wv_high.size(); i3++) {
	  convert(struct_v.sns_bk_wv_high[i3], ros_v.sns_bk_wv_high[i3], type);
  }
  for (size_t i4 = 0; i4 < ros_v.sns_measure_mode.size(); i4++) {
	  convert(struct_v.sns_measure_mode[i4], ros_v.sns_measure_mode[i4], type);
  }
  for (size_t i5 = 0; i5 < ros_v.sns_bk_wv_obj_type.size(); i5++) {
	  convert(struct_v.sns_bk_wv_obj_type[i5], ros_v.sns_bk_wv_obj_type[i5], type);
  }
  for (size_t i6 = 0; i6 < ros_v.sns_work_mode.size(); i6++) {
	  convert(struct_v.sns_work_mode[i6], ros_v.sns_work_mode[i6], type);
  }
  for (size_t i7 = 0; i7 < ros_v.sns_ring_time.size(); i7++) {
	  convert(struct_v.sns_ring_time[i7], ros_v.sns_ring_time[i7], type);
  }
  for (size_t i8 = 0; i8 < ros_v.echo_type.size(); i8++) {
	  convert(struct_v.echo_type[i8], ros_v.echo_type[i8], type);
  }
}

template <typename T2>
void convert(iflyauto::SnsBkWvTimeType &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  for (size_t i0 = 0; i0 < ros_v.sns_bk_wv_time_value.size(); i0++) {
	  convert(struct_v.sns_bk_wv_time_value[i0], ros_v.sns_bk_wv_time_value[i0], type);
  }
}

template <typename T2>
void convert(iflyauto::UssWaveDebugInfo &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.msg_header, ros_v.msg_header, type);
  convert_ros_header(struct_v.msg_header, ros_v.header, type);
  convert(struct_v.sensor_meta, ros_v.sensor_meta, type);
  for (size_t i0 = 0; i0 < ros_v.sns_cmd_eeprom_data_buf.size(); i0++) {
	  convert(struct_v.sns_cmd_eeprom_data_buf[i0], ros_v.sns_cmd_eeprom_data_buf[i0], type);
  }
  for (size_t i1 = 0; i1 < ros_v.sns_cmd_eepromwrite_data_buf.size(); i1++) {
	  convert(struct_v.sns_cmd_eepromwrite_data_buf[i1], ros_v.sns_cmd_eepromwrite_data_buf[i1], type);
  }
  for (size_t i2 = 0; i2 < ros_v.uss_en_data_one_channel.size(); i2++) {
	  convert(struct_v.uss_en_data_one_channel[i2], ros_v.uss_en_data_one_channel[i2], type);
  }
  convert(struct_v.sns_command_data, ros_v.sns_command_data, type);
  for (size_t i3 = 0; i3 < ros_v.sns_bk_wv_time.size(); i3++) {
	  convert(struct_v.sns_bk_wv_time[i3], ros_v.sns_bk_wv_time[i3], type);
  }
  for (size_t i4 = 0; i4 < ros_v.sns_input_capture_int_cnt.size(); i4++) {
	  convert(struct_v.sns_input_capture_int_cnt[i4], ros_v.sns_input_capture_int_cnt[i4], type);
  }
  convert(struct_v.prepare_tobe_compare_data, ros_v.prepare_tobe_compare_data, type);
  convert(struct_v.sns_source_data, ros_v.sns_source_data, type);
}

