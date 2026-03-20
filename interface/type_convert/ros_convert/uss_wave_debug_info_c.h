#pragma once

#include "base_convert.h"
#include "rosbag_converter/converter_register.h"

#include "struct_msgs/EchoInfo.h"
#include "struct_msgs_v2_10/EchoInfo.h"
#include "struct_msgs/EchoTypeBuf.h"
#include "struct_msgs_v2_10/EchoTypeBuf.h"
#include "struct_msgs/RcDataTypeDebug.h"
#include "struct_msgs_v2_10/RcDataTypeDebug.h"
#include "struct_msgs/SnsBkWvHighBuf.h"
#include "struct_msgs_v2_10/SnsBkWvHighBuf.h"
#include "struct_msgs/SnsBkWvObjTypeBuf.h"
#include "struct_msgs_v2_10/SnsBkWvObjTypeBuf.h"
#include "struct_msgs/SnsBkWvStartTimeBuf.h"
#include "struct_msgs_v2_10/SnsBkWvStartTimeBuf.h"
#include "struct_msgs/SnsBkWvTimeType.h"
#include "struct_msgs_v2_10/SnsBkWvTimeType.h"
#include "struct_msgs/SnsBkWvWidthBuf.h"
#include "struct_msgs_v2_10/SnsBkWvWidthBuf.h"
#include "struct_msgs/SnsCmdEepromData.h"
#include "struct_msgs_v2_10/SnsCmdEepromData.h"
#include "struct_msgs/SnsDtBufType.h"
#include "struct_msgs_v2_10/SnsDtBufType.h"
#include "struct_msgs/SnsOpInfoType.h"
#include "struct_msgs_v2_10/SnsOpInfoType.h"
#include "struct_msgs/USSenDataOneChannel.h"
#include "struct_msgs_v2_10/USSenDataOneChannel.h"
#include "struct_msgs/UssWaveDebugInfo.h"
#include "struct_msgs_v2_10/UssWaveDebugInfo.h"

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::EchoInfo &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.u8s_uss_echo_type, ros_v.u8s_uss_echo_type, type);
	convert(old_ros_v.u16s_uss_echo_timestamp, ros_v.u16s_uss_echo_timestamp, type);
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::EchoTypeBuf &ros_v, ConvertTypeInfo type) {
	for (int i = 0; i < 10; i++) {
	    convert(old_ros_v.echo_type_value[i], ros_v.echo_type_value[i], type);
	}
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::RcDataTypeDebug &ros_v, ConvertTypeInfo type) {
	for (int i = 0; i < 20; i++) {
	    convert(old_ros_v.rcv_data_value[i], ros_v.rcv_data_value[i], type);
	}
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::SnsBkWvHighBuf &ros_v, ConvertTypeInfo type) {
	for (int i = 0; i < 30; i++) {
	    convert(old_ros_v.sns_bk_wv_high_value[i], ros_v.sns_bk_wv_high_value[i], type);
	}
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::SnsBkWvObjTypeBuf &ros_v, ConvertTypeInfo type) {
	for (int i = 0; i < 10; i++) {
	    convert(old_ros_v.sns_bk_wv_obj_type_value[i], ros_v.sns_bk_wv_obj_type_value[i], type);
	}
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::SnsBkWvStartTimeBuf &ros_v, ConvertTypeInfo type) {
	for (int i = 0; i < 30; i++) {
	    convert(old_ros_v.sns_bk_wv_start_time_value[i], ros_v.sns_bk_wv_start_time_value[i], type);
	}
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::SnsBkWvTimeType &ros_v, ConvertTypeInfo type) {
	for (int i = 0; i < 70; i++) {
	    convert(old_ros_v.sns_bk_wv_time_value[i], ros_v.sns_bk_wv_time_value[i], type);
	}
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::SnsBkWvWidthBuf &ros_v, ConvertTypeInfo type) {
	for (int i = 0; i < 30; i++) {
	    convert(old_ros_v.sns_bk_wv_width_value[i], ros_v.sns_bk_wv_width_value[i], type);
	}
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::SnsCmdEepromData &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.data_valid_flag, ros_v.data_valid_flag, type);
	convert(old_ros_v.void0, ros_v.void0, type);
	convert(old_ros_v.osc_trim, ros_v.osc_trim, type);
	convert(old_ros_v.void1, ros_v.void1, type);
	convert(old_ros_v.customer_bits, ros_v.customer_bits, type);
	convert(old_ros_v.void2, ros_v.void2, type);
	convert(old_ros_v.gain_digital, ros_v.gain_digital, type);
	convert(old_ros_v.void3, ros_v.void3, type);
	convert(old_ros_v.gain_analog, ros_v.gain_analog, type);
	convert(old_ros_v.void4, ros_v.void4, type);
	convert(old_ros_v.drive_current, ros_v.drive_current, type);
	convert(old_ros_v.void5, ros_v.void5, type);
	convert(old_ros_v.drive_frequence, ros_v.drive_frequence, type);
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::SnsDtBufType &ros_v, ConvertTypeInfo type) {
	for (int i = 0; i < 12; i++) {
	    convert(old_ros_v.sns_bk_wv_start_time[i], ros_v.sns_bk_wv_start_time[i], type);
	}
	for (int i = 0; i < 12; i++) {
	    convert(old_ros_v.sns_wv_number[i], ros_v.sns_wv_number[i], type);
	}
	for (int i = 0; i < 12; i++) {
	    convert(old_ros_v.sns_bk_wv_width[i], ros_v.sns_bk_wv_width[i], type);
	}
	for (int i = 0; i < 12; i++) {
	    convert(old_ros_v.sns_bk_wv_high[i], ros_v.sns_bk_wv_high[i], type);
	}
	for (int i = 0; i < 12; i++) {
	    convert(old_ros_v.sns_measure_mode[i], ros_v.sns_measure_mode[i], type);
	}
	for (int i = 0; i < 12; i++) {
	    convert(old_ros_v.sns_bk_wv_obj_type[i], ros_v.sns_bk_wv_obj_type[i], type);
	}
	for (int i = 0; i < 12; i++) {
	    convert(old_ros_v.sns_work_mode[i], ros_v.sns_work_mode[i], type);
	}
	for (int i = 0; i < 12; i++) {
	    convert(old_ros_v.sns_ring_time[i], ros_v.sns_ring_time[i], type);
	}
	for (int i = 0; i < 12; i++) {
	    convert(old_ros_v.echo_type[i], ros_v.echo_type[i], type);
	}
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::SnsOpInfoType &ros_v, ConvertTypeInfo type) {
	for (int i = 0; i < 12; i++) {
	    convert(old_ros_v.sns_data[i], ros_v.sns_data[i], type);
	}
	convert(old_ros_v.cur_bit_index, ros_v.cur_bit_index, type);
	convert(old_ros_v.sns_op_cmd_tx_bit_cnt, ros_v.sns_op_cmd_tx_bit_cnt, type);
	convert(old_ros_v.data_bit_index, ros_v.data_bit_index, type);
	convert(old_ros_v.data_cnt, ros_v.data_cnt, type);
	convert(old_ros_v.data_addr, ros_v.data_addr, type);
	convert(old_ros_v.data_rx_try_cnt, ros_v.data_rx_try_cnt, type);
	for (int i = 0; i < 12; i++) {
	    convert(old_ros_v.check_sum[i], ros_v.check_sum[i], type);
	}
	for (int i = 0; i < 12; i++) {
	    convert(old_ros_v.prev_sns_bkwv_time[i], ros_v.prev_sns_bkwv_time[i], type);
	}
	for (int i = 0; i < 12; i++) {
	    convert(old_ros_v.rcv_data_completed_flag[i], ros_v.rcv_data_completed_flag[i], type);
	}
	for (int i = 0; i < 12; i++) {
	    convert(old_ros_v.need_rcv_data_bit_length[i], ros_v.need_rcv_data_bit_length[i], type);
	}
	for (int i = 0; i < 12; i++) {
	    convert(old_ros_v.rcv_data_bit_index[i], ros_v.rcv_data_bit_index[i], type);
	}
	for (int i = 0; i < 12; i++) {
	    convert(old_ros_v.cmd_processed_state[i], ros_v.cmd_processed_state[i], type);
	}
	for (int i = 0; i < 12; i++) {
	    convert(old_ros_v.rcv_data_cmd_type[i], ros_v.rcv_data_cmd_type[i], type);
	}
	for (int i = 0; i < 12; i++) {
	    convert(old_ros_v.rcv_data_buf[i], ros_v.rcv_data_buf[i], type);
	}
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::USSenDataOneChannel &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.u8s_uss_sfl_burst_mode, ros_v.u8s_uss_sfl_burst_mode, type);
	convert(old_ros_v.u8s_uss_sfl_meas_mode, ros_v.u8s_uss_sfl_meas_mode, type);
	for (int i = 0; i < 10; i++) {
	    convert(old_ros_v.echo_info[i], ros_v.echo_info[i], type);
	}
}

template <typename T1>
void convert(T1 &&old_ros_v, struct_msgs::UssWaveDebugInfo &ros_v, ConvertTypeInfo type) {
	convert(old_ros_v.header, ros_v.header, type);
	convert(old_ros_v.msg_header, ros_v.msg_header, type);
	convert(old_ros_v.sensor_meta, ros_v.sensor_meta, type);
	for (int i = 0; i < 12; i++) {
	    convert(old_ros_v.sns_cmd_eeprom_data_buf[i], ros_v.sns_cmd_eeprom_data_buf[i], type);
	}
	for (int i = 0; i < 12; i++) {
	    convert(old_ros_v.sns_cmd_eepromwrite_data_buf[i], ros_v.sns_cmd_eepromwrite_data_buf[i], type);
	}
	for (int i = 0; i < 12; i++) {
	    convert(old_ros_v.uss_en_data_one_channel[i], ros_v.uss_en_data_one_channel[i], type);
	}
	convert(old_ros_v.sns_command_data, ros_v.sns_command_data, type);
	for (int i = 0; i < 12; i++) {
	    convert(old_ros_v.sns_bk_wv_time[i], ros_v.sns_bk_wv_time[i], type);
	}
	for (int i = 0; i < 12; i++) {
	    convert(old_ros_v.sns_input_capture_int_cnt[i], ros_v.sns_input_capture_int_cnt[i], type);
	}
	convert(old_ros_v.prepare_tobe_compare_data, ros_v.prepare_tobe_compare_data, type);
	convert(old_ros_v.sns_source_data, ros_v.sns_source_data, type);
}

