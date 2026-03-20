#pragma once

#include "base_convert.h"
#include "c/engineer_mode_c.h"
#include "struct_convert/common_c.h"

using namespace iflyauto;

template <typename T2>
void convert(iflyauto::Log_Req &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.start_time, ros_v.start_time, type);
  convert(struct_v.stop_time, ros_v.stop_time, type);
  convert(struct_v.logmode, ros_v.logmode, type);
}

template <typename T2>
void convert(iflyauto::Engineer_Inner &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.msg_header, ros_v.msg_header, type);
  convert_ros_header(struct_v.msg_header, ros_v.header, type);
  convert(struct_v.msg_meta, ros_v.msg_meta, type);
  convert(struct_v.engineer_mode, ros_v.engineer_mode, type);
  convert(struct_v.fault_degraded_swt, ros_v.fault_degraded_swt, type);
  convert(struct_v.seatbelt_swt, ros_v.seatbelt_swt, type);
  convert(struct_v.lever_swt, ros_v.lever_swt, type);
  convert(struct_v.str_swt, ros_v.str_swt, type);
  convert(struct_v.log_req, ros_v.log_req, type);
}

template <typename T2>
void convert(iflyauto::Process_Status &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  for (size_t i0 = 0; i0 < ros_v.process_name.size(); i0++) {
	  convert(struct_v.process_name[i0], ros_v.process_name[i0], type);
  }
  convert(struct_v.em_status, ros_v.em_status, type);
}

template <typename T2>
void convert(iflyauto::MDC_Status &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  for (size_t i0 = 0; i0 < ros_v.version.size(); i0++) {
	  convert(struct_v.version[i0], ros_v.version[i0], type);
  }
  convert(struct_v.factory_sts, ros_v.factory_sts, type);
  convert(struct_v.calibration_sts, ros_v.calibration_sts, type);
  for (size_t i1 = 0; i1 < ros_v.veh_vin.size(); i1++) {
	  convert(struct_v.veh_vin[i1], ros_v.veh_vin[i1], type);
  }
  convert(struct_v.process_num, ros_v.process_num, type);
  for (size_t i2 = 0; i2 < ros_v.process_status.size(); i2++) {
	  convert(struct_v.process_status[i2], ros_v.process_status[i2], type);
  }
}

template <typename T2>
void convert(iflyauto::Sftp_Info &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  for (size_t i0 = 0; i0 < ros_v.usr_name.size(); i0++) {
	  convert(struct_v.usr_name[i0], ros_v.usr_name[i0], type);
  }
  for (size_t i1 = 0; i1 < ros_v.password.size(); i1++) {
	  convert(struct_v.password[i1], ros_v.password[i1], type);
  }
  for (size_t i2 = 0; i2 < ros_v.ftppath.size(); i2++) {
	  convert(struct_v.ftppath[i2], ros_v.ftppath[i2], type);
  }
}

template <typename T2>
void convert(iflyauto::Engineer_FaultInfo &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.alarmId, ros_v.alarmId, type);
  convert(struct_v.alarmObj, ros_v.alarmObj, type);
}

template <typename T2>
void convert(iflyauto::Engineer_Outer &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.msg_header, ros_v.msg_header, type);
  convert_ros_header(struct_v.msg_header, ros_v.header, type);
  convert(struct_v.msg_meta, ros_v.msg_meta, type);
  convert(struct_v.mdc_status, ros_v.mdc_status, type);
  convert(struct_v.fault_info_size, ros_v.fault_info_size, type);
  // flex array
  if (type == ConvertTypeInfo::TO_ROS) {
    if (struct_v.fault_info_size >= 0 && struct_v.fault_info_size <= FAULT_INFO_SIZE) {
      ros_v.fault_info.resize(struct_v.fault_info_size);
    } else {
      std::cout << "convert/engineer_mode_c.h:" << __LINE__ 
                << " [convert][TO_ROS] fault_info_size=" << struct_v.fault_info_size 
                << " not in range FAULT_INFO_SIZE=" << FAULT_INFO_SIZE 
                << std::endl;
      ros_v.fault_info_size = FAULT_INFO_SIZE;
      ros_v.fault_info.resize(FAULT_INFO_SIZE);
    }
    for (size_t i0 = 0; i0 < ros_v.fault_info.size(); i0++) {
      convert(struct_v.fault_info[i0], ros_v.fault_info[i0], type);
    }
  } 
  if (type == ConvertTypeInfo::TO_STRUCT) {
    if (ros_v.fault_info_size > FAULT_INFO_SIZE || ros_v.fault_info_size < 0 || ros_v.fault_info.size() > FAULT_INFO_SIZE) {
      std::cout << "convert/engineer_mode_c.h:" << __LINE__ 
                << "[convert][TO_STRUCT] fault_info_size=" << ros_v.fault_info_size 
                << " ros_v.fault_info.size()=" << ros_v.fault_info.size()
                << " not in range FAULT_INFO_SIZE=" << FAULT_INFO_SIZE 
                << std::endl;
    }
    if (ros_v.fault_info.size() > FAULT_INFO_SIZE) {
      for (size_t i0 = 0; i0 < FAULT_INFO_SIZE; i0++) {
        convert(struct_v.fault_info[i0], ros_v.fault_info[i0], type);
      }
    } else {
      for (size_t i0 = 0; i0 < ros_v.fault_info.size(); i0++) {
        convert(struct_v.fault_info[i0], ros_v.fault_info[i0], type);
      }
    }
  }
  //
  convert(struct_v.sftp_info, ros_v.sftp_info, type);
}

