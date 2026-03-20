#pragma once

#include "base_convert.h"
#include "c/dtc_code_c.h"
#include "struct_convert/common_c.h"

using namespace iflyauto;

template <typename T2>
void convert(iflyauto::IFLYDtcCode &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  for (size_t i0 = 0; i0 < ros_v.code.size(); i0++) {
	  convert(struct_v.code[i0], ros_v.code[i0], type);
  }
}

template <typename T2>
void convert(iflyauto::IFLYAllDtcState &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.msg_header, ros_v.msg_header, type);
  convert_ros_header(struct_v.msg_header, ros_v.header, type);
  convert(struct_v.msg_meta, ros_v.msg_meta, type);
  convert(struct_v.dtc_code_size, ros_v.dtc_code_size, type);
  // flex array
  if (type == ConvertTypeInfo::TO_ROS) {
    if (struct_v.dtc_code_size >= 0 && struct_v.dtc_code_size <= DTC_CODE_ALL_STATE_MAX_NUM) {
      ros_v.dtc_code.resize(struct_v.dtc_code_size);
    } else {
      std::cout << "convert/dtc_code_c.h:" << __LINE__ 
                << " [convert][TO_ROS] dtc_code_size=" << struct_v.dtc_code_size 
                << " not in range DTC_CODE_ALL_STATE_MAX_NUM=" << DTC_CODE_ALL_STATE_MAX_NUM 
                << std::endl;
      ros_v.dtc_code_size = DTC_CODE_ALL_STATE_MAX_NUM;
      ros_v.dtc_code.resize(DTC_CODE_ALL_STATE_MAX_NUM);
    }
    for (size_t i0 = 0; i0 < ros_v.dtc_code.size(); i0++) {
      convert(struct_v.dtc_code[i0], ros_v.dtc_code[i0], type);
    }
  } 
  if (type == ConvertTypeInfo::TO_STRUCT) {
    if (ros_v.dtc_code_size > DTC_CODE_ALL_STATE_MAX_NUM || ros_v.dtc_code_size < 0 || ros_v.dtc_code.size() > DTC_CODE_ALL_STATE_MAX_NUM) {
      std::cout << "convert/dtc_code_c.h:" << __LINE__ 
                << "[convert][TO_STRUCT] dtc_code_size=" << ros_v.dtc_code_size 
                << " ros_v.dtc_code.size()=" << ros_v.dtc_code.size()
                << " not in range DTC_CODE_ALL_STATE_MAX_NUM=" << DTC_CODE_ALL_STATE_MAX_NUM 
                << std::endl;
    }
    if (ros_v.dtc_code.size() > DTC_CODE_ALL_STATE_MAX_NUM) {
      for (size_t i0 = 0; i0 < DTC_CODE_ALL_STATE_MAX_NUM; i0++) {
        convert(struct_v.dtc_code[i0], ros_v.dtc_code[i0], type);
      }
    } else {
      for (size_t i0 = 0; i0 < ros_v.dtc_code.size(); i0++) {
        convert(struct_v.dtc_code[i0], ros_v.dtc_code[i0], type);
      }
    }
  }
  //
}

