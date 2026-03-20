#pragma once

#include "base_convert.h"
#include "c/can_raw_eth_data_c.h"
#include "struct_convert/common_c.h"

using namespace iflyauto;

template <typename T2>
void convert(iflyauto::CanRawMsg &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.canId, ros_v.canId, type);
  convert(struct_v.can_data_size, ros_v.can_data_size, type);
  // flex array
  if (type == ConvertTypeInfo::TO_ROS) {
    if (struct_v.can_data_size >= 0 && struct_v.can_data_size <= CAN_DATA_NUM_MAX) {
      ros_v.can_data.resize(struct_v.can_data_size);
    } else {
      std::cout << "convert/can_raw_eth_data_c.h:" << __LINE__ 
                << " [convert][TO_ROS] can_data_size=" << struct_v.can_data_size 
                << " not in range CAN_DATA_NUM_MAX=" << CAN_DATA_NUM_MAX 
                << std::endl;
      ros_v.can_data_size = CAN_DATA_NUM_MAX;
      ros_v.can_data.resize(CAN_DATA_NUM_MAX);
    }
    for (size_t i0 = 0; i0 < ros_v.can_data.size(); i0++) {
      convert(struct_v.can_data[i0], ros_v.can_data[i0], type);
    }
  } 
  if (type == ConvertTypeInfo::TO_STRUCT) {
    if (ros_v.can_data_size > CAN_DATA_NUM_MAX || ros_v.can_data_size < 0 || ros_v.can_data.size() > CAN_DATA_NUM_MAX) {
      std::cout << "convert/can_raw_eth_data_c.h:" << __LINE__ 
                << "[convert][TO_STRUCT] can_data_size=" << ros_v.can_data_size 
                << " ros_v.can_data.size()=" << ros_v.can_data.size()
                << " not in range CAN_DATA_NUM_MAX=" << CAN_DATA_NUM_MAX 
                << std::endl;
    }
    if (ros_v.can_data.size() > CAN_DATA_NUM_MAX) {
      for (size_t i0 = 0; i0 < CAN_DATA_NUM_MAX; i0++) {
        convert(struct_v.can_data[i0], ros_v.can_data[i0], type);
      }
    } else {
      for (size_t i0 = 0; i0 < ros_v.can_data.size(); i0++) {
        convert(struct_v.can_data[i0], ros_v.can_data[i0], type);
      }
    }
  }
  //
  for (size_t i1 = 0; i1 < ros_v.resv.size(); i1++) {
	  convert(struct_v.resv[i1], ros_v.resv[i1], type);
  }
  convert(struct_v.timestamp, ros_v.timestamp, type);
}

template <typename T2>
void convert(iflyauto::CanRawEthData &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.msg_header, ros_v.msg_header, type);
  convert_ros_header(struct_v.msg_header, ros_v.header, type);
  convert(struct_v.canRawMsg_size, ros_v.canRawMsg_size, type);
  // flex array
  if (type == ConvertTypeInfo::TO_ROS) {
    if (struct_v.canRawMsg_size >= 0 && struct_v.canRawMsg_size <= MESSAGE_NUM_MAX) {
      ros_v.canRawMsg.resize(struct_v.canRawMsg_size);
    } else {
      std::cout << "convert/can_raw_eth_data_c.h:" << __LINE__ 
                << " [convert][TO_ROS] canRawMsg_size=" << struct_v.canRawMsg_size 
                << " not in range MESSAGE_NUM_MAX=" << MESSAGE_NUM_MAX 
                << std::endl;
      ros_v.canRawMsg_size = MESSAGE_NUM_MAX;
      ros_v.canRawMsg.resize(MESSAGE_NUM_MAX);
    }
    for (size_t i0 = 0; i0 < ros_v.canRawMsg.size(); i0++) {
      convert(struct_v.canRawMsg[i0], ros_v.canRawMsg[i0], type);
    }
  } 
  if (type == ConvertTypeInfo::TO_STRUCT) {
    if (ros_v.canRawMsg_size > MESSAGE_NUM_MAX || ros_v.canRawMsg_size < 0 || ros_v.canRawMsg.size() > MESSAGE_NUM_MAX) {
      std::cout << "convert/can_raw_eth_data_c.h:" << __LINE__ 
                << "[convert][TO_STRUCT] canRawMsg_size=" << ros_v.canRawMsg_size 
                << " ros_v.canRawMsg.size()=" << ros_v.canRawMsg.size()
                << " not in range MESSAGE_NUM_MAX=" << MESSAGE_NUM_MAX 
                << std::endl;
    }
    if (ros_v.canRawMsg.size() > MESSAGE_NUM_MAX) {
      for (size_t i0 = 0; i0 < MESSAGE_NUM_MAX; i0++) {
        convert(struct_v.canRawMsg[i0], ros_v.canRawMsg[i0], type);
      }
    } else {
      for (size_t i0 = 0; i0 < ros_v.canRawMsg.size(); i0++) {
        convert(struct_v.canRawMsg[i0], ros_v.canRawMsg[i0], type);
      }
    }
  }
  //
}

template <typename T2>
void convert(iflyauto::CanAscData &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.msg_header, ros_v.msg_header, type);
  convert_ros_header(struct_v.msg_header, ros_v.header, type);
  for (size_t i0 = 0; i0 < ros_v.asc_header.size(); i0++) {
	  convert(struct_v.asc_header[i0], ros_v.asc_header[i0], type);
  }
  convert(struct_v.asc_data_size, ros_v.asc_data_size, type);
  // flex array
  if (type == ConvertTypeInfo::TO_ROS) {
    if (struct_v.asc_data_size >= 0 && struct_v.asc_data_size <= ASC_DATA_NUM_MAX) {
      ros_v.asc_data.resize(struct_v.asc_data_size);
    } else {
      std::cout << "convert/can_raw_eth_data_c.h:" << __LINE__ 
                << " [convert][TO_ROS] asc_data_size=" << struct_v.asc_data_size 
                << " not in range ASC_DATA_NUM_MAX=" << ASC_DATA_NUM_MAX 
                << std::endl;
      ros_v.asc_data_size = ASC_DATA_NUM_MAX;
      ros_v.asc_data.resize(ASC_DATA_NUM_MAX);
    }
    for (size_t i1 = 0; i1 < ros_v.asc_data.size(); i1++) {
      convert(struct_v.asc_data[i1], ros_v.asc_data[i1], type);
    }
  } 
  if (type == ConvertTypeInfo::TO_STRUCT) {
    if (ros_v.asc_data_size > ASC_DATA_NUM_MAX || ros_v.asc_data_size < 0 || ros_v.asc_data.size() > ASC_DATA_NUM_MAX) {
      std::cout << "convert/can_raw_eth_data_c.h:" << __LINE__ 
                << "[convert][TO_STRUCT] asc_data_size=" << ros_v.asc_data_size 
                << " ros_v.asc_data.size()=" << ros_v.asc_data.size()
                << " not in range ASC_DATA_NUM_MAX=" << ASC_DATA_NUM_MAX 
                << std::endl;
    }
    if (ros_v.asc_data.size() > ASC_DATA_NUM_MAX) {
      for (size_t i1 = 0; i1 < ASC_DATA_NUM_MAX; i1++) {
        convert(struct_v.asc_data[i1], ros_v.asc_data[i1], type);
      }
    } else {
      for (size_t i1 = 0; i1 < ros_v.asc_data.size(); i1++) {
        convert(struct_v.asc_data[i1], ros_v.asc_data[i1], type);
      }
    }
  }
  //
}

