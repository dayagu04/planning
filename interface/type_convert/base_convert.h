#pragma once
#include <iostream>

enum ConvertTypeInfo {
  TO_ROS = 0, 
  TO_STRUCT,
  TO_PROTO
};

template <typename T1, typename T2> 
void convert(T1 &&struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  if (type == ConvertTypeInfo::TO_ROS) {
    ros_v = struct_v;
  }
  if (type == ConvertTypeInfo::TO_STRUCT) {
    // enum 不能直接赋值
    struct_v = (T1)ros_v;
  }
};


template <typename T1, typename T2> 
void convert_ros_header(T1 &&struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  if (type == ConvertTypeInfo::TO_ROS) {
    //171 764 0053 382 945
    uint64_t timestamp = struct_v.stamp;
    int count = 1;
    while(timestamp / 10) {
      count++;
      timestamp = timestamp/10;
    }
    if (count == 13) {
      ros_v.stamp.sec = struct_v.stamp / 1000;
      ros_v.stamp.nsec = struct_v.stamp % 1000 * 1000000;
    } else {
      ros_v.stamp.sec = struct_v.stamp / 1000000;
      ros_v.stamp.nsec = struct_v.stamp % 1000000 * 1000;
    }
  }
};

template <typename T1, typename T2> 
void convert_ros_header_legacy(T1 &&struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  if (type == ConvertTypeInfo::TO_ROS) {
    //171 764 0053 382 945
    uint64_t timestamp = struct_v.timestamp;
    int count = 1;
    while(timestamp / 10) {
      count++;
      timestamp = timestamp/10;
    }
    if (count == 13) {
      ros_v.stamp.sec = struct_v.timestamp / 1000;
      ros_v.stamp.nsec = struct_v.timestamp % 1000 * 1000000;
    } else {
      ros_v.stamp.sec = struct_v.timestamp / 1000000;
      ros_v.stamp.nsec = struct_v.timestamp % 1000000 * 1000;
    }
  }
};