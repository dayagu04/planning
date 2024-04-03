#pragma once

#include <pybind11/pytypes.h>
#include <ros/ros.h>
#include <ros/serialization.h>

#include "interface/type_convert/base_convert.h"

template <class T>
inline T BytesToProto(pybind11::bytes &bytes) {
  T proto_obj;
  pybind11::buffer buf(bytes);
  pybind11::buffer_info input_info = buf.request();
  char *input_ptr = static_cast<char *>(input_info.ptr);
  std::string input_s(input_ptr, input_info.size);

  T input;
  input.ParseFromString(input_s);
  return input;
}

template <class T>
pybind11::bytes ProtoToBytes(T proto) {
  std::string serialized_message;
  proto.SerializeToString(&serialized_message);
  return serialized_message;
}

template <typename T1, typename T2>
inline T1 BytesToStruct(const pybind11::bytes &bytes) {
  pybind11::buffer buffer(bytes);
  pybind11::buffer_info input_info = buffer.request();
  ros::serialization::IStream stream(static_cast<uint8_t *>(input_info.ptr),
                                     input_info.size);

  T2 ros_msg;
  ros::serialization::deserialize(stream, ros_msg);

  T1 c_struct{};
  convert(c_struct, ros_msg, ConvertTypeInfo::TO_STRUCT);

  return c_struct;
}

template <typename T1, typename T2>
pybind11::bytes StructToBytes(T1 &c_struct) {
  T2 ros_msg;
  convert(c_struct, ros_msg, ConvertTypeInfo::TO_ROS);

  size_t serial_size = ros::serialization::serializationLength(ros_msg);
  boost::shared_array<uint8_t> buffer(new uint8_t[serial_size]);
  ros::serialization::OStream stream(buffer.get(), serial_size);
  ros::serialization::serialize(stream, ros_msg);
  std::string str_msg;
  str_msg.reserve(serial_size);
  for (size_t i = 0; i < serial_size; ++i) {
    str_msg.push_back(buffer[i]);
  }
  return str_msg;
}