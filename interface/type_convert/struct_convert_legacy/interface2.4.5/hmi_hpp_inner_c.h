#pragma once

#include "base_convert.h"
#include "legacy/interface2.4.5/hmi_hpp_inner_c.h"
using namespace iflyauto;

template <typename T2>
void convert(iflyauto::interface_2_4_5::HmiHppInput &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.header, ros_v.msg_header, type);
  convert_ros_header_legacy(struct_v.header, ros_v.header, type);
  convert(struct_v.apa_active_switch, ros_v.apa_active_switch, type);
  convert(struct_v.hpp_main_switch, ros_v.hpp_main_switch, type);
  convert(struct_v.hpp_cancel_switch, ros_v.hpp_cancel_switch, type);
  convert(struct_v.hpp_active_switch, ros_v.hpp_active_switch, type);
  convert(struct_v.start_memory_parking, ros_v.start_memory_parking, type);
  convert(struct_v.continue_memory_parking, ros_v.continue_memory_parking, type);
  convert(struct_v.start_route_learning, ros_v.start_route_learning, type);
  convert(struct_v.start_parking, ros_v.start_parking, type);
  convert(struct_v.continue_parking, ros_v.continue_parking, type);
  convert(struct_v.complete_route_learning, ros_v.complete_route_learning, type);
  convert(struct_v.select_slot_id, ros_v.select_slot_id, type);
  convert(struct_v.resume_location, ros_v.resume_location, type);
}

