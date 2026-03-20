#pragma once

#include "base_convert.h"
#include "legacy/interface2.4.5/func_state_machine_c.h"
using namespace iflyauto;

template <typename T2>
void convert(iflyauto::interface_2_4_5::FuncStateMachine &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.header, ros_v.msg_header, type);
  convert_ros_header_legacy(struct_v.header, ros_v.header, type);
  convert(struct_v.current_state, ros_v.current_state, type);
  convert(struct_v.state_duration, ros_v.state_duration, type);
  convert(struct_v.message, ros_v.message, type);
}

