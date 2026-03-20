#pragma once

#include "base_convert.h"
#include "c/fsm_mcu_out_c.h"
#include "struct_convert/common_c.h"

using namespace iflyauto;

template <typename T2>
void convert(iflyauto::FsmMcuOut &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.msg_header, ros_v.msg_header, type);
  convert_ros_header(struct_v.msg_header, ros_v.header, type);
  convert(struct_v.msg_meta, ros_v.msg_meta, type);
  convert(struct_v.mcu_state, ros_v.mcu_state, type);
  convert(struct_v.lat_override_apa, ros_v.lat_override_apa, type);
  convert(struct_v.lat_override_pilot, ros_v.lat_override_pilot, type);
  convert(struct_v.heat_req_info, ros_v.heat_req_info, type);
}

