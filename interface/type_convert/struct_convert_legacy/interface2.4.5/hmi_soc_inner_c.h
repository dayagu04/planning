#pragma once

#include "base_convert.h"
#include "legacy/interface2.4.5/hmi_soc_inner_c.h"
using namespace iflyauto;

template <typename T2>
void convert(iflyauto::interface_2_4_5::HmiSocInner &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.header, ros_v.msg_header, type);
  convert_ros_header_legacy(struct_v.header, ros_v.header, type);
  convert(struct_v.apa_main_switch, ros_v.apa_main_switch, type);
  convert(struct_v.apa_active_switch, ros_v.apa_active_switch, type);
  convert(struct_v.apa_active_mode, ros_v.apa_active_mode, type);
  convert(struct_v.apa_cancel_switch, ros_v.apa_cancel_switch, type);
  convert(struct_v.apa_select_slot_id, ros_v.apa_select_slot_id, type);
  convert(struct_v.apa_start, ros_v.apa_start, type);
  convert(struct_v.apa_resume, ros_v.apa_resume, type);
  convert(struct_v.calib_active_switch, ros_v.calib_active_switch, type);
}

