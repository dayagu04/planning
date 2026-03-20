#pragma once

#include "base_convert.h"
#include "legacy/interface2.4.5/hmi_mcu_inner_c.h"
using namespace iflyauto;

template <typename T2>
void convert(iflyauto::interface_2_4_5::HmiMcuInner &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.header, ros_v.msg_header, type);
  convert_ros_header_legacy(struct_v.header, ros_v.header, type);
  convert(struct_v.acc_main_switch, ros_v.acc_main_switch, type);
  convert(struct_v.acc_active_switch, ros_v.acc_active_switch, type);
  convert(struct_v.acc_cancel_switch, ros_v.acc_cancel_switch, type);
  convert(struct_v.acc_set_real_speed, ros_v.acc_set_real_speed, type);
  convert(struct_v.acc_set_disp_speed, ros_v.acc_set_disp_speed, type);
  convert(struct_v.acc_set_time_interval, ros_v.acc_set_time_interval, type);
  convert(struct_v.acc_stop2go, ros_v.acc_stop2go, type);
  convert(struct_v.scc_main_switch, ros_v.scc_main_switch, type);
  convert(struct_v.scc_active_switch, ros_v.scc_active_switch, type);
  convert(struct_v.scc_cancel_switch, ros_v.scc_cancel_switch, type);
  convert(struct_v.noa_main_switch, ros_v.noa_main_switch, type);
  convert(struct_v.noa_active_switch, ros_v.noa_active_switch, type);
  convert(struct_v.noa_voice_promp_switch, ros_v.noa_voice_promp_switch, type);
  convert(struct_v.noa_cruise_dclc_switch, ros_v.noa_cruise_dclc_switch, type);
  convert(struct_v.noa_cancel_switch, ros_v.noa_cancel_switch, type);
  convert(struct_v.fcw_main_switch, ros_v.fcw_main_switch, type);
  convert(struct_v.fcw_set_sensitivity_level, ros_v.fcw_set_sensitivity_level, type);
  convert(struct_v.aeb_main_switch, ros_v.aeb_main_switch, type);
  convert(struct_v.tsr_main_switch, ros_v.tsr_main_switch, type);
  convert(struct_v.ihc_main_switch, ros_v.ihc_main_switch, type);
  convert(struct_v.ldw_main_switch, ros_v.ldw_main_switch, type);
  convert(struct_v.ldw_set_sensitivity_level, ros_v.ldw_set_sensitivity_level, type);
  convert(struct_v.elk_main_switch, ros_v.elk_main_switch, type);
  convert(struct_v.bsd_main_switch, ros_v.bsd_main_switch, type);
  convert(struct_v.lca_main_switch, ros_v.lca_main_switch, type);
  convert(struct_v.dow_main_switch, ros_v.dow_main_switch, type);
  convert(struct_v.fcta_main_switch, ros_v.fcta_main_switch, type);
  convert(struct_v.rcta_main_switch, ros_v.rcta_main_switch, type);
  convert(struct_v.rcw_main_switch, ros_v.rcw_main_switch, type);
  convert(struct_v.ldp_main_switch, ros_v.ldp_main_switch, type);
  convert(struct_v.apa_main_switch, ros_v.apa_main_switch, type);
  convert(struct_v.apa_active_switch, ros_v.apa_active_switch, type);
  convert(struct_v.apa_cancel_switch, ros_v.apa_cancel_switch, type);
  convert(struct_v.apa_select_slot_id, ros_v.apa_select_slot_id, type);
  convert(struct_v.apa_avm_main_switch, ros_v.apa_avm_main_switch, type);
  convert(struct_v.apa_parking_direction, ros_v.apa_parking_direction, type);
  convert(struct_v.apa_park_out_direction, ros_v.apa_park_out_direction, type);
}

