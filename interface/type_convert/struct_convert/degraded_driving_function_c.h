#pragma once

#include "base_convert.h"
#include "c/degraded_driving_function_c.h"
#include "struct_convert/common_c.h"

using namespace iflyauto;

template <typename T2>
void convert(iflyauto::FunctionStatus &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.degraded, ros_v.degraded, type);
  convert(struct_v.hmi_mask, ros_v.hmi_mask, type);
}

template <typename T2>
void convert(iflyauto::DegradedDrivingFunction &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.msg_header, ros_v.msg_header, type);
  convert_ros_header(struct_v.msg_header, ros_v.header, type);
  convert(struct_v.msg_meta, ros_v.msg_meta, type);
  convert(struct_v.acc, ros_v.acc, type);
  convert(struct_v.lcc, ros_v.lcc, type);
  convert(struct_v.hnoa, ros_v.hnoa, type);
  convert(struct_v.mnoa, ros_v.mnoa, type);
  convert(struct_v.aeb, ros_v.aeb, type);
  convert(struct_v.ldw, ros_v.ldw, type);
  convert(struct_v.ldp, ros_v.ldp, type);
  convert(struct_v.elk, ros_v.elk, type);
  convert(struct_v.tsr, ros_v.tsr, type);
  convert(struct_v.isli, ros_v.isli, type);
  convert(struct_v.ihc, ros_v.ihc, type);
  convert(struct_v.rpa, ros_v.rpa, type);
  convert(struct_v.apa, ros_v.apa, type);
  convert(struct_v.rads, ros_v.rads, type);
  convert(struct_v.meb, ros_v.meb, type);
  convert(struct_v.hpp, ros_v.hpp, type);
  convert(struct_v.mrm, ros_v.mrm, type);
  convert(struct_v.pa, ros_v.pa, type);
  convert(struct_v.nra, ros_v.nra, type);
  convert(struct_v.mapping, ros_v.mapping, type);
  convert(struct_v.fcta_fctb, ros_v.fcta_fctb, type);
  convert(struct_v.rcta_rctb, ros_v.rcta_rctb, type);
  convert(struct_v.rcw, ros_v.rcw, type);
  convert(struct_v.dow, ros_v.dow, type);
  convert(struct_v.bsd, ros_v.bsd, type);
  convert(struct_v.amap, ros_v.amap, type);
  convert(struct_v.dai, ros_v.dai, type);
}

