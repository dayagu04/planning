#pragma once

#include "base_convert.h"
#include "c/fm_info_c.h"
#include "struct_convert/common_c.h"

using namespace iflyauto;

template <typename T2>
void convert(iflyauto::FmInfo &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.alarmId, ros_v.alarmId, type);
  convert(struct_v.alarmObj, ros_v.alarmObj, type);
  convert(struct_v.alarmCount, ros_v.alarmCount, type);
  convert(struct_v.clss, ros_v.clss, type);
  convert(struct_v.level, ros_v.level, type);
  convert(struct_v.status, ros_v.status, type);
  convert(struct_v.time, ros_v.time, type);
  for (size_t i0 = 0; i0 < ros_v.desc.size(); i0++) {
	  convert(struct_v.desc[i0], ros_v.desc[i0], type);
  }
}

