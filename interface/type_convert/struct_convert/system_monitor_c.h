#pragma once

#include "base_convert.h"
#include "c/system_monitor_c.h"
#include "struct_convert/common_c.h"

using namespace iflyauto;

template <typename T2>
void convert(iflyauto::ProcessStat &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.soc_id, ros_v.soc_id, type);
  convert(struct_v.pid, ros_v.pid, type);
  convert(struct_v.stamp, ros_v.stamp, type);
  for (size_t i0 = 0; i0 < ros_v.name.size(); i0++) {
	  convert(struct_v.name[i0], ros_v.name[i0], type);
  }
  convert(struct_v.cpu_usage, ros_v.cpu_usage, type);
  convert(struct_v.memory_usage, ros_v.memory_usage, type);
  convert(struct_v.rss, ros_v.rss, type);
  convert(struct_v.shr, ros_v.shr, type);
  convert(struct_v.pss, ros_v.pss, type);
  convert(struct_v.uss, ros_v.uss, type);
  convert(struct_v.vsize, ros_v.vsize, type);
  convert(struct_v.shr_pool, ros_v.shr_pool, type);
  convert(struct_v.read_bytes, ros_v.read_bytes, type);
  convert(struct_v.write_bytes, ros_v.write_bytes, type);
}

