#pragma once

#include "base_convert.h"
#include "c/time_synchronization_c.h"
#include "struct_convert/common_c.h"

using namespace iflyauto;

template <typename T2>
void convert(iflyauto::TimeSynchronizationInfo &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.msg_header, ros_v.msg_header, type);
  convert_ros_header(struct_v.msg_header, ros_v.header, type);
  convert(struct_v.msg_meta, ros_v.msg_meta, type);
  convert(struct_v.vartualTimeStamp, ros_v.vartualTimeStamp, type);
  convert(struct_v.realTimeStamp, ros_v.realTimeStamp, type);
}

