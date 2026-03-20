#pragma once

#include "base_convert.h"
#include "c/ifly_mapping_status_c.h"
#include "struct_convert/common_c.h"

using namespace iflyauto;

template <typename T2>
void convert(iflyauto::MappingStatusInfo &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.msg_header, ros_v.msg_header, type);
  convert_ros_header(struct_v.msg_header, ros_v.header, type);
  convert(struct_v.msg_meta, ros_v.msg_meta, type);
  convert(struct_v.driving_distance, ros_v.driving_distance, type);
  convert(struct_v.backward_distance, ros_v.backward_distance, type);
  convert(struct_v.saved_progress, ros_v.saved_progress, type);
  convert(struct_v.running_status, ros_v.running_status, type);
  convert(struct_v.failed_reason, ros_v.failed_reason, type);
  convert(struct_v.is_in_parking_slot, ros_v.is_in_parking_slot, type);
  convert(struct_v.in_slot_id, ros_v.in_slot_id, type);
}

