#pragma once

#include "base_convert.h"
#include "c/fdl_trigger_c.h"
#include "struct_convert/common_c.h"

using namespace iflyauto;

template <typename T2>
void convert(iflyauto::FdlTriggerState &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.msg_header, ros_v.msg_header, type);
  convert_ros_header(struct_v.msg_header, ros_v.header, type);
  convert(struct_v.msg_meta, ros_v.msg_meta, type);
  convert(struct_v.trigger_ack, ros_v.trigger_ack, type);
  for (size_t i0 = 0; i0 < ros_v.sftp_path.size(); i0++) {
	  convert(struct_v.sftp_path[i0], ros_v.sftp_path[i0], type);
  }
  for (size_t i1 = 0; i1 < ros_v.sftp_A_log_file_name.size(); i1++) {
	  convert(struct_v.sftp_A_log_file_name[i1], ros_v.sftp_A_log_file_name[i1], type);
  }
  for (size_t i2 = 0; i2 < ros_v.sftp_plat_log_file_name.size(); i2++) {
	  convert(struct_v.sftp_plat_log_file_name[i2], ros_v.sftp_plat_log_file_name[i2], type);
  }
  for (size_t i3 = 0; i3 < ros_v.bak.size(); i3++) {
	  convert(struct_v.bak[i3], ros_v.bak[i3], type);
  }
}

