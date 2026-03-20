#pragma once

#include "base_convert.h"
#include "c/system_version_c.h"
#include "struct_convert/common_c.h"

using namespace iflyauto;

template <typename T2>
void convert(iflyauto::ModuleInfo &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  for (size_t i0 = 0; i0 < ros_v.name.size(); i0++) {
	  convert(struct_v.name[i0], ros_v.name[i0], type);
  }
  for (size_t i1 = 0; i1 < ros_v.branch.size(); i1++) {
	  convert(struct_v.branch[i1], ros_v.branch[i1], type);
  }
  for (size_t i2 = 0; i2 < ros_v.commitid.size(); i2++) {
	  convert(struct_v.commitid[i2], ros_v.commitid[i2], type);
  }
}

template <typename T2>
void convert(iflyauto::SystemVersion &struct_v, T2 &&ros_v, ConvertTypeInfo type) {
  convert(struct_v.msg_header, ros_v.msg_header, type);
  convert_ros_header(struct_v.msg_header, ros_v.header, type);
  for (size_t i0 = 0; i0 < ros_v.system_version.size(); i0++) {
	  convert(struct_v.system_version[i0], ros_v.system_version[i0], type);
  }
  for (size_t i1 = 0; i1 < ros_v.interface_version.size(); i1++) {
	  convert(struct_v.interface_version[i1], ros_v.interface_version[i1], type);
  }
  convert(struct_v.module_count, ros_v.module_count, type);
  for (size_t i2 = 0; i2 < ros_v.modules.size(); i2++) {
	  convert(struct_v.modules[i2], ros_v.modules[i2], type);
  }
}

