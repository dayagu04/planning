// Copyright (C) iflyauto Technologies (2023). All rights reserved.
// Modified: 2023/11/4

#ifndef _IFLYAUTO_LEGACY_INTERFACE2_4_5_HMI_HPP_INNER_H_
#define _IFLYAUTO_LEGACY_INTERFACE2_4_5_HMI_HPP_INNER_H_

#ifdef __cplusplus
#include "common_platform_type_soc.h"
#else
#include "Platform_Types.h"
#endif  // __cplusplus

#include "interface2.4.5/common_c.h"

#ifdef __cplusplus
namespace iflyauto {
  namespace interface_2_4_5 {
#endif

#pragma pack(4)

typedef struct {
  Header header;                    // 头信息
  boolean apa_active_switch;        // APA功能开启按钮      (true:Press/false:Release)
  boolean hpp_main_switch;          // HPP主开关            (true:Press/false:Release)
  boolean hpp_cancel_switch;        // HPP取消开关          (true:Press/false:Release)
  boolean hpp_active_switch;        // HPP激活按键          (true:Press/false:Release)
  boolean start_memory_parking;     // 开始记忆泊车         (true:Press/false:Release)
  boolean continue_memory_parking;  // 恢复记忆泊车         (true:Press/false:Release)
  boolean start_route_learning;     // 开始路径学习         (true:Press/false:Release)
  boolean start_parking;            // 开始泊车             (true:Press/false:Release)
  boolean continue_parking;         // 继续泊车             (true:Press/false:Release)
  boolean complete_route_learning;  // 完成路径学习         (true:Press/false:Release)
  uint32 select_slot_id;            // 选择的车位id
  boolean resume_location;          // 重定位               (true:Press/false:Release)
} _STRUCT_ALIGNED_ HmiHppInput;

#pragma pack()
#ifdef __cplusplus
  }  // namespace interface_2_4_5
}  // namespace iflyauto
#endif  // __cplusplus
#endif  // _IFLYAUTO_LEGACY_INTERFACE2_4_5_HMI_HPP_INNER_H_