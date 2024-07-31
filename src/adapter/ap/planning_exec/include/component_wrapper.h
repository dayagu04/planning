#pragma once

#include <atomic>
#include <iostream>
#include <memory>
#include <thread>

#include "iflyauto_container_pub.h"
#include "iflyauto_timer_component.h"
#include "mdc/common_swc_lib/common_swc_lib_swc.h"
#include "mdc/common_swc_lib/fmalarmreceiveservice_client.h"
#include "mdc/common_swc_lib/structcontainerinterface_client.h"
#include "mdc/common_swc_lib/structcontainerinterface_server.h"
#include "mdc/mdc_adaptive_application.h"
#include "planning_adapter.h"
namespace iflyauto {

class ComponentWrapper {
 public:
  ComponentWrapper();
  ~ComponentWrapper();

  bool Init();
  void Run();
  void Stop();
  void InitClient();

 private:
  static bool InitHandle();
  static void StopHandle();
  void SendAlarmInfo(const ara::core::String &portName,
                     const mdc::fm::AlarmInfo &alarmInfo);

  bool work_flag_ = false;
  std::unique_ptr<mdc::common_swc_lib::CommonSwcLibSwC> swc_ptr_ = nullptr;
  std::unique_ptr<planning::PlanningAdapter> component_ptr_ = nullptr;
  std::unique_ptr<TimerComponent> timer_ptr_ = nullptr;
};
}  // namespace iflyauto
