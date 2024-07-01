/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2021-2022. All rights reserved.
 * Description: MDC平台AP标准框架
 */
#ifndef ARA_APPLICATIONSWCEXECUTABLE2_APPLICATIONSWCEXECUTABLE2_MANAGER_H
#define ARA_APPLICATIONSWCEXECUTABLE2_APPLICATIONSWCEXECUTABLE2_MANAGER_H

#include <atomic>
#include <iostream>
#include <memory>
#include <thread>
#include "ara/core/string.h"
#include "component_wrapper.h"
#include "mdc/applicationswcexecutable2_logger.h"
#include "mdc/common_swc_lib/common_swc_lib_swc.h"
#include "mdc/common_swc_lib/structcontainerinterface_client.h"
#include "mdc/common_swc_lib/structcontainerinterface_server.h"
#include "mdc/mdc_adaptive_application.h"
namespace mdc {
namespace applicationswcexecutable2 {
class Applicationswcexecutable2Manager : public MdcAdaptiveApplication {
 public:
  /* 若需传入参数，请手动添加 */
  Applicationswcexecutable2Manager() = default;
  virtual ~Applicationswcexecutable2Manager() = default;

 protected:
  virtual bool OnInitialize() override;
  virtual void Run() override;
  virtual void OnTerminate() override;

 private:
  std::unique_ptr<iflyauto::ComponentWrapper> componentPtr_;
};
} /* namespace applicationswcexecutable2 */
} /* namespace mdc */
#endif /* ARA_APPLICATIONSWCEXECUTABLE2_APPLICATIONSWCEXECUTABLE2_MANAGER_H */