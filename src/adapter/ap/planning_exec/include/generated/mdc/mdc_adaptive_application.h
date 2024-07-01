/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2021-2022. All rights reserved.
 * Description: MDC平台AP标准框架
 */
#ifndef MDC_ADAPTIVE_APPLICATION_H
#define MDC_ADAPTIVE_APPLICATION_H

#include <atomic>
#include "ara/exec/execution_client.h"
#include "mdc/applicationswcexecutable2_logger.h"

namespace mdc {
namespace applicationswcexecutable2 {
const int32_t SELF_PIPE_NUM = 2;
const int32_t SELF_PIPE_READ = 0;
const int32_t SELF_PIPE_WRITE = 0;

using LOG_SPACE =
    mdc::applicationswcexecutable2::Applicationswcexecutable2Logger;

class MdcAdaptiveApplication {
 public:
  MdcAdaptiveApplication() = default;
  MdcAdaptiveApplication(const MdcAdaptiveApplication&) = delete;
  MdcAdaptiveApplication& operator=(const MdcAdaptiveApplication&) = delete;
  MdcAdaptiveApplication(MdcAdaptiveApplication&&) = delete;
  MdcAdaptiveApplication& operator=(MdcAdaptiveApplication&&) = delete;
  virtual ~MdcAdaptiveApplication() = default;
  int32_t Execute();

 protected:
  /* 子类完成初始化函数重载 */
  virtual bool OnInitialize() = 0;
  /* 子类完成运行函数重载 */
  virtual void Run() = 0;
  /* 子类完成终止函数重载 */
  virtual void OnTerminate() = 0;
  /* 信号量等待函数 */
  static void WaitUntilTerm();

 private:
  /* EM模块 */
  ara::exec::ExecutionClient execClient_;
  /* 初始化函数 */
  bool Initialize();
  /* 终止函数 */
  void Terminate();
  /* 管道相关变量及处理函数 */
  static void CloseSelfPipe();
  static constexpr int32_t PipeReadingEnd = 0;
  static constexpr int32_t PipeWritingEnd = 1;
  static int32_t selfpipe[SELF_PIPE_NUM];
  /* bool类型信号量 */
  static std::atomic_bool exit_req_sigterm_;
  static std::atomic_bool exit_req_sigint_;
  /* 信号量处理函数 */
  static void SigTermHandler(int32_t sig);
  static void SigIntHandler(int32_t sig);
};
} /* namespace applicationswcexecutable2 */
} /* namespace mdc */
#endif /* MDC_ADAPTIVE_APPLICATION_H */