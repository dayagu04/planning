/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2021-2022. All rights reserved.
 * Description: MDC平台AP标准框架
 */
#include "mdc/mdc_adaptive_application.h"
#include <unistd.h>
#include <atomic>
#include <csignal>
#include <cstdint>
#include <ctime>
#include <iostream>

namespace mdc {
namespace applicationswcexecutable2 {
int32_t MdcAdaptiveApplication::selfpipe[SELF_PIPE_NUM] = {SELF_PIPE_READ,
                                                           SELF_PIPE_WRITE};

std::atomic_bool MdcAdaptiveApplication::exit_req_sigterm_(false);
std::atomic_bool MdcAdaptiveApplication::exit_req_sigint_(false);

void MdcAdaptiveApplication::SigTermHandler(int32_t sig) {
  if (sig == SIGTERM) {
    exit_req_sigterm_ = true;
    static_cast<void>(write(selfpipe[PipeWritingEnd], "\0", 1U));
  }
}
void MdcAdaptiveApplication::SigIntHandler(int32_t sig) {
  if (sig == SIGINT) {
    exit_req_sigint_ = true;
    static_cast<void>(write(selfpipe[PipeWritingEnd], "\0", 1U));
  }
}
void MdcAdaptiveApplication::WaitUntilTerm() {
  while ((!exit_req_sigterm_) && (!exit_req_sigint_)) {
    int32_t buf;
    const ssize_t readRes = read(selfpipe[PipeReadingEnd], &buf, sizeof(buf));
    static_cast<void>(readRes);
  }
}
void MdcAdaptiveApplication::CloseSelfPipe() {
  static_cast<void>(close(selfpipe[PipeReadingEnd]));
  static_cast<void>(close(selfpipe[PipeWritingEnd]));
}

int32_t MdcAdaptiveApplication::Execute() {
  /* 初始化流程 */
  if (Initialize()) {
    LOG_SPACE::GetLoggerIns("MAA")->LogInfo()
        << "Mdc Adaptive Application start...";
    /* 状态上报: 向EM上报kRunning状态 */
    ara::core::Result<void> res =
        execClient_.ReportExecutionState(ara::exec::ExecutionState::kRunning);
    if (res.HasValue() == false) {
      static_cast<void>(sleep(1U));
      static_cast<void>(execClient_.ReportExecutionState(
          ara::exec::ExecutionState::kRunning));
    }
    /* 调用运行流: 完成线程以及服务启动等动作 */
    Run();
    LOG_SPACE::GetLoggerIns("MAA")->LogInfo()
        << "Mdc Adaptive Application Start Terminate";
    /* 终止等待函数: 根据信号量判断是否终止主进程 */
    Terminate();
    LOG_SPACE::GetLoggerIns("MAA")->LogInfo()
        << "Mdc Adaptive Application Terminated";
    return EXIT_SUCCESS;
  }
  LOG_SPACE::GetLoggerIns("MAA")->LogError()
      << "Mdc Adaptive Application failed to initialize!";
  return EXIT_FAILURE;
}

bool MdcAdaptiveApplication::Initialize() {
  LOG_SPACE::InitLoggerCtx("MAA", "Adaptive Application general context");
  /* 调用子类重写的初始化函数 */
  if (!OnInitialize()) {
    return false;
  }
  /* 初始化管道相关变量 */
  if (pipe(selfpipe) == -1) {
    LOG_SPACE::GetLoggerIns("MAA")->LogError()
        << "Execution stopped. Unable to create a pipe";
    return false;
  }
  struct sigaction sa_term;
  sa_term.sa_handler = SigTermHandler;
  sa_term.sa_flags = 0;
  static_cast<void>(sigemptyset(&sa_term.sa_mask));
  if (sigaction(SIGTERM, &sa_term, NULL) == -1) {
    LOG_SPACE::GetLoggerIns("MAA")->LogError()
        << "Execution stopped. Unable to register sigterm handler";
    CloseSelfPipe();
    return false;
  }
  struct sigaction sa_int;
  sa_int.sa_handler = SigIntHandler;
  sa_int.sa_flags = 0;
  static_cast<void>(sigemptyset(&sa_int.sa_mask));
  if (sigaction(SIGINT, &sa_int, NULL) == -1) {
    LOG_SPACE::GetLoggerIns("MAA")->LogError()
        << "Execution stopped. Unable to register sigint handler";
    CloseSelfPipe();
    return false;
  }
  return true;
}

void MdcAdaptiveApplication::Terminate() {
  /* 等待信号量决定程序是否结束 */
  WaitUntilTerm();
  /* 状态上报: 向EM上报kTerminating状态 */
  static_cast<void>(execClient_.ReportExecutionState(
      ara::exec::ExecutionState::kTerminating));
  /* 调用子类重载终止函数完成所有退出动作 */
  OnTerminate();
  return;
} /* namespace applicationswcexecutable2 */
}  // namespace applicationswcexecutable2
}  // namespace mdc