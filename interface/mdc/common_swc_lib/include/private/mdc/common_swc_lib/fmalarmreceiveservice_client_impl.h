/* *
 * Class: FmAlarmReceiveService client declaration
 * Copyright (c) Shenzhen Yinwang Intelligent Technologies Co., Ltd. 2024. All rights reserved.
 * */

#ifndef MDC_FM_FMALARMRECEIVESERVICECLIENT_IMPL
#define MDC_FM_FMALARMRECEIVESERVICECLIENT_IMPL

#include <memory>
#include <vector>
#include <atomic>
#include "mdc/fm/fmalarmreceiveservice_proxy.h"
#include "ara/exec/execution_client.h"
#include "mdc/common_swc_lib_logger.h"
namespace mdc {
namespace fm {
using FmAlarmReceiveServiceProxy = mdc::fm::proxy::FmAlarmReceiveServiceProxy;

using ReportAlarmOutput = mdc::fm::proxy::methods::ReportAlarm::Output;
namespace {
using LOG_SPACE = mdc::common_swc_lib::CommonSwcLibLogger;
}

class FmAlarmReceiveServiceClientImpl {
public:
    explicit FmAlarmReceiveServiceClientImpl(const uint32_t instanceId);
    virtual ~FmAlarmReceiveServiceClientImpl();
    bool Init();
    void Stop();

    
    uint32_t GetRecvQSize() const
    {
        return recvQSize_;
    }

    bool IsStop() const
    {
        return !workFlag_;
    }

    uint32_t GetInstanceId() const
    {
        return instanceIdx_;
    }
    /* method relative */
    bool ReportAlarm(const mdc::fm::AlarmInfo& alarmMsg);
    ReportAlarmOutput GetReportAlarmData();
    
private:
    /* 服务实例ID */
    uint32_t instanceIdx_;

    /* 服务标识 */
    std::atomic<bool> workFlag_;

    /* 寻找服务标识 */
    std::atomic<bool> findServFlag_;

    /* 注册标识 */
    std::atomic<bool> registerFlag_;

    /* EM模块 */
    ara::exec::ExecutionClient execClient_ {};

    /* EM 上报标识 */
    std::atomic<bool> emReport_;

    /* 服务发现回调 避免多线程同时执行标识 */
    std::once_flag callFlag_{};

    void FmAlarmReceiveServiceCallback(ara::com::ServiceHandleContainer<FmAlarmReceiveServiceProxy::HandleType> handles,
        const ara::com::FindServiceHandle findServiceHandle);
    std::unique_ptr<FmAlarmReceiveServiceProxy> proxyPtr_{nullptr};
    uint32_t recvQSize_{15U};
    ara::com::FindServiceHandle serviceHandle_{};
    void EmReportExec();

    ReportAlarmOutput ReportAlarmOutputRes_{};
    
};
} /* namespace fm */
} /* namespace mdc */
#endif