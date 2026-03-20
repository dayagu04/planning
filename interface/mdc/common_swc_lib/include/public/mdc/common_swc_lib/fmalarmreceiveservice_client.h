/* *
 * Class: FmAlarmReceiveService client declaration
 * Copyright (c) Shenzhen Yinwang Intelligent Technologies Co., Ltd. 2024. All rights reserved.
 * */

#ifndef MDC_FM_FMALARMRECEIVESERVICECLIENT
#define MDC_FM_FMALARMRECEIVESERVICECLIENT

#include <memory>
#include <vector>
#include <atomic>





#include "mdc/fm/fmalarmreceiveservice_proxy.h"

namespace mdc {
namespace fm {

using ReportAlarmOutput = mdc::fm::proxy::methods::ReportAlarm::Output;
class FmAlarmReceiveServiceClient final {
public:
    FmAlarmReceiveServiceClient() = delete;
    explicit FmAlarmReceiveServiceClient(const uint32_t instanceId);
    virtual ~FmAlarmReceiveServiceClient();
    bool Init();
    void Stop();

    

    uint32_t GetRecvQSize() const;
    bool IsStop() const;
    uint32_t GetInstanceId() const;

    /* method relative */
    bool ReportAlarm(const mdc::fm::AlarmInfo& alarmMsg);
    ReportAlarmOutput GetReportAlarmData();
    
private:
    class Impl;
    std::unique_ptr<Impl> pImpl_;
};
} /* namespace fm */
} /* namespace mdc */

#endif