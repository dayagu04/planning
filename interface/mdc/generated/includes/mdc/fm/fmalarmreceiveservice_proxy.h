/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2026. All rights reserved.
 */

#ifndef MDC_FM_FMALARMRECEIVESERVICE_PROXY_H
#define MDC_FM_FMALARMRECEIVESERVICE_PROXY_H

#include "ara/com/internal/proxy/proxy_adapter.h"
#include "ara/com/internal/proxy/event_adapter.h"
#include "ara/com/internal/proxy/field_adapter.h"
#include "ara/com/internal/proxy/method_adapter.h"
#include "ara/com/crc_verification.h"
#include "mdc/fm/fmalarmreceiveservice_common.h"
#include <string>

namespace mdc {
namespace fm {
namespace proxy {
namespace events {
}

namespace fields {
}

namespace methods {
static constexpr ara::com::internal::EntityId FmAlarmReceiveServiceReportAlarmId = 30845U; //ReportAlarm_method_hash


class ReportAlarm {
public:
    using Output = mdc::fm::methods::ReportAlarm::Output;

    ReportAlarm(const std::shared_ptr<rtf::cm::vcc::Proxy>& proxy, const ara::com::internal::EntityId entityId, const rtf::cm::type::ServiceInterfaceInfo& serviceInterfaceInfo)
        : method_(std::make_shared<ara::com::internal::proxy::method::MethodAdapter<Output, ::mdc::fm::AlarmInfo>>(proxy, entityId, serviceInterfaceInfo)) {}

    ara::core::Future<Output> operator()(const ::mdc::fm::AlarmInfo& alarmMsg)
    {
        return method_->operator()(alarmMsg);
    }

    ara::com::e2e::SMState GetSMState() const noexcept
    {
        return method_->GetSMState();
    }

    /* Do not use such internal interfaces!!! */
    std::shared_ptr<ara::com::internal::proxy::method::MethodAdapter<Output, ::mdc::fm::AlarmInfo>> GetMethodAdapter() const noexcept
    {
        return method_;
    }
private:
    std::shared_ptr<ara::com::internal::proxy::method::MethodAdapter<Output, ::mdc::fm::AlarmInfo>> method_;
};
} // namespace methods

class FmAlarmReceiveServiceProxy {
private:
    std::shared_ptr<ara::com::internal::proxy::ProxyAdapter> proxyAdapter;
public:
    using HandleType = ara::com::HandleType;
    class ConstructionToken {
    public:
        explicit ConstructionToken(const std::shared_ptr<ara::com::internal::proxy::ProxyAdapter> & proxy)
            : proxyAdapter(proxy),
              ReportAlarm(proxyAdapter->GetProxy(), methods::FmAlarmReceiveServiceReportAlarmId, proxyAdapter->GetServiceInterfaceInfo()){
        }

        ConstructionToken(ConstructionToken&& other) = default;
        ConstructionToken& operator=(ConstructionToken && other) = default;

        ConstructionToken(const ConstructionToken&) = delete;
        ConstructionToken& operator = (const ConstructionToken&) = delete;
        
        virtual ~ConstructionToken() = default;

        /* Do not use such internal interfaces!!! */
        ara::core::Result<void> InitializeAll()
        {
            ara::core::Result<void> initResult;
            initResult = ReportAlarm.GetMethodAdapter()->Initialize();
            if (!initResult.HasValue()) {
                return initResult;
            }
            return initResult;
        }
        std::shared_ptr<ara::com::internal::proxy::ProxyAdapter> GetProxyAdapter()
        {
            return proxyAdapter;
        }
        methods::ReportAlarm GetReportAlarm() noexcept { return std::move(ReportAlarm); }
    private:
        std::shared_ptr<ara::com::internal::proxy::ProxyAdapter> proxyAdapter;
        methods::ReportAlarm ReportAlarm;
    };

    virtual ~FmAlarmReceiveServiceProxy()
    {

    }

    explicit FmAlarmReceiveServiceProxy(const HandleType &handle)
        : proxyAdapter(std::make_shared<ara::com::internal::proxy::ProxyAdapter>(::mdc::fm::FmAlarmReceiveService::ServiceIdentifier, handle)),
          ReportAlarm(proxyAdapter->GetProxy(), methods::FmAlarmReceiveServiceReportAlarmId, proxyAdapter->GetServiceInterfaceInfo()){
            ara::core::Result<void> resultReportAlarm = ReportAlarm.GetMethodAdapter()->Initialize();
            ThrowError(resultReportAlarm);
        }

    void ThrowError(const ara::core::Result<void>& result) const
    {
        if (!(result.HasValue())) {
#ifndef NOT_SUPPORT_EXCEPTIONS
            ara::core::ErrorCode errorcode(result.Error());
            throw ara::com::ComException(std::move(errorcode));
#else
            std::cerr << "Error: Not support exception, create proxy failed!"<< std::endl;
#endif
        }
    }

    FmAlarmReceiveServiceProxy(const FmAlarmReceiveServiceProxy&) = delete;
    FmAlarmReceiveServiceProxy& operator=(const FmAlarmReceiveServiceProxy&) = delete;

    FmAlarmReceiveServiceProxy(FmAlarmReceiveServiceProxy&&) = default;
    FmAlarmReceiveServiceProxy& operator=(FmAlarmReceiveServiceProxy&&) = default;
    FmAlarmReceiveServiceProxy(ConstructionToken&& token) noexcept
        : proxyAdapter(token.GetProxyAdapter()),
          ReportAlarm(token.GetReportAlarm()){
        static_cast<void>(std::move(token));
    }

    static ara::core::Result<ConstructionToken> Preconstruct(
        const ara::com::HandleType &handle)
    {
        std::shared_ptr<ara::com::internal::proxy::ProxyAdapter> preProxyAdapter =
            std::make_shared<ara::com::internal::proxy::ProxyAdapter>(
               ::mdc::fm::FmAlarmReceiveService::ServiceIdentifier, handle);
        ConstructionToken token{std::move(preProxyAdapter)};
        ara::core::Result<void> initResult {token.InitializeAll()};
        if (initResult.HasValue()) {
            return ara::core::Result<ConstructionToken>(std::move(token));
        } else {
            const ara::core::ErrorCode errorcode{initResult.Error()};
            ara::core::Result<FmAlarmReceiveServiceProxy::ConstructionToken> preResult{errorcode};
            return preResult;
        }
    }

    static ara::com::FindServiceHandle StartFindService(
        const ara::com::FindServiceHandler<ara::com::internal::proxy::ProxyAdapter::HandleType>& handler,
        const ara::com::InstanceIdentifier instance)
    {
        return ara::com::internal::proxy::ProxyAdapter::StartFindService(handler, ::mdc::fm::FmAlarmReceiveService::ServiceIdentifier, instance);
    }

    static ara::com::FindServiceHandle StartFindService(
        const ara::com::FindServiceHandler<ara::com::internal::proxy::ProxyAdapter::HandleType> handler,
        const ara::core::InstanceSpecifier specifier)
    {
        return ara::com::internal::proxy::ProxyAdapter::StartFindService(handler, ::mdc::fm::FmAlarmReceiveService::ServiceIdentifier, specifier);
    }

    static ara::com::ServiceHandleContainer<ara::com::internal::proxy::ProxyAdapter::HandleType> FindService(
        const ara::com::InstanceIdentifier instance)
    {
        return ara::com::internal::proxy::ProxyAdapter::FindService(::mdc::fm::FmAlarmReceiveService::ServiceIdentifier, instance);
    }

    static ara::com::ServiceHandleContainer<ara::com::internal::proxy::ProxyAdapter::HandleType> FindService(
        const ara::core::InstanceSpecifier specifier)
    {
        return ara::com::internal::proxy::ProxyAdapter::FindService(::mdc::fm::FmAlarmReceiveService::ServiceIdentifier, specifier);
    }

    static void StopFindService(const ara::com::FindServiceHandle& handle)
    {
        ara::com::internal::proxy::ProxyAdapter::StopFindService(handle);
    }

    HandleType GetHandle() const
    {
        return proxyAdapter->GetHandle();
    }
    bool SetEventThreadNumber(const std::uint16_t number, const std::uint16_t queueSize)
    {
        return proxyAdapter->SetEventThreadNumber(number, queueSize);
    }
    methods::ReportAlarm ReportAlarm;
};
} // namespace proxy
} // namespace fm
} // namespace mdc

#endif // MDC_FM_FMALARMRECEIVESERVICE_PROXY_H
