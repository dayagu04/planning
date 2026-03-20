/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2026. All rights reserved.
 */

#ifndef ARA_SM_TRIGGERINOUT_PROXY_H
#define ARA_SM_TRIGGERINOUT_PROXY_H

#include "ara/com/internal/proxy/proxy_adapter.h"
#include "ara/com/internal/proxy/event_adapter.h"
#include "ara/com/internal/proxy/field_adapter.h"
#include "ara/com/internal/proxy/method_adapter.h"
#include "ara/com/crc_verification.h"
#include "ara/sm/triggerinout_common.h"
#include <string>

namespace ara {
namespace sm {
namespace proxy {
namespace events {
}

namespace fields {
    using Notifier = ara::com::internal::proxy::field::FieldAdapter<::ara::sm::TriggerDataType>;
    using Trigger = ara::com::internal::proxy::field::FieldAdapter<::ara::sm::TriggerDataType>;
    static constexpr ara::com::internal::EntityId TriggerInOutNotifierId = 54564U; //Notifier_field_hash
    static constexpr ara::com::internal::EntityId TriggerInOutNotifierGetterId = 21625U; //Notifier_getter_hash
    static constexpr ara::com::internal::EntityId TriggerInOutTriggerId = 24507U; //Trigger_field_hash
    static constexpr ara::com::internal::EntityId TriggerInOutTriggerSetterId = 57439U; //Trigger_setter_hash
}

namespace methods {
static constexpr ara::com::internal::EntityId TriggerInOutAcquireFunctionGroupInfoId = 36548U; //AcquireFunctionGroupInfo_method_hash
static constexpr ara::com::internal::EntityId TriggerInOutProcessSyncRequestId = 8148U; //ProcessSyncRequest_method_hash
static constexpr ara::com::internal::EntityId TriggerInOutResetSystemId = 43154U; //ResetSystem_method_hash
static constexpr ara::com::internal::EntityId TriggerInOutProcessAsyncRequestId = 477U; //ProcessAsyncRequest_method_hash


class AcquireFunctionGroupInfo {
public:
    using Output = ara::sm::methods::AcquireFunctionGroupInfo::Output;

    AcquireFunctionGroupInfo(const std::shared_ptr<rtf::cm::vcc::Proxy>& proxy, const ara::com::internal::EntityId entityId, const rtf::cm::type::ServiceInterfaceInfo& serviceInterfaceInfo)
        : method_(std::make_shared<ara::com::internal::proxy::method::MethodAdapter<Output>>(proxy, entityId, serviceInterfaceInfo)) {}

    ara::core::Future<Output> operator()()
    {
        return method_->operator()();
    }

    ara::com::e2e::SMState GetSMState() const noexcept
    {
        return method_->GetSMState();
    }

    /* Do not use such internal interfaces!!! */
    std::shared_ptr<ara::com::internal::proxy::method::MethodAdapter<Output>> GetMethodAdapter() const noexcept
    {
        return method_;
    }
private:
    std::shared_ptr<ara::com::internal::proxy::method::MethodAdapter<Output>> method_;
};

class ProcessSyncRequest {
public:
    using Output = void;

    ProcessSyncRequest(const std::shared_ptr<rtf::cm::vcc::Proxy>& proxy, const ara::com::internal::EntityId entityId, const rtf::cm::type::ServiceInterfaceInfo& serviceInterfaceInfo)
        : method_(std::make_shared<ara::com::internal::proxy::method::MethodAdapter<Output, ::ara::sm::StateTransitionVec>>(proxy, entityId, serviceInterfaceInfo)) {}

    ara::core::Future<void> operator()(const ::ara::sm::StateTransitionVec& stateTrans)
    {
        return method_->operator()(stateTrans);
    }

    ara::com::e2e::SMState GetSMState() const noexcept
    {
        return method_->GetSMState();
    }

    /* Do not use such internal interfaces!!! */
    std::shared_ptr<ara::com::internal::proxy::method::MethodAdapter<Output, ::ara::sm::StateTransitionVec>> GetMethodAdapter() const noexcept
    {
        return method_;
    }
private:
    std::shared_ptr<ara::com::internal::proxy::method::MethodAdapter<Output, ::ara::sm::StateTransitionVec>> method_;
};

class ResetSystem {
public:
    using Output = void;

    ResetSystem(const std::shared_ptr<rtf::cm::vcc::Proxy>& proxy, const ara::com::internal::EntityId entityId, const rtf::cm::type::ServiceInterfaceInfo& serviceInterfaceInfo)
        : method_(std::make_shared<ara::com::internal::proxy::method::MethodAdapter<Output, ::ara::sm::ResetCode, ::String, ::ara::sm::ResetCause>>(proxy, entityId, serviceInterfaceInfo)) {}

    ara::core::Future<void> operator()(const ::ara::sm::ResetCode& resetParams, const ::String& user, const ::ara::sm::ResetCause& resetReason)
    {
        return method_->operator()(resetParams, user, resetReason);
    }

    ara::com::e2e::SMState GetSMState() const noexcept
    {
        return method_->GetSMState();
    }

    /* Do not use such internal interfaces!!! */
    std::shared_ptr<ara::com::internal::proxy::method::MethodAdapter<Output, ::ara::sm::ResetCode, ::String, ::ara::sm::ResetCause>> GetMethodAdapter() const noexcept
    {
        return method_;
    }
private:
    std::shared_ptr<ara::com::internal::proxy::method::MethodAdapter<Output, ::ara::sm::ResetCode, ::String, ::ara::sm::ResetCause>> method_;
};

class ProcessAsyncRequest {
public:
    using Output = void;

    ProcessAsyncRequest(const std::shared_ptr<rtf::cm::vcc::Proxy>& proxy, const ara::com::internal::EntityId entityId, const rtf::cm::type::ServiceInterfaceInfo& serviceInterfaceInfo)
        : method_(std::make_shared<ara::com::internal::proxy::method::MethodAdapter<Output, ::ara::sm::StateTransitionVec>>(proxy, entityId, serviceInterfaceInfo)) {}

    void operator()(const ::ara::sm::StateTransitionVec& stateTrans)
    {
        method_->operator()(stateTrans);
    }

    ara::com::e2e::SMState GetSMState() const noexcept
    {
        return method_->GetSMState();
    }

    /* Do not use such internal interfaces!!! */
    std::shared_ptr<ara::com::internal::proxy::method::MethodAdapter<Output, ::ara::sm::StateTransitionVec>> GetMethodAdapter() const noexcept
    {
        return method_;
    }
private:
    std::shared_ptr<ara::com::internal::proxy::method::MethodAdapter<Output, ::ara::sm::StateTransitionVec>> method_;
};
} // namespace methods

class TriggerInOutProxy {
private:
    std::shared_ptr<ara::com::internal::proxy::ProxyAdapter> proxyAdapter;
public:
    using HandleType = ara::com::HandleType;
    class ConstructionToken {
    public:
        explicit ConstructionToken(const std::shared_ptr<ara::com::internal::proxy::ProxyAdapter> & proxy)
            : proxyAdapter(proxy),
              AcquireFunctionGroupInfo(proxyAdapter->GetProxy(), methods::TriggerInOutAcquireFunctionGroupInfoId, proxyAdapter->GetServiceInterfaceInfo()),
              ProcessSyncRequest(proxyAdapter->GetProxy(), methods::TriggerInOutProcessSyncRequestId, proxyAdapter->GetServiceInterfaceInfo()),
              ResetSystem(proxyAdapter->GetProxy(), methods::TriggerInOutResetSystemId, proxyAdapter->GetServiceInterfaceInfo()),
              ProcessAsyncRequest(proxyAdapter->GetProxy(), methods::TriggerInOutProcessAsyncRequestId, proxyAdapter->GetServiceInterfaceInfo()),
              Notifier(proxyAdapter->GetProxy(),
              ara::com::internal::proxy::field::FieldAdapter<::ara::sm::TriggerDataType>::FieldEntityIds{
              fields::TriggerInOutNotifierId, ara::com::internal::UNDEFINED_ENTITYID, fields::TriggerInOutNotifierGetterId},
              proxyAdapter->GetHandle(), proxyAdapter->GetServiceInterfaceInfo()),
              Trigger(proxyAdapter->GetProxy(),
              ara::com::internal::proxy::field::FieldAdapter<::ara::sm::TriggerDataType>::FieldEntityIds{
              fields::TriggerInOutTriggerId, fields::TriggerInOutTriggerSetterId, ara::com::internal::UNDEFINED_ENTITYID},
              proxyAdapter->GetHandle(), proxyAdapter->GetServiceInterfaceInfo()) {
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
            initResult = AcquireFunctionGroupInfo.GetMethodAdapter()->Initialize();
            if (!initResult.HasValue()) {
                return initResult;
            }
            initResult = ProcessSyncRequest.GetMethodAdapter()->Initialize();
            if (!initResult.HasValue()) {
                return initResult;
            }
            initResult = ResetSystem.GetMethodAdapter()->Initialize();
            if (!initResult.HasValue()) {
                return initResult;
            }
            initResult = ProcessAsyncRequest.GetMethodAdapter()->Initialize();
            if (!initResult.HasValue()) {
                return initResult;
            }
            initResult = Notifier.Initialize();
            if (!initResult.HasValue()) {
                return initResult;
            }
            initResult = Trigger.Initialize();
            if (!initResult.HasValue()) {
                return initResult;
            }
            return initResult;
        }
        std::shared_ptr<ara::com::internal::proxy::ProxyAdapter> GetProxyAdapter()
        {
            return proxyAdapter;
        }
        methods::AcquireFunctionGroupInfo GetAcquireFunctionGroupInfo() noexcept { return std::move(AcquireFunctionGroupInfo); }
        methods::ProcessSyncRequest GetProcessSyncRequest() noexcept { return std::move(ProcessSyncRequest); }
        methods::ResetSystem GetResetSystem() noexcept { return std::move(ResetSystem); }
        methods::ProcessAsyncRequest GetProcessAsyncRequest() noexcept { return std::move(ProcessAsyncRequest); }
        fields::Notifier GetNotifier() noexcept { return std::move(Notifier); }
        fields::Trigger GetTrigger() noexcept { return std::move(Trigger); }
    private:
        std::shared_ptr<ara::com::internal::proxy::ProxyAdapter> proxyAdapter;
        methods::AcquireFunctionGroupInfo AcquireFunctionGroupInfo;
        methods::ProcessSyncRequest ProcessSyncRequest;
        methods::ResetSystem ResetSystem;
        methods::ProcessAsyncRequest ProcessAsyncRequest;
        fields::Notifier Notifier;
        fields::Trigger Trigger;
    };

    virtual ~TriggerInOutProxy()
    {

        Notifier.UnsetReceiveHandler();
        Notifier.Unsubscribe();
        Trigger.UnsetReceiveHandler();
        Trigger.Unsubscribe();
    }

    explicit TriggerInOutProxy(const HandleType &handle)
        : proxyAdapter(std::make_shared<ara::com::internal::proxy::ProxyAdapter>(::ara::sm::TriggerInOut::ServiceIdentifier, handle)),
          AcquireFunctionGroupInfo(proxyAdapter->GetProxy(), methods::TriggerInOutAcquireFunctionGroupInfoId, proxyAdapter->GetServiceInterfaceInfo()),
          ProcessSyncRequest(proxyAdapter->GetProxy(), methods::TriggerInOutProcessSyncRequestId, proxyAdapter->GetServiceInterfaceInfo()),
          ResetSystem(proxyAdapter->GetProxy(), methods::TriggerInOutResetSystemId, proxyAdapter->GetServiceInterfaceInfo()),
          ProcessAsyncRequest(proxyAdapter->GetProxy(), methods::TriggerInOutProcessAsyncRequestId, proxyAdapter->GetServiceInterfaceInfo()),
          Notifier(proxyAdapter->GetProxy(), ara::com::internal::proxy::field::FieldAdapter<::ara::sm::TriggerDataType>::FieldEntityIds{
          fields::TriggerInOutNotifierId, ara::com::internal::UNDEFINED_ENTITYID, fields::TriggerInOutNotifierGetterId},
          proxyAdapter->GetHandle(), proxyAdapter->GetServiceInterfaceInfo()),
          Trigger(proxyAdapter->GetProxy(), ara::com::internal::proxy::field::FieldAdapter<::ara::sm::TriggerDataType>::FieldEntityIds{
          fields::TriggerInOutTriggerId, fields::TriggerInOutTriggerSetterId, ara::com::internal::UNDEFINED_ENTITYID},
          proxyAdapter->GetHandle(), proxyAdapter->GetServiceInterfaceInfo()) {
            ara::core::Result<void> resultAcquireFunctionGroupInfo = AcquireFunctionGroupInfo.GetMethodAdapter()->Initialize();
            ThrowError(resultAcquireFunctionGroupInfo);
            ara::core::Result<void> resultProcessSyncRequest = ProcessSyncRequest.GetMethodAdapter()->Initialize();
            ThrowError(resultProcessSyncRequest);
            ara::core::Result<void> resultResetSystem = ResetSystem.GetMethodAdapter()->Initialize();
            ThrowError(resultResetSystem);
            ara::core::Result<void> resultProcessAsyncRequest = ProcessAsyncRequest.GetMethodAdapter()->Initialize();
            ThrowError(resultProcessAsyncRequest);
            ara::core::Result<void> resultNotifier = Notifier.Initialize();
            ThrowError(resultNotifier);
            ara::core::Result<void> resultTrigger = Trigger.Initialize();
            ThrowError(resultTrigger);
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

    TriggerInOutProxy(const TriggerInOutProxy&) = delete;
    TriggerInOutProxy& operator=(const TriggerInOutProxy&) = delete;

    TriggerInOutProxy(TriggerInOutProxy&&) = default;
    TriggerInOutProxy& operator=(TriggerInOutProxy&&) = default;
    TriggerInOutProxy(ConstructionToken&& token) noexcept
        : proxyAdapter(token.GetProxyAdapter()),
          AcquireFunctionGroupInfo(token.GetAcquireFunctionGroupInfo()),
          ProcessSyncRequest(token.GetProcessSyncRequest()),
          ResetSystem(token.GetResetSystem()),
          ProcessAsyncRequest(token.GetProcessAsyncRequest()),
          Notifier(token.GetNotifier()),
          Trigger(token.GetTrigger()) {
        static_cast<void>(std::move(token));
    }

    static ara::core::Result<ConstructionToken> Preconstruct(
        const ara::com::HandleType &handle)
    {
        std::shared_ptr<ara::com::internal::proxy::ProxyAdapter> preProxyAdapter =
            std::make_shared<ara::com::internal::proxy::ProxyAdapter>(
               ::ara::sm::TriggerInOut::ServiceIdentifier, handle);
        ConstructionToken token{std::move(preProxyAdapter)};
        ara::core::Result<void> initResult {token.InitializeAll()};
        if (initResult.HasValue()) {
            return ara::core::Result<ConstructionToken>(std::move(token));
        } else {
            const ara::core::ErrorCode errorcode{initResult.Error()};
            ara::core::Result<TriggerInOutProxy::ConstructionToken> preResult{errorcode};
            return preResult;
        }
    }

    static ara::com::FindServiceHandle StartFindService(
        const ara::com::FindServiceHandler<ara::com::internal::proxy::ProxyAdapter::HandleType>& handler,
        const ara::com::InstanceIdentifier instance)
    {
        return ara::com::internal::proxy::ProxyAdapter::StartFindService(handler, ::ara::sm::TriggerInOut::ServiceIdentifier, instance);
    }

    static ara::com::FindServiceHandle StartFindService(
        const ara::com::FindServiceHandler<ara::com::internal::proxy::ProxyAdapter::HandleType> handler,
        const ara::core::InstanceSpecifier specifier)
    {
        return ara::com::internal::proxy::ProxyAdapter::StartFindService(handler, ::ara::sm::TriggerInOut::ServiceIdentifier, specifier);
    }

    static ara::com::ServiceHandleContainer<ara::com::internal::proxy::ProxyAdapter::HandleType> FindService(
        const ara::com::InstanceIdentifier instance)
    {
        return ara::com::internal::proxy::ProxyAdapter::FindService(::ara::sm::TriggerInOut::ServiceIdentifier, instance);
    }

    static ara::com::ServiceHandleContainer<ara::com::internal::proxy::ProxyAdapter::HandleType> FindService(
        const ara::core::InstanceSpecifier specifier)
    {
        return ara::com::internal::proxy::ProxyAdapter::FindService(::ara::sm::TriggerInOut::ServiceIdentifier, specifier);
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
    methods::AcquireFunctionGroupInfo AcquireFunctionGroupInfo;
    methods::ProcessSyncRequest ProcessSyncRequest;
    methods::ResetSystem ResetSystem;
    methods::ProcessAsyncRequest ProcessAsyncRequest;
    fields::Notifier Notifier;
    fields::Trigger Trigger;
};
} // namespace proxy
} // namespace sm
} // namespace ara

#endif // ARA_SM_TRIGGERINOUT_PROXY_H
