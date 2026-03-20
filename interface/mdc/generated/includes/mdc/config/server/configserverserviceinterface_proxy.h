/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2026. All rights reserved.
 */

#ifndef MDC_CONFIG_SERVER_CONFIGSERVERSERVICEINTERFACE_PROXY_H
#define MDC_CONFIG_SERVER_CONFIGSERVERSERVICEINTERFACE_PROXY_H

#include "ara/com/internal/proxy/proxy_adapter.h"
#include "ara/com/internal/proxy/event_adapter.h"
#include "ara/com/internal/proxy/field_adapter.h"
#include "ara/com/internal/proxy/method_adapter.h"
#include "ara/com/crc_verification.h"
#include "mdc/config/server/configserverserviceinterface_common.h"
#include <string>

namespace mdc {
namespace config {
namespace server {
namespace proxy {
namespace events {
    using ParamUpdateEvent = ara::com::internal::proxy::event::EventAdapter<::mdc::config::server::ParamUpdateData>;
    using ServerNotifyEvent = ara::com::internal::proxy::event::EventAdapter<::mdc::config::server::ServerNotifyData>;
    static constexpr ara::com::internal::EntityId ConfigServerServiceInterfaceParamUpdateEventId = 35207U; //ParamUpdateEvent_event_hash
    static constexpr ara::com::internal::EntityId ConfigServerServiceInterfaceServerNotifyEventId = 27155U; //ServerNotifyEvent_event_hash
}

namespace fields {
}

namespace methods {
static constexpr ara::com::internal::EntityId ConfigServerServiceInterfaceAnswerAliveId = 30329U; //AnswerAlive_method_hash
static constexpr ara::com::internal::EntityId ConfigServerServiceInterfaceDelParamId = 56227U; //DelParam_method_hash
static constexpr ara::com::internal::EntityId ConfigServerServiceInterfaceGetMonitorClientsId = 42802U; //GetMonitorClients_method_hash
static constexpr ara::com::internal::EntityId ConfigServerServiceInterfaceGetParamId = 5505U; //GetParam_method_hash
static constexpr ara::com::internal::EntityId ConfigServerServiceInterfaceMonitorParamId = 3875U; //MonitorParam_method_hash
static constexpr ara::com::internal::EntityId ConfigServerServiceInterfaceSetParamId = 31959U; //SetParam_method_hash
static constexpr ara::com::internal::EntityId ConfigServerServiceInterfaceUnMonitorParamId = 36659U; //UnMonitorParam_method_hash
static constexpr ara::com::internal::EntityId ConfigServerServiceInterfaceInitClientId = 55067U; //InitClient_method_hash


class AnswerAlive {
public:
    using Output = void;

    AnswerAlive(const std::shared_ptr<rtf::cm::vcc::Proxy>& proxy, const ara::com::internal::EntityId entityId, const rtf::cm::type::ServiceInterfaceInfo& serviceInterfaceInfo)
        : method_(std::make_shared<ara::com::internal::proxy::method::MethodAdapter<Output, ::String>>(proxy, entityId, serviceInterfaceInfo)) {}

    ara::core::Future<void> operator()(const ::String& clientName)
    {
        return method_->operator()(clientName);
    }

    ara::com::e2e::SMState GetSMState() const noexcept
    {
        return method_->GetSMState();
    }

    /* Do not use such internal interfaces!!! */
    std::shared_ptr<ara::com::internal::proxy::method::MethodAdapter<Output, ::String>> GetMethodAdapter() const noexcept
    {
        return method_;
    }
private:
    std::shared_ptr<ara::com::internal::proxy::method::MethodAdapter<Output, ::String>> method_;
};

class DelParam {
public:
    using Output = mdc::config::server::methods::DelParam::Output;

    DelParam(const std::shared_ptr<rtf::cm::vcc::Proxy>& proxy, const ara::com::internal::EntityId entityId, const rtf::cm::type::ServiceInterfaceInfo& serviceInterfaceInfo)
        : method_(std::make_shared<ara::com::internal::proxy::method::MethodAdapter<Output, ::String, ::String>>(proxy, entityId, serviceInterfaceInfo)) {}

    ara::core::Future<Output> operator()(const ::String& clientName, const ::String& paramName)
    {
        return method_->operator()(clientName, paramName);
    }

    ara::com::e2e::SMState GetSMState() const noexcept
    {
        return method_->GetSMState();
    }

    /* Do not use such internal interfaces!!! */
    std::shared_ptr<ara::com::internal::proxy::method::MethodAdapter<Output, ::String, ::String>> GetMethodAdapter() const noexcept
    {
        return method_;
    }
private:
    std::shared_ptr<ara::com::internal::proxy::method::MethodAdapter<Output, ::String, ::String>> method_;
};

class GetMonitorClients {
public:
    using Output = mdc::config::server::methods::GetMonitorClients::Output;

    GetMonitorClients(const std::shared_ptr<rtf::cm::vcc::Proxy>& proxy, const ara::com::internal::EntityId entityId, const rtf::cm::type::ServiceInterfaceInfo& serviceInterfaceInfo)
        : method_(std::make_shared<ara::com::internal::proxy::method::MethodAdapter<Output, ::String, ::String>>(proxy, entityId, serviceInterfaceInfo)) {}

    ara::core::Future<Output> operator()(const ::String& clientName, const ::String& paramName)
    {
        return method_->operator()(clientName, paramName);
    }

    ara::com::e2e::SMState GetSMState() const noexcept
    {
        return method_->GetSMState();
    }

    /* Do not use such internal interfaces!!! */
    std::shared_ptr<ara::com::internal::proxy::method::MethodAdapter<Output, ::String, ::String>> GetMethodAdapter() const noexcept
    {
        return method_;
    }
private:
    std::shared_ptr<ara::com::internal::proxy::method::MethodAdapter<Output, ::String, ::String>> method_;
};

class GetParam {
public:
    using Output = mdc::config::server::methods::GetParam::Output;

    GetParam(const std::shared_ptr<rtf::cm::vcc::Proxy>& proxy, const ara::com::internal::EntityId entityId, const rtf::cm::type::ServiceInterfaceInfo& serviceInterfaceInfo)
        : method_(std::make_shared<ara::com::internal::proxy::method::MethodAdapter<Output, ::String, ::String>>(proxy, entityId, serviceInterfaceInfo)) {}

    ara::core::Future<Output> operator()(const ::String& clientName, const ::String& paramName)
    {
        return method_->operator()(clientName, paramName);
    }

    ara::com::e2e::SMState GetSMState() const noexcept
    {
        return method_->GetSMState();
    }

    /* Do not use such internal interfaces!!! */
    std::shared_ptr<ara::com::internal::proxy::method::MethodAdapter<Output, ::String, ::String>> GetMethodAdapter() const noexcept
    {
        return method_;
    }
private:
    std::shared_ptr<ara::com::internal::proxy::method::MethodAdapter<Output, ::String, ::String>> method_;
};

class MonitorParam {
public:
    using Output = mdc::config::server::methods::MonitorParam::Output;

    MonitorParam(const std::shared_ptr<rtf::cm::vcc::Proxy>& proxy, const ara::com::internal::EntityId entityId, const rtf::cm::type::ServiceInterfaceInfo& serviceInterfaceInfo)
        : method_(std::make_shared<ara::com::internal::proxy::method::MethodAdapter<Output, ::String, ::String>>(proxy, entityId, serviceInterfaceInfo)) {}

    ara::core::Future<Output> operator()(const ::String& clientName, const ::String& paramName)
    {
        return method_->operator()(clientName, paramName);
    }

    ara::com::e2e::SMState GetSMState() const noexcept
    {
        return method_->GetSMState();
    }

    /* Do not use such internal interfaces!!! */
    std::shared_ptr<ara::com::internal::proxy::method::MethodAdapter<Output, ::String, ::String>> GetMethodAdapter() const noexcept
    {
        return method_;
    }
private:
    std::shared_ptr<ara::com::internal::proxy::method::MethodAdapter<Output, ::String, ::String>> method_;
};

class SetParam {
public:
    using Output = mdc::config::server::methods::SetParam::Output;

    SetParam(const std::shared_ptr<rtf::cm::vcc::Proxy>& proxy, const ara::com::internal::EntityId entityId, const rtf::cm::type::ServiceInterfaceInfo& serviceInterfaceInfo)
        : method_(std::make_shared<ara::com::internal::proxy::method::MethodAdapter<Output, ::String, ::String, ::String, ::UInt8, ::UInt8, ::String>>(proxy, entityId, serviceInterfaceInfo)) {}

    ara::core::Future<Output> operator()(const ::String& clientName, const ::String& paramName, const ::String& paramValue, const ::UInt8& paramType, const ::UInt8& persistType, const ::String& uuid)
    {
        return method_->operator()(clientName, paramName, paramValue, paramType, persistType, uuid);
    }

    ara::com::e2e::SMState GetSMState() const noexcept
    {
        return method_->GetSMState();
    }

    /* Do not use such internal interfaces!!! */
    std::shared_ptr<ara::com::internal::proxy::method::MethodAdapter<Output, ::String, ::String, ::String, ::UInt8, ::UInt8, ::String>> GetMethodAdapter() const noexcept
    {
        return method_;
    }
private:
    std::shared_ptr<ara::com::internal::proxy::method::MethodAdapter<Output, ::String, ::String, ::String, ::UInt8, ::UInt8, ::String>> method_;
};

class UnMonitorParam {
public:
    using Output = mdc::config::server::methods::UnMonitorParam::Output;

    UnMonitorParam(const std::shared_ptr<rtf::cm::vcc::Proxy>& proxy, const ara::com::internal::EntityId entityId, const rtf::cm::type::ServiceInterfaceInfo& serviceInterfaceInfo)
        : method_(std::make_shared<ara::com::internal::proxy::method::MethodAdapter<Output, ::String, ::String>>(proxy, entityId, serviceInterfaceInfo)) {}

    ara::core::Future<Output> operator()(const ::String& clientName, const ::String& paramName)
    {
        return method_->operator()(clientName, paramName);
    }

    ara::com::e2e::SMState GetSMState() const noexcept
    {
        return method_->GetSMState();
    }

    /* Do not use such internal interfaces!!! */
    std::shared_ptr<ara::com::internal::proxy::method::MethodAdapter<Output, ::String, ::String>> GetMethodAdapter() const noexcept
    {
        return method_;
    }
private:
    std::shared_ptr<ara::com::internal::proxy::method::MethodAdapter<Output, ::String, ::String>> method_;
};

class InitClient {
public:
    using Output = mdc::config::server::methods::InitClient::Output;

    InitClient(const std::shared_ptr<rtf::cm::vcc::Proxy>& proxy, const ara::com::internal::EntityId entityId, const rtf::cm::type::ServiceInterfaceInfo& serviceInterfaceInfo)
        : method_(std::make_shared<ara::com::internal::proxy::method::MethodAdapter<Output, ::String>>(proxy, entityId, serviceInterfaceInfo)) {}

    ara::core::Future<Output> operator()(const ::String& clientName)
    {
        return method_->operator()(clientName);
    }

    ara::com::e2e::SMState GetSMState() const noexcept
    {
        return method_->GetSMState();
    }

    /* Do not use such internal interfaces!!! */
    std::shared_ptr<ara::com::internal::proxy::method::MethodAdapter<Output, ::String>> GetMethodAdapter() const noexcept
    {
        return method_;
    }
private:
    std::shared_ptr<ara::com::internal::proxy::method::MethodAdapter<Output, ::String>> method_;
};
} // namespace methods

class ConfigServerServiceInterfaceProxy {
private:
    std::shared_ptr<ara::com::internal::proxy::ProxyAdapter> proxyAdapter;
public:
    using HandleType = ara::com::HandleType;
    class ConstructionToken {
    public:
        explicit ConstructionToken(const std::shared_ptr<ara::com::internal::proxy::ProxyAdapter> & proxy)
            : proxyAdapter(proxy),
              ParamUpdateEvent(proxyAdapter->GetProxy(), events::ConfigServerServiceInterfaceParamUpdateEventId, proxyAdapter->GetHandle(), proxyAdapter->GetServiceInterfaceInfo()),
              ServerNotifyEvent(proxyAdapter->GetProxy(), events::ConfigServerServiceInterfaceServerNotifyEventId, proxyAdapter->GetHandle(), proxyAdapter->GetServiceInterfaceInfo()),
              AnswerAlive(proxyAdapter->GetProxy(), methods::ConfigServerServiceInterfaceAnswerAliveId, proxyAdapter->GetServiceInterfaceInfo()),
              DelParam(proxyAdapter->GetProxy(), methods::ConfigServerServiceInterfaceDelParamId, proxyAdapter->GetServiceInterfaceInfo()),
              GetMonitorClients(proxyAdapter->GetProxy(), methods::ConfigServerServiceInterfaceGetMonitorClientsId, proxyAdapter->GetServiceInterfaceInfo()),
              GetParam(proxyAdapter->GetProxy(), methods::ConfigServerServiceInterfaceGetParamId, proxyAdapter->GetServiceInterfaceInfo()),
              MonitorParam(proxyAdapter->GetProxy(), methods::ConfigServerServiceInterfaceMonitorParamId, proxyAdapter->GetServiceInterfaceInfo()),
              SetParam(proxyAdapter->GetProxy(), methods::ConfigServerServiceInterfaceSetParamId, proxyAdapter->GetServiceInterfaceInfo()),
              UnMonitorParam(proxyAdapter->GetProxy(), methods::ConfigServerServiceInterfaceUnMonitorParamId, proxyAdapter->GetServiceInterfaceInfo()),
              InitClient(proxyAdapter->GetProxy(), methods::ConfigServerServiceInterfaceInitClientId, proxyAdapter->GetServiceInterfaceInfo()){
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
            initResult = AnswerAlive.GetMethodAdapter()->Initialize();
            if (!initResult.HasValue()) {
                return initResult;
            }
            initResult = DelParam.GetMethodAdapter()->Initialize();
            if (!initResult.HasValue()) {
                return initResult;
            }
            initResult = GetMonitorClients.GetMethodAdapter()->Initialize();
            if (!initResult.HasValue()) {
                return initResult;
            }
            initResult = GetParam.GetMethodAdapter()->Initialize();
            if (!initResult.HasValue()) {
                return initResult;
            }
            initResult = MonitorParam.GetMethodAdapter()->Initialize();
            if (!initResult.HasValue()) {
                return initResult;
            }
            initResult = SetParam.GetMethodAdapter()->Initialize();
            if (!initResult.HasValue()) {
                return initResult;
            }
            initResult = UnMonitorParam.GetMethodAdapter()->Initialize();
            if (!initResult.HasValue()) {
                return initResult;
            }
            initResult = InitClient.GetMethodAdapter()->Initialize();
            if (!initResult.HasValue()) {
                return initResult;
            }
            return initResult;
        }
        std::shared_ptr<ara::com::internal::proxy::ProxyAdapter> GetProxyAdapter()
        {
            return proxyAdapter;
        }
        events::ParamUpdateEvent GetParamUpdateEvent() noexcept { return std::move(ParamUpdateEvent); }
        events::ServerNotifyEvent GetServerNotifyEvent() noexcept { return std::move(ServerNotifyEvent); }
        methods::AnswerAlive GetAnswerAlive() noexcept { return std::move(AnswerAlive); }
        methods::DelParam GetDelParam() noexcept { return std::move(DelParam); }
        methods::GetMonitorClients GetGetMonitorClients() noexcept { return std::move(GetMonitorClients); }
        methods::GetParam GetGetParam() noexcept { return std::move(GetParam); }
        methods::MonitorParam GetMonitorParam() noexcept { return std::move(MonitorParam); }
        methods::SetParam GetSetParam() noexcept { return std::move(SetParam); }
        methods::UnMonitorParam GetUnMonitorParam() noexcept { return std::move(UnMonitorParam); }
        methods::InitClient GetInitClient() noexcept { return std::move(InitClient); }
    private:
        std::shared_ptr<ara::com::internal::proxy::ProxyAdapter> proxyAdapter;
        events::ParamUpdateEvent ParamUpdateEvent;
        events::ServerNotifyEvent ServerNotifyEvent;
        methods::AnswerAlive AnswerAlive;
        methods::DelParam DelParam;
        methods::GetMonitorClients GetMonitorClients;
        methods::GetParam GetParam;
        methods::MonitorParam MonitorParam;
        methods::SetParam SetParam;
        methods::UnMonitorParam UnMonitorParam;
        methods::InitClient InitClient;
    };

    virtual ~ConfigServerServiceInterfaceProxy()
    {
        ParamUpdateEvent.UnsetReceiveHandler();
        ParamUpdateEvent.Unsubscribe();
        ServerNotifyEvent.UnsetReceiveHandler();
        ServerNotifyEvent.Unsubscribe();

    }

    explicit ConfigServerServiceInterfaceProxy(const HandleType &handle)
        : proxyAdapter(std::make_shared<ara::com::internal::proxy::ProxyAdapter>(::mdc::config::server::ConfigServerServiceInterface::ServiceIdentifier, handle)),
          ParamUpdateEvent(proxyAdapter->GetProxy(), events::ConfigServerServiceInterfaceParamUpdateEventId, proxyAdapter->GetHandle(), proxyAdapter->GetServiceInterfaceInfo()),
          ServerNotifyEvent(proxyAdapter->GetProxy(), events::ConfigServerServiceInterfaceServerNotifyEventId, proxyAdapter->GetHandle(), proxyAdapter->GetServiceInterfaceInfo()),
          AnswerAlive(proxyAdapter->GetProxy(), methods::ConfigServerServiceInterfaceAnswerAliveId, proxyAdapter->GetServiceInterfaceInfo()),
          DelParam(proxyAdapter->GetProxy(), methods::ConfigServerServiceInterfaceDelParamId, proxyAdapter->GetServiceInterfaceInfo()),
          GetMonitorClients(proxyAdapter->GetProxy(), methods::ConfigServerServiceInterfaceGetMonitorClientsId, proxyAdapter->GetServiceInterfaceInfo()),
          GetParam(proxyAdapter->GetProxy(), methods::ConfigServerServiceInterfaceGetParamId, proxyAdapter->GetServiceInterfaceInfo()),
          MonitorParam(proxyAdapter->GetProxy(), methods::ConfigServerServiceInterfaceMonitorParamId, proxyAdapter->GetServiceInterfaceInfo()),
          SetParam(proxyAdapter->GetProxy(), methods::ConfigServerServiceInterfaceSetParamId, proxyAdapter->GetServiceInterfaceInfo()),
          UnMonitorParam(proxyAdapter->GetProxy(), methods::ConfigServerServiceInterfaceUnMonitorParamId, proxyAdapter->GetServiceInterfaceInfo()),
          InitClient(proxyAdapter->GetProxy(), methods::ConfigServerServiceInterfaceInitClientId, proxyAdapter->GetServiceInterfaceInfo()){
            ara::core::Result<void> resultAnswerAlive = AnswerAlive.GetMethodAdapter()->Initialize();
            ThrowError(resultAnswerAlive);
            ara::core::Result<void> resultDelParam = DelParam.GetMethodAdapter()->Initialize();
            ThrowError(resultDelParam);
            ara::core::Result<void> resultGetMonitorClients = GetMonitorClients.GetMethodAdapter()->Initialize();
            ThrowError(resultGetMonitorClients);
            ara::core::Result<void> resultGetParam = GetParam.GetMethodAdapter()->Initialize();
            ThrowError(resultGetParam);
            ara::core::Result<void> resultMonitorParam = MonitorParam.GetMethodAdapter()->Initialize();
            ThrowError(resultMonitorParam);
            ara::core::Result<void> resultSetParam = SetParam.GetMethodAdapter()->Initialize();
            ThrowError(resultSetParam);
            ara::core::Result<void> resultUnMonitorParam = UnMonitorParam.GetMethodAdapter()->Initialize();
            ThrowError(resultUnMonitorParam);
            ara::core::Result<void> resultInitClient = InitClient.GetMethodAdapter()->Initialize();
            ThrowError(resultInitClient);
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

    ConfigServerServiceInterfaceProxy(const ConfigServerServiceInterfaceProxy&) = delete;
    ConfigServerServiceInterfaceProxy& operator=(const ConfigServerServiceInterfaceProxy&) = delete;

    ConfigServerServiceInterfaceProxy(ConfigServerServiceInterfaceProxy&&) = default;
    ConfigServerServiceInterfaceProxy& operator=(ConfigServerServiceInterfaceProxy&&) = default;
    ConfigServerServiceInterfaceProxy(ConstructionToken&& token) noexcept
        : proxyAdapter(token.GetProxyAdapter()),
          ParamUpdateEvent(token.GetParamUpdateEvent()),
          ServerNotifyEvent(token.GetServerNotifyEvent()),
          AnswerAlive(token.GetAnswerAlive()),
          DelParam(token.GetDelParam()),
          GetMonitorClients(token.GetGetMonitorClients()),
          GetParam(token.GetGetParam()),
          MonitorParam(token.GetMonitorParam()),
          SetParam(token.GetSetParam()),
          UnMonitorParam(token.GetUnMonitorParam()),
          InitClient(token.GetInitClient()){
        static_cast<void>(std::move(token));
    }

    static ara::core::Result<ConstructionToken> Preconstruct(
        const ara::com::HandleType &handle)
    {
        std::shared_ptr<ara::com::internal::proxy::ProxyAdapter> preProxyAdapter =
            std::make_shared<ara::com::internal::proxy::ProxyAdapter>(
               ::mdc::config::server::ConfigServerServiceInterface::ServiceIdentifier, handle);
        ConstructionToken token{std::move(preProxyAdapter)};
        ara::core::Result<void> initResult {token.InitializeAll()};
        if (initResult.HasValue()) {
            return ara::core::Result<ConstructionToken>(std::move(token));
        } else {
            const ara::core::ErrorCode errorcode{initResult.Error()};
            ara::core::Result<ConfigServerServiceInterfaceProxy::ConstructionToken> preResult{errorcode};
            return preResult;
        }
    }

    static ara::com::FindServiceHandle StartFindService(
        const ara::com::FindServiceHandler<ara::com::internal::proxy::ProxyAdapter::HandleType>& handler,
        const ara::com::InstanceIdentifier instance)
    {
        return ara::com::internal::proxy::ProxyAdapter::StartFindService(handler, ::mdc::config::server::ConfigServerServiceInterface::ServiceIdentifier, instance);
    }

    static ara::com::FindServiceHandle StartFindService(
        const ara::com::FindServiceHandler<ara::com::internal::proxy::ProxyAdapter::HandleType> handler,
        const ara::core::InstanceSpecifier specifier)
    {
        return ara::com::internal::proxy::ProxyAdapter::StartFindService(handler, ::mdc::config::server::ConfigServerServiceInterface::ServiceIdentifier, specifier);
    }

    static ara::com::ServiceHandleContainer<ara::com::internal::proxy::ProxyAdapter::HandleType> FindService(
        const ara::com::InstanceIdentifier instance)
    {
        return ara::com::internal::proxy::ProxyAdapter::FindService(::mdc::config::server::ConfigServerServiceInterface::ServiceIdentifier, instance);
    }

    static ara::com::ServiceHandleContainer<ara::com::internal::proxy::ProxyAdapter::HandleType> FindService(
        const ara::core::InstanceSpecifier specifier)
    {
        return ara::com::internal::proxy::ProxyAdapter::FindService(::mdc::config::server::ConfigServerServiceInterface::ServiceIdentifier, specifier);
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
    events::ParamUpdateEvent ParamUpdateEvent;
    events::ServerNotifyEvent ServerNotifyEvent;
    methods::AnswerAlive AnswerAlive;
    methods::DelParam DelParam;
    methods::GetMonitorClients GetMonitorClients;
    methods::GetParam GetParam;
    methods::MonitorParam MonitorParam;
    methods::SetParam SetParam;
    methods::UnMonitorParam UnMonitorParam;
    methods::InitClient InitClient;
};
} // namespace proxy
} // namespace server
} // namespace config
} // namespace mdc

#endif // MDC_CONFIG_SERVER_CONFIGSERVERSERVICEINTERFACE_PROXY_H
