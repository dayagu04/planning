/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2026. All rights reserved.
 */

#ifndef IFLYAUTO_STRUCT_CONTAINER_CLIENTSERVICECONTAINERINTERFACE_PROXY_H
#define IFLYAUTO_STRUCT_CONTAINER_CLIENTSERVICECONTAINERINTERFACE_PROXY_H

#include "ara/com/internal/proxy/proxy_adapter.h"
#include "ara/com/internal/proxy/event_adapter.h"
#include "ara/com/internal/proxy/field_adapter.h"
#include "ara/com/internal/proxy/method_adapter.h"
#include "ara/com/crc_verification.h"
#include "iflyauto/struct_container/clientservicecontainerinterface_common.h"
#include <string>

namespace iflyauto {
namespace struct_container {
namespace proxy {
namespace events {
}

namespace fields {
}

namespace methods {
static constexpr ara::com::internal::EntityId ClientServiceContainerInterfacemethodId = 17484U; //method_method_hash


class method {
public:
    using Output = iflyauto::struct_container::methods::method::Output;

    method(const std::shared_ptr<rtf::cm::vcc::Proxy>& proxy, const ara::com::internal::EntityId entityId, const rtf::cm::type::ServiceInterfaceInfo& serviceInterfaceInfo)
        : method_(std::make_shared<ara::com::internal::proxy::method::MethodAdapter<Output, ::iflyauto::struct_container::StructContainer>>(proxy, entityId, serviceInterfaceInfo)) {}

    ara::core::Future<Output> operator()(const ::iflyauto::struct_container::StructContainer& msg)
    {
        return method_->operator()(msg);
    }

    ara::com::e2e::SMState GetSMState() const noexcept
    {
        return method_->GetSMState();
    }

    /* Do not use such internal interfaces!!! */
    std::shared_ptr<ara::com::internal::proxy::method::MethodAdapter<Output, ::iflyauto::struct_container::StructContainer>> GetMethodAdapter() const noexcept
    {
        return method_;
    }
private:
    std::shared_ptr<ara::com::internal::proxy::method::MethodAdapter<Output, ::iflyauto::struct_container::StructContainer>> method_;
};
} // namespace methods

class ClientServiceContainerInterfaceProxy {
private:
    std::shared_ptr<ara::com::internal::proxy::ProxyAdapter> proxyAdapter;
public:
    using HandleType = ara::com::HandleType;
    class ConstructionToken {
    public:
        explicit ConstructionToken(const std::shared_ptr<ara::com::internal::proxy::ProxyAdapter> & proxy)
            : proxyAdapter(proxy),
              method(proxyAdapter->GetProxy(), methods::ClientServiceContainerInterfacemethodId, proxyAdapter->GetServiceInterfaceInfo()){
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
            initResult = method.GetMethodAdapter()->Initialize();
            if (!initResult.HasValue()) {
                return initResult;
            }
            return initResult;
        }
        std::shared_ptr<ara::com::internal::proxy::ProxyAdapter> GetProxyAdapter()
        {
            return proxyAdapter;
        }
        methods::method Getmethod() noexcept { return std::move(method); }
    private:
        std::shared_ptr<ara::com::internal::proxy::ProxyAdapter> proxyAdapter;
        methods::method method;
    };

    virtual ~ClientServiceContainerInterfaceProxy()
    {

    }

    explicit ClientServiceContainerInterfaceProxy(const HandleType &handle)
        : proxyAdapter(std::make_shared<ara::com::internal::proxy::ProxyAdapter>(::iflyauto::struct_container::ClientServiceContainerInterface::ServiceIdentifier, handle)),
          method(proxyAdapter->GetProxy(), methods::ClientServiceContainerInterfacemethodId, proxyAdapter->GetServiceInterfaceInfo()){
            ara::core::Result<void> resultmethod = method.GetMethodAdapter()->Initialize();
            ThrowError(resultmethod);
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

    ClientServiceContainerInterfaceProxy(const ClientServiceContainerInterfaceProxy&) = delete;
    ClientServiceContainerInterfaceProxy& operator=(const ClientServiceContainerInterfaceProxy&) = delete;

    ClientServiceContainerInterfaceProxy(ClientServiceContainerInterfaceProxy&&) = default;
    ClientServiceContainerInterfaceProxy& operator=(ClientServiceContainerInterfaceProxy&&) = default;
    ClientServiceContainerInterfaceProxy(ConstructionToken&& token) noexcept
        : proxyAdapter(token.GetProxyAdapter()),
          method(token.Getmethod()){
        static_cast<void>(std::move(token));
    }

    static ara::core::Result<ConstructionToken> Preconstruct(
        const ara::com::HandleType &handle)
    {
        std::shared_ptr<ara::com::internal::proxy::ProxyAdapter> preProxyAdapter =
            std::make_shared<ara::com::internal::proxy::ProxyAdapter>(
               ::iflyauto::struct_container::ClientServiceContainerInterface::ServiceIdentifier, handle);
        ConstructionToken token{std::move(preProxyAdapter)};
        ara::core::Result<void> initResult {token.InitializeAll()};
        if (initResult.HasValue()) {
            return ara::core::Result<ConstructionToken>(std::move(token));
        } else {
            const ara::core::ErrorCode errorcode{initResult.Error()};
            ara::core::Result<ClientServiceContainerInterfaceProxy::ConstructionToken> preResult{errorcode};
            return preResult;
        }
    }

    static ara::com::FindServiceHandle StartFindService(
        const ara::com::FindServiceHandler<ara::com::internal::proxy::ProxyAdapter::HandleType>& handler,
        const ara::com::InstanceIdentifier instance)
    {
        return ara::com::internal::proxy::ProxyAdapter::StartFindService(handler, ::iflyauto::struct_container::ClientServiceContainerInterface::ServiceIdentifier, instance);
    }

    static ara::com::FindServiceHandle StartFindService(
        const ara::com::FindServiceHandler<ara::com::internal::proxy::ProxyAdapter::HandleType> handler,
        const ara::core::InstanceSpecifier specifier)
    {
        return ara::com::internal::proxy::ProxyAdapter::StartFindService(handler, ::iflyauto::struct_container::ClientServiceContainerInterface::ServiceIdentifier, specifier);
    }

    static ara::com::ServiceHandleContainer<ara::com::internal::proxy::ProxyAdapter::HandleType> FindService(
        const ara::com::InstanceIdentifier instance)
    {
        return ara::com::internal::proxy::ProxyAdapter::FindService(::iflyauto::struct_container::ClientServiceContainerInterface::ServiceIdentifier, instance);
    }

    static ara::com::ServiceHandleContainer<ara::com::internal::proxy::ProxyAdapter::HandleType> FindService(
        const ara::core::InstanceSpecifier specifier)
    {
        return ara::com::internal::proxy::ProxyAdapter::FindService(::iflyauto::struct_container::ClientServiceContainerInterface::ServiceIdentifier, specifier);
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
    methods::method method;
};
} // namespace proxy
} // namespace struct_container
} // namespace iflyauto

#endif // IFLYAUTO_STRUCT_CONTAINER_CLIENTSERVICECONTAINERINTERFACE_PROXY_H
