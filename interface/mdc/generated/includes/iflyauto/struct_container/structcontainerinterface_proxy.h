/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2026. All rights reserved.
 */

#ifndef IFLYAUTO_STRUCT_CONTAINER_STRUCTCONTAINERINTERFACE_PROXY_H
#define IFLYAUTO_STRUCT_CONTAINER_STRUCTCONTAINERINTERFACE_PROXY_H

#include "ara/com/internal/proxy/proxy_adapter.h"
#include "ara/com/internal/proxy/event_adapter.h"
#include "ara/com/internal/proxy/field_adapter.h"
#include "ara/com/internal/proxy/method_adapter.h"
#include "ara/com/crc_verification.h"
#include "iflyauto/struct_container/structcontainerinterface_common.h"
#include <string>

namespace iflyauto {
namespace struct_container {
namespace proxy {
namespace events {
    using event = ara::com::internal::proxy::event::EventAdapter<::ara::diag::ByteVector>;
    static constexpr ara::com::internal::EntityId StructContainerInterfaceeventId = 31614U; //event_event_hash
}

namespace fields {
}

namespace methods {

} // namespace methods

class StructContainerInterfaceProxy {
private:
    std::shared_ptr<ara::com::internal::proxy::ProxyAdapter> proxyAdapter;
public:
    using HandleType = ara::com::HandleType;
    class ConstructionToken {
    public:
        explicit ConstructionToken(const std::shared_ptr<ara::com::internal::proxy::ProxyAdapter> & proxy)
            : proxyAdapter(proxy),
              event(proxyAdapter->GetProxy(), events::StructContainerInterfaceeventId, proxyAdapter->GetHandle(), proxyAdapter->GetServiceInterfaceInfo()){
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
            return initResult;
        }
        std::shared_ptr<ara::com::internal::proxy::ProxyAdapter> GetProxyAdapter()
        {
            return proxyAdapter;
        }
        events::event Getevent() noexcept { return std::move(event); }
    private:
        std::shared_ptr<ara::com::internal::proxy::ProxyAdapter> proxyAdapter;
        events::event event;
    };

    virtual ~StructContainerInterfaceProxy()
    {
        event.UnsetReceiveHandler();
        event.Unsubscribe();

    }

    explicit StructContainerInterfaceProxy(const HandleType &handle)
        : proxyAdapter(std::make_shared<ara::com::internal::proxy::ProxyAdapter>(::iflyauto::struct_container::StructContainerInterface::ServiceIdentifier, handle)),
          event(proxyAdapter->GetProxy(), events::StructContainerInterfaceeventId, proxyAdapter->GetHandle(), proxyAdapter->GetServiceInterfaceInfo()){
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

    StructContainerInterfaceProxy(const StructContainerInterfaceProxy&) = delete;
    StructContainerInterfaceProxy& operator=(const StructContainerInterfaceProxy&) = delete;

    StructContainerInterfaceProxy(StructContainerInterfaceProxy&&) = default;
    StructContainerInterfaceProxy& operator=(StructContainerInterfaceProxy&&) = default;
    StructContainerInterfaceProxy(ConstructionToken&& token) noexcept
        : proxyAdapter(token.GetProxyAdapter()),
          event(token.Getevent()){
        static_cast<void>(std::move(token));
    }

    static ara::core::Result<ConstructionToken> Preconstruct(
        const ara::com::HandleType &handle)
    {
        std::shared_ptr<ara::com::internal::proxy::ProxyAdapter> preProxyAdapter =
            std::make_shared<ara::com::internal::proxy::ProxyAdapter>(
               ::iflyauto::struct_container::StructContainerInterface::ServiceIdentifier, handle);
        ConstructionToken token{std::move(preProxyAdapter)};
        ara::core::Result<void> initResult {token.InitializeAll()};
        if (initResult.HasValue()) {
            return ara::core::Result<ConstructionToken>(std::move(token));
        } else {
            const ara::core::ErrorCode errorcode{initResult.Error()};
            ara::core::Result<StructContainerInterfaceProxy::ConstructionToken> preResult{errorcode};
            return preResult;
        }
    }

    static ara::com::FindServiceHandle StartFindService(
        const ara::com::FindServiceHandler<ara::com::internal::proxy::ProxyAdapter::HandleType>& handler,
        const ara::com::InstanceIdentifier instance)
    {
        return ara::com::internal::proxy::ProxyAdapter::StartFindService(handler, ::iflyauto::struct_container::StructContainerInterface::ServiceIdentifier, instance);
    }

    static ara::com::FindServiceHandle StartFindService(
        const ara::com::FindServiceHandler<ara::com::internal::proxy::ProxyAdapter::HandleType> handler,
        const ara::core::InstanceSpecifier specifier)
    {
        return ara::com::internal::proxy::ProxyAdapter::StartFindService(handler, ::iflyauto::struct_container::StructContainerInterface::ServiceIdentifier, specifier);
    }

    static ara::com::ServiceHandleContainer<ara::com::internal::proxy::ProxyAdapter::HandleType> FindService(
        const ara::com::InstanceIdentifier instance)
    {
        return ara::com::internal::proxy::ProxyAdapter::FindService(::iflyauto::struct_container::StructContainerInterface::ServiceIdentifier, instance);
    }

    static ara::com::ServiceHandleContainer<ara::com::internal::proxy::ProxyAdapter::HandleType> FindService(
        const ara::core::InstanceSpecifier specifier)
    {
        return ara::com::internal::proxy::ProxyAdapter::FindService(::iflyauto::struct_container::StructContainerInterface::ServiceIdentifier, specifier);
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
    events::event event;
};
} // namespace proxy
} // namespace struct_container
} // namespace iflyauto

#endif // IFLYAUTO_STRUCT_CONTAINER_STRUCTCONTAINERINTERFACE_PROXY_H
