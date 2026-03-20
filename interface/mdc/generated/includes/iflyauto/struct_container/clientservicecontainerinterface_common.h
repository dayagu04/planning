/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2026. All rights reserved.
 */

#ifndef IFLYAUTO_STRUCT_CONTAINER_CLIENTSERVICECONTAINERINTERFACE_COMMON_H
#define IFLYAUTO_STRUCT_CONTAINER_CLIENTSERVICECONTAINERINTERFACE_COMMON_H

#include "ara/com/types.h"
#include "ara/com/init_config.h"
#include "iflyauto/struct_container/impl_type_structcontainer.h"
#include <cfloat>
#include <cmath>

namespace iflyauto {
namespace struct_container {
namespace methods {
namespace method {
struct Output {
    ::iflyauto::struct_container::StructContainer result;

    static bool IsPlane()
    {
        return false;
    }

    using IsEnumerableTag = void;
    template<typename F>
    void enumerate(F& fun)
    {
        fun(result);
    }

    template<typename F>
    void enumerate(F& fun) const
    {
        fun(result);
    }

    bool operator==(const Output& t) const
    {
       return (result == t.result);
    }
};
} // namespace method
} // namespace methods

class ClientServiceContainerInterface {
public:
    constexpr ClientServiceContainerInterface() = default;
    constexpr static ara::com::ServiceIdentifierType ServiceIdentifier = ara::com::ServiceIdentifierType("/HuaweiMDC/interface_manifests/interfaces/ClientServiceContainerInterface");
    constexpr static ara::com::ServiceVersionType ServiceVersion = ara::com::ServiceVersionType("1.1");
};
} // namespace struct_container
} // namespace iflyauto

#endif // IFLYAUTO_STRUCT_CONTAINER_CLIENTSERVICECONTAINERINTERFACE_COMMON_H
