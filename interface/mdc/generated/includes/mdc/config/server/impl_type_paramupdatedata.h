/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2026. All rights reserved.
 */

#ifndef MDC_CONFIG_SERVER_IMPL_TYPE_PARAMUPDATEDATA_H
#define MDC_CONFIG_SERVER_IMPL_TYPE_PARAMUPDATEDATA_H

#include "impl_type_string.h"
#include "impl_type_uint8.h"

namespace mdc {
namespace config {
namespace server {
struct ParamUpdateData {
    ::String paramName;
    ::String paramValue;
    ::UInt8 paramType;
    ::String clientName;
    ::String uuid;

    static bool IsPlane()
    {
        return false;
    }


    using IsEnumerableTag = void;
    template<typename F>
    void enumerate(F& fun)
    {
        fun(paramName);
        fun(paramValue);
        fun(paramType);
        fun(clientName);
        fun(uuid);
    }

    template<typename F>
    void enumerate(F& fun) const
    {
        fun(paramName);
        fun(paramValue);
        fun(paramType);
        fun(clientName);
        fun(uuid);
    }

    template<typename F>
    void enumerate_internal(F& fun)
    {
        fun("paramName", paramName);
        fun("paramValue", paramValue);
        fun("paramType", paramType);
        fun("clientName", clientName);
        fun("uuid", uuid);
    }

    template<typename F>
    void enumerate_internal(F& fun) const
    {
        fun("paramName", paramName);
        fun("paramValue", paramValue);
        fun("paramType", paramType);
        fun("clientName", clientName);
        fun("uuid", uuid);
    }

    friend bool operator==(const ::mdc::config::server::ParamUpdateData& lhs, const ::mdc::config::server::ParamUpdateData& rhs) noexcept
    {
        return (lhs.paramName == rhs.paramName) && (lhs.paramValue == rhs.paramValue) && (lhs.paramType == rhs.paramType) && (lhs.clientName == rhs.clientName) && (lhs.uuid == rhs.uuid);
    }
};
} // namespace server
} // namespace config
} // namespace mdc


#endif // MDC_CONFIG_SERVER_IMPL_TYPE_PARAMUPDATEDATA_H
