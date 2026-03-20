/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2026. All rights reserved.
 */

#ifndef MDC_CONFIG_SERVER_IMPL_TYPE_SERVERNOTIFYDATA_H
#define MDC_CONFIG_SERVER_IMPL_TYPE_SERVERNOTIFYDATA_H

#include "impl_type_string.h"

namespace mdc {
namespace config {
namespace server {
struct ServerNotifyData {
    invalid notifyType;
    ::String clientName;
    ::String extraData;

    static bool IsPlane()
    {
        return false;
    }


    using IsEnumerableTag = void;
    template<typename F>
    void enumerate(F& fun)
    {
    }

    template<typename F>
    void enumerate(F& fun) const
    {
    }

    template<typename F>
    void enumerate_internal(F& fun)
    {
    }

    template<typename F>
    void enumerate_internal(F& fun) const
    {
    }

    friend bool operator==(const ::mdc::config::server::ServerNotifyData& lhs, const ::mdc::config::server::ServerNotifyData& rhs) noexcept
    {
        return (lhs.notifyType == rhs.notifyType) && (lhs.clientName == rhs.clientName) && (lhs.extraData == rhs.extraData);
    }
};
} // namespace server
} // namespace config
} // namespace mdc


#endif // MDC_CONFIG_SERVER_IMPL_TYPE_SERVERNOTIFYDATA_H
