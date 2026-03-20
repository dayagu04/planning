/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2026. All rights reserved.
 */

#ifndef MDC_SWM_IMPL_TYPE_VERSIONITEM_H
#define MDC_SWM_IMPL_TYPE_VERSIONITEM_H

#include "impl_type_string.h"
#include "impl_type_int8.h"
#include "mdc/swm/impl_type_versioninfovector.h"
#include "impl_type_uint8.h"

namespace mdc {
namespace swm {
struct VersionItem {
    ::String name;
    ::String location;
    ::Int8 state;
    ::Int8 type;
    ::mdc::swm::VersionInfoVector versions;
    ::UInt8 deviceStatus;

    static bool IsPlane()
    {
        return false;
    }


    using IsEnumerableTag = void;
    template<typename F>
    void enumerate(F& fun)
    {
        fun(name);
        fun(location);
        fun(state);
        fun(type);
        fun(versions);
        fun(deviceStatus);
    }

    template<typename F>
    void enumerate(F& fun) const
    {
        fun(name);
        fun(location);
        fun(state);
        fun(type);
        fun(versions);
        fun(deviceStatus);
    }

    template<typename F>
    void enumerate_internal(F& fun)
    {
        fun("name", name);
        fun("location", location);
        fun("state", state);
        fun("type", type);
        fun("versions", versions);
        fun("deviceStatus", deviceStatus);
    }

    template<typename F>
    void enumerate_internal(F& fun) const
    {
        fun("name", name);
        fun("location", location);
        fun("state", state);
        fun("type", type);
        fun("versions", versions);
        fun("deviceStatus", deviceStatus);
    }

    friend bool operator==(const ::mdc::swm::VersionItem& lhs, const ::mdc::swm::VersionItem& rhs) noexcept
    {
        return (lhs.name == rhs.name) && (lhs.location == rhs.location) && (lhs.state == rhs.state) && (lhs.type == rhs.type) && (lhs.versions == rhs.versions) && (lhs.deviceStatus == rhs.deviceStatus);
    }
};
} // namespace swm
} // namespace mdc


#endif // MDC_SWM_IMPL_TYPE_VERSIONITEM_H
