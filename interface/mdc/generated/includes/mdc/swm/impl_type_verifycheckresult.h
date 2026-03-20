/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2026. All rights reserved.
 */

#ifndef MDC_SWM_IMPL_TYPE_VERIFYCHECKRESULT_H
#define MDC_SWM_IMPL_TYPE_VERIFYCHECKRESULT_H

#include "impl_type_string.h"

namespace mdc {
namespace swm {
struct VerifyCheckResult {
    ::String pkgName;
    ::String deviceName;
    ::String currentVersion;
    ::String expectVersion;
    ::String message;

    static bool IsPlane()
    {
        return false;
    }


    using IsEnumerableTag = void;
    template<typename F>
    void enumerate(F& fun)
    {
        fun(pkgName);
        fun(deviceName);
        fun(currentVersion);
        fun(expectVersion);
        fun(message);
    }

    template<typename F>
    void enumerate(F& fun) const
    {
        fun(pkgName);
        fun(deviceName);
        fun(currentVersion);
        fun(expectVersion);
        fun(message);
    }

    template<typename F>
    void enumerate_internal(F& fun)
    {
        fun("pkgName", pkgName);
        fun("deviceName", deviceName);
        fun("currentVersion", currentVersion);
        fun("expectVersion", expectVersion);
        fun("message", message);
    }

    template<typename F>
    void enumerate_internal(F& fun) const
    {
        fun("pkgName", pkgName);
        fun("deviceName", deviceName);
        fun("currentVersion", currentVersion);
        fun("expectVersion", expectVersion);
        fun("message", message);
    }

    friend bool operator==(const ::mdc::swm::VerifyCheckResult& lhs, const ::mdc::swm::VerifyCheckResult& rhs) noexcept
    {
        return (lhs.pkgName == rhs.pkgName) && (lhs.deviceName == rhs.deviceName) && (lhs.currentVersion == rhs.currentVersion) && (lhs.expectVersion == rhs.expectVersion) && (lhs.message == rhs.message);
    }
};
} // namespace swm
} // namespace mdc


#endif // MDC_SWM_IMPL_TYPE_VERIFYCHECKRESULT_H
