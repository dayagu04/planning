/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2026. All rights reserved.
 */

#ifndef MDC_DCAP_IMPL_TYPE_PACKAGEDIGESTTYPE_H
#define MDC_DCAP_IMPL_TYPE_PACKAGEDIGESTTYPE_H

#include "impl_type_string.h"
#include "impl_type_int32.h"

namespace mdc {
namespace dcap {
struct PackageDigestType {
    ::String digestStr;
    ::Int32 retCode;

    static bool IsPlane()
    {
        return false;
    }


    using IsEnumerableTag = void;
    template<typename F>
    void enumerate(F& fun)
    {
        fun(digestStr);
        fun(retCode);
    }

    template<typename F>
    void enumerate(F& fun) const
    {
        fun(digestStr);
        fun(retCode);
    }

    template<typename F>
    void enumerate_internal(F& fun)
    {
        fun("digestStr", digestStr);
        fun("retCode", retCode);
    }

    template<typename F>
    void enumerate_internal(F& fun) const
    {
        fun("digestStr", digestStr);
        fun("retCode", retCode);
    }

    friend bool operator==(const ::mdc::dcap::PackageDigestType& lhs, const ::mdc::dcap::PackageDigestType& rhs) noexcept
    {
        return (lhs.digestStr == rhs.digestStr) && (lhs.retCode == rhs.retCode);
    }
};
} // namespace dcap
} // namespace mdc


#endif // MDC_DCAP_IMPL_TYPE_PACKAGEDIGESTTYPE_H
