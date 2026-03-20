/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2026. All rights reserved.
 */

#ifndef MDC_DCAP_IMPL_TYPE_PACKPARAMTYPE_H
#define MDC_DCAP_IMPL_TYPE_PACKPARAMTYPE_H

#include "impl_type_string.h"
#include "mdc/dcap/impl_type_objectmap.h"
#include "impl_type_uint32.h"

namespace mdc {
namespace dcap {
struct PackParamType {
    ::String startTimestamp;
    ::String endTimestamp;
    ::mdc::dcap::ObjectMap fileType;
    ::UInt32 packageSize;
    ::String path;

    static bool IsPlane()
    {
        return false;
    }


    using IsEnumerableTag = void;
    template<typename F>
    void enumerate(F& fun)
    {
        fun(startTimestamp);
        fun(endTimestamp);
        fun(fileType);
        fun(packageSize);
        fun(path);
    }

    template<typename F>
    void enumerate(F& fun) const
    {
        fun(startTimestamp);
        fun(endTimestamp);
        fun(fileType);
        fun(packageSize);
        fun(path);
    }

    template<typename F>
    void enumerate_internal(F& fun)
    {
        fun("startTimestamp", startTimestamp);
        fun("endTimestamp", endTimestamp);
        fun("fileType", fileType);
        fun("packageSize", packageSize);
        fun("path", path);
    }

    template<typename F>
    void enumerate_internal(F& fun) const
    {
        fun("startTimestamp", startTimestamp);
        fun("endTimestamp", endTimestamp);
        fun("fileType", fileType);
        fun("packageSize", packageSize);
        fun("path", path);
    }

    friend bool operator==(const ::mdc::dcap::PackParamType& lhs, const ::mdc::dcap::PackParamType& rhs) noexcept
    {
        return (lhs.startTimestamp == rhs.startTimestamp) && (lhs.endTimestamp == rhs.endTimestamp) && (lhs.fileType == rhs.fileType) && (lhs.packageSize == rhs.packageSize) && (lhs.path == rhs.path);
    }
};
} // namespace dcap
} // namespace mdc


#endif // MDC_DCAP_IMPL_TYPE_PACKPARAMTYPE_H
