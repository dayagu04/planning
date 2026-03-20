/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2026. All rights reserved.
 */

#ifndef ARA_DIAG_IMPL_TYPE_DIDINFO_H
#define ARA_DIAG_IMPL_TYPE_DIDINFO_H

#include "impl_type_uint16.h"
#include "ara/diag/impl_type_bytearray.h"

namespace ara {
namespace diag {
struct DidInfo {
    ::UInt16 didId;
    ::ara::diag::ByteArray didValue;

    static bool IsPlane()
    {
        return false;
    }


    using IsEnumerableTag = void;
    template<typename F>
    void enumerate(F& fun)
    {
        fun(didId);
        fun(didValue);
    }

    template<typename F>
    void enumerate(F& fun) const
    {
        fun(didId);
        fun(didValue);
    }

    template<typename F>
    void enumerate_internal(F& fun)
    {
        fun("didId", didId);
        fun("didValue", didValue);
    }

    template<typename F>
    void enumerate_internal(F& fun) const
    {
        fun("didId", didId);
        fun("didValue", didValue);
    }

    friend bool operator==(const ::ara::diag::DidInfo& lhs, const ::ara::diag::DidInfo& rhs) noexcept
    {
        return (lhs.didId == rhs.didId) && (lhs.didValue == rhs.didValue);
    }
};
} // namespace diag
} // namespace ara


#endif // ARA_DIAG_IMPL_TYPE_DIDINFO_H
