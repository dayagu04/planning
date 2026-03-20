/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2026. All rights reserved.
 */

#ifndef ARA_DIAG_IMPL_TYPE_GIDSTATUSSTRUCT_H
#define ARA_DIAG_IMPL_TYPE_GIDSTATUSSTRUCT_H

#include "ara/diag/impl_type_bytearray.h"
#include "impl_type_uint8.h"

namespace ara {
namespace diag {
struct GidStatusStruct {
    ::ara::diag::ByteArray GID;
    ::UInt8 furtherActionReq;
    ::UInt8 syncStatus;

    static bool IsPlane()
    {
        return false;
    }


    using IsEnumerableTag = void;
    template<typename F>
    void enumerate(F& fun)
    {
        fun(GID);
        fun(furtherActionReq);
        fun(syncStatus);
    }

    template<typename F>
    void enumerate(F& fun) const
    {
        fun(GID);
        fun(furtherActionReq);
        fun(syncStatus);
    }

    template<typename F>
    void enumerate_internal(F& fun)
    {
        fun("GID", GID);
        fun("furtherActionReq", furtherActionReq);
        fun("syncStatus", syncStatus);
    }

    template<typename F>
    void enumerate_internal(F& fun) const
    {
        fun("GID", GID);
        fun("furtherActionReq", furtherActionReq);
        fun("syncStatus", syncStatus);
    }

    friend bool operator==(const ::ara::diag::GidStatusStruct& lhs, const ::ara::diag::GidStatusStruct& rhs) noexcept
    {
        return (lhs.GID == rhs.GID) && (lhs.furtherActionReq == rhs.furtherActionReq) && (lhs.syncStatus == rhs.syncStatus);
    }
};
} // namespace diag
} // namespace ara


#endif // ARA_DIAG_IMPL_TYPE_GIDSTATUSSTRUCT_H
