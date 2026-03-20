/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2026. All rights reserved.
 */

#ifndef MDC_VO_IMPL_TYPE_CHNINFO_H
#define MDC_VO_IMPL_TYPE_CHNINFO_H

#include "mdc/vo/impl_type_slotid.h"
#include "impl_type_string.h"
#include "mdc/vo/impl_type_videoparam.h"

namespace mdc {
namespace vo {
struct ChnInfo {
    invalid chnId;
    ::mdc::vo::SlotId slotId{ ::mdc::vo::SlotId::D4 };
    invalid isEnabled;
    ::String function;
    ::mdc::vo::VideoParam videoParam;

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

    friend bool operator==(const ::mdc::vo::ChnInfo& lhs, const ::mdc::vo::ChnInfo& rhs) noexcept
    {
        return (lhs.chnId == rhs.chnId) && (lhs.slotId == rhs.slotId) && (lhs.isEnabled == rhs.isEnabled) && (lhs.function == rhs.function) && (lhs.videoParam == rhs.videoParam);
    }
};
} // namespace vo
} // namespace mdc


#endif // MDC_VO_IMPL_TYPE_CHNINFO_H
