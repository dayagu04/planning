/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2026. All rights reserved.
 */

#ifndef MDC_VO_IMPL_TYPE_VIDEOPARAM_H
#define MDC_VO_IMPL_TYPE_VIDEOPARAM_H

#include "mdc/vo/impl_type_resolution.h"

namespace mdc {
namespace vo {
struct VideoParam {
    invalid imageFormat;
    invalid frameRate;
    ::mdc::vo::Resolution resolution;

    static bool IsPlane()
    {
        return true;
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

    friend bool operator==(const ::mdc::vo::VideoParam& lhs, const ::mdc::vo::VideoParam& rhs) noexcept
    {
        return (lhs.imageFormat == rhs.imageFormat) && (lhs.frameRate == rhs.frameRate) && (lhs.resolution == rhs.resolution);
    }
};
} // namespace vo
} // namespace mdc


#endif // MDC_VO_IMPL_TYPE_VIDEOPARAM_H
