/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2026. All rights reserved.
 */

#ifndef MDC_VO_IMPL_TYPE_RESOLUTION_H
#define MDC_VO_IMPL_TYPE_RESOLUTION_H


namespace mdc {
namespace vo {
struct Resolution {
    invalid width;
    invalid height;

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

    friend bool operator==(const ::mdc::vo::Resolution& lhs, const ::mdc::vo::Resolution& rhs) noexcept
    {
        return (lhs.width == rhs.width) && (lhs.height == rhs.height);
    }
};
} // namespace vo
} // namespace mdc


#endif // MDC_VO_IMPL_TYPE_RESOLUTION_H
