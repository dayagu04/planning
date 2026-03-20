/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2026. All rights reserved.
 */

#ifndef MDC_FM_IMPL_TYPE_FMMCUDATARETINFO_H
#define MDC_FM_IMPL_TYPE_FMMCUDATARETINFO_H


namespace mdc {
namespace fm {
struct FmMcuDataRetInfo {
    invalid type;
    invalid seqId;
    invalid retCode;

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

    friend bool operator==(const ::mdc::fm::FmMcuDataRetInfo& lhs, const ::mdc::fm::FmMcuDataRetInfo& rhs) noexcept
    {
        return (lhs.type == rhs.type) && (lhs.seqId == rhs.seqId) && (lhs.retCode == rhs.retCode);
    }
};
} // namespace fm
} // namespace mdc


#endif // MDC_FM_IMPL_TYPE_FMMCUDATARETINFO_H
