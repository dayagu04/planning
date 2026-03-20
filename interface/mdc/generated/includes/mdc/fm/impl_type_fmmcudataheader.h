/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2026. All rights reserved.
 */

#ifndef MDC_FM_IMPL_TYPE_FMMCUDATAHEADER_H
#define MDC_FM_IMPL_TYPE_FMMCUDATAHEADER_H


namespace mdc {
namespace fm {
struct FmMcuDataHeader {
    invalid type;
    invalid version;
    invalid checkSum;
    invalid seqId;
    invalid sendTime;

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

    friend bool operator==(const ::mdc::fm::FmMcuDataHeader& lhs, const ::mdc::fm::FmMcuDataHeader& rhs) noexcept
    {
        return (lhs.type == rhs.type) && (lhs.version == rhs.version) && (lhs.checkSum == rhs.checkSum) && (lhs.seqId == rhs.seqId) && (lhs.sendTime == rhs.sendTime);
    }
};
} // namespace fm
} // namespace mdc


#endif // MDC_FM_IMPL_TYPE_FMMCUDATAHEADER_H
