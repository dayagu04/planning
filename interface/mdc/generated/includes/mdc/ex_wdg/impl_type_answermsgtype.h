/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2026. All rights reserved.
 */

#ifndef MDC_EX_WDG_IMPL_TYPE_ANSWERMSGTYPE_H
#define MDC_EX_WDG_IMPL_TYPE_ANSWERMSGTYPE_H

#include "impl_type_uint32.h"
#include "impl_type_uint8.h"
#include "mdc/ex_wdg/impl_type_paramtype.h"

namespace mdc {
namespace ex_wdg {
struct AnswerMsgType {
    ::UInt32 versionId;
    ::UInt32 msgId;
    ::UInt8 answerParamLen;
    ::mdc::ex_wdg::ParamType qaParam;

    static bool IsPlane()
    {
        return false;
    }


    using IsEnumerableTag = void;
    template<typename F>
    void enumerate(F& fun)
    {
        fun(versionId);
        fun(msgId);
        fun(answerParamLen);
        fun(qaParam);
    }

    template<typename F>
    void enumerate(F& fun) const
    {
        fun(versionId);
        fun(msgId);
        fun(answerParamLen);
        fun(qaParam);
    }

    template<typename F>
    void enumerate_internal(F& fun)
    {
        fun("versionId", versionId);
        fun("msgId", msgId);
        fun("answerParamLen", answerParamLen);
        fun("qaParam", qaParam);
    }

    template<typename F>
    void enumerate_internal(F& fun) const
    {
        fun("versionId", versionId);
        fun("msgId", msgId);
        fun("answerParamLen", answerParamLen);
        fun("qaParam", qaParam);
    }

    friend bool operator==(const ::mdc::ex_wdg::AnswerMsgType& lhs, const ::mdc::ex_wdg::AnswerMsgType& rhs) noexcept
    {
        return (lhs.versionId == rhs.versionId) && (lhs.msgId == rhs.msgId) && (lhs.answerParamLen == rhs.answerParamLen) && (lhs.qaParam == rhs.qaParam);
    }
};
} // namespace ex_wdg
} // namespace mdc


#endif // MDC_EX_WDG_IMPL_TYPE_ANSWERMSGTYPE_H
