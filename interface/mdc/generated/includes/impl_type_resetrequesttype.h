/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2026. All rights reserved.
 */

#ifndef IMPL_TYPE_RESETREQUESTTYPE_H
#define IMPL_TYPE_RESETREQUESTTYPE_H

enum class ResetRequestType : invalid
{
    kSoftReset = 0,
    kHardReset = 1,
    kKeyOffOnReset = 2,
    kCustomReset = 3
};


#endif // IMPL_TYPE_RESETREQUESTTYPE_H
