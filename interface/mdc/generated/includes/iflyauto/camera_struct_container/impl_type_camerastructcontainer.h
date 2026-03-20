/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2020-2026. All rights reserved.
 */

#ifndef IFLYAUTO_CAMERA_STRUCT_CONTAINER_IMPL_TYPE_CAMERASTRUCTCONTAINER_H
#define IFLYAUTO_CAMERA_STRUCT_CONTAINER_IMPL_TYPE_CAMERASTRUCTCONTAINER_H

#include "impl_type_string.h"
#include "impl_type_rawbuffer.h"

namespace iflyauto {
namespace camera_struct_container {
struct CameraStructContainer {
    ::String meta_data;
    ::rawBuffer *camera_data{ nullptr };

    static bool IsPlane()
    {
        return false;
    }

    ::rawBuffer* GetMbufPtr() const
    {
        return camera_data;
    }

    void SetMbufPtr(::rawBuffer *p)
    {
        camera_data = p;
    }

    using IsDpRawDataTag = void;
    using IsEnumerableTag = void;
    template<typename F>
    void enumerate(F& fun)
    {
        fun(meta_data);
    }

    template<typename F>
    void enumerate(F& fun) const
    {
        fun(meta_data);
    }

    template<typename F>
    void enumerate_internal(F& fun)
    {
        fun("meta_data", meta_data);
    }

    template<typename F>
    void enumerate_internal(F& fun) const
    {
        fun("meta_data", meta_data);
    }

    friend bool operator==(const ::iflyauto::camera_struct_container::CameraStructContainer& lhs, const ::iflyauto::camera_struct_container::CameraStructContainer& rhs) noexcept
    {
        return (lhs.meta_data == rhs.meta_data) && (lhs.camera_data == rhs.camera_data);
    }
};
} // namespace camera_struct_container
} // namespace iflyauto


#endif // IFLYAUTO_CAMERA_STRUCT_CONTAINER_IMPL_TYPE_CAMERASTRUCTCONTAINER_H
