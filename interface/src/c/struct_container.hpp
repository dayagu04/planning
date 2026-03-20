// Copyright (C) iflyauto Technologies (2023). All rights reserved.
// Modified: 2023/11/5

#pragma once

#include <algorithm>
#include <cstring>
#include <memory>

#include "struct_container.pb.h"

namespace iflyauto {

template <class T>
inline T* struct_cast(std::shared_ptr<iflyauto::StructContainer> struct_container) {
  auto payload = struct_container->mutable_payload();
  payload->resize(sizeof(T));
  auto struct_ptr = reinterpret_cast<T*>(&(*payload)[0]);
  return struct_ptr;
}

template <class T>
inline T* struct_cast(iflyauto::StructContainer* struct_container) {
  auto payload = struct_container->mutable_payload();
  payload->resize(sizeof(T));
  auto struct_ptr = reinterpret_cast<T*>(&(*payload)[0]);
  return struct_ptr;
}

template <class T, size_t N>
inline void strcpy_array(T (&dst)[N], const char* src) {
  size_t n = strlen(src);
  if (n + 1 > N) {
    n = N - 1;
  }
  strncpy((char*)dst, src, n);
  dst[n] = '\0';
}

}  // namespace iflyauto
