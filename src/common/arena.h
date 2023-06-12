#ifndef ZNQC_COMMON_ARENA_H
#define ZNQC_COMMON_ARENA_H

#include "google/protobuf/arena.h"
#include "macro.h"

namespace planning {
namespace common {

class Arena {
 public:
  Arena() {}
  virtual ~Arena() {}

  template <typename T>
  typename std::enable_if<std::is_base_of<::google::protobuf::Message, T>::value, T>::type* alloc() {
    return ::google::protobuf::Arena::CreateMessage<T>(&arena_);
  }

  template <typename T, typename... Args>
  typename std::enable_if<!std::is_base_of<::google::protobuf::Message, T>::value, T>::type* alloc(Args&&... args) {
    return ::google::protobuf::Arena::Create<T>(&arena_, std::forward<Args>(args)...);
  }

  template <typename T>
  typename std::enable_if<std::is_base_of<::google::protobuf::Message, T>::value, T>::type* alloc_persist() {
    return ::google::protobuf::Arena::CreateMessage<T>(&persist_arena_);
  }

  template <typename T>
  typename std::enable_if<!std::is_base_of<::google::protobuf::Message, T>::value, T>::type* alloc_persist() {
    return ::google::protobuf::Arena::Create<T>(&persist_arena_);
  }

  ::google::protobuf::Arena arena_;
  static ::google::protobuf::Arena persist_arena_;

  DISALLOW_COPY_AND_ASSIGN(Arena);
};

}  // namespace common
}  // namespace planning

#endif  // ZNQC_COMMON_ARENA_H
