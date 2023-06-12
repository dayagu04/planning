#ifndef ZNQC_COMMON_MACRO_H_
#define ZNQC_COMMON_MACRO_H_

#include <iostream>
#include <memory>
#include <mutex>
#include <type_traits>
#include <utility>

/**
 * @brief Macro definition for class feature including
 *        DISALLOW_COPY, DISALLOW_ASSIGN, DISALLOW_COPY_AND_ASSIGN,
 *        DISALLOW_IMPLICIT_CONSTRUCTORS and DECLARE_SINGLETON
 */
#define DISALLOW_COPY(ClassName) \
 public:                         \
  ClassName(ClassName const &) = delete;

#define DISALLOW_ASSIGN(ClassName) \
 public:                           \
  void operator=(ClassName const &) = delete;

#define DISALLOW_COPY_AND_ASSIGN(ClassName) \
 public:                                    \
  ClassName(ClassName const &) = delete;    \
  void operator=(ClassName const &) = delete;

#define DISALLOW_IMPLICIT_CONSTRUCTORS(ClassName) \
 private:                                         \
  ClassName() = default;                          \
  DISALLOW_COPY_AND_ASSIGN(ClassName);

#define DECLARE_SINGLETON(ClassName)                                            \
 public:                                                                        \
  static ClassName *Instance(bool create_if_needed = true) {                    \
    static ClassName *instance = nullptr;                                       \
    if (!instance && create_if_needed) {                                        \
      static std::once_flag flag;                                               \
      std::call_once(flag, [&] { instance = new (std::nothrow) ClassName(); }); \
    }                                                                           \
    return instance;                                                            \
  }                                                                             \
  ClassName() = default;                                                        \
  DISALLOW_COPY_AND_ASSIGN(ClassName);                                          \
                                                                                \
 private:

#endif  // ZNQC_COMMON_MACRO_H_
