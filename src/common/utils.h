/**
 * @file
 * @brief Some util functions.
 */

#pragma once

#include <algorithm>
#include <cassert>
#include <chrono>
#include <cstdio>
#include <iostream>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

#include "vec2d.h"

// The helper function "std::make_unique()" is defined since C++14.
// The definition of "std::make_unique()" borrowed from C++14 is given here
// so that it can be used in C++11.
#if __cplusplus == 201103L
namespace std {

template <typename _Tp>
struct _MakeUniq {
  typedef unique_ptr<_Tp> __single_object;
};

template <typename _Tp>
struct _MakeUniq<_Tp[]> {
  typedef unique_ptr<_Tp[]> __array;
};

template <typename _Tp, size_t _Bound>
struct _MakeUniq<_Tp[_Bound]> {
  struct __invalid_type {};
};

// std::make_unique for single objects
template <typename _Tp, typename... _Args>
inline typename _MakeUniq<_Tp>::__single_object make_unique(_Args&&... __args) {
  return unique_ptr<_Tp>(new _Tp(std::forward<_Args>(__args)...));
}

// Alias template for remove_extent
template <typename _Tp>
using remove_extent_t = typename remove_extent<_Tp>::type;

// std::make_unique for arrays of unknown bound
template <typename _Tp>
inline typename _MakeUniq<_Tp>::__array make_unique(size_t __num) {
  return unique_ptr<_Tp>(new remove_extent_t<_Tp>[__num]());
}

// Disable std::make_unique for arrays of known bound
template <typename _Tp, typename... _Args>
inline typename _MakeUniq<_Tp>::__invalid_type make_unique(_Args&&...) = delete;
}  // namespace std
#endif

#define READ_STR_PARA(param_reader, key) \
  planning::common::trim(param_reader.get<std::string>(key)).c_str()

#define READ_STR_PARA_OPT(param_reader, key) \
  planning::common::trim(param_reader.get<std::string>(key, false)).c_str()

/**
 * @namespace planning::common::util
 * @brief planning::common::util
 */
namespace planning {
namespace common {
template <typename ProtoA, typename ProtoB>
bool IsProtoEqual(const ProtoA& a, const ProtoB& b) {
  return a.GetTypeName() == b.GetTypeName() &&
         a.SerializeAsString() == b.SerializeAsString();
  // Test shows that the above method is 5 times faster than the
  // API: google::protobuf::util::MessageDifferencer::Equals(a, b);
}

struct PairHash {
  template <typename T, typename U>
  size_t operator()(const std::pair<T, U>& pair) const {
    return std::hash<T>()(pair.first) ^ std::hash<U>()(pair.second);
  }
};

template <typename Container>
typename Container::value_type MinElement(const Container& elements) {
  return *std::min_element(elements.begin(), elements.end());
}

/**
 * calculate the distance beteween Point u and Point v, which are all have
 * member function x() and y() in XY dimension.
 * @param u one point that has member function x() and y().
 * @param b one point that has member function x() and y().
 * @return sqrt((u.x-v.x)^2 + (u.y-v.y)^2), i.e., the Euclid distance on XY
 * dimension.
 */
template <typename U, typename V>
double DistanceXY(const U& u, const V& v) {
  return std::hypot(u.x() - v.x(), u.y() - v.y());
}

/**
 * Check if two points u and v are the same point on XY dimension.
 * @param u one point that has member function x() and y().
 * @param v one point that has member function x() and y().
 * @return sqrt((u.x-v.x)^2 + (u.y-v.y)^2) < epsilon, i.e., the Euclid distance
 * on XY dimension.
 */
template <typename U, typename V>
bool SamePointXY(const U& u, const V& v) {
  constexpr double kMathEpsilonSqr = 1e-8 * 1e-8;
  return (u.x() - v.x()) * (u.x() - v.x()) < kMathEpsilonSqr &&
         (u.y() - v.y()) * (u.y() - v.y()) < kMathEpsilonSqr;
}

// PathPoint GetWeightedAverageOfTwoPathPoints(const PathPoint& p1,
//                                             const PathPoint& p2,
//                                             const double w1, const double
//                                             w2);

// a wrapper template function for remove_if (notice that remove_if cannot
// change the Container size)
template <class Container, class F>
void erase_where(Container& c, F&& f) {  // NOLINT
  c.erase(std::remove_if(c.begin(), c.end(), std::forward<F>(f)), c.end());
}

// a wrapper template function for remove_if on associative containers
template <class Container, class F>
void erase_map_where(Container& c, F&& f) {  // NOLINT
  for (auto it = c.begin(); it != c.end();) {
    if (f(*it)) {
      it = c.erase(it);
    } else {
      ++it;
    }
  }
}

template <typename T>
void QuaternionToRotationMatrix(const T* quat, T* R) {
  T x2 = quat[0] * quat[0];
  T xy = quat[0] * quat[1];
  T rx = quat[3] * quat[0];
  T y2 = quat[1] * quat[1];
  T yz = quat[1] * quat[2];
  T ry = quat[3] * quat[1];
  T z2 = quat[2] * quat[2];
  T zx = quat[2] * quat[0];
  T rz = quat[3] * quat[2];
  T r2 = quat[3] * quat[3];
  R[0] = r2 + x2 - y2 - z2;  // fill diagonal terms
  R[4] = r2 - x2 + y2 - z2;
  R[8] = r2 - x2 - y2 + z2;
  R[3] = 2 * (xy + rz);  // fill off diagonal terms
  R[6] = 2 * (zx - ry);
  R[7] = 2 * (yz + rx);
  R[1] = 2 * (xy - rz);
  R[2] = 2 * (zx + ry);
  R[5] = 2 * (yz - rx);
}

// Test whether two float or double numbers are equal.
// ulp: units in the last place.
template <typename T>
typename std::enable_if<!std::numeric_limits<T>::is_integer, bool>::type
IsFloatEqual(T x, T y, int ulp = 2) {
  // the machine epsilon has to be scaled to the magnitude of the values used
  // and multiplied by the desired precision in ULPs (units in the last place)
  return std::fabs(x - y) <
             std::numeric_limits<T>::epsilon() * std::fabs(x + y) * ulp
         // unless the result is subnormal
         || std::fabs(x - y) < std::numeric_limits<T>::min();
}

template <long unsigned int N, typename T>
std::string list_to_string(const T list_array) {
  if (sizeof(list_array) == 0) return std::string();
  std::stringstream ss;
  for (size_t i = 0; i < N - 1; i++) {
    ss << list_array[i] << ",";
  }
  ss << list_array[N - 1];
  std::string result;
  ss >> result;
  return result;
}

template <typename T>
std::string vec_to_string(const std::vector<T>& vec) {
  if (vec.empty()) return std::string();
  std::stringstream ss;
  for (size_t i = 0; i + 1 < vec.size(); i++) {
    ss << vec[i] << ",";
  }
  ss << vec.back();
  std::string result;
  ss >> result;
  return result;
}

// Reversed iterable

template <typename T>
struct reversion_wrapper {
  T& iterable;
};

template <typename T>
auto begin(reversion_wrapper<T> w) {
  return std::rbegin(w.iterable);
}

template <typename T>
auto end(reversion_wrapper<T> w) {
  return std::rend(w.iterable);
}

template <typename T>
reversion_wrapper<T> reverse(T&& iterable) {
  return {iterable};
}

inline const std::string getenv(const std::string& path) {
  const char* env_p = std::getenv(path.c_str());
  if (!env_p) {
    return std::string();
  }
  return std::string(env_p);
}

inline std::string trim(const std::string& source) {
  std::string s(source);
  s.erase(0, s.find_first_not_of(" \n\r\t"));
  s.erase(s.find_last_not_of(" \n\r\t") + 1);
  return s;
}

std::string to_string(const uint64_t value);
std::string to_string(const int64_t value);
std::string to_string(const uint32_t value);
std::string to_string(const int32_t value);
std::string to_string(const uint16_t value);
std::string to_string(const int16_t value);
std::string to_string(const uint8_t value);
std::string to_string(const int8_t value);
std::string to_string(const double value);

class StopWatch {
 public:
  StopWatch() { start(); }

  void start() { start_time_ = std::chrono::steady_clock::now(); }

  uint64_t elapsed_milliseconds() {
    const auto end_time = std::chrono::steady_clock::now();
    return std::chrono::duration_cast<std::chrono::milliseconds>(end_time -
                                                                 start_time_)
        .count();
  }

  uint64_t elapsed_microseconds() {
    const auto end_time = std::chrono::steady_clock::now();
    return std::chrono::duration_cast<std::chrono::microseconds>(end_time -
                                                                 start_time_)
        .count();
  }

 private:
  std::chrono::time_point<std::chrono::steady_clock> start_time_;
};

}  // namespace common
}  // namespace planning

template <typename T>
class FunctionInfo {
 public:
  typedef int (T::*Function)();
  Function function_;
  std::string fun_name_;
};

// template <typename T, size_t count>
// bool ExcuteAllFunctions(T* obj, FunctionInfo<T> fun_list[]) {
//   for (size_t i = 0; i < count; i++) {
//     if ((obj->*(fun_list[i].function_))() != apollo::cyber::SUCC) {
//       AERROR << fun_list[i].fun_name_ << " failed.";
//       return false;
//     }
//   }
//   return true;
// }

#define EXEC_ALL_FUNS(type, obj, list) \
  ExcuteAllFunctions<type, sizeof(list) / sizeof(FunctionInfo<type>)>(obj, list)

template <typename A, typename B>
std::ostream& operator<<(std::ostream& os, std::pair<A, B>& p) {
  return os << "first: " << p.first << ", second: " << p.second;
}
