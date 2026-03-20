#pragma once

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <unordered_map>
#include <utility>
#include <vector>

#include "Eigen/Dense"
#include "ad_common/math/vec2d.h"

#define HYPOT_MIN 1000.0

namespace ad_common {
namespace math {

double Sqr(const double x);

/**
 * @brief Cross product between two 2-D vectors from the common start point,
 *        and end at two other points.
 * @param start_point The common start point of two vectors in 2-D.
 * @param end_point_1 The end point of the first vector.
 * @param end_point_2 The end point of the second vector.
 *
 * @return The cross product result.
 */
double CrossProd(const Vec2d& start_point, const Vec2d& end_point_1,
                 const Vec2d& end_point_2);

/**
 * @brief Inner product between two 2-D vectors from the common start point,
 *        and end at two other points.
 * @param start_point The common start point of two vectors in 2-D.
 * @param end_point_1 The end point of the first vector.
 * @param end_point_2 The end point of the second vector.
 *
 * @return The inner product result.
 */
double InnerProd(const Vec2d& start_point, const Vec2d& end_point_1,
                 const Vec2d& end_point_2);

/**
 * @brief Cross product between two vectors.
 *        One vector is formed by 1st and 2nd parameters of the function.
 *        The other vector is formed by 3rd and 4th parameters of the function.
 * @param x0 The x coordinate of the first vector.
 * @param y0 The y coordinate of the first vector.
 * @param x1 The x coordinate of the second vector.
 * @param y1 The y coordinate of the second vector.
 *
 * @return The cross product result.
 */
double CrossProd(const double x0, const double y0, const double x1,
                 const double y1);

/**
 * @brief Inner product between two vectors.
 *        One vector is formed by 1st and 2nd parameters of the function.
 *        The other vector is formed by 3rd and 4th parameters of the function.
 * @param x0 The x coordinate of the first vector.
 * @param y0 The y coordinate of the first vector.
 * @param x1 The x coordinate of the second vector.
 * @param y1 The y coordinate of the second vector.
 *
 * @return The inner product result.
 */
double InnerProd(const double x0, const double y0, const double x1,
                 const double y1);

/**
 * @brief Wrap angle to [0, 2 * PI).
 * @param angle the original value of the angle.
 * @return The wrapped value of the angle.
 */
double WrapAngle(const double angle);

/**
 * @brief Normalize angle to [-PI, PI).
 * @param angle the original value of the angle.
 * @return The normalized value of the angle.
 */
double NormalizeAngle(const double angle);

/**
 * @brief Calculate the difference between angle from and to
 * @param from the start angle
 * @param from the end angle
 * @return The difference between from and to. The range is between [-PI, PI).
 */
double AngleDiff(const double from, const double to);

/**
 * @brief Get a random integer between two integer values by a random seed.
 * @param s The lower bound of the random integer.
 * @param t The upper bound of the random integer.
 * @param random_seed The random seed.
 * @return A random integer between s and t based on the input random_seed.
 */
int RandomInt(const int s, const int t, unsigned int rand_seed = 1);

/**
 * @brief Get a random double between two integer values by a random seed.
 * @param s The lower bound of the random double.
 * @param t The upper bound of the random double.
 * @param random_seed The random seed.
 * @return A random double between s and t based on the input random_seed.
 */
double RandomDouble(const double s, const double t, unsigned int rand_seed = 1);

/**
 * @brief Compute squared value.
 * @param value The target value to get its squared value.
 * @return Squared value of the input value.
 */
template <typename T>
inline T Square(const T value) {
  return value * value;
}

/**
 * @brief Clamp a value between two bounds.
 *        If the value goes beyond the bounds, return one of the bounds,
 *        otherwise, return the original value.
 * @param value The original value to be clamped.
 * @param bound1 One bound to clamp the value.
 * @param bound2 The other bound to clamp the value.
 * @return The clamped value.
 */
template <typename T>
T Clamp(const T value, T bound1, T bound2) {
  if (bound1 > bound2) {
    std::swap(bound1, bound2);
  }

  if (value < bound1) {
    return bound1;
  } else if (value > bound2) {
    return bound2;
  }
  return value;
}

// Gaussian
double Gaussian(const double u, const double std, const double x);

// 2-dimension Gaussian
double Gaussian2d(const double u1, const double u2, const double std1,
                  const double std2, const double x1, const double x2,
                  const double rho);

// Sigmoid
double Sigmoid(const double x);

// Rotate a 2d vector counter-clockwise by theta
Eigen::Vector2d RotateVector2d(const Eigen::Vector2d& v_in, const double theta);

inline std::pair<double, double> RFUToFLU(const double x, const double y) {
  return std::make_pair(y, -x);
}

inline std::pair<double, double> FLUToRFU(const double x, const double y) {
  return std::make_pair(-y, x);
}

inline void L2Norm(int feat_dim, float* feat_data) {
  if (feat_dim == 0) {
    return;
  }
  // feature normalization
  float l2norm = 0.0f;
  for (int i = 0; i < feat_dim; ++i) {
    l2norm += feat_data[i] * feat_data[i];
  }
  if (l2norm == 0) {
    float val = 1.f / std::sqrt(static_cast<float>(feat_dim));
    for (int i = 0; i < feat_dim; ++i) {
      feat_data[i] = val;
    }
  } else {
    l2norm = std::sqrt(l2norm);
    for (int i = 0; i < feat_dim; ++i) {
      feat_data[i] /= l2norm;
    }
  }
}

// Cartesian coordinates to Polar coordinates
std::pair<double, double> Cartesian2Polar(double x, double y);

template <class T>
class IntervalMethodSolution {
 public:
  using Interval = std::pair<T, T>;
  static bool cmp(Interval a, Interval b) { return a.first < b.first; }
  std::vector<Interval> merge(std::vector<Interval>& intervals) {
    if (intervals.size() <= 1) {
      return intervals;
    }
    std::sort(intervals.begin(), intervals.end(), cmp);
    std::vector<Interval> res;
    res.push_back(intervals[0]);
    for (size_t i = 0; i < intervals.size(); i++) {
      if (res.back().second < intervals[i].first) {
        res.push_back(intervals[i]);
      } else {
        res.back().second = std::max(res.back().second, intervals[i].second);
      }
    }
    return res;
  }

  std::vector<Interval> intersect(const std::vector<Interval>& intervals,
                                  const Interval& section) {
    std::vector<Interval> res;
    for (auto& interval : intervals) {
      if (interval.second < section.first || interval.first > section.second) {
        continue;
      }
      res.push_back({std::max(interval.first, section.first),
                     std::min(interval.second, section.second)});
    }
    return res;
  }

  std::vector<Interval> intersect(const std::vector<Interval>& a,
                                  const std::vector<Interval>& b) {
    size_t i = 0;
    size_t j = 0;
    std::vector<Interval> result;
    while (i < a.size() && j < b.size()) {
      auto lo = std::max(a[i].first, b[j].first);
      auto hi = std::min(a[i].second, b[j].second);
      if (lo < hi) {
        result.push_back({lo, hi});
      }
      if (a[i].second < b[j].second) {
        i++;
      } else if (a[i].second > b[j].second) {
        j++;
      } else {
        i++;
        j++;
      }
    }
    return result;
  }

  void filter(std::vector<Interval>& intervals, T lower_length,
              T upper_length) {
    for (auto itor_s = intervals.begin(); itor_s != intervals.end();) {
      auto cur_length = std::abs(itor_s->second - itor_s->first);
      if (cur_length < lower_length || cur_length > upper_length) {
        itor_s = intervals.erase(itor_s);
      } else {
        ++itor_s;
      }
    }
  }

  std::vector<Interval> complement(const std::vector<Interval>& intervals,
                                   T lower_bound, T upper_bound) {
    std::vector<Interval> result;
    T sweep_line = lower_bound;
    for (auto& interval : intervals) {
      if (sweep_line >= upper_bound) {
        return result;
      }
      // if (internal.first >= upper_bound) {
      //   return result;
      // }
      if (sweep_line < interval.first) {
        result.push_back({sweep_line, std::min(upper_bound, interval.first)});
      }
      if (interval.second > sweep_line) {
        sweep_line = interval.second;
      }
    }
    if (sweep_line < upper_bound) {
      result.push_back({sweep_line, upper_bound});
    }
    return result;
  }

  T calculate_length(std::vector<Interval>& intervals) {
    T length;
    for (auto itor_s = intervals.begin(); itor_s != intervals.end(); itor_s++) {
      length += std::abs(itor_s->second - itor_s->first);
    }
    return length;
  }

  bool is_in_intervals(const std::vector<Interval>& intervals, T t) {
    for (auto& interval : intervals) {
      if (t < interval.second && t > interval.first) {
        return true;
      }
    }
    return false;
  }
};

double Interpolate(double x1, double y1, double x2, double y2, double x);

double Interpolate(double y1, double y2, double ratio);

double InterpolateAngle(double x1, double y1, double x2, double y2, double x);

double InterpolateAngle(double y1, double y2, double ratio);

inline double fast_hypot(double a, double b) {
  if (a <= HYPOT_MIN && b <= HYPOT_MIN) {
    return std::sqrt(a * a + b * b);
  }
  return std::hypot(a, b);
}

// 基于倍增算法的后缀数组构建
inline std::vector<int> build_sa(const std::vector<int>& s) {
  int n = s.size();
  std::vector<int> sa(n), rnk(n), tmp(n);

  for (int i = 0; i < n; ++i) {
    sa[i] = i;
    rnk[i] = s[i];
  }

  for (int k = 1;; k <<= 1) {
    auto cmp = [&](int i, int j) {
      if (rnk[i] != rnk[j]) return rnk[i] < rnk[j];
      int ri = (i + k < n) ? rnk[i + k] : -1;
      int rj = (j + k < n) ? rnk[j + k] : -1;
      return ri < rj;
    };

    std::sort(sa.begin(), sa.end(), cmp);

    tmp[sa[0]] = 0;
    for (int i = 1; i < n; ++i) {
      tmp[sa[i]] = tmp[sa[i - 1]] + (cmp(sa[i - 1], sa[i]) ? 1 : 0);
    }

    for (int i = 0; i < n; ++i) {
      rnk[i] = tmp[i];
    }

    if (rnk[sa[n - 1]] == n - 1) break;
  }
  return sa;
}

// 后缀数组检索重复子串
inline std::vector<int> build_lcp(const std::vector<int>& s,
                                  const std::vector<int>& sa) {
  int n = s.size();
  std::vector<int> rank(n);
  std::vector<int> lcp(n - 1);

  for (int i = 0; i < n; ++i) rank[sa[i]] = i;

  int h = 0;
  for (int i = 0; i < n; ++i) {
    int r = rank[i];
    if (r == 0) continue;

    int j = sa[r - 1];
    while (i + h < n && j + h < n && s[i + h] == s[j + h]) ++h;

    lcp[r - 1] = h;
    if (h > 0) --h;
  }
  return lcp;
}

inline bool ConfirmTargetIndex(const std::vector<uint64_t>& old_sequence,
                               const std::vector<uint64_t>& new_sequence,
                               const int& source_index, int& target_index) {
  if (static_cast<int>(old_sequence.size()) <= source_index ||
      new_sequence.empty()) {
    return false;
  }
  std::unordered_map<uint64_t, std::pair<size_t, size_t>> optional_index_map;
  optional_index_map.reserve(new_sequence.size());
  for (size_t i = 0; i < new_sequence.size(); ++i) {
    if (new_sequence[i] == old_sequence[source_index]) {
      optional_index_map.insert(std::make_pair(i, std::make_pair(0, 0)));
      /*
        first for front matching degree, second for back matching degree
      */
    }
  }
  if (optional_index_map.empty()) {
    return false;
  }
  if (optional_index_map.size() == 1) {
    target_index = optional_index_map.begin()->first;
    return true;
  }

  // 向后检索
  for (size_t si = source_index; si < old_sequence.size() - 1; ++si) {
    const auto& old_element = old_sequence[si + 1];
    // 计算当前比较元素在 new_sequence 中的相对偏移量 (si 是当前 old_sequence
    // 的遍历索引)
    const size_t offset = si + 1 - source_index;

    for (auto& optional_index : optional_index_map) {
      if (optional_index.first + offset >= new_sequence.size()) {
        continue;
      }
      if (new_sequence[optional_index.first + offset] != old_element) {
        continue;
      }
      optional_index.second.second = offset;
    }
  }

  // 向前检索
  for (size_t si = source_index; si > 0; --si) {
    const auto& old_element = old_sequence[si - 1];
    // 计算当前比较元素在 new_sequence 中的相对偏移量
    const size_t offset = source_index - (si - 1);

    for (auto& optional_index : optional_index_map) {
      if (optional_index.first < offset) {
        continue;
      }
      if (new_sequence[optional_index.first - offset] != old_element) {
        continue;
      }
      optional_index.second.first = offset;
    }
  }

  // 寻找最佳匹配index，策略：以back优先级最高，其次front，最终选择index最小的
  size_t max_back_match = 0;
  size_t max_front_match = 0;
  int best_index = -1;

  for (const auto& entry : optional_index_map) {
    const int index = entry.first;
    const size_t front_match = entry.second.first;
    const size_t back_match = entry.second.second;

    if (back_match > max_back_match) {
      max_back_match = back_match;
      max_front_match = front_match;
      best_index = index;
    } else if (back_match == max_back_match) {
      if (front_match > max_front_match) {
        max_front_match = front_match;
        best_index = index;
      } else if (front_match == max_front_match) {
        if (best_index == -1 || index < best_index) {
          best_index = index;
        }
      }
    }
  }

  if (best_index != -1) {
    target_index = best_index;
    return true;
  }

  return false;
}

// 动态规划找到两个数组的最大子串
// 两个数组中可能存在重复元素，但是重复元素不连续
struct SubStringInfo {
  SubStringInfo(int max_len, int a_end_idx, int b_end_idx)
      : max_len(max_len), a_end_idx(a_end_idx), b_end_idx(b_end_idx) {}
  int max_len = 0;
  int a_end_idx = -1;
  int b_end_idx = -1;
};
inline bool SolveLongestSubString(
    const std::vector<uint64_t>& A, const std::vector<uint64_t>& B,
    std::vector<SubStringInfo>& sub_string_infos) {
  if (A.empty() || B.empty()) {
    // 数组为空不进行匹配
    return false;
  }
  sub_string_infos.clear();

  // 初始化dp数组
  std::vector<std::vector<int>> dp(A.size(), std::vector<int>(B.size(), 0));

  /*
    动态规划：
    边界：
      dp[i][0] = {A[i] == B[0] ? 1 : 0}
      dp[0][j] = {A[0] == B[j] ? 1 : 0}
    状态转移方程、最优子结构：
      dp[i][j] = {A[i] == B[j] ? dp[i-1][j-1] + 1 : 0, dp[i][j]}
    构建的过程中将最大值的序列找出来
  */
  int max_len = 0;
  for (int i = 0; i < static_cast<int>(A.size()); ++i) {
    for (int j = 0; j < static_cast<int>(B.size()); ++j) {
      if (A[i] == B[j]) {
        dp[i][j] = (i == 0 || j == 0) ? 1 : dp[i - 1][j - 1] + 1;
        if (dp[i][j] > max_len) {
          sub_string_infos.clear();
          max_len = dp[i][j];
          sub_string_infos.push_back(SubStringInfo(max_len, i, j));
        } else if (dp[i][j] == max_len && max_len != 0) {
          sub_string_infos.push_back(SubStringInfo(max_len, i, j));
        }
      } else {
        dp[i][j] = 0;
      }
    }
  }

  return true;
}

}  // namespace math
}  // namespace ad_common
