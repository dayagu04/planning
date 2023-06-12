/***************************************************************
 * @file       window_lib.hpp
 * @brief      for window lib design
 * @author     xiaoliang.wang
 * @version    v0.0
 * @date       May-06-2022
 **************************************************************/

#ifndef __WINDOW_LIB__
#define __WINDOW_LIB__

#include <cmath>
#include <queue>
#include <vector>
#include "controller/common/math.h"
#include "math_lib.h"

namespace pnc {
namespace windows {
#define eps 1e-6
template <typename T>
class CircleQueue {
 public:
  bool Init(int size) {
    if (size < 0) {
      return false;
    }

    extended_flag_ = true;
    div_ = 1;

    size_ = size;
    Reset();

    return true;
  }

  bool Init(int size, bool extended_flag) {
    if (size < 0) {
      return false;
    }

    extended_flag_ = extended_flag;
    div_ = 1;
    size_ = size;
    Reset();

    return true;
  }

  bool Init(int size, int div) {
    if (size < 0) {
      return false;
    }

    extended_flag_ = true;
    div_ = div;

    size_ = size;
    Reset();

    return true;
  }

  bool Reset() {
    data_.clear();
    data_.resize(size_);
    front_index_ = -1;
    back_index_ = -1;
    div_count_ = std::max(div_ - 1, 0);
    count_ = 0;
    sum_ = {};
    mean_ = {};
    sq_sum_ = {};
    rms_ = {};
    var_ = {};
    std_var_ = {};
    max_ = {};
    min_ = {};
    peak2peak_ = {};

    return true;
  }

  T Front() {
    if (IsEmpty()) {
      return -1;
    } else {
      return data_[front_index_];
    }
  }

  T Back() {
    if (IsEmpty()) {
      return -1;
    } else {
      return data_[back_index_];
    }
  }

  int Size() { return size_; }

  bool Push(T u) {
    if (IsFull()) {
      return false;
    }

    count_++;

    if (IsEmpty()) {
      front_index_ = 0;
      back_index_ = 0;
    } else {
      back_index_ = (back_index_ + 1) % size_;
    }

    data_[back_index_] = u;

    return true;
  }

  void Pop() {
    if (!IsEmpty()) {
      if ((back_index_ == front_index_) && (back_index_ != -1)) {
        front_index_ = back_index_ = -1;
      } else {
        front_index_ = (front_index_ + 1) % size_;
      }
      count_--;
    }
  }

  void Update(T u) {
    if ((++div_count_) < div_) {
      return;
    } else {
      div_count_ = 0;
    }

    if (IsFull()) {
      sum_ -= Front();
      sq_sum_ -= Front() * Front();
      Pop();
    }

    Push(u);

    sum_ += u;
    sq_sum_ += u * u;
    mean_ = sum_ / count_;
    rms_ = std::sqrt(sq_sum_ / count_);
    var_ = sq_sum_ / count_ - mean_ * mean_;
    std_var_ = std::sqrt(var_);

    if (extended_flag_) {
      T min = data_[0];
      T max = data_[0];
      for (int i = 1; i < count_; ++i) {
        if (data_[i] < min) {
          min = data_[i];
        }
        if (data_[i] > max) {
          max = data_[i];
        }
      }
      min_ = min;
      max_ = max;
      peak2peak_ = max - min;
    } else {
      min_ = {};
      max_ = {};
      peak2peak_ = {};
    }
  }

  bool IsEmpty() { return ((back_index_ == front_index_) && (back_index_ == -1)); }

  bool IsFull() { return (front_index_ == (back_index_ + 1) % size_); }

  T At(int index) { return data_[(front_index_ + index) % size_]; }

  T GetSum() { return sum_; }
  T GetMean() { return mean_; }
  T GetRms() { return rms_; }
  T GetVar() { return var_; }
  T GetStdVar() { return std_var_; }
  T GetMax() { return max_; }
  T GetMin() { return min_; }
  T GetPeak2Peak() { return peak2peak_; }

 private:
  std::vector<T> data_ = {};
  int size_ = 0;
  int front_index_ = -1;
  int back_index_ = -1;
  int count_ = 0;
  int div_ = 1;
  int div_count_ = 0;
  bool extended_flag_ = false;
  T sum_{};
  T mean_{};
  T sq_sum_{};
  T rms_{};
  T var_{};
  T std_var_{};
  T peak2peak_{};
  T max_{};
  T min_{};
};

template <typename T>
class SlidingWindow {
 public:
  void Init(int size) {
    size_ = size;
    Reset();
  }

  void Init(int size, int div) {
    size_ = size;
    div_ = div;
    Reset();
  }

  void Reset() {
    count_ = 0;
    div_count_ = std::max(div_ - 1, 0);
    sum_ = {};
    mean_ = {};
    sq_sum_ = {};
    rms_ = {};
    var_ = {};
    std_var_ = {};
    max_ = {};
    min_ = {};
    peak2peak_ = {};

    std::queue<T> empty_queue;
    std::swap(empty_queue, data_);
    std::deque<T> empty_deque;
    std::swap(empty_deque, data_max_);
    std::swap(empty_deque, data_min_);
  }

  bool Push(T u) {
    if (IsFull()) {
      return false;
    }

    count_++;
    data_.push(u);
    while (!data_max_.empty() && u >= data_max_.back()) {
      data_max_.pop_back();
    }
    data_max_.push_back(u);

    while (!data_min_.empty() && u <= data_min_.back()) {
      data_min_.pop_back();
    }
    data_min_.push_back(u);

    return true;
  }

  void Pop() {
    if (!data_.empty()) {
      count_--;
      T temp = data_.front();
      data_.pop();
      if (std::fabs(temp - data_max_.front()) <= eps) {
        data_max_.pop_front();
      }
      if (std::fabs(temp - data_min_.front()) <= eps) {
        data_min_.pop_front();
      }
    }
  }

  bool IsEmpty() { return (count_ == 0); }
  bool IsFull() { return (count_ == size_); }

  void Update(T u) {
    if ((++div_count_) < div_) {
      return;
    } else {
      div_count_ = 0;
    }

    if (IsFull()) {
      sum_ -= data_.front();
      sq_sum_ -= data_.front() * data_.front();
      Pop();
    }

    auto out = Push(u);
    UNUSED(out);

    sum_ += u;
    sq_sum_ += u * u;
    mean_ = sum_ / count_;
    rms_ = std::sqrt(sq_sum_ / count_);
    var_ = sq_sum_ / count_ - mean_ * mean_;
    std_var_ = std::sqrt(var_);

    if (!data_max_.empty()) {
      max_ = data_max_.front();
    }

    if (!data_min_.empty()) {
      min_ = data_min_.front();
    }

    peak2peak_ = max_ - min_;
  }

  T GetSum() { return sum_; }
  T GetMean() { return mean_; }
  T GetRms() { return rms_; }
  T GetVar() { return var_; }
  T GetStdVar() { return std_var_; }
  T GetMax() { return max_; }
  T GetMin() { return min_; }
  T GetPeak2Peak() { return peak2peak_; }

 private:
  int size_ = 0;
  int count_ = 0;
  int div_ = 1;
  int div_count_ = 0;
  std::queue<T> data_;
  std::deque<T> data_max_;
  std::deque<T> data_min_;
  T sum_{};
  T mean_{};
  T sq_sum_{};
  T rms_{};
  T var_{};
  T std_var_{};
  T peak2peak_{};
  T max_{};
  T min_{};
};

}  // namespace windows
}  // namespace pnc

#endif
