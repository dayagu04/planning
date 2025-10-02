#include "mean_filter.h"

#include <limits>

namespace planning {
namespace planning_math {

using uint8 = std::uint_fast8_t;
using TimedValue = std::pair<uint8, double>;

MeanFilter::MeanFilter(const uint8 window_size) : window_size_(window_size) {
  initialized_ = true;
}

double MeanFilter::GetMin() const {
  if (min_candidates_.empty()) {
    return std::numeric_limits<double>::infinity();
  } else {
    return min_candidates_.front().second;
  }
}

double MeanFilter::GetMax() const {
  if (max_candidates_.empty()) {
    return -std::numeric_limits<double>::infinity();
  } else {
    return max_candidates_.front().second;
  }
}

double MeanFilter::Update(const double measurement) {
  ++time_;
  time_ %= static_cast<std::uint_fast8_t>(2 * window_size_);
  if (values_.size() == window_size_) {
    RemoveEarliest();
  }
  Insert(measurement);
  if (values_.size() > 2) {
    mean_value_ =
        (sum_ - GetMin() - GetMax()) /
        static_cast<double>(values_.size() - 2);
    return mean_value_;
  } else {
    mean_value_ =
        sum_ / static_cast<double>(values_.size());
    return mean_value_;
  }
}

bool MeanFilter::ShouldPopOldestCandidate(const uint8 old_time) const {
  if (old_time < window_size_) {
    return old_time + window_size_ == time_;
  } else if (time_ < window_size_) {
    return old_time == time_ + window_size_;
  } else {
    return false;
  }
}

void MeanFilter::RemoveEarliest() {
  double removed = values_.front();
  values_.pop_front();
  sum_ -= removed;
  if (ShouldPopOldestCandidate(min_candidates_.front().first)) {
    min_candidates_.pop_front();
  }
  if (ShouldPopOldestCandidate(max_candidates_.front().first)) {
    max_candidates_.pop_front();
  }
}

void MeanFilter::Insert(const double value) {
  values_.push_back(value);
  sum_ += value;
  while (min_candidates_.size() > 0 && min_candidates_.back().second > value) {
    min_candidates_.pop_back();
  }
  min_candidates_.push_back(std::make_pair(time_, value));
  while (max_candidates_.size() > 0 && max_candidates_.back().second < value) {
    max_candidates_.pop_back();
  }
  max_candidates_.push_back(std::make_pair(time_, value));
}

bool MeanFilter::Reset() {
  sum_ = 0.0;
  mean_value_ = 0.0;
  time_ = 0;
  values_.clear();
  min_candidates_.clear();
  max_candidates_.clear();
  return true;
}

}  // namespace planning_math
}  // namespace planning