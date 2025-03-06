#include "speed_limit_profile.h"

#include <algorithm>
#include <cstddef>

#include "log_glog.h"

namespace planning {
void SpeedLimitProfile::AppendSpeedLimit(const double s, const double v) {
  if (!speed_limit_points_.empty()) {
    if (s < speed_limit_points_.back().first) {
      return;
    }
  }
  speed_limit_points_.emplace_back(s, v);

  return;
}

const std::vector<std::pair<double, double>>&
SpeedLimitProfile::SpeedLimitPoints() const {
  return speed_limit_points_;
}

double SpeedLimitProfile::GetSpeedLimitByS(const double s) const {
  if (speed_limit_points_.empty()) {
    ILOG_INFO << "no points";
    return 10.0;
  }

  if (s > speed_limit_points_.back().first) {
    return speed_limit_points_.back().first;
  }

  if (s < speed_limit_points_.front().first) {
    return speed_limit_points_.front().first;
  }

  auto compare_s = [](const std::pair<double, double>& point, const double s) {
    return point.first < s;
  };

  auto it_lower = std::lower_bound(speed_limit_points_.begin(),
                                   speed_limit_points_.end(), s, compare_s);

  if (it_lower == speed_limit_points_.end()) {
    return (it_lower - 1)->second;
  }
  return it_lower->second;
}

const size_t SpeedLimitProfile::GetIndexByS(const double s) {
  if (speed_limit_points_.size() < 1) {
    return 0;
  }

  if (s > speed_limit_points_.back().first) {
    return speed_limit_points_.size() - 1;
  }
  if (s < speed_limit_points_.begin()->first) {
    return 0;
  }

  auto compare_s = [](const std::pair<double, double>& point, const double s) {
    return point.first < s;
  };

  auto it_lower = std::lower_bound(speed_limit_points_.begin(),
                                   speed_limit_points_.end(), s, compare_s);
  if (it_lower == speed_limit_points_.end()) {
    return speed_limit_points_.size() - 1;
  }

  size_t index = std::distance(speed_limit_points_.begin(), it_lower);

  return index;
}

void SpeedLimitProfile::Clear() { speed_limit_points_.clear(); }

void SpeedLimitProfile::DebugString() {
  for (size_t i = 0; i < speed_limit_points_.size(); i++) {
    ILOG_INFO << "s: " << speed_limit_points_[i].first << ", v "
              << speed_limit_points_[i].second;
  }

  return;
}

void SpeedLimitProfile::SetSpeedLimitByIndex(const int index, const double v) {
  if (index < 0 || index >= speed_limit_points_.size()) {
    return;
  }

  speed_limit_points_[index].second = v;
}

double SpeedLimitProfile::GetSpeedLimitByRange(const double range_start_s,
                                               const double range_end_s) const {
  if (speed_limit_points_.empty()) {
    ILOG_INFO << "no points";
    return 10.0;
  }

  if (range_start_s > speed_limit_points_.back().first) {
    return speed_limit_points_.back().first;
  }

  if (range_end_s < speed_limit_points_.front().first) {
    return speed_limit_points_.front().first;
  }

  bool find_point = false;
  double min_speed = 100.0;
  for (size_t i = 0; i < speed_limit_points_.size(); i++) {
    if (speed_limit_points_[i].first <= range_end_s &&
        speed_limit_points_[i].first >= range_start_s) {
      min_speed = std::min(min_speed, speed_limit_points_[i].second);
      find_point = true;
    }
  }

  if (!find_point) {
    auto compare_s = [](const std::pair<double, double>& point,
                        const double s) { return point.first < s; };

    auto it_lower =
        std::lower_bound(speed_limit_points_.begin(), speed_limit_points_.end(),
                         (range_start_s + range_end_s) / 2.0, compare_s);

    if (it_lower == speed_limit_points_.end()) {
      min_speed = (it_lower - 1)->second;
    } else {
      min_speed = it_lower->second;
    }
  }

  return min_speed;
}

}  // namespace planning
