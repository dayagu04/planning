#include "traffic_congestion_decider.h"
#include <limits>
#include "obstacle.h"

namespace planning {

CongestionDetector::CongestionDetector(
    const CongestionDetectionConfig* config_builder,
    const std::shared_ptr<ReferencePath> ref_path) {
  // 初始化参考路径
  ref_path_ = ref_path;
  congestion_config_ = *config_builder;
  Init();
}

void CongestionDetector::Init() {
  const auto& obstacles = ref_path_->get_obstacles();
  for(const auto& obstacle : obstacles) {
    //目前在计算目标车道的拥堵程度时，只考虑自车前后一定范围内的车流状况
    if (obstacle->d_s_rel() < 120.0 &&
        obstacle->d_s_rel() > -30.0) {
      obstacles_.push_back(obstacle);
    }
  }

  // 按车辆s的顺序排序
  std::sort(obstacles_.begin(), obstacles_.end(),
            [](const std::shared_ptr<FrenetObstacle> a,
               const std::shared_ptr<FrenetObstacle> b) {
              return a->frenet_s() < b->frenet_s();
            });
}

CongestionResult CongestionDetector::DetectLaneCongestion() {
    // 初始化结果
    CongestionResult result;

    // 检查车道是否有效
    if (obstacles_.empty()) {
        result.level = CongestionLevel::FREE_FLOW;
        return result;
    }

    // 1. 计算车辆密度
    result.density = CalculateDensity();

    // 2. 计算平均速度
    result.avg_speed = CalculateAverageSpeed();

    // 3. 计算速度标准差
    result.speed_deviation = CalculateSpeedDeviation( result.avg_speed);

    // 5. 确定拥堵等级
    result.level = CalculateCongestionScore(result.density, result.avg_speed, result.speed_deviation, congestion_config_);

    return result;
}

double CongestionDetector::CalculateDensity() const {

    // 计算车道内车辆密度（辆/公里）
    double effective_length =
        obstacles_.back()->frenet_s() - obstacles_.front()->frenet_s();

    return (obstacles_.size() / effective_length) * 1000.0; // 转换为辆/公里
}

double CongestionDetector::CalculateAverageSpeed() const {
    double sum_speed = 0.0;

    for (const auto& obstacle : obstacles_) {
        sum_speed += obstacle->velocity();
    }

    return sum_speed / obstacles_.size();
}

double CongestionDetector::CalculateSpeedDeviation(
    double avg_speed) const {

    double sum_squared_diff = 0.0;
    for (const auto& obstacle : obstacles_) {
        double diff = obstacle->velocity() - avg_speed;
        sum_squared_diff += diff * diff;
    }

    return std::sqrt(sum_squared_diff / (obstacles_.size() - 1));
}

CongestionLevel CongestionDetector::CalculateCongestionScore(
    double density,
    double avg_speed,
    double speed_deviation,
    const CongestionDetectionConfig& config) {

    if (density > config.heavy_density &&
        avg_speed < config.jam_speed &&
        speed_deviation < config.speed_deviation) {
        return CongestionLevel::CONGESTION;
    }

    return CongestionLevel::FREE_FLOW;
}

double CongestionDetector::CalculateAverageTimeHeadway() const {
  double total_headway = 0.0;
  int valid_pairs = 0;

  for (size_t i = 1; i < obstacles_.size(); ++i) {
    const auto& front = obstacles_[i];
    const auto& rear = obstacles_[i-1];

    // 计算两车间距
    double distance = front->frenet_s() - rear->frenet_s() -
                      front->length() / 2 - rear->length() / 2;

    // 避免除零
    if (rear->velocity() > 0.1) {
      // 时间头距 = 间距 / 后车速度
      total_headway += distance / rear->velocity();
      valid_pairs++;
    }
  }

  return valid_pairs > 0 ? total_headway / valid_pairs : 0.0;
}

double CongestionDetector::CalculateAverageSpaceHeadway() const {

  double total_space = 0.0;

  for (size_t i = 1; i < obstacles_.size(); ++i) {
    const auto& front = obstacles_[i];
    const auto& rear = obstacles_[i-1];

    // 车间距 = 两车距离 - 前车长度/2 - 后车长度/2
    double distance = front->frenet_s() - rear->frenet_s();
    double space = distance - front->length() / 2 - rear->length() / 2;

    total_space += space;
  }

  return total_space / (obstacles_.size() - 1);
}

} // namespace autopilot
