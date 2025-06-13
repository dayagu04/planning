#pragma once

#include <vector>
#include <deque>
#include <memory>
#include <cmath>
#include <algorithm>
#include <Eigen/Dense>
#include "ego_planning_config.h"
#include "reference_path.h"

namespace planning {

// 拥堵状态检测器
class CongestionDetector {
public:
    CongestionDetector(const CongestionDetectionConfig *config_builder,
        const std::shared_ptr<ReferencePath> ref_path);
    ~ CongestionDetector() = default;

    void Init();

    // 检测单车道拥堵状态
    CongestionResult DetectLaneCongestion();

private:
    std::shared_ptr<ReferencePath> ref_path_;

    std::vector<std::shared_ptr<FrenetObstacle>> obstacles_;

    CongestionDetectionConfig congestion_config_;

    // 计算车辆密度（辆/公里）
    double CalculateDensity() const;

    // 计算平均速度（m/s）
    double CalculateAverageSpeed() const;

    // 计算速度标准差（m/s）
    double CalculateSpeedDeviation(double avg_speed) const;

    // 计算平均车头时距
    double CalculateAverageTimeHeadway() const;

    // 计算平均车间距
    double CalculateAverageSpaceHeadway() const;

    // 根据多指标确定拥堵等级
    CongestionLevel DetermineCongestionLevel(
        double density,
        double avg_speed,
        double speed_deviation,
        double avg_time_headway,
        double avg_space_headway) const;
    double MapToScore(
      double value,
      double standard_val,
      double deviation_factor,
      bool is_lower_better);
    CongestionLevel CalculateCongestionScore(
        double density,
        double avg_speed,
        double speed_deviation,
        const CongestionDetectionConfig& config);
};

} // namespace autopilot