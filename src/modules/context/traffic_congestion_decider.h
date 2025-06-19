#pragma once

#include <vector>
#include <deque>
#include <memory>
#include <cmath>
#include <algorithm>
#include <Eigen/Dense>
#include "ego_planning_config.h"
#include "reference_path.h"
#include "session.h"
#include "dynamic_world/dynamic_agent_node.h"

namespace planning {

// 拥堵状态检测器
class CongestionDetector {
public:
    CongestionDetector(const CongestionDetectionConfig *config_builder,
        framework::Session *session,
        const int fix_lane_id);
    ~ CongestionDetector() = default;

    void Init();

    // 检测单车道拥堵状态
    CongestionResult DetectLaneCongestion();

private:
    // std::shared_ptr<ReferencePath> ref_path_;
    framework::Session * session_;

    int fix_lane_id_;

    std::vector<const planning_data::DynamicAgentNode*> agent_node_;

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