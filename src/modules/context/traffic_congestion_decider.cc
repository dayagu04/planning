#include "traffic_congestion_decider.h"
#include <limits>
#include "obstacle.h"
#include "environmental_model.h"

namespace planning {

CongestionDetector::CongestionDetector(
    const CongestionDetectionConfig* config_builder,
    framework::Session *session,
    const int fix_lane_id) {
  // 初始化参考路径
  session_ = session;
  congestion_config_ = *config_builder;
  fix_lane_id_ = fix_lane_id;
  Init();
}

void CongestionDetector::Init() {
  const auto &dynamic_world =
      session_->environmental_model().get_dynamic_world();
  const auto& nodes = dynamic_world->GetNodesByLaneId(fix_lane_id_);
  if(nodes.empty()) {
    return;
  }

  agent_node_.clear();
  for(const auto& node : nodes) {
    // 检查是否找到
    agent_node_.push_back(node);
  }

  // 按车辆s的顺序排序
  std::sort(agent_node_.begin(), agent_node_.end(),
            [](const planning_data::DynamicAgentNode* a,
               const planning_data::DynamicAgentNode* b) {
              return a->node_s() < b->node_s();
            });
}

CongestionResult CongestionDetector::DetectLaneCongestion() {
  // 初始化结果
  CongestionResult result;

  // 检查车道是否有效
  if (agent_node_.empty()) {
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
  result.level =
      CalculateCongestionScore(result.density, result.avg_speed,
                               result.speed_deviation, congestion_config_);

  return result;
}

double CongestionDetector::CalculateDensity() const {

    // 计算车道内车辆密度（辆/公里）
    double effective_length =
        agent_node_.back()->node_s() - agent_node_.front()->node_s();

    return (agent_node_.size() / effective_length) * 1000.0; // 转换为辆/公里
}

double CongestionDetector::CalculateAverageSpeed() const {
    double sum_speed = 0.0;

    for (const auto& obstacle : agent_node_) {
        sum_speed += obstacle->node_speed();
    }

    return sum_speed / agent_node_.size();
}

double CongestionDetector::CalculateSpeedDeviation(
    double avg_speed) const {

    double sum_squared_diff = 0.0;
    for (const auto& obstacle : agent_node_) {
        double diff = obstacle->node_speed() - avg_speed;
        sum_squared_diff += diff * diff;
    }

    return std::sqrt(sum_squared_diff / (agent_node_.size() - 1));
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

  for (size_t i = 1; i < agent_node_.size(); ++i) {
    const auto& front = agent_node_[i];
    const auto& rear = agent_node_[i-1];

    // 计算两车间距
    double distance = front->node_s() - rear->node_s() -
                      front->node_length() / 2 - rear->node_length() / 2;

    // 避免除零
    if (rear->node_speed() > 0.1) {
      // 时间头距 = 间距 / 后车速度
      total_headway += distance / rear->node_speed();
      valid_pairs++;
    }
  }

  return valid_pairs > 0 ? total_headway / valid_pairs : 0.0;
}

double CongestionDetector::CalculateAverageSpaceHeadway() const {

  double total_space = 0.0;

  for (size_t i = 1; i < agent_node_.size(); ++i) {
    const auto& front = agent_node_[i];
    const auto& rear = agent_node_[i-1];

    // 车间距 = 两车距离 - 前车长度/2 - 后车长度/2
    double distance = front->node_s() - rear->node_s();
    double space = distance - front->node_length() / 2 - rear->node_length() / 2;

    total_space += space;
  }

  return total_space / (agent_node_.size() - 1);
}

} // namespace autopilot
