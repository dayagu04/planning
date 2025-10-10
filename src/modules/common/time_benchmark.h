#pragma once

namespace planning {

enum TimeBenchmarkType : int32_t {
  TB_APA_TOTAL_TIME = 0,
  TB_APA_SLOT_MANAGER_TIME = 1,
  TB_APA_QP_TIME = 2,
  TB_APA_DP_TIME = 3,
  TB_APA_JLT_TIME = 4,
  TB_APA_STOP_DECIDER = 5,
  TB_APA_SPEED_LIMIT_DECIDER = 6,
  TB_APA_ASTAR = 7,
  TB_PLANNING_TOTAL = 8,
  TB_OSQP = 9,
  TB_MAX,
};

class TimeBenchmarkItem {
 public:
  TimeBenchmarkItem() = default;
  TimeBenchmarkItem(const TimeBenchmarkItem&) = default;

  TimeBenchmarkItem(const double time) : time_ms_(time) {}

 public:
  double time_ms_;
  std::string name_;
};

class TimeBenchmark {
 public:
  void Clear() {
    for (int32_t i = 0; i < TimeBenchmarkType::TB_MAX; i++) {
      times[i].time_ms_ = 0;
    }
    return;
  }

  static TimeBenchmark& Instance() {
    static TimeBenchmark benchmark_;
    return benchmark_;
  }

  void SetTime(const TimeBenchmarkType type, const double time) {
    times[type].time_ms_ = time;

    return;
  }

  void DebugString() {
    for (int32_t i = 0; i < TimeBenchmarkType::TB_MAX; i++) {
      ILOG_INFO << times[i].name_ << "(" << times[i].time_ms_ << ")";
    }

    return;
  }

  void InitName() {
    times[TimeBenchmarkType::TB_APA_DP_TIME].name_ = "dp optimizer time";
    times[TimeBenchmarkType::TB_APA_SLOT_MANAGER_TIME].name_ =
        "slot manager time";
    times[TimeBenchmarkType::TB_APA_QP_TIME].name_ = "speed qp time";
    times[TimeBenchmarkType::TB_APA_JLT_TIME].name_ = "jlt optimizer time";
    times[TimeBenchmarkType::TB_APA_TOTAL_TIME].name_ = "apa_total_time";
    times[TimeBenchmarkType::TB_APA_STOP_DECIDER].name_ = "apa_stop_decider";
    times[TimeBenchmarkType::TB_APA_SPEED_LIMIT_DECIDER].name_ =
        "speed_limit_decider";

    times[TimeBenchmarkType::TB_APA_ASTAR].name_ = "astar_time";
    times[TimeBenchmarkType::TB_PLANNING_TOTAL].name_ = "planning_total_time";
    times[TimeBenchmarkType::TB_OSQP].name_ = "osqp_time";
    return;
  }

 public:
  TimeBenchmarkItem times[TimeBenchmarkType::TB_MAX];
};

}  // namespace planning