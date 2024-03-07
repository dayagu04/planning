#pragma once

#include <stdexcept>
#include <string>

namespace planning {

enum class StatusCode {
  OK,
  COMMON_ERROR,
  PLANNING_ERROR,
  ONLINE_MAP_ERROR,
  SIM_ERROR,
  OSQP_SETUP_ERROR,
  SPEED_PLANNING_ERROR,
  TRAFFIC_LIGHT_DECIDER_ERROR
};

class SpeedStatus : public std::exception {
 public:
  SpeedStatus() = default;

  SpeedStatus(const StatusCode& code, const std::string& msg)
      : code_(code), msg_(msg) {}

  virtual ~SpeedStatus() = default;

  static SpeedStatus OK();

  bool ok() const;

  StatusCode code() const;
  bool operator==(const SpeedStatus& rh) const;

  bool operator!=(const SpeedStatus& rh) const;

  std::string error_message() const;

  std::string ToString() const;

  std::string GetCodeName(const StatusCode& status) const;

 private:
  StatusCode code_ = StatusCode::OK;
  std::string msg_ = "OK";
};

}  // namespace planning
