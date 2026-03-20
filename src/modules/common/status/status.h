#pragma once

#include <stdexcept>
#include <string>

namespace planning {
namespace common {

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

class Status : public std::exception {
 public:
  Status();

  Status(const StatusCode& code, const std::string& msg);

  virtual ~Status() = default;

  static Status OK();

  bool ok() const;

  StatusCode code() const;
  bool operator==(const Status& rh) const;

  bool operator!=(const Status& rh) const;

  std::string error_message() const;

  std::string ToString() const;

  std::string GetCodeName(const StatusCode& status) const;

 private:
  StatusCode code_ = StatusCode::OK;
  std::string msg_ = "OK";
};
}  // namespace common
}  // namespace planning
