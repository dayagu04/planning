#include "status.h"

namespace planning {
namespace common {

Status::Status() : Status(StatusCode::OK, "") {}

Status::Status(const StatusCode& code, const std::string& msg)
    : code_(code), msg_(msg.data()) {}

Status Status::OK() { return Status(); }

bool Status::ok() const { return code_ == StatusCode::OK; }

StatusCode Status::code() const { return code_; }

bool Status::operator==(const Status& rh) const {
  return (this->code_ == rh.code_) && (this->msg_ == rh.msg_);
}

bool Status::operator!=(const Status& rh) const { return !(*this == rh); }

std::string Status::error_message() const { return msg_; }

std::string Status::ToString() const {
  if (ok()) {
    return "OK";
  }
  return GetCodeName(code_) + ": " + msg_;
}

std::string Status::GetCodeName(const StatusCode& status) const {
  switch (status) {
    case StatusCode::OK:
      return "OK";
    case StatusCode::COMMON_ERROR:
      return "COMMON_ERROR";
    case StatusCode::PLANNING_ERROR:
      return "PLANNING_ERROR";
    case StatusCode::SIM_ERROR:
      return "SIM_ERROR";
  }
  return "OK";
}

}  // namespace common
}  // namespace planning
