#include "hybrid_astar_response.h"
#include <string>
#include "hybrid_astar_common.h"
#include "hybrid_astar_request.h"
#include "ifly_time.h"
#include "log_glog.h"
#include "node3d.h"

namespace planning {

std::string TimeStampString(const double ms) {
  auto tp = std::chrono::time_point<std::chrono::system_clock,
                                    std::chrono::microseconds>(
      std::chrono::microseconds((int64_t)(ms / 1000.0)));

  auto tt = std::chrono::system_clock::to_time_t(tp);
  std::tm* now = std::gmtime(&tt);

  return std::to_string(now->tm_year + 1900) + std::to_string(now->tm_mon + 1) +
         std::to_string(now->tm_mday) + std::to_string(now->tm_hour) +
         std::to_string(now->tm_min) + std::to_string(now->tm_sec);
}

const bool IsResponseNice(const AstarRequest& request,
                          const AstarResponse& response) {
  if (request.space_type != response.request.space_type) {
    ILOG_INFO << "slot type is change";
    return false;
  }

  if (request.slot_id != response.request.slot_id) {
    ILOG_INFO << "slot id is change, id = " << request.slot_id
              << ", history id = " << response.request.slot_id;
    return false;
  }

  return true;
}

}  // namespace planning