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
  // check time
  // double time_diff = request.timestamp_ms - response.request.timestamp_ms;
  // // 5 minute
  // if (time_diff > 5.0 * 60 * 1000.0) {
  //   // std::string t1 = TimeStampString(request.timestamp_ms);
  //   // std::string t2 = TimeStampString(response.request.timestamp_ms);

  //   ILOG_INFO << "time is overdue(ms): " << time_diff << " t1 "
  //             << TimeStampString(request.timestamp_ms) << " ,t2 "
  //             << TimeStampString(response.request.timestamp_ms);

  //   return false;
  // }

  // check start
  // double dist = CalcPointDist(&request.start_, &response.request.start_);

  // if (dist > 0.1)
  // {
  //   ILOG_INFO << "start pose is change";
  //   return false;
  // }

  // check goal
  // dist = CalcPointDist(&request.goal_, &response.request.goal_);

  // if (dist > 0.2) {
  //   ILOG_INFO << "goal pose is change";
  //   return false;
  // }

  // dist = CalcPointDist(&request.real_goal, &response.request.real_goal);
  // if (dist > 0.2) {
  //   ILOG_INFO << "goal pose is change";
  //   return false;
  // }

  // if (request.rs_request != response.request.rs_request) {
  //   ILOG_INFO << "rs request is change";
  //   return false;
  // }

  // if (request.path_generate_method != response.request.path_generate_method)
  // {
  //   ILOG_INFO << "method is change, cur method: "
  //             << static_cast<int>(request.path_generate_method) << " , last:
  //             "
  //             << static_cast<int>(response.request.path_generate_method);

  //   return false;
  // }

  if (request.space_type != response.request.space_type) {
    ILOG_INFO << "slot type is change";
    return false;
  }

  if (request.slot_id != response.request.slot_id) {
    ILOG_INFO << "slot id is change";
    return false;
  }

  if (request.path_generate_method != response.request.path_generate_method) {
    ILOG_INFO << "path generate request is different";
    return false;
  }

  // if (request.plan_reason != response.request.plan_reason) {
  //   ILOG_INFO << "plan reason is change";
  //   return false;
  // }

  return true;
}

}  // namespace planning