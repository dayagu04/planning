#pragma once

#include <cstddef>
#include <mutex>
#include <thread>

#include "hybrid_a_star.h"
#include "hybrid_astar_common.h"
#include "hybrid_astar_interface.h"
#include "hybrid_astar_response.h"
#include "log_glog.h"
#include "park_reference_line.h"
#include "pose2d.h"

namespace planning {

enum class RequestResponseState {
  NONE = 0,
  HAS_REQUEST,
  HAS_RESPONSE,
  HAS_PUBLISHED_RESPONSE,
  MAX_NUM,
};

enum class AstarThreadState { INITTED, RUNNING, STOPPED };

// this is a thread version for astar, because control module need planning
// process heartbreak. So main thread need to publish message to control and
// call astar thread's result. Of course you can directly use
// hybrid_astar_interface, and no need use this thread if you publish message in
// astar main loop.
class HybridAStarThreadSolver {
 public:
  HybridAStarThreadSolver();

  ~HybridAStarThreadSolver();

  int Init(const double back_edge_to_rear_axis, const double car_length,
           const double car_width, const double steer_ratio,
           const double wheel_base, const double min_turn_radius,
           const double mirror_width);

  void SetRequest(const ParkObstacleList& obs_list,
                  const AstarRequest& request);

  void SetResponse();

  void Start();

  void Stop();

  void HybridAStarThreadFunction();

  int Process();

  // for debug
  std::shared_ptr<HybridAStarInterface> GetHybridAStarInterface() {
    return solver_interface_;
  }

  const bool HasRequest();

  const bool HasResponse();

  void GetThreadState(RequestResponseState* state);

  void Clear();

  // get result api
  const int PublishResponse(AstarResponse* response);

  AstarRequest GetAstarRequest();

  // for publish
  void GetNodeListMessagePublish(planning::common::AstarNodeList* list);

 public:
  // get result api
  // const int PublishSearchPath(HybridAStarResult* result,
  //                             std::vector<AStarPathPoint>& first_seg_path,
  //                             Pose2D* base_pose);

  // for debug
  const void GetFullLengthPathInThread(HybridAStarResult* result,
                                       Pose2D* base_pose);

  // for debug
  const Pose2D GetAstarTargetPose();

  // for debug
  void GetNodeListMessageInThread(
      std::vector<std::vector<Eigen::Vector2d>>& list);

  // for debug
  void GetRSPathHeuristicInThread(
      std::vector<std::vector<ad_common::math::Vec2d>>& path_list);

  // for debug
  void GetRSPathLinkInThread(std::vector<ad_common::math::Vec2d>& path);

  // for debug
  void GetRefLine(ParkReferenceLine* ref_line);

  // for debug
  void GetVirtualWallPoints(std::vector<Position2D>* virtual_obs);

 private:
  // in
  std::mutex mutex_;
  AstarRequest thread_request_data_;

  // out
  AstarResponse thread_response_data_;

  std::atomic<AstarSearchState> search_state_{AstarSearchState::NONE};

  // only main thread read/write it, so not use std::atomic.
  bool init_ = false;
  std::atomic<RequestResponseState> request_response_state_;

  std::atomic<AstarThreadState> thread_state_{AstarThreadState::INITTED};
  std::thread thread_;

  // for debug
  std::vector<std::vector<Eigen::Vector2d>> all_child_node_list_;
  // for debug
  std::vector<std::vector<ad_common::math::Vec2d>> rs_path_list_;
  // for debug
  std::shared_ptr<HybridAStarInterface> solver_interface_;
};

}  // namespace planning