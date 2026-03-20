#pragma once

#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "apa_obstacle_manager.h"
#include "collision_detector_interface.h"
#include "curve_node.h"
#include "geometry_math.h"
#include "hybrid_astar_config.h"
#include "hybrid_astar_context.h"
#include "hybrid_astar_path_generator_interface.h"
#include "tail_in/hybrid_astar_perpendicular_tail_in_path_generator.h"
namespace planning {
namespace apa_planner {

enum class PathGenRequestResponseState : uint8_t {
  NONE = 0,
  HAS_REQUEST,
  HAS_RESPONSE,
  HAS_PUBLISHED_RESPONSE,
  MAX_NUM,
};

enum class PathGenThreadState : uint8_t {
  INITTED,
  RUNNING,
  STOPPED,
  MAX_NUM,
};

const std::string PathGenRequestResponseStateToString(
    const PathGenRequestResponseState& state);

const std::string PathGenThreadStateToString(const PathGenThreadState& state);

void PrintPathGenThreadState(const PathGenThreadState& state);

void PrintPathGenRequestResponseState(const PathGenRequestResponseState& state);

struct PathGenThreadRequest {
  HybridAStarRequest hybrid_astar_request;
  std::shared_ptr<CollisionDetectorInterface> col_det_interface_ptr;
  PlannerOpenSpaceConfig config;
};

class PathGeneratorThread final {
 public:
  PathGeneratorThread();
  ~PathGeneratorThread();
  void Init();
  void SetRequest(const PathGenThreadRequest& request);

  void Start();

  void Stop();

  const std::shared_ptr<HybridAstarPathGeneratorInterface>&
  GetHybridAstarPathGeneratorInterfacePtr() {
    return hybrid_astar_path_generator_interface_ptr_;
  }

  const bool CheckHasRequest();
  const bool CheckHasResponse();

  const PathGenThreadState GetThreadState();
  const PathGenRequestResponseState GetRequestResponseState();

  void Reset();

  const bool PublishResponseData(HybridAstarResponse& response);

  const HybridAStarRequest GetRequest();

  const bool CheckIfInit() { return init_flag_; }

  // for debug
  const bool PublishChildNode(
      std::vector<DebugAstarSearchPoint>& child_node_debug);

  const bool PublishQueuePath(std::vector<Eigen::Vector2d>& queue_path);

  const bool PublishDeleteQueuePath(
      std::vector<Eigen::Vector2d>& del_queue_path);

  const bool PublishSearchNodeListMessage(
      std::vector<std::vector<Eigen::Vector2d>>& search_node_list);

  const bool PublishCurveNodeListMessage(
      std::vector<std::vector<Eigen::Vector2d>>& curve_node_list);

  const bool PublishAllSuccessCurvePath(
      std::vector<CurvePath>& all_success_curve_path);

  const bool PublishAllSuccessCurvePathFirstGearSwitchPose(
      std::vector<geometry_lib::PathPoint>&
          all_success_curve_path_first_gear_switch_pose);

  const bool PublishIntersetingArea(cdl::AABB& interseting_area);

  const std::shared_ptr<CollisionDetectorInterface>& GetColDetInterfacePtr()
      const {
    return col_det_interface_ptr_;
  }

 private:
  void SetResponse();

  void Process();

  void PathGenThreadFunc();

 private:
  std::mutex mutex_;
  std::thread thread_;
  std::atomic<PathGenThreadState> thread_state_{PathGenThreadState::INITTED};

  bool init_flag_ = false;

  HybridAstarResponse response_;

  std::atomic<AstarSearchState> search_state_{AstarSearchState::NONE};

  std::atomic<PathGenRequestResponseState> request_response_state_;

  HybridAStarRequest hybrid_astar_request_;
  PlannerOpenSpaceConfig config_;
  std::shared_ptr<CollisionDetectorInterface> col_det_interface_ptr_;
  std::shared_ptr<ApaObstacleManager> obs_manager_ptr_;

  std::shared_ptr<HybridAstarPathGeneratorInterface>
      hybrid_astar_path_generator_interface_ptr_;

  std::vector<DebugAstarSearchPoint> child_node_debug_;
  std::vector<Eigen::Vector2d> queue_path_debug_;
  std::vector<Eigen::Vector2d> delete_queue_path_debug_;
  std::vector<std::vector<Eigen::Vector2d>> all_search_node_list_;
  std::vector<std::vector<Eigen::Vector2d>> all_curve_node_list_;
  std::vector<CurvePath> all_success_curve_path_debug_;
};

}  // namespace apa_planner
}  // namespace planning