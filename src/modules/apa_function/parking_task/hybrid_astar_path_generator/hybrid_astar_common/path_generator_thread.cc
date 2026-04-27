#include "path_generator_thread.h"

#include <memory>

#include "apa_context.h"
#include "apa_obstacle_manager.h"
#include "collision_detector_interface.h"
#include "hybrid_astar_common.h"
#include "hybrid_astar_path_generator_interface.h"
#include "log_glog.h"

namespace planning {
namespace apa_planner {

PathGeneratorThread::PathGeneratorThread() { Init(); }

PathGeneratorThread::~PathGeneratorThread() {
  thread_state_.store(PathGenThreadState::STOPPED, std::memory_order_release);
  ILOG_INFO << "PathGeneratorThread destructor";
  Stop();
}

void PathGeneratorThread::Init() {
  if (init_flag_) {
    return;
  }

  obs_manager_ptr_ = std::make_shared<ApaObstacleManager>();

  col_det_interface_ptr_ = std::make_shared<CollisionDetectorInterface>();

  hybrid_astar_path_generator_interface_ptr_ =
      std::make_shared<HybridAstarPathGeneratorInterface>();

  search_state_.store(AstarSearchState::NONE, std::memory_order_release);

  request_response_state_.store(PathGenRequestResponseState::NONE,
                                std::memory_order_release);

  thread_state_.store(PathGenThreadState::INITTED, std::memory_order_release);

  ILOG_INFO << "PathGeneratorThread init success";

  init_flag_ = true;
}

void PathGeneratorThread::Start() {
  ILOG_INFO << "try start path generator thread";

  if (!init_flag_) {
    ILOG_ERROR << "PathGeneratorThread is not init, return";
    return;
  }

  thread_state_.store(PathGenThreadState::RUNNING, std::memory_order_seq_cst);

  thread_ = std::thread(&PathGeneratorThread::PathGenThreadFunc, this);

  pthread_setname_np(thread_.native_handle(), "Pln_PathPlan_01");

  ILOG_INFO << "PathGenerator start thread success";

  return;
}

void PathGeneratorThread::Stop() {
  ILOG_INFO << "try stop path generator thread";

  if (!init_flag_) {
    ILOG_INFO << "PathGeneratorThread is not init, return";
    return;
  }

  if (thread_.joinable()) {
    thread_.join();
  }

  return;
}

void PathGeneratorThread::SetRequest(const PathGenThreadRequest& request) {
  std::lock_guard<std::mutex> lock(mutex_);

  if (search_state_.load(std::memory_order_acquire) ==
      AstarSearchState::SEARCHING) {
    ILOG_INFO << "still searching, no need to set request, return";
    return;
  }

  if (request_response_state_.load(std::memory_order_acquire) ==
      PathGenRequestResponseState::HAS_REQUEST) {
    ILOG_INFO << "there is already a request, please wait for the result of "
                 "the last call, return";
    return;
  }

  if (request_response_state_.load(std::memory_order_acquire) ==
      PathGenRequestResponseState::HAS_RESPONSE) {
    ILOG_INFO << "there is already a response, please directly use it, return";
    return;
  }

  const std::shared_ptr<CollisionDetectorInterface> par_col_det_interface_ptr =
      request.col_det_interface_ptr;

  if (par_col_det_interface_ptr == nullptr ||
      par_col_det_interface_ptr->GetObsManagerPtr() == nullptr) {
    ILOG_ERROR << "col_det_interface_ptr is nullptr, return";
    return;
  }

  hybrid_astar_request_ = request.hybrid_astar_request;

  config_ = request.config;

  ILOG_INFO
      << "parent obs manager size: "
      << par_col_det_interface_ptr->GetObsManagerPtr()->GetObstacles().size();

  // deep copy, the class should not have ptr variable
  *obs_manager_ptr_ = *(par_col_det_interface_ptr->GetObsManagerPtr());

  ILOG_INFO << "child obs manager size: "
            << obs_manager_ptr_->GetObstacles().size();

  col_det_interface_ptr_->SetObsManagerPtr(obs_manager_ptr_);

  col_det_interface_ptr_->Init(hybrid_astar_request_.mirror_has_folded_flag);

  col_det_interface_ptr_->GetEDTColDetPtr()->PreProcess(
      par_col_det_interface_ptr->GetEDTColDetPtr()->GetOgmBound(),
      par_col_det_interface_ptr->GetEDTColDetPtr()->GetUseObsHeightMethod());

  col_det_interface_ptr_->GetEDTColDetPtr()->UpdateObsClearZone(
      std::vector<Eigen::Vector2d>{
          hybrid_astar_request_.ego_info_under_slot.cur_pose.pos,
          hybrid_astar_request_.ego_info_under_slot.target_pose.pos});

  hybrid_astar_path_generator_interface_ptr_->SetRequest(hybrid_astar_request_);
  hybrid_astar_path_generator_interface_ptr_->SetColDetIntefacePtr(
      col_det_interface_ptr_);
  hybrid_astar_path_generator_interface_ptr_->UpdateConfig(config_);

  search_state_.store(AstarSearchState::NONE, std::memory_order_release);
  request_response_state_.store(PathGenRequestResponseState::HAS_REQUEST,
                                std::memory_order_seq_cst);

  ILOG_INFO << "path generator thread set request success";

  return;
}

const bool PathGeneratorThread::CheckHasRequest() {
  return request_response_state_.load(std::memory_order_acquire) ==
         PathGenRequestResponseState::HAS_REQUEST;
}

const bool PathGeneratorThread::CheckHasResponse() {
  return request_response_state_.load(std::memory_order_acquire) ==
         PathGenRequestResponseState::HAS_RESPONSE;
}

const PathGenThreadState PathGeneratorThread::GetThreadState() {
  PathGenThreadState thread_state =
      thread_state_.load(std::memory_order_acquire);
  PrintPathGenThreadState(thread_state);
  return thread_state;
}

const PathGenRequestResponseState
PathGeneratorThread::GetRequestResponseState() {
  PathGenRequestResponseState request_response_state =
      request_response_state_.load(std::memory_order_acquire);
  PrintPathGenRequestResponseState(request_response_state);
  return request_response_state;
}

void PathGeneratorThread::Reset() {
  request_response_state_.store(PathGenRequestResponseState::NONE,
                                std::memory_order_release);

  search_state_.store(AstarSearchState::NONE, std::memory_order_release);

  ILOG_INFO << "PathGeneratorThread Reset";
  return;
}

const bool PathGeneratorThread::PublishResponseData(
    HybridAstarResponse& response) {
  response.Clear();
  std::lock_guard<std::mutex> lock(mutex_);
  if (request_response_state_.load(std::memory_order_acquire) !=
      PathGenRequestResponseState::HAS_RESPONSE) {
    ILOG_INFO << "no response or response has been published, return";
    return false;
  }

  response = response_;

  request_response_state_.store(
      PathGenRequestResponseState::HAS_PUBLISHED_RESPONSE,
      std::memory_order_release);

  return true;
}

const HybridAStarRequest PathGeneratorThread::GetRequest() {
  std::lock_guard<std::mutex> lock(mutex_);
  return hybrid_astar_request_;
}

void PathGeneratorThread::PathGenThreadFunc() {
  ILOG_INFO << "path generator thread loop begin";

  while (thread_state_.load(std::memory_order_seq_cst) ==
         PathGenThreadState::RUNNING) {
    // sleep 100 ms, the circle is 0.1s
    usleep(5 * 1000);

    // ILOG_INFO << "astar thread loop";

    if (request_response_state_.load(std::memory_order_seq_cst) ==
        PathGenRequestResponseState::HAS_REQUEST) {
      Process();
      SetResponse();
    }
  }

  ILOG_INFO << "path generator thread loop end";

  return;
}

void PathGeneratorThread::Process() {
  ILOG_INFO << "process begin in path generator thread";
  search_state_.store(AstarSearchState::SEARCHING, std::memory_order_release);
  // when the path generator is running, the state is searching,
  // and when update over, the state is success or failure
  if (hybrid_astar_path_generator_interface_ptr_->Update()) {
    search_state_.store(AstarSearchState::SUCCESS, std::memory_order_release);
  } else {
    search_state_.store(AstarSearchState::FAILURE, std::memory_order_release);
  }
  ILOG_INFO << "process end in path generator thread";

  return;
}

void PathGeneratorThread::SetResponse() {
  std::lock_guard<std::mutex> lock(mutex_);

  ILOG_INFO << "set response in path generator thread";

  response_.Clear();
  child_node_debug_.clear();
  queue_path_debug_.clear();
  delete_queue_path_debug_.clear();
  all_search_node_list_.clear();
  all_curve_node_list_.clear();
  all_success_curve_path_debug_.clear();

  // the path gen update must have already ended, and the search state would be
  // success or fail
  hybrid_astar_path_generator_interface_ptr_->GetResult(response_.result);

  // hybrid_astar_path_generator_interface_ptr_->GetChildNodeForDebug(
  //     child_node_debug_);

  // hybrid_astar_path_generator_interface_ptr_->GetQueuePathForDebug(
  //     queue_path_debug_);

  // hybrid_astar_path_generator_interface_ptr_->GetDeleteQueuePathForDebug(
  //     delete_queue_path_debug_);

  // hybrid_astar_path_generator_interface_ptr_->GetSearchNodeListMessage(
  //     all_search_node_list_);

  // hybrid_astar_path_generator_interface_ptr_->GetCurveNodeListMessage(
  //     all_curve_node_list_);

  // hybrid_astar_path_generator_interface_ptr_->GetAllSuccessCurvePathForDebug(
  //     all_success_curve_path_debug_);

  response_.request = hybrid_astar_request_;

  request_response_state_.store(PathGenRequestResponseState::HAS_RESPONSE,
                                std::memory_order_release);
}

const bool PathGeneratorThread::PublishChildNode(
    std::vector<DebugAstarSearchPoint>& child_node_debug) {
  std::lock_guard<std::mutex> lock(mutex_);
  child_node_debug.clear();
  if (search_state_.load(std::memory_order_acquire) !=
          AstarSearchState::SUCCESS &&
      search_state_.load(std::memory_order_acquire) !=
          AstarSearchState::FAILURE) {
    return false;
  }
  hybrid_astar_path_generator_interface_ptr_->GetChildNodeForDebug(
      child_node_debug);
  return true;
}

const bool PathGeneratorThread::PublishAllSuccessCurvePath(
    std::vector<CurvePath>& all_success_curve_path) {
  all_success_curve_path.clear();
  if (search_state_.load(std::memory_order_acquire) !=
          AstarSearchState::SUCCESS &&
      search_state_.load(std::memory_order_acquire) !=
          AstarSearchState::FAILURE) {
    return false;
  }
  hybrid_astar_path_generator_interface_ptr_->GetAllSuccessCurvePathForDebug(
      all_success_curve_path);
  return true;
}

const bool PathGeneratorThread::PublishAllSuccessCurvePathFirstGearSwitchPose(
    std::vector<geometry_lib::PathPoint>&
        all_success_curve_path_first_gear_switch_pose) {
  all_success_curve_path_first_gear_switch_pose.clear();
  if (search_state_.load(std::memory_order_acquire) !=
          AstarSearchState::SUCCESS &&
      search_state_.load(std::memory_order_acquire) !=
          AstarSearchState::FAILURE) {
    return false;
  }
  hybrid_astar_path_generator_interface_ptr_
      ->GetAllSuccessCurvePathFirstGearSwitchPoseForDebug(
          all_success_curve_path_first_gear_switch_pose);
  return true;
}

const bool PathGeneratorThread::PublishQueuePath(
    std::vector<Eigen::Vector2d>& queue_path) {
  std::lock_guard<std::mutex> lock(mutex_);
  queue_path.clear();
  if (search_state_.load(std::memory_order_acquire) !=
          AstarSearchState::SUCCESS &&
      search_state_.load(std::memory_order_acquire) !=
          AstarSearchState::FAILURE) {
    return false;
  }
  hybrid_astar_path_generator_interface_ptr_->GetQueuePathForDebug(queue_path);
  return true;
}

const bool PathGeneratorThread::PublishDeleteQueuePath(
    std::vector<Eigen::Vector2d>& del_queue_path) {
  std::lock_guard<std::mutex> lock(mutex_);
  del_queue_path.clear();
  if (search_state_.load(std::memory_order_acquire) !=
          AstarSearchState::SUCCESS &&
      search_state_.load(std::memory_order_acquire) !=
          AstarSearchState::FAILURE) {
    return false;
  }
  hybrid_astar_path_generator_interface_ptr_->GetDeleteQueuePathForDebug(
      del_queue_path);
  return true;
}

const bool PathGeneratorThread::PublishIntersetingArea(
    cdl::AABB& interseting_area) {
  std::lock_guard<std::mutex> lock(mutex_);
  interseting_area.Reset();
  if (search_state_.load(std::memory_order_acquire) !=
          AstarSearchState::SUCCESS &&
      search_state_.load(std::memory_order_acquire) !=
          AstarSearchState::FAILURE) {
    return false;
  }
  hybrid_astar_path_generator_interface_ptr_->GetIntersetingAreaForDebug(
      interseting_area);
  return true;
}

const bool PathGeneratorThread::PublishSearchNodeListMessage(
    std::vector<std::vector<Eigen::Vector2d>>& search_node_list) {
  std::lock_guard<std::mutex> lock(mutex_);
  search_node_list.clear();
  if (search_state_.load(std::memory_order_acquire) !=
          AstarSearchState::SUCCESS &&
      search_state_.load(std::memory_order_acquire) !=
          AstarSearchState::FAILURE) {
    return false;
  }
  hybrid_astar_path_generator_interface_ptr_->GetSearchNodeListMessage(
      search_node_list);
  return true;
}

const bool PathGeneratorThread::PublishCurveNodeListMessage(
    std::vector<std::vector<Eigen::Vector2d>>& curve_node_list) {
  std::lock_guard<std::mutex> lock(mutex_);
  curve_node_list.clear();
  if (search_state_.load(std::memory_order_acquire) !=
          AstarSearchState::SUCCESS &&
      search_state_.load(std::memory_order_acquire) !=
          AstarSearchState::FAILURE) {
    return false;
  }
  hybrid_astar_path_generator_interface_ptr_->GetCurveNodeListMessage(
      curve_node_list);
  return true;
}

const std::string PathGenRequestResponseStateToString(
    const PathGenRequestResponseState& state) {
  switch (state) {
    case PathGenRequestResponseState::HAS_REQUEST:
      return "HAS_REQUEST";
    case PathGenRequestResponseState::HAS_RESPONSE:
      return "HAS_RESPONSE";
    case PathGenRequestResponseState::HAS_PUBLISHED_RESPONSE:
      return "HAS_PUBLISHED_RESPONSE";
    default:
      return "NONE";
  }
}

const std::string PathGenThreadStateToString(const PathGenThreadState& state) {
  switch (state) {
    case PathGenThreadState::INITTED:
      return "INITTED";
    case PathGenThreadState::RUNNING:
      return "RUNNING";
    case PathGenThreadState::STOPPED:
      return "STOPPED";
    default:
      return "NONE";
  }
}

void PrintPathGenThreadState(const PathGenThreadState& state) {
  ILOG_INFO << "PathGenThreadState: " << PathGenThreadStateToString(state);
}

void PrintPathGenRequestResponseState(
    const PathGenRequestResponseState& state) {
  ILOG_INFO << "PathGenRequestResponseState: "
            << PathGenRequestResponseStateToString(state);
}

}  // namespace apa_planner
}  // namespace planning