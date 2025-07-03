
#include "hybrid_astar_thread.h"

#include <cstdio>

#include "log_glog.h"
#include "park_reference_line.h"
#include "transform2d.h"

namespace planning {

HybridAStarThreadSolver::HybridAStarThreadSolver() {
  printf("HybridAStarThreadSolver init\n");
  request_response_state_.store(RequestResponseState::NONE);
}

void HybridAStarThreadSolver::HybridAStarThreadFunction() {
  while (thread_state_ == AstarThreadState::RUNNING) {
    // sleep 100 ms
    usleep(100 * 1000);

    // ILOG_INFO << "astar thread loop";

    if (request_response_state_ != RequestResponseState::HAS_REQUEST) {
      continue;
    }

    Process();

    SetResponse();
  }

  return;
}

int HybridAStarThreadSolver::Init(const float back_edge_to_rear_axis,
                                  const float car_length, const float car_width,
                                  const float steer_ratio,
                                  const float wheel_base,
                                  const float min_turn_radius,
                                  const float mirror_width) {
  solver_interface_ = std::make_shared<HybridAStarInterface>();

  request_response_state_.store(RequestResponseState::NONE);

  solver_interface_->Init(back_edge_to_rear_axis, car_length, car_width,
                          steer_ratio, wheel_base, min_turn_radius,
                          mirror_width);

  init_ = true;
  search_state_.store(AstarSearchState::NONE);
  thread_state_.store(AstarThreadState::INITTED);
  ILOG_INFO << "HybridAStarThreadSolver init success";

  return 0;
}

void HybridAStarThreadSolver::SetRequest(const ParkObstacleList& obs_list,
                                         const AstarRequest& request) {
  std::lock_guard<std::mutex> lock(mutex_);

  if (search_state_ == AstarSearchState::SEARCHING) {
    ILOG_INFO << "searching";
    return;
  }

  if (request_response_state_ == RequestResponseState::HAS_REQUEST) {
    ILOG_INFO << "has path request, please use it and wait result";
    return;
  }

  if (request_response_state_ == RequestResponseState::HAS_RESPONSE) {
    ILOG_INFO << "has path, please get it ";
    return;
  }

  thread_request_data_ = request;

  solver_interface_->UpdateInput(obs_list, thread_request_data_);

  search_state_.store(AstarSearchState::NONE);
  request_response_state_.store(RequestResponseState::HAS_REQUEST);

  ILOG_INFO << "thread, set input";

  return;
}

void HybridAStarThreadSolver::SetResponse() {
  std::lock_guard<std::mutex> lock(mutex_);

  ILOG_INFO << "set output init in thread";

  thread_response_data_.result.Clear();
  solver_interface_->GetFullLengthPath(&thread_response_data_.result);

  thread_response_data_.first_seg_path.clear();
  thread_response_data_.kappa_change_too_much =
      solver_interface_->GetFirstSegmentPath(
          thread_response_data_.first_seg_path);

  search_state_.store(AstarSearchState::SUCCESS);

  thread_response_data_.request = thread_request_data_;

  // child node
  all_child_node_list_.clear();
  solver_interface_->GetNodeListMessage(all_child_node_list_);

  // all rs path
  rs_path_list_.clear();
  solver_interface_->GetRSPathHeuristic(rs_path_list_);

  request_response_state_.store(RequestResponseState::HAS_RESPONSE);

  ILOG_INFO << "set output finish in thread";

  return;
}

const int HybridAStarThreadSolver::PublishResponse(AstarResponse* response) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (request_response_state_ != RequestResponseState::HAS_RESPONSE) {
    return 0;
  }

  *response = thread_response_data_;

  request_response_state_.store(RequestResponseState::HAS_PUBLISHED_RESPONSE);

  return 0;
}

// const int HybridAStarThreadSolver::PublishSearchPath(
//     HybridAStarResult* result,
//     std::vector<AStarPathPoint>& first_seg_path, Pose2f* base_pose) {
//   std::lock_guard<std::mutex> lock(mutex_);
//   if (!has_response_) {
//     return 0;
//   }

//   *result = response_.result;
//   first_seg_path = response_.first_seg_path;
//   *base_pose = response_.request.base_pose_;

//   return 0;
// }

const bool HybridAStarThreadSolver::HasRequest() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (request_response_state_ == RequestResponseState::HAS_REQUEST) {
    return true;
  }

  return false;
}

const bool HybridAStarThreadSolver::HasResponse() {
  if (request_response_state_.load() == RequestResponseState::HAS_RESPONSE) {
    return true;
  }

  return false;
}

void HybridAStarThreadSolver::GetThreadState(RequestResponseState* state) {
  std::lock_guard<std::mutex> lock(mutex_);

  *state = request_response_state_.load();

  ILOG_INFO << "thread " << static_cast<int>(request_response_state_.load());

  return;
}

const void HybridAStarThreadSolver::GetFullLengthPathInThread(
    HybridAStarResult* result, Pose2D* base_pose) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (search_state_ != AstarSearchState::SUCCESS) {
    ILOG_INFO << "no output";
    return;
  }

  *result = thread_response_data_.result;

  *base_pose = thread_response_data_.request.base_pose_;

  ILOG_INFO << "result size " << result->x.size() << " dist "
            << result->accumulated_s.back();

  return;
}

const Pose2f HybridAStarThreadSolver::GetAstarTargetPose() {
  std::lock_guard<std::mutex> lock(mutex_);

  if (solver_interface_ != nullptr) {
    return solver_interface_->GetAstarTargetPose();
  }

  return Pose2f(0, 0, 0);
}

AstarRequest HybridAStarThreadSolver::GetAstarRequest() {
  std::lock_guard<std::mutex> lock(mutex_);

  return thread_request_data_;
}

void HybridAStarThreadSolver::GetNodeListMessageInThread(
    std::vector<std::vector<Eigen::Vector2d>>& list) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (search_state_ != AstarSearchState::SUCCESS) {
    return;
  }

  list = all_child_node_list_;

  ILOG_INFO << "node size " << list.size();

  return;
}

void HybridAStarThreadSolver::GetRSPathHeuristicInThread(
    std::vector<std::vector<Vec2f>>& path_list) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (search_state_ != AstarSearchState::SUCCESS) {
    return;
  }

  path_list = rs_path_list_;

  return;
}

void HybridAStarThreadSolver::GetRSPathLinkInThread(std::vector<Vec2f>& path) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (search_state_ != AstarSearchState::SUCCESS) {
    return;
  }

  std::vector<float> x;
  std::vector<float> y;
  std::vector<float> phi;

  solver_interface_->GetRSPathForDebug(x, y, phi);

  path.reserve(x.size());
  for (size_t i = 0; i < x.size(); i++) {
    path.emplace_back(Vec2f(x[i], y[i]));
  }

  return;
}

void HybridAStarThreadSolver::GetNodeListMessagePublish(
    planning::common::AstarNodeList* list) {
  std::lock_guard<std::mutex> lock(mutex_);

  if (search_state_ == AstarSearchState::SUCCESS) {
    planning::common::TrajectoryPoint point;

    for (size_t i = 0; i < all_child_node_list_.size(); i++) {
      planning::common::AstarNode* tmp_node = list->add_nodes();

      for (size_t m = 0; m < all_child_node_list_[i].size(); m++) {
        point.set_x(all_child_node_list_[i][m][0]);
        point.set_y(all_child_node_list_[i][m][1]);

        planning::common::TrajectoryPoint* tmp_point =
            tmp_node->add_path_point();

        tmp_point->CopyFrom(point);
      }
    }
  }

  return;
}

void HybridAStarThreadSolver::Start() {
  printf("start astar thread\n");

  if (!init_) {
    ILOG_INFO << "init fail";
    return;
  }

  thread_state_.store(AstarThreadState::RUNNING);
  thread_ =
      std::thread(&HybridAStarThreadSolver::HybridAStarThreadFunction, this);

  ILOG_INFO << "HybridAStarThreadSolver start thread success";

  return;
}

void HybridAStarThreadSolver::Stop() {
  if (!init_) {
    ILOG_INFO << "init fail";
    return;
  }

  if (thread_.joinable()) {
    thread_.join();
  }

  return;
}

int HybridAStarThreadSolver::Process() {
  search_state_.store(AstarSearchState::SEARCHING);
  solver_interface_->UpdateOutput();

  return 0;
}

void HybridAStarThreadSolver::Clear() {
  request_response_state_.store(RequestResponseState::NONE);

  ILOG_INFO << "thread clear";
  return;
}

void HybridAStarThreadSolver::GetRefLine(ParkReferenceLine* ref_line) {
  std::lock_guard<std::mutex> lock(mutex_);

  const ParkReferenceLine& const_ref_line =
      solver_interface_->GetConstRefLine();

  *ref_line = const_ref_line;

  return;
}

HybridAStarThreadSolver::~HybridAStarThreadSolver() {
  thread_state_.store(AstarThreadState::STOPPED);
  printf("HybridAStarThreadSolver\n");
  Stop();
}

void HybridAStarThreadSolver::GetVirtualWallPoints(
    std::vector<Position2D>* virtual_obs) {
  std::lock_guard<std::mutex> lock(mutex_);

  if (solver_interface_ != nullptr && virtual_obs != nullptr) {
    const ParkObstacleList& obs_list = solver_interface_->GetConstObstacles();

    *virtual_obs = obs_list.virtual_obs;
  }

  return;
}

}  // namespace planning