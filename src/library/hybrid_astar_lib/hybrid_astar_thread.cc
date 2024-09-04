
#include "hybrid_astar_thread.h"

#include "hybrid_a_star.h"
#include "hybrid_astar_common.h"
#include "log_glog.h"
#include "park_reference_line.h"
#include "pose2d.h"
#include "transform2d.h"

namespace planning {

HybridAStarThreadSolver::HybridAStarThreadSolver() {
  has_request_ = false;
  has_response_ = false;
  request_response_state_ = RequestResponseState::none;
}

void HybridAStarThreadSolver::HybridAStarThreadFunction() {
  while (true) {
    // sleep 100 ms
    usleep(100 * 1000);

    // ILOG_INFO << "astar thread loop";

    if (!has_request_) {
      continue;
    }

    Process();

    SetResponse();
  }

  return;
}

int HybridAStarThreadSolver::Init(
    const double back_edge_to_rear_axis, const double car_length,
    const double car_width, const double steer_ratio, const double wheel_base,
    const double min_turn_radius, const double mirror_width) {
  solver_interface_ = std::make_shared<HybridAStarInterface>();

  has_request_ = false;
  has_response_ = false;
  request_response_state_ = RequestResponseState::none;

  solver_interface_->Init(back_edge_to_rear_axis, car_length, car_width,
                          steer_ratio, wheel_base, min_turn_radius,
                          mirror_width);

  init_ = true;
  response_.search_state = SearchState::none;
  ILOG_INFO << "HybridAStarThreadSolver init success";

  return 0;
}

void HybridAStarThreadSolver::SetRequest(const ParkObstacleList& obs_list,
                                         const AstarRequest& request) {
  std::lock_guard<std::mutex> lock(mutex_);

  if (response_.search_state == SearchState::searching) {
    ILOG_INFO << "searching";
    return;
  }

  if (has_request_) {
    ILOG_INFO << "has path request, please use it and wait result";
    return;
  }

  if (has_response_) {
    ILOG_INFO << "has path, please get it ";
    return;
  }

  request_ = request;

  solver_interface_->UpdateInput(obs_list, request);

  has_request_ = true;
  has_response_ = false;
  response_.search_state = SearchState::none;
  request_response_state_ = RequestResponseState::has_request;

  ILOG_INFO << "thread, set input";

  return;
}

void HybridAStarThreadSolver::SetResponse() {
  std::lock_guard<std::mutex> lock(mutex_);

  ILOG_INFO << "set output init in thread";

  response_.result.Clear();
  solver_interface_->GetFullLengthPath(&response_.result);

  response_.first_seg_path.clear();
  response_.kappa_change_too_much =
      solver_interface_->GetFirstSegmentPath(response_.first_seg_path);

  response_.search_state = SearchState::success;

  if (response_.result.gear.size() > 0) {
    if (response_.result.gear[0] == AstarPathGear::drive) {
      request_.history_gear = AstarPathGear::drive;
    } else if (response_.result.gear[0] == AstarPathGear::reverse) {
      request_.history_gear = AstarPathGear::reverse;
    } else {
      request_.history_gear = AstarPathGear::parking;
    }
  }
  response_.request = request_;

  // child node
  all_child_node_list_.clear();
  solver_interface_->GetNodeListMessage(all_child_node_list_);

  // all rs path
  rs_path_list_.clear();
  solver_interface_->GetRSPathHeuristic(rs_path_list_);

  has_response_ = true;
  has_request_ = false;
  request_response_state_ = RequestResponseState::has_response;

  ILOG_INFO << "set output finish in thread";

  return;
}

const int HybridAStarThreadSolver::PublishResponse(AstarResponse* response) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!has_response_) {
    return 0;
  }

  *response = response_;

  request_response_state_ = RequestResponseState::has_published_response;

  return 0;
}

// const int HybridAStarThreadSolver::PublishSearchPath(
//     HybridAStarResult* result,
//     std::vector<AStarPathPoint>& first_seg_path, Pose2D* base_pose) {
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
  return has_request_;
}

const bool HybridAStarThreadSolver::HasResponse() {
  std::lock_guard<std::mutex> lock(mutex_);
  return has_response_;
}

void HybridAStarThreadSolver::GetThreadState(RequestResponseState* state) {
  std::lock_guard<std::mutex> lock(mutex_);

  *state = request_response_state_;
  return;
}

const int HybridAStarThreadSolver::GetFullLengthPathInThread(
    HybridAStarResult* result, Pose2D* base_pose) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (response_.search_state != SearchState::success) {
    ILOG_INFO << "no output";
    return -1;
  }

  *result = response_.result;

  *base_pose = response_.request.base_pose_;

  ILOG_INFO << "result size " << result->x.size() << " dist "
            << result->accumulated_s.back();

  return 0;
}

const Pose2D HybridAStarThreadSolver::GetAstarTargetPose() {
  std::lock_guard<std::mutex> lock(mutex_);

  if (solver_interface_ != nullptr) {
    return solver_interface_->GetAstarTargetPose();
  }

  return Pose2D(0, 0, 0);
}

AstarRequest HybridAStarThreadSolver::GetAstarRequest() {
  std::lock_guard<std::mutex> lock(mutex_);

  return request_;
}

void HybridAStarThreadSolver::GetNodeListMessageInThread(
    std::vector<std::vector<Eigen::Vector2d>>& list) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (response_.search_state != SearchState::success) {
    return;
  }

  list = all_child_node_list_;

  ILOG_INFO << "node size " << list.size();

  return;
}

void HybridAStarThreadSolver::GetRSPathHeuristicInThread(
    std::vector<std::vector<ad_common::math::Vec2d>>& path_list) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (response_.search_state != SearchState::success) {
    return;
  }

  path_list = rs_path_list_;
}

void HybridAStarThreadSolver::GetRSPathLinkInThread(
    std::vector<ad_common::math::Vec2d>& path) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (response_.search_state != SearchState::success) {
    return;
  }

  std::vector<double> x;
  std::vector<double> y;
  std::vector<double> phi;

  solver_interface_->GetRSPathForDebug(x, y, phi);

  path.reserve(x.size());
  for (size_t i = 0; i < x.size(); i++) {
    path.emplace_back(ad_common::math::Vec2d(x[i], y[i]));
  }

  return;
}

void HybridAStarThreadSolver::GetNodeListMessagePublish(
    planning::common::AstarNodeList* list) {
  std::lock_guard<std::mutex> lock(mutex_);

  if (response_.search_state == SearchState::success) {
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

  const auto& update_func = [this] { HybridAStarThreadFunction(); };

  thread_.reset(new std::thread(update_func));

  ILOG_INFO << "HybridAStarThreadSolver start thread success";
}

void HybridAStarThreadSolver::Stop() {
  if (!init_) {
    ILOG_INFO << "init fail";
    return;
  }

  if (thread_ != nullptr && thread_->joinable()) {
    thread_->join();
    thread_.reset();
  }

  return;
}

int HybridAStarThreadSolver::Process() {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    response_.search_state = SearchState::searching;
  }

  solver_interface_->UpdateOutput();

  return 0;
}

void HybridAStarThreadSolver::ResetResponse() {
  std::lock_guard<std::mutex> lock(mutex_);
  has_response_ = false;
}

void HybridAStarThreadSolver::Clear() {
  std::lock_guard<std::mutex> lock(mutex_);

  has_request_ = false;
  has_response_ = false;
  response_.search_state = SearchState::none;
  request_response_state_ = RequestResponseState::none;
  return;
}

void HybridAStarThreadSolver::GetRefLine(ParkReferenceLine* ref_line) {
  std::lock_guard<std::mutex> lock(mutex_);

  const ParkReferenceLine& const_ref_line =
      solver_interface_->GetConstRefLine();

  *ref_line = const_ref_line;

  return;
}

}  // namespace planning