#include "hybrid_astar_path_generator_interface.h"

namespace planning {
namespace apa_planner {

#define PUBLISH_SEARCH_NODE_LIST (0)
#define PUBLISH_CURVE_NODE_LIST (0)

HybridAstarPathGeneratorInterface::HybridAstarPathGeneratorInterface() {
  if (has_constructed_flag_) {
    return;
  }

  path_generator_list_[ParkingScenarioType::SCENARIO_PERPENDICULAR_TAIL_IN] =
      std::make_shared<HybridAStarPerpendicularTailInPathGenerator>();

  path_generator_list_[ParkingScenarioType::SCENARIO_PERPENDICULAR_HEAD_IN] =
      std::make_shared<HybridAStarPerpendicularHeadingInPathGenerator>();

  has_constructed_flag_ = true;

  // todo: add more func

  // Init();
}

HybridAstarPathGeneratorInterface::HybridAstarPathGeneratorInterface(
    const std::shared_ptr<CollisionDetectorInterface>& col_det_interface_ptr) {
  if (has_constructed_flag_) {
    return;
  }

  path_generator_list_[ParkingScenarioType::SCENARIO_PERPENDICULAR_TAIL_IN] =
      std::make_shared<HybridAStarPerpendicularTailInPathGenerator>(
          col_det_interface_ptr);

  path_generator_list_[ParkingScenarioType::SCENARIO_PERPENDICULAR_HEAD_IN] =
      std::make_shared<HybridAStarPerpendicularHeadingInPathGenerator>(
          col_det_interface_ptr);

  has_constructed_flag_ = true;

  // todo: add more func

  // Init();
}

void HybridAstarPathGeneratorInterface::Init() {
  if (init_flag_) {
    return;
  }

  for (auto& path_generator : path_generator_list_) {
    path_generator.second->Init();
  }

  init_flag_ = true;
}

void HybridAstarPathGeneratorInterface::Reset() {
  for (auto& path_generator : path_generator_list_) {
    path_generator.second->Reset();
  }

  init_flag_ = false;
  scenario_type_ = ParkingScenarioType::SCENARIO_UNKNOWN;
  cur_path_generator_ = nullptr;
}

HybridAstarPathGeneratorInterface::~HybridAstarPathGeneratorInterface() {
  Reset();
  has_constructed_flag_ = false;
}

const bool HybridAstarPathGeneratorInterface::Update() {
  if (cur_path_generator_ == nullptr) {
    return false;
  }
  return cur_path_generator_->Update();
}

void HybridAstarPathGeneratorInterface::SetRequest(
    const HybridAStarRequest& request) {
  scenario_type_ = request.scenario_type;
  cur_path_generator_ = nullptr;
  if (path_generator_list_.find(scenario_type_) != path_generator_list_.end()) {
    cur_path_generator_ = path_generator_list_[scenario_type_];
  }
  if (cur_path_generator_ == nullptr) {
    return;
  }
  cur_path_generator_->SetRequest(request);
  return;
}

void HybridAstarPathGeneratorInterface::SetColDetIntefacePtr(
    const std::shared_ptr<CollisionDetectorInterface>& col_det_interface_ptr) {
  if (cur_path_generator_ == nullptr) {
    return;
  }
  return cur_path_generator_->SetCollisionDetectorIntefacePtr(
      col_det_interface_ptr);
}

void HybridAstarPathGeneratorInterface::UpdateConfig(
    const PlannerOpenSpaceConfig& config) {
  if (cur_path_generator_ == nullptr) {
    return;
  }
  return cur_path_generator_->UpdateConfig(config);
}

const bool HybridAstarPathGeneratorInterface::GetResult(
    HybridAStarResult& result) {
  result.Clear();
  if (cur_path_generator_ == nullptr) {
    return false;
  }
  result = cur_path_generator_->GetResult();
  return true;
}

const bool HybridAstarPathGeneratorInterface::GetChildNodeForDebug(
    std::vector<DebugAstarSearchPoint>& child_node_debug) {
  child_node_debug.clear();
  if (cur_path_generator_ == nullptr) {
    return false;
  }
  child_node_debug = cur_path_generator_->GetChildNodeForDebug();
  return true;
}

const bool HybridAstarPathGeneratorInterface::GetQueuePathForDebug(
    std::vector<Eigen::Vector2d>& queue_path) {
  queue_path.clear();
  if (cur_path_generator_ == nullptr) {
    return false;
  }
  queue_path = cur_path_generator_->GetQueuePathForDebug();
  return true;
}

const bool HybridAstarPathGeneratorInterface::GetDeleteQueuePathForDebug(
    std::vector<Eigen::Vector2d>& del_queue_path) {
  del_queue_path.clear();
  if (cur_path_generator_ == nullptr) {
    return false;
  }
  del_queue_path = cur_path_generator_->GetDeleteQueuePathForDebug();
  return true;
}

const bool HybridAstarPathGeneratorInterface::GetSearchNodeListMessage(
    std::vector<std::vector<Eigen::Vector2d>>& search_node_list) {
  search_node_list.clear();
  if (!PUBLISH_SEARCH_NODE_LIST) {
    return false;
  }
  if (cur_path_generator_ == nullptr) {
    return false;
  }
  search_node_list = cur_path_generator_->GetSearchNodeListMessage();
  return true;
}

const bool HybridAstarPathGeneratorInterface::GetCurveNodeListMessage(
    std::vector<std::vector<Eigen::Vector2d>>& curve_node_list) {
  curve_node_list.clear();
  if (!PUBLISH_CURVE_NODE_LIST) {
    return false;
  }
  if (cur_path_generator_ == nullptr) {
    return false;
  }
  curve_node_list = cur_path_generator_->GetCurveNodeListMessage();
  return true;
}

const bool HybridAstarPathGeneratorInterface::GetAllSuccessCurvePathForDebug(
    std::vector<CurvePath>& all_success_curve_path_debug) {
  all_success_curve_path_debug.clear();
  if (cur_path_generator_ == nullptr) {
    return false;
  }
  all_success_curve_path_debug =
      cur_path_generator_->GetAllSuccessCurvePathForDebug();
  return true;
}

const bool HybridAstarPathGeneratorInterface::
    GetAllSuccessCurvePathFirstGearSwitchPoseForDebug(
        std::vector<geometry_lib::PathPoint>&
            all_success_curve_path_first_gear_switch_pose_debug) {
  all_success_curve_path_first_gear_switch_pose_debug.clear();
  if (cur_path_generator_ == nullptr) {
    return false;
  }
  all_success_curve_path_first_gear_switch_pose_debug =
      cur_path_generator_->GetAllSuccessCurvePathFirstGearSwitchPoseForDebug();
  return true;
}

const bool HybridAstarPathGeneratorInterface::GetPreSearchABBoxForDebug(
    cdl::AABB& pre_search_abbox) {
  pre_search_abbox.Reset();
  if (cur_path_generator_ == nullptr) {
    return false;
  }
  pre_search_abbox = cur_path_generator_->GetPreSearchABBoxForDebug();
  return true;
}

}  // namespace apa_planner
}  // namespace planning