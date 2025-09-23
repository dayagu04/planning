#include "hybrid_astar_path_generator_interface.h"

namespace planning {
namespace apa_planner {

#define PUBLISH_SEARCH_NODE_LIST (0)
#define PUBLISH_CURVE_NODE_LIST (0)

HybridAstarPathGeneratorInterface::HybridAstarPathGeneratorInterface() {
  hybrid_astar_perpendicular_tail_in_path_generator_ptr_ =
      std::make_shared<HybridAStarPerpendicularTailInPathGenerator>();

  // todo: add more func

  // Init();
}

HybridAstarPathGeneratorInterface::HybridAstarPathGeneratorInterface(
    const std::shared_ptr<CollisionDetectorInterface>& col_det_interface_ptr) {
  hybrid_astar_perpendicular_tail_in_path_generator_ptr_ =
      std::make_shared<HybridAStarPerpendicularTailInPathGenerator>(
          col_det_interface_ptr);

  // todo: add more func

  // Init();
}

void HybridAstarPathGeneratorInterface::Init() {
  if (init_flag_) {
    return;
  }
  // todo: more func init
  hybrid_astar_perpendicular_tail_in_path_generator_ptr_->Init();

  init_flag_ = true;
}

void HybridAstarPathGeneratorInterface::Reset() {
  // todo: more func reset
  hybrid_astar_perpendicular_tail_in_path_generator_ptr_->Reset();

  init_flag_ = false;
  scenario_type_ = ParkingScenarioType::SCENARIO_UNKNOWN;
}

HybridAstarPathGeneratorInterface::~HybridAstarPathGeneratorInterface() {}

const bool HybridAstarPathGeneratorInterface::Update() {
  switch (scenario_type_) {
    case ParkingScenarioType::SCENARIO_PERPENDICULAR_TAIL_IN:
      return hybrid_astar_perpendicular_tail_in_path_generator_ptr_->Update();
    default:
      return false;
  }

  return false;
}

void HybridAstarPathGeneratorInterface::SetRequest(
    const HybridAStarRequest& request) {
  scenario_type_ = request.scenario_type;
  switch (scenario_type_) {
    case ParkingScenarioType::SCENARIO_PERPENDICULAR_TAIL_IN:
      hybrid_astar_perpendicular_tail_in_path_generator_ptr_->SetRequest(
          request);
      break;
    default:
      break;
  }
}

void HybridAstarPathGeneratorInterface::SetColDetIntefacePtr(
    const std::shared_ptr<CollisionDetectorInterface>& col_det_interface_ptr) {
  switch (scenario_type_) {
    case ParkingScenarioType::SCENARIO_PERPENDICULAR_TAIL_IN:
      hybrid_astar_perpendicular_tail_in_path_generator_ptr_
          ->SetCollisionDetectorIntefacePtr(col_det_interface_ptr);
      break;
    default:
      break;
  }

  return;
}

void HybridAstarPathGeneratorInterface::UpdateConfig(
    const PlannerOpenSpaceConfig& config) {
  switch (scenario_type_) {
    case ParkingScenarioType::SCENARIO_PERPENDICULAR_TAIL_IN:
      hybrid_astar_perpendicular_tail_in_path_generator_ptr_->UpdateConfig(
          config);
      break;
    default:
      break;
  }

  return;
}

const bool HybridAstarPathGeneratorInterface::GetResult(
    HybridAStarResult& result) {
  result.Clear();
  switch (scenario_type_) {
    case ParkingScenarioType::SCENARIO_PERPENDICULAR_TAIL_IN:
      result =
          hybrid_astar_perpendicular_tail_in_path_generator_ptr_->GetResult();
    default:
      return false;
  }
  return true;
}

const bool HybridAstarPathGeneratorInterface::GetChildNodeForDebug(
    std::vector<DebugAstarSearchPoint>& child_node_debug) {
  child_node_debug.clear();
  switch (scenario_type_) {
    case ParkingScenarioType::SCENARIO_PERPENDICULAR_TAIL_IN:
      child_node_debug = hybrid_astar_perpendicular_tail_in_path_generator_ptr_
                             ->GetChildNodeForDebug();
    default:
      return false;
  }
  return true;
}

const bool HybridAstarPathGeneratorInterface::GetQueuePathForDebug(
    std::vector<Eigen::Vector2d>& queue_path) {
  queue_path.clear();
  switch (scenario_type_) {
    case ParkingScenarioType::SCENARIO_PERPENDICULAR_TAIL_IN:
      queue_path = hybrid_astar_perpendicular_tail_in_path_generator_ptr_
                       ->GetQueuePathForDebug();
    default:
      return false;
  }
  return true;
}

const bool HybridAstarPathGeneratorInterface::GetDeleteQueuePathForDebug(
    std::vector<Eigen::Vector2d>& del_queue_path) {
  del_queue_path.clear();
  switch (scenario_type_) {
    case ParkingScenarioType::SCENARIO_PERPENDICULAR_TAIL_IN:
      del_queue_path = hybrid_astar_perpendicular_tail_in_path_generator_ptr_
                           ->GetDeleteQueuePathForDebug();
    default:
      return false;
  }
  return true;
}

const bool HybridAstarPathGeneratorInterface::GetSearchNodeListMessage(
    std::vector<std::vector<Eigen::Vector2d>>& search_node_list) {
  search_node_list.clear();
  if (!PUBLISH_SEARCH_NODE_LIST) {
    return false;
  }
  switch (scenario_type_) {
    case ParkingScenarioType::SCENARIO_PERPENDICULAR_TAIL_IN:
      search_node_list = hybrid_astar_perpendicular_tail_in_path_generator_ptr_
                             ->GetSearchNodeListMessage();
    default:
      return false;
  }
  return true;
}

const bool HybridAstarPathGeneratorInterface::GetCurveNodeListMessage(
    std::vector<std::vector<Eigen::Vector2d>>& curve_node_list) {
  curve_node_list.clear();
  if (!PUBLISH_CURVE_NODE_LIST) {
    return false;
  }
  switch (scenario_type_) {
    case ParkingScenarioType::SCENARIO_PERPENDICULAR_TAIL_IN:
      curve_node_list = hybrid_astar_perpendicular_tail_in_path_generator_ptr_
                            ->GetCurveNodeListMessage();
    default:
      return false;
  }
  return true;
}

const bool HybridAstarPathGeneratorInterface::GetAllSuccessCurvePathForDebug(
    std::vector<CurvePath>& all_success_curve_path_debug) {
  all_success_curve_path_debug.clear();
  switch (scenario_type_) {
    case ParkingScenarioType::SCENARIO_PERPENDICULAR_TAIL_IN:
      all_success_curve_path_debug =
          hybrid_astar_perpendicular_tail_in_path_generator_ptr_
              ->GetAllSuccessCurvePathForDebug();
    default:
      return false;
  }
  return true;
}

const bool HybridAstarPathGeneratorInterface::
    GetAllSuccessCurvePathFirstGearSwitchPoseForDebug(
        std::vector<geometry_lib::PathPoint>&
            all_success_curve_path_first_gear_switch_pose_debug) {
  all_success_curve_path_first_gear_switch_pose_debug.clear();
  switch (scenario_type_) {
    case ParkingScenarioType::SCENARIO_PERPENDICULAR_TAIL_IN:
      all_success_curve_path_first_gear_switch_pose_debug =
          hybrid_astar_perpendicular_tail_in_path_generator_ptr_
              ->GetAllSuccessCurvePathFirstGearSwitchPoseForDebug();
    default:
      return false;
  }
  return true;
}

const bool HybridAstarPathGeneratorInterface::GetPreSearchABBoxForDebug(
    cdl::AABB& pre_search_abbox) {
  pre_search_abbox.Reset();
  switch (scenario_type_) {
    case ParkingScenarioType::SCENARIO_PERPENDICULAR_TAIL_IN:
      pre_search_abbox = hybrid_astar_perpendicular_tail_in_path_generator_ptr_
                             ->GetPreSearchABBoxForDebug();
    default:
      return false;
  }
  return true;
}

}  // namespace apa_planner
}  // namespace planning