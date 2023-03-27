#include "src/modules/context/environmental_model.h"
#include "src/modules/common/trajectory/trajectory_stitcher.h"
#include "src/modules/context/ego_planning_config.h"
#include "src/modules/context/cart_ego_state.h"
#include "src/modules/context/frenet_ego_state.h"
#include "src/modules/context/ego_state_manager.h"
#include "src/modules/context/obstacle_manager.h"
#include "src/modules/context/reference_path_manager.h"
#include "src/modules/context/traffic_light_decision_manager.h"
#include "src/modules/context/virtual_lane_manager.h"

namespace planning {
EnvironmentalModel::EnvironmentalModel() {}

// EnvironmentalModel::~EnvironmentalModel() {}

bool EnvironmentalModel::Init(common::SceneType scene_type) {
  cart_ego_state_manager_ = std::make_shared<CartEgoStateManager>(vehicle_param_);
  return true;
}

bool EnvironmentalModel::Update() {
  return true;
}

void EnvironmentalModel::UpdateMembers() {

}

}  // namespace planning