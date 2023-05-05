#include "context/environmental_model.h"
#include "common/trajectory/trajectory_stitcher.h"
#include "context/ego_planning_config.h"
#include "context/frenet_ego_state.h"
#include "context/ego_state_manager.h"
#include "context/obstacle_manager.h"
#include "context/reference_path_manager.h"
#include "context/traffic_light_decision_manager.h"
#include "context/virtual_lane_manager.h"
#include "context/lateral_obstacle.h"

namespace planning {
EnvironmentalModel::EnvironmentalModel() {}

// EnvironmentalModel::~EnvironmentalModel() {}

bool EnvironmentalModel::Init(common::SceneType scene_type) {
  return true;
}

bool EnvironmentalModel::Update() {

  return true;
}

void EnvironmentalModel::UpdateMembers() {

}

}  // namespace planning