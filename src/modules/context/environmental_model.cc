#include "environmental_model.h"
#include "debug_info_log.h"
#include "ego_planning_config.h"
#include "ego_state_manager.h"
#include "frenet_ego_state.h"
#include "history_obstacle_manager.h"
#include "lateral_obstacle.h"
#include "obstacle_manager.h"
#include "reference_path_manager.h"
#include "traffic_light_decision_manager.h"
#include "trajectory/trajectory_stitcher.h"
#include "virtual_lane_manager.h"

namespace {
constexpr uint64_t kStaticMapOvertimeThreshold = 20000000;  // 20s
}  // namespace

namespace planning {
EnvironmentalModel::EnvironmentalModel() {}

// EnvironmentalModel::~EnvironmentalModel() {}

bool EnvironmentalModel::Init(common::SceneType scene_type) { return true; }

bool EnvironmentalModel::Update() { return true; }

void EnvironmentalModel::UpdateMembers() {}

}  // namespace planning