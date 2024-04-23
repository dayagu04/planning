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

void EnvironmentalModel::UpdateStaticMap(const LocalView &local_view) {
  const auto static_map_info_current_timestamp =
      local_view.static_map_info.header().timestamp();
  if (static_map_info_current_timestamp != static_map_info_updated_timestamp_) {
    ad_common::hdmap::HDMap hd_map_tmp;
    const int res =
        hd_map_tmp.LoadMapFromProto(local_view.static_map_info.road_map());
    if (res == 0) {
      hd_map_ = std::move(hd_map_tmp);
      hdmap_valid_ = true;
      static_map_info_updated_timestamp_ = static_map_info_current_timestamp;
    }
  }
  if (static_map_info_current_timestamp - static_map_info_updated_timestamp_ >
      kStaticMapOvertimeThreshold) {
    //距离上一次更新时间超过阈值，则认为无效报错
    hdmap_valid_ = false;
    std::cout << "error!!! because more than 20s no update hdmap!!!"
              << std::endl;
  }
  JSON_DEBUG_VALUE("hdmap_valid_", hdmap_valid_)
}

}  // namespace planning