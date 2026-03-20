#include "construction_warning_hmi.h"

#include "planning_context.h"
#include "environmental_model.h"

namespace planning {
const static int kStartRunningCount = 3;
const static int kStopRunningCount = 3;
ConstructionWarningHMIDecider::ConstructionWarningHMIDecider(
    framework::Session* session,
    const HmiDeciderConfig& config) {
  session_ = session;
  config_ = config;
  has_enough_speed_hysteresis_.SetThreValue(
      config_.construction_warning_hmi_speed_max + 5,
      config_.construction_warning_hmi_speed_max - 5);
}

bool ConstructionWarningHMIDecider::Execute() {
  if (session_ == nullptr) {
    return false;
  }

  has_construction_ = HasConstruction();

  switch (current_state_) {
    case ConstructionWarningState::IDLE:
      if (IsStartRunning()) {
        current_state_ = ConstructionWarningState::RUNNING;
        stop_running_count_ = 0;
      }
      break;

    case ConstructionWarningState::RUNNING:
      if (IsStopRunning()) {
        current_state_ = ConstructionWarningState::EXITING;
        start_running_count_ = 0;
      }
      break;

    case ConstructionWarningState::EXITING:
      current_state_ = ConstructionWarningState::IDLE;
      stop_running_count_ = 0;
      stop_running_count_ = 0;
      break;
  }
  JSON_DEBUG_VALUE("ConstructionWarningState",
                    static_cast<uint32_t>(current_state_));
  SaveHmiOutput();
  return true;
}

bool ConstructionWarningHMIDecider::HasConstruction() {
  const auto &construction_scene = session_->environmental_model()
                                       .get_construction_scene_manager()
                                       ->get_construction_scene_output();
  if (!construction_scene.is_exist_construction_area) {
    return false;
  }
  const auto ego_cart_state_manager =
      session_->environmental_model().get_ego_state_manager();
  has_enough_speed_hysteresis_.SetIsValidByValue(
      ego_cart_state_manager->ego_v() * 3.6);
  bool is_high_speed = has_enough_speed_hysteresis_.IsValid();
  double lateral_distance_to_centerline_thr = 6;
  int construction_agent_numble_thr = 5;
  double construction_area_length_thr = 5;
  double lon_tcc_to_construction_area_thr = 5;
  if (!is_high_speed) {
    // 低速时条件更加严格
    lateral_distance_to_centerline_thr = 4;
    construction_agent_numble_thr = 6;
    construction_area_length_thr = 10;
    lon_tcc_to_construction_area_thr = 3.5;
  }
  const auto &frenet_coord =
      session_->planning_context()
          .lane_change_decider_output()
          .coarse_planning_info.reference_path->get_frenet_coord();
  const auto ego_s =
      session_->planning_context()
          .lane_change_decider_output()
          .coarse_planning_info.reference_path->get_frenet_ego_state()
          .s();
  // 检查横向距离条件
  const auto &cluster_is_construction_area_map =
      construction_scene.cluster_is_construction_area_map;
  const auto &construction_agent_cluster_attribute_map =
      construction_scene.construction_agent_cluster_attribute_map;
  for (const auto & [ cluster_id, cluster_area ] :
       construction_agent_cluster_attribute_map) {
    auto it = cluster_is_construction_area_map.find(cluster_id);
    if (it == cluster_is_construction_area_map.end() || !it->second) {
      continue;
    }
    if (cluster_area.points.size() < construction_agent_numble_thr) {
      // 数量达到要求
      continue;
    }
    double construction_nearest_l = std::numeric_limits<double>::max();
    double construction_area_s_start = std::numeric_limits<double>::max();
    double construction_area_s_end = std::numeric_limits<double>::lowest();
    for (const auto &pt : cluster_area.points) {
      Point2D point_xy(pt.x, pt.y);
      Point2D point_sl;
      if (!frenet_coord->XYToSL(point_xy, point_sl)) {
        continue;
      }
      construction_nearest_l = std::abs(point_sl.y) <
                      std::abs(construction_nearest_l)
                      ? std::abs(point_sl.y)
                      : construction_nearest_l;
      construction_area_s_start = point_sl.x <
                      construction_area_s_start
                      ? point_sl.x
                      : construction_area_s_start;
      construction_area_s_end = point_sl.x >
                      construction_area_s_end
                      ? point_sl.x
                      : construction_area_s_end;
    }
    if (construction_nearest_l > lateral_distance_to_centerline_thr) {
      continue;
    }
    if (construction_area_s_end - construction_area_s_start <
        construction_area_length_thr) {
      continue;
    }
    double ego_v = std::fmax(ego_cart_state_manager->ego_v(), 0.01);
    double lon_ttc = std::fmax((construction_area_s_start - ego_s) / ego_v, 0);
    if (lon_ttc > lon_tcc_to_construction_area_thr) {
      continue;
    }
    return true;
  }
  return false;
}

bool ConstructionWarningHMIDecider::IsStartRunning() {
  if (has_construction_) {
    start_running_count_ =
        std::min(kStartRunningCount, start_running_count_ + 1);
  }

  if (start_running_count_ >= kStartRunningCount) {
    return true;
  }
  return false;
}

bool ConstructionWarningHMIDecider::IsStopRunning() {
  if (!has_construction_) {
    stop_running_count_ = std::min(kStartRunningCount, stop_running_count_ + 1);
  }

  if (stop_running_count_ >= kStopRunningCount) {
    return true;
  }
  return false;
}

void ConstructionWarningHMIDecider::SaveHmiOutput() {
  const auto& construction_scene = session_->environmental_model()
                                       .get_construction_scene_manager()
                                       ->get_construction_scene_output();
  auto hmi_info =
      session_->mutable_planning_context()->mutable_planning_hmi_info();
  if (current_state_ == ConstructionWarningState::RUNNING) {
    if (construction_scene.is_pass_construction_area) {
      hmi_info->ad_info.construction_info.construction_state =
          iflyauto::ConstructionState::CONSTRUCTION_IN;
    } else if (construction_scene.is_exist_construction_area) {
      hmi_info->ad_info.construction_info.construction_state =
          iflyauto::ConstructionState::CONSTRUCTION_APPROACH;
    } else {
      hmi_info->ad_info.construction_info.construction_state =
          last_construction_state_;
    }
  } else {
    hmi_info->ad_info.construction_info.construction_state =
        iflyauto::ConstructionState::CONSTRUCTION_NONE;
  }

  last_construction_state_ =
      hmi_info->ad_info.construction_info.construction_state;
}

void ConstructionWarningHMIDecider::Reset() {
  start_running_count_ = 0;
  stop_running_count_ = 0;

  current_state_ = ConstructionWarningState::IDLE;
  last_construction_state_ = iflyauto::ConstructionState::CONSTRUCTION_NONE;
}

}  // namespace planning