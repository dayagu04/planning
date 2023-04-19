#include "modules/context/virtual_lane_manager.h"
#include "modules/context/reference_path_manager.h"


namespace planning {

VirtualLaneManager::VirtualLaneManager(planning::framework::Session *session) {
  session_ = session;
}

double VirtualLaneManager::get_distance_to_dash_line(
    const RequestType direction, int order_id) const {
  return 0.0;
}

bool VirtualLaneManager::update(const FusionRoad::RoadInfo& roads) {
  current_lane_ = nullptr;
  left_lane_ = nullptr;
  right_lane_ = nullptr;
  relative_id_lanes_.clear();

  for(auto& lane : roads.lanes()) {
    std::shared_ptr<VirtualLane> virtual_lane_tmp = std::make_shared<VirtualLane>();
    virtual_lane_tmp->update_data(lane);
    relative_id_lanes_.emplace_back(virtual_lane_tmp);
  }

  auto compare_relative_id = [&](std::shared_ptr<VirtualLane> lane1, std::shared_ptr<VirtualLane> lane2) {
    return  lane1->get_relative_id() < lane2->get_relative_id();
  };
  std::sort(relative_id_lanes_.begin(), relative_id_lanes_.end(), compare_relative_id);

  // if (relative_id_lanes_.size() == 0) {
  //   std::shared_ptr<VirtualLane> virtual_lane_tmp = std::make_shared<VirtualLane>();
  //   FusionRoad::Lane lane;
  //   lane.set_order_id(0);
  //   lane.set_relative_id(0);
  //   lane.set_ego_lateral_offset(0);
  //   lane.set_lane_type(FusionRoad::LaneType::LANE_TYPE_NORMAL);
  //   lane.set_lane_marks(FusionRoad::LaneDrivableDirection::DIRECTION_STRAIGHT);
  //   lane.mutable_lane_reference_line().set_existence(false);
  //   lane.mutable_lane_merge_split_point.set_existence(false);
  //   lane.mutable_left_lane_boundary.set_existence(false);
  //   lane.mutable_right_lane_boundary.set_existence(false);
  //   virtual_lane_tmp->update_data(lane);
  //   relative_id_lanes_.emplace_back(virtual_lane_tmp);
  // }

  for (auto relative_id_lanes : relative_id_lanes_) {
    if (relative_id_lanes->get_relative_id() == 0) {
      current_lane_ = relative_id_lanes;
    } else if (relative_id_lanes->get_relative_id() == -1) {
      left_lane_ = relative_id_lanes;
    } else if (relative_id_lanes->get_relative_id() == 1) {
      right_lane_ = relative_id_lanes;
    }
  }

  update_virtual_id();
}

const std::shared_ptr<VirtualLane> VirtualLaneManager::get_lane_with_virtual_id(int virtual_id) {
  if (virtual_id_mapped_lane_.find(virtual_id) != virtual_id_mapped_lane_.end()) {
    return virtual_id_mapped_lane_[virtual_id];
  }
  else {
    return nullptr;
  }
}

// std::shared_ptr<VirtualLane> VirtualLaneManager::get_lane_with_order_id(uint order_id) {
//   if(order_id_mapped_lanes_.size() < order_id + 1) {
//     return nullptr;
//   }
//   return order_id_mapped_lanes_.at(order_id);
// }

void VirtualLaneManager::update_virtual_id() {
  virtual_id_mapped_lane_.clear();
  LaneChangeStatus change_statu = is_lane_change();
  int lane_virtual_id;
  if (change_statu == LaneChangeStatus::NO_LANE_CHANGE) {
    lane_virtual_id  = current_lane_virtual_id_;
  } else if (change_statu == LaneChangeStatus::ON_LEFT_LANE) {
    lane_virtual_id  = current_lane_virtual_id_ - 1;
  } else {
    lane_virtual_id  = current_lane_virtual_id_ + 1;
  }

  virtual_id_mapped_lane_[lane_virtual_id] = current_lane_;
  virtual_id_mapped_lane_[lane_virtual_id - 1] = left_lane_;
  virtual_id_mapped_lane_[lane_virtual_id + 1] = right_lane_;

  current_lane_->set_virtual_id(lane_virtual_id);
  if (left_lane_ != nullptr) {
    left_lane_->set_virtual_id(lane_virtual_id - 1);
  }
  if (right_lane_ != nullptr) {
    right_lane_->set_virtual_id(lane_virtual_id + 1);
  }

  current_lane_virtual_id_ = lane_virtual_id;
}

LaneChangeStatus VirtualLaneManager::is_lane_change() {

  LaneChangeStatus change_statu = LaneChangeStatus::NO_LANE_CHANGE;
  const float lane_change_thre = 1;

  if (virtual_id_mapped_lane_.find(current_lane_virtual_id_) != virtual_id_mapped_lane_.end()) {
    auto last_virtual_lane = virtual_id_mapped_lane_[current_lane_virtual_id_];
    double left_C0, right_C0, last_left_C0, last_right_C0;
    if (last_virtual_lane->get_left_lane_boundary().existence()) {
      last_left_C0 = last_virtual_lane->get_left_lane_boundary().poly_coefficient().data()[0];
    }
    else {
      last_left_C0 = 8.0;
    }

    if (last_virtual_lane->get_right_lane_boundary().existence()) {
      last_right_C0 = last_virtual_lane->get_right_lane_boundary().poly_coefficient().data()[0];
    }
    else {
      last_right_C0 = -8.0;
    }

    if (current_lane_->get_left_lane_boundary().existence()) {
      left_C0 = current_lane_->get_left_lane_boundary().poly_coefficient().data()[0];
    }
    else {
      left_C0 = 8.0;
    }

    if (current_lane_->get_right_lane_boundary().existence()) {
      right_C0 = current_lane_->get_right_lane_boundary().poly_coefficient().data()[0];
    }
    else {
      right_C0 = -8.0;
    }

    double left_diff = left_C0 - last_left_C0;
    double right_diff = right_C0 - last_right_C0;

    if (left_diff > lane_change_thre && right_diff > lane_change_thre && last_left_diff_ < lane_change_thre) {
      change_statu = ON_LEFT_LANE;
    } else if (left_diff < -lane_change_thre && right_diff < -lane_change_thre && last_right_diff_ > -lane_change_thre) {
      change_statu = ON_RIGHT_LANE;
    }
    last_left_diff_ = left_diff;
    last_right_diff_ = right_diff;
  } else {
  last_left_diff_ = 0;
  last_right_diff_ = 0;
}

  return change_statu;
}

void VirtualLaneManager::reset() {
  last_fix_lane_virtual_id_ = 0;
  current_lane_virtual_id_ = 0;
  virtual_id_mapped_lane_.clear();
  relative_id_lanes_.clear();
  //order_id_mapped_lanes_.clear();
  current_lane_ = nullptr;
  left_lane_ = nullptr;
  right_lane_ = nullptr;
  last_left_diff_ = 0;
  last_right_diff_ = 0;
}

std::vector<std::shared_ptr<Obstacle>> VirtualLaneManager::get_current_lane_obstacle(){
  std::vector<std::shared_ptr<Obstacle>> tr;
  if(current_lane_ == nullptr) {
    return tr;
  }
  int virtual_id = current_lane_->get_virtual_id();
  auto reference_path_manager = session_->environmental_model().get_reference_path_manager();
  std::shared_ptr<ReferencePath> reference_path = reference_path_manager->get_reference_path_by_lane(virtual_id);
  //tr = reference_path->cal_obstacles_on_lane() //todo
  return tr;
}

std::vector<std::shared_ptr<Obstacle>> VirtualLaneManager::get_left_lane_obstacle(){
  std::vector<std::shared_ptr<Obstacle>> tr;
  if(left_lane_ == nullptr) {
    return tr;
  }
  int virtual_id = left_lane_->get_virtual_id();
  auto reference_path_manager = session_->environmental_model().get_reference_path_manager();
  std::shared_ptr<ReferencePath> reference_path = reference_path_manager->get_reference_path_by_lane(virtual_id);
  //tr = reference_path->cal_obstacles_on_lane() //todo
  return tr;
}

std::vector<std::shared_ptr<Obstacle>> VirtualLaneManager::get_right_lane_obstacle(){
  std::vector<std::shared_ptr<Obstacle>> tr;
  if(right_lane_ == nullptr) {
    return tr;
  }
  int virtual_id = right_lane_->get_virtual_id();
  auto reference_path_manager = session_->environmental_model().get_reference_path_manager();
  std::shared_ptr<ReferencePath> reference_path = reference_path_manager->get_reference_path_by_lane(virtual_id);
  //tr = reference_path->cal_obstacles_on_lane() //todo
  return tr;
}

bool VirtualLaneManager::has_lane(int virtual_lane_id) {
  if (virtual_id_mapped_lane_.find(virtual_lane_id) != virtual_id_mapped_lane_.end()) {
    return true;
  } else {
    return false;
  }
}

void VirtualLaneManager::update_speed_limit(double ego_vel, double ego_v_cruise) { //todo
  // update vision only v_cruise_
  if (current_lane_ != nullptr) {
    if(current_lane_->get_lane_source() == FusionRoad::LaneSource::SOURCE_FUSION) {
      v_cruise_ = ego_v_cruise;
      return;
    }
  }

  auto &referece_path_points = current_lane_->get_reference_path()->get_points();
  if (referece_path_points.size() > 1) {
    double last_speed;
    bool find_last = false;
    bool find_change = false;
    double acc_brake_min = 100.0;
    for (size_t i = 1; i < referece_path_points.size(); ++i) {
      if (!find_last && referece_path_points[i].frenet_point.x > 0.0) { //hack: frenet_point
        find_last = true;
        last_speed = referece_path_points[i - 1].max_velocity;
        current_lane_speed_limit_ = last_speed;
        continue;
      }

      if (find_last && referece_path_points[i].max_velocity != last_speed) {
        double acc_brake = (std::pow(referece_path_points[i].max_velocity, 2) - std::pow(ego_vel, 2)) / std::max(1.0,
                           std::sqrt(std::pow(referece_path_points[i].frenet_point.x, 2) + std::pow(referece_path_points[i].frenet_point.y, 2)));
        if (acc_brake < acc_brake_min) {
          acc_brake_min = acc_brake;
          find_change = true;
          speed_change_point_.x = referece_path_points[i].frenet_point.x;
          speed_change_point_.y = referece_path_points[i].frenet_point.y;
          speed_change_point_.speed = referece_path_points[i].max_velocity;
        }
      }
    }

    if (!find_change) {
      speed_change_point_.x = referece_path_points.back().frenet_point.x;
      speed_change_point_.y = referece_path_points.back().frenet_point.y;
      speed_change_point_.speed = referece_path_points.back().max_velocity;
    }
  }

  current_lane_speed_limit_ = std::min(current_lane_speed_limit_, ego_v_cruise);

  v_cruise_ =
      std::min(current_lane_speed_limit_, speed_change_point_.speed);
}

int VirtualLaneManager::current_lane_index() const {
  if (relative_id_lanes_.size() > 0) {
    return 0 - relative_id_lanes_[0]->get_relative_id();
  }

  return 0;
}

}