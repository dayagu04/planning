#include "context/virtual_lane_manager.h"
#include "context/reference_path_manager.h"


namespace planning {

VirtualLaneManager::VirtualLaneManager(planning::framework::Session *session) {
  session_ = session;
}

bool VirtualLaneManager::update(const FusionRoad::RoadInfo& roads) {
  LOG_DEBUG("update VirtualLaneManager\n");
  current_lane_ = nullptr;
  left_lane_ = nullptr;
  right_lane_ = nullptr;
  relative_id_lanes_.clear();

  if(roads.lanes().size() == 0 ) {
    LOG_ERROR("roads' lanes are empty \n");
    return false;
  }
  for(auto& lane : roads.lanes()) {
    std::shared_ptr<VirtualLane> virtual_lane_tmp = std::make_shared<VirtualLane>();
    virtual_lane_tmp->update_data(lane);
    printf("lane relative_id:%d, order_id:%d, virtual_id:%d\n", lane.relative_id(),
    lane.order_id(), lane.virtual_id());
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

  for (auto relative_id_lane : relative_id_lanes_) {
    if (relative_id_lane->get_relative_id() == 0) {
      current_lane_ = relative_id_lane;
      LOG_DEBUG("create current_lane_\n");
    } else if (relative_id_lane->get_relative_id() == -1) {
      left_lane_ = relative_id_lane;
    } else if (relative_id_lane->get_relative_id() == 1) {
      right_lane_ = relative_id_lane;
    }
  }

  update_virtual_id();
  LOG_DEBUG("current lane virtual id:%d\n", current_lane_virtual_id_);
  for (auto lane : relative_id_lanes_) {
    LOG_DEBUG(" %d", lane->get_virtual_id());
  }
  LOG_DEBUG("\n");
  return true;
}

const std::shared_ptr<VirtualLane> VirtualLaneManager::get_lane_with_virtual_id(int virtual_id) const{
  if (virtual_id_mapped_lane_.find(virtual_id) != virtual_id_mapped_lane_.end()) {
    LOG_DEBUG("get lane virtual %d id\n", virtual_id);
    return virtual_id_mapped_lane_[virtual_id];
  } else {
    LOG_DEBUG("lane virtual %d id is null\n", virtual_id);
    return nullptr;
  }
}

const std::shared_ptr<VirtualLane> VirtualLaneManager::get_lane_with_order_id(uint order_id) const{
  if (order_id > relative_id_lanes_.size() -1) {
    return nullptr;
  }
  return relative_id_lanes_.at(order_id);
}

void VirtualLaneManager::update_virtual_id() {
  LaneChangeStatus change_status = is_lane_change();
  int lane_virtual_id;
  if (change_status == LaneChangeStatus::NO_LANE_CHANGE) {
    lane_virtual_id  = current_lane_virtual_id_;
  } else if (change_status == LaneChangeStatus::ON_LEFT_LANE) {
    lane_virtual_id  = current_lane_virtual_id_ - 1;
  } else {
    lane_virtual_id  = current_lane_virtual_id_ + 1;
  }
  current_lane_->set_virtual_id(lane_virtual_id);
  if (left_lane_ != nullptr) {
    left_lane_->set_virtual_id(lane_virtual_id - 1);
  }
  if (right_lane_ != nullptr) {
    right_lane_->set_virtual_id(lane_virtual_id + 1);
  }

  current_lane_virtual_id_ = lane_virtual_id;
  virtual_id_mapped_lane_.clear();
  for (auto lane : relative_id_lanes_) {
    virtual_id_mapped_lane_[lane->get_relative_id() + current_lane_virtual_id_] = lane;
    LOG_DEBUG("virtual_id_mapped_lane_ virtual id :%d\n",
      lane->get_relative_id() + current_lane_virtual_id_);
  }
}

LaneChangeStatus VirtualLaneManager::is_lane_change() {
  LaneChangeStatus change_status = LaneChangeStatus::NO_LANE_CHANGE;
  const float lane_change_thre = 1;

  if (virtual_id_mapped_lane_.find(current_lane_virtual_id_) != virtual_id_mapped_lane_.end()) {
    auto last_virtual_lane = virtual_id_mapped_lane_[current_lane_virtual_id_];
    double left_C0, right_C0, last_left_C0, last_right_C0;
    if (last_virtual_lane->get_left_lane_boundary().existence()) {
      last_left_C0 = last_virtual_lane->get_left_lane_boundary().poly_coefficient(0);
    }
    else {
      last_left_C0 = 8.0;
    }

    if (last_virtual_lane->get_right_lane_boundary().existence()) {
      last_right_C0 = last_virtual_lane->get_right_lane_boundary().poly_coefficient(0);
    }
    else {
      last_right_C0 = -8.0;
    }

    if (current_lane_->get_left_lane_boundary().existence()) {
      left_C0 = current_lane_->get_left_lane_boundary().poly_coefficient(0);
    }
    else {
      left_C0 = 8.0;
    }

    if (current_lane_->get_right_lane_boundary().existence()) {
      right_C0 = current_lane_->get_right_lane_boundary().poly_coefficient(0);
    }
    else {
      right_C0 = -8.0;
    }

    double left_diff = left_C0 - last_left_C0;
    double right_diff = right_C0 - last_right_C0;

    if (left_diff > lane_change_thre && right_diff > lane_change_thre && last_left_diff_ < lane_change_thre) {
      change_status = ON_LEFT_LANE;
    } else if (left_diff < -lane_change_thre && right_diff < -lane_change_thre && last_right_diff_ > -lane_change_thre) {
      change_status = ON_RIGHT_LANE;
    }
    last_left_diff_ = left_diff;
    last_right_diff_ = right_diff;
    std::cout<<"last_left_C0: " << last_left_C0 << " left_C0: " << left_C0 << " left_diff: " << left_diff << " last_right_C0: " << last_right_C0 << " right_C0: " << right_C0 << " right_diff: " << right_diff << " last_left_diff_: " << last_left_diff_ << " change_status: " << change_status << std::endl;

  } else {
    last_left_diff_ = 0;
    last_right_diff_ = 0;
  }

  return change_status;
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

double VirtualLaneManager::get_distance_to_final_dash_line(const RequestType direction, uint order_id) const{
  auto virtual_lane = get_lane_with_order_id(order_id);
  if (virtual_lane == nullptr) {
    return std::numeric_limits<double>::max();
  }
  return lc_map_decision_offset(virtual_lane);
}

int VirtualLaneManager::get_lane_index(const std::shared_ptr<VirtualLane> virtual_lane) const {
  if(virtual_lane != nullptr) {
    return virtual_lane->get_relative_id() - relative_id_lanes_.at(0)->get_relative_id();
  }
  return 0;
}

int VirtualLaneManager::get_tasks(const std::shared_ptr<VirtualLane> virtual_lane) const {
  int current_tasks = 0;
  if(virtual_lane != nullptr) {
    auto current_tasks_vector = virtual_lane->get_current_tasks();
    if (current_tasks_vector.empty()) {
      return 0;
    }
    for (int i = 0; i < current_tasks_vector.size(); i++) {
      if (current_tasks_vector[i] != current_tasks_vector[0]) {
        break;
      }
      current_tasks += current_tasks_vector[i];
    }
    // clip tasks according to lane nums
    int lane_index = get_lane_index(virtual_lane);
    int right_lane_nums = std::max((int)get_lane_num() - lane_index - 1, 0);
    int left_lane_nums = lane_index;
    current_tasks = std::max(std::min(current_tasks, right_lane_nums), -left_lane_nums);
    return current_tasks;
  } else {
    return 0;
  }
}

bool VirtualLaneManager::must_change_lane(const std::shared_ptr<VirtualLane> virtual_lane, double on_route_distance_threshold) const {
  if (virtual_lane == nullptr) {
    return 0;
  }
  return lc_map_decision(virtual_lane) != 0 && lc_map_decision_offset(virtual_lane) < on_route_distance_threshold;
}

int VirtualLaneManager::lc_map_decision(const std::shared_ptr<VirtualLane> virtual_lane) const {
  if(virtual_lane == nullptr) {
    return 0;
  }
  int tasks_id = get_tasks(virtual_lane);
  int lane_index = get_lane_index(virtual_lane);

  // hack valid and on rightest way
  if (virtual_lane->hack() && lane_index == (get_lane_num() - 1)) {
    if (tasks_id <= 0) {
      tasks_id = std::min(tasks_id, -1);
    }
  }

  return tasks_id;
}

}  // namespace planning