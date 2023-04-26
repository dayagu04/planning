#ifndef ZNQC_MODULES_CONTEXT_VIRTUAL_LANE_MANAGER_H_
#define ZNQC_MODULES_CONTEXT_VIRTUAL_LANE_MANAGER_H_

#include "src/modules/context/virtual_lane.h"
#include "src/modules/context/virtual_lane_manager.h"
#include "src/modules/context/intersection.h"
#include "src/modules/context/Ramp.h"
#include "../../res/include/proto/fusion_road.pb.h"
#include <vector>

namespace planning {

enum LaneChangeStatus {
  NO_LANE_CHANGE = 0,
  ON_LEFT_LANE = 1,
  ON_RIGHT_LANE = 2,
};

class VirtualLaneManager {
 public:
  VirtualLaneManager(planning::framework::Session *session);
  // VirtualLaneManager() = default;
  ~VirtualLaneManager() {};

  std::shared_ptr<VirtualLane> get_left_neighbor(std::shared_ptr<VirtualLane> this_lane) const {
    return this_lane->get_order_id() > 0
           ? relative_id_lanes_[this_lane->get_order_id() - 1] : nullptr;
  }
  std::shared_ptr<VirtualLane> get_right_neighbor(std::shared_ptr<VirtualLane> this_lane) const {
     return this_lane->get_order_id() < static_cast<int>(relative_id_lanes_.size()) - 1
              ? relative_id_lanes_[this_lane->get_order_id() + 1] : nullptr;
  }

  const std::shared_ptr<VirtualLane> get_current_lane() const { return current_lane_; }
  const std::shared_ptr<VirtualLane> get_left_lane() const { return left_lane_; }
  const std::shared_ptr<VirtualLane> get_right_lane() const { return right_lane_; }
  const std::shared_ptr<VirtualLane> get_lane_with_virtual_id(int virtual_id) const;
  const std::shared_ptr<VirtualLane> get_lane_with_order_id(uint order_id) const;
  int current_lane_virtual_id() {
    if (current_lane_ != nullptr) {
      return current_lane_->get_virtual_id();
    }
  }
  std::shared_ptr<VirtualLane> get_mutable_lane_with_virtual_id(int virtual_id);
  std::vector<std::shared_ptr<VirtualLane>>& get_virtual_lanes() { return relative_id_lanes_; }
  uint get_lane_num() const { return relative_id_lanes_.size(); };
  std::vector<std::shared_ptr<Obstacle>> get_current_lane_obstacle();
  std::vector<std::shared_ptr<Obstacle>> get_left_lane_obstacle();
  std::vector<std::shared_ptr<Obstacle>> get_right_lane_obstacle();
  bool has_lane(int virtual_lane_id);
  const Intersection &get_intersection_info() const { return intersection_; }
  const Ramp &get_ramp() const { return ramp_; }

  //Destination destination_;

  //void update_current_lane();
  void update_last_fix_lane_id(int flane_virtual_id) { last_fix_lane_virtual_id_ = flane_virtual_id;  }
  bool update(const FusionRoad::RoadInfo& roads);
  void reset();

  double get_distance_to_dash_line(const RequestType direction, uint order_id) const {
    return std::numeric_limits<double>::max();
  }

  double get_distance_to_final_dash_line(const RequestType direction, uint order_id) const;

  int get_lane_index(const std::shared_ptr<VirtualLane> virtual_lane) const;
  int get_tasks(const std::shared_ptr<VirtualLane> virtual_lane) const;
  bool must_change_lane(const std::shared_ptr<VirtualLane> virtual_lane, double on_route_distance_threshold) const;
  int lc_map_decision(const std::shared_ptr<VirtualLane> virtual_lane) const;
  double lc_map_decision_offset(const std::shared_ptr<VirtualLane> virtual_lane) const {
    //HACK
    return 5000.;
  };

 private:
  LaneChangeStatus is_lane_change();
  void update_virtual_id();

  planning::framework::Session *session_ = nullptr;
  //ReferencePathManager reference_path_manager_;
  int last_fix_lane_virtual_id_ = 0;
  int current_lane_virtual_id_ = 0;
  std::unordered_map<int, std::shared_ptr<VirtualLane>> virtual_id_mapped_lane_;
  std::vector<std::shared_ptr<VirtualLane>> relative_id_lanes_;
  std::shared_ptr<VirtualLane> current_lane_ = nullptr;
  std::shared_ptr<VirtualLane> left_lane_ = nullptr;
  std::shared_ptr<VirtualLane> right_lane_ = nullptr;

  double last_left_diff_ = 0;
  double last_right_diff_ = 0;
  Intersection intersection_;
  Ramp ramp_;
};
}
#endif