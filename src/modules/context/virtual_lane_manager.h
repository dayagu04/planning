#ifndef ZNQC_MODULES_CONTEXT_VIRTUAL_LANE_MANAGER_H_
#define ZNQC_MODULES_CONTEXT_VIRTUAL_LANE_MANAGER_H_

#include <climits>
#include <vector>
#include "ad_common/hdmap/hdmap.h"
#include "ego_planning_config.h"
#include "fusion_road.pb.h"
#include "intersection.h"
#include "local_view.h"
#include "log.h"
#include "session.h"
#include "virtual_lane.h"
namespace planning {

using Map::CurrentRouting;
using Map::CurrentRouting;
using ad_common::hdmap::LaneGroupConstPtr;

enum LaneChangeStatus {
  NO_LANE_CHANGE = 0,
  ON_LEFT_LANE = 1,
  ON_RIGHT_LANE = 2,
};

class VirtualLaneManager {
 public:
  VirtualLaneManager(const EgoPlanningConfigBuilder *config_builder,
                     planning::framework::Session *session);
  // VirtualLaneManager() = default;
  ~VirtualLaneManager(){};

  std::shared_ptr<VirtualLane> get_left_neighbor(
      std::shared_ptr<VirtualLane> this_lane) const {
    return this_lane->get_order_id() > 0
               ? relative_id_lanes_[this_lane->get_order_id() - 1]
               : nullptr;
  }
  std::shared_ptr<VirtualLane> get_right_neighbor(
      std::shared_ptr<VirtualLane> this_lane) const {
    return this_lane->get_order_id() <
                   static_cast<int>(relative_id_lanes_.size()) - 1
               ? relative_id_lanes_[this_lane->get_order_id() + 1]
               : nullptr;
  }

  const std::shared_ptr<VirtualLane> get_current_lane() const {
    assert(current_lane_ != nullptr);
    if (current_lane_ == nullptr) {
      LOG_DEBUG("current_lane_ is nullptr\n");
    }
    return current_lane_;
  }

  int current_lane_index() const {
    if (relative_id_lanes_.size() > 0) {
      return 0 - relative_id_lanes_[0]->get_relative_id();
    }
    return 0;
  }

  const std::shared_ptr<VirtualLane> get_left_lane() const {
    return left_lane_;
  }
  const std::shared_ptr<VirtualLane> get_right_lane() const {
    return right_lane_;
  }
  const std::shared_ptr<VirtualLane> get_lane_with_virtual_id(
      int virtual_id) const;
  const std::shared_ptr<VirtualLane> get_lane_with_order_id(
      uint order_id) const;
  int current_lane_virtual_id() {
    // assert(current_lane_ != nullptr);
    if (current_lane_ == nullptr) {
      return current_lane_virtual_id_;
      LOG_DEBUG("current_lane_ is nullptr\n");
    }
    current_lane_virtual_id_ = current_lane_->get_virtual_id();
    return current_lane_->get_virtual_id();
  }
  std::shared_ptr<VirtualLane> mutable_lane_with_virtual_id(int virtual_id) {
    if (virtual_id_mapped_lane_.find(virtual_id) !=
        virtual_id_mapped_lane_.end()) {
      LOG_DEBUG("get mutable lane virtual %d id\n", virtual_id);
      return virtual_id_mapped_lane_[virtual_id];
    } else {
      LOG_DEBUG("mutable lane virtual %d id is null\n", virtual_id);
      return nullptr;
    }
  }
  std::vector<std::shared_ptr<VirtualLane>> &get_virtual_lanes() {
    return relative_id_lanes_;
  }
  uint get_lane_num() const { return lane_num_; };
  std::vector<std::shared_ptr<Obstacle>> get_current_lane_obstacle();
  std::vector<std::shared_ptr<Obstacle>> get_left_lane_obstacle();
  std::vector<std::shared_ptr<Obstacle>> get_right_lane_obstacle();
  bool has_lane(int virtual_lane_id);
  const Intersection &get_intersection_info() const { return intersection_; }
  // const Ramp &get_ramp() const { return ramp_; }

  // Destination destination_;

  // void update_current_lane();
  void update_last_fix_lane_id(int flane_virtual_id) {
    last_fix_lane_virtual_id_ = flane_virtual_id;
  }
  int get_last_fix_lane_id() const { return last_fix_lane_virtual_id_; }
  const std::shared_ptr<VirtualLane> get_last_fix_lane() const {
    return get_lane_with_virtual_id(last_fix_lane_virtual_id_);
  }

  bool update(const FusionRoad::RoadInfo &roads);
  void reset();

  double get_distance_to_dash_line(const RequestType direction,
                                   uint virtual_id) const;
  double get_distance_to_final_dash_line(const RequestType direction,
                                         uint order_id) const;

  double get_distance_to_first_road_merge() const;
  double get_distance_to_first_road_split() const;
  int get_lane_index(const std::shared_ptr<VirtualLane> virtual_lane) const;
  int get_tasks(const std::shared_ptr<VirtualLane> virtual_lane) const;
  bool must_change_lane(const std::shared_ptr<VirtualLane> virtual_lane,
                        double on_route_distance_threshold) const;
  int lc_map_decision(const std::shared_ptr<VirtualLane> virtual_lane) const;
  double lc_map_decision_offset(
      const std::shared_ptr<VirtualLane> virtual_lane) const {
    // HACK
    double lc_end_dis = dis_to_ramp_;
    if (lc_map_decision(virtual_lane) < 0) {
      lc_end_dis = distance_to_first_road_split_;
    }
    return lc_end_dis;
  };

  double dis_to_ramp() const { return dis_to_ramp_; }
  RampDirection ramp_direction() const { return ramp_direction_; }
  double distance_to_first_road_merge() const {
    return distance_to_first_road_merge_;
  }
  double distance_to_first_road_split() const {
    return distance_to_first_road_split_;
  }
  const double GetDistanceToDestination() const {
    return distance_to_destination_;
  }

  bool is_on_ramp() const { return is_on_ramp_; }

  bool is_local_valid() const { return is_local_valid_; }

  bool is_on_hpp_lane() const { return is_on_hpp_lane_; }
  bool is_reached_hpp_start_point() const {
    return is_reached_hpp_start_point_;
  }
  double sum_distance_driving() const { return sum_distance_driving_; }
  double distance_to_target_slot() const { return distance_to_target_slot_; }

  void CalculateDistanceToRamp(planning::framework::Session *session);
  void CalculateDistanceToFirstRoadSplit(planning::framework::Session *session);
  void CalculateDistanceToFirstRoadMerge(planning::framework::Session *session);

 private:
  LaneChangeStatus is_lane_change();
  void update_virtual_id();

  double JudgeIfTheRamp(const int current_index,
                        const CurrentRouting &current_routing,
                        const ad_common::hdmap::HDMap &hd_map);
  void CalculateRampDirection(const ad_common::hdmap::HDMap &hd_map,
                              LaneGroupConstPtr lane_group_ptr);
  double JudgeIfTheFirstSplit(const int current_index,
                              const CurrentRouting &current_routing,
                              const ad_common::hdmap::HDMap &hd_map) const;
  double JudgeIfTheFirstMerge(const int current_index,
                              const CurrentRouting &current_routing,
                              const ad_common::hdmap::HDMap &hd_map) const;
  bool GetCurrentIndexAndDis(const planning::framework::Session &session,
                             int *current_index, double *remaining_dis);

  bool CalculateSortedLaneGroupIdsInRouting(
      const planning::framework::Session &session);

  bool JudgeEgoIfOnRamp(const planning::framework::Session &session);

  bool GetCurrentNearestLane(const planning::framework::Session &session);
  void CalculateDistanceToRampSplitMerge(planning::framework::Session *session);
  void CalculateHPPInfo(planning::framework::Session *session);
  void ResetHpp();
  void CalculateDistanceToTargetSlot(planning::framework::Session *session);

  planning::framework::Session *session_ = nullptr;
  EgoPlanningVirtualLaneManagerConfig config_;
  int last_fix_lane_virtual_id_ = 0;
  int current_lane_virtual_id_ = 0;
  std::unordered_map<int, std::shared_ptr<VirtualLane>> virtual_id_mapped_lane_;
  std::vector<std::shared_ptr<VirtualLane>> relative_id_lanes_;
  std::shared_ptr<VirtualLane> current_lane_ = nullptr;
  std::shared_ptr<VirtualLane> left_lane_ = nullptr;
  std::shared_ptr<VirtualLane> right_lane_ = nullptr;
  uint lane_num_ = 0;
  double last_left_diff_ = 0;
  double last_right_diff_ = 0;
  Intersection intersection_;
  // Ramp ramp_;
  double dis_to_ramp_ = NL_NMAX;
  RampDirection ramp_direction_ = RampDirection::RAMP_NONE;
  double distance_to_first_road_merge_ = NL_NMAX;
  double distance_to_first_road_split_ = NL_NMAX;
  bool is_local_valid_ = false;
  bool is_select_split_nearing_ramp_ = true;
  std::unordered_set<uint64_t> lane_group_set_;
  std::vector<uint64_t> sorted_lane_groups_in_route_;
  bool is_leaving_ramp_ = false;
  bool is_on_ramp_ = false;
  ad_common::hdmap::LaneInfoConstPtr nearest_lane_;
  double nearest_s_ = 0.0;
  // HPP
  bool is_on_hpp_lane_ = false;
  bool is_reached_hpp_start_point_ = false;
  double sum_distance_driving_ = -1;
  ad_common::math::Vec2d last_point_hpp_{NL_NMAX, NL_NMAX};
  double distance_to_destination_ = NL_NMAX;
  // target slot
  double distance_to_target_slot_ = NL_NMAX;
};
}  // namespace planning
#endif