#ifndef ZNQC_MODULES_CONTEXT_VIRTUAL_LANE_MANAGER_H_
#define ZNQC_MODULES_CONTEXT_VIRTUAL_LANE_MANAGER_H_

#include <climits>
#include <memory>
#include <queue>
#include <utility>
#include <vector>

#include "ad_common/hdmap/hdmap.h"
#include "ego_lane_track_manager.h"
#include "ego_planning_config.h"
#include "fusion_road_c.h"
#include "generated_refline.pb.h"
#include "intersection.h"
#include "local_view.h"
#include "route_info.h"
#include "sdmap/sdmap.h"
#include "session.h"
#include "virtual_lane.h"

namespace planning {

#define LANE_BOUNDARY_POINT_SET_NUM 20

using ad_common::hdmap::LaneGroupConstPtr;
using Map::CurrentRouting;

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

  void SetConfig(const EgoPlanningConfigBuilder *config_builder);

  void UpdateAllVirtualLaneInfo();

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
    if (current_lane_ == nullptr) {
      ILOG_DEBUG << "current_lane_ is nullptr";
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
    }
    current_lane_virtual_id_ = current_lane_->get_virtual_id();
    return current_lane_->get_virtual_id();
  }
  std::shared_ptr<VirtualLane> mutable_lane_with_virtual_id(int virtual_id) {
    if (virtual_id_mapped_lane_.find(virtual_id) !=
        virtual_id_mapped_lane_.end()) {
      ILOG_DEBUG << "get mutable lane virtual id " << virtual_id;
      return virtual_id_mapped_lane_[virtual_id];
    } else {
      ILOG_DEBUG << "mutable lane virtual " << virtual_id << "id is null";
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
  void update_last_fix_lane_id(int fix_lane_virtual_id) {
    last_fix_lane_virtual_id_ = fix_lane_virtual_id;
  }
  int get_last_fix_lane_id() const { return last_fix_lane_virtual_id_; }
  const std::shared_ptr<VirtualLane> get_last_fix_lane() const {
    return get_lane_with_virtual_id(last_fix_lane_virtual_id_);
  }

  std::vector<double> construct_reference_line_acc(void);
  std::vector<double> construct_reference_line_scc(void);
  bool update(const iflyauto::RoadInfo &roads);

  bool update(const iflyauto::FuncStateMachine &func_state_machine_msg);

  bool CheckLaneValid(const iflyauto::RoadInfo &roads);

  void reset();

  std::shared_ptr<VirtualLane> GetNearestLane(Point2D point, double *nearest_s,
                                              double *nearest_l);

  void set_is_exist_split_on_ramp(const bool is_exist_split_on_ramp) {
    is_exist_split_on_ramp_ = is_exist_split_on_ramp;
  }

  bool get_is_exist_split_on_ramp() const { return is_exist_split_on_ramp_; };

  void set_is_exist_ramp_on_road(const bool is_exist_ramp_on_road) {
    is_exist_ramp_on_road_ = is_exist_ramp_on_road;
  }

  bool get_is_exist_ramp_on_road() const { return is_exist_ramp_on_road_; };

  void set_is_exist_split_on_expressway(
      const bool is_exist_split_on_expressway) {
    is_exist_split_on_expressway_ = is_exist_split_on_expressway;
  }

  bool get_is_exist_split_on_expressway() const {
    return is_exist_split_on_expressway_;
  };

  void set_is_exist_intersection_split(const bool is_exist_intersection_split) {
    is_exist_intersection_split_ = is_exist_intersection_split;
  }

  bool get_is_exist_intersection_split() const {
    return is_exist_intersection_split_;
  };

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
    double lc_end_dis = route_info_output_.dis_to_ramp;
    if (lc_map_decision(virtual_lane) < 0) {
      lc_end_dis = route_info_output_.distance_to_first_road_split;
    }
    return lc_end_dis;
  };

  const double dis_threshold_to_merged_point() const {
    return dis_threshold_to_is_merged_point_;
  }

  const double dis_threshold_to_last_merge_point() const {
    return dis_threshold_to_last_merge_point_;
  }
  bool is_continuous_ramp() const { return is_continuous_ramp_; }

  // bool is_local_valid() const { return is_local_valid_; }

  int origin_relative_id_zero_nums() const {
    return origin_relative_id_zero_nums_;
  }

  void construct_reference_line_msg(
      const std::vector<double> &current_lane_virtual_poly,
      iflyauto::ReferenceLineMsg &current_lane_virtual);
  void construct_reference_line_msg(
      const iflyauto::FuncStateMachine &func_state_machine_msg,
      iflyauto::ReferenceLineMsg *const current_lane_virtual);
  void ExtendReferenceLineForRads(
      const iflyauto::FuncStateMachine &func_state_machine_msg,
      iflyauto::ReferenceLineMsg *const current_lane_virtual);
  double GetEgoDistanceToStopline() { return distance_to_stopline_; };
  double GetEgoDistanceToCrosswalk() { return distance_to_crosswalk_; };
  const planning::common::IntersectionState GetIntersectionState() {
    return Intersection_state_;
  };
  bool IsPosXOnVirtualLaneType(double x_pos);

  std::vector<int> GetZeroRelativeIdOrderIds() {
    return order_ids_of_same_zero_relative_id_;
  }

  std::shared_ptr<planning_math::KDPath> MakeBoundaryPath(
      const iflyauto::LaneBoundary &boundary);

 private:
  void SetGeneratedReflineToDebugInfo(
      const iflyauto::LaneReferenceLine &refline);
  std::vector<std::shared_ptr<VirtualLane>> UpdateLanes(
      const iflyauto::RoadInfo *roads_ptr);
  bool UpdateEgoDistanceToStopline();
  bool UpdateEgoDistanceToCrosswalk(const iflyauto::RoadInfo *roads_ptr);
  bool UpdateIntersectionState();
  bool IsEgoBothSidesHaveRoadBorder();

  planning::framework::Session *session_ = nullptr;
  EgoPlanningVirtualLaneManagerConfig config_;
  EgoLaneTrackManger ego_lane_track_manager_;
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
  double ego_pose_x_;
  double ego_pose_y_;
  double yaw_;
  // bool in_intersection_ = false;
  iflyauto::ReferenceLineMsg intersection_lane_generated_;

  // NOA
  RouteInfoOutput route_info_output_;
  bool is_continuous_ramp_ = false;
  bool is_accumulate_dis_to_last_merge_point_more_than_threshold_ = false;
  double sum_dis_to_last_split_point_ = NL_NMAX;
  bool virtual_lane_relative_id_switch_flag_ = false;
  bool is_exist_split_on_ramp_ = false;
  bool is_exist_ramp_on_road_ = false;
  bool is_exist_split_on_expressway_ = false;
  bool is_exist_intersection_split_ = false;
  double current_segment_passed_distance_ = 0.0;
  const double dis_threshold_to_last_merge_point_ = 600.0;
  const double dis_threshold_to_is_merged_point_ = 800.0;
  std::pair<SplitRelativeDirection, double> first_split_dir_dis_info_;
  std::vector<std::pair<SplitRelativeDirection, double>>
      split_dir_dis_info_list_;
  bool is_select_split_nearing_ramp_ = true;

  int origin_relative_id_zero_nums_ = 0;
  std::vector<int> order_ids_of_same_zero_relative_id_;
  // bool is_within_hdmap_ = false;

  // 到停止线的距离，可以为负，表示停止线在车后
  double distance_to_stopline_ = NL_NMAX;
  double distance_to_crosswalk_ = NL_NMAX;
  std::deque<double> stopline_window_ = {NL_NMAX, NL_NMAX, NL_NMAX};
  std::deque<double> crosswalk_window_ = {NL_NMAX, NL_NMAX, NL_NMAX};
  planning::common::IntersectionState Intersection_state_ =
      planning::common::NO_INTERSECTION;

  // rads relevance
  iflyauto::FunctionalState last_fsm_state_ =
      iflyauto::FunctionalState::FunctionalState_RADS_ABORT;
};
}  // namespace planning
#endif