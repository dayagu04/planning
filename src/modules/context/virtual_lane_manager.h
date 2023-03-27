#ifndef ZNQC_MODULES_CONTEXT_VIRTUAL_LANE_MANAGER_H_
#define ZNQC_MODULES_CONTEXT_VIRTUAL_LANE_MANAGER_H_

#include <iomanip>
#include <unordered_map>

#include "framework/session.h"
#include "modules/common/config/basic_type.h"
#include "modules/common/transform.h"
#include "modules/context/vehicle_config_context.h"
#include "modules/context/environmental_model.h"
#include "../res/include/proto/asw_proto_road_fusion.pb.h"

namespace planning {

class Obstacle;

struct VirtualLanePoint {
  VirtualPoint2D car_point;
  VirtualPoint3D enu_point;
  double curvature = 0;
  double yaw = 0;
  double distance_to_left_road_border = 1.6;
  double distance_to_right_road_border = 1.6;
  double distance_to_left_lane_border = 1.6;
  double distance_to_right_lane_border = 1.6;
  double lane_width = 3.2;
  double speed_limit = std::numeric_limits<double>::max();  // unit: m/s
  int track_id;
  LaneBoundaryType left_road_border_type;
  LaneBoundaryType right_road_border_type;
  LaneBoundaryType left_lane_border_type;
  LaneBoundaryType right_lane_border_type;
  bool on_route = true;
  bool is_in_intersection = false;
  Asw::RoadFusion::LaneType lane_type = Asw::RoadFusion::LaneType::LANE_TYPE_UNKNOWN;
};

class VirtualLane {
 public:
  VirtualLane() {}
  // VirtualLane(const planning::RefLine &ref_line,  //
  //             const planning::Transform &enu2car,
  //             const planning::common::CartEgoState &ego_state);

  virtual ~VirtualLane() = default;

  // order_id: 有效的车道线（车道中心线点数量大于5）从左到右的顺序，最左车道为0
  int order_id() const { return order_id_; }
  // lane的track id。连续多帧的相同的车道，virtual id相同
  int virtual_id() const { return virtual_id_; }

  const std::vector<VirtualLanePoint> &lane_points() const {
    return lane_points_;
  }
  int size() const { return lane_points_.size(); }

  // // 车道投影到车道中心线的距离，单位是m, 没有正负(左右)关系
  // double fabs_lateral_offset() const { return std::fabs(lateral_offset_); }
  // double calc_fabs_lateral_offset(const double x, const double y) const;

  // 车道投影到车道中心线的距离，单位是m, 有正负(左右)关系
  double lateral_offset() const { return lateral_offset_; }
  double calc_lateral_offset(const double x, const double y) const;
  double calc_lateral_offset(
      const std::shared_ptr<VirtualLane> target_lane) const {
    if (target_lane == nullptr) {
      LOG_ERROR("calc_lateral_offset: target_lane not exist");
      return std::numeric_limits<double>::max();
    } else {
      return (target_lane->lateral_offset() - lateral_offset());
    }
  }

  void set_order_id(int order_id) { order_id_ = order_id; }
  void set_virtual_id(int virtual_id) { virtual_id_ = virtual_id; }

  int get_common_point_num(const std::shared_ptr<VirtualLane> other) const;
  bool is_obstacle_on(const Obstacle *obstacle) const;
  bool get_point_by_distance(double distance, VirtualLanePoint &point) const;
 
  Asw::RoadFusion::LaneType lane_type() const { return lane_type_; }
  const std::unordered_set<int> &lane_type_set() const {
    return lane_type_set_;
  }
  void init(planning::framework::Session *session);

  // 能让车沿着route形式，在当前位置所在的lanegroup中，最少需要变道几次
  // +： right; -: left
  int lc_map_decision() const {
    // HACK
    return 0;
  }

  // 到最远变道点距离，即：为了不出route，在该车道最远可以继续行驶的距离
  double lc_map_decision_offset() const {
    //HACK
    return 5000.;
  };

  // 到最远变道点距离是否小于给定距离阈值，用来判断是否应该开始换道。
  double must_change_lane(double on_route_distance_threshold) const {
    return lc_map_decision() != 0 &&
           lc_map_decision_offset() < on_route_distance_threshold;
  }

 private:
  std::vector<VirtualLanePoint> lane_points_{};

  int virtual_id_ = -1;
  int order_id_ = -1;
  double lateral_offset_ = 0;
  std::unordered_set<int> lane_type_set_{};
  Direction lane_marks_ = Direction::DIRECTION_UNKNOWN;
  Asw::RoadFusion::LaneType lane_type_ = Asw::RoadFusion::LaneType::LANE_TYPE_UNKNOWN;
  std::vector<int> track_ids_;

};


class VirtualLaneManager {
 public:
  VirtualLaneManager(planning::framework::Session *session);
  ~VirtualLaneManager() {}

  // bool is_on_highway() const {
  //   return session_->mutable_environmental_model()->is_on_highway();
  // }
  // bool ego_in_intersection() const {
  //   return session_->mutable_environmental_model()->is_in_intersection();
  // }

  // 在该lane上，到下一个实线车道线的距离
  // 如果当前是实线车道线，即返回0
  double get_distance_to_dash_line(const RequestType direction,
                                   int order_id) const;

  // calculate distance to merge && split point

  // 就是到最远变道点的距离
  double get_distance_to_final_dash_line(const RequestType direction,
                                         int order_id) {
    if (order_id < 0 ||
        order_id >= static_cast<int>(order_id_to_lane_.size())) {
      LOG_ERROR("wrong order_id %d", order_id);
      //HACK
      return 5000.;
    }
    return order_id_to_lane_.at(order_id)->lc_map_decision_offset();
  }

  double velocity_limit() const;
  double map_velocity_limit() const;
  double user_velocity_limit() const;

  ///////////
  bool update();
  int lanes_num() const { return order_id_to_lane_.size(); }

  bool has_lane(int virtual_id) const {
    if (virtual_id_to_lane_.count(virtual_id) == 0) {
      LOG_ERROR("[VirtualLaneManager::has_lane] Virtual id: %d not found",
            virtual_id);
      return false;
    }
    return true;
  }
  int current_lane_virtual_id() const { return current_lane_virtual_id_; }
  int left_lane_virtual_id() const {
    return left_lane() == nullptr ? -1 : left_lane()->virtual_id();
  }
  int right_lane_virtual_id() const {
    return right_lane() == nullptr ? -1 : right_lane()->virtual_id();
  }

  std::shared_ptr<VirtualLane> current_lane() const { return current_lane_; }
  std::shared_ptr<VirtualLane> left_lane() const { return left_lane_; }
  std::shared_ptr<VirtualLane> right_lane() const { return right_lane_; }

  std::shared_ptr<VirtualLane> get_lane_with_virtual_id(int virtual_id) const {
    return has_lane(virtual_id) ? virtual_id_to_lane_.at(virtual_id) : nullptr;
  }
  std::shared_ptr<VirtualLane> get_lane_with_order_id(int order_id) const {
    return order_id >= 0 &&
                   order_id < static_cast<int>(order_id_to_lane_.size())
               ? order_id_to_lane_[order_id]
               : nullptr;
  }
  std::shared_ptr<VirtualLane> get_left_neighbor(
      std::shared_ptr<VirtualLane> this_lane) const {
    return this_lane->order_id() > 0
               ? order_id_to_lane_[this_lane->order_id() - 1]
               : nullptr;
  }
  std::shared_ptr<VirtualLane> get_right_neighbor(
      std::shared_ptr<VirtualLane> this_lane) const {
    return this_lane->order_id() <
                   static_cast<int>(order_id_to_lane_.size()) - 1
               ? order_id_to_lane_[this_lane->order_id() + 1]
               : nullptr;
  }

  int get_left_next_order_id_with_offset(int origin_lane_order_id) const;
  int get_right_next_order_id_with_offset(int origin_lane_order_id) const;

  const std::vector<std::shared_ptr<VirtualLane>> &get_virtual_lanes() const {
    return order_id_to_lane_;
  }
  void update_last_fix_lane_id(int last_fix_lane_id) {
    last_fix_lane_id_ = last_fix_lane_id;
  }

 private:
  std::unordered_map<int, std::shared_ptr<VirtualLane>> virtual_id_to_lane_;
  std::vector<std::shared_ptr<VirtualLane>> order_id_to_lane_;
  std::shared_ptr<VirtualLane> current_lane_;
  std::shared_ptr<VirtualLane> left_lane_;
  std::shared_ptr<VirtualLane> right_lane_;
  planning::framework::Session *session_ = nullptr;
  int current_lane_virtual_id_ = 0;
  int next_id_ = 0;
  int last_fix_lane_id_ = -1;

  void update_current_lane();

};

// 计算两个virtual lane相同的点
int get_common_point_num(const std::shared_ptr<VirtualLane> lane1,
                         const std::shared_ptr<VirtualLane> lane2);
// 比较两个virtual lane在车所在的lane group处是否共线（共享同一根lane
// centerline）
bool have_same_root(const std::shared_ptr<VirtualLane> lane1,
                    const std::shared_ptr<VirtualLane> lane2);

}
#endif