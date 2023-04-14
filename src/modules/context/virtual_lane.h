#ifndef ZNQC_MODULES_CONTEXT_VIRTUAL_LANE_H_
#define ZNQC_MODULES_CONTEXT_VIRTUAL_LANE_H_

#include "src/modules/common/config/basic_type.h"
#include "../../res/include/proto/lane_lines.pb.h"
#include "../../res/include/proto/fusion_road.pb.h"
#include "src/modules/context/reference_path_manager.h"
#include "src/modules/context/reference_path.h"


namespace planning {

class VirtualLane {
 public:
  VirtualLane();
  ~VirtualLane() = default;
 public:
  void update_data(const FusionRoad::Lane& lane);
  void set_order_id(uint order_id) { order_id_ = order_id; };
  void set_virtual_id (int virtual_id) { virtual_id_ = virtual_id; };
  void set_relative_id(int relative_id) { relative_id_ = relative_id; };

  uint get_order_id() const { return order_id_; };
  int get_virtual_id () const { return virtual_id_; };
  int get_relative_id() const { return relative_id_; };
  const FusionRoad::LaneBoundary &get_left_lane_boundary() { return left_lane_boundary_; }
  const FusionRoad::LaneBoundary &get_right_lane_boundary() { return right_lane_boundary_; }
  const FusionRoad::LaneReferenceLine& get_center_line() const {
    return lane_reference_line_;
  };
  const std::vector<FusionRoad::VirtualLanePoint> &lane_points() const {
    std::vector<FusionRoad::VirtualLanePoint> virtual_lane_points; //todo
    for (auto p : lane_reference_line_.virtual_lane_refline_points()) {
      virtual_lane_points.emplace_back(p);
    }
    return virtual_lane_points;
  } 
  void update_reference_path(std::shared_ptr<ReferencePath> reference_path) {
    reference_path_ = reference_path;
  };
  double get_ego_lateral_offset() const { return ego_lateral_offset_; };
  FusionRoad::LaneType get_lane_type() const { return lane_type_; };
  FusionRoad::LaneDrivableDirection get_lane_marks() const { return lane_marks_; };
  FusionRoad::LaneSource get_lane_source() const { return lane_source_; };

  //int lc_map_decision();
  //double lc_end_dist();
  bool has_lines(LineDirection direction) const;
  double distance_to_line(double s, double l, LineDirection direction);
  bool is_obstacle_on(const Obstacle &tr);

  uint get_common_point_num(const std::shared_ptr<VirtualLane> &other);

  bool get_point_by_distance(double distance, FusionRoad::VirtualLanePoint *point);

  const std::vector<std::string> &center_line_points_track_id() const {
    return center_line_points_track_id_;
  }

 private:
  int order_id_ = 0;
  int virtual_id_ = 0;
  int relative_id_ = 0;
  float ego_lateral_offset_ = 0;
  LaneStatusEx lane_status_;
  FusionRoad::LaneType lane_type_;
  FusionRoad::LaneDrivableDirection lane_marks_;
  FusionRoad::LaneSource lane_source_;
  FusionRoad::LaneReferenceLine lane_reference_line_;

  FusionRoad::LaneMergeSplitPoint lane_merge_split_point_;
  FusionRoad::LaneBoundary left_lane_boundary_;
  FusionRoad::LaneBoundary right_lane_boundary_;

  std::vector<std::string> center_line_points_track_id_;
  std::shared_ptr<ReferencePath> reference_path_ = nullptr;
};
}
#endif