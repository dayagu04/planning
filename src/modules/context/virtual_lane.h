#ifndef ZNQC_MODULES_CONTEXT_VIRTUAL_LANE_H_
#define ZNQC_MODULES_CONTEXT_VIRTUAL_LANE_H_

#include "common/config/basic_type.h"
#include "lane_lines.pb.h"
#include "fusion_road.pb.h"
#include "context/reference_path_manager.h"
#include "context/lane_reference_path.h"
#include "common/refline.h"
#include <float.h>
#include <limits.h>

namespace planning {

struct SpeedChangePoint {
  double x;
  double y;
  double speed;
};

// hack :clren
struct VirtualLaneMember {
  int order_id_ = -1;
  int virtual_id_ = 0;
  int relative_id_ = 0;
  float ego_lateral_offset_ = 0;
  LaneStatusEx lane_status_;
  FusionRoad::LaneType lane_type_;
  FusionRoad::LaneDrivableDirection lane_marks_;
  FusionRoad::LaneSource lane_source_;
  FusionRoad::LaneReferenceLine lane_reference_line_;
  std::vector<double> c_poly_;
  FusionRoad::LaneMergeSplitPoint lane_merge_split_point_;
  FusionRoad::LaneBoundary left_lane_boundary_;
  FusionRoad::LaneBoundary right_lane_boundary_;
};

class VirtualLane {
 public:
  VirtualLane();
  ~VirtualLane() = default;

 public:
  void update_data(const FusionRoad::Lane &lane);
  void set_order_id(uint order_id) { order_id_ = order_id; };
  void set_virtual_id(int virtual_id) { virtual_id_ = virtual_id; };
  void set_relative_id(int relative_id) { relative_id_ = relative_id; };

  uint get_order_id() const { return order_id_; };
  int get_virtual_id() const { return virtual_id_; };
  int get_relative_id() const { return relative_id_; };
  double width_by_s(double s);
  double width(double x);
  double width();
  // WB：用户设置巡航车速不应与地图限速耦合
  double velocity_limit() const { return v_cruise_; };
  const std::vector<double> &c_poly() const { return c_poly_; }
  LaneStatusEx status() { return lane_status_; }
  const FusionRoad::LaneBoundary &get_left_lane_boundary() {
    return left_lane_boundary_;
  }
  const FusionRoad::LaneBoundary &get_right_lane_boundary() {
    return right_lane_boundary_;
  }
  const FusionRoad::LaneReferenceLine &get_center_line() const {
    return lane_reference_line_;
  };
  const FusionRoad::LaneMergeSplitPoint &get_lane_merge_split_point() { return lane_merge_split_point_; }
  const google::protobuf::RepeatedPtrField<FusionRoad::VirtualLanePoint> &lane_points() const {
    return lane_reference_line_.virtual_lane_refline_points();
  }
  void update_reference_path(std::shared_ptr<LaneReferencePath> reference_path) {
    //assert(reference_path != nullptr);
    if (reference_path == nullptr)
      std::cout<<"reference_path is null"<<std::endl;
    // printf("reference_path_:%p\n",reference_path_);
    // printf("reference_path:%p\n",reference_path);
    reference_path_ = reference_path;
  };

  const std::shared_ptr<LaneReferencePath> get_reference_path() { return reference_path_; }
  double get_ego_lateral_offset() const { return ego_lateral_offset_; };
  FusionRoad::LaneType get_lane_type() const { return lane_type_; };
  FusionRoad::LaneDrivableDirection get_lane_marks() const {
    return lane_marks_;
  };
  FusionRoad::LaneSource get_lane_source() const { return lane_source_; };

  // 能让车沿着route形式，在当前位置所在的lanegroup中，最少需要变道几次
  // +： right; -: left
  bool has_lines(LineDirection direction) const;
  double distance_to_line(double s, double l, LineDirection direction);
  bool is_obstacle_on(int obj_id);

  uint get_common_point_num(const std::shared_ptr<VirtualLane> &other);

  bool get_point_by_distance(double distance,
                             FusionRoad::VirtualLanePoint *point);

  const std::vector<std::string> &center_line_points_track_id() const {
    return center_line_points_track_id_;
  }

  // side: 0-left, 1-right
  bool is_solid_line(int side) const;

  double min_width();
  double max_width();
  bool hack() const { return hack_; }
  const std::vector<int> &get_current_tasks() const { return current_tasks_; };
  // 到最远变道点距离，即：为了不出route，在该车道最远可以继续行驶的距离

  void update_speed_limit(double ego_vel, double ego_v_cruise);
  void save_context(VirtualLaneContext &context) const;
  void restore_context(const VirtualLaneContext &context);
  bool calc_c_poly(std::vector<double> &output);

 private:
  int order_id_ = -1;

  int virtual_id_ = 0;
  int relative_id_ = 0;
  float ego_lateral_offset_ = 0;
  LaneStatusEx lane_status_;
  FusionRoad::LaneType lane_type_;
  FusionRoad::LaneDrivableDirection lane_marks_;
  FusionRoad::LaneSource lane_source_;
  FusionRoad::LaneReferenceLine lane_reference_line_;
  std::vector<double> c_poly_;
  FusionRoad::LaneMergeSplitPoint lane_merge_split_point_;
  FusionRoad::LaneBoundary left_lane_boundary_;
  FusionRoad::LaneBoundary right_lane_boundary_;

  std::vector<std::string> center_line_points_track_id_;
  // todo:clren 后面改成map，适配多种reference_path
  std::shared_ptr<LaneReferencePath>  reference_path_;

  std::vector<int> current_tasks_;
  bool hack_ = false;

  SpeedChangePoint speed_change_point_{};
  double v_cruise_ = 0.0;
  double current_lane_speed_limit_ = 0.0;
};
}  // namespace planning
#endif