#ifndef ZNQC_MODULES_CONTEXT_VIRTUAL_LANE_H_
#define ZNQC_MODULES_CONTEXT_VIRTUAL_LANE_H_

#include <float.h>
#include <limits.h>

#include <vector>

#include "camera_perception_lane_lines_c.h"
#include "config/basic_type.h"
#include "fusion_road_c.h"
#include "lane_reference_path.h"
#include "log.h"
#include "map_info_manager.h"
#include "reference_path_manager.h"
#include "refline.h"
#include "session.h"

// #include "virtual_lane_manager.h"

namespace planning {

struct SpeedChangePoint {
  double x;
  double y;
  double speed;
};

// hack :clren
// struct VirtualLaneMember {
//   int order_id_ = -1;
//   int virtual_id_ = 0;
//   int relative_id_ = 0;
//   float ego_lateral_offset_ = 0;
//   LaneStatusEx lane_status_;
//   iflyauto::LaneType lane_type_;
//   iflyauto::LaneDrivableDirection lane_marks_;
//   iflyauto::LaneSource lane_source_;
//   iflyauto::LaneReferenceLine lane_reference_line_;
//   std::vector<double> c_poly_;
//   iflyauto::LaneMergeSplitPoint lane_merge_split_point_;
//   iflyauto::LaneBoundary left_lane_boundary_;
//   iflyauto::LaneBoundary right_lane_boundary_;
// };

class VirtualLane {
 public:
  VirtualLane();
  ~VirtualLane() = default;

 public:
  void update_data(const iflyauto::ReferenceLineMsg &lane);
  void set_order_id(uint order_id) { order_id_ = order_id; };
  void set_virtual_id(int virtual_id) { virtual_id_ = virtual_id; };
  void set_relative_id(int relative_id) { relative_id_ = relative_id; };

  uint get_order_id() const { return order_id_; };
  int get_virtual_id() const { return virtual_id_; };
  int get_relative_id() const { return relative_id_; };
  double width_by_s(double s);
  double width(double x);
  double width() { return width_; };

  // WB：用户设置巡航车速不应与地图限速耦合
  double velocity_limit() const { return v_cruise_; };
  const std::vector<double> &c_poly() const { return c_poly_; }
  LaneStatusEx status() { return lane_status_; }
  const iflyauto::LaneBoundary &get_left_lane_boundary() {
    return left_lane_boundary_;
  }
  const iflyauto::LaneBoundary &get_right_lane_boundary() {
    return right_lane_boundary_;
  }
  const iflyauto::LaneBoundary &get_stop_line() { return stop_line_; }

  const std::vector<double> &get_center_line() const { return c_poly_; };
  const iflyauto::LaneMergeSplitPoint &get_lane_merge_split_point() {
    return lane_merge_split_point_;
  }
  const std::vector<iflyauto::ReferencePoint> &lane_points() const {
    return virtual_lane_refline_points_;
  }

  const std::vector<iflyauto::LaneTypeMsg> &get_lane_types() {
    return lane_types_;
  }

  void update_reference_path(
      std::shared_ptr<LaneReferencePath> reference_path) {
    // assert(reference_path != nullptr);
    if (reference_path == nullptr) {
      LOG_ERROR("reference_path is null! \n");
    }
    reference_path_ = reference_path;
  };

  const std::shared_ptr<LaneReferencePath> get_reference_path() {
    return reference_path_;
  }
  void set_ego_lateral_offset(double ego_lateral_offset) {
    ego_lateral_offset_ = ego_lateral_offset;
  };

  double get_ego_lateral_offset() const { return ego_lateral_offset_; };

  void set_lane_frenet_coord(
      std::shared_ptr<planning_math::KDPath> frenet_coord) {
    lane_frenet_coord_ = frenet_coord;
  };

  const std::shared_ptr<planning_math::KDPath> get_lane_frenet_coord() {
    return lane_frenet_coord_;
  };

  iflyauto::LaneType get_lane_type() const {
    return lane_types_.size() > 0 ? lane_types_[0].type
                                  : iflyauto::LANETYPE_UNKNOWN;
  };
  iflyauto::LaneDrivableDirection get_lane_marks() const {
    return lane_marks_.size() > 0
               ? lane_marks_[0].lane_mark
               : iflyauto::LaneDrivableDirection_DIRECTION_UNKNOWN;
  };
  std::vector<iflyauto::LaneMarkMsg> lane_marks() const { return lane_marks_; }
  iflyauto::LaneSource get_lane_source() const {
    return lane_sources_.size() > 0 ? lane_sources_[0].source
                                    : iflyauto::LaneSource_SOURCE_UNKNOWN;
  };

  // 能让车沿着route形式，在当前位置所在的lanegroup中，最少需要变道几次
  // +： right; -: left
  bool has_lines(LineDirection direction) const;
  double distance_to_line(double s, double l, LineDirection direction);

  uint get_common_point_num(const std::shared_ptr<VirtualLane> &other);

  bool get_point_by_distance(double distance, iflyauto::ReferencePoint &point);

  const std::vector<std::string> &center_line_points_track_id() const {
    return center_line_points_track_id_;
  }

  // side: 0-left, 1-right
  bool is_dash_line(
      const planning::framework::Session &session,
      const RequestType request_type,
      const std::shared_ptr<planning_math::KDPath> target_boundary_path) const;
  double min_width() const;
  double max_width() const;
  bool hack() const { return hack_; }

  void update_lane_tasks(const RouteInfoOutput &route_info_output);
  const std::vector<int> &get_current_tasks() const { return current_tasks_; };
  // 到最远变道点距离，即：为了不出route，在该车道最远可以继续行驶的距离

  void update_speed_limit(double ego_vel, double ego_v_cruise);
  void save_context(VirtualLaneContext &context) const;
  void restore_context(const VirtualLaneContext &context);
  bool calc_c_poly(std::vector<double> &output);
  void set_is_in_merge_area(bool is_in_merge_area) {
    is_in_merge_area_ = is_in_merge_area;
  }
  bool is_nearing_ramp_mlc_task() const { return is_nearing_ramp_mlc_task_; }
  bool is_nearing_split_mlc_task() const { return is_nearing_split_mlc_task_; }
  void ProcessEgoOnRoadMLC(const RouteInfoOutput &route_info_output);
  void ProcessEgoOnRampMLC(const RouteInfoOutput &route_info_output);

 private:
  planning::framework::Session *session_ = nullptr;
  int order_id_ = -1;

  int virtual_id_ = 0;
  int relative_id_ = 0;
  float ego_lateral_offset_ = 0;
  double width_ = 2.8;
  std::shared_ptr<planning_math::KDPath> lane_frenet_coord_;
  LaneStatusEx lane_status_;
  std::vector<iflyauto::LaneTypeMsg> lane_types_;
  std::vector<iflyauto::LaneMarkMsg> lane_marks_;
  std::vector<iflyauto::LaneSourceMsg> lane_sources_;
  std::vector<iflyauto::ReferencePoint> virtual_lane_refline_points_;
  std::vector<double> c_poly_;
  iflyauto::LaneMergeSplitPoint lane_merge_split_point_;
  iflyauto::LaneBoundary left_lane_boundary_;
  iflyauto::LaneBoundary right_lane_boundary_;

  iflyauto::LaneBoundary stop_line_;

  std::vector<std::string> center_line_points_track_id_;
  // todo:clren 后面改成map，适配多种reference_path
  std::shared_ptr<LaneReferencePath> reference_path_;

  std::vector<int> current_tasks_;
  bool hack_ = false;

  SpeedChangePoint speed_change_point_{};
  double v_cruise_ = 0.0;
  double current_lane_speed_limit_ = 0.0;
  bool is_in_merge_area_ = false;
  bool is_nearing_ramp_mlc_task_ = false;
  bool is_nearing_split_mlc_task_ = false;
};
}  // namespace planning
#endif