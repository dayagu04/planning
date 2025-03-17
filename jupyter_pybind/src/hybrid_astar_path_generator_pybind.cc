#include <cyber/binary.h>
#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <Eigen/Core>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <iostream>
#include <memory>
#include <vector>

#include "ad_common/math/linear_interpolation.h"

#include "apa_param_config.h"
#include "src/modules/apa_function/parking_scenario/parking_scenario.h"
#include "apa_plan_interface.h"
#include "collision_detection/collision_detection.h"
#include "common.h"
#include "hybrid_a_star.h"
#include "hybrid_astar_common.h"
#include "hybrid_astar_interface.h"
#include "narrow_space_scenario.h"
#include "log_glog.h"
#include "math_lib.h"
#include "polygon_base.h"
#include "pose2d.h"
#include "reeds_shepp.h"
#include "rs_path_interpolate.h"
#include "src/library/convex_collision_detection/aabb2d.h"
#include "src/library/hybrid_astar_lib/hybrid_a_star.h"
#include "src/library/hybrid_astar_lib/hybrid_astar_thread.h"
#include "src/library/occupancy_grid_map/euler_distance_transform.h"
#include "src/library/occupancy_grid_map/point_cloud_obstacle.h"
#include "virtual_wall_decider.h"
#include "src/library/reeds_shepp/reeds_shepp_interface.h"
#include "transform2d.h"
#include "ad_common/math/math_utils.h"

namespace py = pybind11;
using namespace planning::apa_planner;
using namespace planning;
using namespace pnc::geometry_lib;

static std::shared_ptr<planning::HybridAStarInterface> hybrid_astar_interface_;
static planning::apa_planner::ApaPlanInterface*parking_interface = nullptr;

std::vector<Eigen::Vector3d> global_path_;
std::vector<double> global_path_s_;
std::vector<Eigen::Vector3d> rs_path_;
std::vector<Eigen::Vector3d> polynomial_path_;
// 独立的rs，验证rs
std::vector<Eigen::Vector3d> rs_path_alone_;
Eigen::Vector3d car_pose_by_s_;
Eigen::Vector3d global_target_pose_;
Eigen::Vector3d safe_circle_tang_pose_;
Eigen::Vector2d pt_inside_pose_;
// global park space
std::vector<Eigen::Vector2d> corrected_park_space_points_;
Eigen::Vector2d right_obs_start_;

std::vector<Eigen::Vector2d> obs_global_points_;
ParkObstacleList hybrid_astar_obs_;
std::vector<Eigen::Vector4d> obs_line_list_;

std::vector<std::vector<Eigen::Vector2d>> real_time_node_list_;
std::vector<Eigen::Vector2d> search_sequence_path_;
// all search node, not only include: open + close, and include deleted node.
std::vector<Eigen::Vector3d> all_searched_node_;
std::vector<std::vector<Eigen::Vector2d>> rs_h_path_;
std::vector<Eigen::Vector3d> footprint_circle_model_global_;
std::vector<Eigen::Vector3d> footprint_circle_model_local_;
Eigen::Vector3d astar_end_pose_;
Eigen::Vector3d astar_current_path_end_;

#define OBS_SAMPLING_DIST (0.1)

int Init() {
  FilePath::SetName("hybrid_astar_pybind");
  InitGlog(FilePath::GetName().c_str());

  ILOG_INFO << "log init finish";

  // parking
  parking_interface = new planning::apa_planner::ApaPlanInterface();

  parking_interface->Init();

  std::shared_ptr<apa_planner::ParkingScenario> planner =
      parking_interface->GetPlannerByType(
          ParkingScenarioType::SCENARIO_NARROW_SPACE);

  std::shared_ptr<apa_planner::NarrowSpaceScenario> hybrid_astar_park_ =
      std::dynamic_pointer_cast<apa_planner::NarrowSpaceScenario>(planner);

  HybridAStarThreadSolver *thread = hybrid_astar_park_->GetThread();
  hybrid_astar_interface_ = thread->GetHybridAStarInterface();

  if (hybrid_astar_interface_ == nullptr) {
    ILOG_INFO << "hybrid_astar_interface_ is null";
  }

  return 0;
}

int StopPybind() {
  // StopGlog();
  return 0;
}

void ResetHybridAstarPath() {
  global_path_.clear();
  global_path_s_.clear();
  search_sequence_path_.clear();
  rs_h_path_.clear();
  rs_path_.clear();

  return;
}

int GetPathFromHybridAstar(const EgoInfoUnderSlot &ego_slot_info,
                           const double vertical_slot_end_straight_dist,
                           const Eigen::Vector3d &ego_pose) {
  //
  global_path_.clear();
  global_path_s_.clear();
  bool success = false;

  Transform2d tf;
  tf.SetBasePose(Pose2D(ego_slot_info.origin_pose_global.pos[0],
                        ego_slot_info.origin_pose_global.pos[1],
                        ego_slot_info.origin_pose_global.heading));

  size_t i;
  Eigen::Vector2d local_position;
  Eigen::Vector2d global_position;
  double heading;

  HybridAStarResult result;
  AstarSearchState state;
  state = hybrid_astar_interface_->GetFullLengthPath(&result);
  if (result.x.size() > 0) {
    global_path_.reserve(result.x.size());
    global_path_s_.reserve(result.x.size());

    rs_path_.clear();
    polynomial_path_.clear();

    for (i = 0; i < result.x.size(); i++) {
      local_position[0] = result.x[i];
      local_position[1] = result.y[i];

      global_position = ego_slot_info.l2g_tf.GetPos(local_position);

      heading = ego_slot_info.l2g_tf.GetHeading(result.phi[i]);

      // ILOG_INFO << "heading " << heading * 57.4;

      global_path_.emplace_back(
          Eigen::Vector3d(global_position[0], global_position[1], heading));

      global_path_s_.emplace_back(result.accumulated_s[i]);

      if (result.type[i] == AstarPathType::REEDS_SHEPP) {
        rs_path_.emplace_back(
            Eigen::Vector3d(global_position[0], global_position[1], heading));
      }
      if (result.type[i] == AstarPathType::QUNTIC_POLYNOMIAL) {
        polynomial_path_.emplace_back(
            Eigen::Vector3d(global_position[0], global_position[1], heading));
      }
    }
  }

  real_time_node_list_.clear();
  hybrid_astar_interface_->GetNodeListMessage(real_time_node_list_);

  for (int i = 0; i < real_time_node_list_.size(); i++) {
    for (int j = 0; j < real_time_node_list_[i].size(); j++) {
      global_position = ego_slot_info.l2g_tf.GetPos(Eigen::Vector2d(
          real_time_node_list_[i][j].x(), real_time_node_list_[i][j].y()));

      real_time_node_list_[i][j] = global_position;
    }
  }

  const std::vector<ad_common::math::Vec2d> &search_path =
      hybrid_astar_interface_->GetPriorQueueNode();

  search_sequence_path_.clear();
  for (i = 0; i < search_path.size(); i++) {
    local_position[0] = search_path[i].x();
    local_position[1] = search_path[i].y();

    global_position = ego_slot_info.l2g_tf.GetPos(local_position);

    search_sequence_path_.emplace_back(
        Eigen::Vector2d(global_position[0], global_position[1]));
  }

  // plot all searched node
  const std::vector<DebugAstarSearchPoint> &all_search_node =
      hybrid_astar_interface_->GetChildNodeForDebug();

  all_searched_node_.clear();
  double is_safe = 0;
  for (i = 0; i < all_search_node.size(); i++) {
    local_position[0] = all_search_node[i].pos.x;
    local_position[1] = all_search_node[i].pos.y;

    global_position = ego_slot_info.l2g_tf.GetPos(local_position);

    is_safe = all_search_node[i].safe ? 1.0 : 0.0;

    all_searched_node_.emplace_back(
        Eigen::Vector3d(global_position[0], global_position[1], is_safe));
  }

  ILOG_INFO << "rs path copy ";
  std::vector<std::vector<ad_common::math::Vec2d>> path_list;
  hybrid_astar_interface_->GetRSPathHeuristic(path_list);

  rs_h_path_.clear();
  for (i = 0; i < path_list.size(); i++) {
    ILOG_INFO << "path id = " << i;
    std::vector<Eigen::Vector2d> path;
    for (size_t j = 0; j < path_list[i].size(); j++) {
      local_position[0] = path_list[i][j].x();
      local_position[1] = path_list[i][j].y();

      global_position = ego_slot_info.l2g_tf.GetPos(local_position);

      path.emplace_back(
          Eigen::Vector2d(global_position[0], global_position[1]));

      ILOG_INFO << "pos id= " << j << "pos =" << global_position[0] << ","
                << global_position[1];
    }

    rs_h_path_.emplace_back(path);
  }

  // get astar target pose
  const Pose2D target_local = hybrid_astar_interface_->GetAstarTargetPose();
  Pose2D global_pose;

  tf.ULFLocalPoseToGlobal(&global_pose, target_local);

  astar_end_pose_[0] = global_pose.x;
  astar_end_pose_[1] = global_pose.y;
  astar_end_pose_[2] = global_pose.theta;

  // current gear path end
  std::vector<AStarPathPoint> current_gear_path;
  hybrid_astar_interface_->GetFirstSegmentPath(current_gear_path);
  Pose2D local;
  if (current_gear_path.size() == 0) {
    astar_current_path_end_[0] = 0;
    astar_current_path_end_[1] = 0;
    astar_current_path_end_[2] = 0;
  } else {
    local.x = current_gear_path.back().x;
    local.y = current_gear_path.back().y;
    local.theta = current_gear_path.back().phi;

    tf.ULFLocalPoseToGlobal(&global_pose, local);

    astar_current_path_end_[0] = global_pose.x;
    astar_current_path_end_[1] = global_pose.y;
    astar_current_path_end_[2] = global_pose.theta;
  }

  // ego collision check
  EulerDistanceTransform *edt = hybrid_astar_interface_->GetMutableEDT();
  Transform2d ego_local_tf;
  ego_local_tf.SetBasePose(Pose2D(ego_slot_info.cur_pose.pos[0],
                                  ego_slot_info.cur_pose.pos[1],
                                  ego_slot_info.cur_pose.heading));
  bool ego_collision =
      edt->IsCollisionForPoint(&ego_local_tf, AstarPathGear::DRIVE);
  ILOG_INFO << "ego_collision = " << ego_collision;

  return 0;
}

void UpdateFootprintCircle(const Eigen::Vector3d &ego_pose) {
  planning::Pose2D ego_global_pose =
      planning::Pose2D(ego_pose[0], ego_pose[1], ego_pose[2]);
  planning::Transform2d tf;
  tf.SetBasePose(ego_global_pose);

  const EulerDistanceTransform *edt_ =
      hybrid_astar_interface_->GetEulerDistanceTransform();
  const FootPrintCircleList circle_footprint =
      edt_->GetCircleFootPrint(AstarPathGear::NORMAL);
  footprint_circle_model_global_.clear();
  footprint_circle_model_local_.clear();
  const FootPrintCircle *circle = &circle_footprint.max_circle;

  planning::Position2D global_position2d;
  tf.ULFLocalPointToGlobal(&global_position2d, circle->pos);

  footprint_circle_model_global_.push_back(Eigen::Vector3d(
      global_position2d.x, global_position2d.y, circle->radius));
  footprint_circle_model_local_.push_back(Eigen::Vector3d(
      circle->pos.x, circle->pos.y, circle->radius));

  for (int i = 0; i < circle_footprint.size; i++) {
    circle = &circle_footprint.circles[i];
    tf.ULFLocalPointToGlobal(&global_position2d, circle->pos);

    footprint_circle_model_global_.push_back(Eigen::Vector3d(
        global_position2d.x, global_position2d.y, circle->radius));
    footprint_circle_model_local_.push_back(Eigen::Vector3d(
        circle->pos.x, circle->pos.y, circle->radius));
  }

  return;
}

int GetParkingSpaceOccupiedRatio(const ApaParameters &parking_param,
                                 EgoInfoUnderSlot &slot_info) {
  if (std::fabs(slot_info.terminal_err.pos.y()) <
          parking_param.slot_occupied_ratio_max_lat_err &&
      std::fabs(slot_info.terminal_err.heading) <
          parking_param.slot_occupied_ratio_max_heading_err * kDeg2Rad) {
    const std::vector<double> x_tab = {
        slot_info.terminal_err.pos.x(),
        slot_info.slot.slot_length_ + parking_param.rear_overhanging};

    const std::vector<double> occupied_ratio_tab = {1.0, 0.0};
    slot_info.slot_occupied_ratio = mathlib::Interp1(
        x_tab, occupied_ratio_tab, slot_info.cur_pose.pos.x());
  } else {
    slot_info.slot_occupied_ratio = 0.0;
  }

  return 0;
}

int GetParkSpaceRelativePosition(const Eigen::Vector2d &upper_middle_pt,
                                 const Eigen::Vector2d &lower_middle_pt,
                                 const Eigen::Vector2d &ego_global_position,
                                 const Eigen::Vector3d &ego_global_pose,
                                 EgoInfoUnderSlot &slot_info,
                                 ParkingScenario::Frame &frame) {
  Eigen::Vector2d ego_to_slot_center =
      0.5 * (upper_middle_pt + lower_middle_pt) - ego_global_position;

  const auto heading_ego_vec = Eigen::Vector2d(std::cos(ego_global_pose.z()),
                                               std::sin(ego_global_pose.z()));

  const double cross_ego_to_slot_center =
      pnc::geometry_lib::GetCrossFromTwoVec2d(heading_ego_vec,
                                              ego_to_slot_center);

  pnc::geometry_lib::RotateDirection slot_to_veh = RotateDirection::NONE;

  if (cross_ego_to_slot_center > 0) {
    slot_to_veh = RotateDirection::COUNTER_CLOCKWISE;
  } else if (cross_ego_to_slot_center < 0) {
    slot_to_veh = RotateDirection::CLOCKWISE;
  }

  const double cross_ego_to_slot_heading =
      pnc::geometry_lib::GetCrossFromTwoVec2d(
          heading_ego_vec, slot_info.origin_pose_global.heading_vec);

  RotateDirection slot_origin_to_veh = RotateDirection::NONE;

  if (cross_ego_to_slot_heading > 0) {
    slot_origin_to_veh = RotateDirection::COUNTER_CLOCKWISE;
  } else if (cross_ego_to_slot_heading < 0) {
    slot_origin_to_veh = RotateDirection::CLOCKWISE;
  }

  // judge slot side via slot center and heading
  frame.current_gear = pnc::geometry_lib::SEG_GEAR_REVERSE;

  if (cross_ego_to_slot_heading > 0.0 && cross_ego_to_slot_center < 0.0) {
    frame.current_arc_steer = pnc::geometry_lib::SEG_STEER_RIGHT;
  } else if (cross_ego_to_slot_heading < 0.0 &&
             cross_ego_to_slot_center > 0.0) {
    frame.current_arc_steer = pnc::geometry_lib::SEG_STEER_LEFT;
  } else {
    frame.current_arc_steer = pnc::geometry_lib::SEG_STEER_INVALID;
    frame.current_gear = pnc::geometry_lib::SEG_GEAR_INVALID;
    // ILOG_INFO << "calculate slot side error ";
    // return false;
  }

  return 0;
}

int UpdateParkSpaceKeyPoints(
    const ApaParameters &parking_param, EgoInfoUnderSlot &slot_info,
    const std::vector<Eigen::Vector2d> &global_park_space_points,
    double cos_theta, double real_slot_length) {
  const double car_width_include_mirror =
      parking_param.car_width + 2.0 * parking_param.max_car_width;

  const double virtual_slot_width =
      car_width_include_mirror + parking_param.slot_compare_to_car_width;

  const double real_slot_width = slot_info.slot.GetWidth();

  // construct slot_t_lane, left is positive, right is negative
  const double slot_min_width = std::min(virtual_slot_width, real_slot_width);

  Eigen::Vector2d slot_left_upper_point(slot_info.slot.GetLength(),
                                        0.5 * slot_min_width);

  Eigen::Vector2d slot_right_upper_point(slot_info.slot.GetLength(),
                                         -0.5 * slot_min_width);

  return 0;
}

void DebugLineSegment(ad_common::math::LineSegment2d &line) {
  // ILOG_INFO << "start " << line.start().x() << " " << line.start().y();
  // ILOG_INFO << "end " << line.end().x() << " " << line.end().y();

  return;
}

int GenerateObstacleByJupyter(
    const ApaParameters &parking_param,
    const std::vector<Eigen::Vector2d> &global_park_space_points,
    const std::vector<double> &obs_params, const Eigen::Vector2d &vec_01,
    const Eigen::Vector2d &vec_02, const Eigen::Vector2d &unit_vec_02,
    const Eigen::Vector2d &unit_vec_01,
    EgoInfoUnderSlot &slot_info) {
  obs_global_points_.clear();

  // base point is slot right upper point
  const double right_obj_dx = obs_params[0];
  const double right_obj_dy = obs_params[1];

  // base point is slot left upper point
  const double left_obj_dx = obs_params[2];
  const double left_obj_dy = obs_params[3];

  // width is slot upper edge to a virtual barrier
  const double channel_width = obs_params[4];

  const double channel_length = parking_param.channel_length;

  const double obs_length = (channel_length - vec_01.norm()) * 0.5;

  // right obs
  right_obs_start_ =
      global_park_space_points[0] + right_obj_dx * unit_vec_02 +
      right_obj_dy * Eigen::Vector2d(-unit_vec_02.y(), unit_vec_02.x());

  const Eigen::Vector2d right_obs_horizontal_pt =
      right_obs_start_ - obs_length * unit_vec_01;

  const Eigen::Vector2d right_obs_vertical_pt =
      right_obs_start_ + unit_vec_02 * (vec_02.norm() - right_obj_dx);

  // left obs
  const Eigen::Vector2d left_obs_start =
      global_park_space_points[1] + left_obj_dx * unit_vec_02 +
      left_obj_dy * Eigen::Vector2d(unit_vec_02.y(), -unit_vec_02.x());

  const Eigen::Vector2d left_obs_horizontal_pt =
      left_obs_start + obs_length * unit_vec_01;

  const Eigen::Vector2d left_obs_vertical_pt =
      left_obs_start + unit_vec_02 * (vec_02.norm() - left_obj_dx);

  const Eigen::Vector2d channel_mid_pt =
      (global_park_space_points[0] + global_park_space_points[1]) * 0.5 +
      channel_width * Eigen::Vector2d(unit_vec_01.y(), -unit_vec_01.x());

  const Eigen::Vector2d channel_left =
      channel_mid_pt + channel_length * 0.5 * unit_vec_01;
  const Eigen::Vector2d channel_right =
      channel_mid_pt - channel_length * 0.5 * unit_vec_01;

  // right virtual wall
  const Eigen::Vector2d right_virtual_wall_start =
      global_park_space_points[0] - obs_params[5] * unit_vec_01;
  const Eigen::Vector2d right_virtual_wall_end =
      right_virtual_wall_start - unit_vec_02 * channel_width;

  const Eigen::Vector2d left_virtual_wall_start =
      global_park_space_points[0] - obs_params[6] * unit_vec_01;
  const Eigen::Vector2d left_virtual_wall_end =
      left_virtual_wall_start - unit_vec_02 * channel_width;

  // slot back wall
  const Eigen::Vector2d back_wall_left =
      left_obs_start + unit_vec_02 * (vec_02.norm() - left_obj_dx + 2.0);

  const Eigen::Vector2d back_wall_right =
      right_obs_start_ + unit_vec_02 * (vec_02.norm() - right_obj_dx + 2.0);

  //
  std::vector<pnc::geometry_lib::LineSegment> line_vec;

  pnc::geometry_lib::LineSegment line;

  line.SetPoints(right_obs_start_, right_obs_horizontal_pt);
  line_vec.emplace_back(line);

  line.SetPoints(right_obs_start_, right_obs_vertical_pt);
  line_vec.emplace_back(line);

  line.SetPoints(left_obs_start, left_obs_horizontal_pt);
  line_vec.emplace_back(line);

  line.SetPoints(left_obs_start, left_obs_vertical_pt);
  line_vec.emplace_back(line);

  line.SetPoints(channel_left, channel_right);
  line_vec.emplace_back(line);

  line.SetPoints(right_virtual_wall_start, right_virtual_wall_end);
  line_vec.emplace_back(line);

  line.SetPoints(left_virtual_wall_start, left_virtual_wall_end);
  line_vec.emplace_back(line);

  line.SetPoints(back_wall_left, back_wall_right);
  line_vec.emplace_back(line);

  // ILOG_INFO << "line_vec size " << line_vec.size();

  // sampling
  std::vector<Eigen::Vector2d> obs_sampling_points;
  obs_global_points_.clear();

  //
  hybrid_astar_obs_.Clear();

  for (const auto &line : line_vec) {
    pnc::geometry_lib::SamplePointSetInLineSeg(obs_sampling_points, line,
                                               OBS_SAMPLING_DIST);

    obs_global_points_.insert(obs_global_points_.end(),
                              obs_sampling_points.begin(),
                              obs_sampling_points.end());

    for (size_t i = 0; i < obs_sampling_points.size(); i++) {
      const Eigen::Vector2d &global = obs_sampling_points[i];
      const Eigen::Vector2d local = slot_info.g2l_tf.GetPos(global);

      hybrid_astar_obs_.virtual_obs.emplace_back(
          Position2D(local.x(), local.y()));
    }
  }

  // publish to python
  obs_line_list_.clear();

  for (const auto &line : line_vec) {
    const Eigen::Vector2d &start = line.pA;
    const Eigen::Vector2d &end = line.pB;

    Eigen::Vector4d point =
        Eigen::Vector4d(start.x(), start.y(), end.x(), end.y());

    obs_line_list_.push_back(point);
  }

  return 0;
}

// 用来调试垂直车位
// enum class ParkingVehDirection {
//   NONE,
//   TAIL_IN,
//   TAIL_OUT_TO_LEFT,
//   TAIL_OUT_TO_RIGHT,
//   TAIL_OUT_TO_MIDDLE,
//   HEAD_IN,
//   HEAD_OUT_TO_LEFT,
//   HEAD_OUT_TO_RIGHT,
//   HEAD_OUT_TO_MIDDLE,
//   MAX_NUMBER,
// };

// enum class AstarPathGenerateType {
//   NONE,
//   REEDS_SHEPP_SAMPLING,
//   ASTAR_SEARCHING,
//   GEAR_REVERSE_SEARCHING,
//   GEAR_DRIVE_SEARCHING,
//   SPIRAL_SAMPLING,
//   CUBIC_POLYNOMIAL_SAMPLING,
//   QUNTIC_POLYNOMIAL_SAMPLING,
//   // 点击车位之后，尝试搜索
//   TRY_SEARCHING,
//   MAX_NUMBER,
// };
std::vector<Eigen::Vector3d> Update(
    Eigen::Vector3d ego_global_pose,
    std::vector<Eigen::Vector2d> global_park_space_points,
    const int plan_method,
    std::vector<double> obs_params, const bool trigger_plan,
    const int parking_dir) {
  obs_global_points_.clear();
  planning::apa_planner::ParkingScenario::Frame frame;
  EgoInfoUnderSlot ego_slot_info;

  corrected_park_space_points_.clear();
  corrected_park_space_points_.resize(4);

  const Eigen::Vector2d vec_01 =
      global_park_space_points[1] - global_park_space_points[0];
  const Eigen::Vector2d unit_vec_01 = vec_01.normalized();

  const Eigen::Vector2d vec_02 =
      global_park_space_points[2] - global_park_space_points[0];
  const Eigen::Vector2d unit_vec_02 = vec_02.normalized();

  // [-90,+90 degree], cos_theta > 0
  const double cos_theta = unit_vec_01.dot(unit_vec_02);

  // update parking space for slant space
  if (cos_theta > 0.0) {
    const double h_02 = vec_01.dot(unit_vec_02);
    const Eigen::Vector2d right_upper_point =
        global_park_space_points[0] + h_02 * unit_vec_02;

    const double h_13 = vec_02.norm() - h_02;
    const Eigen::Vector2d left_lower_point =
        global_park_space_points[1] + h_13 * unit_vec_02;

    corrected_park_space_points_[0] = right_upper_point;
    corrected_park_space_points_[1] = global_park_space_points[1];
    corrected_park_space_points_[2] = global_park_space_points[2];
    corrected_park_space_points_[3] = left_lower_point;

  } else {
    const Eigen::Vector2d vec_10 = -vec_01;
    const Eigen::Vector2d vec_13 =
        global_park_space_points[3] - global_park_space_points[1];

    const Eigen::Vector2d unit_vec_13 = vec_13.normalized();

    const double h_13 = vec_10.dot(unit_vec_13);
    const Eigen::Vector2d pt_1_dot =
        global_park_space_points[1] + h_13 * unit_vec_13;

    const double h_02 = vec_13.norm() - h_13;
    const Eigen::Vector2d pt_2_dot =
        global_park_space_points[0] + h_02 * unit_vec_13;

    corrected_park_space_points_[0] = global_park_space_points[0];
    corrected_park_space_points_[1] = pt_1_dot;
    corrected_park_space_points_[2] = pt_2_dot;
    corrected_park_space_points_[3] = global_park_space_points[3];
  }

  // update space
  const auto upper_middle_pt =
      0.5 * (corrected_park_space_points_[0] + corrected_park_space_points_[1]);
  const auto lower_middle_pt =
      0.5 * (corrected_park_space_points_[2] + corrected_park_space_points_[3]);

  const double real_slot_length = (upper_middle_pt - lower_middle_pt).norm();

  const auto vector_to_left =
      (corrected_park_space_points_[1] - corrected_park_space_points_[0])
          .normalized();

  const auto vector_to_up =
      Eigen::Vector2d(vector_to_left.y(), -vector_to_left.x());

  corrected_park_space_points_[2] =
      corrected_park_space_points_[0] - real_slot_length * vector_to_up;
  corrected_park_space_points_[3] =
      corrected_park_space_points_[1] - real_slot_length * vector_to_up;

  ego_slot_info.slot.origin_corner_coord_global_.pt_0 =
      corrected_park_space_points_[0];
  ego_slot_info.slot.origin_corner_coord_global_.pt_1 =
      corrected_park_space_points_[1];
  ego_slot_info.slot.origin_corner_coord_global_.pt_2 =
      corrected_park_space_points_[2];
  ego_slot_info.slot.origin_corner_coord_global_.pt_3 =
      corrected_park_space_points_[3];

  ego_slot_info.origin_pose_global.pos =
      upper_middle_pt - real_slot_length * vector_to_up;

  ego_slot_info.origin_pose_global.heading =
      std::atan2(vector_to_up.y(), vector_to_up.x());

  ego_slot_info.origin_pose_global.heading_vec = vector_to_up;
  ego_slot_info.slot.slot_length_ = real_slot_length;

  ego_slot_info.slot.slot_width_ =
      (corrected_park_space_points_[0] - corrected_park_space_points_[1])
          .norm();

  // base coordinate
  ego_slot_info.g2l_tf.Init(ego_slot_info.origin_pose_global.pos,
                            ego_slot_info.origin_pose_global.heading);

  ego_slot_info.l2g_tf.Init(ego_slot_info.origin_pose_global.pos,
                            ego_slot_info.origin_pose_global.heading);

  // update ego pose
  ILOG_INFO << "ego x " << ego_global_pose.x();

  Eigen::Vector2d ego_global_position(ego_global_pose.x(), ego_global_pose.y());
  double heading_ego = ego_global_pose.z();

  ego_slot_info.cur_pose.pos = ego_slot_info.g2l_tf.GetPos(ego_global_position);
  ego_slot_info.cur_pose.heading = ego_slot_info.g2l_tf.GetHeading(heading_ego);

  ILOG_INFO << "ego_pos_slot = " << ego_slot_info.cur_pose.pos.x() << ","
            << ego_slot_info.cur_pose.pos.y()
            << ",heading = " << ego_slot_info.cur_pose.heading * 57.3;

  ego_slot_info.cur_pose.heading_vec =
      Eigen::Vector2d(std::cos(ego_slot_info.cur_pose.heading),
                      std::sin(ego_slot_info.cur_pose.heading));

  // cal target pos
  const ApaParameters &parking_param = apa_param.GetParam();

  if (parking_dir == 1) {
    ego_slot_info.target_pose.heading = parking_param.terminal_target_heading;
    ego_slot_info.target_pose.pos = Eigen::Vector2d(
        parking_param.terminal_target_x, parking_param.terminal_target_y);
  } else if (parking_dir == 5) {
    ego_slot_info.target_pose.heading = M_PI;
    ego_slot_info.target_pose.pos = Eigen::Vector2d(4.0, 0.0);
  }

  // get global
  const auto target_ego_pos_global =
      ego_slot_info.l2g_tf.GetPos(ego_slot_info.target_pose.pos);
  const auto target_ego_heading_global =
      ego_slot_info.l2g_tf.GetHeading(ego_slot_info.target_pose.heading);

  global_target_pose_ =
      Eigen::Vector3d(target_ego_pos_global.x(), target_ego_pos_global.y(),
                      target_ego_heading_global);

  // ILOG_INFO << "target_ego_pos_slot = " <<
  // ego_slot_info.target_ego_pos_slot[0]
  //           << ", " << ego_slot_info.target_ego_pos_slot[1]
  //           << "  target_ego_heading_slot = "
  //           << ego_slot_info.target_ego_heading_slot * 57.3;

  // cal terminal error
  ego_slot_info.terminal_err.Set(
      ego_slot_info.cur_pose.pos - ego_slot_info.target_pose.pos,
      ego_slot_info.cur_pose.heading - ego_slot_info.target_pose.heading);

  // cal slot occupied ratio
  GetParkingSpaceOccupiedRatio(parking_param, ego_slot_info);

  // todo
  GetParkSpaceRelativePosition(upper_middle_pt, lower_middle_pt,
                               ego_global_position, ego_global_pose,
                               ego_slot_info, frame);

  UpdateParkSpaceKeyPoints(parking_param, ego_slot_info,
                           global_park_space_points, cos_theta,
                           real_slot_length);

  obs_global_points_.clear();

  // obs_params = [right_obj_dx, right_obj_dy, left_obj_dx, left_obj_dy,
  // channel_width]
  // ILOG_INFO << "obs params number " << obs_params.size();

  if (obs_params.size() != 7) {
    global_path_.clear();

    ILOG_INFO << " invalid obstacle input";

    return global_path_;
  }

  GenerateObstacleByJupyter(parking_param, global_park_space_points, obs_params,
                            vec_01, vec_02, unit_vec_02, unit_vec_01,
                            ego_slot_info);

  // start
  Eigen::Vector3d start;
  start << ego_slot_info.cur_pose.pos[0], ego_slot_info.cur_pose.pos[1],
      ego_slot_info.cur_pose.heading;

  // end

  Eigen::Vector3d end;
  end[0] = ego_slot_info.target_pose.pos[0];
  if (parking_dir == 1) {
    end[0] += parking_param.astar_config.vertical_tail_in_end_straight_dist;
  } else if (parking_dir == 5) {
    end[0] += parking_param.astar_config.vertical_head_in_end_straight_dist;
  }

  end[1] = ego_slot_info.target_pose.pos[1];
  end[2] = ego_slot_info.target_pose.heading;

  // obs

  if (hybrid_astar_interface_ != nullptr && trigger_plan) {
    ILOG_INFO << "trigger astar plan";

    AstarRequest request;
    request.slot_width = ego_slot_info.slot.GetWidth();
    request.slot_length = ego_slot_info.slot.GetLength();
    request.first_action_request.has_request = false;

    switch (plan_method) {
      case 2:
        request.path_generate_method =
            planning::AstarPathGenerateType::ASTAR_SEARCHING;
        break;
      case 3:
        request.path_generate_method =
            planning::AstarPathGenerateType::GEAR_REVERSE_SEARCHING;
        break;
      case 4:
        request.path_generate_method =
            planning::AstarPathGenerateType::GEAR_DRIVE_SEARCHING;
        break;
      case 8:
        request.path_generate_method =
            planning::AstarPathGenerateType::TRY_SEARCHING;
        break;
      default:
        request.path_generate_method =
            planning::AstarPathGenerateType::CUBIC_POLYNOMIAL_SAMPLING;
        break;
    }

    request.start_ = Pose2D(start[0], start[1], start[2]);
    request.start_.theta =
        ad_common::math::NormalizeAngle(request.start_.theta);

    request.goal_ = Pose2D(end[0], end[1], end[2]);
    request.goal_.theta = ad_common::math::NormalizeAngle(request.goal_.theta);

    request.real_goal = Pose2D(ego_slot_info.target_pose.pos[0],
                               ego_slot_info.target_pose.pos[1],
                               ego_slot_info.target_pose.heading);
    request.base_pose_ = Pose2D(0, 0, 0);

    ILOG_INFO <<"planning goal";
    request.goal_.DebugString();

    ILOG_INFO <<"real goal";
    request.real_goal.DebugString();

    request.space_type = ParkSpaceType::VERTICAL;
    request.direction_request = ParkingVehDirection::TAIL_IN;
    if (parking_dir == 5) {
      request.direction_request = ParkingVehDirection::HEAD_IN;
    }
    request.rs_request = RSPathRequestType::NONE;
    request.history_gear = AstarPathGear::NONE;
    request.swap_start_goal = false;

    hybrid_astar_interface_->GeneratePath(start, end, hybrid_astar_obs_,
                                          request);
    hybrid_astar_interface_->ExtendPathToRealTargetPose(request.real_goal);

    // hybrid_astar_interface_->UpdateEDTByObs(hybrid_astar_obs_);

    ILOG_INFO << "hybrid_astar_interface_ finish";
    GetPathFromHybridAstar(
        ego_slot_info,
        parking_param.astar_config.vertical_tail_in_end_straight_dist,
        ego_global_pose);

    // just test rs library

    bool is_connected_to_goal;

    Pose2D start_pose = {start[0], start[1], start[2]};
    Pose2D end_pose = {end[0], end[1], end[2]};

    RSPathInterface rs_interface;
    RSPath rs_path;
    RSPathRequestType rs_request = RSPathRequestType::NONE;
    rs_interface.GeneSCSPath(&rs_path, &is_connected_to_goal, &start_pose,
                             &end_pose, parking_param.min_turn_radius,
                             rs_request);

    rs_path_alone_.clear();
    Eigen::Vector2d local_position;
    Eigen::Vector2d global_position;
    for (int i = 0; i < rs_path.size; i++) {
      const RSPathSegment *segment = &rs_path.paths[i];

      for (int j = 0; j < segment->size; j++) {
        local_position[0] = segment->points[j].x;
        local_position[1] = segment->points[j].y;
        global_position = ego_slot_info.l2g_tf.GetPos(local_position);

        rs_path_alone_.emplace_back(
            Eigen::Vector3d(global_position[0], global_position[1], 0.0));
      }
    }

    planning::HybridAStar astar;
    // astar.DebugRSPath(&rs_path);

  } else {
    // ResetHybridAstarPath();

    ILOG_INFO << "get_history_astar_path";
  }

  // GetTrajPoseBySDist(s);

  UpdateFootprintCircle(ego_global_pose);

  return global_path_;
}

std::vector<Eigen::Vector2d> GetVirtualWallObstacles() {
  return obs_global_points_;
}

Eigen::Vector3d GetTargetPose() { return global_target_pose_; }

Eigen::Vector3d GetCircleTangentPose() { return safe_circle_tang_pose_; }

Eigen::Vector3d GetTrajPoseByDist() { return car_pose_by_s_; }

Eigen::Vector2d GetPtInsidePose() { return pt_inside_pose_; }

const std::vector<Eigen::Vector2d> &GetRectangleSoltPos() {
  return corrected_park_space_points_;
}

const std::vector<Eigen::Vector4d> &GetObsLineList() { return obs_line_list_; }

const std::vector<Eigen::Vector3d> &GetReedsShapePath() { return rs_path_; }

const std::vector<Eigen::Vector3d> &GetPolynomialPath() {
  return polynomial_path_;
}

const std::vector<std::vector<Eigen::Vector2d>> &GetAstarAllNodes() {
  return real_time_node_list_;
}

// this is a search path by queue.
const std::vector<Eigen::Vector2d> &GetSearchPathPoint() {
  return search_sequence_path_;
}

const std::vector<Eigen::Vector3d> &GetAllSearchNode() {
  return all_searched_node_;
}

const std::vector<std::vector<Eigen::Vector2d>> &GetRSHeuristicPath() {
  ILOG_INFO << "rs path size=" << rs_h_path_.size();
  return rs_h_path_;
}

const std::vector<Eigen::Vector3d> &GetRSLibPath() { return rs_path_alone_; }

const std::vector<Eigen::Vector3d> &GetFootPrintModelGlobal() {
  return footprint_circle_model_global_;
}

const std::vector<Eigen::Vector3d> &GetFootPrintModelLocal() {
  return footprint_circle_model_local_;
}

const Eigen::Vector3d GetAstarEndPose() { return astar_end_pose_; }
const Eigen::Vector3d GetCurrentGearPathEnd() { return astar_current_path_end_; }

PYBIND11_MODULE(hybrid_astar_py, m) {
  m.doc() = "m";

  m.def("Init", &Init)
      .def("Update", &Update)
      .def("GetTargetPose", &GetTargetPose)
      .def("GetCircleTangentPose", &GetCircleTangentPose)
      .def("GetPtInsidePose", &GetPtInsidePose)
      .def("GetRectangleSoltPos", &GetRectangleSoltPos)
      .def("GetReedsShapePath", &GetReedsShapePath)
      .def("GetAstarAllNodes", &GetAstarAllNodes)
      .def("GetSearchPathPoint", &GetSearchPathPoint)
      .def("GetRSHeuristicPath", &GetRSHeuristicPath)
      .def("GetObsLineList", &GetObsLineList)
      .def("StopPybind", &StopPybind)
      .def("GetRSLibPath", &GetRSLibPath)
      .def("GetFootPrintModelGlobal", &GetFootPrintModelGlobal)
      .def("GetAllSearchNode", &GetAllSearchNode)
      .def("GetPolynomialPath", &GetPolynomialPath)
      .def("GetAstarEndPose", &GetAstarEndPose)
      .def("GetCurrentGearPathEnd", &GetCurrentGearPathEnd)
      .def("GetFootPrintModelLocal", &GetFootPrintModelLocal)
      .def("GetVirtualWallObstacles", &GetVirtualWallObstacles);
  ;
}