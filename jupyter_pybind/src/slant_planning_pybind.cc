
#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <cstdint>
#include <iostream>
#include <vector>

#include "Eigen/Core"
#include "apa_param_setting.h"
#include "apa_plan_base.h"
#include "apa_plan_interface.h"
#include "collision_detection.h"
#include "math_lib.h"
#include "perpendicular_park_in_planner.h"
#include "perpendicular_path_planner.h"

namespace py = pybind11;
using namespace planning::apa_planner;

static planning::apa_planner::PerpendicularPathPlanner *pBase = nullptr;
static planning::apa_planner::ApaPlanInterface *pApaPlanInterface = nullptr;

int Init() {
  pBase = new PerpendicularPathPlanner();
  pBase->Reset();

  pApaPlanInterface = new planning::apa_planner::ApaPlanInterface();

  pApaPlanInterface->Init();

  return 0;
}

template <class T>
inline T BytesToProto(py::bytes &bytes) {
  T proto_obj;
  py::buffer buf(bytes);
  py::buffer_info input_info = buf.request();
  char *input_ptr = static_cast<char *>(input_info.ptr);
  std::string input_s(input_ptr, input_info.size);

  T input;
  input.ParseFromString(input_s);
  return input;
}

static PerpendicularPathPlanner::DebugInfo debuginfo;
static std::vector<double> res;
std::vector<Eigen::Vector3d> current_path_point_global_vec_;
Eigen::Vector3d global_target_pose_;
Eigen::Vector3d safe_circle_tang_pose_;
Eigen::Vector2d pt_inside_pose_;
std::vector<Eigen::Vector2d> pt_;

std::vector<Eigen::Vector2d> obs_pts_;

std::vector<Eigen::Vector3d> Update(Eigen::Vector3d ego_pose,
                                    std::vector<Eigen::Vector2d> raw_pt,
                                    double ds, bool is_complete_path,
                                    double inside_dx,
                                    std::vector<double> obs_params) {
  obs_pts_.clear();
  planning::apa_planner::ApaPlannerBase::Frame frame;
  auto &ego_slot_info = frame.ego_slot_info;
  pt_.clear();
  pt_.resize(4);
  const Eigen::Vector2d vec_01 = raw_pt[1] - raw_pt[0];
  const Eigen::Vector2d unit_vec_01 = vec_01.normalized();
  const Eigen::Vector2d vec_02 = raw_pt[2] - raw_pt[0];
  const Eigen::Vector2d unit_vec_02 = vec_02.normalized();
  const double cos_theta = unit_vec_01.dot(unit_vec_02);

  if (cos_theta > 0.0) {
    const double h_02 = vec_01.dot(unit_vec_02);
    const Eigen::Vector2d pt_0_dot = raw_pt[0] + h_02 * unit_vec_02;
    const double h_13 = vec_02.norm() - h_02;
    const Eigen::Vector2d pt_3_dot = raw_pt[1] + h_13 * unit_vec_02;
    pt_[0] = pt_0_dot;
    pt_[1] = raw_pt[1];
    pt_[2] = raw_pt[2];
    pt_[3] = pt_3_dot;
  } else {
    const Eigen::Vector2d vec_10 = -vec_01;
    const Eigen::Vector2d vec_13 = raw_pt[3] - raw_pt[1];
    const Eigen::Vector2d unit_vec_13 = vec_13.normalized();
    const double h_13 = vec_10.dot(unit_vec_13);
    const Eigen::Vector2d pt_1_dot = raw_pt[1] + h_13 * unit_vec_13;
    const double h_02 = vec_13.norm() - h_13;
    const Eigen::Vector2d pt_2_dot = raw_pt[0] + h_02 * unit_vec_13;
    pt_[0] = raw_pt[0];
    pt_[1] = pt_1_dot;
    pt_[2] = pt_2_dot;
    pt_[3] = raw_pt[3];
  }

  const auto pM01 = 0.5 * (pt_[0] + pt_[1]);
  const auto pM23 = 0.5 * (pt_[2] + pt_[3]);
  const double real_slot_length = (pM01 - pM23).norm();
  const auto t = (pt_[1] - pt_[0]).normalized();
  // const auto n = (pM01 - pM23).normalized();
  const auto n = Eigen::Vector2d(t.y(), -t.x());
  pt_[2] = pt_[0] - real_slot_length * n;
  pt_[3] = pt_[1] - real_slot_length * n;

  ego_slot_info.slot_corner = pt_;

  const double virtual_slot_length = real_slot_length;

  ego_slot_info.slot_origin_pos = pM01 - virtual_slot_length * n;
  ego_slot_info.slot_origin_heading = std::atan2(n.y(), n.x());
  ego_slot_info.slot_origin_heading_vec = n;
  ego_slot_info.slot_length = virtual_slot_length;
  ego_slot_info.slot_width = (pt_[0] - pt_[1]).norm();

  // std::cout << "slot_origin_pos = " <<
  // ego_slot_info.slot_origin_pos.transpose()
  //           << std::endl;

  ego_slot_info.g2l_tf.Init(ego_slot_info.slot_origin_pos,
                            ego_slot_info.slot_origin_heading);

  ego_slot_info.l2g_tf.Init(ego_slot_info.slot_origin_pos,
                            ego_slot_info.slot_origin_heading);

  Eigen::Vector2d pos_ego(ego_pose.x(), ego_pose.y());
  double heading_ego = ego_pose.z();

  ego_slot_info.ego_pos_slot = ego_slot_info.g2l_tf.GetPos(pos_ego);

  ego_slot_info.ego_heading_slot = ego_slot_info.g2l_tf.GetHeading(heading_ego);

  std::cout << "  ego_pos_slot = " << ego_slot_info.ego_pos_slot.transpose()
            << "  ego_heading_slot = " << ego_slot_info.ego_heading_slot * 57.3
            << std::endl;

  ego_slot_info.ego_heading_slot_vec
      << std::cos(ego_slot_info.ego_heading_slot),
      std::sin(ego_slot_info.ego_heading_slot);

  // cal target pos
  ego_slot_info.target_ego_pos_slot << apa_param.GetParam().terminal_target_x,
      apa_param.GetParam().terminal_target_y;
  ego_slot_info.target_ego_heading_slot =
      apa_param.GetParam().terminal_target_heading;

  const auto &target_ego_pos_global =
      ego_slot_info.l2g_tf.GetPos(ego_slot_info.target_ego_pos_slot);
  const auto &target_ego_heading_global =
      ego_slot_info.l2g_tf.GetHeading(ego_slot_info.target_ego_heading_slot);

  global_target_pose_ << target_ego_pos_global.x(), target_ego_pos_global.y(),
      target_ego_heading_global;

  std::cout << "target_ego_pos_slot = "
            << ego_slot_info.target_ego_pos_slot.transpose()
            << "  target_ego_heading_slot = "
            << ego_slot_info.target_ego_heading_slot * 57.3 << std::endl;

  // cal terminal error
  ego_slot_info.terminal_err.Set(
      ego_slot_info.ego_pos_slot - ego_slot_info.target_ego_pos_slot,
      ego_slot_info.ego_heading_slot - ego_slot_info.target_ego_heading_slot);

  // cal slot occupied ratio
  if (std::fabs(ego_slot_info.terminal_err.pos.y()) <
          apa_param.GetParam().slot_occupied_ratio_max_lat_err &&
      std::fabs(ego_slot_info.ego_heading_slot) <
          apa_param.GetParam().slot_occupied_ratio_max_heading_err / 57.3) {
    ego_slot_info.slot_occupied_ratio = pnc::mathlib::Clamp(
        1.0 - (ego_slot_info.terminal_err.pos.x() / ego_slot_info.slot_length),
        0.0, 1.0);
  } else {
    ego_slot_info.slot_occupied_ratio = 0.0;
  }

  Eigen::Vector2d ego_to_slot_center_vec = 0.5 * (pM01 + pM23) - pos_ego;

  const auto heading_ego_vec =
      Eigen::Vector2d(std::cos(ego_pose.z()), std::sin(ego_pose.z()));
  const double cross_ego_to_slot_center =
      pnc::geometry_lib::GetCrossFromTwoVec2d(heading_ego_vec,
                                              ego_to_slot_center_vec);

  const double cross_ego_to_slot_heading =
      pnc::geometry_lib::GetCrossFromTwoVec2d(
          heading_ego_vec, ego_slot_info.slot_origin_heading_vec);

  planning::apa_planner::PerpendicularPathPlanner::Tlane slot_t_lane;
  // judge slot side via slot center and heading
  frame.current_gear = pnc::geometry_lib::SEG_GEAR_REVERSE;
  if (cross_ego_to_slot_heading > 0.0 && cross_ego_to_slot_center < 0.0) {
    slot_t_lane.slot_side = pnc::geometry_lib::SLOT_SIDE_RIGHT;
    frame.current_arc_steer = pnc::geometry_lib::SEG_STEER_RIGHT;
  } else if (cross_ego_to_slot_heading < 0.0 &&
             cross_ego_to_slot_center > 0.0) {
    slot_t_lane.slot_side = pnc::geometry_lib::SLOT_SIDE_LEFT;
    frame.current_arc_steer = pnc::geometry_lib::SEG_STEER_LEFT;
  } else {
    slot_t_lane.slot_side = pnc::geometry_lib::SLOT_SIDE_INVALID;
    frame.current_arc_steer = pnc::geometry_lib::SEG_STEER_INVALID;
    frame.current_gear = pnc::geometry_lib::SEG_GEAR_INVALID;
    std::cout << "calculate slot side error " << std::endl;
    // return false;
  }

  bool is_same_direction1 = false;
  if (slot_t_lane.slot_side == pnc::geometry_lib::SLOT_SIDE_RIGHT) {
    if (cos_theta > 0.01) {
      is_same_direction1 = true;
    }
  } else {
    if (cos_theta < -0.01) {
      is_same_direction1 = true;
    }
  }

  if (!is_same_direction1 && !(std::fabs(cos_theta) < 0.005)) {
    current_path_point_global_vec_.clear();
    return current_path_point_global_vec_;
  }

  const double car_width_include_mirror =
      apa_param.GetParam().car_width + 2.0 * apa_param.GetParam().mirror_width;
  const double car_y_right_include_mirror = -car_width_include_mirror / 2.0;
  const double car_y_left_include_mirror = car_width_include_mirror / 2.0;

  const double virtual_slot_width =
      car_width_include_mirror + apa_param.GetParam().slot_compare_to_car_width;

  const double real_slot_width = ego_slot_info.slot_width;

  // construct slot_t_lane, left is positive, right is negative
  const double slot_width = std::min(virtual_slot_width, real_slot_width);

  Eigen::Vector2d corner_left_slot(ego_slot_info.slot_length, 0.5 * slot_width);

  Eigen::Vector2d corner_right_slot(ego_slot_info.slot_length,
                                    -0.5 * slot_width);

  const auto &slot_side = slot_t_lane.slot_side;
  if (slot_side == pnc::geometry_lib::SLOT_SIDE_RIGHT) {
    // inside is right, outside is left
    slot_t_lane.corner_outside_slot = corner_left_slot;
    slot_t_lane.corner_inside_slot = corner_right_slot;
    slot_t_lane.pt_outside = corner_left_slot;
    slot_t_lane.pt_inside = corner_right_slot;
  } else if (slot_side == pnc::geometry_lib::SLOT_SIDE_LEFT) {
    // outside is right, inside is left
    slot_t_lane.corner_outside_slot = corner_right_slot;
    slot_t_lane.corner_inside_slot = corner_left_slot;
    slot_t_lane.pt_outside = corner_right_slot;
    slot_t_lane.pt_inside = corner_left_slot;
  }

  slot_t_lane.pt_terminal_pos << ego_slot_info.target_ego_pos_slot.x(),
      ego_slot_info.target_ego_pos_slot.y();

  slot_t_lane.pt_terminal_heading = ego_slot_info.target_ego_heading_slot;

  slot_t_lane.pt_lower_boundry_pos = slot_t_lane.pt_terminal_pos;
  slot_t_lane.pt_lower_boundry_pos.x() =
      slot_t_lane.pt_lower_boundry_pos.x() -
      apa_param.GetParam().rear_overhanging -
      apa_param.GetParam().col_obs_safe_dist - 0.05;

  const Eigen::Vector2d pt_0 = ego_slot_info.g2l_tf.GetPos(raw_pt[0]);
  const Eigen::Vector2d pt_1 = ego_slot_info.g2l_tf.GetPos(raw_pt[1]);

  ego_slot_info.pt_0 = pt_0;
  ego_slot_info.pt_1 = pt_1;
  const Eigen::Vector2d pt_01_vec = pt_1 - pt_0;

  if (std::fabs(cos_theta) < 0.01) {
    ego_slot_info.sin_angle = 1.0;
    ego_slot_info.origin_pt_0_heading = 0.0;
  } else {
    double angle = std::fabs(pnc::geometry_lib::GetAngleFromTwoVec(
                       Eigen::Vector2d(virtual_slot_length, 0.0), pt_01_vec)) *
                   57.3;

    if (angle > 90.0) {
      angle = 180.0 - angle;
    }

    angle = pnc::mathlib::DoubleConstrain(angle, 10.0, 80.0);
    ego_slot_info.sin_angle = std::sin(angle / 57.3);
    ego_slot_info.origin_pt_0_heading = 90.0 - angle;
  }

  std::cout << "origin_pt_0_heading = " << ego_slot_info.origin_pt_0_heading
            << std::endl;

  obs_pts_.clear();

  if (obs_params.size() != 6) {
    current_path_point_global_vec_.clear();
    return current_path_point_global_vec_;
  }
  const double right_obj_dx = obs_params[0];
  const double right_obj_dy = obs_params[1];
  const double left_obj_dx = obs_params[2];
  const double left_obj_dy = obs_params[3];
  const double channel_width = obs_params[4];
  const double slot_inside_obs = obs_params[5];

  const double channel_length = apa_param.GetParam().channel_length;

  const double obs_length = (channel_length - vec_01.norm()) / 2.0;

  const Eigen::Vector2d obj_pt_0 =
      raw_pt[0] + right_obj_dx * unit_vec_02 +
      right_obj_dy * Eigen::Vector2d(-unit_vec_02.y(), unit_vec_02.x());

  const Eigen::Vector2d right_obj_pt_horizontal =
      obj_pt_0 - obs_length * unit_vec_01;

  const Eigen::Vector2d right_obj_pt_vertical =
      obj_pt_0 + unit_vec_02 * (vec_02.norm() - right_obj_dx + 0.15);

  const Eigen::Vector2d obj_pt_1 =
      raw_pt[1] + left_obj_dx * unit_vec_02 +
      left_obj_dy * Eigen::Vector2d(unit_vec_02.y(), -unit_vec_02.x());

  const Eigen::Vector2d left_obj_pt_horizontal =
      obj_pt_1 + obs_length * unit_vec_01;

  const Eigen::Vector2d left_obj_pt_vertical =
      obj_pt_1 + unit_vec_02 * (vec_02.norm() - left_obj_dx + 0.15);

  const Eigen::Vector2d channel_mid =
      (raw_pt[0] + raw_pt[1]) / 2.0 +
      channel_width * Eigen::Vector2d(unit_vec_01.y(), -unit_vec_01.x());
  const Eigen::Vector2d channel_left =
      channel_mid + channel_length / 2.0 * unit_vec_01;
  const Eigen::Vector2d channel_right =
      channel_mid - channel_length / 2.0 * unit_vec_01;

  Eigen::Vector2d A, B, C, D, E, F, G, H;
  if (slot_side == pnc::geometry_lib::SLOT_SIDE_RIGHT) {
    A = left_obj_pt_horizontal;
    B = obj_pt_1;
    C = left_obj_pt_vertical;
    D = right_obj_pt_vertical;
    E = obj_pt_0;
    F = right_obj_pt_horizontal;
    F = obj_pt_0 - slot_inside_obs * unit_vec_01;
    G = channel_right;
    G = channel_mid - ((raw_pt[0] - raw_pt[1]).norm() / 2.0 + slot_inside_obs +
                       right_obj_dy) *
                          unit_vec_01;
    H = channel_left;
  } else {
    A = right_obj_pt_horizontal;
    B = obj_pt_0;
    C = right_obj_pt_vertical;
    D = left_obj_pt_vertical;
    E = obj_pt_1;
    F = left_obj_pt_horizontal;
    F = obj_pt_1 + slot_inside_obs * unit_vec_01;
    G = channel_left;
    G = channel_mid +
        ((raw_pt[0] - raw_pt[1]).norm() / 2.0 + slot_inside_obs + left_obj_dy) *
            unit_vec_01;
    H = channel_right;
  }

  std::vector<pnc::geometry_lib::LineSegment> line_vec;
  pnc::geometry_lib::LineSegment line;
  line.SetPoints(A, B);
  line_vec.emplace_back(line);
  line.SetPoints(B, C);
  line_vec.emplace_back(line);
  line.SetPoints(C, D);
  line_vec.emplace_back(line);
  line.SetPoints(D, E);
  line_vec.emplace_back(line);
  line.SetPoints(E, F);
  line_vec.emplace_back(line);
  line.SetPoints(F, G);
  line_vec.emplace_back(line);
  line.SetPoints(G, H);
  line_vec.emplace_back(line);

  std::vector<Eigen::Vector2d> pt_vec;
  obs_pts_.clear();

  for (const auto &line : line_vec) {
    pnc::geometry_lib::SamplePointSetInLineSeg(pt_vec, line, 0.25);
    obs_pts_.insert(obs_pts_.end(), pt_vec.begin(), pt_vec.end());
  }

  std::shared_ptr<planning::CollisionDetector> collision_detector_ptr = nullptr;
  collision_detector_ptr = std::make_shared<planning::CollisionDetector>();
  collision_detector_ptr->ClearObstacles();

  std::vector<Eigen::Vector2d> obs_local_pts;
  for (const auto &obs_pt : obs_pts_) {
    obs_local_pts.emplace_back(ego_slot_info.g2l_tf.GetPos(obs_pt));
  }

  collision_detector_ptr->SetObstacles(obs_local_pts,
                                       planning::CollisionDetector::TLANE_OBS);

  planning::apa_planner::PerpendicularPathPlanner::Input input;
  input.pt_0 = ego_slot_info.pt_0;
  input.pt_1 = ego_slot_info.pt_1;
  input.sin_angle = ego_slot_info.sin_angle;
  input.origin_pt_0_heading = ego_slot_info.origin_pt_0_heading;
  Eigen::Vector2d pt_inside = obj_pt_0;
  if (slot_side == pnc::geometry_lib::SLOT_SIDE_LEFT) {
    pt_inside = obj_pt_1;
  }
  slot_t_lane.pt_inside.x() = ego_slot_info.g2l_tf.GetPos(pt_inside).x();
  input.slot_occupied_ratio = ego_slot_info.slot_occupied_ratio;
  input.tlane = slot_t_lane;
  input.is_complete_path = is_complete_path;
  input.sample_ds = 0.2;
  input.ref_arc_steer = frame.current_arc_steer;
  input.ref_gear = frame.current_gear;
  input.is_replan_first = frame.is_replan_first;
  input.is_replan_second = frame.is_replan_second;
  input.is_replan_dynamic = frame.is_replan_dynamic;
  input.ego_pose.Set(ego_slot_info.ego_pos_slot,
                     ego_slot_info.ego_heading_slot);

  pt_inside_pose_ = ego_slot_info.l2g_tf.GetPos(slot_t_lane.pt_inside);

  pBase->SetInput(input);

  pBase->SetColPtr(collision_detector_ptr);

  pBase->Preprocess();

  current_path_point_global_vec_.clear();
  bool success = false;

  while (true) {
    if (pBase->CheckReachTargetPosePb()) {
      success = true;
      break;
    }

    if (pBase->PreparePlanPb()) {
      std::cout << "prepare plan success\n";
      // success = true;
      // break;
    } else {
      std::cout << "prepare is complete fail\n\n";
      success = false;
      break;
    }

    if (pBase->CheckReachTargetPosePb()) {
      success = true;
      break;
    }

    if (pBase->PreparePlanSecondPb()) {
      std::cout << "prepare second plan success\n";
    } else {
      std::cout << "prepare second fail\n";
    }

    if (pBase->CheckReachTargetPosePb()) {
      success = true;
      break;
    }
    if (pBase->MultiPlanPb()) {
      std::cout << "multi plan success\n";
    }

    if (pBase->CheckReachTargetPosePb()) {
      success = true;
      break;
    }
    if (pBase->AdjustPlanPb()) {
      std::cout << "adjust plan success\n";
    }
    if (pBase->CheckReachTargetPosePb()) {
      success = true;
      break;
    }
    success = false;
    break;
  }

  pt_inside_pose_.setZero();
  const Eigen::Vector2d pt_inside_local = pBase->GetCalcParams().pt_inside;
  pt_inside_pose_ = ego_slot_info.l2g_tf.GetPos(pt_inside_local);

  safe_circle_tang_pose_.setZero();
  const auto &tange_pos_global = ego_slot_info.l2g_tf.GetPos(
      pBase->GetCalcParams().safe_circle_tang_pt.pos);
  const auto &tange_heading_global = ego_slot_info.l2g_tf.GetHeading(
      pBase->GetCalcParams().safe_circle_tang_pt.heading);
  safe_circle_tang_pose_ << tange_pos_global.x(), tange_pos_global.y(),
      tange_heading_global;

  std::cout << "this plan success = " << success << "\n\n\n";
  if (success) {
    pBase->SampleCurrentPathSeg();

    const auto &planner_output = pBase->GetOutput();

    current_path_point_global_vec_.reserve(
        planner_output.path_point_vec.size());

    pnc::geometry_lib::PathPoint global_point;
    for (const auto &path_point : planner_output.path_point_vec) {
      global_point.Set(ego_slot_info.l2g_tf.GetPos(path_point.pos),
                       ego_slot_info.l2g_tf.GetHeading(path_point.heading));

      current_path_point_global_vec_.emplace_back(Eigen::Vector3d(
          global_point.pos.x(), global_point.pos.y(), global_point.heading));
    }
  }

  return current_path_point_global_vec_;
}

std::vector<Eigen::Vector2d> GetObstacles() { return obs_pts_; }

Eigen::Vector3d GetTargetPose() { return global_target_pose_; }

Eigen::Vector3d GetCircleTangentPose() { return safe_circle_tang_pose_; }

Eigen::Vector2d GetPtInsidePose() { return pt_inside_pose_; }

const std::vector<Eigen::Vector2d> &GetRectangleSoltPos() { return pt_; }

PYBIND11_MODULE(slant_planning_py, m) {
  m.doc() = "m";

  m.def("Init", &Init)
      .def("Update", &Update)
      .def("GetTargetPose", &GetTargetPose)
      .def("GetCircleTangentPose", &GetCircleTangentPose)
      .def("GetPtInsidePose", &GetPtInsidePose)
      .def("GetRectangleSoltPos", &GetRectangleSoltPos)
      .def("GetObstacles", &GetObstacles);
  ;
}