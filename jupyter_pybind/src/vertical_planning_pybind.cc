#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <cstddef>
#include <cstdint>
#include <iostream>
#include <vector>

#include "Eigen/Core"
#include "apa_plan_base.h"
#include "apa_plan_interface.h"
#include "collision_detection.h"
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

std::vector<Eigen::Vector3d> Update(Eigen::Vector3d ego_pose,
                                    std::vector<Eigen::Vector2d> pt, double ds,
                                    bool is_complete_path, double inside_dx) {
  planning::apa_planner::ApaPlannerBase::Frame frame;
  auto &ego_slot_info = frame.ego_slot_info;

  const auto pM01 = 0.5 * (pt[0] + pt[1]);
  const auto pM23 = 0.5 * (pt[2] + pt[3]);
  const double real_slot_length = (pM01 - pM23).norm();
  // const auto t = (pt[1] - pt[0]).normalized();
  // const auto n = Eigen::Vector2d(t.y(), -t.x());
  const auto n = (pM01 - pM23).normalized();
  pt[2] = pt[0] - real_slot_length * n;
  pt[3] = pt[1] - real_slot_length * n;

  ego_slot_info.slot_corner = pt;

  const double virtual_slot_length = real_slot_length;

  ego_slot_info.slot_origin_pos = pM01 - virtual_slot_length * n;
  ego_slot_info.slot_origin_heading = std::atan2(n.y(), n.x());
  ego_slot_info.slot_origin_heading_vec = n;
  ego_slot_info.slot_length = virtual_slot_length;
  ego_slot_info.slot_width = (pt[0] - pt[1]).norm();

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

  // std::cout << "  ego_pos_slot = " << ego_slot_info.ego_pos_slot.transpose()
  //           << "  ego_heading_slot = " << ego_slot_info.ego_heading_slot
  //           * 57.3
  //           << std::endl;

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

  const double car_width_include_mirror =
      apa_param.GetParam().car_width + 2.0 * apa_param.GetParam().mirror_width;
  const double car_y_right_include_mirror = -car_width_include_mirror * 0.5;
  const double car_y_left_include_mirror = car_width_include_mirror * 0.5;

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

  slot_t_lane.pt_inside.x() += inside_dx;

  pt_inside_pose_ = ego_slot_info.l2g_tf.GetPos(slot_t_lane.pt_inside);

  slot_t_lane.pt_terminal_pos << ego_slot_info.target_ego_pos_slot.x(),
      ego_slot_info.target_ego_pos_slot.y();

  slot_t_lane.pt_terminal_heading = ego_slot_info.target_ego_heading_slot;

  slot_t_lane.pt_lower_boundry_pos = slot_t_lane.pt_terminal_pos;
  slot_t_lane.pt_lower_boundry_pos.x() =
      slot_t_lane.pt_lower_boundry_pos.x() -
      apa_param.GetParam().rear_overhanging -
      apa_param.GetParam().col_obs_safe_dist_normal - 0.05;

  const auto pt_0 = ego_slot_info.g2l_tf.GetPos(pt[0]);
  const auto pt_1 = ego_slot_info.g2l_tf.GetPos(pt[1]);
  ego_slot_info.pt_0 = pt_0;
  ego_slot_info.pt_1 = pt_1;

  planning::apa_planner::PerpendicularPathPlanner::Input input;
  input.pt_0 = ego_slot_info.pt_0;
  input.pt_1 = ego_slot_info.pt_1;
  input.slot_occupied_ratio = ego_slot_info.slot_occupied_ratio;
  input.tlane = slot_t_lane;
  input.is_complete_path = is_complete_path;
  input.sample_ds = ds;
  input.ref_arc_steer = frame.current_arc_steer;
  input.ref_gear = frame.current_gear;
  input.is_replan_first = frame.is_replan_first;
  input.is_replan_second = frame.is_replan_second;
  input.is_replan_dynamic = frame.is_replan_dynamic;
  input.ego_pose.Set(ego_slot_info.ego_pos_slot,
                     ego_slot_info.ego_heading_slot);

  pBase->SetInput(input);

  std::shared_ptr<planning::CollisionDetector> collision_detector_ptr = nullptr;
  collision_detector_ptr = std::make_shared<planning::CollisionDetector>();
  collision_detector_ptr->ClearObstacles();
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
      pBase->GenPathOutputByDubinsPb();
    } else if (pBase->GetCalcParams().directly_use_ego_pose) {
      std::cout << "ego pose is close to safe_circle_tang_pt, directly use"
                   "ego pose to multi plan \n";
    } else {
      std::cout << "prepare is complete fail\n";
      success = false;
      break;
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

  safe_circle_tang_pose_.setZero();
  const auto &tange_pos_global = ego_slot_info.l2g_tf.GetPos(
      pBase->GetCalcParams().safe_circle_tang_pt.pos);
  const auto &tange_heading_global = ego_slot_info.l2g_tf.GetHeading(
      pBase->GetCalcParams().safe_circle_tang_pt.heading);
  safe_circle_tang_pose_ << tange_pos_global.x(), tange_pos_global.y(),
      tange_heading_global;

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

Eigen::Vector3d GetTargetPose() { return global_target_pose_; }

Eigen::Vector3d GetCircleTangentPose() { return safe_circle_tang_pose_; }

Eigen::Vector2d GetPtInsidePose() { return pt_inside_pose_; }

PYBIND11_MODULE(vertical_planning_py, m) {
  m.doc() = "m";

  m.def("Init", &Init)
      .def("Update", &Update)
      .def("GetTargetPose", &GetTargetPose)
      .def("GetCircleTangentPose", &GetCircleTangentPose)
      .def("GetPtInsidePose", &GetPtInsidePose);
  ;
}