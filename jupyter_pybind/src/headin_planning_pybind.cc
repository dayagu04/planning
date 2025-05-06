#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <cstdint>
#include <iostream>
#include <vector>

#include "Eigen/Core"
#include "apa_plan_interface.h"
#include "apa_slot.h"
#include "collision_detection/collision_detection.h"
#include "config_context.h"
#include "geometry_math.h"
#include "ifly_time.h"
#include "math_lib.h"
#include "perpendicular_head_in_path_generator.h"
#include "perpendicular_head_in_scenario.h"
#include "src/modules/apa_function/apa_param_config.h"
namespace py = pybind11;
using namespace planning::apa_planner;

static PerpendicularPathHeadingInPlanner *pBase = nullptr;
static planning::apa_planner::ApaPlanInterface *pApaPlanInterface = nullptr;
int Init() {
  // Init glog
  planning::FilePath::SetName("headin_simulation_py");
  FLAGS_log_dir = "/asw/planning/log";
  planning::InitGlog(planning::FilePath::GetName().c_str());

  (void)planning::common::ConfigurationContext::Instance();

  pBase = new PerpendicularPathHeadingInPlanner();
  pBase->Reset();
  pApaPlanInterface = new planning::apa_planner::ApaPlanInterface();
  pApaPlanInterface->Init(true);

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

static PerpendicularTailInPathGenerator::DebugInfo debuginfo;
static std::vector<double> res;
std::vector<Eigen::Vector3d> current_path_point_global_vec_;
Eigen::Vector3d global_target_pose_;
Eigen::Vector3d safe_circle_tang_pose_;
Eigen::Vector2d pt_inside_pose_;
Eigen::Vector2d real_pt_inside_pose_;
std::vector<Eigen::Vector2d> pt_;

std::vector<Eigen::Vector2d> obs_pts_;

std::vector<Eigen::Vector3d> Update(Eigen::Vector3d ego_pose,
                                    std::vector<Eigen::Vector2d> raw_pt,
                                    double ds, bool is_complete_path,
                                    double inside_dx, double radius_add,
                                    std::vector<double> obs_params) {
  std::cout << "---------- HEADIN PARK TEST -----------\n";

  if (obs_params.size() != 7) {
    current_path_point_global_vec_.clear();
    return current_path_point_global_vec_;
  }

  planning::apa_planner::ParkingScenario::Frame frame;
  // 1. Ego Slot Info
  auto &ego_slot_info = frame.ego_slot_info;
  pt_.clear();
  pt_.resize(4);

  // to decide slot corners
  // default: left side slot
  pt_[0] = raw_pt[0];
  pt_[1] = raw_pt[1];
  pt_[2] = raw_pt[2];
  pt_[3] = raw_pt[3];
  //

  const auto pM01 = 0.5 * (pt_[0] + pt_[1]);
  const auto pM23 = 0.5 * (pt_[2] + pt_[3]);
  const double real_slot_length = (pM01 - pM23).norm();
  const auto t = (pt_[1] - pt_[0]).normalized();
  const auto n = Eigen::Vector2d(t.y(), -t.x());

  ego_slot_info.slot_origin_pos = pM01 - real_slot_length * n;
  ego_slot_info.slot_origin_heading = std::atan2(n.y(), n.x());
  ego_slot_info.slot_origin_heading_vec = n;
  ego_slot_info.slot_length = real_slot_length;
  ego_slot_info.slot_width = (pt_[0] - pt_[1]).norm();
  ego_slot_info.g2l_tf.Init(ego_slot_info.slot_origin_pos,
                            ego_slot_info.slot_origin_heading);

  ego_slot_info.l2g_tf.Init(ego_slot_info.slot_origin_pos,
                            ego_slot_info.slot_origin_heading);

  // std::cout << "slot_origin_pos = " <<
  // ego_slot_info.slot_origin_pos.transpose()
  //           << std::endl;

  // 2. Ego Pose in slot
  Eigen::Vector2d pos_ego(ego_pose.x(), ego_pose.y());
  double heading_ego = ego_pose.z();
  ego_slot_info.ego_pos_slot = ego_slot_info.g2l_tf.GetPos(pos_ego);
  ego_slot_info.ego_heading_slot = ego_slot_info.g2l_tf.GetHeading(heading_ego);
  ego_slot_info.ego_heading_slot_vec
      << std::cos(ego_slot_info.ego_heading_slot),
      std::sin(ego_slot_info.ego_heading_slot);

  std::cout << "ego_pos_slot = " << ego_slot_info.ego_pos_slot.transpose()
            << "  ego_heading_slot = " << ego_slot_info.ego_heading_slot * 57.3
            << std::endl;

  // 3. Target Pos in slot
  ego_slot_info.target_ego_pos_slot
      << apa_param.GetParam().terminal_target_x + 3.0,
      apa_param.GetParam().terminal_target_y;

  if (ego_slot_info.ego_heading_slot > 0.0) {
    ego_slot_info.target_ego_heading_slot =
        apa_param.GetParam().terminal_target_heading + 180.0 / 57.3;
  } else {
    ego_slot_info.target_ego_heading_slot =
        apa_param.GetParam().terminal_target_heading - 180.0 / 57.3;
  }

  const auto &target_ego_pos_global =
      ego_slot_info.l2g_tf.GetPos(ego_slot_info.target_ego_pos_slot);

  const auto &target_ego_heading_global =
      ego_slot_info.l2g_tf.GetHeading(ego_slot_info.target_ego_heading_slot);

  global_target_pose_ << target_ego_pos_global.x(), target_ego_pos_global.y(),
      target_ego_heading_global;

  std::cout << "target_pos_local = "
            << ego_slot_info.target_ego_pos_slot.transpose()
            << " target_heading_local = "
            << ego_slot_info.target_ego_heading_slot * 57.3 << std::endl;

  // 4. Terminal Error in slot
  ego_slot_info.terminal_err.Set(
      ego_slot_info.ego_pos_slot - ego_slot_info.target_ego_pos_slot,
      ego_slot_info.ego_heading_slot - ego_slot_info.target_ego_heading_slot);

  // 5. Slot Occupied Ratio
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

  // 6. Slot Side
  // judge slot side via slot center and heading
  Eigen::Vector2d ego_to_slot_center_vec = 0.5 * (pM01 + pM23) - pos_ego;
  const auto heading_ego_vec =
      Eigen::Vector2d(std::cos(ego_pose.z()), std::sin(ego_pose.z()));

  const double cross_ego_to_slot_center =
      pnc::geometry_lib::GetCrossFromTwoVec2d(heading_ego_vec,
                                              ego_to_slot_center_vec);

  const double cross_ego_to_slot_heading =
      pnc::geometry_lib::GetCrossFromTwoVec2d(
          heading_ego_vec, ego_slot_info.slot_origin_heading_vec);

  planning::apa_planner::Tlane slot_t_lane;

  frame.current_gear = pnc::geometry_lib::SEG_GEAR_DRIVE;
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

  // 7. slot corner same as tail in slot
  if (slot_t_lane.slot_side == pnc::geometry_lib::SLOT_SIDE_RIGHT) {
    pt_[0] = raw_pt[1];
    pt_[1] = raw_pt[0];
    pt_[2] = raw_pt[3];
    pt_[3] = raw_pt[2];
  } else {
    pt_[0] = raw_pt[0];
    pt_[1] = raw_pt[1];
    pt_[2] = raw_pt[2];
    pt_[3] = raw_pt[3];
  }

  // std::cout << "pt_[0] = " << pt_[0].transpose() << std::endl;
  // std::cout << "pt_[1] = " << pt_[1].transpose() << std::endl;
  // std::cout << "pt_[2] = " << pt_[2].transpose() << std::endl;
  // std::cout << "pt_[3] = " << pt_[3].transpose() << std::endl;

  // final corner in local
  Eigen::Vector2d pt_0 = ego_slot_info.g2l_tf.GetPos(pt_[0]);
  Eigen::Vector2d pt_1 = ego_slot_info.g2l_tf.GetPos(pt_[1]);
  ego_slot_info.pt_0 = pt_0;
  ego_slot_info.pt_1 = pt_1;
  ego_slot_info.sin_angle = 1.0;
  ego_slot_info.origin_pt_0_heading = 0.0;
  ego_slot_info.slot_corner = pt_;
  // above ego_slot_info

  // 8. Tlane
  // construct slot_t_lane, left is positive, right is negative
  const double car_width_include_mirror =
      apa_param.GetParam().car_width + 2.0 * 0.2;

  const double car_y_right_include_mirror = -car_width_include_mirror / 2.0;
  const double car_y_left_include_mirror = car_width_include_mirror / 2.0;
  const double virtual_slot_width =
      car_width_include_mirror + apa_param.GetParam().slot_compare_to_car_width;

  const double real_slot_width = ego_slot_info.slot_width;
  const double slot_width = std::max(virtual_slot_width, real_slot_width);
  Eigen::Vector2d corner_left_slot(ego_slot_info.slot_length, 0.5 * slot_width);
  Eigen::Vector2d corner_right_slot(ego_slot_info.slot_length,
                                    -0.5 * slot_width);

  const auto &slot_side = slot_t_lane.slot_side;
  if (slot_side == pnc::geometry_lib::SLOT_SIDE_RIGHT) {
    // inside is left, outside is right
    slot_t_lane.corner_outside_slot = corner_right_slot;
    slot_t_lane.corner_inside_slot = corner_left_slot;
    slot_t_lane.pt_outside = corner_right_slot;
    slot_t_lane.pt_inside = corner_left_slot;
  } else if (slot_side == pnc::geometry_lib::SLOT_SIDE_LEFT) {
    // outside is left, inside is right
    slot_t_lane.corner_outside_slot = corner_left_slot;
    slot_t_lane.corner_inside_slot = corner_right_slot;
    slot_t_lane.pt_outside = corner_left_slot;
    slot_t_lane.pt_inside = corner_right_slot;
  }

  slot_t_lane.pt_terminal_pos << ego_slot_info.target_ego_pos_slot.x(),
      ego_slot_info.target_ego_pos_slot.y();

  slot_t_lane.pt_terminal_heading = ego_slot_info.target_ego_heading_slot;

  slot_t_lane.pt_lower_boundry_pos = slot_t_lane.pt_terminal_pos;
  slot_t_lane.pt_lower_boundry_pos.x() =
      slot_t_lane.pt_lower_boundry_pos.x() -
      apa_param.GetParam().rear_overhanging -
      apa_param.GetParam().col_obs_safe_dist_normal - 0.05;

  // 9. Tlane Obstacle
  // default : right side params
  double right_obj_dx = obs_params[0];
  double right_obj_dy = obs_params[1];
  double left_obj_dx = obs_params[2];
  double left_obj_dy = obs_params[3];
  if (slot_side == pnc::geometry_lib::SLOT_SIDE_LEFT) {
    right_obj_dx = obs_params[2];
    right_obj_dy = -obs_params[3];
    left_obj_dx = obs_params[0];
    left_obj_dy = -obs_params[1];
  }

  const double channel_width = obs_params[4];
  // to adjust monoplan circle center
  const double slot_inside_obs_x = obs_params[5];
  const double slot_inside_obs_y = obs_params[6];
  apa_param.SetPram().max_pt_inside_drop_dx_mono_headin = slot_inside_obs_x;
  apa_param.SetPram().max_pt_inside_drop_dx_multi_headin = slot_inside_obs_x;
  apa_param.SetPram().headin_max_pt_inside_drop_dy = slot_inside_obs_y;
  const double channel_length = apa_param.GetParam().channel_length;
  std::cout << " channel_length = " << channel_length << std::endl;

  const Eigen::Vector2d vec_01 = pt_[1] - pt_[0];
  const Eigen::Vector2d unit_vec_01 = vec_01.normalized();
  const Eigen::Vector2d vec_02 = pt_[2] - pt_[0];
  const Eigen::Vector2d unit_vec_02 = vec_02.normalized();
  const double obs_length = (channel_length - vec_01.norm()) / 2.0;

  const Eigen::Vector2d obj_pt_0 =
      pt_[0] + left_obj_dx * unit_vec_02 +
      left_obj_dy * Eigen::Vector2d(unit_vec_02.y(), -unit_vec_02.x());

  const Eigen::Vector2d left_obj_pt_horizontal =
      obj_pt_0 - obs_length * unit_vec_01;

  const Eigen::Vector2d left_obj_pt_vertical =
      obj_pt_0 + unit_vec_02 * (vec_02.norm() - left_obj_dx + 0.3);

  const Eigen::Vector2d obj_pt_1 =
      pt_[1] + right_obj_dx * unit_vec_02 +
      right_obj_dy * Eigen::Vector2d(-unit_vec_02.y(), unit_vec_02.x());

  const Eigen::Vector2d right_obj_pt_horizontal =
      obj_pt_1 + obs_length * unit_vec_01;

  const Eigen::Vector2d right_obj_pt_vertical =
      obj_pt_1 + unit_vec_02 * (vec_02.norm() - right_obj_dx + 0.3);

  Eigen::Vector2d channel_mid =
      (pt_[0] + pt_[1]) / 2.0 +
      channel_width * Eigen::Vector2d(-unit_vec_01.y(), unit_vec_01.x());

  if (slot_side == pnc::geometry_lib::SLOT_SIDE_LEFT) {
    channel_mid =
        (pt_[0] + pt_[1]) / 2.0 +
        channel_width * Eigen::Vector2d(unit_vec_01.y(), -unit_vec_01.x());
  }

  const Eigen::Vector2d channel_left =
      channel_mid - channel_length / 2.0 * unit_vec_01;

  const Eigen::Vector2d channel_right =
      channel_mid + channel_length / 2.0 * unit_vec_01;

  Eigen::Vector2d A, B, C, D, E, F, G, H;
  A = left_obj_pt_horizontal;
  B = obj_pt_0;
  C = left_obj_pt_vertical;
  D = right_obj_pt_vertical;
  E = obj_pt_1;
  F = right_obj_pt_horizontal;
  G << Eigen::Vector2d(right_obj_pt_horizontal.x(), channel_right.y());
  H << Eigen::Vector2d(left_obj_pt_horizontal.x(), channel_right.y());

  if (slot_side == pnc::geometry_lib::SLOT_SIDE_LEFT) {
    slot_t_lane.corner_inside_slot = ego_slot_info.g2l_tf.GetPos(E);
    slot_t_lane.corner_outside_slot = ego_slot_info.g2l_tf.GetPos(B);
  } else {
    slot_t_lane.corner_inside_slot = ego_slot_info.g2l_tf.GetPos(B);
    slot_t_lane.corner_outside_slot = ego_slot_info.g2l_tf.GetPos(E);
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

  std::shared_ptr<planning::apa_planner::CollisionDetector>
      collision_detector_ptr = nullptr;
  collision_detector_ptr =
      std::make_shared<planning::apa_planner::CollisionDetector>();
  collision_detector_ptr->ClearObstacles();

  std::vector<Eigen::Vector2d> obs_local_pts;
  for (const auto &obs_pt : obs_pts_) {
    obs_local_pts.emplace_back(ego_slot_info.g2l_tf.GetPos(obs_pt));
  }

  collision_detector_ptr->SetObstacles(
      obs_local_pts, planning::apa_planner::CollisionDetector::TLANE_OBS);

  // assemble input
  // new slot management
  planning::apa_planner::GeometryPathInput new_input;
  planning::apa_planner::EgoInfoUnderSlot slot_input;

  slot_input.slot_type = planning::apa_planner::SlotType::PERPENDICULAR;
  slot_input.cur_pose.Set(ego_slot_info.ego_pos_slot,
                          ego_slot_info.ego_heading_slot);

  slot_input.target_pose.Set(slot_t_lane.pt_terminal_pos,
                             slot_t_lane.pt_terminal_heading);

  slot_input.g2l_tf = ego_slot_info.g2l_tf;
  slot_input.l2g_tf = ego_slot_info.l2g_tf;
  slot_input.pt_inside << ego_slot_info.pt_0.x() + inside_dx,
      ego_slot_info.pt_0.y() + obs_params[6];

  slot_input.slot.origin_corner_coord_global_.pt_0 = pt_[0];
  slot_input.slot.origin_corner_coord_global_.pt_1 = pt_[1];
  slot_input.slot.origin_corner_coord_global_.pt_2 = pt_[2];
  slot_input.slot.origin_corner_coord_global_.pt_3 = pt_[3];
  slot_input.slot.origin_corner_coord_global_.CalExtraCoord();
  slot_input.slot.processed_corner_coord_global_ =
      slot_input.slot.origin_corner_coord_global_;

  if (slot_input.slot_type == planning::apa_planner::SlotType::PERPENDICULAR) {
    slot_input.slot.sin_angle_ = 1.0;
    slot_input.slot.angle_ = 90.0;
  } else {
    slot_input.slot.angle_ =
        std::fabs(geometry_lib::GetAngleFromTwoVec(
            slot_input.slot.origin_corner_coord_global_.pt_23mid_01mid_vec,
            slot_input.slot.origin_corner_coord_global_.pt_01_vec)) *
        kRad2Deg;

    if (slot_input.slot.angle_ > 90.0) {
      slot_input.slot.angle_ = 180.0 - slot_input.slot.angle_;
    }

    slot_input.slot.angle_ =
        mathlib::DoubleConstrain(slot_input.slot.angle_, 10.0, 80.0);

    slot_input.slot.sin_angle_ = std::sin(slot_input.slot.angle_ * kDeg2Rad);
    ILOG_INFO << "slant slot, should postprocess corner to perpendicular";

    const Eigen::Vector2d pt_01_vec =
        slot_input.slot.origin_corner_coord_global_.pt_1 -
        slot_input.slot.origin_corner_coord_global_.pt_0;
    const Eigen::Vector2d pt_01_unit_vec = pt_01_vec.normalized();

    const Eigen::Vector2d pt_02_vec =
        slot_input.slot.origin_corner_coord_global_.pt_2 -
        slot_input.slot.origin_corner_coord_global_.pt_0;
    const Eigen::Vector2d pt_02_unit_vec = pt_02_vec.normalized();

    const Eigen::Vector2d pt_13_vec =
        slot_input.slot.origin_corner_coord_global_.pt_3 -
        slot_input.slot.origin_corner_coord_global_.pt_1;
    const Eigen::Vector2d pt_13_unit_vec = pt_13_vec.normalized();

    const double cos_theta = pt_01_unit_vec.dot(pt_02_unit_vec);

    if (cos_theta > 0.0) {
      // toward right
      const double dis_0_0dot = pt_01_vec.dot(pt_02_unit_vec);
      const Eigen::Vector2d pt_0dot =
          slot_input.slot.origin_corner_coord_global_.pt_0 +
          dis_0_0dot * pt_02_unit_vec;
      const double dist_0dot_2 = pt_02_vec.norm() - dis_0_0dot;
      const Eigen::Vector2d pt_3dot =
          slot_input.slot.origin_corner_coord_global_.pt_1 +
          dist_0dot_2 * pt_02_unit_vec;
      slot_input.slot.processed_corner_coord_global_.pt_0 = pt_0dot;
      slot_input.slot.processed_corner_coord_global_.pt_3 = pt_3dot;
    } else {
      // toward left
      const Eigen::Vector2d pt_10_vec = -pt_01_vec;
      const double dist_1_1dot = pt_10_vec.dot(pt_13_unit_vec);
      const Eigen::Vector2d pt_1dot =
          slot_input.slot.origin_corner_coord_global_.pt_1 +
          dist_1_1dot * pt_13_unit_vec;
      const double dist_1dot_3 = pt_13_vec.norm() - dist_1_1dot;
      const Eigen::Vector2d pt_2dot =
          slot_input.slot.origin_corner_coord_global_.pt_0 +
          dist_1dot_3 * pt_13_unit_vec;
      slot_input.slot.processed_corner_coord_global_.pt_1 = pt_1dot;
      slot_input.slot.processed_corner_coord_global_.pt_2 = pt_2dot;
    }

    slot_input.slot.processed_corner_coord_global_.CalExtraCoord();
  }

  slot_input.slot.slot_length_ =
      (slot_input.slot.processed_corner_coord_global_.pt_01_mid -
       slot_input.slot.processed_corner_coord_global_.pt_23_mid)
          .norm();

  slot_input.slot.slot_width_ =
      (slot_input.slot.processed_corner_coord_global_.pt_1 -
       slot_input.slot.processed_corner_coord_global_.pt_0)
          .norm();

  slot_input.slot.origin_corner_coord_local_ =
      slot_input.slot.origin_corner_coord_global_.GlobalToLocal(
          slot_input.g2l_tf);

  slot_input.slot.processed_corner_coord_local_ =
      slot_input.slot.processed_corner_coord_global_.GlobalToLocal(
          slot_input.g2l_tf);

  slot_input.slot.origin_corner_coord_local_.pt_0 =
      slot_input.slot.origin_corner_coord_local_.pt_1;

  slot_input.slot.origin_corner_coord_local_.pt_1 =
      slot_input.slot.origin_corner_coord_local_.pt_0;
  // no use
  slot_input.origin_pose_global.heading_vec =
      slot_input.slot.processed_corner_coord_global_.pt_23mid_01mid_unit_vec;

  slot_input.origin_pose_global.heading =
      std::atan2(slot_input.origin_pose_global.heading_vec.y(),
                 slot_input.origin_pose_global.heading_vec.x());

  slot_input.origin_pose_global.pos =
      slot_input.slot.processed_corner_coord_global_.pt_01_mid -
      slot_input.slot.slot_length_ * slot_input.origin_pose_global.heading_vec;

  slot_input.origin_pose_local.pos =
      slot_input.g2l_tf.GetPos(slot_input.origin_pose_global.pos);

  slot_input.origin_pose_local.heading =
      slot_input.g2l_tf.GetHeading(slot_input.origin_pose_global.heading);

  slot_input.origin_pose_local.heading_vec =
      geometry_lib::GenHeadingVec(slot_input.origin_pose_local.heading);
  //

  slot_input.slot.limiter_.valid = true;

  slot_input.slot.limiter_.start_pt =
      Eigen::Vector2d(global_target_pose_.x(), global_target_pose_.y()) -
      slot_input.slot.processed_corner_coord_global_.pt_01_unit_vec * 0.5 *
          slot_input.slot.slot_width_;

  slot_input.slot.limiter_.end_pt =
      Eigen::Vector2d(global_target_pose_.x(), global_target_pose_.y()) +
      slot_input.slot.processed_corner_coord_global_.pt_01_unit_vec * 0.5 *
          slot_input.slot.slot_width_;

  slot_input.virtual_limiter.first =
      slot_input.g2l_tf.GetPos(slot_input.slot.limiter_.start_pt);

  slot_input.virtual_limiter.second =
      slot_input.g2l_tf.GetPos(slot_input.slot.limiter_.end_pt);

  slot_input.terminal_err.Set(
      slot_input.cur_pose.pos - slot_input.target_pose.pos,
      geometry_lib::NormalizeAngle(slot_input.cur_pose.heading -
                                   slot_input.target_pose.heading));

  new_input.ego_info_under_slot = slot_input;
  new_input.is_complete_path = is_complete_path;
  new_input.sample_ds = 0.2;
  new_input.ref_arc_steer = frame.current_arc_steer;
  new_input.ref_gear = frame.current_gear;
  new_input.is_replan_first = frame.is_replan_first;
  new_input.is_replan_second = frame.is_replan_second;
  new_input.is_replan_dynamic = frame.is_replan_dynamic;
  new_input.ego_info_under_slot.slot_side =
      static_cast<pnc::geometry_lib::SlotSide>(slot_t_lane.slot_side);
  pBase->SetInput(new_input);

  apa_param.SetPram().radius_add = radius_add;

  // std::cout << "input.ref_gear = " << input.ref_gear << std::endl;
  // std::cout << "input.ref_arc_steer = " << input.ref_arc_steer << std::endl;
  pBase->SetColPtr(collision_detector_ptr);
  pBase->PreprocessForSimu();

  bool success = false;
  current_path_point_global_vec_.clear();

  double time_start = IflyTime::Now_ms();
  double time_end = time_start;
  success = pBase->UpdatePb();
  double time_allend = IflyTime::Now_ms();
  std::cout << "prepare plan cost time = " << time_end - time_start
            << std::endl;
  std::cout << "whole plan cost time = " << time_allend - time_start
            << std::endl;

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

  std::cout << "this plan success = " << success << std::endl;
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

Eigen::Vector3d GetCircleTangentPose() {
  // std::cout << "tange_pos_global = " << safe_circle_tang_pose_.transpose()
  //           << std::endl;
  return safe_circle_tang_pose_;
}

Eigen::Vector2d GetPtInsidePose() { return pt_inside_pose_; }

Eigen::Vector2d GetRealPtInsidePose() { return real_pt_inside_pose_; }

const std::vector<Eigen::Vector2d> &GetRectangleSoltPos() { return pt_; }

PYBIND11_MODULE(headin_planning_py, m) {
  m.doc() = "m";

  m.def("Init", &Init)
      .def("Update", &Update)
      .def("GetTargetPose", &GetTargetPose)
      .def("GetCircleTangentPose", &GetCircleTangentPose)
      .def("GetPtInsidePose", &GetPtInsidePose)
      .def("GetRealPtInsidePose", &GetRealPtInsidePose)
      .def("GetRectangleSoltPos", &GetRectangleSoltPos)
      .def("GetObstacles", &GetObstacles);
  ;
}