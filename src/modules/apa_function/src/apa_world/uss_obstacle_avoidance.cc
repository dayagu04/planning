#include "uss_obstacle_avoidance.h"

#include <cstddef>
#include <utility>

#include "apa_param_setting.h"
#include "common_c.h"
#include "debug_info_log.h"
#include "geometry_math.h"
#include "transform_lib.h"

namespace planning {
namespace apa_planner {
void UssObstacleAvoidance::Init() {
  // init car local vertex
  car_local_vertex_vec_.clear();
  car_local_vertex_vec_.reserve(apa_param.GetParam().car_vertex_x_vec.size() +
                                4);
  for (size_t i = 0; i < apa_param.GetParam().car_vertex_x_vec.size(); ++i) {
    car_local_vertex_vec_.emplace_back(
        Eigen::Vector2d(apa_param.GetParam().car_vertex_x_vec[i],
                        apa_param.GetParam().car_vertex_y_vec[i]));
    if (i == 0) {
      car_local_vertex_vec_.emplace_back(
          Eigen::Vector2d((apa_param.GetParam().car_vertex_x_vec[0] +
                           apa_param.GetParam().car_vertex_x_vec[1]) *
                              0.5,
                          (apa_param.GetParam().car_vertex_y_vec[0] +
                           apa_param.GetParam().car_vertex_y_vec[1]) *
                              0.5));
    }

    if (i == 4) {
      car_local_vertex_vec_.emplace_back(
          Eigen::Vector2d((apa_param.GetParam().car_vertex_x_vec[4] +
                           apa_param.GetParam().car_vertex_x_vec[5]) *
                              0.5,
                          (apa_param.GetParam().car_vertex_y_vec[4] +
                           apa_param.GetParam().car_vertex_y_vec[5]) *
                              0.5));
    }

    if (i == 2) {
      car_local_vertex_vec_.emplace_back(
          Eigen::Vector2d(apa_param.GetParam().car_vertex_x_vec[i],
                          apa_param.GetParam().uss_vertex_y_vec[2]));
      car_local_vertex_vec_.emplace_back(
          Eigen::Vector2d(apa_param.GetParam().car_vertex_x_vec[i],
                          apa_param.GetParam().uss_vertex_y_vec[3]));
    }

    if (i == 10) {
      car_local_vertex_vec_.emplace_back(
          Eigen::Vector2d((apa_param.GetParam().car_vertex_x_vec[10] +
                           apa_param.GetParam().car_vertex_x_vec[11]) *
                              0.5,
                          (apa_param.GetParam().car_vertex_y_vec[10] +
                           apa_param.GetParam().car_vertex_y_vec[11]) *
                              0.5));
    }

    if (i == 12) {
      car_local_vertex_vec_.emplace_back(
          Eigen::Vector2d(apa_param.GetParam().car_vertex_x_vec[i],
                          apa_param.GetParam().uss_vertex_y_vec[8]));
      car_local_vertex_vec_.emplace_back(
          Eigen::Vector2d(apa_param.GetParam().car_vertex_x_vec[i],
                          apa_param.GetParam().uss_vertex_y_vec[9]));
    }

    if (i == 14) {
      car_local_vertex_vec_.emplace_back(
          Eigen::Vector2d((apa_param.GetParam().car_vertex_x_vec[14] +
                           apa_param.GetParam().car_vertex_x_vec[15]) *
                              0.5,
                          (apa_param.GetParam().car_vertex_y_vec[14] +
                           apa_param.GetParam().car_vertex_y_vec[15]) *
                              0.5));
    }
  }

  for (size_t i = 0; i < car_local_vertex_vec_.size(); ++i) {
    if (car_local_vertex_vec_[i].y() > 0.46) {
      car_local_vertex_vec_[i].y() += param_.lat_inflation;
    } else if (car_local_vertex_vec_[i].y() < -0.46) {
      car_local_vertex_vec_[i].y() -= param_.lat_inflation;
    }
  }

  // init uss local vertex and normal angle
  uss_local_vertex_vec_.clear();
  uss_local_vertex_vec_.reserve(apa_param.GetParam().uss_vertex_x_vec.size());
  uss_local_normal_angle_vec_.clear();
  uss_local_normal_angle_vec_.reserve(
      apa_param.GetParam().uss_vertex_x_vec.size());

  for (size_t i = 0; i < apa_param.GetParam().uss_vertex_x_vec.size(); ++i) {
    uss_local_vertex_vec_.emplace_back(
        Eigen::Vector2d(apa_param.GetParam().uss_vertex_x_vec[i],
                        apa_param.GetParam().uss_vertex_y_vec[i]));
    // converted to the angle relative to the car axis
    uss_local_normal_angle_vec_.emplace_back(pnc::mathlib::Deg2Rad(
        apa_param.GetParam().uss_normal_angle_deg_vec[i] - 90.0));
  }
}

const std::vector<Eigen::Vector2d> UssObstacleAvoidance::GetCarLocalVertex() {
  return car_local_vertex_vec_;
}

const std::vector<Eigen::Vector2d> UssObstacleAvoidance::GetUssLocalVertex() {
  return uss_local_vertex_vec_;
}

const std::vector<double> UssObstacleAvoidance::GetUssLocalAngle() {
  return uss_local_normal_angle_vec_;
}

void UssObstacleAvoidance::SetLatInflation() { Init(); }

void UssObstacleAvoidance::GenCarLine() {
  car_local_line_vec_.clear();
  car_local_line_vec_.reserve(car_local_vertex_vec_.size());

  pnc::geometry_lib::LineSegment car_local_line;
  for (const auto &car_local_vertex : car_local_vertex_vec_) {
    car_local_line.pA = car_local_vertex;
    car_local_line.pB = car_local_line.pA;

    if (car_motion_info_.reverse_flag) {
      car_local_line.pB.x() -= param_.detection_distance;
    } else {
      car_local_line.pB.x() += param_.detection_distance;
    }

    car_local_line_vec_.emplace_back(car_local_line);
  }
}

void UssObstacleAvoidance::GenCarArc() {
  car_local_arc_vec_.clear();
  car_local_arc_vec_.reserve(car_local_vertex_vec_.size());

  //  1.0 represents car rotates anticlockwise
  // -1.0 represents car rotates clockwise
  double sign = 1.0;
  if ((!car_motion_info_.reverse_flag && car_motion_info_.steer_angle > 0.0) ||
      (car_motion_info_.reverse_flag && car_motion_info_.steer_angle < 0.0)) {
    // forward and turn left, or back and turn right
    sign = 1.0;
  } else {
    // forward and turn right, or back and turn left
    sign = -1.0;
  }

  // rotation angle
  const auto rot_angle = param_.detection_distance /
                         car_motion_info_.rear_axle_center_turn_radius * sign;

  // rotation matrix
  const auto rot_matrix = pnc::geometry_lib::GetRotm2dFromTheta(rot_angle);

  pnc::geometry_lib::Arc car_local_arc;
  for (const auto &car_local_vertex : car_local_vertex_vec_) {
    // p is rear axle center, O is turn center
    // OA = pA - pO,  OB = rot_m * OA,  pB = pO + OB
    car_local_arc.circle_info.center = car_motion_info_.turning_center;
    car_local_arc.pA = car_local_vertex;

    car_local_arc.circle_info.radius =
        (car_local_arc.pA - car_local_arc.circle_info.center).norm();

    car_local_arc.pB =
        car_local_arc.circle_info.center +
        rot_matrix * (car_local_arc.pA - car_local_arc.circle_info.center);

    car_local_arc_vec_.emplace_back(car_local_arc);
  }
}

void UssObstacleAvoidance::GenUssArc() {
  uss_local_arc_vec_.clear();
  uss_local_arc_vec_.reserve(uss_local_vertex_vec_.size());

  if (apa_param.GetParam().enable_corner_uss_process) {
    const double apa_scan_angle_deg =
        apa_param.GetParam().uss_apa_scan_angle_deg;
    const double upa_scan_angle_deg =
        apa_param.GetParam().uss_upa_scan_angle_deg;

    const auto rot_matrix_apa = pnc::geometry_lib::GetRotm2dFromTheta(
        pnc::mathlib::Deg2Rad(apa_scan_angle_deg * 0.5));

    const auto rot_matrix_upa = pnc::geometry_lib::GetRotm2dFromTheta(
        pnc::mathlib::Deg2Rad(upa_scan_angle_deg * 0.5));

    const auto rot_matrix_corner_uss_straight =
        pnc::geometry_lib::GetRotm2dFromTheta(pnc::mathlib::Deg2Rad(
            apa_param.GetParam().corner_uss_scan_angle_deg_straight));

    const auto rot_matrix_corner_uss_turn =
        pnc::geometry_lib::GetRotm2dFromTheta(pnc::mathlib::Deg2Rad(
            apa_param.GetParam().corner_uss_scan_angle_deg_turn));

    const double corner_uss_dist_diff = 0.1068;

    Eigen::Matrix2d rot_matrix;

    pnc::geometry_lib::Arc uss_local_arc;
    Eigen::Vector2d uss_local_vertex;
    Eigen::Vector2d OC;
    double uss_local_normal_angle;
    for (size_t i = 0; i < uss_local_vertex_vec_.size(); ++i) {
      uss_local_vertex = uss_local_vertex_vec_[i];

      if (uss_raw_dist_vec_[i] > param_.detection_distance) {
        // beyond the detection distance will not be considered
        uss_local_arc.is_ignored = true;
      } else {
        uss_local_arc.is_ignored = false;
      }

      uss_local_arc.circle_info.center = uss_local_vertex;

      uss_local_normal_angle = uss_local_normal_angle_vec_[i];

      uss_local_arc.circle_info.radius = uss_raw_dist_vec_[i];

      // p is rear axle center, O is uss vertex, C is center of uss arc
      OC = uss_local_arc.circle_info.radius *
           Eigen::Vector2d(std::cos(uss_local_normal_angle),
                           std::sin(uss_local_normal_angle));

      if (i == 0 || i == 5 || i == 6 || i == 11) {
        rot_matrix = rot_matrix_apa;
      } else {
        rot_matrix = rot_matrix_upa;
      }

      // pB = pO + OB, pA = pO + OA
      // OC rotates anticlockwise to obtain OB
      // OC rotates clockwise to obtain OA
      uss_local_arc.pB = rot_matrix * OC + uss_local_arc.circle_info.center;

      uss_local_arc.pA =
          rot_matrix.transpose() * OC + uss_local_arc.circle_info.center;

      // hack: avoid corner uss invalid stuck
      // todo: need move when uss can be trust
      if (std::fabs(car_motion_info_.steer_angle * 57.3) <
          apa_param.GetParam().corner_uss_steer_angle) {
        if (i == 1 || i == 7) {
          uss_local_arc.pA = rot_matrix_corner_uss_straight.transpose() * OC +
                             uss_local_arc.circle_info.center;
        } else if (i == 4 || i == 10) {
          uss_local_arc.pB = rot_matrix_corner_uss_straight * OC +
                             uss_local_arc.circle_info.center;
        }
      } else {
        if (car_motion_info_.steer_angle > 0.0) {
          if (i == 1) {
            if (uss_raw_dist_vec_[1] >
                uss_raw_dist_vec_[0] + corner_uss_dist_diff) {
              uss_local_arc.pA =
                  rot_matrix_corner_uss_straight.transpose() * OC +
                  uss_local_arc.circle_info.center;
            } else {
              uss_local_arc.pA = rot_matrix_corner_uss_turn.transpose() * OC +
                                 uss_local_arc.circle_info.center;
            }
          }
          if (i == 4) {
            uss_local_arc.pB = rot_matrix_corner_uss_straight * OC +
                               uss_local_arc.circle_info.center;
          }
          if (i == 7) {
            if (uss_raw_dist_vec_[7] >
                uss_raw_dist_vec_[6] + corner_uss_dist_diff) {
              uss_local_arc.pA =
                  rot_matrix_corner_uss_straight.transpose() * OC +
                  uss_local_arc.circle_info.center;
            } else {
              uss_local_arc.pA = rot_matrix_corner_uss_turn.transpose() * OC +
                                 uss_local_arc.circle_info.center;
            }
          }
          if (i == 10) {
            uss_local_arc.pB = rot_matrix_corner_uss_straight * OC +
                               uss_local_arc.circle_info.center;
          }
        } else {
          if (i == 1) {
            uss_local_arc.pA = rot_matrix_corner_uss_straight.transpose() * OC +
                               uss_local_arc.circle_info.center;
          }
          if (i == 4) {
            if (uss_raw_dist_vec_[4] >
                uss_raw_dist_vec_[5] + corner_uss_dist_diff) {
              uss_local_arc.pB = rot_matrix_corner_uss_straight * OC +
                                 uss_local_arc.circle_info.center;
            } else {
              uss_local_arc.pB = rot_matrix_corner_uss_turn * OC +
                                 uss_local_arc.circle_info.center;
            }
          }
          if (i == 7) {
            uss_local_arc.pA = rot_matrix_corner_uss_straight.transpose() * OC +
                               uss_local_arc.circle_info.center;
          }
          if (i == 10) {
            if (uss_raw_dist_vec_[10] >
                uss_raw_dist_vec_[11] + corner_uss_dist_diff) {
              uss_local_arc.pB = rot_matrix_corner_uss_straight * OC +
                                 uss_local_arc.circle_info.center;
            } else {
              uss_local_arc.pB = rot_matrix_corner_uss_turn * OC +
                                 uss_local_arc.circle_info.center;
            }
          }
        }
      }
      uss_local_arc_vec_.emplace_back(uss_local_arc);
    }
    return;
  }

  // 1 / 4 / 7 / 10 vertexs are more likely to collide, so set a larger scan
  // angle
  const auto rot_matrix_normal = pnc::geometry_lib::GetRotm2dFromTheta(
      pnc::mathlib::Deg2Rad(apa_param.GetParam().uss_face_scan_angle_gain *
                            apa_param.GetParam().uss_scan_angle_deg * 0.5));

  const auto rot_matrix_large = pnc::geometry_lib::GetRotm2dFromTheta(
      pnc::mathlib::Deg2Rad(apa_param.GetParam().uss_corner_scan_angle_gain *
                            apa_param.GetParam().uss_scan_angle_deg * 0.5));

  Eigen::Matrix2d rot_matrix;

  pnc::geometry_lib::Arc uss_local_arc;
  Eigen::Vector2d uss_local_vertex;
  Eigen::Vector2d OC;
  double uss_local_normal_angle;
  for (size_t i = 0; i < uss_local_vertex_vec_.size(); ++i) {
    if (i == 1 || i == 4 || i == 7 || i == 10) {
      rot_matrix = rot_matrix_large;
    } else {
      rot_matrix = rot_matrix_normal;
    }
    uss_local_vertex = uss_local_vertex_vec_[i];

    if (uss_raw_dist_vec_[i] > param_.detection_distance) {
      // beyond the detection distance will not be considered
      uss_local_arc.is_ignored = true;
    } else {
      uss_local_arc.is_ignored = false;
    }

    uss_local_arc.circle_info.center = uss_local_vertex;

    uss_local_normal_angle = uss_local_normal_angle_vec_[i];

    uss_local_arc.circle_info.radius = uss_raw_dist_vec_[i];

    // p is rear axle center, O is uss vertex, C is center of uss arc
    OC = uss_local_arc.circle_info.radius *
         Eigen::Vector2d(std::cos(uss_local_normal_angle),
                         std::sin(uss_local_normal_angle));

    // pB = pO + OB, pA = pO + OA
    // OC rotates anticlockwise to obtain OB
    // OC rotates clockwise to obtain OA
    uss_local_arc.pB = rot_matrix * OC + uss_local_arc.circle_info.center;

    uss_local_arc.pA =
        rot_matrix.transpose() * OC + uss_local_arc.circle_info.center;

    uss_local_arc_vec_.emplace_back(uss_local_arc);
  }
  return;
}

const bool UssObstacleAvoidance::Preprocess() {
  if (apa_data_ptr_ == nullptr) {
    DEBUG_PRINT("uss apa_data_ptr_ is nullptr!");
    return false;
  }

  car_motion_info_.steer_angle =
      apa_data_ptr_->measurement_data.steer_wheel_angle;

  const bool trajectory_available =
      planning_output_->trajectory.available &&
      planning_output_->trajectory.trajectory_points_size > 1;

  const bool gear_available =
      planning_output_->gear_command.available &&
      (planning_output_->gear_command.gear_command_value ==
           iflyauto::GEAR_COMMAND_VALUE_DRIVE ||
       planning_output_->gear_command.gear_command_value ==
           iflyauto::GEAR_COMMAND_VALUE_REVERSE);

  if (gear_available == false || trajectory_available == false) {
    return false;
  }

  uint8_t gear_cmd = iflyauto::GEAR_COMMAND_VALUE_NONE;
  if (planning_output_->gear_command.available) {
    gear_cmd = planning_output_->gear_command.gear_command_value;
  }
  car_motion_info_.reverse_flag =
      (gear_cmd == iflyauto::GEAR_COMMAND_VALUE_REVERSE);

  // set uss raw dist data
  uss_raw_dist_vec_.clear();
  uss_raw_dist_vec_.reserve(uss_local_vertex_vec_.size());

  if (apa_param.GetParam().is_uss_dist_from_perception) {
    const auto &uss_dis_info_buf =
        apa_data_ptr_->uss_percept_info_ptr->dis_from_car_to_obj;

    // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    // TODO(xjli32): NOTE
    // uss info is stored in the fifth buf,
    // uss_dis_info_buf size is 4 in c struct definition now, need to
    // confirm interface with xuliu15;
    // disable uss_dis_info temporarily;
    // return false;
    //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    // const auto &uss_dis_info = uss_dis_info_buf[4];
    // if (uss_dis_info.obj_pt_cnt != 12) {
    //   DEBUG_PRINT("uss dis nums = " << uss_dis_info.obj_pt_cnt << " != 12 ");
    //   return false;
    // }
    // raw dist info stored in dis_from_car_to_obj in clockwise direction, the
    // first four are front upa, the middle four are rear upa, the rest are apa.

    //  front uss: uss dis need to be transfered from mm to m. order: fl apa, 4
    //  upa, fr apa
    for (const auto &front_uss_idx :
         apa_param.GetParam().uss_wdis_index_front) {
      uss_raw_dist_vec_.emplace_back(0.001 * uss_dis_info_buf[front_uss_idx]);
    }

    // rear uss: uss dis need to be transfered from mm to m. order: rr apa, 4
    // upa, rl apa
    for (const auto &rear_uss_idx : apa_param.GetParam().uss_wdis_index_back) {
      uss_raw_dist_vec_.emplace_back(0.001 * uss_dis_info_buf[rear_uss_idx]);
    }
  } else {
    // load uss dist from uss wave, m id f
    const auto &upa_dis_info_buf =
        apa_data_ptr_->uss_wave_info_ptr->upa_dis_info_buf;

    const std::vector<int> front_wids_idx_vec = {0, 9, 6, 3, 1, 11};
    const std::vector<int> rear_wids_idx_vec = {0, 1, 3, 6, 9, 11};

    const std::vector<std::vector<int>> wids_idx_vec = {front_wids_idx_vec,
                                                        rear_wids_idx_vec};
    for (size_t i = 0; i < wids_idx_vec.size(); i++) {
      for (size_t j = 0; j < wids_idx_vec[i].size(); j++) {
        const auto idx = wids_idx_vec[i][j];

        uss_raw_dist_vec_.emplace_back(
            upa_dis_info_buf[i].wdis[idx].wdis_value[0]);
      }
    }
  }

  std::cout << "uss_raw_dist = ";
  for (const double dist : uss_raw_dist_vec_) {
    std::cout << dist << " ";
  }
  std::cout << "\n";

  JSON_DEBUG_VECTOR("uss_raw_dist_vec", uss_raw_dist_vec_, 2)

  return true;
}

void UssObstacleAvoidance::CalRemainDist() {
  remain_dist_info_.Reset();
  remain_dist_info_.remain_dist = param_.detection_distance;
  double dist = param_.detection_distance;
  pnc::geometry_lib::Arc uss_local_arc;
  pnc::geometry_lib::Arc car_local_arc;
  pnc::geometry_lib::LineSegment car_local_line;
  Eigen::Vector2d intersection;
  double rot_angle = 0.0;
  for (size_t i = 0; i < uss_local_vertex_vec_.size(); ++i) {
    uss_local_arc = uss_local_arc_vec_[i];
    if (uss_local_arc.is_ignored == true) {
      continue;
    }
    for (size_t j = 0; j < car_local_vertex_vec_.size(); ++j) {
      if (car_motion_info_.car_motion_mode == LINE_MODE) {
        car_local_line = car_local_line_vec_[j];
        if (pnc::geometry_lib::GetArcLineIntersection(
                intersection, uss_local_arc, car_local_line)) {
          dist = (intersection - car_local_line.pA).norm();
        }
      } else if (car_motion_info_.car_motion_mode == ARC_MODE) {
        car_local_arc = car_local_arc_vec_[j];
        if (pnc::geometry_lib::GetTwoArcIntersection(
                intersection, uss_local_arc, car_local_arc)) {
          rot_angle = pnc::geometry_lib::GetAngleFromTwoVec(
              car_local_arc.pA - car_local_arc.circle_info.center,
              intersection - car_local_arc.circle_info.center);

          dist = car_motion_info_.rear_axle_center_turn_radius *
                 std::fabs(rot_angle);
        }
      }

      if (dist < remain_dist_info_.remain_dist) {
        remain_dist_info_.remain_dist = dist;
        remain_dist_info_.uss_index = i;
        remain_dist_info_.car_index = j;
        remain_dist_info_.is_available = true;
      }
    }
  }
  std::cout << "uss_index = " << remain_dist_info_.uss_index
            << "  car_index = " << remain_dist_info_.car_index << std::endl;
}

void UssObstacleAvoidance::Update(
    iflyauto::PlanningOutput *const planning_output,
    const std::shared_ptr<ApaData> apa_data_ptr) {
  // update local_view
  apa_data_ptr_ = apa_data_ptr;

  // update planning output
  planning_output_ = planning_output;

  // set some necessary info, if info is abnormal, quit directly
  if (!Preprocess()) {
    Reset();
    return;
  }

  // update car traj (line or arc)
  if (std::abs(car_motion_info_.steer_angle) <
      pnc::mathlib::Deg2Rad(param_.arc_line_shift_steer_angle_deg)) {
    // car go straight, update car line
    car_motion_info_.car_motion_mode = LINE_MODE;
    car_motion_info_.rear_axle_center_turn_radius = 1.0e4;
    car_motion_info_.turning_center.setZero();

    GenCarLine();
  } else {
    // car turn, update car arc
    car_motion_info_.car_motion_mode = ARC_MODE;
    const auto &front_wheel_angle =
        std::fabs(car_motion_info_.steer_angle / param_.steer_ratio);
    car_motion_info_.rear_axle_center_turn_radius =
        1.0 / (param_.c1 * std::tan(front_wheel_angle));

    car_motion_info_.turning_center.setZero();

    if (car_motion_info_.steer_angle > 0.0) {
      car_motion_info_.turning_center.y() =
          car_motion_info_.rear_axle_center_turn_radius;
    } else {
      car_motion_info_.turning_center.y() =
          -car_motion_info_.rear_axle_center_turn_radius;
    }

    GenCarArc();
  }

  // update uss arc
  GenUssArc();

  // calculate remaining distance
  CalRemainDist();

  // the remaining distance greater than 2m does not need to be considered
  if (remain_dist_info_.remain_dist <= 2.0) {
    remain_dist_info_.is_available = true;
  } else {
    remain_dist_info_.is_available = false;
  }
}

const bool UssObstacleAvoidance::CheckIsDirectlyBehindUss() {
  if (!remain_dist_info_.is_available) {
    return false;
  }
  for (size_t i = 0; i < apa_param.GetParam().uss_directly_behind_index.size();
       ++i) {
    if (remain_dist_info_.uss_index ==
        apa_param.GetParam().uss_directly_behind_index[i]) {
      return true;
    }
  }
  return false;
}

void UssObstacleAvoidance::SetUssRawDist(const double &uss_raw_dist) {
  // set uss raw dist data
  uss_raw_dist_vec_.clear();
  uss_raw_dist_vec_.reserve(uss_local_vertex_vec_.size());

  for (size_t i = 0; i < uss_local_vertex_vec_.size(); ++i) {
    uss_raw_dist_vec_.emplace_back(uss_raw_dist);
  }
}

void UssObstacleAvoidance::UpdateByPybind() {
  // update car traj (line or arc)
  if (std::fabs(car_motion_info_.steer_angle) <
      pnc::mathlib::Deg2Rad(param_.arc_line_shift_steer_angle_deg)) {
    // car go straight, update car line
    car_motion_info_.car_motion_mode = LINE_MODE;
    car_motion_info_.rear_axle_center_turn_radius = 1.0e4;
    car_motion_info_.turning_center.setZero();

    GenCarLine();
  } else {
    // car turn, update car arc
    car_motion_info_.car_motion_mode = ARC_MODE;
    const auto &front_wheel_angle =
        std::fabs(car_motion_info_.steer_angle / param_.steer_ratio);
    car_motion_info_.rear_axle_center_turn_radius =
        1.0 / (param_.c1 * std::tan(front_wheel_angle));

    car_motion_info_.turning_center.setZero();
    if (car_motion_info_.steer_angle > 0) {
      car_motion_info_.turning_center.y() =
          car_motion_info_.rear_axle_center_turn_radius;
    } else {
      car_motion_info_.turning_center.y() =
          -car_motion_info_.rear_axle_center_turn_radius;
    }

    GenCarArc();
  }

  // update uss arc
  GenUssArc();

  // calculate remaining distance
  CalRemainDist();

  // the remaining distance greater than 2m does not need to be considered
  if (remain_dist_info_.remain_dist > 2.0) {
    remain_dist_info_.is_available = false;
  }
}
}  // namespace apa_planner
}  // namespace planning