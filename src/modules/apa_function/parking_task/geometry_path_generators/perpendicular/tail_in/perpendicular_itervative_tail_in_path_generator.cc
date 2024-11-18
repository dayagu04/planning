#include <cstddef>
#include <cstdint>
#include <limits>
#include <utility>

#include "apa_param_config.h"
#include "geometry_math.h"
#include "ifly_time.h"
#include "log_glog.h"
#include "perpendicular_tail_in_path_generator.h"
#include "reeds_shepp_interface.h"

namespace planning {
namespace apa_planner {

static const double kMaxArcLength = 12.68;
static const double kMinArcLength = 0.0168;
static const double kMinSingleGearPathLength = 0.35;
static const bool kUseEDTColDet = true;

const bool PerpendicularTailInPathGenerator::NewUpdatePathPlan() {
  // preprocess
  Preprocess();

  // reset output
  output_.Reset();

  // prepare plan, only for first plan
  if (input_.is_replan_first && !input_.is_replan_dynamic) {
    calc_params_.first_multi_plan = true;
    if (!NewPreparePathPlan()) {
      return false;
    }
  }

  if ((input_.is_replan_first || input_.is_replan_second) &&
      calc_params_.pre_plan_case != PrePlanCase::EGO_POSE &&
      !input_.is_replan_dynamic) {
    calc_params_.first_multi_plan = true;
    NewPreparePathSecondPlan();
  }

  if (MultiAdjustPathPlan(input_.ego_pose, input_.ref_gear,
                          PlanRequest::OPTIMAL_PATH)) {
    calc_params_.first_multi_plan = false;
    return true;
  }

  return false;
}

const bool PerpendicularTailInPathGenerator::NewPreparePathPlan() {
  ILOG_INFO << "\n ---enter prepare plan---";

  if (collision_detector_ptr_ == nullptr) {
    std::cout << "collision_detector_ptr_ is nullptr\n";
    return false;
  }

  // first check ego pose if collision
  if (!calc_params_.is_searching_stage) {
    collision_detector_ptr_->SetParam(
        CollisionDetector::Paramters(calc_params_.strict_car_lat_inflation));
    if (collision_detector_ptr_->IsObstacleInCar(input_.ego_pose)) {
      ILOG_INFO << "ego pose has obs, force quit PreparePathPlan, fail";
      return false;
    }
  }

  ILOG_INFO << "first prepare init pos = " << input_.ego_pose.pos.transpose()
            << "  heading = " << input_.ego_pose.heading * kRad2Deg;

  const double pre_start_time = IflyTime::Now_ms();
  bool find_mid_pt = false;
  std::vector<std::pair<geometry_lib::GeometryPath, geometry_lib::GeometryPath>>
      pair_geometry_path_vec;

  if (NewPrepareSinglePathPlan(input_.ego_pose, pair_geometry_path_vec)) {
    // if is in searching stage, directly quit
    if (calc_params_.is_searching_stage) {
      return true;
    }

    // if is in parking stage, choose a better path
    // How to choose a better path, temporarily reverse, shorter and one step
    // path
    int better_index = -1;
    double min_cost = std::numeric_limits<double>::infinity();

    // 优先一把进 随后dubins换挡最少 长度最短
    const double gear_change_cost = 12.0;
    const double unit_length_cost = 1.0;
    const double steer_change_cost = 6.0;
    uint8_t gear_change_count = 0;
    uint8_t steer_change_count = 0;
    double cost = 0.0;
    double length = 0.0;
    for (size_t i = 0; i < pair_geometry_path_vec.size(); ++i) {
      const geometry_lib::GeometryPath& dubins_geometry_path =
          pair_geometry_path_vec[i].first;
      const geometry_lib::GeometryPath& rough_geometry_path =
          pair_geometry_path_vec[i].second;

      cost = 0.0;

      gear_change_count = dubins_geometry_path.gear_change_count +
                          rough_geometry_path.gear_change_count;

      if (dubins_geometry_path.last_gear != geometry_lib::SEG_GEAR_INVALID &&
          dubins_geometry_path.last_gear != rough_geometry_path.cur_gear) {
        gear_change_count += 1;
      }

      cost += gear_change_count * gear_change_cost;

      if (!apa_param.GetParam().actual_mono_plan_enable &&
          gear_change_count < 2) {
        // ILOG_INFO << "mono plan is not allowed";
        cost += 200.0;
      }

      length =
          dubins_geometry_path.total_length + rough_geometry_path.total_length;

      cost += length * unit_length_cost;

      steer_change_count = dubins_geometry_path.steer_change_count +
                           rough_geometry_path.steer_change_count;

      if (dubins_geometry_path.last_steer != geometry_lib::SEG_STEER_INVALID &&
          dubins_geometry_path.last_steer != rough_geometry_path.last_steer) {
        steer_change_count += 1;
      }

      cost += steer_change_count * steer_change_cost;

      if (cost < min_cost) {
        better_index = static_cast<int>(i);
        min_cost = cost;
      }
    }

    if (better_index == -1) {
      ILOG_INFO << "no optimal_dubins_geometry_path, quit";

      output_.Reset();

      return false;
    }

    geometry_lib::GeometryPath optimal_dubins_geometry_path =
        pair_geometry_path_vec[better_index].first;

    geometry_lib::GeometryPath optimal_rough_geometry_path =
        pair_geometry_path_vec[better_index].first;

    if (optimal_dubins_geometry_path.path_count < 1) {
      ILOG_INFO << "use ego pose to multi_adjust plan";
      optimal_rough_geometry_path.PrintInfo();
      calc_params_.pre_plan_case = PrePlanCase::EGO_POSE;
      return true;
    } else {
      ILOG_INFO << "use mid point to multi_adjust plan";
      optimal_dubins_geometry_path.PrintInfo();
      calc_params_.pre_plan_case = PrePlanCase::MID_POINT;
    }

    optimal_dubins_geometry_path.PrintInfo();

    for (const auto& path_seg : optimal_dubins_geometry_path.path_segment_vec) {
      output_.path_available = true;
      if (path_seg.seg_gear == optimal_dubins_geometry_path.cur_gear) {
        output_.path_segment_vec.emplace_back(path_seg);
        output_.steer_vec.emplace_back(path_seg.seg_steer);
        output_.gear_cmd_vec.emplace_back(path_seg.seg_gear);
        output_.length += path_seg.Getlength();
      }
    }

    // save safe_circle_tang_pt
    calc_params_.safe_circle_tang_pt = optimal_dubins_geometry_path.end_pose;

    ILOG_INFO << "prepare path plan consume time = "
              << IflyTime::Now_ms() - pre_start_time << "ms";

    if (output_.path_segment_vec.size() > 0) {
      geometry_lib::PathSegment last_seg = output_.path_segment_vec.back();
      if (geometry_lib::CheckTwoPoseIsSame(last_seg.GetEndPose(),
                                           calc_params_.safe_circle_tang_pt) &&
          last_seg.seg_gear == geometry_lib::SEG_GEAR_REVERSE) {
        // directly multi_adjust to target line
      } else {
        // 延长一条直线 方便控制跟踪以及后续规划
        geometry_lib::PathSegment line_seg;
        geometry_lib::CalLineFromPt(last_seg.seg_gear,
                                    apa_param.GetParam().insert_line_after_arc,
                                    last_seg.GetEndPose(), line_seg);
        TrimPathByObs(line_seg, calc_params_.strict_car_lat_inflation,
                      calc_params_.strict_col_lon_safe_dist);
        if (line_seg.Getlength() > 1e-2) {
          output_.path_segment_vec.emplace_back(line_seg);
          output_.steer_vec.emplace_back(line_seg.seg_steer);
          output_.gear_cmd_vec.emplace_back(line_seg.seg_gear);
          output_.length += line_seg.Getlength();
          ILOG_INFO << "first prepare insert line success";
        } else {
          ILOG_INFO << "first prepare insert line fail";
        }
      }

      output_.current_gear = output_.gear_cmd_vec.front();
      input_.ego_pose = output_.path_segment_vec.back().GetEndPose();
      calc_params_.first_path_gear = output_.current_gear;
      ILOG_INFO << "first prepare target pos = "
                << input_.ego_pose.pos.transpose()
                << "  heading = " << input_.ego_pose.heading * kRad2Deg
                << "  safe_circle_tang_pt pos = "
                << calc_params_.safe_circle_tang_pt.pos.transpose()
                << " heading = "
                << calc_params_.safe_circle_tang_pt.heading * kRad2Deg
                << "  path length = " << output_.length;

      return true;
    }
  }

  output_.Reset();

  return false;
}

const bool PerpendicularTailInPathGenerator::NewPrepareSinglePathPlan(
    const pnc::geometry_lib::PathPoint& cur_pose,
    std::vector<
        std::pair<geometry_lib::GeometryPath, geometry_lib::GeometryPath>>&
        pair_geometry_path_vec) {
  ILOG_INFO << "enter single prepare plan";
  const double pre_start_time = IflyTime::Now_ms();

  const double slot_side_sgn = calc_params_.slot_side_sgn;
  const double slot_angle = input_.origin_pt_0_heading;  // 0 30 45
  const double sin_slot_angle = input_.sin_angle;  // sin(90) sin(60) sin(45)
  const double slot_x = ((input_.pt_0 + input_.pt_1) * 0.5).x();

  std::vector<double> x_offset_vec;
  std::vector<double> heading_offset_vec;
  x_offset_vec.reserve(5);
  heading_offset_vec.reserve(40);

  double max_heading =
      std::min(90.0 - slot_angle,
               apa_param.GetParam().prepare_line_max_heading_offset_slot_deg);
  max_heading = slot_side_sgn * (90.0 - slot_angle - max_heading) * kDeg2Rad;

  double min_heading =
      apa_param.GetParam().prepare_line_min_heading_offset_slot_deg;
  min_heading = slot_side_sgn * (90.0 - slot_angle - min_heading) * kDeg2Rad;

  const double dheading =
      apa_param.GetParam().prepare_line_dheading_offset_slot_deg * kDeg2Rad;

  double heading = max_heading;
  while (slot_side_sgn * heading < slot_side_sgn * min_heading) {
    heading_offset_vec.emplace_back(heading);
    heading += slot_side_sgn * dheading;
  }

  double min_x = slot_x + (apa_param.GetParam().max_car_width * 0.5 +
                           calc_params_.strict_car_lat_inflation + 0.05) /
                              sin_slot_angle;

  min_x =
      std::max(min_x, cur_pose.pos.x() +
                          apa_param.GetParam().prepare_line_min_x_offset_slot /
                              sin_slot_angle);

  const double max_x =
      cur_pose.pos.x() +
      apa_param.GetParam().prepare_line_max_x_offset_slot / sin_slot_angle;

  const double dx =
      apa_param.GetParam().prepare_line_dx_offset_slot / sin_slot_angle;
  double x = min_x;
  while (x < max_x) {
    x_offset_vec.emplace_back(x);
    x += dx;
  }

  std::vector<geometry_lib::LineSegment> prepare_line_vec;
  prepare_line_vec.reserve(heading_offset_vec.size() * x_offset_vec.size() + 1);
  for (const double heading : heading_offset_vec) {
    for (const double x : x_offset_vec) {
      prepare_line_vec.emplace_back(
          geometry_lib::BuildLineSegByPose(Eigen::Vector2d(x, 0.0), heading));
    }
  }
  prepare_line_vec.emplace_back(
      geometry_lib::BuildLineSegByPose(cur_pose.pos, cur_pose.heading));

  ILOG_INFO << "prepare_line_vec size = " << prepare_line_vec.size();

  // calculating these target points is actually not time-consuming, can set up
  // more target points
  const uint8_t count = 5;
  const double ds = 0.5;
  const double virtual_1r_arc_length = (count - 1) * ds + 0.068;
  const uint8_t gear = geometry_lib::SEG_GEAR_REVERSE;
  const uint8_t steer = (calc_params_.is_left_side)
                            ? geometry_lib::SEG_STEER_LEFT
                            : geometry_lib::SEG_STEER_RIGHT;

  std::vector<std::vector<std::vector<geometry_lib::PathPoint>>> tang_pose_vec;
  tang_pose_vec.clear();
  tang_pose_vec.reserve(prepare_line_vec.size() * count * count * 2);

  int number = 0;
  std::vector<std::vector<geometry_lib::PathPoint>> inner_tang_pose_vec;
  std::vector<geometry_lib::PathPoint> inner_inner_tang_pose_vec;
  Eigen::Vector2d line_tangent_vec;
  Eigen::Vector2d line_normal_vec;
  bool cal_tang_pt_success = false;
  for (const geometry_lib::LineSegment& line : prepare_line_vec) {
    // cal pre line tangent vec and normal vec
    line_tangent_vec = geometry_lib::GenHeadingVec(line.heading);

    if (line_tangent_vec.y() > 0.0) {
      line_normal_vec << -line_tangent_vec.y(), line_tangent_vec.x();
    } else {
      line_normal_vec << line_tangent_vec.y(), -line_tangent_vec.x();
    }

    // line_normal_vec(line_tangent_vec.y(), -line_tangent_vec.x());
    // // sure line_normal_vec towards downward along the x axis.
    // if (line_normal_vec.x() > 0.0) {
    //   line_normal_vec = -1.0 * line_normal_vec;
    // }

    calc_params_.prepare_line = line;

    calc_params_.pre_line_tangent_vec = line_tangent_vec;
    calc_params_.pre_line_normal_vec = line_normal_vec;

    cal_tang_pt_success = false;

    for (uint8_t i = 0; i < 2; ++i) {
      geometry_lib::PathPoint pose;

      pose.heading = line.heading;
      if (i == 0 && apa_param.GetParam().actual_mono_plan_enable &&
          MonoPreparePlan(pose.pos)) {
        cal_tang_pt_success = true;
      }
      if (i == 1 && MultiPreparePlan(pose.pos)) {
        cal_tang_pt_success = true;
      }

      geometry_lib::PathSegment arc_seg;
      if (cal_tang_pt_success) {
        geometry_lib::CalArcFromPt(gear, steer, virtual_1r_arc_length,
                                   calc_params_.turn_radius, pose, arc_seg);
      }

      inner_tang_pose_vec.clear();
      inner_tang_pose_vec.reserve(count + 1);

      for (uint8_t j = 0; j < count && cal_tang_pt_success; ++j) {
        inner_inner_tang_pose_vec.clear();
        inner_inner_tang_pose_vec.reserve(count + 1);
        geometry_lib::CalPtFromPathSeg(pose, arc_seg, (count - j - 1) * ds);
        geometry_lib::PathPoint temp_pose = pose;
        const Eigen::Vector2d heading_vec =
            geometry_lib::GenHeadingVec(pose.heading);
        for (uint8_t k = 0; k < count; ++k) {
          temp_pose.pos = pose.pos + ds * k * heading_vec;
          inner_inner_tang_pose_vec.emplace_back(temp_pose);
          number++;
        }
        inner_tang_pose_vec.emplace_back(inner_inner_tang_pose_vec);
      }

      if (cal_tang_pt_success) {
        tang_pose_vec.emplace_back(inner_tang_pose_vec);
      }
    }
  }

  ILOG_INFO << "link point size = " << number
            << "\ncal tang pose consume time = "
            << IflyTime::Now_us() - pre_start_time * 1000.0 << "us";

  bool exceed_time_flag = false;
  double max_allow_time = apa_param.GetParam().prepare_single_max_allow_time;
  if (!calc_params_.is_searching_stage) {
    max_allow_time = 999.9;
  }
  pair_geometry_path_vec.clear();
  pair_geometry_path_vec.reserve(number);
  geometry_lib::GeometryPath dubins_geometry_path;
  geometry_lib::GeometryPath rough_geometry_path;
  geometry_lib::PathSegment arc_seg;
  DubinsPlanResult result;
  bool find_all_result = true;

  const size_t max_path_count = 30;
  const uint8_t max_dubins_gear_change_count = 1;

  // 先尝试用自车位置规划一下 看是否能规划成功
  if (RoughMultiAdjustPathPlan(input_.ego_pose, geometry_lib::SEG_GEAR_REVERSE,
                               rough_geometry_path) ||
      RoughMultiAdjustPathPlan(input_.ego_pose, geometry_lib::SEG_GEAR_DRIVE,
                               rough_geometry_path)) {
    ILOG_INFO << "ego pose rough plan success";
    rough_geometry_path.PrintInfo();
    pair_geometry_path_vec.emplace_back(
        std::make_pair(dubins_geometry_path, rough_geometry_path));
    if (rough_geometry_path.gear_change_count < 1) {
      ILOG_INFO << "ego pose rough plan gear change count is 0, no need to "
                   "dubins plan";
      find_all_result = false;
    }
  } else {
    ILOG_INFO << "ego pose rough plan fail";
  }
  uint8_t fewer_gear_change_count = 0;
  calc_params_.tange_pose_vec.clear();
  calc_params_.tange_pose_vec.reserve(number);
  for (size_t i = 0;
       i < tang_pose_vec.size() && find_all_result && !exceed_time_flag; ++i) {
    bool continue_use_same_1arc = true;
    bool reverse_1arc_safe = false;
    const auto& inner_tang_pose_vec = tang_pose_vec[i];
    for (size_t j = 0; j < inner_tang_pose_vec.size() && find_all_result &&
                       !exceed_time_flag && continue_use_same_1arc;
         ++j) {
      const auto& inner_inner_tang_pose_vec = inner_tang_pose_vec[j];

      if (inner_inner_tang_pose_vec.size() < 1) {
        break;
      }

      // when the 1r corresponding to tangpt is safe, all subsequent tangpt
      // are safe
      if (!reverse_1arc_safe) {
        // construct 1r arc and check 1r is safe
        if (geometry_lib::CalArcFromPt(gear, steer,
                                       virtual_1r_arc_length - j * ds,
                                       calc_params_.turn_radius,
                                       inner_inner_tang_pose_vec[0], arc_seg) &&
            IsGeometryPathSafe(geometry_lib::GeometryPath(arc_seg),
                               calc_params_.strict_car_lat_inflation,
                               calc_params_.strict_col_lon_safe_dist)) {
          reverse_1arc_safe = true;
        } else {
          continue;
        }
      }

      for (size_t k = 0;
           k < inner_inner_tang_pose_vec.size() && find_all_result &&
           !exceed_time_flag && continue_use_same_1arc;
           ++k) {
        if (IflyTime::Now_ms() - pre_start_time > max_allow_time) {
          exceed_time_flag = true;
          ILOG_INFO << "try time is out, quit find result.";
          break;
        }

        const auto& tang_pose = inner_inner_tang_pose_vec[k];

        result = NewDubinsPathPlan(
            cur_pose, tang_pose, calc_params_.turn_radius,
            kMinSingleGearPathLength, max_dubins_gear_change_count,
            geometry_lib::SEG_GEAR_DRIVE, true, dubins_geometry_path);

        if (result == DubinsPlanResult::NO_PATH) {
          continue;
        }

        if (result == DubinsPlanResult::PATH_COLLISION) {
          break;
        }

        if (result == DubinsPlanResult::SUCCESS) {
          if (RoughMultiAdjustPathPlan(tang_pose,
                                       geometry_lib::SEG_GEAR_REVERSE,
                                       rough_geometry_path)) {
            pair_geometry_path_vec.emplace_back(
                std::make_pair(dubins_geometry_path, rough_geometry_path));

            uint8_t gear_change_count = rough_geometry_path.gear_change_count +
                                        dubins_geometry_path.gear_change_count;
            if (dubins_geometry_path.last_gear !=
                rough_geometry_path.cur_gear) {
              gear_change_count++;
            }

            if (gear_change_count < 3) {
              continue_use_same_1arc = false;
            }

            if (calc_params_.is_searching_stage ||
                pair_geometry_path_vec.size() > max_path_count ||
                fewer_gear_change_count > 3) {
              find_all_result = false;
            }

            calc_params_.tange_pose_vec.emplace_back(tang_pose);
          }
          continue;
        }
      }
    }
  }

  ILOG_INFO << "there arc " << pair_geometry_path_vec.size()
            << " tangpt can be target\n"
            << "prepare single path plan consume time = "
            << IflyTime::Now_ms() - pre_start_time << "ms"
            << "  dubins_plan_time = " << calc_params_.dubins_plan_time << "ms"
            << "  rough_plan_time= " << calc_params_.rough_plan_time << "ms"
            << "  col_det_time = " << calc_params_.col_det_time << "ms";

  calc_params_.col_det_time = 0.0;

  return pair_geometry_path_vec.size() > 0;
}

const bool PerpendicularTailInPathGenerator::NewPreparePathSecondPlan() {
  if (calc_params_.pre_plan_case == PrePlanCase::EGO_POSE) {
    return false;
  }

  const uint8_t ref_gear =
      geometry_lib::ReverseGear(calc_params_.first_path_gear);

  const geometry_lib::PathPoint start_pose = input_.ego_pose;
  const geometry_lib::PathPoint target_pose = calc_params_.safe_circle_tang_pt;

  ILOG_INFO << "second prepare init pos = " << input_.ego_pose.pos.transpose()
            << "  heading = " << input_.ego_pose.heading * kRad2Deg;

  if (geometry_lib::CheckTwoPoseIsSame(start_pose, target_pose, 0.0468,
                                       0.168 * kRad2Deg)) {
    ILOG_INFO
        << "second prepare, pose is same, directly use pose to multi_adjust";
    return false;
  }

  double min_length = kMinSingleGearPathLength;
  if (ref_gear == geometry_lib::SEG_GEAR_REVERSE) {
    min_length = 0.0168;
  }

  geometry_lib::GeometryPath geometry_path;
  // no col det, ensure successful planning, relying on real-time braking
  DubinsPlanResult result =
      NewDubinsPathPlan(start_pose, target_pose, calc_params_.turn_radius,
                        min_length, 0, ref_gear, false, geometry_path);

  // 如果失败， 分为两种，参考挡位是前进挡还是倒退档
  // 如果是倒退档， 不再尝试，直接以当前位置接着后续规划
  // 如果是前进挡，是否可以再找其他的目标点相连，
  // 但是这样目标点怎么确定也是个问题， 能不能把原先成功规划的目标点都试一次

  if (result != DubinsPlanResult::SUCCESS &&
      ref_gear == geometry_lib::SEG_GEAR_DRIVE) {
    ILOG_INFO << "try dubins to connect first tang pose fail, and gear is "
                 "drive, try connect any tang_pose";
    for (const auto& tang_pose : calc_params_.tange_pose_vec) {
      result =
          NewDubinsPathPlan(start_pose, tang_pose, calc_params_.turn_radius,
                            min_length, 0, ref_gear, false, geometry_path);
      if (result == DubinsPlanResult::SUCCESS) {
        break;
      }
    }
  }

  if (result == DubinsPlanResult::SUCCESS && geometry_path.path_count > 0 &&
      geometry_path.cur_gear == ref_gear) {
    geometry_path.PrintInfo();
    output_.path_available = true;
    for (const pnc::geometry_lib::PathSegment& path_seg :
         geometry_path.path_segment_vec) {
      output_.path_segment_vec.emplace_back(path_seg);
      output_.length += path_seg.Getlength();
      output_.gear_cmd_vec.emplace_back(path_seg.seg_gear);
      output_.steer_vec.emplace_back(path_seg.seg_steer);
    }
    // 如果当前挡位是前进挡 延长一条直线便于控制跟踪
    if (ref_gear == geometry_lib::SEG_GEAR_DRIVE) {
      geometry_lib::PathSegment line_seg;
      geometry_lib::CalLineFromPt(ref_gear,
                                  apa_param.GetParam().insert_line_after_arc,
                                  geometry_path.end_pose, line_seg);
      TrimPathByObs(line_seg,
                    apa_param.GetParam().car_lat_inflation_strict + 0.01,
                    apa_param.GetParam().col_obs_safe_dist_strict + 0.01);
      PrintSegmentInfo(line_seg);
      if (line_seg.Getlength() > 1e-2) {
        output_.path_segment_vec.emplace_back(line_seg);
        output_.steer_vec.emplace_back(line_seg.seg_steer);
        output_.gear_cmd_vec.emplace_back(line_seg.seg_gear);
        output_.length += line_seg.Getlength();
        ILOG_INFO << "second prepare, insert line success";
      } else {
        ILOG_INFO << "second prepare, insert line fail";
      }
    }

    output_.current_gear = output_.gear_cmd_vec.front();
    input_.ego_pose = output_.path_segment_vec.back().GetEndPose();
    ILOG_INFO << "second prepare, from first prepare pos to safe circle tange "
                 "success";

    ILOG_INFO << "second prepare target pos = "
              << input_.ego_pose.pos.transpose()
              << "  heading = " << input_.ego_pose.heading * kRad2Deg;

    return true;
  } else {
    ILOG_INFO
        << "second prepare, from first prepare pos to safe circle tange fail";
    return false;
  }

  return false;
}

const PerpendicularTailInPathGenerator::DubinsPlanResult
PerpendicularTailInPathGenerator::NewDubinsPathPlan(
    const pnc::geometry_lib::PathPoint& start_pose,
    const pnc::geometry_lib::PathPoint& target_pose, const double turn_radius,
    const double min_length, const uint8_t max_gear_change_count,
    const uint8_t ref_gear, const bool need_col_det,
    geometry_lib::GeometryPath& geometry_path) {
  const double time = IflyTime::Now_ms();

  dubins_lib::DubinsLibrary::Input input(start_pose.pos, target_pose.pos,
                                         start_pose.heading,
                                         target_pose.heading, turn_radius);

  dubins_planner_.SetInput(input);

  std::vector<dubins_lib::DubinsLibrary::Output> dubins_output_vec =
      dubins_planner_.Update();

  if (dubins_output_vec.empty()) {
    return DubinsPlanResult::NO_PATH;
  }

  // 筛选出满足挡位的output
  std::vector<dubins_lib::DubinsLibrary::Output> temp_output_vec;
  temp_output_vec.reserve(dubins_output_vec.size());

  for (const auto& output : dubins_output_vec) {
    bool gear_condition = (output.gear_change_count <= max_gear_change_count);

    bool length_conditon = false;
    while (true) {
      length_conditon =
          (output.gear_change_count == 0 && output.length > min_length + 1e-3);

      if (!length_conditon && output.gear_change_count == 1) {
        // 如果当前挡位是倒挡  即是由倒挡到前进挡的切换，
        // 那么倒挡长度和前进挡长度都需要满足长度要求
        double first_length = 0.0;
        double second_length = 0.0;
        for (size_t i = 0; i < output.gear_cmd_vec.size(); ++i) {
          if (output.gear_cmd_vec[i] != geometry_lib::SEG_GEAR_INVALID) {
            if (output.gear_cmd_vec[i] == output.current_gear_cmd) {
              first_length += output.length_vec[i];
            } else {
              second_length += output.length_vec[i];
            }
          }
        }
        if (output.current_gear_cmd == geometry_lib::SEG_GEAR_REVERSE &&
            first_length > min_length + 1e-3 && first_length < 16.8 &&
            second_length > min_length + 1e-3 && second_length < 16.8) {
          length_conditon = true;
        }
        if (output.current_gear_cmd == geometry_lib::SEG_GEAR_DRIVE &&
            first_length > min_length + 1e-3) {
          length_conditon = true;
        }
      }

      if (!length_conditon && output.gear_change_count == 2) {
        // D->R->D   R->D>R
        const size_t index =
            (output.gear_cmd_vec.back() == geometry_lib::SEG_GEAR_REVERSE) ? 2
                                                                           : 3;
        for (size_t i = 0; i < index; ++i) {
          if (output.length_vec[i] > min_length + 1e-3) {
            length_conditon = true;
          } else {
            length_conditon = false;
          }
        }
      }

      break;
    }

    bool heading_condition = false;
    while (true) {
      heading_condition = (output.gear_change_count == 0);

      if (!heading_condition && output.gear_change_count == 1) {
        double heading = 0.0;
        if (output.gear_change_index == 1) {
          heading = output.line_BC.heading;
        } else if (output.gear_change_index == 2) {
          heading = output.arc_CD.headingA;
        }
        // The heading at the gear shift should not be oriented towards
        // the direction where the heading error increases as much as possible
        heading_condition =
            ((std::fabs(heading) - std::fabs(start_pose.heading)) * kRad2Deg <
             apa_param.GetParam().prepare_max_reverse_heading_err);
      }

      break;
    }

    if (gear_condition && length_conditon && heading_condition) {
      temp_output_vec.emplace_back(output);
    }
  }

  if (temp_output_vec.empty()) {
    return DubinsPlanResult::NO_PATH;
  }

  double min_cost = std::numeric_limits<double>::infinity();
  int optimal_index = -1;
  for (int i = 0; i < temp_output_vec.size(); ++i) {
    double cost = 0.0;

    cost += temp_output_vec[i].gear_change_count * 12.0;

    cost += temp_output_vec[i].length * 1.0;

    cost += (temp_output_vec[i].current_gear_cmd == ref_gear) ? 0.0 : 15.0;

    if (cost < min_cost) {
      min_cost = cost;
      optimal_index = i;
    }
  }

  if (optimal_index == -1) {
    return DubinsPlanResult::NO_PATH;
  }

  dubins_lib::DubinsLibrary::Output final_output =
      temp_output_vec[optimal_index];

  std::vector<geometry_lib::PathSegment> path_seg_vec;
  path_seg_vec.reserve(3);

  pnc::geometry_lib::PathSegment path_seg;
  // set arc AB
  if (final_output.gear_cmd_vec[0] != pnc::geometry_lib::SEG_GEAR_INVALID) {
    path_seg.seg_gear = final_output.gear_cmd_vec[0];
    path_seg.seg_type = pnc::geometry_lib::SEG_TYPE_ARC;
    path_seg.arc_seg = final_output.arc_AB;
    path_seg.seg_steer = geometry_lib::CalArcSteer(path_seg.arc_seg);
    path_seg_vec.emplace_back(path_seg);
  }
  // set line BC
  if (final_output.gear_cmd_vec[1] != pnc::geometry_lib::SEG_GEAR_INVALID) {
    path_seg.seg_gear = final_output.gear_cmd_vec[1];
    path_seg.seg_type = pnc::geometry_lib::SEG_TYPE_LINE;
    path_seg.line_seg = final_output.line_BC;
    path_seg.seg_steer = geometry_lib::SEG_STEER_STRAIGHT;
    path_seg_vec.emplace_back(path_seg);
  }
  // set arc CD
  if (final_output.gear_cmd_vec[2] != pnc::geometry_lib::SEG_GEAR_INVALID) {
    path_seg.seg_gear = final_output.gear_cmd_vec[2];
    path_seg.seg_type = pnc::geometry_lib::SEG_TYPE_ARC;
    path_seg.arc_seg = final_output.arc_CD;
    path_seg.seg_steer = geometry_lib::CalArcSteer(path_seg.arc_seg);
    path_seg_vec.emplace_back(path_seg);
  }

  if (path_seg_vec.empty()) {
    return DubinsPlanResult::NO_PATH;
  }

  geometry_path.SetPath(path_seg_vec);

  calc_params_.dubins_plan_time += IflyTime::Now_ms() - time;

  if (need_col_det) {
    if (geometry_path.gear_change_count < 1) {
      if (!IsGeometryPathSafe(geometry_path,
                              calc_params_.strict_car_lat_inflation,
                              calc_params_.strict_col_lon_safe_dist)) {
        geometry_path.Reset();
        return DubinsPlanResult::PATH_COLLISION;
      }
    } else {
      for (size_t i = 0; i < path_seg_vec.size(); ++i) {
        path_seg = path_seg_vec[i];
        if (TrimPathByObs(path_seg, calc_params_.strict_car_lat_inflation,
                          calc_params_.strict_col_lon_safe_dist,
                          false) != PathColDetRes::NORMAL) {
          geometry_path.Reset();
          return DubinsPlanResult::PATH_COLLISION;
        }
      }
    }
  }

  return DubinsPlanResult::SUCCESS;
}

const PerpendicularTailInPathGenerator::DubinsPlanResult
PerpendicularTailInPathGenerator::RSPathPlan(
    const pnc::geometry_lib::PathPoint& start_pose,
    const pnc::geometry_lib::PathPoint& target_pose, const double turn_radius,
    const double min_length, const uint8_t max_gear_change_count,
    const bool need_col_det, geometry_lib::GeometryPath& geometry_path) {
  RSPathInterface rs;
  RSPath path;
  bool is_connected_to_goal = false;
  Pose2D start(start_pose.pos.x(), start_pose.pos.y(), start_pose.heading);
  Pose2D end(target_pose.pos.x(), target_pose.pos.y(), target_pose.heading);
  RSPathRequestType request_type = RSPathRequestType::none;
  rs.GeneShortestRSPath(&path, &is_connected_to_goal, &start, &end, turn_radius,
                        true, request_type, 100.0);
  if (!is_connected_to_goal || path.size < 1) {
    return DubinsPlanResult::NO_PATH;
  }

  // check gear
  if (path.gear_change_number > max_gear_change_count) {
    return DubinsPlanResult::NO_PATH;
  }

  // check total length
  if (path.total_length < kMinSingleGearPathLength + 1e-3) {
    return DubinsPlanResult::NO_PATH;
  }

  std::vector<double> length_vec;
  // check every gear length
  double cur_length = std::fabs(path.paths[0].length);
  length_vec.emplace_back(cur_length);
  AstarPathGear cur_gear = path.paths[0].gear;
  for (int i = 1; i < path.size; ++i) {
    if (path.paths[i].gear == cur_gear) {
      cur_length += std::fabs(path.paths[i].length);
      length_vec.back() = cur_length;
    } else {
      cur_gear = path.paths[i].gear;
      cur_length = std::fabs(path.paths[i].length);
      length_vec.emplace_back(cur_length);
    }
  }

  for (size_t i = 0; i < length_vec.size(); ++i) {
    if (length_vec[i] < kMinSingleGearPathLength + 1e-3) {
      return DubinsPlanResult::NO_PATH;
    }
  }

  std::vector<geometry_lib::PathSegment> path_seg_vec;
  for (size_t i = 0; i < path.size; ++i) {
    RSPathSegment rs_path_seg = path.paths[i];
    const uint8_t gear = (rs_path_seg.gear == AstarPathGear::drive)
                             ? geometry_lib::SEG_GEAR_DRIVE
                             : geometry_lib::SEG_GEAR_REVERSE;
    uint8_t steer = 0;
    if (rs_path_seg.steer == RSPathSteer::RS_STRAIGHT) {
      steer = geometry_lib::SEG_STEER_STRAIGHT;
    } else if (rs_path_seg.steer == RSPathSteer::RS_LEFT) {
      steer = geometry_lib::SEG_STEER_LEFT;
    } else if (rs_path_seg.steer == RSPathSteer::RS_RIGHT) {
      steer = geometry_lib::SEG_STEER_RIGHT;
    }
    geometry_lib::PathPoint start_pose(
        Eigen::Vector2d(rs_path_seg.points[0].x, rs_path_seg.points[0].y),
        rs_path_seg.points[0].theta);
    geometry_lib::PathSegment path_seg;

    if (steer == geometry_lib::SEG_STEER_STRAIGHT) {
      if (geometry_lib::CalLineFromPt(gear, rs_path_seg.length, start_pose,
                                      path_seg)) {
        path_seg_vec.emplace_back(path_seg);
      }
    } else {
      if (geometry_lib::CalArcFromPt(gear, steer, rs_path_seg.length,
                                     turn_radius, start_pose, path_seg)) {
        path_seg_vec.emplace_back(path_seg);
      }
    }
  }

  geometry_path.SetPath(path_seg_vec);

  if (need_col_det) {
    for (size_t i = 0; i < path_seg_vec.size(); ++i) {
      geometry_lib::PathSegment path_seg = path_seg_vec[i];
      if (TrimPathByObs(path_seg, calc_params_.strict_car_lat_inflation,
                        calc_params_.strict_col_lon_safe_dist,
                        false) != PathColDetRes::NORMAL) {
        geometry_path.Reset();
        return DubinsPlanResult::PATH_COLLISION;
      }
    }
  }

  return DubinsPlanResult::SUCCESS;
}

const PerpendicularTailInPathGenerator::PathColDetRes
PerpendicularTailInPathGenerator::TrimPathByObs(
    pnc::geometry_lib::PathSegment& path_seg, const double lat_inflation,
    const double lon_safe_dist, const bool enable_log) {
  const double time = IflyTime::Now_ms();

  // if (path_seg.seg_type != geometry_lib::SEG_TYPE_LINE &&
  //     path_seg.seg_type != geometry_lib::SEG_TYPE_ARC) {
  //   return PathColDetRes::INVALID;
  // }

  CollisionDetector::CollisionResult col_res;
  if (kUseEDTColDet) {
    if (path_seg.seg_type == geometry_lib::SEG_TYPE_LINE &&
        path_seg.Getlength() > 1.68) {
      // geometric detection may be more time-saving for straight lines and
      // longer paths
      col_res = collision_detector_ptr_->UpdateByObsMap(path_seg, lat_inflation,
                                                        lon_safe_dist);
    } else if (lat_inflation < 0.168) {
      // Because there is an error in the lateral detection of EDT col det, it
      // would be danger when the buffer is small The closest distance between
      // the obstacle and the car, if it is relatively small, use precise
      // solution
      col_res = collision_detector_ptr_->UpdateByEDT(path_seg, lat_inflation,
                                                     lon_safe_dist, true);
      // 目前更加保守 没撞才需要二次精度检查 撞了就当撞了
      // 其实撞了也可能没撞 所以看需不需要更激进
      const bool radical_flag = false;
      // 7厘米是edt碰撞检测的最大横向误差
      if (!col_res.collision_flag || radical_flag) {
        if (col_res.obs2car_dist < 0.071) {
          col_res = collision_detector_ptr_->UpdateByObsMap(
              path_seg, lat_inflation, lon_safe_dist);
        }
      }

    } else {
      col_res = collision_detector_ptr_->UpdateByEDT(path_seg, lat_inflation,
                                                     lon_safe_dist, false);
    }
  }

  else {
    col_res = collision_detector_ptr_->UpdateByObsMap(path_seg, lat_inflation,
                                                      lon_safe_dist);
  }

  calc_params_.col_det_time += IflyTime::Now_ms() - time;

  const double remain_car_dist = col_res.remain_car_dist;
  const double remain_obs_dist = col_res.remain_obstacle_dist;
  // the dist that the car can go
  const double safe_remain_dist = col_res.remain_dist;

  ILOG_INFO_IF(enable_log) << "remain_car_dist = " << remain_car_dist
                           << "  remain_obs_dist = " << remain_obs_dist
                           << "  safe_remain_dist = " << safe_remain_dist;

  if (safe_remain_dist < 0.016) {
    path_seg.collision_flag = true;
    geometry_lib::CompletePathSegInfo(path_seg, 1e-3);
    ILOG_INFO_IF(enable_log) << "this path is invalid";
    return PathColDetRes::INVALID;
  }

  // if you only hit a little bit, consider it safe, because it has lon safe
  // dist
  if (safe_remain_dist + 0.016 > remain_car_dist) {
    path_seg.collision_flag = false;
    ILOG_INFO_IF(enable_log) << "this path is normal";
    return PathColDetRes::NORMAL;
  }

  // the path would col, but can trim it to keep the path safe
  geometry_lib::CompletePathSegInfo(path_seg, safe_remain_dist);

  ILOG_INFO_IF(enable_log) << "this path is shorten";
  path_seg.collision_flag = true;
  return PathColDetRes::SHORTEN;
}

const bool PerpendicularTailInPathGenerator::RoughMultiAdjustPathPlan(
    const geometry_lib::PathPoint& pose, const uint8_t ref_gear,
    geometry_lib::GeometryPath& ahead_path, const bool enable_log) {
  // ILOG_INFO_IF(enable_log) << " --- enter RoughMultiAdjustPathPlan --- ";

  // plan a successful path to exit, suitable for the search stage
  uint8_t single_ref_gear = ref_gear;
  geometry_lib::PathPoint single_cur_pose = pose;
  std::vector<geometry_lib::PathPoint> cur_pose_vec{single_cur_pose};

  // const double time_start = IflyTime::Now_ms();

  std::vector<geometry_lib::GeometryPath> geometry_path_vec;
  std::vector<geometry_lib::GeometryPath> success_geometry_path_vec;
  std::vector<geometry_lib::GeometryPath> temp_geometry_path_vec;
  geometry_lib::GeometryPath geometry_path;
  geometry_lib::GeometryPath geometry_path_copy;

  for (size_t i = 0; i < 8 && success_geometry_path_vec.size() < 1; ++i) {
    // ILOG_INFO_IF(enable_log) << i << "th try RoughMultiAdjustPathPlan ";

    double lat_buffer = apa_param.GetParam().car_lat_inflation_normal;
    double lon_buffer = apa_param.GetParam().col_obs_safe_dist_normal;

    if (calc_params_.first_multi_plan && i == 0) {
      lat_buffer = apa_param.GetParam().car_lat_inflation_strict;
      lon_buffer = apa_param.GetParam().col_obs_safe_dist_strict;
    }

    for (const geometry_lib::PathPoint& cur_pose : cur_pose_vec) {
      if (CalOccupiedRatio(cur_pose) < 1e-3 &&
          single_ref_gear == geometry_lib::SEG_GEAR_DRIVE) {
        lat_buffer = apa_param.GetParam().car_lat_inflation_strict;
        lon_buffer = apa_param.GetParam().col_obs_safe_dist_strict;
      }

      SingleMultiAdjustPathPlan(cur_pose, single_ref_gear, lat_buffer,
                                lon_buffer, temp_geometry_path_vec, enable_log);

      geometry_path_copy.Reset();
      if (geometry_path_vec.size() > 0) {
        geometry_path_copy = geometry_path_vec.front();
        geometry_path_vec.erase(geometry_path_vec.begin());
      } else {
        // only protect, when the geometry_path_vec is empty, the cur_pose
        // should equal to single_cur_pose
        if (!geometry_lib::CheckTwoPoseIsSame(cur_pose, single_cur_pose)) {
          break;
        }
        // when is single cur pose, directly try ego pose to one line plan,
        // just seeking one more solution, only i=0, first plan
        ILOG_INFO_IF(enable_log)
            << "use single_cur_pose to one line path plan, single_cur_pose = "
            << single_cur_pose.pos.transpose() << "  "
            << single_cur_pose.heading * kRad2Deg;
        if (OneLinePathPlan(single_cur_pose, single_ref_gear,
                            apa_param.GetParam().car_lat_inflation_normal,
                            apa_param.GetParam().col_obs_safe_dist_normal, 0.0,
                            geometry_path, enable_log)) {
          if (geometry_path.cur_gear == geometry_lib::SEG_GEAR_REVERSE) {
            ILOG_INFO_IF(enable_log)
                << "one line path success and gear is reverse, add it to "
                   "success geometry path vec";
            success_geometry_path_vec.emplace_back(geometry_path);
            break;
          } else {
            geometry_path_vec.emplace_back(geometry_path);
          }
        }
      }

      for (const geometry_lib::GeometryPath& temp_geometry_path :
           temp_geometry_path_vec) {
        geometry_path = geometry_path_copy;

        geometry_path.AddPath(temp_geometry_path.path_segment_vec);

        geometry_lib::GeometryPath one_line_geometry_path;
        double last_seg_reverse_length = 0.0;
        if (temp_geometry_path.cur_gear == geometry_lib::SEG_GEAR_REVERSE) {
          last_seg_reverse_length = temp_geometry_path.total_length;
        }
        ILOG_INFO_IF(enable_log)
            << "continue use single multi_adjust plan end pose to one "
               "line path plan, _pose = "
            << geometry_path.end_pose.pos.transpose() << "  "
            << geometry_path.end_pose.heading * kRad2Deg;
        if (OneLinePathPlan(geometry_path.end_pose, single_ref_gear,
                            apa_param.GetParam().car_lat_inflation_normal,
                            apa_param.GetParam().col_obs_safe_dist_normal,
                            last_seg_reverse_length, one_line_geometry_path,
                            enable_log)) {
          geometry_path.AddPath(one_line_geometry_path.path_segment_vec);
          if (geometry_path.cur_gear_length > kMinSingleGearPathLength) {
            if (one_line_geometry_path.cur_gear ==
                geometry_lib::SEG_GEAR_REVERSE) {
              ILOG_INFO_IF(enable_log)
                  << "one line path success and gear is reverse, add it to "
                     "success geometry path vec";
              success_geometry_path_vec.emplace_back(geometry_path);
              break;
            } else {
              geometry_path_vec.emplace_back(geometry_path);
            }
          }

        } else {
          if (geometry_path.cur_gear_length > kMinSingleGearPathLength) {
            geometry_path_vec.emplace_back(geometry_path);
          }
        }
      }

      ILOG_INFO_IF(enable_log)
          << "geometry_path_vec_size = " << geometry_path_vec.size()
          << "  temp_geometry_path_vec = " << temp_geometry_path_vec.size()
          << "  success_geometry_path_vec = "
          << success_geometry_path_vec.size();

      if (success_geometry_path_vec.size() > 0) {
        break;
      }
    }

    // If the 1R path is stuck on the inside or blocked by obstacles in the
    // channel, directly quit
    bool reverse_1arc_safe = true;
    if (calc_params_.first_multi_plan &&
        calc_params_.pre_plan_case != PrePlanCase::EGO_POSE && i == 0 &&
        success_geometry_path_vec.size() < 1) {
      for (const geometry_lib::GeometryPath& geometry_path :
           geometry_path_vec) {
        // if 1r path col, also start pose and end pose is on the same side of
        // the target line, should enter this logic
        if (geometry_path.collide_flag &&
            (geometry_path.end_pose.pos.y() + calc_params_.target_line.pA.y()) *
                    (geometry_path.start_pose.pos.y() +
                     calc_params_.target_line.pA.y()) >
                0.168 &&
            std::fabs(geometry_path.end_pose.heading * kRad2Deg) > 6.8) {
          reverse_1arc_safe = false;
          break;
        }
      }
    }

    if (!reverse_1arc_safe || success_geometry_path_vec.size() > 0 ||
        geometry_path_vec.empty()) {
      break;
    }

    cur_pose_vec.clear();

    for (const geometry_lib::GeometryPath& geometry_path : geometry_path_vec) {
      cur_pose_vec.emplace_back(geometry_path.end_pose);
    }

    single_ref_gear = geometry_lib::ReverseGear(single_ref_gear);
  }

  // ILOG_INFO_IF(enable_log) << "RoughMultiAdjustPathPlan consume time = "
  //                          << IflyTime::Now_ms() - time_start
  //                          << "ms  col consume time = "
  //                          << calc_params_.col_det_time << "ms";

  // calc_params_.rough_plan_time += IflyTime::Now_ms() - time_start;

  if (success_geometry_path_vec.empty()) {
    ILOG_INFO_IF(enable_log)
        << "there is no success plan path to target line\n";
    return false;
  } else {
    ILOG_INFO_IF(enable_log)
        << "there are success plan path to target line, num = "
        << success_geometry_path_vec.size();
    ahead_path = success_geometry_path_vec.front();
    return true;
  }
}

const bool PerpendicularTailInPathGenerator::OneStepMultiAdjustPathPlan(
    const geometry_lib::PathPoint& pose, const uint8_t ref_gear,
    geometry_lib::GeometryPath& ahead_path, const bool enable_log) {
  ILOG_INFO_IF(enable_log) << "--- enter OneStepMultiAdjustPathPlan --- ";

  // only looking at whether the 1R stage can be stored in one step, suitable
  // for selecting the optimal cut-off point during the formal parking pre
  // planning stage

  // const double time_start = IflyTime::Now_ms();

  for (size_t i = 0; i < 1; ++i) {
    ILOG_INFO_IF(enable_log) << i << "th try OneStepMultiAdjustPathPlan";

    const double lat_buffer = apa_param.GetParam().car_lat_inflation_strict;
    const double lon_buffer = apa_param.GetParam().col_obs_safe_dist_strict;

    geometry_lib::GeometryPath geometry_path;
    if (OneLinePathPlan(pose, ref_gear,
                        apa_param.GetParam().car_lat_inflation_normal,
                        apa_param.GetParam().col_obs_safe_dist_normal, 0.0,
                        geometry_path, enable_log)) {
      if (geometry_path.cur_gear == geometry_lib::SEG_GEAR_REVERSE) {
        ILOG_INFO_IF(enable_log) << "one line path success and gear is reverse";
        return true;
      }
    }

    std::vector<geometry_lib::GeometryPath> geometry_path_vec;
    SingleMultiAdjustPathPlan(pose, ref_gear, lat_buffer, lon_buffer,
                              geometry_path_vec, enable_log);

    for (geometry_lib::GeometryPath& geometry_path : geometry_path_vec) {
      double last_seg_reverse_length = 0.0;
      if (geometry_path.cur_gear == geometry_lib::SEG_GEAR_REVERSE) {
        last_seg_reverse_length = geometry_path.total_length;
      }
      ILOG_INFO_IF(enable_log)
          << "continue use single multi_adjust plan end pose to one "
             "line path plan, _pose = "
          << geometry_path.end_pose.pos.transpose() << "  "
          << geometry_path.end_pose.heading * kRad2Deg;
      geometry_lib::GeometryPath one_line_geometry_path;
      if (OneLinePathPlan(geometry_path.end_pose, ref_gear,
                          apa_param.GetParam().car_lat_inflation_normal,
                          apa_param.GetParam().col_obs_safe_dist_normal,
                          last_seg_reverse_length, one_line_geometry_path,
                          enable_log)) {
        geometry_path.AddPath(one_line_geometry_path.path_segment_vec);
        if (geometry_path.cur_gear_length > kMinSingleGearPathLength &&
            one_line_geometry_path.cur_gear == geometry_lib::SEG_GEAR_REVERSE) {
          ILOG_INFO_IF(enable_log)
              << "one line path success and gear is reverse";
          return true;
        }
      }
    }
  }

  // ILOG_INFO_IF(enable_log) << "OneStepMultiAdjustPathPlan consume time = "
  //                          << IflyTime::Now_ms() - time_start
  //                          << "ms  col consume time = "
  //                          << calc_params_.col_det_time << "ms";

  return false;
}

const bool PerpendicularTailInPathGenerator::OptimalMultiAdjustPathPlan(
    const geometry_lib::PathPoint& pose, const uint8_t ref_gear,
    geometry_lib::GeometryPath& optimal_geometry_path) {
  ILOG_INFO << "--- enter OptimalMultiAdjustPathPlan --- ";

  // plan all paths based on the current pose and select the optimal path

  uint8_t single_ref_gear = ref_gear;
  geometry_lib::PathPoint single_cur_pose = pose;
  std::vector<geometry_lib::PathPoint> cur_pose_vec{single_cur_pose};

  const double time_start = IflyTime::Now_ms();

  bool extra_try_flag = false;
  if (calc_params_.first_multi_plan &&
      calc_params_.pre_plan_case == PrePlanCase::EGO_POSE) {
    extra_try_flag = true;
  }
  int i = 0;
  const int max_compensate_line_try_count = 4;
  int compensate_line_try_count = 0;
  const double compensate_line_length_step = 0.5;
  const size_t max_seg_path_count = 666;
  std::vector<geometry_lib::GeometryPath> geometry_path_vec;
  geometry_path_vec.reserve(max_seg_path_count);
  const size_t max_path_count = 5;
  std::vector<geometry_lib::GeometryPath> success_geometry_path_vec;
  success_geometry_path_vec.reserve(max_path_count);
  geometry_lib::GeometryPath geometry_path;
  geometry_lib::GeometryPath geometry_path_copy;

  int max_try_count = 8;
  if (input_.is_replan_dynamic) {
    max_try_count = 1;
  }

  for (i = 0;
       i < max_try_count && success_geometry_path_vec.size() < max_path_count;
       ++i) {
    if (geometry_path_vec.size() >= max_seg_path_count &&
        success_geometry_path_vec.size() > 0) {
      break;
    }

    ILOG_INFO << i
              << "th try OptimalMultiAdjustPathPlan and "
                 "success_geometry_path_vec_size = "
              << success_geometry_path_vec.size();

    double lat_buffer = apa_param.GetParam().car_lat_inflation_normal;
    double lon_buffer = apa_param.GetParam().col_obs_safe_dist_normal;

    if (calc_params_.first_multi_plan && i == 0) {
      lat_buffer = apa_param.GetParam().car_lat_inflation_strict;
      lon_buffer = apa_param.GetParam().col_obs_safe_dist_strict;
    }

    for (const geometry_lib::PathPoint& cur_pose : cur_pose_vec) {
      if (CalOccupiedRatio(cur_pose) < 1e-3 &&
          single_ref_gear == geometry_lib::SEG_GEAR_DRIVE) {
        lat_buffer = apa_param.GetParam().car_lat_inflation_strict;
        lon_buffer = apa_param.GetParam().col_obs_safe_dist_strict;
      }

      std::vector<geometry_lib::GeometryPath> temp_geometry_path_vec;

      SingleMultiAdjustPathPlan(cur_pose, single_ref_gear, lat_buffer,
                                lon_buffer, temp_geometry_path_vec);

      geometry_path_copy.Reset();
      if (geometry_path_vec.size() > 0) {
        geometry_path_copy = geometry_path_vec.front();
        geometry_path_vec.erase(geometry_path_vec.begin());
      } else {
        // only protect, when the geometry_path_vec is empty, the cur_pose
        // should equal to single_cur_pose
        if (!geometry_lib::CheckTwoPoseIsSame(cur_pose, single_cur_pose)) {
          break;
        }
        // when is single cur pose, directly try ego pose to one line plan,
        // just seeking one more solution, only i=0, first plan
        ILOG_INFO
            << "use single_cur_pose to one line path plan, single_cur_pose = "
            << single_cur_pose.pos.transpose() << "  "
            << single_cur_pose.heading * kRad2Deg;
        if (OneLinePathPlan(single_cur_pose, single_ref_gear,
                            apa_param.GetParam().car_lat_inflation_normal,
                            apa_param.GetParam().col_obs_safe_dist_normal, 0.0,
                            geometry_path)) {
          if (geometry_path.cur_gear == geometry_lib::SEG_GEAR_REVERSE) {
            ILOG_INFO << "one line path success and gear is reverse, add it to "
                         "success geometry path vec";
            success_geometry_path_vec.emplace_back(geometry_path);
          } else {
            geometry_path_vec.emplace_back(geometry_path);
          }
        }
      }

      for (const geometry_lib::GeometryPath& temp_geometry_path :
           temp_geometry_path_vec) {
        geometry_path = geometry_path_copy;

        // when single first plan, try insert line to make easy control
        geometry_lib::GeometryPath geometry_path_temp = temp_geometry_path;
        if (i == 0) {
          if (InsertLineInGeometryPath(
                  lat_buffer, lon_buffer, geometry_path_temp.cur_gear,
                  apa_param.GetParam().insert_line_after_arc,
                  geometry_path_temp)) {
            geometry_path.AddPath(geometry_path_temp.path_segment_vec);
          } else {
            geometry_path.AddPath(temp_geometry_path.path_segment_vec);
          }
        } else {
          geometry_path.AddPath(temp_geometry_path.path_segment_vec);
        }

        geometry_lib::GeometryPath one_line_geometry_path;
        double last_seg_reverse_length = 0.0;
        if (geometry_path_temp.cur_gear == geometry_lib::SEG_GEAR_REVERSE) {
          last_seg_reverse_length = geometry_path_temp.total_length;
        }
        ILOG_INFO << "continue use single multi_adjust plan end pose to one "
                     "line path plan, _pose = "
                  << geometry_path.end_pose.pos.transpose() << "  "
                  << geometry_path.end_pose.heading * kRad2Deg;
        if (OneLinePathPlan(geometry_path.end_pose, single_ref_gear,
                            apa_param.GetParam().car_lat_inflation_normal,
                            apa_param.GetParam().col_obs_safe_dist_normal,
                            last_seg_reverse_length, one_line_geometry_path)) {
          geometry_path.AddPath(one_line_geometry_path.path_segment_vec);
          if (geometry_path.cur_gear_length > kMinSingleGearPathLength) {
            if (one_line_geometry_path.cur_gear ==
                geometry_lib::SEG_GEAR_REVERSE) {
              ILOG_INFO
                  << "one line path success and gear is reverse, add it to "
                     "success geometry path vec";
              success_geometry_path_vec.emplace_back(geometry_path);
            } else {
              geometry_path_vec.emplace_back(geometry_path);
            }
          }

        } else {
          if (geometry_path.cur_gear_length > kMinSingleGearPathLength) {
            geometry_path_vec.emplace_back(geometry_path);
          }
        }
      }

      if (success_geometry_path_vec.size() >= max_path_count) {
        break;
      }

      if (geometry_path_vec.size() >= max_seg_path_count &&
          success_geometry_path_vec.size() > 0) {
        break;
      }

      ILOG_INFO << "geometry_path_vec_size = " << geometry_path_vec.size()
                << "  temp_geometry_path_vec = "
                << temp_geometry_path_vec.size()
                << "  success_geometry_path_vec = "
                << success_geometry_path_vec.size();
    }

    // If the 1R path is stuck on the inside or blocked by obstacles in the
    // channel, it is difficult to plan the initial position to the target
    // line. If the previous trajectory is cleared, then continue to try a
    // reverse gear straight line
    if (calc_params_.first_multi_plan &&
        calc_params_.pre_plan_case != PrePlanCase::EGO_POSE && i == 0 &&
        success_geometry_path_vec.size() < 1) {
      for (const geometry_lib::GeometryPath& geometry_path :
           geometry_path_vec) {
        // if 1r path col, also start pose and end pose is on the same side of
        // the target line, should enter this logic
        if (geometry_path.collide_flag &&
            geometry_path.cur_gear == geometry_lib::SEG_GEAR_REVERSE &&
            (geometry_path.end_pose.pos.y() + calc_params_.target_line.pA.y()) *
                    (geometry_path.start_pose.pos.y() +
                     calc_params_.target_line.pA.y()) >
                0.168 &&
            std::fabs(geometry_path.end_pose.heading * kRad2Deg) > 6.8 &&
            compensate_line_try_count < max_compensate_line_try_count) {
          i = -1;
          geometry_path_vec.clear();
          success_geometry_path_vec.clear();
          cur_pose_vec.clear();
          compensate_line_try_count++;
          single_cur_pose.pos =
              pose.pos - compensate_line_length_step *
                             compensate_line_try_count *
                             geometry_lib::GenHeadingVec(pose.heading);
          single_cur_pose.heading = pose.heading;
          cur_pose_vec.emplace_back(single_cur_pose);
          geometry_lib::PathSegment line_seg(
              geometry_lib::SEG_GEAR_REVERSE,
              geometry_lib::LineSegment(pose.pos, single_cur_pose.pos,
                                        pose.heading));
          if (TrimPathByObs(line_seg, lat_buffer, lon_buffer) !=
              PathColDetRes::NORMAL) {
            break;
          }
          geometry_lib::GeometryPath temp_geometry_path(line_seg);
          temp_geometry_path.PrintInfo();
          geometry_path_vec.emplace_back(temp_geometry_path);
          ILOG_INFO
              << "set i is -1, and continuing a reverse gear straight line";
          break;
        }
      }
    }

    if (geometry_path_vec.empty()) {
      if (i == 0 && !extra_try_flag && success_geometry_path_vec.empty()) {
        // try use reverse gear to plan, it is not good, only try
        i = -1;
        extra_try_flag = true;
        single_ref_gear = geometry_lib::ReverseGear(single_ref_gear);
        ILOG_INFO << "the ref gear canot plan any path, use reverse gear to "
                     "try plan";
        continue;
      } else {
        break;
      }
    }

    cur_pose_vec.clear();

    for (const geometry_lib::GeometryPath& geometry_path : geometry_path_vec) {
      cur_pose_vec.emplace_back(geometry_path.end_pose);
    }

    if (i > -1) {
      single_ref_gear = geometry_lib::ReverseGear(single_ref_gear);
    }
  }

  ILOG_INFO << "OptimalMultiAdjustPathPlan consume time = "
            << IflyTime::Now_ms() - time_start
            << "ms  col consume time = " << calc_params_.col_det_time << "ms";

  if (success_geometry_path_vec.empty()) {
    ILOG_INFO << "there is no success plan path to target line\n";
    return false;
  }

  ILOG_INFO << "there are success plan path to target line, num = "
            << success_geometry_path_vec.size();

  // 怎么选呢 挡位？ 路径长度？ 后续需要加上碰撞代价吗？
  int optimal_path_index = -1;
  const double shorter_path_cost = 3.0;
  const double gear_change_cost = 12.0;
  const double unit_length_cost = 1.0;
  const double steer_change_cost = 6.0;
  double min_cost = std::numeric_limits<double>::infinity();
  for (int k = 0; k < success_geometry_path_vec.size(); ++k) {
    // success_geometry_path_vec[k].PrintInfo();
    double cost = 0.0;
    ILOG_INFO << "gear_change_count = "
              << static_cast<int>(
                     success_geometry_path_vec[k].gear_change_count)
              << "  total_length = "
              << success_geometry_path_vec[k].total_length;
    cost += gear_change_cost * success_geometry_path_vec[k].gear_change_count;
    cost += unit_length_cost * success_geometry_path_vec[k].total_length;
    cost += steer_change_cost * success_geometry_path_vec[k].steer_change_count;
    if (success_geometry_path_vec[k].cur_gear_length <
        apa_param.GetParam().min_one_step_path_length) {
      cost += shorter_path_cost;
    }
    if (success_geometry_path_vec[k].cur_gear_length <
        kMinSingleGearPathLength + 1e-3) {
      cost += 100.0;
    }
    if (cost < min_cost) {
      optimal_path_index = k;
      min_cost = cost;
    }
  }

  optimal_geometry_path = success_geometry_path_vec[optimal_path_index];

  if (input_.is_replan_dynamic && optimal_geometry_path.gear_change_count > 0 &&
      optimal_geometry_path.cur_gear != geometry_lib::SEG_GEAR_REVERSE &&
      optimal_geometry_path.last_steer != geometry_lib::SEG_STEER_STRAIGHT) {
    optimal_geometry_path.Reset();
    return false;
  }

  optimal_geometry_path.PrintInfo();

  return true;
}

const bool PerpendicularTailInPathGenerator::MultiAdjustPathPlan(
    const geometry_lib::PathPoint& pose, const uint8_t ref_gear,
    const PlanRequest plan_request) {
  ILOG_INFO << "\n\n --- enter MultiAdjustPathPlan --- ";
  // 根据 plan_request 有三种情况
  // 1. 规划出一条成功路径即退出， 适用于寻库阶段
  // 2. 只看1R阶段 是否能一把入库，适用于正式泊车预规划阶段选择一个最优的切点
  // 3. 根据当前车辆位姿规划出所有路径，选择一条最优路径
  geometry_lib::GeometryPath geometry_path;
  if (plan_request == PlanRequest::ROUGH_PATH) {
    RoughMultiAdjustPathPlan(pose, ref_gear, geometry_path);
  } else if (plan_request == PlanRequest::ONE_STEP_PATH) {
    OneStepMultiAdjustPathPlan(pose, ref_gear, geometry_path);
  } else if (plan_request == PlanRequest::OPTIMAL_PATH) {
    if (calc_params_.pre_plan_case == PrePlanCase::EGO_POSE &&
        calc_params_.first_multi_plan) {
      geometry_lib::GeometryPath geometry_path_r;
      geometry_lib::GeometryPath geometry_path_d;
      if (OptimalMultiAdjustPathPlan(pose, geometry_lib::SEG_GEAR_REVERSE,
                                     geometry_path_r)) {
        geometry_path = geometry_path_r;
      }
      if (OptimalMultiAdjustPathPlan(pose, geometry_lib::SEG_GEAR_DRIVE,
                                     geometry_path_d)) {
        if (geometry_path.path_count == 0 ||
            geometry_path.gear_change_count >
                geometry_path_d.gear_change_count ||
            (geometry_path.gear_change_count ==
                 geometry_path_d.gear_change_count &&
             geometry_path.total_length > geometry_path_d.total_length)) {
          geometry_path = geometry_path_d;
        }
      }
    } else {
      OptimalMultiAdjustPathPlan(pose, ref_gear, geometry_path);
    }
  } else {
    return false;
  }

  if (geometry_path.path_count < 1) {
    return false;
  }

  output_.path_available = true;
  for (size_t i = 0; i < geometry_path.path_count; ++i) {
    output_.gear_cmd_vec.emplace_back(geometry_path.gear_cmd_vec[i]);
    output_.path_segment_vec.emplace_back(geometry_path.path_segment_vec[i]);
    output_.steer_vec.emplace_back(geometry_path.steer_cmd_vec[i]);
  }

  if (!output_.gear_cmd_vec.empty()) {
    output_.current_gear = output_.gear_cmd_vec.front();
  }

  if (CheckReachTargetPose() || output_.path_segment_vec.size() > 0) {
    return true;
  }

  return CheckReachTargetPose();
}

const bool PerpendicularTailInPathGenerator::SingleMultiAdjustPathPlan(
    const geometry_lib::PathPoint& pose, const uint8_t ref_gear,
    const double lat_buffer, const double lon_buffer,
    std::vector<geometry_lib::GeometryPath>& geometry_path_vec,
    const bool enable_log) {
  geometry_path_vec.clear();
  ILOG_INFO_IF(enable_log) << "\n --- enter SingleMultiAdjustPathPlan ---";
  ILOG_INFO_IF(enable_log) << "pos = " << pose.pos.transpose()
                           << "  heading = " << pose.heading * kRad2Deg
                           << "  ref gear = " << static_cast<int>(ref_gear);

  geometry_lib::GeometryPath geometry_path;
  if (OneArcPathPlan(pose, ref_gear, lat_buffer, lon_buffer, geometry_path,
                     enable_log)) {
    geometry_path_vec.emplace_back(geometry_path);
  }

  if (TwoArcPathPlan(pose, ref_gear, lat_buffer, lon_buffer, geometry_path,
                     false, enable_log)) {
    geometry_path_vec.emplace_back(geometry_path);
  }

  if (TwoArcPathPlan(pose, ref_gear, lat_buffer, lon_buffer, geometry_path,
                     true, enable_log)) {
    geometry_path_vec.emplace_back(geometry_path);
  }

  if (LineArcPathPlan(pose, ref_gear, lat_buffer, lon_buffer, geometry_path,
                      true, enable_log)) {
    geometry_path_vec.emplace_back(geometry_path);
  }

  if (LineArcPathPlan(pose, ref_gear, lat_buffer, lon_buffer, geometry_path,
                      false, enable_log)) {
    geometry_path_vec.emplace_back(geometry_path);
  }

  if (AlignAndSTurnPathPlan(pose, ref_gear, lat_buffer, lon_buffer,
                            geometry_path, enable_log)) {
    geometry_path_vec.emplace_back(geometry_path);
  }

  return geometry_path_vec.size() > 0;
}

const bool PerpendicularTailInPathGenerator::OneArcPathPlan(
    const geometry_lib::PathPoint& pose, const uint8_t ref_gear,
    const double lat_buffer, const double lon_buffer,
    geometry_lib::GeometryPath& geometry_path, const bool enable_log) {
  ILOG_INFO_IF(enable_log) << "\n----enter one arc path plan----";
  geometry_path.Reset();
  geometry_lib::Arc arc;
  arc.pA = pose.pos;
  arc.headingA = pose.heading;

  bool success = geometry_lib::CalOneArcWithLineAndGear(
      arc, calc_params_.target_line, ref_gear);

  success = success && arc.length < kMaxArcLength && arc.length > kMinArcLength;

  if (!success) {
    ILOG_INFO_IF(enable_log) << "one arc calc fail\n";
    return false;
  }

  const uint8_t gear = geometry_lib::CalArcGear(arc);
  const uint8_t steer = geometry_lib::CalArcSteer(arc);

  arc.PrintInfo(enable_log);

  success =
      geometry_lib::IsSameGear(gear, ref_gear) &&
      arc.circle_info.radius >
          calc_params_.turn_radius - apa_param.GetParam().target_radius_err &&
      arc.circle_info.radius < apa_param.GetParam().max_one_step_arc_radius;

  success = geometry_lib::IsSameGear(gear, ref_gear);

  if (!success) {
    ILOG_INFO_IF(enable_log) << "one arc gear is err\n";
    return false;
  }
  if (arc.circle_info.radius > apa_param.GetParam().max_one_step_arc_radius) {
    ILOG_INFO_IF(enable_log) << "one arc gear radius is too big, err\n";
    return false;
  }
  // 如果半径较小的时候 尝试用最小半径去规划一下
  if (arc.circle_info.radius <
      calc_params_.turn_radius - apa_param.GetParam().target_radius_err) {
    geometry_lib::Arc current_arc;
    const Eigen::Vector2d current_tang_vec =
        geometry_lib::GenHeadingVec(pose.heading);
    Eigen::Vector2d current_norm_vec;
    if (steer == geometry_lib::SEG_STEER_RIGHT) {
      current_norm_vec << current_tang_vec.y(), -current_tang_vec.x();
    } else if (steer == geometry_lib::SEG_STEER_LEFT) {
      current_norm_vec << -current_tang_vec.y(), current_tang_vec.x();
    }
    const Eigen::Vector2d current_turn_center =
        pose.pos + current_norm_vec * calc_params_.turn_radius;
    current_arc.circle_info.center = current_turn_center;
    current_arc.circle_info.radius = calc_params_.turn_radius;
    current_arc.pA = pose.pos;
    current_arc.headingA = pnc::geometry_lib::NormalizeAngle(pose.heading);
    if (geometry_lib::CalOneArcWithLine(
            current_arc, calc_params_.target_line,
            apa_param.GetParam().target_radius_err) &&
        current_arc.length < kMaxArcLength &&
        current_arc.length > kMinArcLength &&
        geometry_lib::CalArcSteer(current_arc) == steer &&
        geometry_lib::CalArcGear(current_arc) == gear) {
      arc = current_arc;
    } else {
      ILOG_INFO_IF(enable_log) << "one arc radius is too small, and canot use "
                                  "min radius to plan, fail\n";
      return false;
    }
  }

  if (!success) {
    ILOG_INFO_IF(enable_log) << "one arc path plan fail\n";
    return false;
  }

  geometry_lib::PathSegment arc_seg(steer, gear, arc);

  PathColDetRes res =
      TrimPathByObs(arc_seg, lat_buffer, lon_buffer, enable_log);

  if (res == PathColDetRes::INVALID) {
    ILOG_INFO_IF(enable_log) << "one arc path col fail\n";
    return false;
  }

  if (res == PathColDetRes::SHORTEN) {
    ILOG_INFO_IF(enable_log) << "one arc path col shorten\n";
  }

  if (res == PathColDetRes::NORMAL) {
    ILOG_INFO_IF(enable_log) << "one arc path no col\n";
  }

  geometry_path.SetPath(arc_seg);

  if (geometry_path.path_count > 0) {
    ILOG_INFO_IF(enable_log) << "one arc plan has path";
    geometry_path.PrintInfo(enable_log);
    return true;
  } else {
    ILOG_INFO_IF(enable_log) << "one arc plan has no path";
    return false;
  }
}

const bool PerpendicularTailInPathGenerator::LineArcPathPlan(
    const geometry_lib::PathPoint& pose, const uint8_t ref_gear,
    const double lat_buffer, const double lon_buffer,
    geometry_lib::GeometryPath& geometry_path, const bool same_gear,
    const bool enable_log) {
  ILOG_INFO_IF(enable_log) << "\n----enter line " << same_gear
                           << "arc path plan----";
  // if same_gear is true, the line and arc should have same gear, otherwise
  // must opposite gear
  geometry_path.Reset();
  geometry_lib::LineSegment line1 =
      geometry_lib::BuildLineSegByPose(pose.pos, pose.heading);

  geometry_lib::LineSegment line2 = calc_params_.target_line;

  std::vector<Eigen::Vector2d> centers;
  std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d>> tangent_ptss;

  bool success = CalCommonTangentCircleOfTwoLine(
      line1, line2, calc_params_.turn_radius, centers, tangent_ptss);

  if (!success) {
    ILOG_INFO_IF(enable_log) << "Cal line arc plan has no path";
    return false;
  }

  for (size_t i = 0; i < centers.size(); ++i) {
    geometry_lib::LineSegment line(line1.pA, tangent_ptss[i].first,
                                   line1.heading);

    geometry_lib::Arc arc;
    arc.circle_info.center = centers[i];
    arc.circle_info.radius = calc_params_.turn_radius;
    arc.pA = tangent_ptss[i].first;
    arc.headingA = line.heading;
    arc.pB = tangent_ptss[i].second;
    CompleteArcInfo(arc);

    if (std::fabs(geometry_lib::NormalizeAngle(arc.headingB - line2.heading)) *
            kRad2Deg >
        0.168) {
      continue;
    }

    if (line.length < kMinArcLength) {
      continue;
    }

    if (arc.length < kMinArcLength || arc.length > kMaxArcLength) {
      continue;
    }

    const uint8_t line_gear = geometry_lib::CalLineSegGear(line);

    if (!geometry_lib::IsSameGear(ref_gear, line_gear)) {
      continue;
    }

    const uint8_t arc_gear = geometry_lib::CalArcGear(arc);

    if (same_gear) {
      if (!geometry_lib::IsSameGear(ref_gear, arc_gear)) {
        continue;
      }
    } else {
      if (!geometry_lib::IsOppositeGear(ref_gear, arc_gear)) {
        continue;
      }
    }

    const uint8_t arc_steer = geometry_lib::CalArcSteer(arc);

    geometry_lib::PathSegment line_seg(line_gear, line);

    geometry_lib::PathSegment arc_seg(arc_steer, arc_gear, arc);

    PathColDetRes col_res1 =
        TrimPathByObs(line_seg, lat_buffer, lon_buffer, enable_log);

    if (same_gear) {
      if (col_res1 != PathColDetRes::NORMAL) {
        ILOG_INFO_IF(enable_log) << "same gear, line col, quit";
        continue;
      } else {
        ILOG_INFO_IF(enable_log)
            << "same gear, line is safe and then check arc col, add line "
               "to path";
      }

      std::vector<geometry_lib::PathSegment> path_seg_vec{line_seg};

      PathColDetRes col_res2 =
          TrimPathByObs(arc_seg, lat_buffer, lon_buffer, enable_log);

      if (col_res2 == PathColDetRes::INVALID) {
        ILOG_INFO_IF(enable_log) << "same gear, arc invalid, quit";
        continue;
      }

      if (col_res2 == PathColDetRes::NORMAL) {
        ILOG_INFO_IF(enable_log) << "same gear, arc normal, add arc to path";
      }

      if (col_res2 == PathColDetRes::SHORTEN) {
        ILOG_INFO_IF(enable_log) << "same gear, arc shorten, add arc to path";
      }

      path_seg_vec.emplace_back(arc_seg);
      geometry_path.SetPath(path_seg_vec);

      break;
    }

    else {
      if (col_res1 == PathColDetRes::INVALID) {
        ILOG_INFO_IF(enable_log) << "opposite gear, line invalid, quit";
        continue;
      }

      if (col_res1 == PathColDetRes::NORMAL) {
        ILOG_INFO_IF(enable_log)
            << "opposite gear, line normal, then check arc col";
        PathColDetRes col_res2 =
            TrimPathByObs(arc_seg, lat_buffer, lon_buffer, enable_log);

        if (col_res2 == PathColDetRes::NORMAL) {
          ILOG_INFO_IF(enable_log) << "arc col is normal, add line to path";
          geometry_path.SetPath(line_seg);
          break;
        }
        ILOG_INFO_IF(enable_log)
            << "opposite gear, the arc col is not normal, quit";
        continue;

        if (col_res2 == PathColDetRes::NORMAL ||
            col_res2 == PathColDetRes::SHORTEN) {
          ILOG_INFO_IF(enable_log)
              << "arc col is normal or shorten, add line to path";
          std::vector<geometry_lib::PathSegment> path_seg_vec{line_seg,
                                                              arc_seg};
          // geometry_path.SetPath(path_seg_vec);
          geometry_path.SetPath(line_seg);
          break;
        }

        if (col_res2 == PathColDetRes::INVALID) {
          ILOG_INFO_IF(enable_log)
              << "arc col is invalid, then construct line to sure safe";
          if (ConstructReverseVaildPathSeg(line_seg, arc_seg, lat_buffer,
                                           lon_buffer, enable_log)) {
            ILOG_INFO_IF(enable_log)
                << "construct line success, add line to path";
            geometry_path.SetPath(line_seg);
            geometry_path.collide_flag = true;
            break;
          }
        }
      }

      if (col_res1 == PathColDetRes::SHORTEN) {
        ILOG_INFO_IF(enable_log) << "opposite gear, the line can not shorten";
        continue;
        ILOG_INFO_IF(enable_log)
            << "opposite gear, line shorten, then construct line to "
               "sure safe";
        if (ConstructReverseVaildPathSeg(line_seg, arc_seg, lat_buffer,
                                         lon_buffer, enable_log)) {
          ILOG_INFO_IF(enable_log)
              << "construct line success, add line to path";
          geometry_path.SetPath(line_seg);
          break;
        }
      }
    }
  }

  if (geometry_path.path_count > 0) {
    ILOG_INFO_IF(enable_log) << "line arc plan has path";
    geometry_path.PrintInfo(enable_log);
    return true;
  } else {
    ILOG_INFO_IF(enable_log) << "line arc plan has no path";
    return false;
  }
}

const bool PerpendicularTailInPathGenerator::TwoArcPathPlan(
    const geometry_lib::PathPoint& pose, const uint8_t ref_gear,
    const double lat_buffer, const double lon_buffer,
    geometry_lib::GeometryPath& geometry_path, const bool same_gear,
    const bool enable_log) {
  // if same_gear is true, the arc1 and arc2 should have same gear, otherwise
  // must opposite gear
  ILOG_INFO_IF(enable_log) << "\n----enter two " << same_gear
                           << " arc path plan----";
  geometry_path.Reset();

  std::vector<std::pair<geometry_lib::Arc, geometry_lib::Arc>> arc_pair_vec;
  bool success = geometry_lib::CalTwoArcWithLine(
      pose, calc_params_.target_line, calc_params_.turn_radius,
      calc_params_.turn_radius, arc_pair_vec);

  if (!success) {
    ILOG_INFO_IF(enable_log) << "Cal two arc plan has no path";
    return false;
  }

  for (const auto& arc_pair : arc_pair_vec) {
    geometry_lib::Arc arc1 = arc_pair.first;
    geometry_lib::Arc arc2 = arc_pair.second;

    if (std::fabs(geometry_lib::NormalizeAngle(
            arc2.headingB - calc_params_.target_line.heading)) *
            kRad2Deg >
        0.168) {
      continue;
    }

    if (arc1.length < kMinArcLength || arc1.length > kMaxArcLength) {
      continue;
    }

    if (arc2.length < kMinArcLength || arc2.length > kMaxArcLength) {
      continue;
    }

    const uint8_t arc1_gear = geometry_lib::CalArcGear(arc1);
    const uint8_t arc1_steer = geometry_lib::CalArcSteer(arc1);

    if (!geometry_lib::IsSameGear(arc1_gear, ref_gear)) {
      continue;
    }

    const uint8_t arc2_gear = geometry_lib::CalArcGear(arc2);
    const uint8_t arc2_steer = geometry_lib::CalArcSteer(arc2);

    if (same_gear) {
      if (!geometry_lib::IsSameGear(ref_gear, arc2_gear)) {
        continue;
      }
    } else {
      if (!geometry_lib::IsOppositeGear(ref_gear, arc2_gear)) {
        continue;
      }
    }

    if (!geometry_lib::IsOppositeSteer(arc1_steer, arc2_steer)) {
      continue;
    }

    geometry_lib::PathSegment arc1_seg(arc1_steer, arc1_gear, arc1);

    geometry_lib::PathSegment arc2_seg(arc2_steer, arc2_gear, arc2);

    PathColDetRes col_res1 =
        TrimPathByObs(arc1_seg, lat_buffer, lon_buffer, enable_log);

    if (same_gear) {
      if (col_res1 != PathColDetRes::NORMAL) {
        ILOG_INFO_IF(enable_log) << "same gear, arc1 col, quit\n";
        continue;
      } else {
        ILOG_INFO_IF(enable_log)
            << "same gear, arc1 is safe and then check arc col, add arc1 "
               "to path";
      }
      std::vector<geometry_lib::PathSegment> path_seg_vec{arc1_seg};

      PathColDetRes col_res2 =
          TrimPathByObs(arc2_seg, lat_buffer, lon_buffer, enable_log);

      if (col_res2 == PathColDetRes::INVALID) {
        ILOG_INFO_IF(enable_log) << "same gear, arc2 invalid, quit\n";
        continue;
      }

      if (col_res2 == PathColDetRes::NORMAL) {
        ILOG_INFO_IF(enable_log) << "same gear, arc2 normal, add arc2 to path";
      }

      if (col_res2 == PathColDetRes::SHORTEN) {
        ILOG_INFO_IF(enable_log) << "same gear, arc2 shorten, add arc2 to path";
      }

      path_seg_vec.emplace_back(arc2_seg);
      geometry_path.SetPath(path_seg_vec);

      if (col_res2 == PathColDetRes::SHORTEN) {
        geometry_path.collide_flag = true;
      }

      break;
    }

    else {
      if (col_res1 == PathColDetRes::INVALID) {
        ILOG_INFO_IF(enable_log) << "opposite gear, arc1 invalid, quit\n";
        continue;
      }

      if (col_res1 == PathColDetRes::NORMAL) {
        ILOG_INFO_IF(enable_log)
            << "opposite gear, arc1 normal, then check arc2 col\n";
        PathColDetRes col_res2 =
            TrimPathByObs(arc2_seg, lat_buffer, lon_buffer, enable_log);
        if (col_res2 == PathColDetRes::NORMAL ||
            col_res2 == PathColDetRes::SHORTEN) {
          ILOG_INFO_IF(enable_log)
              << "arc2 col is normal or shorten, add arc1 to path";
          std::vector<geometry_lib::PathSegment> path_seg_vec{arc1_seg,
                                                              arc2_seg};
          // geometry_path.SetPath(path_seg_vec);
          geometry_path.SetPath(arc1_seg);
          break;
        }

        if (col_res2 == PathColDetRes::INVALID) {
          ILOG_INFO_IF(enable_log)
              << "arc2 col is invalid, then construct arc1 to sure safe";
          if (ConstructReverseVaildPathSeg(arc1_seg, arc2_seg, lat_buffer,
                                           lon_buffer, enable_log)) {
            ILOG_INFO_IF(enable_log)
                << "construct arc1 success, add arc1 to path";
            geometry_path.SetPath(arc1_seg);
            geometry_path.collide_flag = true;
            break;
          }
        }
      }

      if (col_res1 == PathColDetRes::SHORTEN) {
        ILOG_INFO_IF(enable_log)
            << "arc1 col is shorten then construct arc1 to sure safe";
        if (ConstructReverseVaildPathSeg(arc1_seg, arc2_seg, lat_buffer,
                                         lon_buffer, enable_log)) {
          ILOG_INFO_IF(enable_log)
              << "construct arc1 success, add arc1 to path";
          geometry_path.SetPath(arc1_seg);
          break;
        }
      }
    }
  }

  if (geometry_path.path_count > 0) {
    ILOG_INFO_IF(enable_log) << "two arc plan has path";
    geometry_path.PrintInfo(enable_log);
    return true;
  } else {
    ILOG_INFO_IF(enable_log) << "two arc plan has no path";
    return false;
  }
}

const bool PerpendicularTailInPathGenerator::AlignAndSTurnPathPlan(
    const geometry_lib::PathPoint& pose, const uint8_t ref_gear,
    const double lat_buffer, const double lon_buffer,
    geometry_lib::GeometryPath& geometry_path, const bool enable_log) {
  ILOG_INFO_IF(enable_log) << "\n----enter align and s_turn path plan----";
  geometry_path.Reset();
  bool success = false;
  const double tar_heading = calc_params_.target_line.heading;
  double cur_heading = pose.heading;
  bool align_arc_vaild = false;
  if (std::fabs(geometry_lib::NormalizeAngle(cur_heading - tar_heading)) <
      0.1 * kDeg2Rad) {
    ILOG_INFO_IF(enable_log) << "force cur heading equal to tar_heading";
    cur_heading = tar_heading;
  } else if (std::fabs(geometry_lib::NormalizeAngle(
                 cur_heading - tar_heading)) > 31.68 * kDeg2Rad) {
    ILOG_INFO_IF(enable_log)
        << "cur heading is not unsuited AlignAndSTurnPathPlan";
    return false;
  } else {
    geometry_lib::Arc arc;
    arc.pA = pose.pos;
    arc.headingA = cur_heading;
    arc.SetRadius(calc_params_.turn_radius);
    success = geometry_lib::CalOneArcWithTargetHeadingAndGear(arc, ref_gear,
                                                              tar_heading);
    const uint8_t steer = pnc::geometry_lib::CalArcSteer(arc);
    const uint8_t gear = pnc::geometry_lib::CalArcGear(arc);
    success = (success && (gear == ref_gear));
    if (!success) {
      ILOG_INFO_IF(enable_log) << "align body false";
      return false;
    }

    geometry_lib::PathSegment arc_seg(steer, gear, arc);
    if (TrimPathByObs(arc_seg, lat_buffer, lon_buffer, enable_log) !=
        PathColDetRes::NORMAL) {
      ILOG_INFO_IF(enable_log) << "align body path col, quit";
      return false;
    }

    ILOG_INFO_IF(enable_log) << "align body path success";

    align_arc_vaild = true;
    geometry_path.SetPath(geometry_lib::PathSegment(steer, gear, arc));
  }

  const double radius = calc_params_.turn_radius;
  const std::vector<double> radius_vec{
      1.0 * radius, 1.25 * radius, 1.5 * radius, 1.75 * radius,
      2.0 * radius, 2.25 * radius, 2.5 * radius};

  geometry_lib::Arc arc_s_1;
  if (align_arc_vaild) {
    arc_s_1.headingA = geometry_path.end_pose.heading;
    arc_s_1.pA = geometry_path.end_pose.pos;
  } else {
    arc_s_1.headingA = tar_heading;
    arc_s_1.pA = pose.pos;
  }

  const double align_lat_err =
      std::fabs(arc_s_1.pA.y() - calc_params_.target_line.pA.y());

  if (align_lat_err < 0.0168) {
    ILOG_INFO_IF(enable_log)
        << "pA is particularly close to the target line, no need to "
           "use s turn, try use align result continue to one line plan";
    geometry_path.PrintInfo(enable_log);
    return true;
  }

  // cal current line
  // const geometry_lib::LineSegment current_line =
  //     pnc::geometry_lib::BuildLineSegByPose(arc_s_1.pA, arc_s_1.headingA);

  // const double steer_change_ratio = 1.0;
  // cal target line according to steer_change_ratio
  // steer_change_ratio = 0.0 -> target_line = current_line
  // steer_change_ratio = 1.0 -> target_line = calc_params_.target_line
  pnc::geometry_lib::LineSegment target_line = calc_params_.target_line;
  //// temp: default use slot target_line, no need for a middle line
  /// transition
  // target_line.SetPoints((1.0 - steer_change_ratio) * current_line.pA +
  //                           steer_change_ratio *
  //                           calc_params_.target_line.pA,
  //                       (1.0 - steer_change_ratio) * current_line.pB +
  //                           steer_change_ratio *
  //                           calc_params_.target_line.pB);

  geometry_lib::Arc arc_s_2;
  arc_s_2.pB = target_line.pA;
  arc_s_2.headingB = target_line.heading;

  for (const double radius : radius_vec) {
    arc_s_1.SetRadius(radius);
    arc_s_2.SetRadius(radius);

    success =
        pnc::geometry_lib::CalTwoArcWithSameHeading(arc_s_1, arc_s_2, ref_gear);

    success =
        success &&
        (arc_s_1.length > kMinArcLength && arc_s_1.length < kMaxArcLength) &&
        (arc_s_2.length > kMinArcLength && arc_s_2.length < kMaxArcLength);

    if (!success) {
      continue;
    }

    const uint8_t steer1 = geometry_lib::CalArcSteer(arc_s_1);
    const uint8_t gear1 = geometry_lib::CalArcGear(arc_s_1);

    const uint8_t steer2 = geometry_lib::CalArcSteer(arc_s_2);
    const uint8_t gear2 = geometry_lib::CalArcGear(arc_s_2);

    success = success && (gear1 == ref_gear) && (gear2 == ref_gear);

    geometry_lib::PathSegment arc_seg1(steer1, gear1, arc_s_1);
    geometry_lib::PathSegment arc_seg2(steer2, gear2, arc_s_2);

    success = success &&
              (TrimPathByObs(arc_seg1, lat_buffer, lon_buffer, enable_log) ==
               PathColDetRes::NORMAL) &&
              (TrimPathByObs(arc_seg2, lat_buffer, lon_buffer, enable_log) ==
               PathColDetRes::NORMAL);

    if (success) {
      geometry_path.AddPath(
          std::vector<geometry_lib::PathSegment>{arc_seg1, arc_seg2});
      break;
    }
  }

  if (success) {
    ILOG_INFO_IF(enable_log) << "AlignAndSTurnPathPlan has path";
    geometry_path.PrintInfo(enable_log);
  } else {
    const double lat_err = (ref_gear == geometry_lib::SEG_GEAR_DRIVE)
                               ? apa_param.GetParam().target_pos_err
                               : apa_param.GetParam().target_pos_err * 0.5;

    if (align_lat_err < lat_err - 1e-3) {
      ILOG_INFO_IF(enable_log)
          << "pA is close to the target line, directly "
             "try use align result continue to one line plan";
      geometry_path.PrintInfo(enable_log);
      success = true;
    } else {
      ILOG_INFO_IF(enable_log) << "AlignAndSTurnPathPlan has no path";
    }
  }

  return success;
}

const bool PerpendicularTailInPathGenerator::OneLinePathPlan(
    const geometry_lib::PathPoint& pose, const uint8_t ref_gear,
    const double lat_buffer, const double lon_buffer,
    const double last_seg_reverse_length,
    geometry_lib::GeometryPath& geometry_path, const bool enable_log) {
  geometry_path.Reset();
  ILOG_INFO_IF(enable_log) << "\n----enter one line path plan----";
  const uint8_t line_gear = (pose.pos.x() > calc_params_.target_line.pA.x())
                                ? geometry_lib::SEG_GEAR_REVERSE
                                : geometry_lib::SEG_GEAR_DRIVE;

  const double lat_err = (line_gear == geometry_lib::SEG_GEAR_DRIVE)
                             ? apa_param.GetParam().target_pos_err
                             : apa_param.GetParam().target_pos_err * 0.5;

  const double heading_err =
      (line_gear == geometry_lib::SEG_GEAR_DRIVE)
          ? apa_param.GetParam().target_heading_err * kDeg2Rad
          : apa_param.GetParam().target_heading_err * kDeg2Rad * 0.5;

  if (geometry_lib::IsPoseOnLine(pose, calc_params_.target_line, lat_err,
                                 heading_err)) {
    const Eigen::Vector2d start_pos = pose.pos;
    if (line_gear == geometry_lib::SEG_GEAR_DRIVE) {
      // try to make the car body go out as much as possible so that it is
      // easier to straighten it in the future
      const Eigen::Vector2d end_pos(4.68, start_pos.y());
      geometry_lib::LineSegment line(start_pos, end_pos, pose.heading);
      const uint8_t seg_gear = geometry_lib::CalLineSegGear(line);

      if (geometry_lib::IsSameGear(seg_gear, line_gear)) {
        geometry_lib::PathSegment line_seg(seg_gear, line);

        TrimPathByObs(line_seg, lat_buffer, lon_buffer, enable_log);

        if (line_seg.Getlength() > kMinSingleGearPathLength + 1e-3 &&
            line_seg.GetEndPos().x() > calc_params_.target_line.pA.x() +
                                           kMinSingleGearPathLength + 1e-3) {
          geometry_path.SetPath(line_seg);
        }
      }
    } else {
      Eigen::Vector2d end_pos(calc_params_.target_line.pA.x(), start_pos.y());
      geometry_lib::LineSegment line(start_pos, end_pos, pose.heading);
      const uint8_t seg_gear = geometry_lib::CalLineSegGear(line);
      if (geometry_lib::IsSameGear(seg_gear, line_gear)) {
        geometry_lib::PathSegment line_seg(seg_gear, line);

        PathColDetRes res =
            TrimPathByObs(line_seg, lat_buffer, lon_buffer, enable_log);

        if (ref_gear == geometry_lib::SEG_GEAR_DRIVE) {
          if (line_seg.Getlength() < kMinSingleGearPathLength + 1e-3) {
            // 强制往前规划使长度满足要求
            end_pos = start_pos +
                      (kMinSingleGearPathLength + 1e-3) * line.heading_vec;
            end_pos.x() = std::max(end_pos.x(), 4.68);
            line_seg.seg_gear = ref_gear;
            line_seg.line_seg.SetPoints(start_pos, end_pos);
            res = TrimPathByObs(line_seg, lat_buffer, lon_buffer, enable_log);
            if (line_seg.Getlength() > kMinSingleGearPathLength + 1e-3) {
              geometry_path.SetPath(line_seg);
            }
          } else {
            geometry_path.SetPath(line_seg);
          }
        } else {
          if (line_seg.Getlength() + last_seg_reverse_length >
              kMinSingleGearPathLength + 1e-3) {
            geometry_path.SetPath(line_seg);
          }
        }
      }
    }
  }

  if (geometry_path.path_count > 0) {
    ILOG_INFO_IF(enable_log) << "one line plan has path";
    geometry_path.PrintInfo(enable_log);
    return true;
  } else {
    ILOG_INFO_IF(enable_log) << "one line plan has no path";
    return false;
  }
}

const bool PerpendicularTailInPathGenerator::InsertLineInGeometryPath(
    const double lat_buffer, const double lon_buffer, const uint8_t ref_gear,
    const double insert_length, geometry_lib::GeometryPath& geometry_path,
    const bool enable_log) {
  if (geometry_path.path_segment_vec.size() < 1 ||
      !geometry_lib::IsSameGear(ref_gear, geometry_path.cur_gear) ||
      geometry_path.collide_flag || geometry_path.gear_change_count > 1) {
    ILOG_INFO_IF(enable_log) << "insert line condition can not meet need 1";
    return false;
  }

  if (geometry_path.total_length >
          apa_param.GetParam().min_one_step_path_length &&
      geometry_path.last_steer == geometry_lib::SEG_STEER_STRAIGHT) {
    ILOG_INFO_IF(enable_log) << "insert line condition can not meet need 2";
    return false;
  }

  // 1. try connecting a straight line to make the control easier to track
  // 2. avoid path lengths that are too short and difficult to control
  geometry_lib::LineSegment line;
  line.pA = geometry_path.end_pose.pos;
  line.heading = geometry_path.end_pose.heading;
  line.heading_vec = geometry_lib::GenHeadingVec(line.heading);
  line.length =
      std::max(insert_length, apa_param.GetParam().min_one_step_path_length -
                                  geometry_path.total_length);

  const int sign = (ref_gear == geometry_lib::SEG_GEAR_DRIVE) ? 1.0 : -1.0;
  line.pB = line.pA + sign * line.length * line.heading_vec;
  geometry_lib::PathSegment line_seg(ref_gear, line);
  PathColDetRes res =
      TrimPathByObs(line_seg, lat_buffer, lon_buffer, enable_log);
  if (res == PathColDetRes::NORMAL || res == PathColDetRes::SHORTEN) {
    geometry_path.AddPath(line_seg);
    ILOG_INFO_IF(enable_log) << "insert line success";
    PrintSegmentInfo(line_seg);
    return true;
  }

  ILOG_INFO_IF(enable_log) << "insert line fail";
  return false;
}

const bool PerpendicularTailInPathGenerator::ConstructReverseVaildPathSeg(
    geometry_lib::PathSegment& seg1, geometry_lib::PathSegment& seg2,
    const double lat_buffer, const double lon_buffer, const bool enable_log) {
  const double trim_path_step = 0.068;
  const double origin_seg1_length = seg1.Getlength();
  size_t i = 0;
  for (i = 0; i < 100 && seg1.Getlength() > kMinSingleGearPathLength + 1e-3;
       ++i) {
    geometry_lib::CompletePathSeg(seg1, origin_seg1_length - i * trim_path_step,
                                  true);
    seg2.SetStartPose(seg1.GetEndPos(), seg1.GetEndHeading());
    geometry_lib::CompletePathSegInfo(seg2, 1.68, seg2.seg_gear,
                                      seg2.seg_steer);

    PathColDetRes col_res2 =
        TrimPathByObs(seg2, lat_buffer, lon_buffer, enable_log);

    if (col_res2 == PathColDetRes::NORMAL ||
        col_res2 == PathColDetRes::SHORTEN) {
      ILOG_INFO_IF(enable_log) << "re construct path seg success, trim path = "
                               << i * trim_path_step;
      return true;
    }
  }

  return false;
}

const bool PerpendicularTailInPathGenerator::ItervativeUpdatePb(
    const Input& input,
    const std::shared_ptr<CollisionDetector>& collision_detector_ptr) {
  input_ = input;
  collision_detector_ptr_ = collision_detector_ptr;
  Preprocess();
  calc_params_.is_searching_stage = false;
  input_.is_simulation = true;
  calc_params_.first_multi_plan = true;
  calc_params_.adjust_fail_count = 0;
  apa_param.SetPram().actual_mono_plan_enable = true;

  if (CheckReachTargetPose()) {
    ILOG_INFO << "init pose is already at target pose";
    return true;
  }

  if (NewPreparePathPlan()) {
    ILOG_INFO << "first prepare plan success";
    // return true;
  } else {
    ILOG_INFO << "first prepare plan fail, quit";
    // return CalTurnAroundPose();
    return false;
  }
  if (CheckReachTargetPose()) {
    ILOG_INFO << "first prepare plan to target pose";
    return true;
  }

  if (NewPreparePathSecondPlan()) {
    ILOG_INFO << "second prepare plan success";
  } else {
    ILOG_INFO << "second prepare  plan fail";
  }
  if (CheckReachTargetPose()) {
    ILOG_INFO << "second prepare plan to target pose";
    return true;
  }

  MultiAdjustPathPlan(input_.ego_pose, geometry_lib::SEG_GEAR_REVERSE,
                      PlanRequest::OPTIMAL_PATH);

  PrintOutputSegmentsInfo();

  return CheckReachTargetPose() || output_.path_segment_vec.size() > 0;
}

}  // namespace apa_planner
}  // namespace planning