#include "perpendicular_tail_in_path_generator.h"

#include <cstddef>
#include <cstdint>
#include <limits>
#include <utility>
#include <vector>

#include "apa_param_config.h"
#include "common_platform_type_soc.h"
#include "debug_info_log.h"
#include "geometry_math.h"
#include "geometry_path_generator.h"
#include "ifly_time.h"
#include "log_glog.h"
#include "reeds_shepp_interface.h"

namespace planning {
namespace apa_planner {

static const double kMaxArcLength = 12.68;
static const double kMinArcLength = 0.0168;
static const double kMinSingleGearPathLength = 0.25;

static const size_t kMaxPerpenParkInSegmentNums = 15;

void PerpendicularTailInPathGenerator::Reset() {
  output_.Reset();
  output_.path_segment_vec.reserve(kMaxPerpenParkInSegmentNums);
  output_.gear_cmd_vec.reserve(kMaxPerpenParkInSegmentNums);

  calc_params_.Reset();
}

void PerpendicularTailInPathGenerator::Preprocess() {
  // calc_params_.Reset();
  calc_params_.multi_plan = false;
  calc_params_.turn_radius = apa_param.GetParam().min_turn_radius + 0.08;
  calc_params_.is_searching_stage = input_.is_searching_stage;
  calc_params_.col_det_time = 0.0;
  calc_params_.dubins_plan_time = 0.0;
  calc_params_.rough_plan_time = 0.0;
  calc_params_.strict_car_lat_inflation =
      apa_param.GetParam().car_lat_inflation_strict + 0.068;
  calc_params_.strict_col_lon_safe_dist =
      apa_param.GetParam().col_obs_safe_dist_strict + 0.068;

  if (input_.ego_info_under_slot.slot_side == geometry_lib::SLOT_SIDE_LEFT) {
    calc_params_.is_left_side = true;
    calc_params_.slot_side_sgn = 1.0;
  } else {
    calc_params_.is_left_side = false;
    calc_params_.slot_side_sgn = -1.0;
  }

  if (input_.force_mid_process_plan != 0 && input_.can_first_plan_again) {
    input_.is_replan_first = false;
    input_.is_replan_second = false;
    input_.is_replan_dynamic = false;
    calc_params_.pre_plan_case = PrePlanCase::MID_POINT;
    if (input_.force_mid_process_plan == 2) {
      calc_params_.first_multi_plan = false;
    }
  }

  // reset output
  output_.Reset();

  // target line
  calc_params_.target_line = geometry_lib::BuildLineSegByPose(
      input_.ego_info_under_slot.target_pose.pos,
      input_.ego_info_under_slot.target_pose.heading);
}

const bool PerpendicularTailInPathGenerator::Update() {
  if (collision_detector_interface_ptr_ == nullptr) {
    return false;
  }
  ILOG_INFO << "--------perpendicular path planner --------";

  // check start ego pose if collision
  if (collision_detector_interface_ptr_->GetGJKCollisionDetectorPtr()
          ->Update(
              std::vector<geometry_lib::PathPoint>{
                  input_.ego_info_under_slot.cur_pose},
              apa_param.GetParam().car_lat_inflation_normal, 0.3,
              GJKColDetRequest())
          .col_flag) {
    ILOG_INFO << "ego pose has obs, force quit PathPlan, fail";
    return false;
  }

  // preprocess
  Preprocess();

  // prepare plan, only for first plan
  if (input_.is_replan_first && !input_.is_replan_dynamic) {
    calc_params_.first_multi_plan = true;
    if (calc_params_.is_searching_stage) {
      return PreparePathPlan();
    } else if (!PreparePathPlan()) {
      return false;
    }
  }

  if ((input_.is_replan_first || input_.is_replan_second) &&
      calc_params_.pre_plan_case != PrePlanCase::EGO_POSE &&
      calc_params_.should_prepare_second && !input_.is_replan_dynamic) {
    calc_params_.first_multi_plan = true;
    PreparePathSecondPlan();
  }

  bool set_multi_plan = true;
  if (calc_params_.pre_plan_case == PrePlanCase::MID_POINT) {
    if (input_.is_replan_first &&
        (output_.current_gear == geometry_lib::SEG_GEAR_DRIVE ||
         (output_.current_gear == geometry_lib::SEG_GEAR_REVERSE &&
          output_.gear_cmd_vec.size() > 0 &&
          output_.gear_cmd_vec.back() == geometry_lib::SEG_GEAR_DRIVE))) {
      set_multi_plan = false;
    } else if (input_.is_replan_second &&
               output_.current_gear == geometry_lib::SEG_GEAR_DRIVE) {
      set_multi_plan = false;
    }
  }

  if (MultiAdjustPathPlan(input_.ego_info_under_slot.cur_pose, input_.ref_gear,
                          PlanRequest::OPTIMAL_PATH)) {
    if (calc_params_.pre_plan_case == PrePlanCase::EGO_POSE &&
        input_.is_replan_first &&
        output_.current_gear == geometry_lib::SEG_GEAR_DRIVE) {
      set_multi_plan = false;
    }
    if (set_multi_plan) {
      calc_params_.first_multi_plan = false;
    }
    return true;
  }

  return false;
}

const bool PerpendicularTailInPathGenerator::PreparePathPlan() {
  ILOG_INFO << "\n ---enter prepare plan---";

  ILOG_INFO << "first prepare init pos = "
            << input_.ego_info_under_slot.cur_pose.pos.transpose()
            << "  heading = "
            << input_.ego_info_under_slot.cur_pose.heading * kRad2Deg;

  const double pre_start_time = IflyTime::Now_ms();
  std::vector<std::pair<geometry_lib::GeometryPath, geometry_lib::GeometryPath>>
      pair_geometry_path_vec;

  if (PrepareSinglePathPlan(input_.ego_info_under_slot.cur_pose,
                            pair_geometry_path_vec)) {
    // if is in searching stage, directly quit
    if (calc_params_.is_searching_stage) {
      return true;
    }

    // if is in parking stage, choose a better path
    // How to choose a better path, temporarily reverse, shorter and one step
    // path
    int better_index = -1;
    double min_cost = std::numeric_limits<double>::infinity();
    int debug_path_number = 10;
    std::map<double, int> cost_index_map;
    double cost = 0.0;
    for (size_t i = 0; i < pair_geometry_path_vec.size(); ++i) {
      geometry_lib::GeometryPath dubins_path = pair_geometry_path_vec[i].first;
      geometry_lib::GeometryPath rough_path = pair_geometry_path_vec[i].second;
      geometry_lib::GeometryPath complete_path = dubins_path;
      complete_path.AddPath(rough_path);
      if (rough_path.path_count < 1) {
        continue;
      }

      cost = 0.0;

      // 怎么设置较为合理的代价来使得第一段路径走的比较合理
      // 根据测试数据不断更新代价计算 使得越来越合理

      double ego_pose_cost = 0.0;
      // 当换挡次数大于1的时候尽量不从自车位置直接规划
      if (dubins_path.path_count < 1) {
        // 只允许0次换挡 即直接倒挡进库 或者1次换挡 D->R进库
        if (rough_path.gear_change_count > 2) {
          ego_pose_cost += 168.0;
        } else if (rough_path.gear_change_count == 2) {
          for (const auto& path_seg : rough_path.path_segment_vec) {
            if (path_seg.seg_gear == geometry_lib::SEG_GEAR_DRIVE) {
              ego_pose_cost += (std::fabs(path_seg.GetStartPos().y()) * 36.8 +
                                std::fabs(path_seg.GetStartHeading()) * 68.8);
              break;
            }
          }
        } else if (rough_path.cur_gear == geometry_lib::SEG_GEAR_DRIVE) {
          // 拿到前进挡的终点  D->R路径
          for (const auto& path_seg : rough_path.path_segment_vec) {
            if (path_seg.seg_gear == geometry_lib::SEG_GEAR_REVERSE) {
              ego_pose_cost += (std::fabs(path_seg.GetStartPos().y()) * 36.8 +
                                std::fabs(path_seg.GetStartHeading()) * 68.8);
              break;
            }
          }
        }
      }

      double dubins_path_cost = 0.0;
      // dubins路径如果换挡 施加惩罚
      if (dubins_path.gear_change_count > 0 &&
          dubins_path.gear_change_pose.size() > 0) {
        dubins_path_cost += 20.0;

        dubins_path_cost +=
            std::fabs(dubins_path.gear_change_pose.back().heading) * 36.0;

        if (!input_.can_first_plan_again) {
          dubins_path_cost += 268.0;
        }
      }

      // dubins路径如果不换挡 那对其终点位置的航向误差施加惩罚
      if (dubins_path.path_count > 0 && dubins_path.gear_change_count < 1) {
        double weight = 0.0;
        if (dubins_path.cur_gear == geometry_lib::SEG_GEAR_DRIVE) {
          weight = 66.0;
        } else {
          weight = 26.0;
        }
        dubins_path_cost += std::fabs(dubins_path.end_pose.heading) * weight;

        if (std::fabs(dubins_path.end_pose.heading) * kRad2Deg > 56.0) {
          const std::vector<double> dx{0.1, 1.0, 1.6, 1.8, 2.0, 2.2, 2.4, 3.0};
          const std::vector<double> dx_cost{288.0, 288.0, 208.0, 148.0,
                                            88.0,  38.0,  20.0,  10.0};
          dubins_path_cost +=
              mathlib::Interp1(dx, dx_cost, dubins_path.end_pose.pos.x() - 5.0);
        }
      }

      if (!input_.can_first_plan_again &&
          dubins_path.cur_gear != input_.ref_gear) {
        dubins_path_cost += 680.0;
      }

      // rough路径 对其当前挡位路径终点的航向和横向误差施加惩罚
      double fist_reverse_path_err_cost = 0.0;
      for (const auto& path_seg : rough_path.path_segment_vec) {
        if (path_seg.seg_gear != rough_path.cur_gear) {
          fist_reverse_path_err_cost +=
              (std::fabs(path_seg.GetStartPos().y()) * 5.0 +
               std::fabs(path_seg.GetStartHeading()) * 16.0) +
              path_seg.GetStartPos().x() * 4.8;
          break;
        }
      }

      // 障碍物距离代价
      double obs_dist_cost = 0.0;
      const double obs_dist_out_slot =
          std::min(dubins_path.obs_dist_info.out_slot.first,
                   rough_path.obs_dist_info.out_slot.first);

      const double obs_dist_in_slot = rough_path.obs_dist_info.in_slot.first;
      if (apa_param.GetParam().use_average_obs_dist) {
        CalcObsDistConsiderSlotForGeometryPath(complete_path);
        obs_dist_cost += 68.8 / complete_path.average_obs_dist;
      } else {
        const double max_dis = 0.68;
        const double min_dis = 0.38;
        double obs_dist_cost_out_slot = 0.0;
        double obs_dist_cost_in_slot = 0.0;
        if (obs_dist_out_slot > max_dis) {
          obs_dist_cost_out_slot += 86.8 / max_dis;
        } else if (obs_dist_out_slot < min_dis) {
          obs_dist_cost_out_slot += 168.8 / obs_dist_out_slot;
        } else {
          obs_dist_cost_out_slot += 86.8 / obs_dist_out_slot;
        }

        if (obs_dist_in_slot > 0.25) {
          obs_dist_cost_in_slot += 18.6 / 0.25;
        } else if (obs_dist_in_slot < 0.18) {
          obs_dist_cost_in_slot += 26.8 / obs_dist_in_slot;
        } else {
          obs_dist_cost_in_slot += 18.6 / obs_dist_in_slot;
        }

        if (rough_path.gear_change_cost > 1) {
          obs_dist_cost_in_slot *= 1.2;
          obs_dist_cost_out_slot *= 1.2;
        }

        obs_dist_cost = obs_dist_cost_in_slot + obs_dist_cost_out_slot;
      }

      // 当前挡位最小长度代价
      double cur_gear_path_length_cost = 0.0;
      if (complete_path.cur_gear_length <
          apa_param.GetParam().min_one_step_path_length) {
        cur_gear_path_length_cost += 3.0;
      }
      if (complete_path.cur_gear_length < kMinSingleGearPathLength + 1e-3) {
        cur_gear_path_length_cost += 100.0;
      }

      // 增加一把进的代价
      double one_step_cost = 0.0;
      if (!apa_param.GetParam().actual_mono_plan_enable &&
          rough_path.gear_change_count < 2) {
        // when mono plan is not allowed, the gear change count of
        // multi_adjust plan should be bigger than one
        one_step_cost += 200.0;
      }

      // 最后一段直线长度的代价
      double last_line_length_cost = 0.0;
      const auto& last_seg = rough_path.path_segment_vec.back();
      if (last_seg.seg_type != geometry_lib::SEG_TYPE_LINE) {
        cost += 100.0;
      } else {
        if (last_seg.Getlength() < 2.06) {
          cost += 16.8 / last_seg.Getlength();
        }
        if (rough_path.path_segment_vec.size() > 1) {
          const auto& pre_seg =
              rough_path
                  .path_segment_vec[rough_path.path_segment_vec.size() - 2];
          if (pre_seg.seg_type == geometry_lib::SEG_TYPE_ARC) {
            cost += 26.8 / std::max(pre_seg.GetArcSeg().circle_info.radius,
                                    calc_params_.turn_radius);
          }
        }
      }

      // 库内换挡的代价
      double gear_change_in_slot_cost = 0.0;
      for (const geometry_lib::PathPoint& pose : rough_path.gear_change_pose) {
        const double ratio = CalOccupiedRatio(pose);
        if (ratio > 0.468) {
          gear_change_in_slot_cost += ratio * 56.8;
        }
      }

      // 第一次和第二次换挡时距离车位过远代价
      double far_to_slot_cost = 0.0;
      for (size_t i = 0; i < complete_path.gear_change_pose.size() && i < 2;
           ++i) {
        const geometry_lib::PathPoint& pose = complete_path.gear_change_pose[i];
        if (std::fabs(pose.pos.y()) > 3.68) {
          far_to_slot_cost += (std::fabs(pose.pos.y()) - 3.67) * 68.8;
        }
        if (std::fabs(pose.pos.x()) > 7.96) {
          far_to_slot_cost += (std::fabs(pose.pos.x()) - 7.95) * 30.8;
        }
      }

      // 路径超过目标终点代价
      double path_out_target_cost = 0.0;
      for (const auto& seg : complete_path.path_segment_vec) {
        if (seg.GetStartPos().x() <
            input_.ego_info_under_slot.target_pose.pos.x()) {
          path_out_target_cost += 368.0;
          break;
        }
      }

      // 再次尝试first plan时与参考挡位不同的代价
      double ref_gear_cost = 0.0;
      if (!input_.can_first_plan_again &&
          complete_path.cur_gear != input_.ref_gear) {
        ref_gear_cost += 999.9;
      }

      // 增加 如果不是最后一段直线 直线较长的话施加惩罚代价
      for (const auto& seg : complete_path.path_segment_vec) {
        if (seg.seg_type == geometry_lib::SEG_TYPE_LINE &&
            std::fabs(seg.GetEndPose().heading) * kRad2Deg > 3.68 &&
            seg.Getlength() > 1.68) {
          cost += (seg.Getlength() - 1.68) * 36.8;
        }
      }

      // cur_gear_path_length_cost = 0.0;
      // fist_reverse_path_err_cost = 0.0;
      // ego_pose_cost = 0.0;
      // dubins_path_cost = 0.0;
      // gear_change_in_slot_cost = 0.0;
      // last_line_length_cost = 0.0;
      // 整条路径的换挡、换向和长度代价
      cost +=
          (complete_path.cost + one_step_cost + cur_gear_path_length_cost +
           obs_dist_cost + fist_reverse_path_err_cost + ego_pose_cost +
           dubins_path_cost + gear_change_in_slot_cost + last_line_length_cost +
           ref_gear_cost + far_to_slot_cost + path_out_target_cost);

      if (cost_index_map.size() > debug_path_number - 1) {
        cost_index_map.erase(std::prev(cost_index_map.end()));
      }
      cost_index_map[cost] = i;

      if (cost < min_cost) {
        better_index = static_cast<int>(i);
        min_cost = cost;
        ILOG_INFO << "gear_change_count = "
                  << static_cast<int>(complete_path.gear_change_count)
                  << "  cost = " << cost
                  << "  gear_change_cost = " << complete_path.gear_change_cost
                  << "  steer_change_cost = " << complete_path.steer_change_cost
                  << "  length_cost = " << complete_path.length_cost
                  << "  obs_dist_cost = " << obs_dist_cost
                  << "  obs_dist_out_slot = " << obs_dist_out_slot
                  << "  obs_dist_in_slot = " << obs_dist_in_slot
                  << "  average_obs_dist = " << complete_path.average_obs_dist
                  << "  fist_reverse_path_err_cost = "
                  << fist_reverse_path_err_cost
                  << "  dubins_path_cost = " << dubins_path_cost
                  << "  ego_pose_cost = " << ego_pose_cost
                  << "  cur_gear_path_length_cost = "
                  << cur_gear_path_length_cost
                  << "  gear_change_in_slot_cost = " << gear_change_in_slot_cost
                  << "  last_line_length_cost = " << last_line_length_cost
                  << "  one_step_cost = " << one_step_cost
                  << "  far_to_slot_cost = " << far_to_slot_cost
                  << "  ref_gear_cost = " << ref_gear_cost
                  << "  path_out_target_cost = " << path_out_target_cost;
      }
    }

    if (better_index == -1) {
      ILOG_INFO << "no optimal_dubins_geometry_path, quit";

      output_.Reset();

      return false;
    }

    ILOG_INFO << " better_index = " << better_index;

    // only for debug
    if (input_.is_simulation) {
      output_.perferred_geometry_path_vec.clear();
      output_.perferred_geometry_path_vec.reserve(cost_index_map.size());
      for (const auto& pair : cost_index_map) {
        if (pair.second == better_index) {
          continue;
        }
        geometry_lib::GeometryPath dubins_path =
            pair_geometry_path_vec[pair.second].first;
        geometry_lib::GeometryPath rough_path =
            pair_geometry_path_vec[pair.second].second;
        geometry_lib::GeometryPath complete_path = dubins_path;
        complete_path.AddPath(rough_path);
        // complete_path.PrintInfo();
        output_.perferred_geometry_path_vec.emplace_back(complete_path);
        ILOG_INFO << "gear_change_count = "
                  << static_cast<int>(complete_path.gear_change_count)
                  << "  cost = " << pair.first
                  << "  gear_change_cost = " << complete_path.gear_change_cost
                  << "  steer_change_cost = " << complete_path.steer_change_cost
                  << "  length_cost = " << complete_path.length_cost
                  << "  average_obs_dist = " << complete_path.average_obs_dist
                  << "  dubins_path dist_to_obs out slot = "
                  << dubins_path.obs_dist_info.out_slot.first
                  << "  rough_path dist_to_obs out slot = "
                  << rough_path.obs_dist_info.out_slot.first
                  << "  rough_path dist_to_obs in slot = "
                  << rough_path.obs_dist_info.in_slot.first;
      }
    }

    ILOG_INFO << "perferred_geometry_path_vec size = "
              << output_.perferred_geometry_path_vec.size();

    geometry_lib::GeometryPath optimal_dubins_geometry_path =
        pair_geometry_path_vec[better_index].first;

    geometry_lib::GeometryPath optimal_rough_geometry_path =
        pair_geometry_path_vec[better_index].second;

    if (optimal_dubins_geometry_path.path_count < 1) {
      ILOG_INFO << "use ego pose to multi_adjust plan";
      optimal_rough_geometry_path.PrintInfo();
      calc_params_.pre_plan_case = PrePlanCase::EGO_POSE;
      calc_params_.first_path_gear = optimal_rough_geometry_path.cur_gear;
    } else {
      ILOG_INFO << "use mid point to multi_adjust plan";
      optimal_dubins_geometry_path.PrintInfo();
      optimal_rough_geometry_path.PrintInfo();
      if (optimal_dubins_geometry_path.gear_change_count < 1 &&
          optimal_dubins_geometry_path.cur_gear ==
              geometry_lib::SEG_GEAR_REVERSE) {
        calc_params_.should_prepare_second = false;
      } else {
        calc_params_.should_prepare_second = true;
      }
      calc_params_.pre_plan_case = PrePlanCase::MID_POINT;
    }

    JSON_DEBUG_VALUE("pre_plan_case",
                     static_cast<int>(calc_params_.pre_plan_case))

    ILOG_INFO << "prepare path plan consume time = "
              << IflyTime::Now_ms() - pre_start_time << "ms";

    if (calc_params_.pre_plan_case == PrePlanCase::EGO_POSE) {
      return true;
    }

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
                      calc_params_.strict_col_lon_safe_dist, true,
                      ColDetMethod::GEOMETRY);
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
      input_.ego_info_under_slot.cur_pose =
          output_.path_segment_vec.back().GetEndPose();
      calc_params_.first_path_gear = output_.current_gear;
      ILOG_INFO << "first prepare target pos = "
                << input_.ego_info_under_slot.cur_pose.pos.transpose()
                << "  heading = "
                << input_.ego_info_under_slot.cur_pose.heading * kRad2Deg
                << "  safe_circle_tang_pt pos = "
                << calc_params_.safe_circle_tang_pt.pos.transpose()
                << " heading = "
                << calc_params_.safe_circle_tang_pt.heading * kRad2Deg
                << "  path length = " << output_.length
                << "  first_path_gear = "
                << geometry_lib::GetGearString(calc_params_.first_path_gear);

      return true;
    }
  }

  output_.Reset();

  return false;
}

const bool PerpendicularTailInPathGenerator::PrepareSinglePathPlan(
    const pnc::geometry_lib::PathPoint& cur_pose,
    std::vector<
        std::pair<geometry_lib::GeometryPath, geometry_lib::GeometryPath>>&
        pair_geometry_path_vec) {
  ILOG_INFO << "enter single prepare plan";
  const double pre_start_time = IflyTime::Now_ms();

  const double slot_side_sgn = calc_params_.slot_side_sgn;

  std::vector<double> x_offset_vec;
  std::vector<double> heading_offset_vec;
  x_offset_vec.reserve(5);
  heading_offset_vec.reserve(40);

  double max_heading =
      std::min(input_.ego_info_under_slot.slot.angle_,
               apa_param.GetParam().prepare_line_max_heading_offset_slot_deg);
  max_heading = slot_side_sgn *
                (input_.ego_info_under_slot.slot.angle_ - max_heading) *
                kDeg2Rad;

  double min_heading =
      apa_param.GetParam().prepare_line_min_heading_offset_slot_deg;
  min_heading = slot_side_sgn *
                (input_.ego_info_under_slot.slot.angle_ - min_heading) *
                kDeg2Rad;

  const double dheading =
      apa_param.GetParam().prepare_line_dheading_offset_slot_deg * kDeg2Rad;

  double heading = max_heading;
  while (slot_side_sgn * heading < slot_side_sgn * min_heading) {
    heading_offset_vec.emplace_back(heading);
    heading += slot_side_sgn * dheading;
  }

  double min_x =
      input_.ego_info_under_slot.slot.origin_corner_coord_local_.pt_01_mid.x() +
      (apa_param.GetParam().max_car_width * 0.5 +
       calc_params_.strict_car_lat_inflation + 0.05) /
          input_.ego_info_under_slot.slot.sin_angle_;

  min_x =
      std::max(min_x, cur_pose.pos.x() +
                          apa_param.GetParam().prepare_line_min_x_offset_slot /
                              input_.ego_info_under_slot.slot.sin_angle_);

  const double max_x =
      cur_pose.pos.x() + apa_param.GetParam().prepare_line_max_x_offset_slot /
                             input_.ego_info_under_slot.slot.sin_angle_;

  const double dx = apa_param.GetParam().prepare_line_dx_offset_slot /
                    input_.ego_info_under_slot.slot.sin_angle_;
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
  for (uint8_t i = 0; i < 2; ++i) {
    std::vector<double> safe_circle_radius_vec;
    if (!input_.can_first_plan_again || i == 1 || input_.is_searching_stage) {
      safe_circle_radius_vec = std::vector<double>{calc_params_.turn_radius};
    } else {
      safe_circle_radius_vec =
          std::vector<double>{8.68, calc_params_.turn_radius};
    }

    for (const double radius : safe_circle_radius_vec) {
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
        geometry_lib::PathPoint pose;
        pose.heading = line.heading;

        if (i == 0 && apa_param.GetParam().actual_mono_plan_enable &&
            MonoPreparePlan(pose.pos, radius)) {
          cal_tang_pt_success = true;
        }

        if (i == 1 && MultiPreparePlan(pose.pos, radius)) {
          cal_tang_pt_success = true;
        }

        if (!cal_tang_pt_success) {
          continue;
        }

        geometry_lib::PathSegment arc_seg;
        if (!geometry_lib::CalArcFromPt(gear, steer, virtual_1r_arc_length,
                                        radius, pose, arc_seg)) {
          continue;
        }

        inner_tang_pose_vec.clear();
        inner_tang_pose_vec.reserve(count + 1);

        for (uint8_t j = 0; j < count; ++j) {
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

  calc_params_.pre_plan_case = PrePlanCase::EGO_POSE;
  // 先尝试用自车位置规划一下 看是否能规划成功
  if (RoughMultiAdjustPathPlan(input_.ego_info_under_slot.cur_pose,
                               geometry_lib::SEG_GEAR_REVERSE,
                               rough_geometry_path, false, true)) {
    ILOG_INFO << "ego pose rough reverse gear plan success";
    rough_geometry_path.PrintInfo();
    pair_geometry_path_vec.emplace_back(
        std::make_pair(dubins_geometry_path, rough_geometry_path));
  }
  if (RoughMultiAdjustPathPlan(input_.ego_info_under_slot.cur_pose,
                               geometry_lib::SEG_GEAR_DRIVE,
                               rough_geometry_path, false, true)) {
    ILOG_INFO << "ego pose rough drive gear plan success";
    rough_geometry_path.PrintInfo();
    pair_geometry_path_vec.emplace_back(
        std::make_pair(dubins_geometry_path, rough_geometry_path));
  }
  for (const auto& pair : pair_geometry_path_vec) {
    if (!calc_params_.is_searching_stage &&
        apa_param.GetParam().actual_mono_plan_enable &&
        pair.second.gear_change_count < 1) {
      ILOG_INFO << "ego pose rough plan gear change count is 0, no need to "
                   "dubins plan";
      find_all_result = false;
    }
  }
  const size_t min_path_count_searching = 4;
  uint8_t fewer_gear_change_count = 0;
  calc_params_.tange_pose_vec.clear();
  calc_params_.tange_pose_vec.reserve(number);
  calc_params_.pre_plan_case = PrePlanCase::MID_POINT;
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
        geometry_lib::CalArcFromPt(gear, steer, virtual_1r_arc_length - j * ds,
                                   calc_params_.turn_radius,
                                   inner_inner_tang_pose_vec[0], arc_seg);
        PathColDetRes col_res =
            TrimPathByObs(arc_seg, calc_params_.strict_car_lat_inflation,
                          calc_params_.strict_col_lon_safe_dist, false);

        if (col_res == PathColDetRes::NORMAL) {
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

        // tang_pose.PrintInfo();

        bool dubins_connect_goal = false;
        std::vector<double> dubins_radius_vec;
        for (uint8_t dubins_gear_change_count = 0;
             dubins_gear_change_count < max_dubins_gear_change_count;
             ++dubins_gear_change_count) {
          if (dubins_gear_change_count == 0) {
            dubins_radius_vec =
                std::vector<double>{6.6, calc_params_.turn_radius};
            if (calc_params_.is_searching_stage) {
              dubins_radius_vec = std::vector<double>{calc_params_.turn_radius};
            }
          } else if (dubins_gear_change_count == 1) {
            dubins_radius_vec = std::vector<double>{6.6};
          }
          for (const double dubins_radius : dubins_radius_vec) {
            // result = RSPathPlan(
            //     cur_pose, tang_pose, dubins_radius, kMinSingleGearPathLength,
            //     dubins_gear_change_count, true, dubins_geometry_path);
            result = DubinsPathPlan(
                cur_pose, tang_pose, dubins_radius, kMinSingleGearPathLength,
                dubins_gear_change_count, geometry_lib::SEG_GEAR_DRIVE, true,
                dubins_geometry_path);
            dubins_connect_goal = (result == DubinsPlanResult::SUCCESS);
            if (dubins_connect_goal) {
              break;
            }
          }
          if (dubins_connect_goal) {
            break;
          }
        }

        // result = DubinsPathPlan(
        //     cur_pose, tang_pose, calc_params_.turn_radius,
        //     kMinSingleGearPathLength, max_dubins_gear_change_count,
        //     geometry_lib::SEG_GEAR_DRIVE, true, dubins_geometry_path);

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
            if (gear_change_count < 2) {
              continue_use_same_1arc = false;
              fewer_gear_change_count++;
            }

            if (calc_params_.is_searching_stage) {
              // searching stage
              if (pair_geometry_path_vec.size() >= min_path_count_searching) {
                find_all_result = false;
              }
            } else {
              // parking stage
              if ((pair_geometry_path_vec.size() > max_path_count &&
                   fewer_gear_change_count > 3) ||
                  (fewer_gear_change_count > 10 &&
                   apa_param.GetParam().actual_mono_plan_enable)) {
                find_all_result = false;
              }
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

  if (calc_params_.is_searching_stage) {
    return pair_geometry_path_vec.size() >= min_path_count_searching;
  }

  for (auto& pair_geometry_path : pair_geometry_path_vec) {
    auto& dubins_geometry_path = pair_geometry_path.first;
    auto& rough_geometry_path = pair_geometry_path.second;
    std::vector<geometry_lib::PathSegment> seg_vec;
    if (dubins_geometry_path.path_count > 0) {
      // mid point
      for (size_t i = 0; i < rough_geometry_path.path_segment_vec.size(); ++i) {
        if (rough_geometry_path.gear_cmd_vec[i] ==
            geometry_lib::SEG_GEAR_REVERSE) {
          if (rough_geometry_path.path_segment_vec[i].seg_type ==
                  geometry_lib::SEG_TYPE_LINE &&
              CalOccupiedRatio(
                  rough_geometry_path.path_segment_vec[i].GetEndPose()) > 0.8) {
            break;
          } else {
            seg_vec.emplace_back(rough_geometry_path.path_segment_vec[i]);
          }
        }
        if (rough_geometry_path.gear_cmd_vec[i + 1] ==
            geometry_lib::SEG_GEAR_DRIVE) {
          break;
        }
      }
    } else {
      // ego pose
      for (size_t i = 0; i < rough_geometry_path.path_segment_vec.size(); ++i) {
        if (rough_geometry_path.path_segment_vec[i].seg_type ==
                geometry_lib::SEG_TYPE_LINE &&
            CalOccupiedRatio(
                rough_geometry_path.path_segment_vec[i].GetEndPose()) > 0.8) {
          break;
        }
        seg_vec.emplace_back(rough_geometry_path.path_segment_vec[i]);
      }
    }
    // seg_vec存储着包含障碍物离轨迹的距离 但是需要考虑库内和库外
    geometry_lib::ObsDistConsiderSlot obs_dist_info;
    for (geometry_lib::PathSegment& seg : seg_vec) {
      CalcObsDistConsiderSlotForPathSeg(seg);
      if (seg.obs_dist_info.in_slot.first < obs_dist_info.in_slot.first) {
        obs_dist_info.in_slot = seg.obs_dist_info.in_slot;
      }
      if (seg.obs_dist_info.out_slot.first < obs_dist_info.out_slot.first) {
        obs_dist_info.out_slot = seg.obs_dist_info.out_slot;
      }
    }
    rough_geometry_path.obs_dist_info = obs_dist_info;
  }

  calc_params_.col_det_time = 0.0;

  return pair_geometry_path_vec.size() > 0;
}

void PerpendicularTailInPathGenerator::CalMonoSafeCircle(const double radius) {
  const double mono_radius = radius;

  calc_params_.mono_safe_circle.center.y() =
      calc_params_.target_line.pA.y() +
      mono_radius * calc_params_.slot_side_sgn;

  calc_params_.mono_safe_circle.radius = mono_radius;

  const Eigen::Vector2d pt_inside = input_.ego_info_under_slot.pt_inside;

  const double deta_x = std::sqrt(
      std::pow((mono_radius - apa_param.GetParam().car_width * 0.5 -
                apa_param.GetParam().car_lat_inflation_strict),
               2) -
      std::pow((calc_params_.mono_safe_circle.center.y() - pt_inside.y()), 2));

  calc_params_.mono_safe_circle.center.x() = pt_inside.x() - deta_x;

  // ILOG_INFO << "mono safe circle info: center = "
  //           << calc_params_.mono_safe_circle.center.transpose()
  //           << "   radius = " << calc_params_.mono_safe_circle.radius
  //          );
}

const bool PerpendicularTailInPathGenerator::CheckMonoIsFeasible() {
  const double dist = CalPoint2LineDist(calc_params_.mono_safe_circle.center,
                                        calc_params_.prepare_line);
  if (dist >= calc_params_.mono_safe_circle.radius) {
    // ILOG_INFO << "prepare_line is tangential or disjoint from mono safe "
    //              "circle, mono is feasible!";
    return true;
  } else {
    // ILOG_INFO << "prepare_line intersects circle, mono is not feasible";
    return false;
  }
}

const bool PerpendicularTailInPathGenerator::MonoPreparePlan(
    Eigen::Vector2d& tag_point, const double radius) {
  CalMonoSafeCircle(radius);

  if (CheckMonoIsFeasible() == false) {
    // ILOG_INFO << "cal monostep safe circle fail!";
    return false;
  }

  // calc actual circle corresponding to mono safe circle
  pnc::geometry_lib::Circle autual_circle;
  autual_circle = calc_params_.mono_safe_circle;

  const Eigen::Vector2d point_start = calc_params_.prepare_line.pA;

  // the length from point_start to point_tangent
  const double length =
      (autual_circle.center.y() -
       autual_circle.radius * calc_params_.pre_line_normal_vec.y() -
       point_start.y()) /
      calc_params_.pre_line_tangent_vec.y();

  tag_point = point_start + length * calc_params_.pre_line_tangent_vec;
  // ILOG_INFO << "point_tangent = " << tag_point.transpose());
  return true;
}

bool PerpendicularTailInPathGenerator::CalMultiSafeCircle(const double radius) {
  const Eigen::Vector2d pt_inside = input_.ego_info_under_slot.pt_inside;

  const double multi_radius = radius;

  pnc::geometry_lib::Circle circle_p1;
  circle_p1.center = pt_inside;
  circle_p1.radius = multi_radius - 0.5 * apa_param.GetParam().car_width -
                     apa_param.GetParam().car_lat_inflation_strict;

  // move down the start line
  const Eigen::Vector2d pt_s = calc_params_.prepare_line.pA +
                               calc_params_.pre_line_normal_vec * multi_radius;

  pnc::geometry_lib::LineSegment line_sa(
      pt_s, pt_s + calc_params_.pre_line_tangent_vec);

  // calc cross points of line and circle
  std::vector<Eigen::Vector2d> cross_points;
  const size_t cross_pt_nums =
      pnc::geometry_lib::CalcCrossPointsOfLineAndCircle(line_sa, circle_p1,
                                                        cross_points);

  if (cross_pt_nums != 2) {
    return false;
  }

  geometry_lib::Circle& multi_safe_circle = calc_params_.multi_safe_circle;

  if (cross_points[0].y() * calc_params_.slot_side_sgn >
      cross_points[1].y() * calc_params_.slot_side_sgn) {
    multi_safe_circle.center = cross_points[0];
  } else {
    multi_safe_circle.center = cross_points[1];
  }

  multi_safe_circle.radius = multi_radius;

  // ILOG_INFO << "multi safa circle info: center = " <<
  // multi_safe_circle.center
  //           << "  radius = " << multi_safe_circle.radius);

  return true;
}

const bool PerpendicularTailInPathGenerator::MultiPreparePlan(
    Eigen::Vector2d& tag_point, const double radius) {
  if (CalMultiSafeCircle(radius) == false) {
    // ILOG_INFO << "cal multistep safe circle fail!";
    return false;
  }

  tag_point =
      calc_params_.multi_safe_circle.center -
      calc_params_.pre_line_normal_vec * calc_params_.multi_safe_circle.radius;

  // ILOG_INFO << "point_tangent = " << tag_point.transpose());
  return true;
}

const bool PerpendicularTailInPathGenerator::PreparePathSecondPlan() {
  const uint8_t ref_gear =
      geometry_lib::ReverseGear(calc_params_.first_path_gear);

  const geometry_lib::PathPoint start_pose =
      input_.ego_info_under_slot.cur_pose;
  const geometry_lib::PathPoint target_pose = calc_params_.safe_circle_tang_pt;

  ILOG_INFO << "second prepare init pos = "
            << input_.ego_info_under_slot.cur_pose.pos.transpose()
            << "  heading = "
            << input_.ego_info_under_slot.cur_pose.heading * kRad2Deg;

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
      DubinsPathPlan(start_pose, target_pose, calc_params_.turn_radius,
                     min_length, 0, ref_gear, true, geometry_path);

  // 如果失败， 分为两种，参考挡位是前进挡还是倒退档
  // 如果是倒退档， 不再尝试，直接以当前位置接着后续规划
  // 如果是前进挡，是否可以再找其他的目标点相连，
  // 但是这样目标点怎么确定也是个问题， 能不能把原先成功规划的目标点都试一次

  if (result != DubinsPlanResult::SUCCESS &&
      ref_gear == geometry_lib::SEG_GEAR_DRIVE) {
    ILOG_INFO << "try dubins to connect first tang pose fail, and gear is "
                 "drive, try connect any tang_pose";
    for (const auto& tang_pose : calc_params_.tange_pose_vec) {
      result = DubinsPathPlan(start_pose, tang_pose, calc_params_.turn_radius,
                              min_length, 0, ref_gear, true, geometry_path);
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
    geometry_lib::Arc arc;
    arc.pA = target_pose.pos;
    arc.headingA = target_pose.heading;
    // 如果当前挡位是前进挡 延长一条直线便于控制跟踪
    if (ref_gear == geometry_lib::SEG_GEAR_DRIVE && input_.is_replan_second &&
        geometry_lib::CalOneArcWithLineAndGear(
            arc, calc_params_.target_line, geometry_lib::SEG_GEAR_REVERSE)) {
      geometry_lib::PathSegment line_seg;
      geometry_lib::CalLineFromPt(ref_gear,
                                  apa_param.GetParam().insert_line_after_arc,
                                  geometry_path.end_pose, line_seg);
      TrimPathByObs(line_seg,
                    apa_param.GetParam().car_lat_inflation_strict + 0.03,
                    apa_param.GetParam().col_obs_safe_dist_strict + 0.03, true,
                    ColDetMethod::GEOMETRY);
      line_seg.PrintInfo(true);
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
    input_.ego_info_under_slot.cur_pose =
        output_.path_segment_vec.back().GetEndPose();
    ILOG_INFO << "second prepare, from first prepare pos to safe circle tange "
                 "success";

    ILOG_INFO << "second prepare target pos = "
              << input_.ego_info_under_slot.cur_pose.pos.transpose()
              << "  heading = "
              << input_.ego_info_under_slot.cur_pose.heading * kRad2Deg;

    return true;
  } else {
    ILOG_INFO
        << "second prepare, from first prepare pos to safe circle tange fail";
    return false;
  }

  return false;
}

const PerpendicularTailInPathGenerator::DubinsPlanResult
PerpendicularTailInPathGenerator::DubinsPathPlan(
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

    cost += temp_output_vec[i].gear_change_count * 30.0;

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
    for (auto& temp_path_seg : geometry_path.path_segment_vec) {
      if (TrimPathByObs(temp_path_seg, calc_params_.strict_car_lat_inflation,
                        calc_params_.strict_col_lon_safe_dist,
                        false) != PathColDetRes::NORMAL) {
        geometry_path.Reset();
        return DubinsPlanResult::PATH_COLLISION;
      }
    }
    CalcObsDistConsiderSlotForGeometryPath(geometry_path);
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
  RSPathRequestType request_type =
      RSPathRequestType::GEAR_SWITCH_LESS_THAN_TWICE;
  rs.GeneShortestRSPath(&path, &is_connected_to_goal, &start, &end, turn_radius,
                        true, true, request_type, 100.0);

  if (!is_connected_to_goal || path.size < 1) {
    return DubinsPlanResult::NO_PATH;
  }

  // check gear
  if (path.gear_change_number > max_gear_change_count) {
    return DubinsPlanResult::NO_PATH;
  }

  AstarPathGear cur_gear = path.paths[0].gear;
  // check length
  if (path.gear_change_number == 0) {
    if (cur_gear == AstarPathGear::DRIVE &&
        path.total_length < min_length + 1e-3) {
      return DubinsPlanResult::NO_PATH;
    }
  } else {
    double drive_length{0.};
    double reverse_length{0.};
    if (cur_gear == AstarPathGear::DRIVE) {
      for (int i = 0; i < path.size; ++i) {
        if (path.paths[i].gear == cur_gear) {
          drive_length += std::fabs(path.paths[i].length);
        }
      }
      reverse_length = path.total_length - drive_length;
      if (drive_length < min_length + 1e-3) {
        return DubinsPlanResult::NO_PATH;
      }
    } else {
      for (int i = 0; i < path.size; ++i) {
        if (path.paths[i].gear == cur_gear) {
          reverse_length += std::fabs(path.paths[i].length);
        }
      }
      drive_length = path.total_length - reverse_length;
      if (drive_length < min_length + 1e-3 ||
          reverse_length < min_length + 1e-3) {
        return DubinsPlanResult::NO_PATH;
      }
    }
  }

  std::vector<geometry_lib::PathSegment> path_seg_vec;
  for (size_t i = 0; i < path.size; ++i) {
    RSPathSegment rs_path_seg = path.paths[i];
    const uint8_t gear = (rs_path_seg.gear == AstarPathGear::DRIVE)
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
        geometry_lib::NormalizeAngle(rs_path_seg.points[0].theta));

    geometry_lib::PathSegment path_seg;

    if (steer == geometry_lib::SEG_STEER_STRAIGHT) {
      if (geometry_lib::CalLineFromPt(gear, std::fabs(rs_path_seg.length),
                                      start_pose, path_seg)) {
        path_seg_vec.emplace_back(path_seg);
      }
    } else {
      if (geometry_lib::CalArcFromPt(gear, steer, std::fabs(rs_path_seg.length),
                                     turn_radius, start_pose, path_seg)) {
        path_seg_vec.emplace_back(path_seg);
      }
    }
  }

  geometry_path.SetPath(path_seg_vec);

  if (need_col_det) {
    for (auto& temp_path_seg : geometry_path.path_segment_vec) {
      if (TrimPathByObs(temp_path_seg, calc_params_.strict_car_lat_inflation,
                        calc_params_.strict_col_lon_safe_dist,
                        false) != PathColDetRes::NORMAL) {
        geometry_path.Reset();
        return DubinsPlanResult::PATH_COLLISION;
      }
    }
    CalcObsDistConsiderSlotForGeometryPath(geometry_path);
  }

  return DubinsPlanResult::SUCCESS;
}

const PerpendicularTailInPathGenerator::PathColDetRes
PerpendicularTailInPathGenerator::TrimPathByObs(
    pnc::geometry_lib::PathSegment& path_seg, const double lat_inflation,
    const double lon_safe_dist, const bool enable_log,
    const ColDetMethod method) {
  const double time = IflyTime::Now_ms();
  path_seg.lat_buffer = lat_inflation;
  ColResult res;

  const auto& edt_col_det_ptr =
      collision_detector_interface_ptr_->GetEDTCollisionDetectorPtr();

  const auto& gjk_col_det_ptr =
      collision_detector_interface_ptr_->GetGJKCollisionDetectorPtr();

  const auto& geometry_col_det_ptr =
      collision_detector_interface_ptr_->GetGeometryCollisionDetectorPtr();

  bool init_pose_near_obs = false;
  if (edt_col_det_ptr
          ->Update(
              std::vector<geometry_lib::PathPoint>{path_seg.GetStartPose()},
              lat_inflation, lon_safe_dist, true)
          .pt_closest2obs.first < 0.151) {
    init_pose_near_obs = true;
    ILOG_INFO_IF(enable_log) << "start pos is closed to obs";
  }

  if (method == ColDetMethod::EDT_GEOMETRY || method == ColDetMethod::EDT_GJK) {
    res = edt_col_det_ptr->Update(path_seg, lat_inflation, lon_safe_dist, true);

    path_seg.obs_dist_info.integrated = res.pt_closest2obs;
    path_seg.pt_obs_dist_info_vec = res.pt_obs_dist_info_vec;

    // EDT碰撞检测有误差 即有碰撞也可能显示无碰撞 无碰撞也可能显示有碰撞
    // 在初始离障碍物较近的时候
    // 在横向buffer相对较小 虽无碰撞但障碍物距离自车很近的时候
    // 需要做一次精确的碰撞检测 来绝对确保路径的安全性
    // 5 * 1.414  = 7.07 目前是edt碰撞检测的最大横向误差

    const bool need_use_accurate =
        init_pose_near_obs ||
        (!res.col_flag && lat_inflation < 0.171 &&
         res.pt_closest2obs.first < 0.071 + lat_inflation);

    ILOG_INFO_IF(enable_log) << "need_use_accurate = " << need_use_accurate;

    if (need_use_accurate) {
      ILOG_INFO_IF(enable_log)
          << "obs dist = " << res.pt_closest2obs.first
          << "  pt = " << res.pt_closest2obs.second.pos.transpose();
      if (method == ColDetMethod::EDT_GEOMETRY) {
        res = geometry_col_det_ptr->Update(path_seg, lat_inflation,
                                           lon_safe_dist);
      } else if (method == ColDetMethod::EDT_GJK) {
        res = gjk_col_det_ptr->Update(path_seg, lat_inflation, lon_safe_dist,
                                      GJKColDetRequest());
      }
    }
  }

  else if (method == ColDetMethod::GEOMETRY) {
    res = geometry_col_det_ptr->Update(path_seg, lat_inflation, lon_safe_dist);
  }

  else if (method == ColDetMethod::GJK) {
    res = gjk_col_det_ptr->Update(path_seg, lat_inflation, lon_safe_dist,
                                  GJKColDetRequest());
  }

  calc_params_.col_det_time += IflyTime::Now_ms() - time;

  const double remain_car_dist = res.remain_car_dist;
  const double remain_obs_dist = res.remain_obs_dist;
  // the dist that the car can go
  const double safe_remain_dist = res.remain_dist;

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
    geometry_lib::GeometryPath& ahead_path, const bool enable_log,
    const bool ego_pose_flag) {
  ILOG_INFO_IF(enable_log) << " --- enter RoughMultiAdjustPathPlan --- ";
  ahead_path.Reset();
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

  calc_params_.optimize_plan = false;

  // const double rough_lat_buffer_penalty_value = 0.0268;
  const double rough_lat_buffer_penalty_value = 0.0;

  for (size_t i = 0; i < 8 && success_geometry_path_vec.size() < 1; ++i) {
    ILOG_INFO_IF(enable_log) << i << "th try RoughMultiAdjustPathPlan ";

    if (i == 0) {
      calc_params_.cur_gear_path_flag = true;
    } else {
      calc_params_.cur_gear_path_flag = false;
    }

    double lat_buffer = apa_param.GetParam().car_lat_inflation_normal;
    double lon_buffer = apa_param.GetParam().col_obs_safe_dist_normal;
    if (calc_params_.first_multi_plan &&
        (i == 0 ||
         (ego_pose_flag && single_ref_gear == geometry_lib::SEG_GEAR_REVERSE &&
          i == 1)) &&
        CalOccupiedRatio(single_cur_pose) < 0.168) {
      lat_buffer = apa_param.GetParam().car_lat_inflation_strict + 0.0368;
      lon_buffer = apa_param.GetParam().col_obs_safe_dist_strict;
    }

    for (const geometry_lib::PathPoint& cur_pose : cur_pose_vec) {
      if (CalOccupiedRatio(cur_pose) < 1e-3 &&
          single_ref_gear == geometry_lib::SEG_GEAR_DRIVE) {
        lat_buffer = apa_param.GetParam().car_lat_inflation_strict;
        lon_buffer = apa_param.GetParam().col_obs_safe_dist_strict;
      }

      lat_buffer += rough_lat_buffer_penalty_value;

      ILOG_INFO_IF(enable_log)
          << "lat_buffer = " << lat_buffer << "  lon_buffer = " << lon_buffer;

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
            geometry_path.cur_gear == geometry_lib::SEG_GEAR_REVERSE) {
          if (CheckStuckedByInside(geometry_path.start_pose,
                                   geometry_path.end_pose, enable_log)) {
            reverse_1arc_safe = false;
            ILOG_INFO_IF(enable_log) << "reverse_1arc is not safe";
            break;
          }
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
  ahead_path.Reset();
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
  optimal_geometry_path.Reset();
  // plan all paths based on the current pose and select the optimal path

  uint8_t single_ref_gear = ref_gear;
  geometry_lib::PathPoint single_cur_pose = pose;
  std::vector<geometry_lib::PathPoint> cur_pose_vec{single_cur_pose};

  const double time_start = IflyTime::Now_ms();

  calc_params_.optimize_plan = true;

  int i = 0;
  const int max_compensate_line_try_count = 6;
  int compensate_line_try_count = 0;
  const double compensate_line_length_step = 0.25;
  const size_t max_seg_path_count = 222;
  std::vector<geometry_lib::GeometryPath> geometry_path_vec;
  geometry_path_vec.reserve(max_seg_path_count);
  const size_t max_path_count = 10;
  std::vector<geometry_lib::GeometryPath> success_geometry_path_vec;
  success_geometry_path_vec.reserve(max_path_count);
  geometry_lib::GeometryPath geometry_path;
  geometry_lib::GeometryPath geometry_path_copy;

  int max_try_count = 10;
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

    if (i == 0) {
      calc_params_.cur_gear_path_flag = true;
    } else {
      calc_params_.cur_gear_path_flag = false;
    }

    ILOG_INFO << i
              << "th try OptimalMultiAdjustPathPlan and "
                 "success_geometry_path_vec_size = "
              << success_geometry_path_vec.size();

    double lat_buffer = apa_param.GetParam().car_lat_inflation_normal;
    double lon_buffer = apa_param.GetParam().col_obs_safe_dist_normal;

    if (calc_params_.first_multi_plan &&
        (i == 0 ||
         (calc_params_.pre_plan_case == PrePlanCase::EGO_POSE &&
          single_ref_gear == geometry_lib::SEG_GEAR_REVERSE && i == 1)) &&
        CalOccupiedRatio(single_cur_pose) < 0.168) {
      lat_buffer = apa_param.GetParam().car_lat_inflation_strict + 0.0368;
      lon_buffer = apa_param.GetParam().col_obs_safe_dist_strict;
    }

    if (input_.is_replan_dynamic) {
      lat_buffer += 0.0268;
      lon_buffer += 0.0268;
    }

    for (const geometry_lib::PathPoint& cur_pose : cur_pose_vec) {
      if (CalOccupiedRatio(cur_pose) < 1e-3 &&
          single_ref_gear == geometry_lib::SEG_GEAR_DRIVE) {
        lat_buffer = apa_param.GetParam().car_lat_inflation_strict;
        lon_buffer = apa_param.GetParam().col_obs_safe_dist_strict;
      }

      // lat_buffer += 0.0268;

      ILOG_INFO << "lat_buffer = " << lat_buffer
                << "  lon_buffer = " << lon_buffer;

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
          if (i == 0 ||
              geometry_path.cur_gear_length > kMinSingleGearPathLength) {
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
    const bool try_compensate_line =
        (calc_params_.pre_plan_case == PrePlanCase::MID_POINT) ||
        (calc_params_.pre_plan_case == PrePlanCase::EGO_POSE &&
         calc_params_.first_path_gear == geometry_lib::SEG_GEAR_DRIVE);

    if (calc_params_.first_multi_plan && try_compensate_line && i == 0 &&
        success_geometry_path_vec.size() < 1) {
      for (const geometry_lib::GeometryPath& geometry_path :
           geometry_path_vec) {
        // if 1r path col, also start pose and end pose is on the same side of
        // the target line, should enter this logic
        if (geometry_path.collide_flag &&
            geometry_path.cur_gear == geometry_lib::SEG_GEAR_REVERSE &&
            compensate_line_try_count < max_compensate_line_try_count) {
          if (CheckStuckedByInside(geometry_path.start_pose,
                                   geometry_path.end_pose)) {
            ILOG_INFO << "try use a reverse gear straight line to cast off "
                         "inside stuck";
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
    }

    // If the car in slot is already close to obstacle , try driving straight
    // ahead for a distance
    if (i == 0 && success_geometry_path_vec.empty() &&
        geometry_path_vec.empty() &&
        CalOccupiedRatio(single_cur_pose) > 0.168 &&
        std::fabs(single_cur_pose.heading) * kRad2Deg < 3.68 &&
        single_ref_gear == geometry_lib::SEG_GEAR_DRIVE &&
        compensate_line_try_count < max_compensate_line_try_count) {
      ILOG_INFO << "try use a drive gear straight line to cast off "
                   "two side stuck in slot";
      i = -1;
      geometry_path_vec.clear();
      success_geometry_path_vec.clear();
      cur_pose_vec.clear();
      compensate_line_try_count++;
      single_cur_pose.pos =
          pose.pos + compensate_line_length_step * compensate_line_try_count *
                         geometry_lib::GenHeadingVec(pose.heading);
      single_cur_pose.heading = pose.heading;
      cur_pose_vec.emplace_back(single_cur_pose);
      geometry_lib::PathSegment line_seg(
          geometry_lib::SEG_GEAR_DRIVE,
          geometry_lib::LineSegment(pose.pos, single_cur_pose.pos,
                                    pose.heading));

      if (TrimPathByObs(line_seg, lat_buffer, lon_buffer) ==
          PathColDetRes::NORMAL) {
        geometry_lib::GeometryPath temp_geometry_path(line_seg);
        temp_geometry_path.PrintInfo();
        geometry_path_vec.emplace_back(temp_geometry_path);
        ILOG_INFO << "set i is -1, and continuing a drive gear straight line";
      }
    }

    if (geometry_path_vec.empty()) {
      break;
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

  int optimal_path_index = -1;
  double min_cost = std::numeric_limits<double>::infinity();
  for (int k = 0; k < success_geometry_path_vec.size(); ++k) {
    // success_geometry_path_vec[k].PrintInfo();
    geometry_lib::GeometryPath complete_path;
    if (output_.path_segment_vec.size() > 0) {
      complete_path.SetPath(output_.path_segment_vec);
      complete_path.AddPath(success_geometry_path_vec[k]);
    } else {
      complete_path.SetPath(success_geometry_path_vec[k].path_segment_vec);
    }

    if (complete_path.path_count < 1) {
      continue;
    }

    double cost = 0.0;

    if (!apa_param.GetParam().actual_mono_plan_enable &&
        calc_params_.first_multi_plan &&
        success_geometry_path_vec[k].gear_change_count < 2) {
      // when mono plan is not allowed, the gear change count of multi_adjust
      // plan should be bigger than one
      cost += 200.0;
    }

    if (calc_params_.first_multi_plan &&
        calc_params_.pre_plan_case == PrePlanCase::EGO_POSE) {
      if (complete_path.gear_change_count > 1) {
        cost += 168.0;
      } else if (complete_path.cur_gear == geometry_lib::SEG_GEAR_DRIVE) {
        for (const auto& path_seg : complete_path.path_segment_vec) {
          if (path_seg.seg_gear == geometry_lib::SEG_GEAR_REVERSE) {
            cost += (std::fabs(path_seg.GetStartPos().y()) * 36.8 +
                     std::fabs(path_seg.GetStartHeading()) * 68.8);
            break;
          }
        }
      }
    }

    if (calc_params_.first_multi_plan) {
      for (const auto& path_seg : complete_path.path_segment_vec) {
        if (path_seg.seg_gear != complete_path.cur_gear) {
          cost += (std::fabs(path_seg.GetStartPos().y()) * 5.0 +
                   std::fabs(path_seg.GetStartHeading()) * 16.0) +
                  path_seg.GetStartPos().x() * 4.8;
          break;
        }
      }
    }

    if (complete_path.cur_gear_length <
        apa_param.GetParam().min_one_step_path_length) {
      cost += 3.0;
    }

    if (complete_path.cur_gear_length < kMinSingleGearPathLength + 1e-3) {
      cost += 100.0;
    }

    // 最后一段直线长度的代价
    const auto& last_seg = complete_path.path_segment_vec.back();
    if (last_seg.seg_type != geometry_lib::SEG_TYPE_LINE) {
      cost += 100.0;
    } else {
      if (last_seg.Getlength() < 2.06) {
        cost += 12.8 / last_seg.Getlength();
      }
      if (complete_path.path_segment_vec.size() > 1) {
        const auto& pre_seg =
            complete_path
                .path_segment_vec[complete_path.path_segment_vec.size() - 2];
        if (pre_seg.seg_type == geometry_lib::SEG_TYPE_ARC) {
          cost += 26.8 / std::max(pre_seg.GetArcSeg().circle_info.radius,
                                  calc_params_.turn_radius);
        }
      }
    }

    // 增加 如果不是最后一段直线 直线较长的话施加惩罚代价
    for (const auto& seg : complete_path.path_segment_vec) {
      if (seg.seg_type == geometry_lib::SEG_TYPE_LINE &&
          std::fabs(seg.GetEndPose().heading) * kRad2Deg > 6.8 &&
          seg.Getlength() > 1.68) {
        cost += (seg.Getlength() - 1.68) * 46.8;
      }
    }

    // 规划终点离目标终点距离代价
    cost += (last_seg.GetEndPos() - input_.ego_info_under_slot.target_pose.pos)
                .norm() *
            46.8;

    // 规划终点离目标终点横向误差代价
    cost += std::fabs((last_seg.GetEndPos() -
                       input_.ego_info_under_slot.target_pose.pos)
                          .y()) *
            4006.8;

    // 库内换挡的代价
    for (const auto& pose : complete_path.gear_change_pose) {
      const double ratio = CalOccupiedRatio(pose);
      if (ratio > 0.468) {
        cost += ratio * 56.8;
      }
    }

    // 加上一个障碍物距离代价
    CalcObsDistConsiderSlotForGeometryPath(success_geometry_path_vec[k]);
    if (apa_param.GetParam().use_average_obs_dist) {
      cost += 16.8 / success_geometry_path_vec[k].average_obs_dist;
    } else {
      cost += 12.68 / success_geometry_path_vec[k].obs_dist_info.out_slot.first;
      cost += 18.68 / success_geometry_path_vec[k].obs_dist_info.in_slot.first;
    }

    // 增加当前挡位路径横向和航向误差代价
    cost +=
        (26.8 *
             std::max(std::fabs(complete_path.cur_gear_path_segments_vec.back()
                                    .GetEndHeading()),
                      8.6 * kDeg2Rad) +
         16.8 *
             std::max(std::fabs(complete_path.cur_gear_path_segments_vec.back()
                                    .GetEndPos()
                                    .y()),
                      0.028));

    // 前进挡 尽量x值不要太大 否则增加代价
    for (const auto& drive_seg : complete_path.drive_seg_vec) {
      const double slot_max_x =
          std::min(std::max(drive_seg.front().GetStartPos().x() + 2.08,
                            calc_params_.target_line.pA.x() + 3.68),
                   input_.ego_info_under_slot.slot.processed_corner_coord_local_
                           .pt_01_mid.x() +
                       1.08);
      const double drive_max_x = drive_seg.back().GetEndPos().x();
      if (drive_max_x > slot_max_x) {
        cost += ((drive_max_x - slot_max_x) * 86.8);
      }
    }

    // 路径超过目标终点代价
    for (const auto& seg : complete_path.path_segment_vec) {
      if (seg.GetStartPos().x() <
          input_.ego_info_under_slot.target_pose.pos.x()) {
        cost += 368.0;
        break;
      }
    }

    // 整条路径的换挡、换向和长度代价
    cost += complete_path.cost;

    // 针对1R路径 如果在库外有S弯可能导致控制跟不上
    // 在外侧有障碍物的情况下可能导致碰撞风险 因此 增大此路径的代价
    // 情愿多换一次挡
    if ((input_.is_left_empty && !calc_params_.is_left_side) ||
        (input_.is_right_empty && calc_params_.is_left_side) &&
            calc_params_.first_multi_plan && complete_path.path_count > 1) {
      const geometry_lib::PathSegment& seg1 = complete_path.path_segment_vec[0];
      const geometry_lib::PathSegment& seg2 = complete_path.path_segment_vec[1];
      if (geometry_lib::IsSTrunPath(seg1, seg2) &&
          seg1.GetArcSeg().circle_info.radius <
              calc_params_.turn_radius * 1.05 &&
          seg1.Getlength() < 1.68 && seg1.GetEndPos().x() > 7.168) {
        cost += 60.0;
      }
    }

    ILOG_INFO << "gear_change_count = "
              << static_cast<int>(
                     success_geometry_path_vec[k].gear_change_count)
              << "  cost = " << cost;
    if (cost < min_cost) {
      optimal_path_index = k;
      min_cost = cost;
    }
  }

  optimal_geometry_path = success_geometry_path_vec[optimal_path_index];

  // 路径超过目标终点直接失败
  for (const auto& seg : optimal_geometry_path.path_segment_vec) {
    if (seg.GetStartPos().x() <
        input_.ego_info_under_slot.target_pose.pos.x() - 0.068) {
      ILOG_INFO << "path is exceed target pose, return false";
      return false;
    }
  }

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
  // 2. 只看1R阶段
  // 是否能一把入库，适用于正式泊车预规划阶段选择一个最优的切点
  // 3. 根据当前车辆位姿规划出所有路径，选择一条最优路径
  ILOG_INFO << "ref_gear = " << geometry_lib::GetGearString(ref_gear);
  geometry_lib::GeometryPath geometry_path;
  if (plan_request == PlanRequest::ROUGH_PATH) {
    RoughMultiAdjustPathPlan(pose, ref_gear, geometry_path);
  } else if (plan_request == PlanRequest::ONE_STEP_PATH) {
    OneStepMultiAdjustPathPlan(pose, ref_gear, geometry_path);
  } else if (plan_request == PlanRequest::OPTIMAL_PATH) {
    if (input_.is_replan_dynamic) {
      OptimalMultiAdjustPathPlan(pose, geometry_lib::SEG_GEAR_REVERSE,
                                 geometry_path);
    } else {
      geometry_lib::GeometryPath geometry_path_r;
      geometry_lib::GeometryPath geometry_path_d;
      if (calc_params_.pre_plan_case == PrePlanCase::EGO_POSE) {
        // 除了第一次规划两个挡位 其他按照参考挡位来
        if (input_.is_replan_first) {
          OptimalMultiAdjustPathPlan(pose, geometry_lib::SEG_GEAR_REVERSE,
                                     geometry_path_r);

          OptimalMultiAdjustPathPlan(pose, geometry_lib::SEG_GEAR_DRIVE,
                                     geometry_path_d);

          if (geometry_path_r.path_count < 1 &&
              geometry_path_d.path_count > 1) {
            geometry_path = geometry_path_d;
          } else if (geometry_path_r.path_count > 1 &&
                     geometry_path_d.path_count < 1) {
            geometry_path = geometry_path_r;
          } else if (geometry_path_r.path_count > 1 &&
                     geometry_path_d.path_count > 1) {
            geometry_path = (geometry_path_r.cost > geometry_path_d.cost)
                                ? geometry_path_d
                                : geometry_path_r;
          }
        } else {
          if (!OptimalMultiAdjustPathPlan(pose, ref_gear, geometry_path)) {
            OptimalMultiAdjustPathPlan(
                pose, geometry_lib::ReverseGear(ref_gear), geometry_path);
          }
        }
      } else if (calc_params_.pre_plan_case == PrePlanCase::MID_POINT) {
        // 第一次规划 或第二次规划 或first multi plan 优先倒挡
        // 其他按照参考挡位来
        uint8_t pre_gear = ref_gear;
        if (input_.is_replan_first || input_.is_replan_second ||
            calc_params_.first_multi_plan) {
          pre_gear = geometry_lib::SEG_GEAR_REVERSE;
        }
        if (!OptimalMultiAdjustPathPlan(pose, pre_gear, geometry_path)) {
          OptimalMultiAdjustPathPlan(pose, geometry_lib::ReverseGear(pre_gear),
                                     geometry_path);
        }
      }
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

  if (CheckReachTargetPose()) {
    return true;
  }

  if (input_.is_replan_dynamic) {
    return false;
  }

  if (output_.path_segment_vec.size() > 0) {
    const geometry_lib::PathPoint& init_pose =
        output_.path_segment_vec.front().GetStartPose();
    if (std::fabs(init_pose.pos.y()) > 0.268 ||
        std::fabs(init_pose.heading) * kRad2Deg > 8.6) {
      return true;
    }

    if (output_.path_segment_vec.front().seg_gear ==
        geometry_lib::SEG_GEAR_DRIVE) {
      return true;
    }
  }

  return false;
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
                            geometry_path, true, enable_log)) {
    geometry_path_vec.emplace_back(geometry_path);
  }

  if (AlignAndSTurnPathPlan(pose, ref_gear, lat_buffer, lon_buffer,
                            geometry_path, false, enable_log)) {
    geometry_path_vec.emplace_back(geometry_path);
  }

  if (DubinsPathPlan(pose, ref_gear, lat_buffer, lon_buffer, geometry_path,
                     enable_log)) {
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

  bool need_shift = false;
  if (ref_gear == geometry_lib::SEG_GEAR_DRIVE) {
    need_shift = true;
  }

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
      arc.circle_info.radius < 15.68;

  success = success && (arc.pB.x() > calc_params_.target_line.pA.x() + 0.368);

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
    need_shift = true;
    if (calc_params_.optimize_plan && calc_params_.cur_gear_path_flag) {
      std::vector<geometry_lib::PathSegment> seg_vec;
      if (ref_gear == geometry_lib::SEG_GEAR_DRIVE &&
          FindPtCanReverseToSlot(
              seg_vec, ref_gear, steer, arc.circle_info.radius,
              arc_seg.Getlength() * 0.5, arc_seg.Getlength(), pose, lat_buffer,
              lon_buffer, GeometryPathType::ALIGNBODY_STURN, 0.468, false)) {
        ILOG_INFO_IF(enable_log) << "drive arc + reverse alignbodysturn";
        geometry_path.SetPath(seg_vec);
      } else {
        // line + two reverse arc, the line and first arc can be same or not
        if (FindPtCanReverseToSlot(
                seg_vec, ref_gear, geometry_lib::SEG_STEER_STRAIGHT, 68.0,
                arc_seg.Getlength() * 0.5, arc_seg.Getlength(), pose,
                lat_buffer, lon_buffer, GeometryPathType::TWO_ARC, 0.368)) {
          ILOG_INFO_IF(enable_log) << "line + two reverse arc";
          geometry_path.SetPath(seg_vec);
        }
      }
    }
  }

  if (res == PathColDetRes::NORMAL) {
    ILOG_INFO_IF(enable_log) << "one arc path no col\n";
    geometry_path.SetPath(arc_seg);
  }

  if (geometry_path.path_count > 0) {
    ILOG_INFO_IF(enable_log) << "one arc plan has path";

    if (calc_params_.optimize_plan && calc_params_.cur_gear_path_flag) {
      std::vector<geometry_lib::PathSegment> seg_vec =
          geometry_path.path_segment_vec;
      geometry_lib::PathSegment seg = seg_vec.back();

      const double min_x = need_shift ? calc_params_.target_line.pA.x() + 0.86
                                      : calc_params_.target_line.pA.x() - 0.068;

      if (seg.GetEndPos().x() < min_x) {
        seg_vec.pop_back();

        bool success = seg.seg_type == geometry_lib::SEG_TYPE_LINE &&
                       FindPtCanReverseToSlot(
                           seg_vec, ref_gear, geometry_lib::SEG_STEER_STRAIGHT,
                           68.0, seg.Getlength() * 0.5, seg.Getlength(),
                           seg.GetStartPose(), lat_buffer, lon_buffer,
                           GeometryPathType::ALIGNBODY_STURN, 0.468, false);

        success =
            success ||
            (seg.seg_type == geometry_lib::SEG_TYPE_ARC &&
             FindPtCanReverseToSlot(
                 seg_vec, ref_gear, seg.seg_steer,
                 seg.arc_seg.circle_info.radius, seg.Getlength() * 0.5,
                 seg.Getlength(), seg.GetStartPose(), lat_buffer, lon_buffer,
                 GeometryPathType::ALIGNBODY_STURN, 0.468, false));

        if (success && seg_vec.back().GetEndPos().x() > min_x) {
          geometry_path.SetPath(seg_vec);
        }
      }
    }

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

  bool need_shift = false;
  if (ref_gear == geometry_lib::SEG_GEAR_DRIVE || !same_gear) {
    need_shift = true;
  }

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
        ILOG_INFO_IF(enable_log) << "same gear, line is safe and "
                                    "then check arc col, add line "
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
        path_seg_vec.emplace_back(arc_seg);
        geometry_path.SetPath(path_seg_vec);
        break;
      }

      if (col_res2 == PathColDetRes::SHORTEN) {
        ILOG_INFO_IF(enable_log)
            << "same gear, arc shorten, try extend line and use two arc";
        // use line + two arc
        std::vector<geometry_lib::PathSegment> seg_vec;

        if (calc_params_.cur_gear_path_flag &&
            FindPtCanReverseToSlot(
                seg_vec, line_gear, geometry_lib::SEG_STEER_STRAIGHT, 68.0,
                line_seg.Getlength(), line_seg.Getlength() + 2.26,
                line_seg.GetStartPose(), lat_buffer, lon_buffer,
                GeometryPathType::TWO_ARC, 0.368)) {
          ILOG_INFO_IF(enable_log) << "line + two reverse arc to target line";
          geometry_path.SetPath(seg_vec);
          need_shift = true;
          break;
        }
      }
      continue;
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
          // 沿着直线遍历 看是否能反向TwoArc成功 且确保两个Arc都没有撞
          CalcObsDistConsiderSlotForPathSeg(arc_seg);
          arc_seg.PrintInfo(enable_log);
          double init_length = line_seg.Getlength() * 0.268;
          int k = 0;
          while (calc_params_.optimize_plan &&
                 calc_params_.cur_gear_path_flag &&
                 init_length < line_seg.Getlength() && k < 20) {
            std::vector<geometry_lib::PathSegment> seg_vec;
            if (!FindPtCanReverseToSlot(
                    seg_vec, ref_gear, geometry_lib::SEG_STEER_STRAIGHT, 68.0,
                    init_length, line_seg.Getlength(), line_seg.GetStartPose(),
                    lat_buffer, lon_buffer, GeometryPathType::TWO_ARC, 0.3,
                    false, true, true)) {
              break;
            }

            geometry_lib::PathSegment temp_arc_seg;
            geometry_lib::CalArcFromPt(
                arc_gear, arc_steer,
                std::fabs(seg_vec.back().GetEndHeading()) *
                    calc_params_.turn_radius,
                calc_params_.turn_radius, seg_vec.back().GetEndPose(),
                temp_arc_seg);
            TrimPathByObs(temp_arc_seg, lat_buffer, lon_buffer, enable_log);
            CalcObsDistConsiderSlotForPathSeg(temp_arc_seg);

            if (arc_seg.obs_dist_info.out_slot.first <
                    temp_arc_seg.obs_dist_info.out_slot.first + 0.068 ||
                (temp_arc_seg.obs_dist_info.out_slot.first > 0.386)) {
              temp_arc_seg.PrintInfo(enable_log);
              geometry_path.SetPath(seg_vec);
              break;
            }

            init_length = seg_vec.front().Getlength() + 0.1;

            k++;
          }

          if (geometry_path.path_count < 1) {
            geometry_path.SetPath(line_seg);
          }
          break;
        }

        if (col_res2 == PathColDetRes::SHORTEN) {
          ILOG_INFO_IF(enable_log)
              << "opposite gear, the arc shorten, then line can move "
                 "more little, try use two reverse arc to line ";
          std::vector<geometry_lib::PathSegment> seg_vec;
          if (calc_params_.cur_gear_path_flag &&
              FindPtCanReverseToSlot(
                  seg_vec, ref_gear, geometry_lib::SEG_STEER_STRAIGHT, 68.0,
                  line_seg.Getlength() * 0.5, line_seg.Getlength(),
                  line_seg.GetStartPose(), lat_buffer, lon_buffer,
                  GeometryPathType::TWO_ARC, 0.468, false)) {
            ILOG_INFO_IF(enable_log)
                << "try use line and two reverse arc to target line";
            geometry_path.SetPath(seg_vec);
            break;
          }
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
          ILOG_INFO_IF(enable_log) << "arc col is invalid, then "
                                      "construct line to sure safe";
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
        ILOG_INFO_IF(enable_log)
            << "opposite gear, the line can not shorten, then line can move "
               "more little, try use two reverse arc to line ";
        std::vector<geometry_lib::PathSegment> seg_vec;
        if (calc_params_.optimize_plan && calc_params_.cur_gear_path_flag &&
            FindPtCanReverseToSlot(
                seg_vec, ref_gear, geometry_lib::SEG_STEER_STRAIGHT, 68.0,
                line_seg.Getlength() * 0.5, line_seg.Getlength(),
                line_seg.GetStartPose(), lat_buffer, lon_buffer,
                GeometryPathType::TWO_ARC, 0.468, false)) {
          ILOG_INFO_IF(enable_log)
              << "try use line and two reverse arc to target line";
          geometry_path.SetPath(seg_vec);
          break;
        }
        continue;
        ILOG_INFO_IF(enable_log) << "opposite gear, line shorten, "
                                    "then construct line to "
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

    if (calc_params_.optimize_plan && calc_params_.cur_gear_path_flag) {
      std::vector<geometry_lib::PathSegment> seg_vec =
          geometry_path.path_segment_vec;
      geometry_lib::PathSegment seg = seg_vec.back();

      const double min_x = need_shift ? calc_params_.target_line.pA.x() + 0.86
                                      : calc_params_.target_line.pA.x() - 0.068;

      if (seg.GetEndPos().x() < min_x) {
        seg_vec.pop_back();

        bool success = seg.seg_type == geometry_lib::SEG_TYPE_LINE &&
                       FindPtCanReverseToSlot(
                           seg_vec, ref_gear, geometry_lib::SEG_STEER_STRAIGHT,
                           68.0, seg.Getlength() * 0.5, seg.Getlength(),
                           seg.GetStartPose(), lat_buffer, lon_buffer,
                           GeometryPathType::ALIGNBODY_STURN, 0.468, false);

        success =
            success ||
            (seg.seg_type == geometry_lib::SEG_TYPE_ARC &&
             FindPtCanReverseToSlot(
                 seg_vec, ref_gear, seg.seg_steer,
                 seg.arc_seg.circle_info.radius, seg.Getlength() * 0.5,
                 seg.Getlength(), seg.GetStartPose(), lat_buffer, lon_buffer,
                 GeometryPathType::ALIGNBODY_STURN, 0.468, false));

        if (success && seg_vec.back().GetEndPos().x() > min_x) {
          geometry_path.SetPath(seg_vec);
        }
      }
    }

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
    const bool enable_log, const bool iter) {
  // if same_gear is true, the arc1 and arc2 should have same gear,
  // otherwise must opposite gear
  ILOG_INFO_IF(enable_log) << "\n----enter two " << same_gear
                           << " arc path plan----";
  geometry_path.Reset();

  bool need_shift = false;
  if (ref_gear == geometry_lib::SEG_GEAR_DRIVE || !same_gear) {
    need_shift = true;
  }

  bool all_path_safe = false;

  const double temp_radius = calc_params_.turn_radius;
  std::vector<double> radius_vec;
  if (same_gear) {
    radius_vec = std::vector<double>{2.0 * temp_radius, 1.75 * temp_radius,
                                     1.5 * temp_radius, 1.25 * temp_radius,
                                     1.0 * temp_radius};

    if (input_.is_replan_dynamic) {
      radius_vec = std::vector<double>{2.0 * temp_radius, 1.75 * temp_radius,
                                       1.5 * temp_radius};
    }
  } else {
    radius_vec = std::vector<double>{1.0 * temp_radius};
  }

  for (const double radius : radius_vec) {
    std::vector<std::pair<geometry_lib::Arc, geometry_lib::Arc>> arc_pair_vec;
    if (!geometry_lib::CalTwoArcWithLine(pose, calc_params_.target_line, radius,
                                         radius, arc_pair_vec)) {
      continue;
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
        }

        ILOG_INFO_IF(enable_log) << "same gear, arc1 is safe and "
                                    "then check arc col, add arc1 "
                                    "to path";

        std::vector<geometry_lib::PathSegment> path_seg_vec{arc1_seg};

        PathColDetRes col_res2 =
            TrimPathByObs(arc2_seg, lat_buffer, lon_buffer, enable_log);

        if (col_res2 == PathColDetRes::INVALID) {
          ILOG_INFO_IF(enable_log) << "same gear, arc2 invalid, quit\n";
          continue;
        }

        if (col_res2 == PathColDetRes::NORMAL) {
          ILOG_INFO_IF(enable_log)
              << "same gear, arc2 normal, add arc2 to path";
          if (input_.is_replan_dynamic &&
              arc2_seg.GetEndPos().x() <
                  calc_params_.target_line.pA.x() + 0.368) {
            break;
          }
          path_seg_vec.emplace_back(arc2_seg);
          geometry_path.SetPath(path_seg_vec);
          all_path_safe = true;
          break;
        }

        if (col_res2 == PathColDetRes::SHORTEN) {
          if (radius > radius_vec.back() + 0.168) {
            ILOG_INFO_IF(enable_log)
                << "same gear, radius is bigger, abort this arc";
            continue;
          }

          // use arc + line + two arc
          if (iter && calc_params_.cur_gear_path_flag &&
              FindPtCanReverseToSlot(
                  path_seg_vec, arc1_gear, geometry_lib::SEG_STEER_STRAIGHT,
                  68.0, 0.468, 2.26, arc1_seg.GetEndPose(), lat_buffer,
                  lon_buffer, GeometryPathType::TWO_ARC, 0.268, true, false,
                  true)) {
            ILOG_INFO_IF(enable_log)
                << "same gear, radius is small, arc2 shorten, then insert "
                   "line after arc1, and use two reverse arc to target line";
            need_shift = true;
            geometry_path.SetPath(path_seg_vec);
            break;
          }
        }

        continue;
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

          if (col_res2 == PathColDetRes::NORMAL) {
            ILOG_INFO_IF(enable_log) << "arc2 col is normal, add arc1 to path";
            geometry_path.SetPath(arc1_seg);
            all_path_safe = true;
            break;
          }

          std::vector<geometry_lib::PathSegment> path_seg_vec{};

          if (col_res2 == PathColDetRes::SHORTEN) {
            ILOG_INFO_IF(enable_log) << "arc2 col is shorten";
            if (arc1_gear == geometry_lib::SEG_GEAR_REVERSE) {
              ILOG_INFO_IF(enable_log) << "the arc1 gear is reverse, extend "
                                          "line after arc1, add arc1 "
                                          "and line to path";
              const double length = std::max(
                  arc1_seg.GetEndPos().x() -
                      (input_.ego_info_under_slot.slot
                           .processed_corner_coord_local_.pt_01_mid.x() -
                       1.56),
                  0.86);
              const geometry_lib::LineSegment line(arc1_seg.GetEndPos(),
                                                   arc1_seg.GetEndHeading(),
                                                   length, arc1_gear);
              geometry_lib::PathSegment line_seg(arc1_gear, line);
              path_seg_vec.emplace_back(arc1_seg);
              if (TrimPathByObs(line_seg, 0.12, 0.4, enable_log) !=
                      PathColDetRes::INVALID &&
                  line_seg.GetEndPos().x() >
                      calc_params_.target_line.pA.x() + 1.368) {
                path_seg_vec.emplace_back(line_seg);
              }
            } else {
              // use arc + two reverse arc
              if (iter && calc_params_.cur_gear_path_flag &&
                  FindPtCanReverseToSlot(
                      path_seg_vec, arc1_gear, arc1_steer,
                      arc1.circle_info.radius, arc1_seg.Getlength() * 0.468,
                      arc1_seg.Getlength(), arc1_seg.GetStartPose(), lat_buffer,
                      lon_buffer, GeometryPathType::TWO_ARC, 0.368, false)) {
                ILOG_INFO_IF(enable_log)
                    << "the arc1 gear is drive, reduce arc1 length and use "
                       "two reverve arc to target line ";
              } else {
                path_seg_vec.emplace_back(arc1_seg);
                ILOG_INFO_IF(enable_log) << "add arc1 to path";
              }
            }
            geometry_path.SetPath(path_seg_vec);
            break;
          }

          if (col_res2 == PathColDetRes::INVALID) {
            ILOG_INFO_IF(enable_log) << "arc2 col is invalid, then "
                                        "construct arc1 to sure safe";
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
          ILOG_INFO_IF(enable_log) << "arc1 col is shorten then "
                                      "construct arc1 to sure safe";
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
      ILOG_INFO_IF(enable_log) << "two arc plan has path radius = " << radius;

      if (calc_params_.optimize_plan && calc_params_.cur_gear_path_flag) {
        std::vector<geometry_lib::PathSegment> seg_vec =
            geometry_path.path_segment_vec;
        geometry_lib::PathSegment seg = seg_vec.back();

        const double min_x = need_shift
                                 ? calc_params_.target_line.pA.x() + 0.86
                                 : calc_params_.target_line.pA.x() - 0.068;

        if (seg.GetEndPos().x() < min_x) {
          seg_vec.pop_back();

          bool success =
              seg.seg_type == geometry_lib::SEG_TYPE_LINE &&
              FindPtCanReverseToSlot(
                  seg_vec, ref_gear, geometry_lib::SEG_STEER_STRAIGHT, 68.0,
                  seg.Getlength() * 0.5, seg.Getlength(), seg.GetStartPose(),
                  lat_buffer, lon_buffer, GeometryPathType::ALIGNBODY_STURN,
                  0.468, false);

          success =
              success ||
              (seg.seg_type == geometry_lib::SEG_TYPE_ARC &&
               FindPtCanReverseToSlot(
                   seg_vec, ref_gear, seg.seg_steer,
                   seg.arc_seg.circle_info.radius, seg.Getlength() * 0.5,
                   seg.Getlength(), seg.GetStartPose(), lat_buffer, lon_buffer,
                   GeometryPathType::ALIGNBODY_STURN, 0.468, false));

          if (success && seg_vec.back().GetEndPos().x() > min_x) {
            geometry_path.SetPath(seg_vec);
          }
        }
      }

      geometry_path.all_path_safe = all_path_safe;

      geometry_path.PrintInfo(enable_log);
      return true;
    } else {
      geometry_path.Reset();
      ILOG_INFO_IF(enable_log)
          << "two arc plan has no path radius = " << radius;
    }
  }

  return false;
}

const bool PerpendicularTailInPathGenerator::AlignAndSTurnPathPlan(
    const geometry_lib::PathPoint& pose, const uint8_t ref_gear,
    const double lat_buffer, const double lon_buffer,
    geometry_lib::GeometryPath& geometry_path, const bool same_gear,
    const bool enable_log, const bool easy_to_line, const bool iter) {
  ILOG_INFO_IF(enable_log) << "\n----enter align" << same_gear
                           << "and s_turn path plan----";
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
                 cur_heading - tar_heading)) > 42.68 * kDeg2Rad) {
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
    double lon_safe_buffer = lon_buffer;
    if (same_gear) {
      lon_safe_buffer = 0.0;
    }
    if (TrimPathByObs(arc_seg, lat_buffer, lon_safe_buffer, enable_log) !=
        PathColDetRes::NORMAL) {
      ILOG_INFO_IF(enable_log) << "align body path col, quit";
      return false;
    }

    ILOG_INFO_IF(enable_log) << "align body path success";

    align_arc_vaild = true;
    geometry_path.SetPath(arc_seg);
  }

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
  //     pnc::geometry_lib::BuildLineSegByPose(arc_s_1.pA,
  //     arc_s_1.headingA);

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

  const double radius = calc_params_.turn_radius;
  std::vector<double> radius_vec{2.5 * radius,  2.25 * radius, 2.0 * radius,
                                 1.75 * radius, 1.5 * radius,  1.25 * radius,
                                 1.0 * radius};

  if (easy_to_line) {
    radius_vec = std::vector<double>{2.5 * radius, 2.25 * radius, 2.0 * radius,
                                     1.75 * radius};
  } else if (input_.is_replan_dynamic) {
    radius_vec = std::vector<double>{2.5 * radius, 2.25 * radius, 2.0 * radius,
                                     1.75 * radius, 1.5 * radius};
  }

  const uint8_t s_turn_ref_gear =
      same_gear ? ref_gear : geometry_lib::ReverseGear(ref_gear);

  std::vector<geometry_lib::PathPoint> virtual_target_pose_vec;

  if (CalOccupiedRatio(geometry_lib::PathPoint(arc_s_1.pA, arc_s_1.headingA)) >
          0.168 &&
      s_turn_ref_gear == geometry_lib::SEG_GEAR_REVERSE) {
    const double dy = (arc_s_1.pA.y() > target_line.pA.y()) ? 0.01 : -0.01;

    double max_lat_err = (input_.is_replan_dynamic)
                             ? apa_param.GetParam().target_pos_err
                             : apa_param.GetParam().target_pos_err * 0.5;

    if (easy_to_line) {
      max_lat_err = 0.0168;
    }

    for (size_t i = 0; std::fabs(i * dy) < max_lat_err; ++i) {
      virtual_target_pose_vec.emplace_back(geometry_lib::PathPoint(
          Eigen::Vector2d(target_line.pA.x(), target_line.pA.y() + i * dy),
          target_line.heading));
    }

    ILOG_INFO_IF(enable_log) << "try use different target pos";
  } else {
    virtual_target_pose_vec.emplace_back(
        geometry_lib::PathPoint(target_line.pA, target_line.heading));
  }

  bool find_res = false;
  for (const geometry_lib::PathPoint& virtual_target_pose :
       virtual_target_pose_vec) {
    for (const double radius : radius_vec) {
      ILOG_INFO_IF(enable_log)
          << "radius = " << radius
          << "  dy = " << virtual_target_pose.pos.y() - target_line.pA.y();
      arc_s_1.SetRadius(radius);
      arc_s_2.SetRadius(radius);
      arc_s_2.pB = virtual_target_pose.pos;
      arc_s_2.headingB = virtual_target_pose.heading;

      success = pnc::geometry_lib::CalTwoArcWithSameHeading(arc_s_1, arc_s_2,
                                                            s_turn_ref_gear);

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

      success =
          success && (gear1 == s_turn_ref_gear) && (gear2 == s_turn_ref_gear);

      geometry_lib::PathSegment arc_seg1(steer1, gear1, arc_s_1);
      geometry_lib::PathSegment arc_seg2(steer2, gear2, arc_s_2);

      success = success &&
                (TrimPathByObs(arc_seg1, lat_buffer, 0.0, enable_log) ==
                 PathColDetRes::NORMAL) &&
                (TrimPathByObs(arc_seg2, lat_buffer, lon_buffer, enable_log) ==
                 PathColDetRes::NORMAL);

      if (success && s_turn_ref_gear == geometry_lib::SEG_GEAR_REVERSE) {
        // ensure that there is a straight line connecting the end
        // pose
        double min_line_length = 0.268;
        if (easy_to_line) {
          min_line_length = 0.68;
        } else if (input_.is_replan_dynamic) {
          min_line_length = 0.368;
        }
        success = arc_seg2.GetEndPos().x() >
                  calc_params_.target_line.pA.x() + min_line_length;
      }

      if (success) {
        find_res = true;
        if (same_gear) {
          geometry_path.AddPath(
              std::vector<geometry_lib::PathSegment>{arc_seg1, arc_seg2});
        }
        break;
      }
    }
    if (find_res) {
      break;
    }
  }

  if (success) {
    ILOG_INFO_IF(enable_log) << "AlignAndSTurnPathPlan has path";
    if (calc_params_.optimize_plan && calc_params_.cur_gear_path_flag && iter &&
        ref_gear == geometry_lib::SEG_GEAR_DRIVE &&
        geometry_path.end_pose.pos.x() >
            input_.ego_info_under_slot.slot.processed_corner_coord_local_
                    .pt_01_mid.x() -
                0.168) {
      std::vector<geometry_lib::PathSegment> seg_vec =
          geometry_path.path_segment_vec;
      geometry_lib::PathSegment seg = seg_vec.back();
      seg_vec.pop_back();
      if (FindPtCanReverseToSlot(
              seg_vec, ref_gear, seg.seg_steer, seg.arc_seg.circle_info.radius,
              seg.Getlength() * 0.5, seg.Getlength(), seg.GetStartPose(),
              lat_buffer, lon_buffer, GeometryPathType::ALIGNBODY_STURN, 0.268,
              false)) {
        geometry_path.SetPath(seg_vec);
      }
    }
    geometry_path.PrintInfo(enable_log);
  } else {
    if (!same_gear) {
      if (geometry_path.path_count > 0) {
        ILOG_INFO_IF(enable_log) << "AlignAndSTurnPathPlan only has align path";
        geometry_path.PrintInfo(enable_log);
        return true;
      }
      return false;
    }

    // car heading err is 0, but no s turn path
    const double lat_err = (ref_gear == geometry_lib::SEG_GEAR_DRIVE)
                               ? apa_param.GetParam().target_pos_err
                               : apa_param.GetParam().target_pos_err * 0.5;

    if (ref_gear == geometry_lib::SEG_GEAR_REVERSE &&
        geometry_path.path_count > 0) {
      if (align_lat_err < lat_err - 1e-3) {
        ILOG_INFO_IF(enable_log)
            << "pA is close to the target line, directly "
               "try use align result continue to one line plan reverse";
        geometry_path.PrintInfo(enable_log);
        return true;
      }
    }

    if (ref_gear == geometry_lib::SEG_GEAR_DRIVE) {
      // use line to go out slot
      const double length = input_.ego_info_under_slot.slot
                                .processed_corner_coord_local_.pt_01_mid.x() +
                            0.168 - arc_s_1.pA.x();

      geometry_lib::PathSegment line_seg;
      geometry_lib::CalLineFromPt(
          geometry_lib::SEG_GEAR_DRIVE, length,
          geometry_lib::PathPoint(arc_s_1.pA, arc_s_1.headingA), line_seg);

      if (length > 0.16) {
        TrimPathByObs(line_seg, lat_buffer, lon_buffer, enable_log);
      }

      if (length < 0.168) {
        if (geometry_path.path_count > 0) {
          ILOG_INFO_IF(enable_log)
              << "AlignAndSTurnPathPlan only has align path";
          geometry_path.PrintInfo(enable_log);
          return true;
        }
        return false;
      }

      geometry_path.AddPath(line_seg);
      ILOG_INFO_IF(enable_log)
          << "AlignAndSTurnPathPlan last try success, use line "
             "to go out slot, and then use reverse path to eliminate lat err";
      geometry_path.PrintInfo(enable_log);
      return true;
    }
  }

  return success;
}

const bool PerpendicularTailInPathGenerator::DubinsPathPlan(
    const geometry_lib::PathPoint& pose, const uint8_t ref_gear,
    const double lat_buffer, const double lon_buffer,
    geometry_lib::GeometryPath& geometry_path, const bool enable_log) {
  ILOG_INFO_IF(enable_log) << "\n----enter dubins path plan----";
  // 根据目标终点计算出一个终点
  geometry_path.Reset();
  geometry_lib::PathPoint tar_pose;
  tar_pose.heading = calc_params_.target_line.heading;
  tar_pose.pos = calc_params_.target_line.pA +
                 1.68 * geometry_lib::GenHeadingVec(tar_pose.heading);
  if (ref_gear == geometry_lib::SEG_GEAR_REVERSE) {
    if (pose.pos.x() < tar_pose.pos.x() + kMinSingleGearPathLength) {
      tar_pose.pos.x() =
          pose.pos.x() - apa_param.GetParam().min_one_step_path_length;
    }
    tar_pose.pos.x() =
        std::max(tar_pose.pos.x(), calc_params_.target_line.pA.x() + 1e-3);
  }

  ILOG_INFO_IF(enable_log) << "dubins target pos = " << tar_pose.pos.transpose()
                           << "  heading = " << tar_pose.heading * kRad2Deg;

  dubins_lib::DubinsLibrary::Input input(pose.pos, tar_pose.pos, pose.heading,
                                         tar_pose.heading,
                                         calc_params_.turn_radius);

  dubins_planner_.SetInput(input);

  std::vector<dubins_lib::DubinsLibrary::Output> dubins_output_vec =
      dubins_planner_.Update();

  if (dubins_output_vec.empty()) {
    return false;
  }

  // 初筛掉不满足挡位和长度的路径
  std::vector<dubins_lib::DubinsLibrary::Output> temp_output_vec;
  temp_output_vec.reserve(dubins_output_vec.size());

  for (const auto& output : dubins_output_vec) {
    if (output.current_gear_cmd != ref_gear || output.gear_cmd_vec.size() < 1) {
      continue;
    }
    uint8_t last_gear = geometry_lib::SEG_GEAR_INVALID;
    for (int i = output.gear_cmd_vec.size() - 1; i >= 0; --i) {
      if (output.gear_cmd_vec[i] != geometry_lib::SEG_GEAR_INVALID) {
        last_gear = output.gear_cmd_vec[i];
        break;
      }
    }
    if (last_gear != geometry_lib::SEG_GEAR_REVERSE) {
      continue;
    }
    if (output.current_length < kMinSingleGearPathLength) {
      continue;
    }
    temp_output_vec.emplace_back(output);
  }

  if (temp_output_vec.empty()) {
    return false;
  }

  // 筛选出不碰撞的路径
  std::vector<geometry_lib::GeometryPath> temp_geometry_path_vec;
  temp_geometry_path_vec.reserve(temp_output_vec.size());
  for (auto& output : temp_output_vec) {
    std::vector<geometry_lib::PathSegment> path_seg_vec;
    path_seg_vec.reserve(3);

    pnc::geometry_lib::PathSegment path_seg;
    // set arc AB
    if (output.gear_cmd_vec[0] != pnc::geometry_lib::SEG_GEAR_INVALID) {
      path_seg.seg_gear = output.gear_cmd_vec[0];
      path_seg.seg_type = geometry_lib::SEG_TYPE_ARC;
      path_seg.arc_seg = output.arc_AB;
      path_seg.seg_steer = geometry_lib::CalArcSteer(path_seg.arc_seg);
      path_seg_vec.emplace_back(path_seg);
    }
    // set line BC
    if (output.gear_cmd_vec[1] != pnc::geometry_lib::SEG_GEAR_INVALID) {
      path_seg.seg_gear = output.gear_cmd_vec[1];
      path_seg.seg_type = geometry_lib::SEG_TYPE_LINE;
      path_seg.line_seg = output.line_BC;
      path_seg.seg_steer = geometry_lib::SEG_STEER_STRAIGHT;
      path_seg_vec.emplace_back(path_seg);
    }
    // set arc CD
    if (output.gear_cmd_vec[2] != pnc::geometry_lib::SEG_GEAR_INVALID) {
      path_seg.seg_gear = output.gear_cmd_vec[2];
      path_seg.seg_type = geometry_lib::SEG_TYPE_ARC;
      path_seg.arc_seg = output.arc_CD;
      path_seg.seg_steer = geometry_lib::CalArcSteer(path_seg.arc_seg);
      path_seg_vec.emplace_back(path_seg);
    }

    int d_r_arc = 0, d_l_arc = 0, r_r_arc = 0, r_l_arc = 0;
    for (const auto& seg : path_seg_vec) {
      if (seg.seg_gear == geometry_lib::SEG_GEAR_DRIVE &&
          seg.seg_steer == geometry_lib::SEG_STEER_RIGHT) {
        d_r_arc++;
      }
      if (seg.seg_gear == geometry_lib::SEG_GEAR_DRIVE &&
          seg.seg_steer == geometry_lib::SEG_STEER_LEFT) {
        d_l_arc++;
      }
      if (seg.seg_gear == geometry_lib::SEG_GEAR_REVERSE &&
          seg.seg_steer == geometry_lib::SEG_STEER_RIGHT) {
        r_r_arc++;
      }
      if (seg.seg_gear == geometry_lib::SEG_GEAR_REVERSE &&
          seg.seg_steer == geometry_lib::SEG_STEER_LEFT) {
        r_l_arc++;
      }
    }

    if ((d_r_arc == r_r_arc && d_r_arc != 0) ||
        (d_l_arc == r_l_arc && d_l_arc != 0)) {
      ILOG_INFO_IF(enable_log) << "should not go backtrack";
      continue;
    }

    bool col_flag = false;
    for (auto& path_seg : path_seg_vec) {
      if (TrimPathByObs(path_seg, lat_buffer, lon_buffer, enable_log) !=
          PathColDetRes::NORMAL) {
        col_flag = true;
        break;
      }
    }
    if (col_flag) {
      continue;
    }

    temp_geometry_path_vec.emplace_back(
        geometry_lib::GeometryPath(path_seg_vec));
  }

  // 选一条较优的路径
  double min_cost = std::numeric_limits<double>::infinity();
  int optimal_index = -1;
  for (int i = 0; i < temp_geometry_path_vec.size(); ++i) {
    double cost = temp_geometry_path_vec[i].cost;
    if (cost < min_cost) {
      min_cost = cost;
      optimal_index = i;
    }
  }

  if (optimal_index == -1) {
    return false;
  }

  ILOG_INFO_IF(enable_log) << "dubins plan success, has path";
  const geometry_lib::GeometryPath& temp_geometry_path =
      temp_geometry_path_vec[optimal_index];
  for (size_t i = 0; i < temp_geometry_path.path_count; ++i) {
    if (temp_geometry_path.gear_cmd_vec[i] == temp_geometry_path.cur_gear) {
      geometry_path.AddPath(temp_geometry_path.path_segment_vec[i]);
    } else {
      break;
    }
  }
  geometry_path.PrintInfo(enable_log);

  return true;
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

  const double lat_err =
      (line_gear == geometry_lib::SEG_GEAR_DRIVE || input_.is_replan_dynamic)
          ? apa_param.GetParam().target_pos_err
          : apa_param.GetParam().target_pos_err * 0.5;

  const double heading_err =
      (line_gear == geometry_lib::SEG_GEAR_DRIVE || input_.is_replan_dynamic)
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
  double suitable_path_length = apa_param.GetParam().min_one_step_path_length;

  if (ref_gear == geometry_lib::SEG_GEAR_DRIVE &&
      std::fabs(geometry_path.end_pose.heading * kRad2Deg) < 2.0) {
    suitable_path_length =
        std::max(suitable_path_length,
                 input_.ego_info_under_slot.slot.processed_corner_coord_local_
                         .pt_01_mid.x() -
                     1.268 - geometry_path.end_pose.pos.x());
  }

  // if (CalOccupiedRatio(geometry_path.end_pose) > 0.8 &&
  //     ref_gear == geometry_lib::SEG_GEAR_DRIVE) {
  //   suitable_path_length *= 2.0;
  // }

  ILOG_INFO << "suitable_path_length = " << suitable_path_length;
  line.length = std::max(insert_length,
                         suitable_path_length - geometry_path.total_length);
  const int sign = (ref_gear == geometry_lib::SEG_GEAR_DRIVE) ? 1.0 : -1.0;
  line.pB = line.pA + sign * line.length * line.heading_vec;
  geometry_lib::PathSegment line_seg(ref_gear, line);
  PathColDetRes res =
      TrimPathByObs(line_seg, lat_buffer, lon_buffer, enable_log);
  if (res == PathColDetRes::NORMAL || res == PathColDetRes::SHORTEN) {
    geometry_path.AddPath(line_seg);
    ILOG_INFO_IF(enable_log) << "insert line success";
    line_seg.PrintInfo(true);
    return true;
  }

  ILOG_INFO_IF(enable_log) << "insert line fail";
  return false;
}

const bool PerpendicularTailInPathGenerator::CheckStuckedByInside(
    const geometry_lib::PathPoint& start_pose,
    const geometry_lib::PathPoint& end_pose, const bool enable_log) {
  const bool case1 =
      (end_pose.pos.y() - calc_params_.target_line.pA.y()) *
          (start_pose.pos.y() - calc_params_.target_line.pA.y()) >
      0.068;
  const bool case2 =
      std::fabs(start_pose.pos.y()) > std::fabs(end_pose.pos.y());
  const bool case3 = std::fabs(end_pose.heading * kRad2Deg) > 6.8;
  ILOG_INFO_IF(enable_log) << "case1 = " << case1 << "  case2 = " << case2
                           << "  case3 = " << case3;
  return case1 && case2 && case3;
}

const bool PerpendicularTailInPathGenerator::ConstructReverseVaildPathSeg(
    geometry_lib::PathSegment& seg1, geometry_lib::PathSegment& seg2,
    const double lat_buffer, const double lon_buffer, const bool enable_log) {
  ILOG_INFO_IF(enable_log) << "ConstructReverseVaildPathSeg";
  const double trim_path_step = 0.068;
  const double origin_seg1_length = seg1.Getlength();
  size_t i = 0;
  double min_length = kMinSingleGearPathLength;
  if (!calc_params_.cur_gear_path_flag) {
    min_length = 0.0368;
  }
  for (i = 0; i < 100 && seg1.Getlength() > min_length + 1e-3; ++i) {
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
    } else {
      ILOG_INFO_IF(enable_log) << "arc2 is invalid ";
    }
  }

  return false;
}

const double PerpendicularTailInPathGenerator::CalOccupiedRatio(
    const pnc::geometry_lib::PathPoint& current_pose) {
  if (std::fabs(current_pose.pos.y()) <
          apa_param.GetParam().slot_occupied_ratio_max_lat_err &&
      std::fabs(current_pose.heading) <
          apa_param.GetParam().slot_occupied_ratio_max_heading_err * kDeg2Rad) {
    const std::vector<double> x_tab = {
        input_.ego_info_under_slot.target_pose.pos.x(),
        input_.ego_info_under_slot.slot.slot_length_ +
            apa_param.GetParam().rear_overhanging};
    const std::vector<double> occupied_ratio_tab = {1.0, 0.0};
    return mathlib::Interp1(x_tab, occupied_ratio_tab, current_pose.pos.x());
  }
  return 0.0;
}

const bool PerpendicularTailInPathGenerator::IsRightCircle(
    const pnc::geometry_lib::PathPoint& ego_pose,
    const Eigen::Vector2d& center) const {
  return IsRightCircle(ego_pose.pos, ego_pose.heading, center);
}

const bool PerpendicularTailInPathGenerator::IsRightCircle(
    const Eigen::Vector2d& ego_pos, const double ego_heading,
    const Eigen::Vector2d& center) const {
  const Eigen::Vector2d center_to_ego_vec = ego_pos - center;

  const Eigen::Vector2d ego_heading_vec(std::cos(ego_heading),
                                        std::sin(ego_heading));

  return (pnc::geometry_lib::GetCrossFromTwoVec2d(ego_heading_vec,
                                                  center_to_ego_vec) > 0.0);
}

const bool PerpendicularTailInPathGenerator::CheckTwoPoseInCircle(
    const Eigen::Vector2d& ego_pos0, const double ego_heading0,
    const Eigen::Vector2d& ego_pos1, const double ego_heading1,
    const Eigen::Vector2d& center) const {
  return IsRightCircle(ego_pos0, ego_heading0, center) &&
         IsRightCircle(ego_pos1, ego_heading1, center);
}

const PerpendicularTailInPathGenerator::PlannerParams&
PerpendicularTailInPathGenerator::GetCalcParams() {
  return calc_params_;
}

const PerpendicularTailInPathGenerator::PoseTypeRelativeToSlot
PerpendicularTailInPathGenerator::GetPoseTypeRelativeToSlot(
    const geometry_lib::PathPoint& pose) {
  const double slot_out_x = input_.ego_info_under_slot.slot.slot_length_ + 1.08;
  const double slot_in_x = input_.ego_info_under_slot.slot.slot_length_ - 1.6;

  const double car_min_x =
      collision_detector_interface_ptr_->GetGeometryCollisionDetectorPtr()
          ->CalCarRectangleBound(pose)
          .min_x;

  if (car_min_x > slot_out_x) {
    return PoseTypeRelativeToSlot::OUT_SLOT;
  } else if (car_min_x > slot_in_x) {
    return PoseTypeRelativeToSlot::OUT_IN_SLOT;
  } else {
    return PoseTypeRelativeToSlot::IN_SLOT;
  }
}

void PerpendicularTailInPathGenerator::CalcObsDistConsiderSlotForPathSeg(
    geometry_lib::PathSegment& path_seg) {
  if (path_seg.pt_obs_dist_info_vec.empty()) {
    return;
  }
  geometry_lib::ObsDistConsiderSlot obs_dist_info;
  PoseTypeRelativeToSlot type;
  double obs_dist = 0.0;
  for (const geometry_lib::Pt2ObsDistInfo& pt_obs_dist_info :
       path_seg.pt_obs_dist_info_vec) {
    obs_dist += pt_obs_dist_info.dist_pt.first;
    type = GetPoseTypeRelativeToSlot(pt_obs_dist_info.dist_pt.second);
    if (type == PoseTypeRelativeToSlot::OUT_SLOT) {
      if (pt_obs_dist_info.dist_pt.first < obs_dist_info.out_slot.first) {
        obs_dist_info.out_slot = pt_obs_dist_info.dist_pt;
      }
    } else if (type == PoseTypeRelativeToSlot::OUT_IN_SLOT) {
      if (pt_obs_dist_info.car_safe_pos ==
          geometry_lib::CarSafePos::CAR_FRONT) {
        // in slot is dangerous
        if (pt_obs_dist_info.dist_pt.first < obs_dist_info.in_slot.first) {
          obs_dist_info.in_slot = pt_obs_dist_info.dist_pt;
        }
      } else if (pt_obs_dist_info.car_safe_pos ==
                 geometry_lib::CarSafePos::CAR_REAR) {
        // out slot is dangerous
        if (pt_obs_dist_info.dist_pt.first < obs_dist_info.out_slot.first) {
          obs_dist_info.out_slot = pt_obs_dist_info.dist_pt;
        }
      }
    } else if (type == PoseTypeRelativeToSlot::IN_SLOT) {
      if (pt_obs_dist_info.dist_pt.first < obs_dist_info.in_slot.first) {
        obs_dist_info.in_slot = pt_obs_dist_info.dist_pt;
      }
    }
  }

  path_seg.average_obs_dist = obs_dist / path_seg.pt_obs_dist_info_vec.size();

  if (obs_dist_info.in_slot.first < obs_dist_info.out_slot.first) {
    obs_dist_info.integrated = obs_dist_info.in_slot;
  } else {
    obs_dist_info.integrated = obs_dist_info.out_slot;
  }
  path_seg.obs_dist_info = obs_dist_info;
}

void PerpendicularTailInPathGenerator::CalcObsDistConsiderSlotForGeometryPath(
    geometry_lib::GeometryPath& geometry_path) {
  if (geometry_path.path_count < 1) {
    return;
  }
  geometry_lib::ObsDistConsiderSlot obs_dist_info;
  double obs_dist = 0.0;
  for (geometry_lib::PathSegment& path_seg : geometry_path.path_segment_vec) {
    CalcObsDistConsiderSlotForPathSeg(path_seg);
    obs_dist += path_seg.average_obs_dist;
    if (path_seg.obs_dist_info.in_slot.first < obs_dist_info.in_slot.first) {
      obs_dist_info.in_slot = path_seg.obs_dist_info.in_slot;
    }
    if (path_seg.obs_dist_info.out_slot.first < obs_dist_info.out_slot.first) {
      obs_dist_info.out_slot = path_seg.obs_dist_info.out_slot;
    }
  }
  if (obs_dist_info.in_slot.first < obs_dist_info.out_slot.first) {
    obs_dist_info.integrated = obs_dist_info.in_slot;
  } else {
    obs_dist_info.integrated = obs_dist_info.out_slot;
  }
  geometry_path.obs_dist_info = obs_dist_info;
  geometry_path.average_obs_dist = obs_dist / geometry_path.path_count;
}

const bool PerpendicularTailInPathGenerator::CheckReachTargetPose(
    const pnc::geometry_lib::PathPoint& current_pose) {
  const double lon_err = current_pose.pos.x() - calc_params_.target_line.pA.x();
  const double lat_err =
      std::fabs(current_pose.pos.y() - calc_params_.target_line.pA.y());
  const double heading_err =
      std::fabs(pnc::geometry_lib::NormalizeAngle(
          current_pose.heading -
          input_.ego_info_under_slot.target_pose.heading)) *
      kRad2Deg;

  if (lon_err < 0.268 && lat_err < apa_param.GetParam().target_pos_err &&
      heading_err < apa_param.GetParam().target_heading_err) {
    return true;
  }
  return false;
}

const bool PerpendicularTailInPathGenerator::CheckReachTargetPose() {
  if (output_.path_segment_vec.empty()) {
    return CheckReachTargetPose(input_.ego_info_under_slot.cur_pose);
  }
  const auto& last_segment = output_.path_segment_vec.back();
  pnc::geometry_lib::PathPoint last_pose;
  if (last_segment.seg_type == pnc::geometry_lib::SEG_TYPE_ARC) {
    last_pose.Set(last_segment.GetArcSeg().pB,
                  last_segment.GetArcSeg().headingB);
  } else if (last_segment.seg_type == pnc::geometry_lib::SEG_TYPE_LINE) {
    last_pose.Set(last_segment.GetLineSeg().pB,
                  last_segment.GetLineSeg().heading);
  }

  return CheckReachTargetPose(last_pose);
}

const bool PerpendicularTailInPathGenerator::CheckCurrentGearLength() {
  if (output_.path_segment_vec.size() < 1) {
    return false;
  }
  double length = 0.0;
  for (size_t i = output_.path_seg_index.first;
       i <= output_.path_seg_index.second; ++i) {
    length += output_.path_segment_vec[i].Getlength();
  }

  return (length > kMinSingleGearPathLength) ? true : false;
}

const bool PerpendicularTailInPathGenerator::FindPtCanReverseToSlot(
    std::vector<geometry_lib::PathSegment>& seg_vec, const uint8_t gear,
    const uint8_t steer, const double radius, const double init_length,
    const double max_length, const geometry_lib::PathPoint& pose,
    const double lat_buffer, const double lon_buffer,
    const GeometryPathType type, const double step, const bool need_col_det,
    const bool should_all_path_safe, const bool should_same_gear) {
  double length = init_length;
  double sample_step = step;
  geometry_lib::PathSegment seg;
  geometry_lib::GeometryPath geometry_path;
  double sign = 1.0;
  if (init_length > max_length) {
    sample_step = -step;
    sign = -1.0;
  }
  int i = 0;
  while (length * sign < max_length * sign) {
    if (steer == geometry_lib::SEG_STEER_STRAIGHT) {
      geometry_lib::CalLineFromPt(gear, length, pose, seg);
    } else {
      geometry_lib::CalArcFromPt(gear, steer, length, radius, pose, seg);
    }

    if (need_col_det && TrimPathByObs(seg, lat_buffer, lon_buffer, false) !=
                            PathColDetRes::NORMAL) {
      return false;
    }

    if (type == GeometryPathType::ALIGNBODY_STURN) {
      if (AlignAndSTurnPathPlan(
              seg.GetEndPose(), geometry_lib::ReverseGear(gear), lat_buffer,
              lon_buffer, geometry_path, true, false, true, false)) {
        seg_vec.emplace_back(seg);
        return true;
      }
    }

    if (type == GeometryPathType::TWO_ARC) {
      bool success = false;
      if (steer == geometry_lib::SEG_STEER_STRAIGHT) {
        if (should_same_gear) {
          success =
              success ||
              (TwoArcPathPlan(seg.GetEndPose(), gear, lat_buffer, lon_buffer,
                              geometry_path, false, false, false) &&
               (!should_all_path_safe || geometry_path.all_path_safe));
        } else {
          success =
              success ||
              (TwoArcPathPlan(seg.GetEndPose(), gear, lat_buffer, lon_buffer,
                              geometry_path, false, false, false) &&
               (!should_all_path_safe || geometry_path.all_path_safe));

          success =
              success ||
              (TwoArcPathPlan(seg.GetEndPose(), geometry_lib::ReverseGear(gear),
                              lat_buffer, lon_buffer, geometry_path, false,
                              false, false) &&
               (!should_all_path_safe || geometry_path.all_path_safe));
        }

      } else {
        success =
            success ||
            (TwoArcPathPlan(seg.GetEndPose(), geometry_lib::ReverseGear(gear),
                            lat_buffer, lon_buffer, geometry_path, false, false,
                            false) &&
             (!should_all_path_safe || geometry_path.all_path_safe));
      }

      if (success) {
        if (CheckStuckedByInside(geometry_path.start_pose,
                                 geometry_path.end_pose, false)) {
          auto& arc_seg = geometry_path.path_segment_vec.back();
          CalcObsDistConsiderSlotForPathSeg(arc_seg);
          const double stuck_y =
              arc_seg.obs_dist_info.integrated.second.pos.y();
          if (arc_seg.seg_gear == geometry_lib::SEG_GEAR_REVERSE) {
            if ((arc_seg.seg_steer == geometry_lib::SEG_STEER_LEFT &&
                 stuck_y > 0.468) ||
                (arc_seg.seg_steer == geometry_lib::SEG_STEER_RIGHT &&
                 stuck_y < -0.468)) {
              success = false;
            }
          }
        }
      }

      if (success) {
        seg_vec.emplace_back(seg);
        if (geometry_path.cur_gear == gear) {
          for (size_t i = 0; i < geometry_path.path_count; ++i) {
            seg_vec.emplace_back(geometry_path.path_segment_vec[i]);
          }
        }
        return true;
      }
    }

    length += step;

    ++i;
    if (i > 68) {
      // only protect the loop
      return false;
    }
  }

  return false;
}

const bool PerpendicularTailInPathGenerator::ItervativeUpdatePb(
    const GeometryPathInput& input,
    const std::shared_ptr<CollisionDetectorInterface>&
        collision_detector_interface_ptr) {
  input_ = input;
  collision_detector_interface_ptr_ = collision_detector_interface_ptr;
  Preprocess();
  calc_params_.is_searching_stage = false;
  calc_params_.first_multi_plan = true;
  calc_params_.adjust_fail_count = 0;
  calc_params_.pre_plan_case = PrePlanCase::FAIL;
  apa_param.SetPram().actual_mono_plan_enable = true;

  if (CheckReachTargetPose()) {
    ILOG_INFO << "init pose is already at target pose";
    return true;
  }

  if (PreparePathPlan()) {
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

  if (PreparePathSecondPlan()) {
    ILOG_INFO << "second prepare plan success";
  } else {
    ILOG_INFO << "second prepare plan fail";
  }
  if (CheckReachTargetPose()) {
    ILOG_INFO << "second prepare plan to target pose";
    return true;
  }

  MultiAdjustPathPlan(input_.ego_info_under_slot.cur_pose,
                      geometry_lib::SEG_GEAR_REVERSE,
                      PlanRequest::OPTIMAL_PATH);

  PrintOutputSegmentsInfo();

  return CheckReachTargetPose() || output_.path_segment_vec.size() > 0;
}

}  // namespace apa_planner
}  // namespace planning