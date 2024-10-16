#include "ifly_time.h"
#include "perpendicular_path_in_planner.h"

namespace planning {
namespace apa_planner {

static const double kMaxArcLength = 12.68;
static const double kMinArcLength = 0.0168;
static const double kMinSingleGearPathLength = 0.35;
static const bool kUseEDTColDet = true;

const PerpendicularPathInPlanner::PathColDetRes
PerpendicularPathInPlanner::TrimPathByObs(
    pnc::geometry_lib::PathSegment& path_seg, const double lat_inflation,
    const double lon_safe_dist) {
  const double time = IflyTime::Now_ms();

  if (path_seg.seg_type != geometry_lib::SEG_TYPE_LINE &&
      path_seg.seg_type != geometry_lib::SEG_TYPE_ARC) {
    return PathColDetRes::INVALID;
  }

  CollisionDetector::CollisionResult col_res;
  if (kUseEDTColDet) {
    col_res = collision_detector_ptr_->UpdateByEDT(path_seg, lat_inflation,
                                                   lon_safe_dist);
  } else {
    col_res = collision_detector_ptr_->UpdateByObsMap(path_seg, lat_inflation,
                                                      lon_safe_dist);
  }

  calc_params_.statistical_time += IflyTime::Now_ms() - time;

  const double remain_car_dist = col_res.remain_car_dist;
  const double remain_obs_dist = col_res.remain_obstacle_dist;
  // the dist that the car can go
  const double safe_remain_dist = col_res.remain_dist;

  ILOG_INFO << "remain_car_dist = " << remain_car_dist
            << "  remain_obs_dist = " << remain_obs_dist
            << "  safe_remain_dist = " << safe_remain_dist;

  if (safe_remain_dist < 0.016) {
    path_seg.collision_flag = true;
    geometry_lib::CompletePathSegInfo(path_seg, 1e-3);
    ILOG_INFO << "this path is invalid";
    return PathColDetRes::INVALID;
  }

  // if you only hit a little bit, consider it safe, because it has lon safe
  // dist
  if (safe_remain_dist + 0.016 > remain_car_dist) {
    path_seg.collision_flag = false;
    ILOG_INFO << "this path is normal";
    return PathColDetRes::NORMAL;
  }

  // the path would col, but can trim it to keep the path safe
  geometry_lib::CompletePathSegInfo(path_seg, safe_remain_dist);

  ILOG_INFO << "this path is shorten";
  path_seg.collision_flag = true;
  return PathColDetRes::SHORTEN;
}

const bool PerpendicularPathInPlanner::RoughMultiAdjustPathPlan(
    const geometry_lib::PathPoint& pose, const uint8_t ref_gear) {
  ILOG_INFO << " --- enter RoughMultiAdjustPathPlan --- ";

  // plan a successful path to exit, suitable for the search stage
  uint8_t single_ref_gear = ref_gear;
  geometry_lib::PathPoint single_cur_pose = pose;
  std::vector<geometry_lib::PathPoint> cur_pose_vec{single_cur_pose};

  const double time_start = IflyTime::Now_ms();

  std::vector<geometry_lib::GeometryPath> geometry_path_vec;
  std::vector<geometry_lib::GeometryPath> success_geometry_path_vec;
  geometry_lib::GeometryPath geometry_path;
  geometry_lib::GeometryPath geometry_path_copy;

  for (size_t i = 0; i < 6; ++i) {
    ILOG_INFO << i << "th try RoughMultiAdjustPathPlan ";

    double lat_buffer = apa_param.GetParam().car_lat_inflation_normal;
    double lon_buffer = apa_param.GetParam().col_obs_safe_dist_normal;

    if (calc_params_.first_multi_plan && i == 0) {
      lat_buffer = apa_param.GetParam().car_lat_inflation_strict;
      lon_buffer = apa_param.GetParam().col_obs_safe_dist_strict;
    }

    for (const geometry_lib::PathPoint& cur_pose : cur_pose_vec) {
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
        // when is single cur pose, directly try ego pose to one line plan, just
        // seeking one more solution, only i=0, first plan
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
            break;
          } else {
            geometry_path_vec.emplace_back(geometry_path);
          }
        }
      }

      for (const geometry_lib::GeometryPath& temp_geometry_path :
           temp_geometry_path_vec) {
        geometry_path = geometry_path_copy;

        // when single first plan, try insert line to make easy control
        geometry_path.AddPath(temp_geometry_path.path_segment_vec);

        geometry_lib::GeometryPath one_line_geometry_path;
        double last_seg_reverse_length = 0.0;
        if (temp_geometry_path.cur_gear == geometry_lib::SEG_GEAR_REVERSE) {
          last_seg_reverse_length = temp_geometry_path.total_length;
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

      ILOG_INFO << "geometry_path_vec_size = " << geometry_path_vec.size()
                << "  temp_geometry_path_vec = "
                << temp_geometry_path_vec.size()
                << "  success_geometry_path_vec = "
                << success_geometry_path_vec.size();

      if (success_geometry_path_vec.size() > 0) {
        break;
      }
    }

    if (geometry_path_vec.empty() || success_geometry_path_vec.size() > 0) {
      break;
    }

    cur_pose_vec.clear();

    for (const geometry_lib::GeometryPath& geometry_path : geometry_path_vec) {
      cur_pose_vec.emplace_back(geometry_path.end_pose);
    }

    single_ref_gear = geometry_lib::ReverseGear(single_ref_gear);
  }

  ILOG_INFO << "RoughMultiAdjustPathPlan consume time = "
            << IflyTime::Now_ms() - time_start
            << "ms  col consume time = " << calc_params_.statistical_time
            << "ms";

  if (success_geometry_path_vec.empty()) {
    ILOG_INFO << "there is no success plan path to target line\n";
    return false;
  } else {
    ILOG_INFO << "there are success plan path to target line, num = "
              << success_geometry_path_vec.size();
    return true;
  }
}

const bool PerpendicularPathInPlanner::OneStepMultiAdjustPathPlan(
    const geometry_lib::PathPoint& pose, const uint8_t ref_gear) {
  ILOG_INFO << "--- enter OneStepMultiAdjustPathPlan --- ";

  // only looking at whether the 1R stage can be stored in one step, suitable
  // for selecting the optimal cut-off point during the formal parking pre
  // planning stage

  const double time_start = IflyTime::Now_ms();

  for (size_t i = 0; i < 1; ++i) {
    ILOG_INFO << i << "th try OneStepMultiAdjustPathPlan";

    const double lat_buffer = apa_param.GetParam().car_lat_inflation_strict;
    const double lon_buffer = apa_param.GetParam().col_obs_safe_dist_strict;

    geometry_lib::GeometryPath geometry_path;
    if (OneLinePathPlan(pose, ref_gear,
                        apa_param.GetParam().car_lat_inflation_normal,
                        apa_param.GetParam().col_obs_safe_dist_normal, 0.0,
                        geometry_path)) {
      if (geometry_path.cur_gear == geometry_lib::SEG_GEAR_REVERSE) {
        ILOG_INFO << "one line path success and gear is reverse";
        return true;
      }
    }

    std::vector<geometry_lib::GeometryPath> geometry_path_vec;
    SingleMultiAdjustPathPlan(pose, ref_gear, lat_buffer, lon_buffer,
                              geometry_path_vec);

    for (geometry_lib::GeometryPath& geometry_path : geometry_path_vec) {
      double last_seg_reverse_length = 0.0;
      if (geometry_path.cur_gear == geometry_lib::SEG_GEAR_REVERSE) {
        last_seg_reverse_length = geometry_path.total_length;
      }
      ILOG_INFO << "continue use single multi_adjust plan end pose to one "
                   "line path plan, _pose = "
                << geometry_path.end_pose.pos.transpose() << "  "
                << geometry_path.end_pose.heading * kRad2Deg;
      geometry_lib::GeometryPath one_line_geometry_path;
      if (OneLinePathPlan(geometry_path.end_pose, ref_gear,
                          apa_param.GetParam().car_lat_inflation_normal,
                          apa_param.GetParam().col_obs_safe_dist_normal,
                          last_seg_reverse_length, one_line_geometry_path)) {
        geometry_path.AddPath(one_line_geometry_path.path_segment_vec);
        if (geometry_path.cur_gear_length > kMinSingleGearPathLength &&
            one_line_geometry_path.cur_gear == geometry_lib::SEG_GEAR_REVERSE) {
          ILOG_INFO << "one line path success and gear is reverse";
          return true;
        }
      }
    }
  }

  ILOG_INFO << "OneStepMultiAdjustPathPlan consume time = "
            << IflyTime::Now_ms() - time_start
            << "ms  col consume time = " << calc_params_.statistical_time
            << "ms";

  return false;
}

const bool PerpendicularPathInPlanner::OptimalMultiAdjustPathPlan(
    const geometry_lib::PathPoint& pose, const uint8_t ref_gear) {
  ILOG_INFO << "--- enter OptimalMultiAdjustPathPlan --- ";

  // plan all paths based on the current pose and select the optimal path

  uint8_t single_ref_gear = ref_gear;
  geometry_lib::PathPoint single_cur_pose = pose;
  std::vector<geometry_lib::PathPoint> cur_pose_vec{single_cur_pose};

  const double time_start = IflyTime::Now_ms();

  bool extra_try_flag = false;
  int i = 0;
  const int max_compensate_line_try_count = 2;
  int compensate_line_try_count = 0;
  const double compensate_line_length_step = 0.5;
  const size_t max_path_count = 5;
  std::vector<geometry_lib::GeometryPath> geometry_path_vec;
  std::vector<geometry_lib::GeometryPath> success_geometry_path_vec;
  geometry_lib::GeometryPath geometry_path;
  geometry_lib::GeometryPath geometry_path_copy;

  for (i = 0; i < 8 && success_geometry_path_vec.size() < max_path_count; ++i) {
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
    if (calc_params_.first_multi_plan && i == 0) {
      for (const geometry_lib::GeometryPath& geometry_path :
           geometry_path_vec) {
        if (geometry_path.collide_flag &&
            std::fabs(geometry_path.end_pose.pos.y()) > 0.86 &&
            std::fabs(geometry_path.end_pose.heading * kRad2Deg) > 10.0 &&
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
          geometry_lib::GeometryPath temp_geometry_path(line_seg);
          temp_geometry_path.PrintInfo();
          geometry_path_vec.emplace_back(temp_geometry_path);
          ILOG_INFO
              << "set i is -1, and continuing a reverse gear straight line";
          continue;
        }
      }
    }

    if (geometry_path_vec.empty()) {
      if (i == 0 && !extra_try_flag) {
        // try use reverse gear to plan, it is not good, only try
        i = -1;
        extra_try_flag = true;
        single_ref_gear = geometry_lib::ReverseGear(single_ref_gear);
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
            << "ms  col consume time = " << calc_params_.statistical_time
            << "ms";

  if (success_geometry_path_vec.empty()) {
    ILOG_INFO << "there is no success plan path to target line\n";
    return false;
  }

  ILOG_INFO << "there are success plan path to target line, num = "
            << success_geometry_path_vec.size();

  // 怎么选呢 挡位？ 路径长度？ 后续需要加上碰撞代价吗？
  int optimal_path_index = -1;
  const double gear_change_cost = 10.0;
  const double unit_length_cost = 1.0;
  double min_cost = std::numeric_limits<double>::infinity();
  for (int k = 0; k < success_geometry_path_vec.size(); ++k) {
    double cost = 0.0;
    ILOG_INFO << "gear_change_count = "
              << static_cast<int>(
                     success_geometry_path_vec[k].gear_change_count)
              << "  total_length = "
              << success_geometry_path_vec[k].total_length;
    cost += gear_change_cost * success_geometry_path_vec[k].gear_change_count;
    cost += unit_length_cost * success_geometry_path_vec[k].total_length;
    if (cost < min_cost) {
      optimal_path_index = k;
      min_cost = cost;
    }
  }

  geometry_lib::GeometryPath optimal_geometry_path =
      success_geometry_path_vec[optimal_path_index];

  optimal_geometry_path.PrintInfo();

  output_.path_available = true;
  for (size_t i = 0; i < optimal_geometry_path.path_count; ++i) {
    output_.gear_cmd_vec.emplace_back(optimal_geometry_path.gear_cmd_vec[i]);
    output_.path_segment_vec.emplace_back(
        optimal_geometry_path.path_segment_vec[i]);
    output_.steer_vec.emplace_back(optimal_geometry_path.steer_cmd_vec[i]);
  }

  if (!output_.gear_cmd_vec.empty()) {
    output_.current_gear = output_.gear_cmd_vec.front();
  }

  return true;
}

const bool PerpendicularPathInPlanner::MultiAdjustPathPlan(
    const geometry_lib::PathPoint& pose, const uint8_t ref_gear,
    const PlanRequest plan_request) {
  ILOG_INFO << "\n\n --- enter MultiAdjustPathPlan --- ";

  // 根据 plan_request 有三种情况
  // 1. 规划出一条成功路径即退出， 适用于寻库阶段
  // 2. 只看1R阶段 是否能一把入库，适用于正式泊车预规划阶段选择一个最优的切点
  // 3. 根据当前车辆位姿规划出所有路径，选择一条最优路径
  if (plan_request == PlanRequest::ROUGH_PATH) {
    return RoughMultiAdjustPathPlan(pose, ref_gear);
  } else if (plan_request == PlanRequest::ONE_STEP_PATH) {
    return OneStepMultiAdjustPathPlan(pose, ref_gear);
  } else if (plan_request == PlanRequest::OPTIMAL_PATH) {
    return OptimalMultiAdjustPathPlan(pose, ref_gear);
  } else {
    return false;
  }

  uint8_t single_ref_gear = ref_gear;
  geometry_lib::PathPoint single_cur_pose = pose;  // 正式1R的初始位置
  std::vector<geometry_lib::PathPoint> cur_pose_vec{pose};

  const double time_start = IflyTime::Now_ms();

  bool extra_try_flag = false;
  int i = 0;
  const int max_compensate_line_try_count = 2;
  int compensate_line_try_count = 0;
  const double compensate_line_length_step = 0.5;
  std::vector<geometry_lib::GeometryPath> geometry_path_vec;
  std::vector<geometry_lib::GeometryPath> success_geometry_path_vec;
  geometry_lib::GeometryPath geometry_path;
  geometry_lib::GeometryPath geometry_path_copy;

  for (i = 0; i < 8 && success_geometry_path_vec.size() < 3; ++i) {
    ILOG_INFO
        << i
        << "th try MultiAdjustPathPlan and success_geometry_path_vec_size = "
        << success_geometry_path_vec.size();

    double lat_buffer = apa_param.GetParam().car_lat_inflation_normal;
    double lon_buffer = apa_param.GetParam().col_obs_safe_dist_normal;

    if (calc_params_.first_multi_plan && i == 0) {
      lat_buffer = apa_param.GetParam().car_lat_inflation_strict;
      lon_buffer = apa_param.GetParam().col_obs_safe_dist_strict;
    }

    for (const geometry_lib::PathPoint& cur_pose : cur_pose_vec) {
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
    if (calc_params_.first_multi_plan && i == 0) {
      for (const geometry_lib::GeometryPath& geometry_path :
           geometry_path_vec) {
        if (geometry_path.collide_flag &&
            std::fabs(geometry_path.end_pose.pos.y()) > 0.86 &&
            std::fabs(geometry_path.end_pose.heading * kRad2Deg) > 10.0 &&
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
          geometry_lib::GeometryPath temp_geometry_path(line_seg);
          temp_geometry_path.PrintInfo();
          geometry_path_vec.emplace_back(temp_geometry_path);
          ILOG_INFO
              << "set i is -1, and continuing a reverse gear straight line";
          continue;
        }
      }
    }

    if (geometry_path_vec.empty()) {
      if (i == 0 && !extra_try_flag) {
        // try use reverse gear to plan, it is not good, only try
        i = -1;
        extra_try_flag = true;
        single_ref_gear = geometry_lib::ReverseGear(single_ref_gear);
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

  ILOG_INFO << "MultiAdjustPathPlan consume time = "
            << IflyTime::Now_ms() - time_start
            << "ms  col consume time = " << calc_params_.statistical_time
            << "ms";

  if (success_geometry_path_vec.empty()) {
    ILOG_INFO << "there is no success plan path to target line\n";
    return false;
  }

  ILOG_INFO << "there are success plan path to target line, num = "
            << success_geometry_path_vec.size();

  // 怎么选呢 挡位？ 路径长度？ 后续需要加上碰撞代价吗？
  int optimal_path_index = -1;
  const double gear_change_cost = 10.0;
  const double unit_length_cost = 1.0;
  double min_cost = std::numeric_limits<double>::infinity();
  for (int k = 0; k < success_geometry_path_vec.size(); ++k) {
    double cost = 0.0;
    ILOG_INFO << "gear_change_count = "
              << static_cast<int>(
                     success_geometry_path_vec[k].gear_change_count)
              << "  total_length = "
              << success_geometry_path_vec[k].total_length;
    cost += gear_change_cost * success_geometry_path_vec[k].gear_change_count;
    cost += unit_length_cost * success_geometry_path_vec[k].total_length;
    if (cost < min_cost) {
      optimal_path_index = k;
      min_cost = cost;
    }
  }

  geometry_lib::GeometryPath optimal_geometry_path =
      success_geometry_path_vec[optimal_path_index];

  optimal_geometry_path.PrintInfo();

  output_.path_available = true;
  for (size_t i = 0; i < optimal_geometry_path.path_count; ++i) {
    output_.gear_cmd_vec.emplace_back(optimal_geometry_path.gear_cmd_vec[i]);
    output_.path_segment_vec.emplace_back(
        optimal_geometry_path.path_segment_vec[i]);
    output_.steer_vec.emplace_back(optimal_geometry_path.steer_cmd_vec[i]);
  }

  if (!output_.gear_cmd_vec.empty()) {
    output_.current_gear = output_.gear_cmd_vec.front();
  }

  return true;
}

const bool PerpendicularPathInPlanner::SingleMultiAdjustPathPlan(
    const geometry_lib::PathPoint& pose, const uint8_t ref_gear,
    const double lat_buffer, const double lon_buffer,
    std::vector<geometry_lib::GeometryPath>& geometry_path_vec) {
  geometry_path_vec.clear();
  ILOG_INFO << "\n --- enter SingleMultiAdjustPathPlan ---";
  ILOG_INFO << "pos = " << pose.pos.transpose()
            << "  heading = " << pose.heading * kRad2Deg
            << "  ref gear = " << static_cast<int>(ref_gear);

  geometry_lib::GeometryPath geometry_path;
  if (OneArcPathPlan(pose, ref_gear, lat_buffer, lon_buffer, geometry_path)) {
    geometry_path_vec.emplace_back(geometry_path);
  }

  if (TwoArcPathPlan(pose, ref_gear, lat_buffer, lon_buffer, geometry_path,
                     false)) {
    geometry_path_vec.emplace_back(geometry_path);
  }

  if (TwoArcPathPlan(pose, ref_gear, lat_buffer, lon_buffer, geometry_path,
                     true)) {
    geometry_path_vec.emplace_back(geometry_path);
  }

  if (LineArcPathPlan(pose, ref_gear, lat_buffer, lon_buffer, geometry_path,
                      true)) {
    geometry_path_vec.emplace_back(geometry_path);
  }

  if (LineArcPathPlan(pose, ref_gear, lat_buffer, lon_buffer, geometry_path,
                      false)) {
    geometry_path_vec.emplace_back(geometry_path);
  }

  if (AlignAndSTurnPathPlan(pose, ref_gear, lat_buffer, lon_buffer,
                            geometry_path)) {
    geometry_path_vec.emplace_back(geometry_path);
  }

  return geometry_path_vec.size() > 0;
}

const bool PerpendicularPathInPlanner::OneArcPathPlan(
    const geometry_lib::PathPoint& pose, const uint8_t ref_gear,
    const double lat_buffer, const double lon_buffer,
    geometry_lib::GeometryPath& geometry_path) {
  ILOG_INFO << "\n----enter one arc path plan----";
  geometry_path.Reset();
  geometry_lib::Arc arc;
  arc.pA = pose.pos;
  arc.headingA = pose.heading;

  bool success = geometry_lib::CalOneArcWithLineAndGear(
      arc, calc_params_.target_line, ref_gear);

  success = success && arc.length < kMaxArcLength && arc.length > kMinArcLength;

  if (!success) {
    return false;
  }

  const uint8_t gear = geometry_lib::CalArcGear(arc);
  const uint8_t steer = geometry_lib::CalArcSteer(arc);

  success =
      geometry_lib::IsSameGear(gear, ref_gear) &&
      arc.circle_info.radius >
          calc_params_.turn_radius - apa_param.GetParam().target_radius_err &&
      arc.circle_info.radius < apa_param.GetParam().max_one_step_arc_radius;

  if (!success) {
    ILOG_INFO << "one arc path plan fail\n";
    return false;
  }

  geometry_lib::PathSegment arc_seg(steer, gear, arc);

  PathColDetRes res = TrimPathByObs(arc_seg, lat_buffer, lon_buffer);

  if (res == PathColDetRes::INVALID) {
    ILOG_INFO << "one arc path col fail\n";
    return false;
  }

  if (res == PathColDetRes::SHORTEN) {
    ILOG_INFO << "one arc path col shorten\n";
  }

  if (res == PathColDetRes::NORMAL) {
    ILOG_INFO << "one arc path no col\n";
  }

  geometry_path.SetPath(arc_seg);

  if (geometry_path.path_count > 0) {
    ILOG_INFO << "one arc plan has path";
    geometry_path.PrintInfo();
    return true;
  } else {
    ILOG_INFO << "one arc plan has no path";
    return false;
  }

  return true;
}

const bool PerpendicularPathInPlanner::LineArcPathPlan(
    const geometry_lib::PathPoint& pose, const uint8_t ref_gear,
    const double lat_buffer, const double lon_buffer,
    geometry_lib::GeometryPath& geometry_path, const bool same_gear) {
  ILOG_INFO << "\n----enter line " << same_gear << "arc path plan----";
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
    ILOG_INFO << "Cal line arc plan has no path";
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

    PathColDetRes col_res1 = TrimPathByObs(line_seg, lat_buffer, lon_buffer);

    if (same_gear) {
      if (col_res1 != PathColDetRes::NORMAL) {
        ILOG_INFO << "same gear, line col, quit";
        continue;
      } else {
        ILOG_INFO << "same gear, line is safe and then check arc col, add line "
                     "to path";
      }

      std::vector<geometry_lib::PathSegment> path_seg_vec{line_seg};

      PathColDetRes col_res2 = TrimPathByObs(arc_seg, lat_buffer, lon_buffer);

      if (col_res2 == PathColDetRes::INVALID) {
        ILOG_INFO << "same gear, arc invalid, quit";
        continue;
      }

      if (col_res2 == PathColDetRes::NORMAL) {
        ILOG_INFO << "same gear, arc normal, add arc to path";
      }

      if (col_res2 == PathColDetRes::SHORTEN) {
        ILOG_INFO << "same gear, arc shorten, add arc to path";
      }

      path_seg_vec.emplace_back(arc_seg);
      geometry_path.SetPath(path_seg_vec);

      break;
    }

    else {
      if (col_res1 == PathColDetRes::INVALID) {
        ILOG_INFO << "opposite gear, line invalid, quit";
        continue;
      }

      if (col_res1 == PathColDetRes::NORMAL) {
        ILOG_INFO << "opposite gear, line normal, then check arc col";
        PathColDetRes col_res2 = TrimPathByObs(arc_seg, lat_buffer, lon_buffer);
        if (col_res2 == PathColDetRes::NORMAL ||
            col_res2 == PathColDetRes::SHORTEN) {
          ILOG_INFO << "arc col is normal or shorten, add line to path";
          std::vector<geometry_lib::PathSegment> path_seg_vec{line_seg,
                                                              arc_seg};
          // geometry_path.SetPath(path_seg_vec);
          geometry_path.SetPath(line_seg);
          break;
        }

        if (col_res2 == PathColDetRes::INVALID) {
          ILOG_INFO << "arc col is invalid, then construct line to sure safe";
          if (ConstructReverseVaildPathSeg(line_seg, arc_seg, lat_buffer,
                                           lon_buffer)) {
            ILOG_INFO << "construct line success, add line to path";
            geometry_path.SetPath(line_seg);
            geometry_path.collide_flag = true;
            break;
          }
        }
      }

      if (col_res1 == PathColDetRes::SHORTEN) {
        ILOG_INFO << "opposite gear, line shorten, then construct line to "
                     "sure safe";
        if (ConstructReverseVaildPathSeg(line_seg, arc_seg, lat_buffer,
                                         lon_buffer)) {
          ILOG_INFO << "construct line success, add line to path";
          geometry_path.SetPath(line_seg);
          break;
        }
      }
    }
  }

  if (geometry_path.path_count > 0) {
    ILOG_INFO << "line arc plan has path";
    geometry_path.PrintInfo();
    return true;
  } else {
    ILOG_INFO << "line arc plan has no path";
    return false;
  }
}

const bool PerpendicularPathInPlanner::TwoArcPathPlan(
    const geometry_lib::PathPoint& pose, const uint8_t ref_gear,
    const double lat_buffer, const double lon_buffer,
    geometry_lib::GeometryPath& geometry_path, const bool same_gear) {
  // if same_gear is true, the arc1 and arc2 should have same gear, otherwise
  // must opposite gear
  ILOG_INFO << "\n----enter two " << same_gear << " arc path plan----";
  geometry_path.Reset();

  std::vector<std::pair<geometry_lib::Arc, geometry_lib::Arc>> arc_pair_vec;
  bool success = geometry_lib::CalTwoArcWithLine(
      pose, calc_params_.target_line, calc_params_.turn_radius,
      calc_params_.turn_radius, arc_pair_vec);

  if (!success) {
    ILOG_INFO << "Cal two arc plan has no path";
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

    PathColDetRes col_res1 = TrimPathByObs(arc1_seg, lat_buffer, lon_buffer);

    if (same_gear) {
      if (col_res1 != PathColDetRes::NORMAL) {
        ILOG_INFO << "same gear, arc1 col, quit\n";
        continue;
      } else {
        ILOG_INFO << "same gear, arc1 is safe and then check arc col, add arc1 "
                     "to path";
      }
      std::vector<geometry_lib::PathSegment> path_seg_vec{arc1_seg};

      PathColDetRes col_res2 = TrimPathByObs(arc2_seg, lat_buffer, lon_buffer);

      if (col_res2 == PathColDetRes::INVALID) {
        ILOG_INFO << "same gear, arc2 invalid, quit\n";
        continue;
      }

      if (col_res2 == PathColDetRes::NORMAL) {
        ILOG_INFO << "same gear, arc2 normal, add arc2 to path";
      }

      if (col_res2 == PathColDetRes::SHORTEN) {
        ILOG_INFO << "same gear, arc2 shorten, add arc2 to path";
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
        ILOG_INFO << "opposite gear, arc1 invalid, quit\n";
        continue;
      }

      if (col_res1 == PathColDetRes::NORMAL) {
        ILOG_INFO << "opposite gear, arc1 normal, then check arc2 col\n";
        PathColDetRes col_res2 =
            TrimPathByObs(arc2_seg, lat_buffer, lon_buffer);
        if (col_res2 == PathColDetRes::NORMAL ||
            col_res2 == PathColDetRes::SHORTEN) {
          ILOG_INFO << "arc2 col is normal or shorten, add arc1 to path";
          std::vector<geometry_lib::PathSegment> path_seg_vec{arc1_seg,
                                                              arc2_seg};
          // geometry_path.SetPath(path_seg_vec);
          geometry_path.SetPath(arc1_seg);
          break;
        }

        if (col_res2 == PathColDetRes::INVALID) {
          ILOG_INFO << "arc2 col is invalid, then construct arc1 to sure safe";
          if (ConstructReverseVaildPathSeg(arc1_seg, arc2_seg, lat_buffer,
                                           lon_buffer)) {
            ILOG_INFO << "construct arc1 success, add arc1 to path";
            geometry_path.SetPath(arc1_seg);
            geometry_path.collide_flag = true;
            break;
          }
        }
      }

      if (col_res1 == PathColDetRes::SHORTEN) {
        ILOG_INFO << "arc1 col is shorten then construct arc1 to sure safe";
        if (ConstructReverseVaildPathSeg(arc1_seg, arc2_seg, lat_buffer,
                                         lon_buffer)) {
          ILOG_INFO << "construct arc1 success, add arc1 to path";
          geometry_path.SetPath(arc1_seg);
          break;
        }
      }
    }
  }

  if (geometry_path.path_count > 0) {
    ILOG_INFO << "two arc plan has path";
    geometry_path.PrintInfo();
    return true;
  } else {
    ILOG_INFO << "two arc plan has no path";
    return false;
  }
}

const bool PerpendicularPathInPlanner::AlignAndSTurnPathPlan(
    const geometry_lib::PathPoint& pose, const uint8_t ref_gear,
    const double lat_buffer, const double lon_buffer,
    geometry_lib::GeometryPath& geometry_path) {
  ILOG_INFO << "\n----enter align and s_turn path plan----";
  geometry_path.Reset();
  bool success = false;
  const double tar_heading = calc_params_.target_line.heading;
  double cur_heading = pose.heading;
  bool align_arc_vaild = false;
  if (std::fabs(geometry_lib::NormalizeAngle(cur_heading - tar_heading)) <
      0.1 * kDeg2Rad) {
    ILOG_INFO << "force cur heading equal to tar_heading";
    cur_heading = tar_heading;
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
      ILOG_INFO << "align body false";
      return false;
    }
    ILOG_INFO << "align body success";

    geometry_lib::PathSegment arc_seg(steer, gear, arc);
    if (TrimPathByObs(arc_seg, lat_buffer, lon_buffer) !=
        PathColDetRes::NORMAL) {
      return false;
    }

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

  if (std::fabs(arc_s_1.pA.y()) < 1e-2) {
    ILOG_INFO << "pA is particularly close to the target line, no need to "
                 "use s turn";
    return false;
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
              (TrimPathByObs(arc_seg1, lat_buffer, lon_buffer) ==
               PathColDetRes::NORMAL) &&
              (TrimPathByObs(arc_seg2, lat_buffer, lon_buffer) ==
               PathColDetRes::NORMAL);

    if (success) {
      geometry_path.AddPath(
          std::vector<geometry_lib::PathSegment>{arc_seg1, arc_seg2});
      break;
    }
  }

  if (success) {
    ILOG_INFO << "AlignAndSTurnPathPlan has path";
    geometry_path.PrintInfo();
  } else {
    ILOG_INFO << "AlignAndSTurnPathPlan has no path";
  }

  return success;
}

const bool PerpendicularPathInPlanner::OneLinePathPlan(
    const geometry_lib::PathPoint& pose, const uint8_t ref_gear,
    const double lat_buffer, const double lon_buffer,
    const double last_seg_reverse_length,
    geometry_lib::GeometryPath& geometry_path) {
  geometry_path.Reset();
  ILOG_INFO << "\n----enter one line path plan----";
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
      // 尽量使车身出去 从而往后比较容易摆正
      const Eigen::Vector2d end_pos(4.68, start_pos.y());
      geometry_lib::LineSegment line(start_pos, end_pos, pose.heading);
      const uint8_t seg_gear = geometry_lib::CalLineSegGear(line);

      if (geometry_lib::IsSameGear(seg_gear, line_gear)) {
        geometry_lib::PathSegment line_seg(seg_gear, line);

        TrimPathByObs(line_seg, lat_buffer, lon_buffer);

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

        PathColDetRes res = TrimPathByObs(line_seg, lat_buffer, lon_buffer);

        if (ref_gear == geometry_lib::SEG_GEAR_DRIVE) {
          if (line_seg.Getlength() < kMinSingleGearPathLength + 1e-3) {
            // 强制往前规划使长度满足要求
            end_pos = start_pos +
                      (kMinSingleGearPathLength + 1e-3) * line.heading_vec;
            end_pos.x() = std::max(end_pos.x(), 4.68);
            line_seg.seg_gear = ref_gear;
            line_seg.line_seg.SetPoints(start_pos, end_pos);
            res = TrimPathByObs(line_seg, lat_buffer, lon_buffer);
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
    ILOG_INFO << "one line plan has path";
    geometry_path.PrintInfo();
    return true;
  } else {
    ILOG_INFO << "one line plan has no path";
    return false;
  }
}

const bool PerpendicularPathInPlanner::InsertLineInGeometryPath(
    const double lat_buffer, const double lon_buffer, const uint8_t ref_gear,
    const double insert_length, geometry_lib::GeometryPath& geometry_path) {
  if (geometry_path.path_segment_vec.size() < 1 ||
      !geometry_lib::IsSameGear(ref_gear, geometry_path.cur_gear) ||
      geometry_path.collide_flag || geometry_path.gear_change_count > 1) {
    ILOG_INFO << "insert line condition can not meet need 1";
    return false;
  }

  if (geometry_path.total_length >
          apa_param.GetParam().min_one_step_path_length &&
      geometry_path.path_segment_vec.back().seg_type ==
          geometry_lib::SEG_TYPE_LINE) {
    ILOG_INFO << "insert line condition can not meet need 2";
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
  PathColDetRes res = TrimPathByObs(line_seg, lat_buffer, lon_buffer);
  if (res == PathColDetRes::NORMAL || res == PathColDetRes::SHORTEN) {
    geometry_path.AddPath(line_seg);
    ILOG_INFO << "insert line success";
    PrintSegmentInfo(line_seg);
    return true;
  }

  ILOG_INFO << "insert line fail";
  return false;
}

const bool PerpendicularPathInPlanner::ConstructReverseVaildPathSeg(
    geometry_lib::PathSegment& seg1, geometry_lib::PathSegment& seg2,
    const double lat_buffer, const double lon_buffer) {
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

    PathColDetRes col_res2 = TrimPathByObs(seg2, lat_buffer, lon_buffer);

    if (col_res2 == PathColDetRes::NORMAL ||
        col_res2 == PathColDetRes::SHORTEN) {
      ILOG_INFO << "re construct path seg success, trim path = "
                << i * trim_path_step;
      return true;
    }
  }

  return false;
}

const bool PerpendicularPathInPlanner::ItervativeUpdatePb(
    const Input& input,
    const std::shared_ptr<CollisionDetector>& collision_detector_ptr) {
  input_ = input;
  collision_detector_ptr_ = collision_detector_ptr;
  Preprocess();
  calc_params_.is_searching_stage = false;
  input_.is_simulation = true;
  apa_param.SetPram().actual_mono_plan_enable = true;

  if (CheckReachTargetPose()) {
    ILOG_INFO << "init pose is already at target pose";
    return true;
  }

  if (PreparePathPlan()) {
    calc_params_.adjust_fail_count = 0;
    calc_params_.first_multi_plan = true;
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

  if (PreparePathPlanSecond()) {
    ILOG_INFO << "second prepare plan success";
  } else {
    ILOG_INFO << "second prepare  plan fail";
  }
  if (CheckReachTargetPose()) {
    ILOG_INFO << "second prepare plan to target pose";
    return true;
  }

  return MultiAdjustPathPlan(input_.ego_pose, geometry_lib::SEG_GEAR_REVERSE,
                             PlanRequest::OPTIMAL_PATH);

  if (MultiPlan()) {
    ILOG_INFO << "multi plan success";
  }

  if (CheckReachTargetPose()) {
    ILOG_INFO << "multi plan to target pose";
    return true;
  }

  if (AdjustPlan()) {
    ILOG_INFO << "adjust plan success";
    return true;
  }
  if (CheckReachTargetPose()) {
    ILOG_INFO << "adjust plan to target pose";
    return true;
  }

  return false;
}

}  // namespace apa_planner
}  // namespace planning