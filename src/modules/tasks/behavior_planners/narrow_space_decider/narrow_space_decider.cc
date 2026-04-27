#include "narrow_space_decider.h"

#include "ego_state_manager.h"
#include "environmental_model.h"
#include "planning_context.h"

namespace planning {

constexpr double kMaxNarrowSpaceWidth = 1.6;
constexpr double kMinNarrowSpaceWidth = 0.4;
constexpr double kBlockedNarrowSpaceWidth = 0.2;
constexpr double kMinNarrowSpaceLength = 1.0;
constexpr double kPreviewFrontNarrowSpaceDistance = 6.0;
constexpr double kPreviewBackNarrowSpaceDistance = 0.3;
constexpr double kMaxCareNarrowSpaceWidth = 5.0;
constexpr double kMaxNarrowSpaceRelativeAngle = 15.0;
constexpr double kMaxNarrowSpaceDrivingDistance = 50.0;
constexpr double kSamplingNarrowSpaceLength = 15.0;
constexpr double kSamplingNarrowSpaceLatGap = 0.05;
constexpr double kSamplingNarrowSpaceLonGap = 0.5;
constexpr double kSamplingNarrowSpaceLonStep = 0.1;
constexpr double kNarrowSpaceWidthBuffer = 0.2;
constexpr double kNarrowSpacesLonGap = 2.0;
constexpr double kCompleteLonBuffer = 0.1;

NarrowSpaceDecider::NarrowSpaceDecider(
    const EgoPlanningConfigBuilder* config_builder, framework::Session* session)
    : Task(config_builder, session) {
  name_ = "NarrowSpaceDecider";
  config_ = config_builder->cast<NarrowSpaceDeciderConfig>();
  const auto& vehicle_param =
      VehicleConfigurationContext::Instance()->get_vehicle_param();
  min_narrow_space_width_ = vehicle_param.max_width + kMinNarrowSpaceWidth;
  max_narrow_space_width_ = vehicle_param.max_width + kMaxNarrowSpaceWidth;
  narrow_spaces_lon_gap_ = vehicle_param.front_edge_to_rear_axle + kNarrowSpacesLonGap;
  ResetNarrowSpace();
}

bool NarrowSpaceDecider::Execute() {
  if (!PreCheck()) {
    ResetNarrowSpace();
    ILOG_DEBUG << "PreCheck failed";
    return false;
  }

  if (!session_->is_nsa_scene()) {
    ResetNarrowSpace();
    return false;
  }

  InitInfo();

  HandleNarrowSpaceData();

  UpdateNarrowSpaceState();

  UpdateNarrowSpaceStatus();

  CalculateDrivingDistance();

  GenerateNarrowSpaceOutput();

  LogNarrowSpaceCorners();

  UpdateLateralObstacleDeicision();

  Log();

  return true;
}

void NarrowSpaceDecider::ResetNarrowSpace() {
  narrow_space_state_ = NarrowSpaceState::NO_NARROW_SPACE;
  narrow_space_status_ = NarrowSpaceStatus::UNKNOWN;
  last_narrow_space_status_ = NarrowSpaceStatus::UNKNOWN;
  is_out_of_narrow_space_ = false;
  distance_to_end_ = 100.0;
  last_ego_cart_.x = 0.0;
  last_ego_cart_.y = 0.0;
  end_point_.x = 0.0;
  end_point_.y = 0.0;
  InitInfo();
}

void NarrowSpaceDecider::InitInfo() {
  const auto& local_view = session_->environmental_model().get_local_view();
  const auto& current_state = local_view.function_state_machine_info.current_state;
  is_exist_narrow_space_ = false;
  is_passable_ = false;
  is_prepare_function_ = current_state >= iflyauto::FunctionalState_NRA_PASSIVE &&
                         current_state <= iflyauto::FunctionalState_NRA_PRE_ACTIVE;
  is_in_function_ = current_state == iflyauto::FunctionalState_NRA_GUIDANCE ||
                    current_state == iflyauto::FunctionalState_NRA_SUSPEND;
  // nsa scene
  if (current_state == iflyauto::FunctionalState_NRA_ERROR ||
      (is_prepare_function_ &&
       narrow_space_state_ <= NarrowSpaceState::APPROACH_NARROW_SPACE)) {
    narrow_space_state_ = NarrowSpaceState::NO_NARROW_SPACE;
    is_out_of_narrow_space_ = false;
    distance_to_end_ = 100.0;
  } else if (current_state == iflyauto::FunctionalState_NRA_COMPLETED) {
    is_out_of_narrow_space_ = true;
  }
  distance_to_narrow_space_ = 100.0;
  rotate_narrow_space_width_ = 0.0;
  narrow_space_width_ = 0.0;
  narrow_space_length_ = 0.0;
  // narrow_space_height_ = 0.0;
  narrow_space_direction_angle_ = 0.0;
  width_slack_factor_ = 0.0;
  angle_slack_factor_ = 0.0;
  vehicle_s_range_.first = 0.0;
  vehicle_s_range_.second = 0.0;
  narrow_space_s_range_.first = 0.0;
  narrow_space_s_range_.second = 0.0;
  //
  if (narrow_space_status_ == NarrowSpaceStatus::NORMAL ||
      narrow_space_status_ == NarrowSpaceStatus::OBLIQUE) {
    width_slack_factor_ = 0.15;
  } else if (narrow_space_status_ == NarrowSpaceStatus::NARROW ||
            narrow_space_status_ == NarrowSpaceStatus::WIDE) {
    width_slack_factor_ = -0.01; // -0.15
  }
  if (narrow_space_status_ == NarrowSpaceStatus::NORMAL) {
    angle_slack_factor_ = 1.5;
  } else if (narrow_space_status_ == NarrowSpaceStatus::OBLIQUE) {
    angle_slack_factor_ = -0.1; // -1.0
  }
}

bool NarrowSpaceDecider::HandleNarrowSpaceData() {
  // 1.extract outline
  std::map<LatObstacleDecisionType, std::map<double, double>> narrow_space_outline;
  if (!ExtractNarrowSpaceOutline(narrow_space_outline)) {
    return false;
  }
  // 2.generate narrow space boundary
  if (!GenerateNarrowSpaceBoundary(narrow_space_outline)) {
    return false;
  }
  // 3.judge obstacle in narrow space
  IsExistObstacleInNarrowSpace(narrow_space_outline);
  return true;
}

bool NarrowSpaceDecider::ExtractNarrowSpaceOutline(
    std::map<LatObstacleDecisionType, std::map<double, double>>& outline) {
  const auto& reference_path = session_->planning_context()
                                       .lane_change_decider_output()
                                       .coarse_planning_info.reference_path;
  const auto& frenet_coord = reference_path->get_frenet_coord();
  // road border
  // occ obstacles
  const auto& obs_vec = reference_path->get_obstacles();
  if (obs_vec.empty()) {
    return false;
  }
  const auto& frenet_ego_state = reference_path->get_frenet_ego_state();
  const auto& vehicle_param =
      VehicleConfigurationContext::Instance()->get_vehicle_param();
  double care_area_s_start = frenet_ego_state.s() - vehicle_param.rear_edge_to_rear_axle;
  double care_area_s_end = frenet_ego_state.head_s() + kPreviewFrontNarrowSpaceDistance;
  const auto& ego_pose = session_->environmental_model().get_ego_state_manager()->ego_pose();
  double tail_x = ego_pose.x - vehicle_param.rear_edge_to_rear_axle * std::cos(ego_pose.theta);
  double tail_y = ego_pose.y - vehicle_param.rear_edge_to_rear_axle * std::sin(ego_pose.theta);
  Point2D frenet_tail_point;
  if (frenet_coord->XYToSL(Point2D(tail_x, tail_y), frenet_tail_point)) {
    care_area_s_start = frenet_tail_point.x;
  }
  vehicle_s_range_.first = care_area_s_start;
  vehicle_s_range_.second = frenet_ego_state.head_s();
  care_area_s_start -= kPreviewBackNarrowSpaceDistance;
  double max_lat_dist = 0.5 * vehicle_param.max_width + kMaxNarrowSpaceWidth;
  const auto& lat_obstacle_decision = session_->planning_context()
                                              .lateral_obstacle_decider_output()
                                              .lat_obstacle_decision;
  for (const auto& obs : obs_vec) {
    if (obs->source_type() != SourceType::OCC || !obs->b_frenet_valid()) {
      continue;
    }
    const auto& frenet_obs_boundary = obs->frenet_obstacle_boundary();
    if (frenet_obs_boundary.s_end < care_area_s_start ||
        frenet_obs_boundary.s_start > care_area_s_end) {
      continue;
    }
    // cross ref
    if (frenet_obs_boundary.l_start < -0.2 && frenet_obs_boundary.l_end > 0.2 &&
        (frenet_obs_boundary.s_start > frenet_ego_state.head_s() + kNarrowSpacesLonGap ||
         (is_out_of_narrow_space_ &&
          frenet_obs_boundary.s_start > frenet_ego_state.s() + distance_to_end_))) {
      continue;
    }
    if (lat_obstacle_decision.find(obs->id()) == lat_obstacle_decision.end()) {
      continue;
    }
    const auto& lat_decision = lat_obstacle_decision.at(obs->id());
    planning_math::Polygon2d frenet_obs_polygon;
    if (obs->get_polygon_at_time(0, reference_path, frenet_obs_polygon)) {
      for (const auto& line_segment : frenet_obs_polygon.line_segments()) {
        if (line_segment.length() < 1e-6) {
          continue;
        }
        double line_seg_start_s = line_segment.start().x();
        double line_seg_start_l = line_segment.start().y();
        double line_seg_end_s = line_segment.end().x();
        double line_seg_end_l = line_segment.end().y();
        if (line_seg_start_s > line_seg_end_s) {
          line_seg_start_s = line_segment.end().x();
          line_seg_start_l = line_segment.end().y();
          line_seg_end_s = line_segment.start().x();
          line_seg_end_l = line_segment.start().y();
        }
        double lon_diff = line_seg_end_s - line_seg_start_s;
        double lat_diff = line_seg_end_l - line_seg_start_l;
        double delta_l_s = lat_diff / std::max(lon_diff, 1e-6);
        for (double line_seg_s = line_seg_start_s; line_seg_s < line_seg_end_s; line_seg_s += kSamplingNarrowSpaceLonGap) {
          double line_seg_l = line_seg_start_l + (line_seg_s - line_seg_start_s) * delta_l_s;
          if ((line_seg_s < vehicle_s_range_.second + kSamplingNarrowSpaceLonGap &&
              std::fabs(line_seg_l - frenet_ego_state.l()) > max_lat_dist) ||
              (std::fabs(line_seg_l - frenet_ego_state.l()) > kMaxCareNarrowSpaceWidth)) {
            continue;
          }
          auto last_point_iter = outline[lat_decision].find(line_seg_s);
          if (last_point_iter != outline[lat_decision].end()) {
            if (lat_decision == LatObstacleDecisionType::LEFT) {
              outline[lat_decision][line_seg_s] = std::max(last_point_iter->second, line_seg_l);
            } else if (lat_decision == LatObstacleDecisionType::RIGHT) {
              outline[lat_decision][line_seg_s] = std::min(last_point_iter->second, line_seg_l);
            } else {
              outline[lat_decision][line_seg_s] = std::min(std::fabs(last_point_iter->second), std::fabs(line_seg_l));
            }
          } else {
            outline[lat_decision][line_seg_s] = line_seg_l;
          }
        }
        //
        if ((line_seg_end_s < vehicle_s_range_.second + kSamplingNarrowSpaceLonGap &&
            std::fabs(line_seg_end_l - frenet_ego_state.l()) > max_lat_dist) ||
            (std::fabs(line_seg_end_l - frenet_ego_state.l()) > kMaxCareNarrowSpaceWidth)) {
          continue;
        }
        auto last_point_iter = outline[lat_decision].find(line_seg_end_s);
        if (last_point_iter != outline[lat_decision].end()) {
          if (lat_decision == LatObstacleDecisionType::LEFT) {
            outline[lat_decision][line_seg_end_s] = std::max(last_point_iter->second, line_seg_end_l);
          } else if (lat_decision == LatObstacleDecisionType::RIGHT) {
            outline[lat_decision][line_seg_end_s] = std::min(last_point_iter->second, line_seg_end_l);
          } else {
            outline[lat_decision][line_seg_end_s] = std::min(std::fabs(last_point_iter->second), std::fabs(line_seg_end_l));
          }
        } else {
          outline[lat_decision][line_seg_end_s] = line_seg_end_l;
        }
      }
      // for (const auto& polygon_point : frenet_obs_polygon.points()) {
      //   if ((polygon_point.x() < vehicle_s_range_.second + kSamplingNarrowSpaceLonGap &&
      //        std::fabs(polygon_point.y() - frenet_ego_state.l()) > max_lat_dist) ||
      //       (std::fabs(polygon_point.y() - frenet_ego_state.l()) > kMaxCareNarrowSpaceWidth)) {
      //     continue;
      //   }
      //   auto last_point_iter = outline[lat_decision].find(polygon_point.x());
      //   if (last_point_iter != outline[lat_decision].end()) {
      //     if (lat_decision == LatObstacleDecisionType::LEFT) {
      //       outline[lat_decision][polygon_point.x()] = std::max(last_point_iter->second, polygon_point.y());
      //     } else if (lat_decision == LatObstacleDecisionType::RIGHT) {
      //       outline[lat_decision][polygon_point.x()] = std::min(last_point_iter->second, polygon_point.y());
      //     } else {
      //       outline[lat_decision][polygon_point.x()] = std::min(std::fabs(last_point_iter->second), std::fabs(polygon_point.y()));
      //     }
      //   } else {
      //     outline[lat_decision][polygon_point.x()] = polygon_point.y();
      //   }
      // }
    } else {
      const auto& frenet_obs_corner_points = obs->corner_points();
      for (const auto& corner_point : frenet_obs_corner_points) {
        if ((corner_point.x() < vehicle_s_range_.second + kSamplingNarrowSpaceLonGap &&
             std::fabs(corner_point.y() - frenet_ego_state.l()) > max_lat_dist) ||
            (std::fabs(corner_point.y() - frenet_ego_state.l()) > kMaxCareNarrowSpaceWidth)) {
          continue;
        }
        auto last_point_iter = outline[lat_decision].find(corner_point.x());
        if (last_point_iter != outline[lat_decision].end()) {
          if (lat_decision == LatObstacleDecisionType::LEFT) {
            outline[lat_decision][corner_point.x()] = std::max(last_point_iter->second, corner_point.y());
          } else if (lat_decision == LatObstacleDecisionType::RIGHT) {
            outline[lat_decision][corner_point.x()] = std::min(last_point_iter->second, corner_point.y());
          } else {
            outline[lat_decision][corner_point.x()] = std::min(std::fabs(last_point_iter->second), std::fabs(corner_point.y()));
          }
        } else {
          outline[lat_decision][corner_point.x()] = corner_point.y();
        }
      }
    }
  }
  return true;
}

bool NarrowSpaceDecider::GenerateNarrowSpaceBoundary(
    const std::map<LatObstacleDecisionType, std::map<double, double>>& outline) {
  // 1.generate left boundary
  std::vector<double> left_s_vec;
  std::vector<double> left_l_vec;
  auto left_outline_iter = outline.find(LatObstacleDecisionType::RIGHT);
  if (left_outline_iter != outline.end()) {
    const auto& left_outline = left_outline_iter->second;
    double last_s = left_outline.begin()->first;
    double last_l = left_outline.begin()->second;
    for (auto iter = left_outline.begin(); iter != left_outline.end(); ++iter) {
      double ds = iter->first - last_s;
      if (ds <= 1e-6) {
        continue;
      }
      if (ds > kSamplingNarrowSpaceLonGap) {
        left_s_vec.emplace_back(last_s);
        left_l_vec.emplace_back(last_l);
        last_s = iter->first;
        last_l = iter->second;
        // if (iter->second - last_l > kSamplingNarrowSpaceLatGap) {
        //   last_l += kSamplingNarrowSpaceLatGap;
        // } else {
        //   last_l = iter->second;
        // }
      } else {
        if (iter->second < last_l) {
          // last_s = iter->first;
          last_l = iter->second;
        }
      }
    }
    if (!left_s_vec.empty()) {
      if (last_s - left_s_vec.back() > 1e-6) {
        left_s_vec.emplace_back(last_s);
        left_l_vec.emplace_back(last_l);
      }
    }
  }
  if (!DenseBoundary(left_s_vec, left_l_vec)) {
    return false;
  }
  // 2.generate right boundary
  std::vector<double> right_s_vec;
  std::vector<double> right_l_vec;
  auto right_outline_iter = outline.find(LatObstacleDecisionType::LEFT);
  if (right_outline_iter != outline.end()) {
    const auto& right_outline = right_outline_iter->second;
    double last_s = right_outline.begin()->first;
    double last_l = right_outline.begin()->second;
    for (auto iter = right_outline.begin(); iter != right_outline.end(); ++iter) {
      double ds = iter->first - last_s;
      if (ds <= 1e-6) {
        continue;
      }
      if (ds > kSamplingNarrowSpaceLonGap) {
        right_s_vec.emplace_back(last_s);
        right_l_vec.emplace_back(last_l);
        last_s = iter->first;
        last_l = iter->second;
        // if (last_l - iter->second > kSamplingNarrowSpaceLatGap) {
        //   last_l -= kSamplingNarrowSpaceLatGap;
        // } else {
        //   last_l = iter->second;
        // }
      } else {
        if (iter->second > last_l) {
          // last_s = iter->first;
          last_l = iter->second;
        }
      }
    }
    if (!right_s_vec.empty()) {
      if (last_s - right_s_vec.back() > 1e-6) {
        right_s_vec.emplace_back(last_s);
        right_l_vec.emplace_back(last_l);
      }
    }
  }
  if (!DenseBoundary(right_s_vec, right_l_vec)) {
    return false;
  }
  // 3.calculate narrow space info
  if (!CalculateNarrowSpaceInfo(left_s_vec, left_l_vec, right_s_vec, right_l_vec)) {
    return false;
  }
  return true;
}

bool NarrowSpaceDecider::DenseBoundary(
    std::vector<double>& s_vec, std::vector<double>& l_vec) {
  if (s_vec.size() < 2 || l_vec.size() < 2 || s_vec.size() != l_vec.size()) {
    return false;
  }
  if (s_vec.back() - s_vec.front() < kMinNarrowSpaceLength) {
    return false;
  }
  if (s_vec.size() > 2) {
    // temp not dense
    return true;
  }
  std::vector<double> dense_s_vec;
  std::vector<double> dense_l_vec;
  dense_s_vec.reserve(2 * s_vec.size());
  dense_l_vec.reserve(2 * l_vec.size());
  for (size_t i = 0; i < s_vec.size() - 1; ++i) {
    double s = s_vec[i];
    double l = l_vec[i];
    double mid_s = 0.5 * (s_vec[i] + s_vec[i + 1]);
    double mid_l = 0.5 * (l_vec[i] + l_vec[i + 1]);
    dense_s_vec.emplace_back(s);
    dense_l_vec.emplace_back(l);
    dense_s_vec.emplace_back(mid_s);
    dense_l_vec.emplace_back(mid_l);
  }
  dense_s_vec.emplace_back(s_vec.back());
  dense_l_vec.emplace_back(l_vec.back());
  s_vec.clear();
  l_vec.clear();
  s_vec = std::move(dense_s_vec);
  l_vec = std::move(dense_l_vec);
  return true;
}

bool NarrowSpaceDecider::CalculateNarrowSpaceInfo(
    const std::vector<double>& left_s_vec, const std::vector<double>& left_l_vec,
    const std::vector<double>& right_s_vec, const std::vector<double>& right_l_vec) {
  const auto& reference_path = session_->planning_context()
                                       .lane_change_decider_output()
                                       .coarse_planning_info.reference_path;
  const auto& frenet_ego_state = reference_path->get_frenet_ego_state();
  // 1.calculate narrow space shape
  // extract boundary range
  double boundary_start = std::max(left_s_vec.front(), right_s_vec.front());
  double boundary_end = std::min(left_s_vec.back(), right_s_vec.back());
  if (is_out_of_narrow_space_) {
    boundary_end = std::min(frenet_ego_state.s() + distance_to_end_, boundary_end);
  }
  double boundary_length_ = std::max((boundary_end - boundary_start), 0.0);
  if (boundary_length_ <= 1e-3 || vehicle_s_range_.first > boundary_end) {
    // boundary length exception
    return false;
  }
  // extract narrow space range
  left_boundary_spline_.set_points(left_s_vec, left_l_vec, pnc::mathlib::spline::linear);
  right_boundary_spline_.set_points(right_s_vec, right_l_vec, pnc::mathlib::spline::linear);
  narrow_space_s_range_.first = boundary_start;
  bool is_find_narrow_space_start_point = false;
  double extra_width_buffer = 0.0;
  if (is_in_function_) {
    extra_width_buffer = 0.1;
  }
  for (double boundary_s = boundary_start; boundary_s < boundary_end + kSamplingNarrowSpaceLonStep - 0.01; boundary_s += kSamplingNarrowSpaceLonStep) {
    boundary_s = std::min(boundary_end, boundary_s);
    double left_boundary_l = left_boundary_spline_(boundary_s);
    double right_boundary_l = right_boundary_spline_(boundary_s);
    double channel_width = left_boundary_l - right_boundary_l;
    if (channel_width <= max_narrow_space_width_ + kNarrowSpaceWidthBuffer + extra_width_buffer) {
      narrow_space_s_range_.first = boundary_s;
      is_find_narrow_space_start_point = true;
      break;
    }
  }
  if (!is_find_narrow_space_start_point) {
    // no narrow space
    return false;
  }
  if (is_in_function_) {
    extra_width_buffer = 0.2;
  }
  narrow_space_s_range_.second = boundary_end;
  std::pair<double, double> narrowest_point{boundary_end, kMaxCareNarrowSpaceWidth};
  bool is_find_narrow_space_end_point = false;
  bool is_find_segment_start = false;
  bool is_find_segment_end = false;
  double segment_start_s = boundary_end;
  double segment_end_s = boundary_end;
  for (double boundary_s = boundary_end; boundary_s > narrow_space_s_range_.first - kSamplingNarrowSpaceLonStep + 0.01; boundary_s -= kSamplingNarrowSpaceLonStep) {
    boundary_s = std::max(narrow_space_s_range_.first, boundary_s);
    double left_boundary_l = left_boundary_spline_(boundary_s);
    double right_boundary_l = right_boundary_spline_(boundary_s);
    double channel_width = std::max(left_boundary_l - right_boundary_l, 0.0);
    if (channel_width < narrowest_point.second) {
      narrowest_point.first = boundary_s;
      narrowest_point.second = channel_width;
    }
    if (channel_width < max_narrow_space_width_ + kNarrowSpaceWidthBuffer + extra_width_buffer) {
      if (!is_find_narrow_space_end_point) {
        narrow_space_s_range_.second = boundary_s;
        is_find_narrow_space_end_point = true;
      }
      if (!is_find_segment_start) {
        is_find_segment_start = true;
        segment_start_s = boundary_s;
      } else if (!is_find_segment_end) {
        is_find_segment_end = true;
        segment_end_s = boundary_s;
      }
      if (is_find_segment_start && is_find_segment_end) {
        is_find_segment_start = false;
        is_find_segment_end = false;
        if (segment_end_s - segment_start_s > narrow_spaces_lon_gap_) {
          narrow_space_s_range_.second = boundary_s;
        }
      }
    }
  }
  double lower_s = std::max(narrow_space_s_range_.first, narrowest_point.first - kSamplingNarrowSpaceLonStep);
  double upper_s = std::min(narrow_space_s_range_.second, narrowest_point.first + kSamplingNarrowSpaceLonStep);
  double lower_s_width = std::max(left_boundary_spline_(lower_s) -  right_boundary_spline_(lower_s), 0.0);
  double upper_s_width = std::max(left_boundary_spline_(upper_s) -  right_boundary_spline_(upper_s), 0.0);
  narrow_space_width_ = std::max((narrowest_point.second + lower_s_width + upper_s_width) / 3.0, 0.0);
  if (narrow_space_s_range_.second <= vehicle_s_range_.second + kSamplingNarrowSpaceLonGap) {
    double valid_boundary_end = narrow_space_s_range_.second;
    for (size_t i = left_s_vec.size() - 2; i > 0; --i) {
      if (left_s_vec[i] <= narrow_space_s_range_.second &&
          left_s_vec[i + 1] - left_s_vec[i] > vehicle_s_range_.second - vehicle_s_range_.first) {
        valid_boundary_end = std::min(left_s_vec[i], valid_boundary_end);
        break;
      }
    }
    for (size_t j = right_s_vec.size() - 2; j > 0; --j) {
      if (right_s_vec[j] <= narrow_space_s_range_.second &&
          right_s_vec[j + 1] - right_s_vec[j] > vehicle_s_range_.second - vehicle_s_range_.first) {
        valid_boundary_end = std::min(right_s_vec[j], valid_boundary_end);
        break;
      }
    }
    narrow_space_s_range_.second = valid_boundary_end;
  }
  narrow_space_length_ = std::max((narrow_space_s_range_.second - narrow_space_s_range_.first), 0.0);
  if (narrow_space_length_ <= 1e-3 || vehicle_s_range_.first > narrow_space_s_range_.second) {
    // narrow space length exception
    return false;
  }
  is_exist_narrow_space_ = true;
  is_passable_ = true;
  // 2.calculate narrow space direction angle
  double mid_narrow_space_start_s = narrow_space_s_range_.first;
  double mid_narrow_space_start_l = 0.5 * (left_boundary_spline_(narrow_space_s_range_.first) + right_boundary_spline_(narrow_space_s_range_.first));
  double mid_narrow_space_end_s = narrow_space_s_range_.second;
  double mid_narrow_space_end_l = 0.5 * (left_boundary_spline_(narrow_space_s_range_.second) + right_boundary_spline_(narrow_space_s_range_.second));
  if (frenet_ego_state.s() >= mid_narrow_space_start_s && vehicle_s_range_.second <= mid_narrow_space_end_s) {
    mid_narrow_space_start_l = 0.5 * (left_boundary_spline_(frenet_ego_state.s()) + right_boundary_spline_(frenet_ego_state.s()));
    mid_narrow_space_end_l = 0.5 * (left_boundary_spline_(vehicle_s_range_.second) + right_boundary_spline_(vehicle_s_range_.second));
  }
  const auto& frenet_coord = reference_path->get_frenet_coord();
  Point2D cart_space_start_pt{0.0, 0.0};
  Point2D frenet_space_start_pt{mid_narrow_space_start_s, mid_narrow_space_start_l};
  Point2D cart_space_end_pt{0.0, 0.0};
  Point2D frenet_space_end_pt{mid_narrow_space_end_s, mid_narrow_space_end_l};
  if (frenet_coord->SLToXY(frenet_space_start_pt, cart_space_start_pt) &&
      frenet_coord->SLToXY(frenet_space_end_pt, cart_space_end_pt)) {
    double dx = cart_space_end_pt.x - cart_space_start_pt.x;
    double dy = cart_space_end_pt.y - cart_space_start_pt.y;
    narrow_space_direction_angle_ = std::atan2(dy, dx);
    if (narrow_space_state_ < NarrowSpaceState::EXITING_NARROW_SPACE) {
      end_point_ = cart_space_end_pt;
    }
  }
  double diff_heading_angle = planning_math::NormalizeAngle(
      narrow_space_direction_angle_ - frenet_coord->GetPathCurveHeading(mid_narrow_space_start_s));
  double rotate_narrow_space_width_ = narrow_space_width_ * std::cos(diff_heading_angle);
  return true;
}

bool NarrowSpaceDecider::IsExistObstacleInNarrowSpace(
    const std::map<LatObstacleDecisionType, std::map<double, double>>& outline) {
  const auto& ego_state_mgr = session_->environmental_model().get_ego_state_manager();
  if (ego_state_mgr->ego_v() <= 1e-2) {  // && stop plan
    return false;
  }
  const auto& vehicle_param =
      VehicleConfigurationContext::Instance()->get_vehicle_param();
  const auto &reference_path = session_->planning_context()
                                       .lane_change_decider_output()
                                       .coarse_planning_info.reference_path;
  // road border
  // occ obstacles
  const auto& obs_vec = reference_path->get_obstacles();
  for (const auto& obs : obs_vec) {
    if (!obs->b_frenet_valid()) {
      continue;
    }
    const auto& frenet_obs_boundary = obs->frenet_obstacle_boundary();
    if (frenet_obs_boundary.s_end < vehicle_s_range_.first ||
        frenet_obs_boundary.s_start > vehicle_s_range_.second + kPreviewFrontNarrowSpaceDistance ||
        frenet_obs_boundary.l_start  > kMaxCareNarrowSpaceWidth ||
        frenet_obs_boundary.l_end < -kMaxCareNarrowSpaceWidth) {
      continue;
    }
    uint barrier_num = 0;
    double min_l = 10.0;
    double max_l = -10.0;
    planning_math::Polygon2d frenet_obs_polygon;
    if (obs->get_polygon_at_time(0, reference_path, frenet_obs_polygon)) {
      for (const auto& polygon_point : frenet_obs_polygon.points()) {
        if (polygon_point.x() < vehicle_s_range_.second ||
            polygon_point.x() > narrow_space_s_range_.second) {
          continue;
        }
        double left_boundary_l = left_boundary_spline_(polygon_point.x());
        if (polygon_point.y() >= left_boundary_l) {
          continue;
        }
        double right_boundary_l = right_boundary_spline_(polygon_point.x());
        if (right_boundary_l >= polygon_point.y()) {
          continue;
        }
        min_l = std::min(min_l, polygon_point.y());
        max_l = std::max(max_l, polygon_point.y());
        double rel_left_dist = left_boundary_l - polygon_point.y();
        double rel_right_dist = polygon_point.y() - right_boundary_l;
        if (rel_left_dist < vehicle_param.max_width + kBlockedNarrowSpaceWidth &&
            rel_right_dist < vehicle_param.max_width + kBlockedNarrowSpaceWidth) {
          barrier_num++;
        }
      }
    } else {
      const auto& frenet_obs_corner_points = obs->corner_points();
      for (const auto& corner_point : frenet_obs_corner_points) {
        if (corner_point.x() < vehicle_s_range_.second ||
            corner_point.x() > narrow_space_s_range_.second) {
          continue;
        }
        double left_boundary_l = left_boundary_spline_(corner_point.x());
        if (corner_point.y() >= left_boundary_l) {
          continue;
        }
        double right_boundary_l = right_boundary_spline_(corner_point.x());
        if (right_boundary_l >= corner_point.y()) {
          continue;
        }
        min_l = std::min(min_l, corner_point.y());
        max_l = std::max(max_l, corner_point.y());
        double rel_left_dist = left_boundary_l - corner_point.y();
        double rel_right_dist = corner_point.y() - right_boundary_l;
        if (rel_left_dist < vehicle_param.max_width + kBlockedNarrowSpaceWidth &&
            rel_right_dist < vehicle_param.max_width + kBlockedNarrowSpaceWidth) {
          barrier_num++;
        }
      }
    }
    if (barrier_num > 1 && (max_l - min_l) >= (narrow_space_width_ - vehicle_param.max_width - kBlockedNarrowSpaceWidth)) {
      is_passable_ = false;
      return true;
    }
  }
  return false;
}

void NarrowSpaceDecider::UpdateNarrowSpaceState() {
  const auto& reference_path = session_->planning_context()
                                       .lane_change_decider_output()
                                       .coarse_planning_info
                                       .reference_path;
  const auto& frenet_coord = reference_path->get_frenet_coord();
  const auto& frenet_ego_state = reference_path->get_frenet_ego_state();
  // calculate remain dist
  Point2D frenet_space_end_pt{0.0, 0.0};
  if (frenet_coord->XYToSL(end_point_, frenet_space_end_pt)) {
    double rear_dist_to_end = frenet_space_end_pt.x - frenet_ego_state.s();
    if (is_prepare_function_) {
      distance_to_end_ = rear_dist_to_end;
    } else if (is_in_function_) {
      if (narrow_space_state_ >= NarrowSpaceState::APPROACH_NARROW_SPACE &&
          narrow_space_state_ <= NarrowSpaceState::IN_NARROW_SPACE) {
        distance_to_end_ = frenet_space_end_pt.x - frenet_ego_state.s();
      } else {
        distance_to_end_ = std::min(distance_to_end_, rear_dist_to_end);
      }
    }
  }
  // is out of channel
  if (is_prepare_function_) {
    if (distance_to_end_ < 1e-6) {
      narrow_space_state_ = NarrowSpaceState::NO_NARROW_SPACE;
      is_out_of_narrow_space_ = true;
    }
  } else if (is_in_function_) {
    if (distance_to_end_ <= (vehicle_s_range_.first - frenet_ego_state.s() - kCompleteLonBuffer)) {
      narrow_space_state_ = NarrowSpaceState::NO_NARROW_SPACE;
      is_out_of_narrow_space_ = true;
    }
  }
  // update state
  if (!is_exist_narrow_space_) {
    narrow_space_state_ = NarrowSpaceState::NO_NARROW_SPACE;
  } else if (!is_out_of_narrow_space_) {
    if (narrow_space_state_ >= NarrowSpaceState::EXITING_NARROW_SPACE) {
      if (vehicle_s_range_.first > narrow_space_s_range_.second) {
        distance_to_narrow_space_ = narrow_space_s_range_.second - vehicle_s_range_.second;
        narrow_space_state_ = NarrowSpaceState::NO_NARROW_SPACE;
        is_out_of_narrow_space_ = true;
      }
    } else if (narrow_space_state_ >= NarrowSpaceState::IN_NARROW_SPACE) {
      if (vehicle_s_range_.first > narrow_space_s_range_.second) {
        distance_to_narrow_space_ = narrow_space_s_range_.second - vehicle_s_range_.second;
        narrow_space_state_ = NarrowSpaceState::NO_NARROW_SPACE;
        is_out_of_narrow_space_ = true;
      } else if (vehicle_s_range_.second > narrow_space_s_range_.second) {
        distance_to_narrow_space_ = narrow_space_s_range_.second - vehicle_s_range_.second;
        narrow_space_state_ = NarrowSpaceState::EXITING_NARROW_SPACE;
      }
    } else if (narrow_space_state_ >= NarrowSpaceState::ENTERING_NARROW_SPACE) {
      if (vehicle_s_range_.first > narrow_space_s_range_.second) {
        distance_to_narrow_space_ = narrow_space_s_range_.second - vehicle_s_range_.second;
        narrow_space_state_ = NarrowSpaceState::NO_NARROW_SPACE;
        is_out_of_narrow_space_ = true;
      } else if (vehicle_s_range_.second > narrow_space_s_range_.second) {
        distance_to_narrow_space_ = narrow_space_s_range_.second - vehicle_s_range_.second;
        narrow_space_state_ = NarrowSpaceState::EXITING_NARROW_SPACE;
      } else if (vehicle_s_range_.first >= narrow_space_s_range_.first) {
        distance_to_narrow_space_ = 0.0;
        narrow_space_state_ = NarrowSpaceState::IN_NARROW_SPACE;
      }
    } else if (narrow_space_state_ >= NarrowSpaceState::APPROACH_NARROW_SPACE) {
      if (vehicle_s_range_.first > narrow_space_s_range_.second) {
        distance_to_narrow_space_ = narrow_space_s_range_.second - vehicle_s_range_.second;
        narrow_space_state_ = NarrowSpaceState::NO_NARROW_SPACE;
        is_out_of_narrow_space_ = true;
      } else if (vehicle_s_range_.second > narrow_space_s_range_.second) {
        distance_to_narrow_space_ = narrow_space_s_range_.second - vehicle_s_range_.second;
        narrow_space_state_ = NarrowSpaceState::EXITING_NARROW_SPACE;
      } else if (vehicle_s_range_.first >= narrow_space_s_range_.first) {
        distance_to_narrow_space_ = 0.0;
        narrow_space_state_ = NarrowSpaceState::IN_NARROW_SPACE;
      } else if (vehicle_s_range_.second >= narrow_space_s_range_.first) {
        distance_to_narrow_space_ = 0.0;
        narrow_space_state_ = NarrowSpaceState::ENTERING_NARROW_SPACE;
      }
    } else if (narrow_space_state_ >= NarrowSpaceState::NO_NARROW_SPACE) {
      if (vehicle_s_range_.first > narrow_space_s_range_.second) {
        distance_to_narrow_space_ = narrow_space_s_range_.second - vehicle_s_range_.second;
        narrow_space_state_ = NarrowSpaceState::NO_NARROW_SPACE;
        is_out_of_narrow_space_ = true;
      } else if (vehicle_s_range_.second > narrow_space_s_range_.second) {
        distance_to_narrow_space_ = narrow_space_s_range_.second - vehicle_s_range_.second;
        narrow_space_state_ = NarrowSpaceState::EXITING_NARROW_SPACE;
      } else if (vehicle_s_range_.first >= narrow_space_s_range_.first) {
        distance_to_narrow_space_ = 0.0;
        narrow_space_state_ = NarrowSpaceState::IN_NARROW_SPACE;
      } else if (vehicle_s_range_.second >= narrow_space_s_range_.first) {
        distance_to_narrow_space_ = 0.0;
        narrow_space_state_ = NarrowSpaceState::ENTERING_NARROW_SPACE;
      } else {
        distance_to_narrow_space_ = narrow_space_s_range_.first - vehicle_s_range_.second;
        narrow_space_state_ = NarrowSpaceState::APPROACH_NARROW_SPACE;
      }
    }
  }
  JSON_DEBUG_VALUE("narrow_space_state", static_cast<int>(narrow_space_state_));
}

void NarrowSpaceDecider::UpdateNarrowSpaceStatus() {
  NarrowSpaceStatus current_narrow_space_status = narrow_space_status_;
  if (!is_exist_narrow_space_) {
    current_narrow_space_status = NarrowSpaceStatus::UNKNOWN;
    narrow_space_status_ = current_narrow_space_status;
  } else {
    if (narrow_space_width_ < min_narrow_space_width_ - width_slack_factor_) {
      current_narrow_space_status = NarrowSpaceStatus::NARROW;
    } else if (narrow_space_width_ > max_narrow_space_width_ + width_slack_factor_) {
      current_narrow_space_status = NarrowSpaceStatus::WIDE;
    } else {
      const auto& ego_pose =
          session_->environmental_model().get_ego_state_manager()->ego_pose();
      double theta_error = narrow_space_direction_angle_ - ego_pose.GetPhi();
      const double pi2 = 2.0 * M_PI;
      if (theta_error > M_PI) {
        narrow_space_direction_angle_ -= pi2;
        theta_error -= pi2;
      } else if (theta_error < -M_PI) {
        narrow_space_direction_angle_ += pi2;
        theta_error += pi2;
      }
      double relative_angle = 57.3 * std::fabs(theta_error);
      if (relative_angle > kMaxNarrowSpaceRelativeAngle + angle_slack_factor_ &&
          narrow_space_state_ < NarrowSpaceState::EXITING_NARROW_SPACE) {
        current_narrow_space_status = NarrowSpaceStatus::OBLIQUE;
      } else {
        current_narrow_space_status = NarrowSpaceStatus::NORMAL;
      }
    }
    if (last_narrow_space_status_ == NarrowSpaceStatus::NORMAL) {
      narrow_space_status_ = last_narrow_space_status_;
    } else {
      narrow_space_status_ = current_narrow_space_status;
    }
  }
  last_narrow_space_status_ = current_narrow_space_status;
  const auto& vehicle_param =
      VehicleConfigurationContext::Instance()->get_vehicle_param();
  if (is_in_function_ && narrow_space_status_ == NarrowSpaceStatus::NARROW &&
      narrow_space_width_ < vehicle_param.max_width + kBlockedNarrowSpaceWidth) {  // && stop plan
    is_passable_ = false;
  }
}

void NarrowSpaceDecider::CalculateDrivingDistance() {
  // calculate driving distance
  const auto& ego_cart = session_->environmental_model().get_ego_state_manager()->ego_carte();
  const auto& fsm_state =
      session_->environmental_model().get_local_view().function_state_machine_info.current_state;
  if (fsm_state < iflyauto::FunctionalState_NRA_SUSPEND ||
      fsm_state > iflyauto::FunctionalState_NRA_COMPLETED) {
    accumulated_driving_distance_ = 0.0;
  } else {
    if (fsm_state == iflyauto::FunctionalState_NRA_GUIDANCE) {
      double ds = std::hypot(ego_cart.x - last_ego_cart_.x, ego_cart.y - last_ego_cart_.y);
      if (ds > 1e-2) {
        accumulated_driving_distance_ += ds;
      }
    }
  }
  last_ego_cart_.x = ego_cart.x;
  last_ego_cart_.y = ego_cart.y;
}

void NarrowSpaceDecider::GenerateNarrowSpaceOutput() {
  auto& output = session_->mutable_planning_context()->mutable_narrow_space_decider_output();
  output.Clear();
  output.is_exist_narrow_space = is_exist_narrow_space_ && !is_out_of_narrow_space_;
  // output.is_passable_narrow_space =
  //     is_passable_ && accumulated_driving_distance_ <= kMaxNarrowSpaceDrivingDistance;
  output.is_passable_narrow_space = is_passable_;
  output.is_in_narrow_space = narrow_space_state_ > NarrowSpaceState::APPROACH_NARROW_SPACE;
  output.is_too_narrow = narrow_space_status_ == NarrowSpaceStatus::NARROW;
  output.is_too_wide = narrow_space_status_ == NarrowSpaceStatus::WIDE;
  output.is_relative_angle_too_large = narrow_space_status_ == NarrowSpaceStatus::OBLIQUE;
  output.is_exiting_narrow_space = narrow_space_state_ == NarrowSpaceState::EXITING_NARROW_SPACE || is_out_of_narrow_space_;
  output.distance_to_end = distance_to_end_;
}

void NarrowSpaceDecider::LogNarrowSpaceCorners() {
  double left_rear_x = 0.0;
  double left_rear_y = 0.0;
  double right_rear_x = 0.0;
  double right_rear_y = 0.0;
  double left_front_x = 0.0;
  double left_front_y = 0.0;
  double right_front_x = 0.0;
  double right_front_y = 0.0;
  if (is_exist_narrow_space_) {
    const auto& reference_path = session_->planning_context()
                                        .lane_change_decider_output()
                                        .coarse_planning_info.reference_path;
    const auto& frenet_coord = reference_path->get_frenet_coord();
    double left_rear_l = left_boundary_spline_(narrow_space_s_range_.first);
    double right_rear_l = right_boundary_spline_(narrow_space_s_range_.first);
    double left_front_l = left_boundary_spline_(narrow_space_s_range_.second);
    double right_front_l = right_boundary_spline_(narrow_space_s_range_.second);
    if (!frenet_coord->SLToXY(narrow_space_s_range_.first, left_rear_l, &left_rear_x, &left_rear_y)) {
      left_rear_x = 0.0;
      left_rear_y = 0.0;
    }
    if (!frenet_coord->SLToXY(narrow_space_s_range_.first, right_rear_l, &right_rear_x, &right_rear_y)) {
      right_rear_x = 0.0;
      right_rear_y = 0.0;
    }
    if (!frenet_coord->SLToXY(narrow_space_s_range_.second, left_front_l, &left_front_x, &left_front_y)) {
      left_front_x = 0.0;
      left_front_y = 0.0;
    }
    if (!frenet_coord->SLToXY(narrow_space_s_range_.second, right_front_l, &right_front_x, &right_front_y)) {
      right_front_x = 0.0;
      right_front_y = 0.0;
    }
  }
  JSON_DEBUG_VALUE("narrow_space_left_rear_x", left_rear_x);
  JSON_DEBUG_VALUE("narrow_space_left_rear_y", left_rear_y);
  JSON_DEBUG_VALUE("narrow_space_right_rear_x", right_rear_x);
  JSON_DEBUG_VALUE("narrow_space_right_rear_y", right_rear_y);
  JSON_DEBUG_VALUE("narrow_space_left_front_x", left_front_x);
  JSON_DEBUG_VALUE("narrow_space_left_front_y", left_front_y);
  JSON_DEBUG_VALUE("narrow_space_right_front_x", right_front_x);
  JSON_DEBUG_VALUE("narrow_space_right_front_y", right_front_y);

  JSON_DEBUG_VALUE("narrow_space_end_point_x", end_point_.x);
  JSON_DEBUG_VALUE("narrow_space_end_point_y", end_point_.y);
}

bool NarrowSpaceDecider::UpdateLateralObstacleDeicision() {
  const auto& reference_path = session_->planning_context()
                                       .lane_change_decider_output()
                                       .coarse_planning_info.reference_path;
  const auto& obs_vec = reference_path->get_obstacles();
  if (obs_vec.empty()) {
    return true;
  }
  const double ego_s = reference_path->get_frenet_ego_state().s();
  auto& lat_obstacle_decision = session_->mutable_planning_context()
                                        ->mutable_lateral_obstacle_decider_output()
                                        .lat_obstacle_decision;
  for (const auto& obstacle : obs_vec) {
    if (obstacle->b_frenet_valid()) {
      const auto& frenet_obs_boundary = obstacle->frenet_obstacle_boundary();
      // cross ref
      if ((frenet_obs_boundary.s_start > narrow_space_s_range_.second ||
           (is_out_of_narrow_space_ &&
            frenet_obs_boundary.s_start > (ego_s + distance_to_end_))) &&
          (frenet_obs_boundary.l_start < 0.2 && frenet_obs_boundary.l_end > -0.2)) {
        lat_obstacle_decision[obstacle->id()] = LatObstacleDecisionType::IGNORE;
      }
    }
  }
  return true;
}

void NarrowSpaceDecider::Log() {
#ifdef ENABLE_PROTO_LOG
  auto &planning_debug_data = DebugInfoManager::GetInstance().GetDebugInfoPb();
  auto environment_model_debug_info =
      planning_debug_data->mutable_environment_model_info();
  auto obstacle_log = environment_model_debug_info->mutable_obstacle();
  auto &lat_obstacle_decision = session_->mutable_planning_context()
                                    ->mutable_lateral_obstacle_decider_output()
                                    .lat_obstacle_decision;
  for (size_t i = 0; i < obstacle_log->size();++i) {
    auto obs = obstacle_log->Mutable(i);
    obs->set_lat_decision(static_cast<uint32_t>(lat_obstacle_decision[obs->id()]));
  }
#endif
}

}  // namespace planning