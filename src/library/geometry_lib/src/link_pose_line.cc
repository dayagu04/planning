#include "link_pose_line.h"

#include <Eigen/src/Core/Matrix.h>

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

#include "geometry_math.h"
#include "log_glog.h"

namespace planning {

static const double kMaxArcLength = 12.68;
static const double kMinArcLength = 0.0168;
static const double kMaxArcRadius = 15.68;
static const double kMaxRadiusErr = 2e-2;

const bool LinkPoseLine::CalLPLPath(const LinkPoseLineInput& input) {
  input_ = input;
  ref_line_num_ = 0;
  output_.Clear();

  const double max_lat_err = std::fabs(input.lat_err);
  const double step = 1e-2;

  double lar_errs[MAX_REF_LINE_NUM];
  lar_errs[ref_line_num_++] = 0.0;
  for (double lat_err = step;
       lat_err < max_lat_err && ref_line_num_ < MAX_REF_LINE_NUM;
       lat_err += step) {
    lar_errs[ref_line_num_++] = lat_err;
    lar_errs[ref_line_num_++] = -lat_err;
  }

  if (ref_line_num_ + 2 <= MAX_REF_LINE_NUM &&
      std::fabs(lar_errs[ref_line_num_ - 1]) < max_lat_err &&
      max_lat_err > step) {
    lar_errs[ref_line_num_++] = max_lat_err;
    lar_errs[ref_line_num_++] = -max_lat_err;
  }

  const Eigen::Vector2d tang_vec = input.ref_line.heading_vec;

  const Eigen::Vector2d move_vec{-tang_vec.y(), tang_vec.x()};

  for (size_t i = 0; i < ref_line_num_; ++i) {
    geometry_lib::LineSegment ref_line = input_.ref_line;
    if (std::fabs(lar_errs[i]) > 1e-4) {
      ref_line.pA += move_vec * lar_errs[i];
      ref_line.pB += move_vec * lar_errs[i];
    }
    ref_lines_[i] = ref_line;
  }

  // 计算一个虚拟圆心，用以提前退出，加快计算
  const Eigen::Vector2d t = input_.pose.heading_vec;
  Eigen::Vector2d n{-t.y(), t.x()};
  Eigen::Vector2d center = input_.pose.pos + n * input_.min_radius;
  virtual_circle1_dist_ =
      geometry_lib::CalPoint2LineDist(center, input_.ref_line);
  n *= -1.0;
  center = input_.pose.pos + n * input_.min_radius;
  virtual_circle2_dist_ =
      geometry_lib::CalPoint2LineDist(center, input_.ref_line);

  for (size_t i = 0; i < MAX_GEAR_NUM; ++i) {
    LinkPoseLinePath lpl_path;
    if (OneArcPath(lpl_path, gears_[i])) {
      output_.lpl_paths[output_.lpl_path_num++] = lpl_path;
    }
    if (TwoArcPath(lpl_path, gears_[i], false)) {
      output_.lpl_paths[output_.lpl_path_num++] = lpl_path;
    }
    if (LineArcPath(lpl_path, gears_[i], true)) {
      output_.lpl_paths[output_.lpl_path_num++] = lpl_path;
    }
    if (AlignBodySTurnPath(lpl_path, gears_[i], true)) {
      output_.lpl_paths[output_.lpl_path_num++] = lpl_path;
    }
  }

  return output_.lpl_path_num > 0;
}

const bool LinkPoseLine::OneArcPath(LinkPoseLinePath& lpl_path,
                                    const uint8_t ref_gear) {
  lpl_path.Clear();
  double radius = input_.min_radius;
  if (std::max(virtual_circle1_dist_, virtual_circle2_dist_) <
      radius - input_.lat_err) {
    return false;
  }
  if (input_.use_bigger_radius &&
      geometry_lib::IsSameGear(ref_gear, input_.ref_last_line_gear)) {
    radius = input_.bigger_radius_no_asssign;
  }
  geometry_lib::Arc arc;
  arc.pA = input_.pose.pos;
  arc.headingA = input_.pose.heading;
  arc.headingA_vec = input_.pose.heading_vec;

  const double heading_err_threshold = input_.theta_err * kDeg2Rad;
  for (size_t i = 0; i < ref_line_num_; ++i) {
    geometry_lib::LineSegment& ref_line = ref_lines_[i];

    if (!geometry_lib::CalcOneArcWithLineAndGear(arc, ref_line, ref_gear,
                                                 radius - 1e-3)) {
      continue;
    }

    if (std::fabs(geometry_lib::NormalizeAngle(
            arc.headingB - ref_line.heading)) > heading_err_threshold) {
      continue;
    }

    if (input_.has_length_require &&
        (arc.length < kMinArcLength || arc.length > kMaxArcLength ||
         arc.circle_info.radius > kMaxArcRadius)) {
      continue;
    }

    const uint8_t gear = geometry_lib::CalcArcGear(arc);
    const uint8_t steer = geometry_lib::CalcArcSteer(arc);

    if (!geometry_lib::IsSameGear(gear, ref_gear)) {
      continue;
    }

    if (arc.circle_info.radius < radius - 1e-3) {
      geometry_lib::Arc current_arc;
      const Eigen::Vector2d current_tang_vec = arc.headingA_vec;
      Eigen::Vector2d current_norm_vec(current_tang_vec.y(),
                                       -current_tang_vec.x());
      if (steer == geometry_lib::SEG_STEER_LEFT) {
        current_norm_vec *= -1.0;
      }
      current_arc.circle_info.center = arc.pA + current_norm_vec * radius;
      current_arc.circle_info.radius = radius;
      current_arc.pA = arc.pA;
      current_arc.headingA = arc.headingA;

      if (!geometry_lib::CalcOneArcWithLine(current_arc, ref_line,
                                            kMaxRadiusErr) ||
          (input_.has_length_require && (current_arc.length < kMinArcLength ||
                                         current_arc.length > kMaxArcLength)) ||
          geometry_lib::CalcArcGear(current_arc) != gear ||
          geometry_lib::CalcArcSteer(current_arc) != steer) {
        continue;
      }

      arc = current_arc;
    }

    if (geometry_lib::CalPoint2LineDist(arc.pB, input_.ref_line) >
        input_.lat_err) {
      continue;
    }

    // there is arc from pose success link to line
    geometry_lib::PathSegment arc_seg =
        geometry_lib::PathSegment(steer, gear, arc);

    uint8_t seg_num = 0;
    geometry_lib::PathSegment segs[MAX_LPL_PATH_NUM];
    segs[seg_num++] = arc_seg;

    // if park out, there is no need to connect arc pB to line pA generally.
    // if park in, link arc pB to line pA
    if (input_.link_line_start_pt) {
      // max lon err: 3cm, but it hardly causes any errors
      if ((arc.pB - ref_line.pA).norm() < 0.01) {
        ref_line.pA += 0.021 * ref_line.heading_vec;
      }
      geometry_lib::LineSegment end_line(arc.pB, ref_line.pA);
      end_line.heading = ref_line.heading;
      end_line.heading_vec = ref_line.heading_vec;
      const uint8_t end_gear = geometry_lib::CalcLineSegGear(end_line);

      if (geometry_lib::IsOppositeGear(end_gear, input_.ref_last_line_gear)) {
        continue;
      }

      if (input_.ref_last_line_gear == geometry_lib::SEG_GEAR_REVERSE) {
        // normal tail in
        if (arc_seg.seg_gear == geometry_lib::SEG_GEAR_REVERSE) {
          if (end_line.length < input_.reverse_last_line_min_length) {
            continue;
          }
        } else {
          if (end_line.length < input_.drive_last_line_min_length) {
            const double extern_length =
                input_.drive_last_line_min_length - end_line.length;
            geometry_lib::PathSegment extern_line_seg;
            geometry_lib::CalLineFromPt(geometry_lib::SEG_GEAR_DRIVE,
                                        extern_length, arc_seg.GetEndPose(),
                                        extern_line_seg);
            segs[seg_num++] = extern_line_seg;
          }
        }
      } else if (input_.ref_last_line_gear == geometry_lib::SEG_GEAR_DRIVE) {
        // normal heading in
        if (arc_seg.seg_gear == geometry_lib::SEG_GEAR_DRIVE) {
          if (end_line.length < input_.drive_last_line_min_length) {
            continue;
          }
        } else {
          if (end_line.length < input_.reverse_last_line_min_length) {
            const double extern_length =
                input_.reverse_last_line_min_length - end_line.length;
            geometry_lib::PathSegment extern_line_seg;
            geometry_lib::CalLineFromPt(geometry_lib::SEG_GEAR_REVERSE,
                                        extern_length, arc_seg.GetEndPose(),
                                        extern_line_seg);
            segs[seg_num++] = extern_line_seg;
          }
        }
      }

      geometry_lib::PathSegment end_line_seg(end_gear, end_line);
      segs[seg_num++] = end_line_seg;
    }

    lpl_path.SetPath(segs, seg_num);
    break;
  }

  return lpl_path.seg_num > 0;
}

const bool LinkPoseLine::TwoArcPath(LinkPoseLinePath& lpl_path,
                                    const uint8_t ref_gear,
                                    const bool same_gear) {
  lpl_path.Clear();
  // now only calc two reverse gear arc
  if (same_gear) {
    return false;
  }

  if (!same_gear && std::min(virtual_circle1_dist_, virtual_circle2_dist_) >
                        input_.min_radius + input_.lat_err) {
    return false;
  }

  double radius1 = same_gear ? input_.sturn_radius : input_.min_radius;
  double radius2 = same_gear ? input_.sturn_radius : input_.min_radius;

  if (input_.use_bigger_radius && !same_gear &&
      geometry_lib::IsOppositeGear(ref_gear, input_.ref_last_line_gear)) {
    radius2 = input_.bigger_radius_asssign;
  }

  const geometry_lib::PathPoint& pose = input_.pose;
  const double heading_err_threshold = input_.theta_err * kDeg2Rad;
  for (size_t i = 0; i < ref_line_num_; ++i) {
    geometry_lib::LineSegment& ref_line = ref_lines_[i];
    std::vector<std::pair<geometry_lib::Arc, geometry_lib::Arc>> arc_pair_vec;
    if (!geometry_lib::CalcTwoArcWithLine(pose, ref_line, radius1, radius2,
                                          arc_pair_vec)) {
      continue;
    }
    for (const auto& arc_pair : arc_pair_vec) {
      const geometry_lib::Arc& arc1 = arc_pair.first;
      const geometry_lib::Arc& arc2 = arc_pair.second;

      if (std::fabs(geometry_lib::NormalizeAngle(
              arc2.headingB - ref_line.heading)) > heading_err_threshold) {
        continue;
      }

      if (input_.has_length_require &&
          (arc1.length < kMinArcLength || arc1.length > kMaxArcLength ||
           arc2.length < kMinArcLength || arc2.length > kMaxArcLength)) {
        continue;
      }

      const uint8_t arc1_gear = geometry_lib::CalcArcGear(arc1);
      const uint8_t arc1_steer = geometry_lib::CalcArcSteer(arc1);

      if (!geometry_lib::IsSameGear(arc1_gear, ref_gear)) {
        continue;
      }

      const uint8_t arc2_gear = geometry_lib::CalcArcGear(arc2);
      const uint8_t arc2_steer = geometry_lib::CalcArcSteer(arc2);

      if (same_gear) {
        if (!geometry_lib::IsSameGear(ref_gear, arc2_gear)) {
          continue;
        }
      } else {
        if (!geometry_lib::IsOppositeGear(ref_gear, arc2_gear)) {
          continue;
        }
      }

      if (geometry_lib::CalPoint2LineDist(arc2.pB, input_.ref_line) >
          input_.lat_err) {
        continue;
      }

      geometry_lib::PathSegment arc1_seg(arc1_steer, arc1_gear, arc1);
      geometry_lib::PathSegment arc2_seg(arc2_steer, arc2_gear, arc2);

      uint8_t seg_num = 0;
      geometry_lib::PathSegment segs[MAX_LPL_PATH_NUM];
      segs[seg_num++] = arc1_seg;
      segs[seg_num++] = arc2_seg;

      if (input_.link_line_start_pt) {
        // max lon err: 3cm, but it hardly causes any errors
        if ((arc2.pB - ref_line.pA).norm() < 0.01) {
          ref_line.pA += 0.021 * ref_line.heading_vec;
        }
        geometry_lib::LineSegment end_line(arc2.pB, ref_line.pA);
        end_line.heading = ref_line.heading;
        end_line.heading_vec = ref_line.heading_vec;
        const uint8_t end_gear = geometry_lib::CalcLineSegGear(end_line);

        if (geometry_lib::IsOppositeGear(end_gear, input_.ref_last_line_gear)) {
          continue;
        }

        if (input_.ref_last_line_gear == geometry_lib::SEG_GEAR_REVERSE) {
          // normal tail in
          if (!same_gear &&
              (arc2.pA - ref_line.pA).dot(ref_line.heading_vec) < 0.2) {
            // not allow any link pt below the ref line pA
            continue;
          }

          if (arc2_seg.seg_gear == geometry_lib::SEG_GEAR_REVERSE) {
            if (end_line.length < input_.reverse_last_line_min_length) {
              continue;
            }
          } else {
            if (end_line.length < input_.drive_last_line_min_length) {
              const double extern_length =
                  input_.drive_last_line_min_length - end_line.length;
              geometry_lib::PathSegment extern_line_seg;
              geometry_lib::CalLineFromPt(geometry_lib::SEG_GEAR_DRIVE,
                                          extern_length, arc2_seg.GetEndPose(),
                                          extern_line_seg);
              segs[seg_num++] = extern_line_seg;
            }
          }
        } else if (input_.ref_last_line_gear == geometry_lib::SEG_GEAR_DRIVE) {
          // normal heading in

          if (!same_gear &&
              (arc2.pA - ref_line.pA).dot(ref_line.heading_vec) > -0.2) {
            // not allow any link pt below the ref line pA
            continue;
          }

          if (arc2_seg.seg_gear == geometry_lib::SEG_GEAR_DRIVE) {
            if (end_line.length < input_.drive_last_line_min_length) {
              continue;
            }
          } else {
            if (end_line.length < input_.reverse_last_line_min_length) {
              const double extern_length =
                  input_.reverse_last_line_min_length - end_line.length;
              geometry_lib::PathSegment extern_line_seg;
              geometry_lib::CalLineFromPt(geometry_lib::SEG_GEAR_REVERSE,
                                          extern_length, arc2_seg.GetEndPose(),
                                          extern_line_seg);
              segs[seg_num++] = extern_line_seg;
            }
          }
        }

        geometry_lib::PathSegment end_line_seg(end_gear, end_line);
        segs[seg_num++] = end_line_seg;
      }

      lpl_path.SetPath(segs, seg_num);
      break;
    }

    if (lpl_path.seg_num > 0) {
      break;
    }
  }

  return lpl_path.seg_num > 0;
}

const bool LinkPoseLine::LineArcPath(LinkPoseLinePath& lpl_path,
                                     const uint8_t ref_gear,
                                     const bool same_gear) {
  lpl_path.Clear();
  // now only calc same gear line and arc
  if (!same_gear) {
    return false;
  }

  double radius = input_.min_radius;

  if (same_gear && std::max(virtual_circle1_dist_, virtual_circle2_dist_) <
                       radius - input_.lat_err) {
    return false;
  }

  if (input_.use_bigger_radius && !same_gear &&
      geometry_lib::IsOppositeGear(ref_gear, input_.ref_last_line_gear)) {
    radius = input_.bigger_radius_asssign;
  }

  if (input_.use_bigger_radius) {
    if (!same_gear &&
        geometry_lib::IsOppositeGear(ref_gear, input_.ref_last_line_gear)) {
      radius = input_.bigger_radius_asssign;
    } else if (same_gear &&
               geometry_lib::IsSameGear(ref_gear, input_.ref_last_line_gear)) {
      radius = input_.bigger_radius_no_asssign;
    }
  }

  geometry_lib::LineSegment line1;
  line1.pA = input_.pose.pos;
  line1.heading = input_.pose.heading;
  line1.heading_vec = input_.pose.heading_vec;
  line1.length = 1.0;
  line1.pB = line1.pA + line1.heading_vec * line1.length;

  if (same_gear) {
    // 计算
    Eigen::Vector2d intersection = Eigen::Vector2d(0.0f, 0.0f);
    if (!GetIntersectionFromTwoLines(intersection, line1, input_.ref_line)) {
      return false;
    }

    geometry_lib::LineSegment line;
    line.pA = input_.pose.pos;
    line.pB = intersection;
    line.heading = input_.pose.heading;
    line.heading_vec = input_.pose.heading_vec;

    if (CalLineSegGear(line) != ref_gear) {
      return false;
    }
  }

  const double heading_err_threshold = input_.theta_err * kDeg2Rad;
  for (size_t i = 0; i < ref_line_num_; ++i) {
    geometry_lib::LineSegment& ref_line = ref_lines_[i];
    std::vector<Eigen::Vector2d> centers;
    std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d>> tangent_ptss;
    if (!CalcCommonTangentCircleOfTwoLine(line1, ref_line, radius, centers,
                                          tangent_ptss)) {
      continue;
    }

    for (size_t i = 0; i < centers.size(); ++i) {
      geometry_lib::LineSegment line(line1.pA, tangent_ptss[i].first);
      line.heading = line1.heading;
      line.heading_vec = line1.heading_vec;

      geometry_lib::Arc arc;
      arc.circle_info.center = centers[i];
      arc.circle_info.radius = radius;
      arc.pA = tangent_ptss[i].first;
      arc.headingA = line.heading;
      arc.headingA_vec = line.heading_vec;
      arc.pB = tangent_ptss[i].second;
      CompleteArcInfomation(arc);

      if (std::fabs(geometry_lib::NormalizeAngle(
              arc.headingB - ref_line.heading)) > heading_err_threshold) {
        continue;
      }

      if (input_.has_length_require &&
          (line.length < kMinArcLength || line.length > kMaxArcLength ||
           arc.length < kMinArcLength || arc.length > kMaxArcLength)) {
        continue;
      }

      if (geometry_lib::CalPoint2LineDist(arc.pB, input_.ref_line) >
          input_.lat_err) {
        continue;
      }

      const uint8_t line_gear = geometry_lib::CalcLineSegGear(line);

      if (!geometry_lib::IsSameGear(ref_gear, line_gear)) {
        continue;
      }

      const uint8_t arc_gear = geometry_lib::CalcArcGear(arc);

      if (same_gear) {
        if (!geometry_lib::IsSameGear(ref_gear, arc_gear)) {
          continue;
        }
      } else {
        if (!geometry_lib::IsOppositeGear(ref_gear, arc_gear)) {
          continue;
        }
      }

      const uint8_t arc_steer = geometry_lib::CalcArcSteer(arc);

      geometry_lib::PathSegment line_seg(line_gear, line);

      geometry_lib::PathSegment arc_seg(arc_steer, arc_gear, arc);

      uint8_t seg_num = 0;
      geometry_lib::PathSegment segs[MAX_LPL_PATH_NUM];
      segs[seg_num++] = line_seg;
      segs[seg_num++] = arc_seg;

      if (input_.link_line_start_pt) {
        // max lon err: 3cm, but it hardly causes any errors
        if ((arc.pB - ref_line.pA).norm() < 0.01) {
          ref_line.pA += 0.021 * ref_line.heading_vec;
        }
        geometry_lib::LineSegment end_line(arc.pB, ref_line.pA);
        end_line.heading = ref_line.heading;
        end_line.heading_vec = ref_line.heading_vec;
        const uint8_t end_gear = geometry_lib::CalcLineSegGear(end_line);

        if (geometry_lib::IsOppositeGear(end_gear, input_.ref_last_line_gear)) {
          continue;
        }

        if (input_.ref_last_line_gear == geometry_lib::SEG_GEAR_REVERSE) {
          // normal tail in
          if (!same_gear &&
              (arc.pA - ref_line.pA).dot(ref_line.heading_vec) < 0.2) {
            // not allow any link pt below the ref line pA
            continue;
          }

          if (arc_seg.seg_gear == geometry_lib::SEG_GEAR_REVERSE) {
            if (end_line.length < input_.reverse_last_line_min_length) {
              continue;
            }
          } else {
            if (end_line.length < input_.drive_last_line_min_length) {
              const double extern_length =
                  input_.drive_last_line_min_length - end_line.length;
              geometry_lib::PathSegment extern_line_seg;
              geometry_lib::CalLineFromPt(geometry_lib::SEG_GEAR_DRIVE,
                                          extern_length, arc_seg.GetEndPose(),
                                          extern_line_seg);
              segs[seg_num++] = extern_line_seg;
            }
          }
        } else if (input_.ref_last_line_gear == geometry_lib::SEG_GEAR_DRIVE) {
          // normal heading in

          if (!same_gear &&
              (arc.pA - ref_line.pA).dot(ref_line.heading_vec) > -0.2) {
            // not allow any link pt below the ref line pA
            continue;
          }

          if (arc_seg.seg_gear == geometry_lib::SEG_GEAR_DRIVE) {
            if (end_line.length < input_.drive_last_line_min_length) {
              continue;
            }
          } else {
            if (end_line.length < input_.reverse_last_line_min_length) {
              const double extern_length =
                  input_.reverse_last_line_min_length - end_line.length;
              geometry_lib::PathSegment extern_line_seg;
              geometry_lib::CalLineFromPt(geometry_lib::SEG_GEAR_REVERSE,
                                          extern_length, arc_seg.GetEndPose(),
                                          extern_line_seg);
              segs[seg_num++] = extern_line_seg;
            }
          }
        }

        geometry_lib::PathSegment end_line_seg(end_gear, end_line);
        segs[seg_num++] = end_line_seg;
      }

      lpl_path.SetPath(segs, seg_num);
      break;
    }

    if (lpl_path.seg_num > 0) {
      break;
    }
  }

  return lpl_path.seg_num > 0;
}

const bool LinkPoseLine::AlignBodySTurnPath(LinkPoseLinePath& lpl_path,
                                            const uint8_t ref_gear,
                                            const bool same_gear) {
  lpl_path.Clear();
  // now the align arc gear should same with sturn arc gear
  if (!same_gear) {
    return false;
  }
  const double align_radius = input_.min_radius;
  const double sturn_radius = input_.sturn_radius;
  const geometry_lib::PathPoint& pose = input_.pose;

  const double tar_heading = input_.ref_line.heading;
  const Eigen::Vector2d tar_heading_vec = input_.ref_line.heading_vec;
  double cur_heading = pose.heading;
  Eigen::Vector2d cur_heading_vec = pose.heading_vec;
  const double heading_err =
      std::fabs(geometry_lib::NormalizeAngle(cur_heading - tar_heading)) *
      kRad2Deg;
  geometry_lib::PathSegment align_arc_seg;
  bool align_arc_valid = false;
  // first align body
  if (heading_err > 26.68) {
    // err is too large, should return
    return false;
  } else if (heading_err < 0.1) {
    // err is too small, no need to align
    cur_heading = tar_heading;
    cur_heading_vec = tar_heading_vec;
  } else {
    geometry_lib::Arc arc;
    arc.pA = pose.pos;
    arc.headingA = cur_heading;
    arc.headingA_vec = cur_heading_vec;
    arc.SetRadius(align_radius);
    if (!geometry_lib::CalcOneArcWithTargetHeadingAndGear(arc, ref_gear,
                                                          tar_heading)) {
      return false;
    }

    const uint8_t steer = pnc::geometry_lib::CalcArcSteer(arc);
    const uint8_t gear = pnc::geometry_lib::CalcArcGear(arc);
    if (gear != ref_gear) {
      return false;
    }
    align_arc_seg = geometry_lib::PathSegment(steer, gear, arc);
    align_arc_valid = true;
  }

  geometry_lib::Arc arc_s_1;
  if (align_arc_valid) {
    arc_s_1.headingA = align_arc_seg.GetEndHeading();
    arc_s_1.headingA_vec = tar_heading_vec;
    arc_s_1.pA = align_arc_seg.GetEndPos();
  } else {
    arc_s_1.headingA = tar_heading;
    arc_s_1.headingA_vec = tar_heading_vec;
    arc_s_1.pA = pose.pos;
  }

  const double align_lat_err =
      geometry_lib::CalPoint2LineDist(arc_s_1.pA, input_.ref_line);

  double max_s_turn_lat_err = 1.68;
  if (std::fabs(pose.heading) * kRad2Deg < 0.168) {
    max_s_turn_lat_err = 4.0 * sturn_radius + input_.lat_err;
  }
  // if lat_dist is too big, arc1 and arc2 cannot be tangent
  if (align_lat_err > align_radius) {
    return false;
  }

  if (align_lat_err < 0.0168) {
    // lat err is small, no need to sturn path
    geometry_lib::PathSegment segs[MAX_LPL_PATH_NUM];
    uint8_t seg_num = 0;
    if (align_arc_valid) {
      segs[seg_num++] = align_arc_seg;
    }

    if (input_.link_line_start_pt) {
      // max lon err: 3cm, but it hardly causes any errors
      if ((arc_s_1.pA - input_.ref_line.pA).norm() < 0.01) {
        input_.ref_line.pA += 0.021 * input_.ref_line.heading_vec;
      }
      geometry_lib::LineSegment end_line(arc_s_1.pA, input_.ref_line.pA,
                                         input_.ref_line.heading);
      end_line.heading = input_.ref_line.heading;
      end_line.heading_vec = input_.ref_line.heading_vec;
      const uint8_t end_gear = geometry_lib::CalcLineSegGear(end_line);

      if (geometry_lib::IsOppositeGear(end_gear, input_.ref_last_line_gear)) {
        return false;
      }

      if (input_.ref_last_line_gear == geometry_lib::SEG_GEAR_REVERSE) {
        if (align_arc_seg.seg_gear == geometry_lib::SEG_GEAR_REVERSE) {
          if (end_line.length < input_.reverse_last_line_min_length) {
            return false;
          }
        } else {
          if (end_line.length < input_.drive_last_line_min_length) {
            const double extern_length =
                input_.drive_last_line_min_length - end_line.length;
            geometry_lib::PathSegment extern_line_seg;
            geometry_lib::CalLineFromPt(
                geometry_lib::SEG_GEAR_DRIVE, extern_length,
                align_arc_seg.GetEndPose(), extern_line_seg);
            segs[seg_num++] = extern_line_seg;
          }
        }
      } else if (input_.ref_last_line_gear == geometry_lib::SEG_GEAR_DRIVE) {
        // normal heading in
        if (align_arc_seg.seg_gear == geometry_lib::SEG_GEAR_DRIVE) {
          if (end_line.length < input_.drive_last_line_min_length) {
            return false;
          }
        } else {
          if (end_line.length < input_.reverse_last_line_min_length) {
            const double extern_length =
                input_.reverse_last_line_min_length - end_line.length;
            geometry_lib::PathSegment extern_line_seg;
            geometry_lib::CalLineFromPt(
                geometry_lib::SEG_GEAR_REVERSE, extern_length,
                align_arc_seg.GetEndPose(), extern_line_seg);
            segs[seg_num++] = extern_line_seg;
          }
        }
      }

      geometry_lib::PathSegment end_line_seg(end_gear, end_line);
      segs[seg_num++] = end_line_seg;
    }
    lpl_path.SetPath(segs, seg_num);
    return true;
  }

  const uint8_t s_turn_ref_gear =
      same_gear ? ref_gear : geometry_lib::ReverseGear(ref_gear);

  const double heading_err_threshold = input_.theta_err * kDeg2Rad;

  geometry_lib::Arc arc_s_2;
  arc_s_1.SetRadius(sturn_radius);
  arc_s_2.SetRadius(sturn_radius);
  for (size_t i = 0; i < ref_line_num_; ++i) {
    geometry_lib::LineSegment& ref_line = ref_lines_[i];
    arc_s_2.pB = ref_line.pA;
    arc_s_2.headingB = ref_line.heading;
    arc_s_2.headingB_vec = ref_line.heading_vec;
    if (!geometry_lib::CalcTwoArcWithSameHeading(arc_s_1, arc_s_2,
                                                 s_turn_ref_gear)) {
      continue;
    }
    if (std::fabs(geometry_lib::NormalizeAngle(
            arc_s_2.headingB - ref_line.heading)) > heading_err_threshold) {
      continue;
    }
    if (input_.has_length_require &&
        (arc_s_1.length < kMinArcLength || arc_s_1.length > kMaxArcLength ||
         arc_s_2.length < kMinArcLength || arc_s_2.length > kMaxArcLength)) {
      continue;
    }

    const uint8_t steer1 = geometry_lib::CalcArcSteer(arc_s_1);
    const uint8_t gear1 = geometry_lib::CalcArcGear(arc_s_1);

    const uint8_t steer2 = geometry_lib::CalcArcSteer(arc_s_2);
    const uint8_t gear2 = geometry_lib::CalcArcGear(arc_s_2);

    if (gear1 != s_turn_ref_gear || gear2 != s_turn_ref_gear) {
      continue;
    }

    geometry_lib::PathSegment arc_seg1(steer1, gear1, arc_s_1);
    geometry_lib::PathSegment arc_seg2(steer2, gear2, arc_s_2);

    geometry_lib::PathSegment segs[MAX_LPL_PATH_NUM];
    uint8_t seg_num = 0;
    if (align_arc_valid) {
      segs[seg_num++] = align_arc_seg;
    }
    segs[seg_num++] = arc_seg1;
    segs[seg_num++] = arc_seg2;

    if (input_.link_line_start_pt) {
      // max lon err: 3cm, but it hardly causes any errors
      if ((arc_s_2.pB - ref_line.pA).norm() < 0.01) {
        ref_line.pA += 0.021 * ref_line.heading_vec;
      }
      geometry_lib::LineSegment end_line(arc_s_2.pB, ref_line.pA);
      end_line.heading = ref_line.heading;
      end_line.heading_vec = ref_line.heading_vec;
      const uint8_t end_gear = geometry_lib::CalcLineSegGear(end_line);

      if (geometry_lib::IsOppositeGear(end_gear, input_.ref_last_line_gear)) {
        continue;
      }

      if (input_.ref_last_line_gear == geometry_lib::SEG_GEAR_REVERSE) {
        // normal tail in
        if (!same_gear &&
            (arc_s_1.pA - ref_line.pA).dot(ref_line.heading_vec) < 0.2) {
          // not allow any link pt below the ref line pA
          continue;
        }

        if (arc_seg2.seg_gear == geometry_lib::SEG_GEAR_REVERSE) {
          if (end_line.length < input_.reverse_last_line_min_length) {
            continue;
          }
        } else {
          if (end_line.length < input_.drive_last_line_min_length) {
            const double extern_length =
                input_.drive_last_line_min_length - end_line.length;
            geometry_lib::PathSegment extern_line_seg;
            geometry_lib::CalLineFromPt(geometry_lib::SEG_GEAR_DRIVE,
                                        extern_length, arc_seg2.GetEndPose(),
                                        extern_line_seg);
            segs[seg_num++] = extern_line_seg;
          }
        }
      } else if (input_.ref_last_line_gear == geometry_lib::SEG_GEAR_DRIVE) {
        // normal heading in

        if (!same_gear &&
            (arc_s_1.pA - ref_line.pA).dot(ref_line.heading_vec) > -0.2) {
          // not allow any link pt below the ref line pA
          continue;
        }

        if (arc_seg2.seg_gear == geometry_lib::SEG_GEAR_DRIVE) {
          if (end_line.length < input_.drive_last_line_min_length) {
            continue;
          }
        } else {
          if (end_line.length < input_.reverse_last_line_min_length) {
            const double extern_length =
                input_.reverse_last_line_min_length - end_line.length;
            geometry_lib::PathSegment extern_line_seg;
            geometry_lib::CalLineFromPt(geometry_lib::SEG_GEAR_REVERSE,
                                        extern_length, arc_seg2.GetEndPose(),
                                        extern_line_seg);
            segs[seg_num++] = extern_line_seg;
          }
        }
      }

      geometry_lib::PathSegment end_line_seg(end_gear, end_line);
      segs[seg_num++] = end_line_seg;
    }

    lpl_path.SetPath(segs, seg_num);
    break;
  }

  return lpl_path.seg_num > 0;
}

void LinkPoseLinePath::SetPath(const geometry_lib::PathSegment& seg) {}

void LinkPoseLinePath::SetPath(const geometry_lib::PathSegment _segs[],
                               uint8_t _seg_num) {
  Clear();
  if (_seg_num < 1 || _seg_num > MAX_LPL_PATH_NUM) {
    return;
  }
  seg_num = _seg_num;
  cur_gear = _segs[0].seg_gear;
  last_gear = _segs[_seg_num - 1].seg_gear;
  last_steer = _segs[_seg_num - 1].seg_steer;
  single_gear_lengths[gear_change_num] = _segs[0].GetLength();
  gear_num++;
  for (uint8_t i = 0; i < seg_num; ++i) {
    segs[i] = _segs[i];
    gears[i] = segs[i].seg_gear;
    steers[i] = segs[i].seg_steer;
    lengths[i] = segs[i].GetLength();
    total_length += lengths[i];

    if (steers[i] == geometry_lib::SEG_STEER_STRAIGHT) {
      kappas[i] = 0.0;
    } else if (steers[i] == geometry_lib::SEG_STEER_LEFT) {
      kappas[i] = 1.0 / std::max(segs[i].GetRadius(), 0.01);
    } else {
      kappas[i] = -1.0 / std::max(segs[i].GetRadius(), 0.01);
    }

    if (i > 0) {
      if (gears[i] == gears[i - 1]) {
        single_gear_lengths[gear_change_num] += segs[i].GetLength();
        if (std::fabs(kappas[i]) > 1e-3f) {
          kappa_change += std::fabs(kappas[i] - kappas[i - 1]);
        }
      } else {
        single_gear_lengths[++gear_change_num] = segs[i].GetLength();
        gear_num++;
      }
    }

    if (i == seg_num - 1 && steers[i] == geometry_lib::SEG_STEER_STRAIGHT) {
      last_line_length = segs[i].GetLength();
    }
  }
  cur_gear_length = single_gear_lengths[0];
}

void LinkPoseLinePath::SetPath(
    const std::vector<geometry_lib::PathSegment>& seg_vec) {}

void LinkPoseLinePath::AddPath(const geometry_lib::PathSegment& seg) {}

void LinkPoseLinePath::AddPath(const geometry_lib::PathSegment _segs[],
                               uint8_t _seg_num) {}

void LinkPoseLinePath::AddPath(
    const std::vector<geometry_lib::PathSegment>& seg_vec) {}

void LinkPoseLinePath::SamplePath(const double sample_ds) {
  ptss.clear();
  ptss.resize(seg_num);
  for (size_t i = 0; i < seg_num; ++i) {
    SamplePointSetInPathSeg(ptss[i], segs[i], sample_ds, kappas[i]);
  }
}

void LinkPoseLinePath::PrintInfo(const bool enable_log) const {
  ILOG_INFO_IF(enable_log) << "path_count = " << static_cast<int>(seg_num)
                           << "  gear_change_count = "
                           << static_cast<int>(gear_change_num)
                           << "  gear_count = " << static_cast<int>(gear_num)
                           << "  total_length = " << total_length
                           << "  cur_gear_length = " << cur_gear_length
                           << "  cur_gear = "
                           << geometry_lib::GetGearString(cur_gear);

  for (size_t i = 0; i < seg_num; i++) {
    ILOG_INFO_IF(enable_log) << "Segment [" << i << "]";
    segs[i].PrintInfo(enable_log);
  }
}

}  // namespace planning