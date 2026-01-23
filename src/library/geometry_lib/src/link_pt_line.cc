#include "link_pt_line.h"

#include <sys/types.h>

#include <cstdint>

#include "common_math.h"
#include "library/hybrid_astar_lib/hybrid_astar_common.h"
#include "log_glog.h"

namespace planning {
namespace link_pt_line {

static const float kMaxArcLengthF = 12.68f;
static const float kMinArcLengthF = 0.0168f;
static const float kMaxArcRadiusF = 15.68f;
static const float kMaxRadiusErrF = 2e-2f;

template <typename T>
const bool LinkPtLine<T>::CalLPLPath(const LinkPtLineInput<T>& input) {
  input_ = input;
  ref_line_num_ = 0;
  output_.Reset();

  const T max_lat_err = std::fabs(input_.lat_err);
  const T step = 1e-2;

  T lar_errs[MAX_REF_LINE_NUM];
  lar_errs[ref_line_num_++] = 0.0;
  for (T lat_err = step;
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

  const Pos<T> tang_vec = input.ref_line.dir;

  const Pos<T> move_vec(-tang_vec.y(), tang_vec.x());

  for (uint8_t i = 0; i < ref_line_num_; ++i) {
    LineSeg<T> ref_line = input_.ref_line;
    if (std::fabs(lar_errs[i]) > 1e-4) {
      ref_line.pA += move_vec * lar_errs[i];
      ref_line.pB += move_vec * lar_errs[i];
    }
    ref_lines_[i] = ref_line;
  }

  const Pos<T> t = input_.pose.dir;
  Pos<T> n(-t.y(), t.x());
  Pos<T> center = input_.pose.pos + n * input_.min_radius;
  virtual_circle1_dist_ = CalPt2LineDist(center, input_.ref_line);
  n *= -1.0;
  center = input_.pose.pos + n * input_.min_radius;
  virtual_circle2_dist_ = CalPt2LineDist(center, input_.ref_line);

  LinkPtLinePath<T> lpl_path;
  for (uint8_t i = 0; i < MAX_GEAR_NUM; ++i) {
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

template <typename T>
const bool LinkPtLine<T>::OneArcPath(LinkPtLinePath<T>& lpl_path,
                                     const AstarPathGear ref_gear) {
  lpl_path.Clear();
  T radius = input_.min_radius;

  if (std::max(virtual_circle1_dist_, virtual_circle2_dist_) <
      radius - input_.lat_err) {
    return false;
  }

  if (input_.use_bigger_radius &&
      IsGearSame(ref_gear, input_.ref_last_line_gear)) {
    radius = input_.bigger_radius_no_asssign;
  }

  ArcSeg<T> arc;
  arc.pA = input_.pose.pos;
  arc.thetaA = input_.pose.theta;
  arc.dirA = input_.pose.dir;

  const T theta_err_threshold = input_.theta_err * kDeg2RadF;

  for (uint8_t i = 0; i < ref_line_num_; ++i) {
    const LineSeg<T>& ref_line = ref_lines_[i];

    if (!CalOneArcWithLineAndGear(arc, ref_line, ref_gear, radius)) {
      continue;
    }

    if (std::fabs(UnifyAngleDiff(arc.thetaB, ref_line.theta)) >
        theta_err_threshold) {
      continue;
    }

    if (input_.has_length_require &&
        (arc.length < kMinArcLengthF || arc.length > kMaxArcLengthF ||
         arc.radius > kMaxArcRadiusF)) {
      continue;
    }

    if (CalPt2LineDist(arc.pB, ref_line) > input_.lat_err) {
      continue;
    }

    const AstarPathGear gear = CalArcSegGear(arc);
    const AstarPathSteer steer = CalArcSegSteer(arc);

    if (!IsGearSame(gear, ref_gear)) {
      continue;
    }

    PathSeg<T> arc_seg(arc, gear, steer);

    uint8_t seg_num = 0;
    PathSeg<T> segs[MAX_LPL_PATH_NUM];
    segs[seg_num++] = arc_seg;

    // if park out, there is no need to connect arc pB to line pA generally.
    // if park in, link arc pB to line pA
    if (input_.link_line_start_pt) {
      LineSeg<T> end_line = ref_line;
      end_line.SetEndPt(arc.pB, ref_line.pA);
      // max lon err: 3cm, but it hardly causes any errors
      if ((end_line.pA - end_line.pB).norm() < T(0.01f)) {
        end_line.pB += T(0.021f) * end_line.dir;
      }

      const AstarPathGear end_gear = CalLineSegGear(end_line);

      if (IsGearDifferent(end_gear, input_.ref_last_line_gear)) {
        continue;
      }

      if (input_.ref_last_line_gear == AstarPathGear::REVERSE) {
        // normal tail in
        if (arc_seg.gear == AstarPathGear::REVERSE) {
          if (end_line.length < input_.reverse_last_line_min_length) {
            continue;
          }
        } else {
          if (end_line.length < input_.drive_last_line_min_length) {
            const T extern_length =
                std::max(input_.drive_last_line_min_length - end_line.length,
                         T(0.0168f));

            PathSeg<T> extern_line_seg;
            CalLineSegByGearAndPose(AstarPathGear::DRIVE, extern_length,
                                    arc_seg.GetEndPose(), extern_line_seg);

            segs[seg_num++] = extern_line_seg;
          }
        }
      } else if (input_.ref_last_line_gear == AstarPathGear::DRIVE) {
        // reverse heading in
        if (arc_seg.gear == AstarPathGear::DRIVE) {
          if (end_line.length < input_.drive_last_line_min_length) {
            continue;
          }
        } else {
          if (end_line.length < input_.reverse_last_line_min_length) {
            const T extern_length =
                std::max(input_.reverse_last_line_min_length - end_line.length,
                         T(0.0168f));
            PathSeg<T> extern_line_seg;
            CalLineSegByGearAndPose(AstarPathGear::REVERSE, extern_length,
                                    arc_seg.GetEndPose(), extern_line_seg);
            segs[seg_num++] = extern_line_seg;
          }
        }
      }

      PathSeg<T> end_line_seg(end_line, end_gear);
      segs[seg_num++] = end_line_seg;
    }

    lpl_path.SetPathSegs(segs, seg_num);
    break;
  }

  return lpl_path.seg_num > 0;
}

template <typename T>
const bool LinkPtLine<T>::TwoArcPath(LinkPtLinePath<T>& lpl_path,
                                     const AstarPathGear ref_gear,
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

  T radius1 = same_gear ? input_.sturn_radius : input_.min_radius;
  T radius2 = same_gear ? input_.sturn_radius : input_.min_radius;

  if (input_.use_bigger_radius && !same_gear &&
      IsGearDifferent(ref_gear, input_.ref_last_line_gear)) {
    radius2 = input_.bigger_radius_asssign;
  }

  const PathPt<T>& pose = input_.pose;
  const T theta_err_threshold = input_.theta_err * kDeg2RadF;
  std::vector<std::pair<ArcSeg<T>, ArcSeg<T>>> arc_pair_vec;
  for (uint8_t i = 0; i < ref_line_num_; ++i) {
    const LineSeg<T>& ref_line = ref_lines_[i];
    if (!CalTwoArcWithLine(pose, ref_line, radius1, radius2, arc_pair_vec)) {
      continue;
    }

    for (const auto& arc_pair : arc_pair_vec) {
      const ArcSeg<T>& arc1 = arc_pair.first;
      const ArcSeg<T>& arc2 = arc_pair.second;

      if (std::fabs(UnifyAngleDiff(arc2.thetaB, ref_line.theta)) >
          theta_err_threshold) {
        continue;
      }

      if (input_.has_length_require &&
          (arc1.length < kMinArcLengthF || arc1.length > kMaxArcLengthF ||
           arc2.length < kMinArcLengthF || arc2.length > kMaxArcLengthF)) {
        continue;
      }

      if (CalPt2LineDist(arc2.pB, ref_line) > input_.lat_err) {
        continue;
      }

      const AstarPathGear gear1 = CalArcSegGear(arc1);
      const AstarPathSteer steer1 = CalArcSegSteer(arc1);

      if (!IsGearSame(gear1, ref_gear)) {
        continue;
      }

      const AstarPathGear gear2 = CalArcSegGear(arc2);
      const AstarPathSteer steer2 = CalArcSegSteer(arc2);

      if (same_gear) {
        if (!IsGearSame(gear2, ref_gear)) {
          continue;
        }
      } else {
        if (!IsGearDifferent(gear2, ref_gear)) {
          continue;
        }
      }

      PathSeg<T> arc_seg1(arc1, gear1, steer1);
      PathSeg<T> arc_seg2(arc2, gear2, steer2);

      uint8_t seg_num = 0;
      PathSeg<T> segs[MAX_LPL_PATH_NUM];
      segs[seg_num++] = arc_seg1;
      segs[seg_num++] = arc_seg2;

      if (input_.link_line_start_pt) {
        LineSeg<T> end_line = ref_line;
        end_line.SetEndPt(arc2.pB, ref_line.pA);
        // max lon err: 3cm, but it hardly causes any errors
        if ((end_line.pA - end_line.pB).norm() < T(0.01f)) {
          end_line.pB += T(0.021f) * end_line.dir;
        }

        const AstarPathGear end_gear = CalLineSegGear(end_line);

        if (IsGearDifferent(end_gear, input_.ref_last_line_gear)) {
          continue;
        }

        if (input_.ref_last_line_gear == AstarPathGear::REVERSE) {
          // normal tail in
          if (arc_seg2.gear == AstarPathGear::REVERSE) {
            if (end_line.length < input_.reverse_last_line_min_length) {
              continue;
            }
          } else {
            if (end_line.length < input_.drive_last_line_min_length) {
              const T extern_length =
                  std::max(input_.drive_last_line_min_length - end_line.length,
                           T(0.0168f));

              PathSeg<T> extern_line_seg;
              CalLineSegByGearAndPose(AstarPathGear::DRIVE, extern_length,
                                      arc_seg2.GetEndPose(), extern_line_seg);

              segs[seg_num++] = extern_line_seg;
            }
          }
        } else if (input_.ref_last_line_gear == AstarPathGear::DRIVE) {
          // reverse heading in
          if (arc_seg2.gear == AstarPathGear::DRIVE) {
            if (end_line.length < input_.drive_last_line_min_length) {
              continue;
            }
          } else {
            if (end_line.length < input_.reverse_last_line_min_length) {
              const T extern_length = std::max(
                  input_.reverse_last_line_min_length - end_line.length,
                  T(0.0168f));
              PathSeg<T> extern_line_seg;
              CalLineSegByGearAndPose(AstarPathGear::REVERSE, extern_length,
                                      arc_seg2.GetEndPose(), extern_line_seg);
              segs[seg_num++] = extern_line_seg;
            }
          }
        }

        PathSeg<T> end_line_seg(end_line, end_gear);
        segs[seg_num++] = end_line_seg;
      }

      lpl_path.SetPathSegs(segs, seg_num);
      break;
    }
    if (lpl_path.seg_num > 0) {
      break;
    }
  }

  return lpl_path.seg_num > 0;
}

template <typename T>
const bool LinkPtLine<T>::LineArcPath(LinkPtLinePath<T>& lpl_path,
                                      const AstarPathGear ref_gear,
                                      const bool same_gear) {
  lpl_path.Clear();
  // now only calc same gear line and arc
  if (!same_gear) {
    return false;
  }

  T radius = input_.min_radius;

  if (same_gear && std::max(virtual_circle1_dist_, virtual_circle2_dist_) <
                       radius - input_.lat_err) {
    return false;
  }

  if (input_.use_bigger_radius) {
    if (!same_gear && IsGearDifferent(ref_gear, input_.ref_last_line_gear)) {
      radius = input_.bigger_radius_asssign;
    } else if (same_gear && IsGearSame(ref_gear, input_.ref_last_line_gear)) {
      radius = input_.bigger_radius_no_asssign;
    }
  }

  LineSeg<T> line1(input_.pose.pos, input_.pose.dir, input_.pose.theta, 1.0);

  if (same_gear) {
    // 计算
    Pos<T> intersection = Pos<T>(0.0f, 0.0f);
    if (!CalIntersectionOfTwoLines(intersection, line1, input_.ref_line)) {
      return false;
    }

    LineSeg<T> line(input_.pose.pos, intersection, input_.pose.theta,
                    input_.pose.dir);

    if (CalLineSegGear(line) != ref_gear) {
      return false;
    }
  }

  const T theta_err_threshold = input_.theta_err * kDeg2RadF;
  for (uint8_t i = 0; i < ref_line_num_; ++i) {
    const LineSeg<T>& ref_line = ref_lines_[i];
    std::vector<Pos<T>> centers;
    std::vector<std::pair<Pos<T>, Pos<T>>> tangent_ptss;
    if (!CalCommonTangentCircleOfTwoLine(line1, ref_line, radius, centers,
                                         tangent_ptss)) {
      continue;
    }

    for (uint8_t j = 0; j < centers.size(); ++j) {
      LineSeg<T> line(line1.pA, tangent_ptss[j].first, line1.theta, line1.dir);

      ArcSeg<T> arc(centers[j], radius);
      arc.pA = tangent_ptss[j].first;
      arc.thetaA = line1.theta;
      arc.dirA = line1.dir;
      arc.pB = tangent_ptss[j].second;

      CompleteArcSeg(arc);

      if (std::fabs(UnifyAngleDiff(arc.thetaB, ref_line.theta)) >
          theta_err_threshold) {
        continue;
      }

      if (input_.has_length_require &&
          (line.length < kMinArcLengthF || line.length > kMaxArcLengthF ||
           arc.length < kMinArcLengthF || arc.length > kMaxArcLengthF)) {
        continue;
      }

      if (CalPt2LineDist(arc.pB, ref_line) > input_.lat_err) {
        continue;
      }

      const AstarPathGear line_gear = CalLineSegGear(line);
      if (!IsGearSame(line_gear, ref_gear)) {
        continue;
      }

      const AstarPathGear arc_gear = CalArcSegGear(arc);
      if (same_gear) {
        if (!IsGearSame(ref_gear, arc_gear)) {
          continue;
        }
      } else {
        if (!IsGearDifferent(ref_gear, arc_gear)) {
          continue;
        }
      }

      const AstarPathSteer arc_steer = CalArcSegSteer(arc);

      PathSeg<T> line_seg(line, line_gear);
      PathSeg<T> arc_seg(arc, arc_gear, arc_steer);

      uint8_t seg_num = 0;
      PathSeg<T> segs[MAX_LPL_PATH_NUM];
      segs[seg_num++] = line_seg;
      segs[seg_num++] = arc_seg;

      // if park out, there is no need to connect arc pB to line pA generally.
      // if park in, link arc pB to line pA
      if (input_.link_line_start_pt) {
        LineSeg<T> end_line = ref_line;
        end_line.SetEndPt(arc.pB, ref_line.pA);
        // max lon err: 3cm, but it hardly causes any errors
        if ((end_line.pA - end_line.pB).norm() < T(0.01f)) {
          end_line.pB += T(0.021f) * end_line.dir;
        }

        const AstarPathGear end_gear = CalLineSegGear(end_line);

        if (IsGearDifferent(end_gear, input_.ref_last_line_gear)) {
          continue;
        }

        if (input_.ref_last_line_gear == AstarPathGear::REVERSE) {
          // normal tail in
          if (arc_seg.gear == AstarPathGear::REVERSE) {
            if (end_line.length < input_.reverse_last_line_min_length) {
              continue;
            }
          } else {
            if (end_line.length < input_.drive_last_line_min_length) {
              const T extern_length =
                  std::max(input_.drive_last_line_min_length - end_line.length,
                           T(0.0168f));

              PathSeg<T> extern_line_seg;
              CalLineSegByGearAndPose(AstarPathGear::DRIVE, extern_length,
                                      arc_seg.GetEndPose(), extern_line_seg);

              segs[seg_num++] = extern_line_seg;
            }
          }
        } else if (input_.ref_last_line_gear == AstarPathGear::DRIVE) {
          // reverse heading in
          if (arc_seg.gear == AstarPathGear::DRIVE) {
            if (end_line.length < input_.drive_last_line_min_length) {
              continue;
            }
          } else {
            if (end_line.length < input_.reverse_last_line_min_length) {
              const T extern_length = std::max(
                  input_.reverse_last_line_min_length - end_line.length,
                  T(0.0168f));
              PathSeg<T> extern_line_seg;
              CalLineSegByGearAndPose(AstarPathGear::REVERSE, extern_length,
                                      arc_seg.GetEndPose(), extern_line_seg);
              segs[seg_num++] = extern_line_seg;
            }
          }
        }

        PathSeg<T> end_line_seg(end_line, end_gear);
        segs[seg_num++] = end_line_seg;
      }

      lpl_path.SetPathSegs(segs, seg_num);
      break;
    }
    if (lpl_path.seg_num > 0) {
      break;
    }
  }

  return lpl_path.seg_num > 0;
}

template <typename T>
const bool LinkPtLine<T>::AlignBodySTurnPath(LinkPtLinePath<T>& lpl_path,
                                             const AstarPathGear ref_gear,
                                             const bool same_gear) {
  lpl_path.Clear();
  // now the align arc gear should same with sturn arc gear
  if (!same_gear) {
    return false;
  }

  const T align_radius = input_.min_radius;
  const T sturn_radius = input_.sturn_radius;
  const PathPt<T>& pose = input_.pose;

  const T tar_theta = input_.ref_line.theta;
  const Pos<T> tar_dir = input_.ref_line.dir;
  T cur_theta = pose.theta;
  Pos<T> cur_dir = pose.dir;
  const T theta_err =
      std::fabs(UnifyAngleDiff(tar_theta, cur_theta)) * kRad2DegF;
  PathSeg<T> align_arc_seg;
  bool align_arc_valid = false;

  // first align body
  if (theta_err > 26.68) {
    // err is too large, should return
    return false;
  } else if (theta_err < 0.1) {
    // err is too small, no need to align
    cur_theta = tar_theta;
    cur_dir = tar_dir;
  } else {
    // should align
    ArcSeg<T> align_arc;
    align_arc.pA = pose.pos;
    align_arc.thetaA = pose.theta;
    align_arc.dirA = pose.dir;
    align_arc.radius = align_radius;

    if (!CalOneArcWithTargetThetaAndGear(align_arc, ref_gear, tar_theta)) {
      return false;
    }

    const AstarPathGear align_gear = CalArcSegGear(align_arc);
    if (!IsGearSame(align_gear, ref_gear)) {
      return false;
    }
    const AstarPathSteer align_steer = CalArcSegSteer(align_arc);
    align_arc_seg = PathSeg<T>(align_arc, align_gear, align_steer);
    align_arc_valid = true;
  }

  ArcSeg<T> arc_s_1;
  if (align_arc_valid) {
    arc_s_1.thetaA = align_arc_seg.GetEndTheta();
    arc_s_1.dirA = align_arc_seg.GetEndDir();
    arc_s_1.pA = align_arc_seg.GetEndPos();
  } else {
    arc_s_1.thetaA = tar_theta;
    arc_s_1.dirA = tar_dir;
    arc_s_1.pA = pose.pos;
  }

  const T align_lat_err = CalPt2LineDist(arc_s_1.pA, input_.ref_line);

  // if lat_dist is too big, arc1 and arc2 cannot be tangent
  if (align_lat_err > T(2.0f) * sturn_radius + input_.lat_err) {
    return false;
  }

  // if lat err is small, no need to sturn path
  if (align_lat_err < T(0.0168f)) {
    PathSeg<T> segs[MAX_LPL_PATH_NUM];
    uint8_t seg_num = 0;
    if (align_arc_valid) {
      segs[seg_num++] = align_arc_seg;
    }

    // if park out, there is no need to connect arc pB to line pA generally.
    // if park in, link arc pB to line pA
    if (input_.link_line_start_pt) {
      // max lon err: 3cm, but it hardly causes any errors.
      if ((arc_s_1.pA - input_.ref_line.pA).norm() < T(0.01f)) {
        input_.ref_line.pA += T(0.021f) * input_.ref_line.dir;
      }
      LineSeg<T> end_line = input_.ref_line;
      end_line.SetEndPt(arc_s_1.pA, input_.ref_line.pA);

      const AstarPathGear end_gear = CalLineSegGear(end_line);

      if (IsGearDifferent(end_gear, input_.ref_last_line_gear)) {
        return false;
      }

      if (!align_arc_valid) {
        // only end line
        if (input_.ref_last_line_gear == AstarPathGear::REVERSE &&
            end_line.length < input_.reverse_last_line_min_length) {
          return false;
        }
        if (input_.ref_last_line_gear == AstarPathGear::DRIVE &&
            end_line.length < input_.drive_last_line_min_length) {
          return false;
        }
      } else {
        // align arc + end line
        if (input_.ref_last_line_gear == AstarPathGear::REVERSE) {
          // normal tail in
          if (align_arc_seg.gear == AstarPathGear::REVERSE) {
            if (end_line.length < input_.reverse_last_line_min_length) {
              return false;
            }
          } else {
            if (end_line.length < input_.drive_last_line_min_length) {
              const T extern_length =
                  std::max(input_.drive_last_line_min_length - end_line.length,
                           T(0.0168f));

              PathSeg<T> extern_line_seg;
              CalLineSegByGearAndPose(AstarPathGear::DRIVE, extern_length,
                                      align_arc_seg.GetEndPose(),
                                      extern_line_seg);

              segs[seg_num++] = extern_line_seg;
            }
          }
        } else if (input_.ref_last_line_gear == AstarPathGear::DRIVE) {
          // reverse heading in
          if (align_arc_seg.gear == AstarPathGear::DRIVE) {
            if (end_line.length < input_.drive_last_line_min_length) {
              return false;
            }
          } else {
            if (end_line.length < input_.reverse_last_line_min_length) {
              const T extern_length = std::max(
                  input_.reverse_last_line_min_length - end_line.length,
                  T(0.0168f));
              PathSeg<T> extern_line_seg;
              CalLineSegByGearAndPose(AstarPathGear::REVERSE, extern_length,
                                      align_arc_seg.GetEndPose(),
                                      extern_line_seg);
              segs[seg_num++] = extern_line_seg;
            }
          }
        }
      }

      PathSeg<T> end_line_seg(end_line, end_gear);
      segs[seg_num++] = end_line_seg;
    }

    lpl_path.SetPathSegs(segs, seg_num);
    return true;
  }

  const AstarPathGear s_turn_ref_gear =
      same_gear ? ref_gear : ReversePathGear(ref_gear);

  const T theta_err_threshold = input_.theta_err * kDeg2RadF;

  ArcSeg<T> arc_s_2;
  arc_s_1.radius = sturn_radius;
  arc_s_2.radius = sturn_radius;

  for (uint8_t i = 0; i < ref_line_num_; ++i) {
    const LineSeg<T>& ref_line = ref_lines_[i];
    arc_s_2.pB = ref_line.pA;
    arc_s_2.thetaB = ref_line.theta;
    arc_s_2.dirB = ref_line.dir;

    if (!CalTwoArcWithSameThetaAndGear(arc_s_1, arc_s_2, s_turn_ref_gear)) {
      continue;
    }

    if (std::fabs(UnifyAngleDiff(arc_s_2.thetaB, ref_line.theta)) >
        theta_err_threshold) {
      continue;
    }

    if (input_.has_length_require &&
        (arc_s_1.length < kMinArcLengthF || arc_s_1.length > kMaxArcLengthF ||
         arc_s_2.length < kMinArcLengthF || arc_s_2.length > kMaxArcLengthF)) {
      continue;
    }

    const AstarPathSteer steer1 = CalArcSegSteer(arc_s_1);
    const AstarPathGear gear1 = CalArcSegGear(arc_s_1);

    const AstarPathSteer steer2 = CalArcSegSteer(arc_s_2);
    const AstarPathGear gear2 = CalArcSegGear(arc_s_2);

    if (gear1 != s_turn_ref_gear || gear2 != s_turn_ref_gear) {
      continue;
    }

    PathSeg<T> arc_seg_1(arc_s_1, gear1, steer1);
    PathSeg<T> arc_seg_2(arc_s_2, gear2, steer2);
    PathSeg<T> segs[MAX_LPL_PATH_NUM];
    uint8_t seg_num = 0;
    if (align_arc_valid) {
      segs[seg_num++] = align_arc_seg;
    }
    segs[seg_num++] = arc_seg_1;
    segs[seg_num++] = arc_seg_2;

    // if park out, there is no need to connect arc pB to line pA generally.
    // if park in, link arc pB to line pA
    if (input_.link_line_start_pt) {
      LineSeg<T> end_line = ref_line;
      end_line.SetEndPt(arc_s_2.pB, ref_line.pA);
      // max lon err: 3cm, but it hardly causes any errors
      if ((end_line.pA - end_line.pB).norm() < T(0.01f)) {
        end_line.pB += T(0.021f) * end_line.dir;
      }

      const AstarPathGear end_gear = CalLineSegGear(end_line);

      if (IsGearDifferent(end_gear, input_.ref_last_line_gear)) {
        continue;
      }

      if (input_.ref_last_line_gear == AstarPathGear::REVERSE) {
        // normal tail in
        if (arc_seg_2.gear == AstarPathGear::REVERSE) {
          if (end_line.length < input_.reverse_last_line_min_length) {
            continue;
          }
        } else {
          if (end_line.length < input_.drive_last_line_min_length) {
            const T extern_length =
                std::max(input_.drive_last_line_min_length - end_line.length,
                         T(0.0168f));

            PathSeg<T> extern_line_seg;
            CalLineSegByGearAndPose(AstarPathGear::DRIVE, extern_length,
                                    arc_seg_2.GetEndPose(), extern_line_seg);

            segs[seg_num++] = extern_line_seg;
          }
        }
      } else if (input_.ref_last_line_gear == AstarPathGear::DRIVE) {
        // reverse heading in
        if (arc_seg_2.gear == AstarPathGear::DRIVE) {
          if (end_line.length < input_.drive_last_line_min_length) {
            continue;
          }
        } else {
          if (end_line.length < input_.reverse_last_line_min_length) {
            const T extern_length =
                std::max(input_.reverse_last_line_min_length - end_line.length,
                         T(0.0168f));
            PathSeg<T> extern_line_seg;
            CalLineSegByGearAndPose(AstarPathGear::REVERSE, extern_length,
                                    arc_seg_2.GetEndPose(), extern_line_seg);
            segs[seg_num++] = extern_line_seg;
          }
        }
      }

      PathSeg<T> end_line_seg(end_line, end_gear);
      segs[seg_num++] = end_line_seg;
    }

    lpl_path.SetPathSegs(segs, seg_num);
    break;
  }
  return lpl_path.seg_num > 0;
}

template <typename T>
void LinkPtLinePath<T>::SetPathSegs(const PathSeg<T> _segs[],
                                    const uint8_t _seg_num) {
  Reset();
  if (_seg_num < 1 || _seg_num > MAX_LPL_PATH_NUM) {
    return;
  }
  seg_num = _seg_num;
  cur_gear = _segs[0].gear;
  last_gear = _segs[_seg_num - 1].gear;
  last_steer = _segs[_seg_num - 1].steer;

  for (uint8_t i = 0; i < seg_num; ++i) {
    segs[i] = _segs[i];
    gears[i] = segs[i].gear;
    steers[i] = segs[i].steer;
    kappas[i] = segs[i].kappa;
    lengths[i] = segs[i].GetLength();
    total_length += segs[i].GetLength();

    if (i > 0) {
      if (gears[i] == gears[i - 1]) {
        if (std::fabs(kappas[i]) > T(1e-3f)) {
          kappa_change += std::fabs(kappas[i] - kappas[i - 1]);
        }
      }
    }

    if (i == seg_num - 1 && steers[i] == AstarPathSteer::STRAIGHT) {
      last_line_length = segs[i].GetLength();
    }
  }
}

template <typename T>
void LinkPtLinePath<T>::SamplePath(const T sample_ds) {
  ptss.clear();
  ptss.resize(seg_num);
  for (uint8_t i = 0; i < seg_num; ++i) {
    SamplePathSeg(ptss[i], segs[i], sample_ds, kappas[i]);
  }
}

template <typename T>
void LinkPtLinePath<T>::PrintInfo(const bool enable_log) const {
  ILOG_INFO_IF(enable_log) << "path_count = " << static_cast<int>(seg_num)
                           << "  gear_change_count = "
                           << static_cast<int>(gear_change_num)
                           << "  total_length = " << total_length
                           << "  kappa_change = " << kappa_change
                           << "  cur_gear = " << PathGearDebugString(cur_gear);

  for (size_t i = 0; i < seg_num; i++) {
    ILOG_INFO_IF(enable_log) << "Segment [" << i << "]";
    segs[i].PrintInfo(enable_log);
  }
}

// 显式模板实例化（解决链接问题）
template struct LinkPtLinePath<float>;   // 支持float
template struct LinkPtLinePath<double>;  // 支持double

template class LinkPtLine<float>;   // 支持float
template class LinkPtLine<double>;  // 支持double

}  // namespace link_pt_line
}  // namespace planning