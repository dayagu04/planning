#pragma once

#include <Eigen/Core>
#include <cstdint>
#include <vector>

#include "../../hybrid_astar_lib/hybrid_astar_common.h"

namespace planning {
namespace common_math {

extern const float kDeg2RadF;
extern const float kRad2DegF;

#define square(x) ((x) * (x))

// template <typename T>
// struct EigenVector {
//   using type = Eigen::Matrix<T, 2, 1>;
// };

template <typename T>
using Pos = Eigen::Matrix<T, 2, 1>;

template <typename T>
const T UnifyAngle(const T angle);

template <typename T>
const T UnifyAngleDiff(const T angle1, const T angle2);

template <typename T>
const T UnifyAngleSum(const T angle1, const T angle2);

template <typename T>
const Eigen::Matrix<T, 2, 2> CalRotMatFromTheta(const T theta);

template <typename T1, typename T2>
const T1 CalTwoPosDist(const Eigen::Matrix<T1, 2, 1>& p1,
                       const Eigen::Matrix<T2, 2, 1>& p2) {
  return std::sqrt((p1(0) - p2(0)) * (p1(0) - p2(0)) +
                   (p1(1) - p2(1)) * (p1(1) - p2(1)));
}

template <typename T>
struct Global2LocalTrans {
  Pos<T> origin = Pos<T>(0.0f, 0.0f);
  T theta = T(0.0f);
  Eigen::Matrix<T, 2, 2> rot_mat = Eigen::Matrix<T, 2, 2>::Identity();

  Global2LocalTrans() {}
  ~Global2LocalTrans() {}

  Global2LocalTrans(const Pos<T>& _origin, const T _theta) {
    Init(_origin, _theta);
  }

  void Init(const Pos<T>& _origin, const T _theta) {
    origin = _origin;
    theta = _theta;
    const T cos_theta = std::cos(theta);
    const T sin_theta = std::sin(theta);
    rot_mat << cos_theta, sin_theta, -sin_theta, cos_theta;
  }

  void Reset() {
    origin.setZero();
    rot_mat.setIdentity();
    theta = T(0.0f);
  }

  const Pos<T> GetPos(const Pos<T>& pn) const {
    return rot_mat * (pn - origin);
  }

  const T GetTheta(const T _theta) const {
    return UnifyAngleDiff(_theta, theta);
  }
};

template <typename T>
struct Local2GlobalTrans {
  Pos<T> origin = Pos<T>(0.0f, 0.0f);
  T theta = T(0.0f);
  Eigen::Matrix<T, 2, 2> rot_mat = Eigen::Matrix<T, 2, 2>::Identity();

  Local2GlobalTrans() {}
  ~Local2GlobalTrans() {}

  Local2GlobalTrans(const Pos<T>& _origin, const T _theta) {
    Init(_origin, _theta);
  }

  void Init(const Pos<T>& _origin, const T _theta) {
    origin = _origin;
    theta = _theta;
    const T cos_theta = std::cos(theta);
    const T sin_theta = std::sin(theta);
    rot_mat << cos_theta, -sin_theta, sin_theta, cos_theta;
  }

  void Reset() {
    origin.setZero();
    rot_mat.setIdentity();
    theta = T(0.0f);
  }

  const Pos<T> GetPos(const Pos<T>& pn) const {
    return (rot_mat * pn + origin);
  }

  const T GetTheta(const T _theta) const {
    return UnifyAngleSum(_theta, theta);
  }
};

template <typename T>
struct PathPt {
  PathPt() {}
  ~PathPt() {}

  PathPt(const Pos<T>& _pos, const T _theta, const T _kappa, const T _s,
         const int _type)
      : pos(_pos), theta(_theta), kappa(_kappa), s(_s), type(_type) {}

  PathPt(const Pos<T>& _pos, const T _theta, const Pos<T>& _dir)
      : pos(_pos), theta(_theta), dir(_dir) {}

  PathPt(const Pos<T>& _pos, const T _theta) : pos(_pos), theta(_theta) {}

  Pos<T> pos = Pos<T>(0.0f, 0.0f);
  T theta = T(0.0f);
  T kappa = T(0.0f);
  T s = T(0.0f);
  int type = 0;
  Pos<T> dir = Pos<T>(0.0f, 0.0f);

  void Reset() {
    pos = Pos<T>(0.0f, 0.0f);
    theta = T(0.0f);
    kappa = T(0.0f);
    s = T(0.0f);
    type = 0;
    dir = Pos<T>(0.0f, 0.0f);
  }

  void SetPos(const Pos<T>& _pos) { pos = _pos; }
  void SetTheta(const T _theta) { theta = _theta; }
  void SetX(const T _x) { pos(0) = _x; }
  void SetY(const T _y) { pos(1) = _y; }
  void SetPos(const T _x, const T _y) { pos << _x, _y; }
  void SetDir(const T _theta) {
    dir << T(std::cos(_theta)), T(std::sin(_theta));
  }

  const Pos<T> GetPos() const { return pos; }
  const T GetTheta() const { return theta; }
  const T GetX() const { return pos(0); }
  const T GetY() const { return pos(1); }
  const Pos<T> GetDir() const { return dir; }

  void PrintInfo(const bool enable_log = true) const {
    ILOG_INFO_IF(enable_log)
        << "pos = " << pos.x() << ", " << pos.y()
        << ", theta = " << theta * kRad2DegF << " dir = " << dir.x() << ", "
        << dir.y() << ", kappa = " << kappa << ", s = " << s
        << ", type = " << type;
  }
};

template <typename T>
struct LineSeg {
  LineSeg() {}
  ~LineSeg() {}

  Pos<T> pA = Pos<T>(0.0f, 0.0f);
  Pos<T> pB = Pos<T>(0.0f, 0.0f);
  Pos<T> dir = Pos<T>(0.0f, 0.0f);
  T theta = T(0.0f);
  T length = T(0.0f);

  LineSeg(const Pos<T>& _pA, const Pos<T>& _pB) { SetEndPt(_pA, _pB); }

  LineSeg(const Pos<T>& _pA, const Pos<T>& _pB, const T _theta,
          const Pos<T>& _dir) {
    Set(_pA, _pB, _theta, _dir);
  }

  LineSeg(const Pos<T>& _pA, const Pos<T>& _dir, const T _theta,
          const T _length) {
    Set(_pA, _dir, _theta, _length);
  }

  LineSeg(const Pos<T>& _pA, const T _length, const T _theta) {
    Set(_pA, _length, _theta);
  }

  void Set(const Pos<T>& _pA, const Pos<T>& _dir, const T _theta,
           const T _length) {
    pA = _pA;
    dir = _dir;
    theta = _theta;
    length = _length;
    pB = pA + length * dir;
  }

  void Set(const Pos<T>& _pA, const Pos<T>& _pB, const T _theta,
           const Pos<T>& _dir) {
    SetEndPt(_pA, _pB);
    theta = _theta;
    dir = _dir;
  }

  void Set(const Pos<T>& _pA, const T _length, const T _theta) {
    pA = _pA;
    length = _length;
    theta = _theta;
    dir << T(std::cos(_theta)), T(std::sin(_theta));
    pB = pA + length * dir;
  }

  void SetEndPt(const Pos<T>& _pA, const Pos<T>& _pB) {
    pA = _pA;
    pB = _pB;
    length = (pB - pA).norm();
  }

  void SetDir(const T _theta) {
    dir << T(std::cos(_theta)), T(std::sin(_theta));
  }

  void Reset() {
    pA = Pos<T>(0.0f, 0.0f);
    pB = Pos<T>(0.0f, 0.0f);
    dir = Pos<T>(0.0f, 0.0f);
    theta = T(0.0f);
    length = T(0.0f);
  }

  void PrintInfo(const bool enable_log = true) const {
    ILOG_INFO_IF(enable_log)
        << "pA = " << pA.x() << ", " << pA.y() << ", pB = " << pB.x() << ", "
        << pB.y() << ", dir = " << dir.x() << ", " << dir.y()
        << ", theta = " << theta * kRad2DegF << ", length = " << length;
  }
};

template <typename T>
struct ArcSeg {
  ArcSeg() {}
  ~ArcSeg() {}

  Pos<T> center = Pos<T>(0.0f, 0.0f);
  T radius = T(0.0f);
  T length = T(0.0f);

  Pos<T> pA = Pos<T>(0.0f, 0.0f);
  Pos<T> pB = Pos<T>(0.0f, 0.0f);
  Pos<T> dirA = Pos<T>(0.0f, 0.0f);
  Pos<T> dirB = Pos<T>(0.0f, 0.0f);
  T thetaA = T(0.0f);
  T thetaB = T(0.0f);
  bool is_anticlockwise = false;

  ArcSeg(const Pos<T>& _center, const T _radius) {
    center = _center;
    radius = _radius;
  }

  void Reset() {
    center = Pos<T>(0.0f, 0.0f);
    radius = T(0.0f);
    length = T(0.0f);
    pA = Pos<T>(0.0f, 0.0f);
    pB = Pos<T>(0.0f, 0.0f);
    dirA = Pos<T>(0.0f, 0.0f);
    dirB = Pos<T>(0.0f, 0.0f);
    thetaA = T(0.0f);
    thetaB = T(0.0f);
    is_anticlockwise = false;
  }

  void PrintInfo(const bool enable_log = true) const {
    ILOG_INFO_IF(enable_log)
        << "pA = " << pA.x() << ", " << pA.y() << "  pB = " << pB.x() << ", "
        << pB.y() << "  thetaA = " << thetaA * kRad2DegF
        << "  thetaB = " << thetaB * kRad2DegF << "  dirA = " << dirA.x()
        << ", " << dirA.y() << "  dirB = " << dirB.x() << ", " << dirB.y()
        << "  center = " << center.x() << " " << center.y()
        << "  radius = " << radius << "  length = " << length
        << "  is_anticlockwise = " << is_anticlockwise;
  }
};

template <typename T>
struct PathSeg {
  PathSeg() {}
  ~PathSeg() {}

  LineSeg<T> line_seg;
  ArcSeg<T> arc_seg;

  AstarPathSteer steer;
  AstarPathGear gear;

  T kappa = T(0.0f);

  PathSeg(const ArcSeg<T>& _arc_seg, const AstarPathGear _gear,
          const AstarPathSteer _steer) {
    SetArcSeg(_arc_seg, _gear, _steer);
  }

  PathSeg(const LineSeg<T>& _line_seg, const AstarPathGear _gear) {
    SetLineSeg(_line_seg, _gear);
  }

  void SetLineSeg(const LineSeg<T>& _line_seg, const AstarPathGear _gear) {
    line_seg = _line_seg;
    gear = _gear;
    steer = AstarPathSteer::STRAIGHT;
    kappa = T(0.0f);
  }

  void SetArcSeg(const ArcSeg<T>& _arc_seg, const AstarPathGear _gear,
                 const AstarPathSteer _steer) {
    arc_seg = _arc_seg;
    gear = _gear;
    steer = _steer;
    if (arc_seg.radius > T(0.01f)) {
      if (steer == AstarPathSteer::LEFT) {
        kappa = T(1.0f) / arc_seg.radius;
      } else {
        kappa = T(-1.0f) / arc_seg.radius;
      }
    } else {
      kappa = T(0.0f);
    }
  }

  const T GetLength() const {
    if (steer == AstarPathSteer::STRAIGHT) {
      return line_seg.length;
    } else {
      return arc_seg.length;
    }
  }

  const Pos<T> GetStartPos() const {
    if (steer == AstarPathSteer::STRAIGHT) {
      return line_seg.pA;
    } else {
      return arc_seg.pA;
    }
  }

  const Pos<T> GetEndPos() const {
    if (steer == AstarPathSteer::STRAIGHT) {
      return line_seg.pB;
    } else {
      return arc_seg.pB;
    }
  }

  const T GetStartTheta() const {
    if (steer == AstarPathSteer::STRAIGHT) {
      return line_seg.theta;
    } else {
      return arc_seg.thetaA;
    }
  }

  const T GetEndTheta() const {
    if (steer == AstarPathSteer::STRAIGHT) {
      return line_seg.theta;
    } else {
      return arc_seg.thetaB;
    }
  }

  const Pos<T> GetStartDir() const {
    if (steer == AstarPathSteer::STRAIGHT) {
      return line_seg.dir;
    } else {
      return arc_seg.dirA;
    }
  }

  const Pos<T> GetEndDir() const {
    if (steer == AstarPathSteer::STRAIGHT) {
      return line_seg.dir;
    } else {
      return arc_seg.dirB;
    }
  }

  const PathPt<T> GetStartPose() const {
    return PathPt<T>(GetStartPos(), GetStartTheta(), GetStartDir());
  }

  const PathPt<T> GetEndPose() const {
    return PathPt<T>(GetEndPos(), GetEndTheta(), GetEndDir());
  }

  const Pos<T> GetCenter() const {
    if (steer == AstarPathSteer::STRAIGHT) {
      return Pos<T>(0.0f, 0.0f);
    } else {
      return arc_seg.center;
    }
  }

  const T GetRadius() const {
    if (steer == AstarPathSteer::STRAIGHT) {
      return T(0.0f);
    } else {
      return arc_seg.radius;
    }
  }

  const bool GetIsAnticlockwise() const {
    if (steer == AstarPathSteer::STRAIGHT) {
      return false;
    } else {
      return arc_seg.is_anticlockwise;
    }
  }

  void PrintInfo(const bool enable_log = true) const {
    ILOG_INFO_IF(enable_log) << "gear = " << PathGearDebugString(gear)
                             << "  steer = " << GetPathSteerDebugString(steer)
                             << "  kappa = " << kappa;
    if (steer == AstarPathSteer::STRAIGHT) {
      line_seg.PrintInfo(enable_log);
    } else {
      arc_seg.PrintInfo(enable_log);
    }
  }
};

template <typename T>
const Pos<T> CalDirFromTheta(const T theta) {
  return Pos<T>(std::cos(theta), std::sin(theta));
}

template <typename T>
const T CalCrossFromTwoVec(const Pos<T>& vec0, const Pos<T>& vec1) {
  return vec0.x() * vec1.y() - vec0.y() * vec1.x();
}

template <typename T>
inline const bool IsTwoNumerEqual(const T a, const T b,
                                  const T err = T(1e-3f)) {
  if (std::fabs(a - b) < err) {
    return true;
  }
  return false;
}

template <typename T>
const AstarPathGear CalArcSegGear(const ArcSeg<T>& arc) {
  const Pos<T> v_ab = arc.pB - arc.pA;
  return (v_ab.dot(arc.dirA) > T(0.0f)) ? AstarPathGear::DRIVE
                                        : AstarPathGear::REVERSE;
}

template <typename T>
const AstarPathSteer CalArcSegSteer(const ArcSeg<T>& arc) {
  const Pos<T> v_oa = arc.pA - arc.center;
  return (CalCrossFromTwoVec(v_oa, arc.dirA) > T(0.0f)) ? AstarPathSteer::LEFT
                                                        : AstarPathSteer::RIGHT;
}

template <typename T>
const AstarPathGear CalLineSegGear(const LineSeg<T>& line) {
  const Pos<T> v1 = (line.pB - line.pA).normalized();
  const Pos<T> v2 = line.dir;
  const T cos_theta = v1.dot(v2);
  if (cos_theta > T(0.085f)) {
    return AstarPathGear::DRIVE;
  }
  return AstarPathGear::REVERSE;
}

template <typename T>
inline const T FastSignedAngle(const Pos<T>& a, const Pos<T>& b) {
  const T a_cross_b = a.x() * b.y() - a.y() * b.x();  // 叉积决定方向
  const T a_dot_b = a.x() * b.x() + a.y() * b.y();    // 点积决定模长

  return std::atan2(a_cross_b, a_dot_b);  // 单次三角函数计算
}

template <typename T>
const T NormSquareOfVector(const Pos<T>& pt) {
  return pt.x() * pt.x() + pt.y() * pt.y();
}

template <typename T>
const T CalPt2LineDistSquare(const Pos<T>& pt, const LineSeg<T>& line);

template <typename T>
const T CalPt2LineDist(const Pos<T>& pt, const LineSeg<T>& line);

template <typename T>
const bool CalLineSegByGearAndPose(const AstarPathGear gear, const T length,
                                   const PathPt<T>& pose, PathSeg<T>& line_seg);

template <typename T>
const bool CalLineUnitNormVecByPt(const Pos<T>& pt, const LineSeg<T>& line,
                                  Pos<T>& line_norm_vec);

template <typename T>
const bool SamplePathSeg(std::vector<PathPt<T>>& pts, const PathSeg<T>& seg,
                         const T ds, const T kappa);

template <typename T>
const bool CompleteArcSeg(ArcSeg<T>& arc);

template <typename T>
const bool CompleteArcSeg(ArcSeg<T>& arc, AstarPathSteer steer);

template <typename T>
const uint8_t CalIntersectionOfLineAndCircle(const LineSeg<T>& line,
                                             const ArcSeg<T>& circle,
                                             std::vector<Pos<T>>& pt_vec);

template <typename T>
const bool CalIntersectionOfTwoLines(Pos<T>& intersection,
                                     const LineSeg<T>& line1,
                                     const LineSeg<T>& line2);

template <typename T>
const bool CalOneArcWithTargetThetaAndGear(ArcSeg<T>& arc,
                                           const AstarPathGear gear,
                                           const T target_theta);

template <typename T>
const bool CalOneArcWithLineAndGear(ArcSeg<T>& arc, const LineSeg<T>& line,
                                    const AstarPathGear gear,
                                    const T min_radius);

template <typename T>
const bool CalTwoArcWithLine(
    const PathPt<T>& pose, const LineSeg<T>& line, const T radius1,
    const T radius2,
    std::vector<std::pair<ArcSeg<T>, ArcSeg<T>>>& arc_pair_vec);

template <typename T>
const bool CalCommonTangentCircleOfTwoLine(
    const LineSeg<T>& line1, const LineSeg<T>& line2, const T radius,
    std::vector<Pos<T>>& centers,
    std::vector<std::pair<Pos<T>, Pos<T>>>& tangent_ptss);

template <typename T>
const bool CalTwoArcWithSameThetaAndGear(ArcSeg<T>& arc1, ArcSeg<T>& arc2,
                                         const AstarPathGear gear);

}  // namespace common_math
}  // namespace planning
