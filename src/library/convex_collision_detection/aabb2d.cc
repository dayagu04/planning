#include "aabb2d.h"

#include "gjk2d.h"
#include "pose2d.h"

namespace cdl {
AABB::AABB()
    : min_(Vector2r(std::numeric_limits<double>::max(),
                    std::numeric_limits<double>::max())),
      max_(Vector2r(std::numeric_limits<double>::lowest(),
                    std::numeric_limits<double>::lowest())) {}

AABB::AABB(const Vector2r &center) : min_(center), max_(center) {}

AABB::AABB(const Vector2r &a, const Vector2r &b)
    : min_(a.cwiseMin(b)), max_(a.cwiseMax(b)) {}

AABB::AABB(const AABB &core, const Vector2r &delta)
    : min_(core.min_ - delta), max_(core.max_ + delta) {}

AABB::AABB(const Vector2r &a, const Vector2r &b, const Vector2r &c)
    : min_(a.cwiseMin(b).cwiseMin(c)), max_(a.cwiseMax(b).cwiseMax(c)) {}

bool AABB::overlap(const AABB &other) const {
  if ((min_.array() > other.max_.array()).any()) return false;
  if ((max_.array() < other.min_.array()).any()) return false;
  return true;
}

void AABB::Reset(const Vector2r &center) {
  min_ = center;
  max_ = center;

  return;
}

void AABB::Set(const Vector2r &center, const real length, const real width,
               const real heading) {
  Vector2r unit_l(std::cos(heading), std::sin(heading));
  Vector2r unit_w(-unit_l.y(), unit_l.x());
  Vector2r p1 = center + unit_l * length * 0.5 + unit_w * width * 0.5;
  Vector2r p2 = center - unit_l * length * 0.5 + unit_w * width * 0.5;
  Vector2r p3 = center - unit_l * length * 0.5 - unit_w * width * 0.5;
  Vector2r p4 = center + unit_l * length * 0.5 - unit_w * width * 0.5;
  min_ = p1.cwiseMin(p2).cwiseMin(p3).cwiseMin(p4);
  max_ = p1.cwiseMax(p2).cwiseMax(p3).cwiseMax(p4);
  return;
}

bool AABB::contain(const AABB &other) const {
  if ((min_.array() > other.min_.array()).any()) return false;
  if ((max_.array() < other.max_.array()).any()) return false;
  return true;
}

bool AABB::axisOverlap(const AABB &other, int axis_id) const {
  if (min_[axis_id] > other.max_[axis_id]) return false;
  if (max_[axis_id] < other.min_[axis_id]) return false;
  return true;
}

bool AABB::overlap(const AABB &other, AABB &overlap_part) const {
  if (!overlap(other)) {
    return false;
  }
  overlap_part.min_ = min_.cwiseMax(other.min_);
  overlap_part.max_ = max_.cwiseMin(other.max_);
  return true;
}

bool AABB::contain(const Vector2r &p) const {
  if ((min_.array() > p.array()).any()) return false;
  if ((max_.array() < p.array()).any()) return false;
  return true;
}

bool AABB::contain(const planning::Position2D &p) const {
  if (min_[0] > p.x || min_[1] > p.y) {
    return false;
  }

  if (max_[0] < p.x || max_[1] < p.y) {
    return false;
  }

  return true;
}

bool AABB::contain(const planning::Pose2D &p) const {
  if (min_[0] > p.x || min_[1] > p.y) {
    return false;
  }

  if (max_[0] < p.x || max_[1] < p.y) {
    return false;
  }

  return true;
}

bool AABB::contain(const planning::Pose2f &p) const {
  if (min_[0] > real(p.x) || min_[1] > real(p.y)) {
    return false;
  }

  if (max_[0] < real(p.x) || max_[1] < real(p.y)) {
    return false;
  }

  return true;
}

AABB &AABB::operator+=(const Vector2r &p) {
  min_ = min_.cwiseMin(p);
  max_ = max_.cwiseMax(p);
  return *this;
}

AABB &AABB::operator+=(const AABB &other) {
  min_ = min_.cwiseMin(other.min_);
  max_ = max_.cwiseMax(other.max_);
  return *this;
}

AABB AABB::operator+(const AABB &other) const {
  AABB res(*this);
  return res += other;
}

real AABB::width() const { return max_[0] - min_[0]; }

real AABB::height() const { return max_[1] - min_[1]; }

real AABB::area() const { return width() * height(); }

real AABB::perimeter() const { return 2.0 * (width() + height()); }

real AABB::size() const { return (max_ - min_).squaredNorm(); }

real AABB::radius() const { return (max_ - min_).norm() * 0.5; }

Vector2r AABB::center() const { return (min_ + max_) * 0.5; }

real AABB::long_side() const { return std::max(width(), height()); }

real AABB::short_side() const { return std::min(width(), height()); }

real AABB::distance(const AABB &other, Vector2r &P, Vector2r &Q) const {
  real result = 0;
  for (std::size_t i = 0; i < 2; ++i) {
    const real &amin = min_[i];
    const real &amax = max_[i];
    const real &bmin = other.min_[i];
    const real &bmax = other.max_[i];

    if (amin > bmax) {
      real delta = bmax - amin;
      result += delta * delta;
      P[i] = amin;
      Q[i] = bmax;
    } else if (bmin > amax) {
      real delta = amax - bmin;
      result += delta * delta;
      P[i] = amax;
      Q[i] = bmin;
    } else {
      if (bmin >= amin) {
        real t = 0.5 * (amax + bmin);
        P[i] = t;
        Q[i] = t;
      } else {
        real t = 0.5 * (amin + bmax);
        P[i] = t;
        Q[i] = t;
      }
    }
  }

  return std::sqrt(result);
}

real AABB::distance(const AABB &other) const {
  real result = 0;
  for (std::size_t i = 0; i < 2; ++i) {
    const real &amin = min_[i];
    const real &amax = max_[i];
    const real &bmin = other.min_[i];
    const real &bmax = other.max_[i];

    if (amin > bmax) {
      real delta = bmax - amin;
      result += delta * delta;
    } else if (bmin > amax) {
      real delta = amax - bmin;
      result += delta * delta;
    }
  }

  return std::sqrt(result);
}

bool AABB::equal(const AABB &other) const {
  return min_.isApprox(other.min_,
                       std::numeric_limits<real>::epsilon() * 100) &&
         max_.isApprox(other.max_, std::numeric_limits<real>::epsilon() * 100);
}

AABB &AABB::expand(const Vector2r &delta) {
  min_ -= delta;
  max_ += delta;
  return *this;
}

void AABB::ExtendX(const double x) {
  min_[0] -= x;
  max_[0] += x;
  return;
}

void AABB::ExtendXlower(const double x) {
  min_[0] -= x;
  return;
}

void AABB::ExtendXupper(const double x) {
  max_[0] += x;
  return;
}

void AABB::ExtendY(const double y) {
  min_[1] -= y;
  max_[1] += y;

  return;
}

void AABB::ExtendYlower(const double y) {
  min_[1] -= y;

  return;
}

void AABB::ExtendYupper(const double y) {
  max_[1] += y;

  return;
}

AABB &AABB::expand(const AABB &core, real ratio) {
  min_ = min_ * ratio - core.min_;
  max_ = max_ * ratio - core.max_;
  return *this;
}

AABB AABB::translate(const AABB &aabb, const Vector2r &t) {
  AABB res(aabb);
  res.min_ += t;
  res.max_ += t;
  return res;
}

AABB2f::AABB2f()
    : min_(Eigen::Vector2f(std::numeric_limits<float>::max(),
                           std::numeric_limits<float>::max())),
      max_(Eigen::Vector2f(std::numeric_limits<float>::lowest(),
                           std::numeric_limits<float>::lowest())) {}

AABB2f::AABB2f(const Eigen::Vector2f &a, const Eigen::Vector2f &b)
    : min_(a.cwiseMin(b)), max_(a.cwiseMax(b)) {}

AABB2f::AABB2f(const std::vector<Eigen::Vector2f> &pt_vec) {
  if (pt_vec.empty()) {
    min_ = Eigen::Vector2f(std::numeric_limits<float>::max(),
                           std::numeric_limits<float>::max());
    max_ = Eigen::Vector2f(std::numeric_limits<float>::lowest(),
                           std::numeric_limits<float>::lowest());
  } else {
    min_ = pt_vec[0];
    max_ = pt_vec[0];
    for (std::size_t i = 1; i < pt_vec.size(); ++i) {
      min_ = min_.cwiseMin(pt_vec[i]);
      max_ = max_.cwiseMax(pt_vec[i]);
    }
  }
}

bool AABB2f::contain(const planning::Pose2f &p) const {
  if (min_[0] > p.x || min_[1] > p.y) {
    return false;
  }

  if (max_[0] < p.x || max_[1] < p.y) {
    return false;
  }

  return true;
}

bool AABB2f::IsContain(const planning::Pose2f &p) const {
  if (min_[0] > p.x || min_[1] > p.y) {
    return false;
  }

  if (max_[0] < p.x || max_[1] < p.y) {
    return false;
  }

  return true;
}

bool AABB2f::IsContain(const Eigen::Vector2f &p) const {
  if ((min_.array() > p.array()).any()) return false;
  if ((max_.array() < p.array()).any()) return false;
  return true;
}

bool AABB2f::IsContain(const Eigen::Vector2d &p) const {
  if ((min_.array() > Eigen::Vector2f(p.x(), p.y()).array()).any())
    return false;
  if ((max_.array() < Eigen::Vector2f(p.x(), p.y()).array()).any())
    return false;
  return true;
}

bool AABB2f::IsContain(const AABB2f &other) const {
  if ((min_.array() > other.min_.array()).any()) return false;
  if ((max_.array() < other.max_.array()).any()) return false;
  return true;
}

/** Width of the AABB */
float AABB2f::GetWidth() const { return max_[0] - min_[0]; }

/** Height of the AABB */
float AABB2f::GetLength() const { return max_[1] - min_[1]; }
}  // namespace cdl
