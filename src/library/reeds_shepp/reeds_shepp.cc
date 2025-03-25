
#include "reeds_shepp.h"
#include <cmath>

#include "log_glog.h"
#include "math_utils.h"
#include "pose2d.h"

namespace planning {

#define DEBUG_RS_GENERATE (0)
#define DEBUG_RS_SCS (0)

static RSPathInfo cached_path_info;
static bool global_data_inited = false;
static const RSPathSteer ReedsSheppPathype[][5] = {
    // 0
    // C|C|C
    // CC|C
    // C|CC

    {RS_LEFT, RS_RIGHT, RS_LEFT, RS_NOP, RS_NOP},
    {RS_RIGHT, RS_LEFT, RS_RIGHT, RS_NOP, RS_NOP},

    // 2
    // CC|CC
    // C|CC|C
    {RS_LEFT, RS_RIGHT, RS_LEFT, RS_RIGHT, RS_NOP},
    {RS_RIGHT, RS_LEFT, RS_RIGHT, RS_LEFT, RS_NOP},

    // 4
    // C|CSC
    {RS_LEFT, RS_RIGHT, RS_STRAIGHT, RS_LEFT, RS_NOP},
    {RS_RIGHT, RS_LEFT, RS_STRAIGHT, RS_RIGHT, RS_NOP},

    // 6
    // CSC|C
    {RS_LEFT, RS_STRAIGHT, RS_RIGHT, RS_LEFT, RS_NOP},
    {RS_RIGHT, RS_STRAIGHT, RS_LEFT, RS_RIGHT, RS_NOP},

    // 8
    // C|CSC
    {RS_LEFT, RS_RIGHT, RS_STRAIGHT, RS_RIGHT, RS_NOP},
    {RS_RIGHT, RS_LEFT, RS_STRAIGHT, RS_LEFT, RS_NOP},

    // 10
    // CSC|C
    {RS_RIGHT, RS_STRAIGHT, RS_RIGHT, RS_LEFT, RS_NOP},
    {RS_LEFT, RS_STRAIGHT, RS_LEFT, RS_RIGHT, RS_NOP},

    // 12
    // CSC
    {RS_LEFT, RS_STRAIGHT, RS_RIGHT, RS_NOP, RS_NOP},
    {RS_RIGHT, RS_STRAIGHT, RS_LEFT, RS_NOP, RS_NOP},
    {RS_LEFT, RS_STRAIGHT, RS_LEFT, RS_NOP, RS_NOP},
    {RS_RIGHT, RS_STRAIGHT, RS_RIGHT, RS_NOP, RS_NOP},

    // 16
    // C|CSC|C
    {RS_LEFT, RS_RIGHT, RS_STRAIGHT, RS_LEFT, RS_RIGHT},
    {RS_RIGHT, RS_LEFT, RS_STRAIGHT, RS_RIGHT, RS_LEFT},

    // 18
    // SCS
    {RS_STRAIGHT, RS_LEFT, RS_STRAIGHT, RS_NOP, RS_NOP},
    {RS_STRAIGHT, RS_RIGHT, RS_STRAIGHT, RS_NOP, RS_NOP}

};

RSPathRequestType re_request_type_;

static int CartesianToPolar(float *r, float *theta, float x, float y) {
  *r = std::sqrt(x * x + y * y);
  *theta = std::atan2(y, x);

  return 0;
}

static int SetRSPathParam(RSPathParam *path, const RSPathSteer *type, float t,
                          float u, float v, float w, float x) {
  path->type = type;
  path->t = t;
  path->u = u;
  path->v = v;
  path->w = w;
  path->x = x;

  return 0;
}

static int calculate_tauOmega(float *tau, float *omega, float u, float v,
                              float xi, float eta, float phi) {
  float delta, A, B, t1, t2;
  float cos_theta, cos_u;

  delta = IflyUnifyTheta(u - v, M_PI);
  cos_theta = ifly_cos(delta);
  cos_u = ifly_cos(u);

  A = ifly_sin(u) - ifly_sin(delta);
  B = cos_u - cos_theta - 1;
  t1 = std::atan2(eta * A - xi * B, xi * A + eta * B);
  t2 = 2 * (cos_theta - ifly_cos(v) - cos_u) + 3;
  *tau = (ifly_fless(t2, 0)) ? IflyUnifyTheta(t1 + M_PI, M_PI)
                             : IflyUnifyTheta(t1, M_PI);
  *omega = IflyUnifyTheta(*tau - u + v - phi, M_PI);

  return 0;
}

/* formula 8.1 */
// CSC
static bool LpSpLp(float *t, float *u, float *v, float x, float y,
                   float phi, float sin_phi, float cos_phi) {
  float t1;

  CartesianToPolar(u, &t1, x - sin_phi, y - 1 + cos_phi);

  if (ifly_fless(t1, 0)) {
    return false;
  } else {
    *t = t1;
    *v = IflyUnifyTheta(phi - t1, M_PI);
    if (ifly_fless(*v, 0)) {
      return false;
    }
  }

  return true;
}

/* formula 8.2 */
// CSC
static bool LpSpRp(float *t, float *u, float *v, float x, float y,
                   float phi, float sin_phi, float cos_phi) {
  float t1, u1, theta;

  CartesianToPolar(&u1, &t1, x + sin_phi, y - 1 - cos_phi);

  u1 = u1 * u1;
  if (ifly_fless(u1, 4)) {
    return false;
  } else {
    *u = ifly_sqrt(u1 - 4);
    theta = std::atan2(2, *u);
    t1 = IflyUnifyTheta(t1 + theta, M_PI);
    *v = IflyUnifyTheta(t1 - phi, M_PI);
    *t = t1;
    if (ifly_fless(*t, 0) || ifly_fless(*v, 0)) {
      return false;
    }
  }

  return true;
}

bool SLS(float *t, float *u, float *v, const float x, const float y,
         const float phi) {
  float phi_mod = IflyUnifyTheta(phi, M_PI);
  float xd = 0.0;
  float epsilon = 1e-1;

  // upper
  if (y > 0.0 && phi_mod > epsilon && phi_mod < M_PI) {
    xd = x - y / std::tan(phi_mod);

    float tan_phi_2 = std::tan(phi_mod / 2.0);
    *t = xd - tan_phi_2;
    *u = phi_mod;
    *v = std::sqrt((x - xd) * (x - xd) + y * y) - tan_phi_2;

    return true;

  } else if (y < 0.0 && phi_mod > epsilon && phi_mod < M_PI) {
    // lower
    xd = x - y / std::tan(phi_mod);

    float tan_phi_2 = std::tan(phi_mod / 2.0);
    *t = xd - tan_phi_2;
    *u = phi_mod;
    *v = -std::sqrt((x - xd) * (x - xd) + y * y) - tan_phi_2;

    return true;
  }

  return false;
}

// todo: has some bugs for steer. fix it.
int SCS(RSPathParam *path, float *Lmin, const RSEndPoint *end) {
  bool pass;
  float t = 0.0;
  float u = 0.0;
  float v = 0.0;
  float L = 0.0;

  // heading is (0, pi),
  pass = SLS(&t, &u, &v, end->pose.x, end->pose.y, end->pose.theta);
  L = std::fabs(t) + std::fabs(u) + std::fabs(v);

  if (pass && ifly_fgreater(*Lmin, L)) {
    SetRSPathParam(path, ReedsSheppPathype[18], t, u, v, 0, 0);

    *Lmin = L;
  }

  // ILOG_INFO << "xy,theta " << end->pose.x << " , "
  //           << " ," << end->pose.y << " , " << end->pose.theta;
  // ILOG_INFO << " pass " << pass << " l " << L << ", t " << t << " , u " << u
  //           << " , v " << v;

  // heading is (-pi, 0), reflect transform.
  pass = SLS(&t, &u, &v, end->pose.x, -end->pose.y, -end->pose.theta);
  L = std::fabs(t) + std::fabs(u) + std::fabs(v);

  if (pass && ifly_fgreater(*Lmin, L)) {
    SetRSPathParam(path, ReedsSheppPathype[19], t, u, v, 0, 0);

    *Lmin = L;
  }

  // ILOG_INFO << "xy,theta " << end->pose.x << " , "
  //           << " ," << -end->pose.y << " ," << -end->pose.theta;
  // ILOG_INFO << " pass " << pass << " l " << L << ", t " << t << " , u " << u
  //           << " , v " << v;

  return 0;
}

static int CSC(RSPathParam *path, float *Lmin, RSEndPoint *end) {
  bool pass;
  float t, u, v, L;

  // 1
  pass = LpSpLp(&t, &u, &v, end->pose.x, end->pose.y, end->pose.theta,
                end->sin_theta, end->cos_theta);
  L = std::fabs(t) + std::fabs(u) + std::fabs(v);

  if (pass && ifly_fgreater(*Lmin, L)) {
    SetRSPathParam(path, ReedsSheppPathype[14], t, u, v, 0, 0);

    *Lmin = L;
  }

  // 2
  pass = LpSpLp(&t, &u, &v, -end->pose.x, end->pose.y, -end->pose.theta,
                end->sin_minus_theta, end->cos_minus_theta);
  L = std::fabs(t) + std::fabs(u) + std::fabs(v);
  if (pass && ifly_fgreater(*Lmin, L)) {
    SetRSPathParam(path, ReedsSheppPathype[14], -t, -u, -v, 0, 0);

    *Lmin = L;
  }

  // 3
  pass = LpSpLp(&t, &u, &v, end->pose.x, -end->pose.y, -end->pose.theta,
                end->sin_minus_theta, end->cos_minus_theta);
  L = std::fabs(t) + std::fabs(u) + std::fabs(v);
  if (pass && ifly_fgreater(*Lmin, L)) {
    SetRSPathParam(path, ReedsSheppPathype[15], t, u, v, 0, 0);

    *Lmin = L;
  }

  // 4
  pass = LpSpLp(&t, &u, &v, -end->pose.x, -end->pose.y, end->pose.theta,
                end->sin_theta, end->cos_theta);
  L = std::fabs(t) + std::fabs(u) + std::fabs(v);
  if (pass && ifly_fgreater(*Lmin, L)) {
    SetRSPathParam(path, ReedsSheppPathype[15], -t, -u, -v, 0, 0);

    *Lmin = L;
  }

  // 5
  pass = LpSpRp(&t, &u, &v, end->pose.x, end->pose.y, end->pose.theta,
                end->sin_theta, end->cos_theta);
  L = std::fabs(t) + std::fabs(u) + std::fabs(v);
  if (pass && ifly_fgreater(*Lmin, L)) {
    SetRSPathParam(path, ReedsSheppPathype[12], t, u, v, 0, 0);

    *Lmin = L;
  }

  // 6
  pass = LpSpRp(&t, &u, &v, -end->pose.x, end->pose.y, -end->pose.theta,
                end->sin_minus_theta, end->cos_minus_theta);
  L = std::fabs(t) + std::fabs(u) + std::fabs(v);
  if (pass && ifly_fgreater(*Lmin, L)) {
    SetRSPathParam(path, ReedsSheppPathype[12], -t, -u, -v, 0, 0);

    *Lmin = L;
  }

  // 7
  pass = LpSpRp(&t, &u, &v, end->pose.x, -end->pose.y, -end->pose.theta,
                end->sin_minus_theta, end->cos_minus_theta);
  L = std::fabs(t) + std::fabs(u) + std::fabs(v);
  if (pass && ifly_fgreater(*Lmin, L)) {
    SetRSPathParam(path, ReedsSheppPathype[13], t, u, v, 0, 0);

    *Lmin = L;
  }

  // 8
  pass = LpSpRp(&t, &u, &v, -end->pose.x, -end->pose.y, end->pose.theta,
                end->sin_theta, end->cos_theta);
  L = std::fabs(t) + std::fabs(u) + std::fabs(v);
  if (pass && ifly_fgreater(*Lmin, L)) {
    SetRSPathParam(path, ReedsSheppPathype[13], -t, -u, -v, 0, 0);

    *Lmin = L;
  }

  return 0;
}

/* formula 8.3, 8.4 */
static bool LpRmL(float *t, float *u, float *v, float x, float y,
                  float phi, float sin_phi, float cos_phi) {
  float xi, eta, t1, u1, theta;

  xi = x - sin_phi, eta = y - 1 + cos_phi;
  CartesianToPolar(&u1, &theta, xi, eta);

  if (ifly_fgreater(u1, 4)) {
    return false;
  } else {
    u1 = -2 * std::asin(0.25 * u1);
    t1 = IflyUnifyTheta(theta + 0.5 * u1 + M_PI, M_PI);
    *v = IflyUnifyTheta(phi - t1 + u1, M_PI);
    *u = u1;
    *t = t1;
    if (ifly_fless(*t, 0) || ifly_fgreater(*u, 0)) {
      return false;
    }
  }

  return true;
}

static int CCC(RSPathParam *path, float *Lmin, RSEndPoint *point) {
  bool pass;
  float t, u, v, L;
  float xb, yb;

  // type 2
  pass = LpRmL(&t, &u, &v, point->pose.x, point->pose.y, point->pose.theta,
               point->sin_theta, point->cos_theta);
  L = std::fabs(t) + std::fabs(u) + std::fabs(v);
  if (pass && ifly_fgreater(*Lmin, L)) {
    SetRSPathParam(path, ReedsSheppPathype[0], t, u, v, 0, 0);

    *Lmin = L;
  }

  pass = LpRmL(&t, &u, &v, -point->pose.x, point->pose.y, -point->pose.theta,
               point->sin_minus_theta, point->cos_minus_theta);
  L = std::fabs(t) + std::fabs(u) + std::fabs(v);
  if (pass && ifly_fgreater(*Lmin, L)) {
    SetRSPathParam(path, ReedsSheppPathype[0], -t, -u, -v, 0, 0);

    *Lmin = L;
  }

  pass = LpRmL(&t, &u, &v, point->pose.x, -point->pose.y, -point->pose.theta,
               point->sin_minus_theta, point->cos_minus_theta);
  L = std::fabs(t) + std::fabs(u) + std::fabs(v);
  if (pass && ifly_fgreater(*Lmin, L)) {
    SetRSPathParam(path, ReedsSheppPathype[1], t, u, v, 0, 0);

    *Lmin = L;
  }

  pass = LpRmL(&t, &u, &v, -point->pose.x, -point->pose.y, point->pose.theta,
               point->sin_theta, point->cos_theta);
  L = std::fabs(t) + std::fabs(u) + std::fabs(v);
  if (pass && ifly_fgreater(*Lmin, L)) {
    SetRSPathParam(path, ReedsSheppPathype[1], -t, -u, -v, 0, 0);

    *Lmin = L;
  }

  /* backwards */
  // type 1
  xb = point->pose.x * point->cos_theta + point->pose.y * point->sin_theta;
  yb = point->pose.x * point->sin_theta - point->pose.y * point->cos_theta;

  pass = LpRmL(&t, &u, &v, xb, yb, point->pose.theta, point->sin_theta,
               point->cos_theta);
  L = std::fabs(t) + std::fabs(u) + std::fabs(v);
  if (pass && ifly_fgreater(*Lmin, L)) {
    SetRSPathParam(path, ReedsSheppPathype[0], v, u, t, 0, 0);

    *Lmin = L;
  }

  pass = LpRmL(&t, &u, &v, -xb, yb, -point->pose.theta, point->sin_minus_theta,
               point->cos_minus_theta);
  L = std::fabs(t) + std::fabs(u) + std::fabs(v);
  if (pass && ifly_fgreater(*Lmin, L)) {
    SetRSPathParam(path, ReedsSheppPathype[0], -v, -u, -t, 0, 0);

    *Lmin = L;
  }

  pass = LpRmL(&t, &u, &v, xb, -yb, -point->pose.theta, point->sin_minus_theta,
               point->cos_minus_theta);
  L = std::fabs(t) + std::fabs(u) + std::fabs(v);
  if (pass && ifly_fgreater(*Lmin, L)) {
    SetRSPathParam(path, ReedsSheppPathype[1], v, u, t, 0, 0);

    *Lmin = L;
  }

  pass = LpRmL(&t, &u, &v, -xb, -yb, point->pose.theta, point->sin_theta,
               point->cos_theta);
  L = std::fabs(t) + std::fabs(u) + std::fabs(v);
  if (pass && ifly_fgreater(*Lmin, L)) {
    SetRSPathParam(path, ReedsSheppPathype[1], -v, -u, -t, 0, 0);

    *Lmin = L;
  }

  return 0;
}

/* formula 8.7 */
// CC|CC
static bool LpRupLumRm(float *t, float *u, float *v, float x, float y,
                       float phi, float sin_phi, float cos_phi) {
  float xi, eta, rho, u1;

  xi = x + sin_phi;
  eta = y - 1 - cos_phi;
  rho = 0.25 * (2 + ifly_sqrt(xi * xi + eta * eta));
  if (ifly_fgreater(rho, 1)) {
    return false;
  } else {
    u1 = std::acos(rho);
    calculate_tauOmega(t, v, u1, -u1, xi, eta, phi);

    *u = u1;
    if (ifly_fless(*t, 0) || ifly_fgreater(*v, 0)) {
      return false;
    }
  }

  return true;
}

/* formula 8.8 */
static bool LpRumLumRp(float *t, float *u, float *v, float x, float y,
                       float phi, float sin_phi, float cos_phi) {
  float xi, eta, rho;

  xi = x + sin_phi;
  eta = y - 1 - cos_phi;
  rho = (20 - xi * xi - eta * eta) / 16;
  if (ifly_fless(rho, 0) || !ifly_fgreater(rho, 1)) {
    return false;
  } else {
    *u = -std::acos(rho);
    if (!ifly_fless(*u, -0.5 * M_PI)) {
      calculate_tauOmega(t, v, *u, *u, xi, eta, phi);

      if (ifly_fless(*t, 0) || ifly_fless(*v, 0)) {
        return false;
      }
    }
  }

  return true;
}

static int CCCC(RSPathParam *path, float *Lmin, RSEndPoint *point) {
  bool pass;
  float t, u, v, L;

  pass = LpRupLumRm(&t, &u, &v, point->pose.x, point->pose.y, point->pose.theta,
                    point->sin_theta, point->cos_theta);
  L = std::fabs(t) + 2 * std::fabs(u) + std::fabs(v);
  if (pass && ifly_fgreater(*Lmin, L)) {
    SetRSPathParam(path, ReedsSheppPathype[2], t, u, -u, v, 0);

    *Lmin = L;
  }

  pass =
      LpRupLumRm(&t, &u, &v, -point->pose.x, point->pose.y, -point->pose.theta,
                 point->sin_minus_theta, point->cos_minus_theta);
  L = std::fabs(t) + 2 * std::fabs(u) + std::fabs(v);
  if (pass && ifly_fgreater(*Lmin, L)) {
    SetRSPathParam(path, ReedsSheppPathype[2], -t, -u, u, -v, 0);

    *Lmin = L;
  }

  pass =
      LpRupLumRm(&t, &u, &v, point->pose.x, -point->pose.y, -point->pose.theta,
                 point->sin_minus_theta, point->cos_minus_theta);
  L = std::fabs(t) + 2 * std::fabs(u) + std::fabs(v);
  if (pass && ifly_fgreater(*Lmin, L)) {
    SetRSPathParam(path, ReedsSheppPathype[3], t, u, -u, v, 0);

    *Lmin = L;
  }

  pass = LpRupLumRm(&t, &u, &v, -point->pose.x, -point->pose.y,
                    point->pose.theta, point->sin_theta, point->cos_theta);
  L = std::fabs(t) + 2 * std::fabs(u) + std::fabs(v);
  if (pass && ifly_fgreater(*Lmin, L)) {
    SetRSPathParam(path, ReedsSheppPathype[3], -t, -u, u, -v, 0);

    *Lmin = L;
  }

  if (re_request_type_ == RSPathRequestType::GEAR_SWITCH_LESS_THAN_TWICE) {
    return 0;
  }
  // C|C_beta C_beta|C
  pass = LpRumLumRp(&t, &u, &v, point->pose.x, point->pose.y, point->pose.theta,
                    point->sin_theta, point->cos_theta);
  L = std::fabs(t) + 2 * std::fabs(u) + std::fabs(v);
  if (pass && ifly_fgreater(*Lmin, L)) {
    SetRSPathParam(path, ReedsSheppPathype[2], t, u, u, v, 0);

    *Lmin = L;
  }

  pass =
      LpRumLumRp(&t, &u, &v, -point->pose.x, point->pose.y, -point->pose.theta,
                 point->sin_minus_theta, point->cos_minus_theta);
  L = std::fabs(t) + 2 * std::fabs(u) + std::fabs(v);
  if (pass && ifly_fgreater(*Lmin, L)) {
    SetRSPathParam(path, ReedsSheppPathype[2], -t, -u, -u, -v, 0);

    *Lmin = L;
  }

  pass =
      LpRumLumRp(&t, &u, &v, point->pose.x, -point->pose.y, -point->pose.theta,
                 point->sin_minus_theta, point->cos_minus_theta);
  L = std::fabs(t) + 2 * std::fabs(u) + std::fabs(v);
  if (pass && ifly_fgreater(*Lmin, L)) {
    SetRSPathParam(path, ReedsSheppPathype[3], t, u, u, v, 0);

    *Lmin = L;
  }

  pass = LpRumLumRp(&t, &u, &v, -point->pose.x, -point->pose.y,
                    point->pose.theta, point->sin_theta, point->cos_theta);
  L = std::fabs(t) + 2 * std::fabs(u) + std::fabs(v);
  if (pass && ifly_fgreater(*Lmin, L)) {
    SetRSPathParam(path, ReedsSheppPathype[3], -t, -u, -u, -v, 0);

    *Lmin = L;
  }

  return 0;
}

/* formula 8.9 */
static bool LpRmSmLm(float *t, float *u, float *v, float x, float y,
                     float phi, float sin_phi, float cos_phi) {
  float xi, eta, rho, theta, r, t1;

  xi = x - sin_phi;
  eta = y - 1 + cos_phi;
  CartesianToPolar(&rho, &theta, xi, eta);

  if (ifly_fless(rho, 2)) {
    return false;
  } else {
    r = ifly_sqrt(rho * rho - 4);
    *u = 2 - r;
    t1 = IflyUnifyTheta(theta + std::atan2(r, -2), M_PI);
    *v = IflyUnifyTheta(phi - 0.5 * M_PI - t1, M_PI);
    *t = t1;
    if (ifly_fless(*t, 0) || ifly_fgreater(*u, 0) || ifly_fgreater(*v, 0)) {
      return false;
    }
  }

  return true;
}

/* formula 8.10 */
static bool LpRmSmRm(float *t, float *u, float *v, float x, float y,
                     float phi, float sin_phi, float cos_phi) {
  float xi, eta, rho, theta;

  xi = x + sin_phi;
  eta = y - 1 - cos_phi;
  CartesianToPolar(&rho, &theta, -eta, xi);

  if (ifly_fless(rho, 2.0)) {
    return false;
  } else {
    *t = theta;
    *u = 2.0 - rho;
    *v = IflyUnifyTheta(*t + 0.5 * M_PI - phi, M_PI);
    if (ifly_fless(*t, 0.0) || ifly_fgreater(*u, 0.0) ||
        ifly_fgreater(*v, 0.0)) {
      return false;
    }
  }

  return true;
}

static int CCSC(RSPathParam *path, float *Lmin, RSEndPoint *point) {
  bool pass;
  float t, u, v, L;
  float xb, yb;

  *Lmin = *Lmin - 0.5 * M_PI;
  pass = LpRmSmLm(&t, &u, &v, point->pose.x, point->pose.y, point->pose.theta,
                  point->sin_theta, point->cos_theta);
  L = std::fabs(t) + std::fabs(u) + std::fabs(v);
  if (pass && ifly_fgreater(*Lmin, L)) {
    SetRSPathParam(path, ReedsSheppPathype[4], t, -0.5 * M_PI, u, v, 0);

    *Lmin = L;
  }

  pass = LpRmSmLm(&t, &u, &v, -point->pose.x, point->pose.y, -point->pose.theta,
                  point->sin_minus_theta, point->cos_minus_theta);
  L = std::fabs(t) + std::fabs(u) + std::fabs(v);
  if (pass && ifly_fgreater(*Lmin, L)) {
    SetRSPathParam(path, ReedsSheppPathype[4], -t, 0.5 * M_PI, -u, -v, 0);

    *Lmin = L;
  }

  pass = LpRmSmLm(&t, &u, &v, point->pose.x, -point->pose.y, -point->pose.theta,
                  point->sin_minus_theta, point->cos_minus_theta);
  L = std::fabs(t) + std::fabs(u) + std::fabs(v);
  if (pass && ifly_fgreater(*Lmin, L)) {
    SetRSPathParam(path, ReedsSheppPathype[5], t, -0.5 * M_PI, u, v, 0);

    *Lmin = L;
  }

  pass = LpRmSmLm(&t, &u, &v, -point->pose.x, -point->pose.y, point->pose.theta,
                  point->sin_theta, point->cos_theta);
  L = std::fabs(t) + std::fabs(u) + std::fabs(v);
  if (pass && ifly_fgreater(*Lmin, L)) {
    SetRSPathParam(path, ReedsSheppPathype[5], -t, 0.5 * M_PI, -u, -v, 0);

    *Lmin = L;
  }

  pass = LpRmSmRm(&t, &u, &v, point->pose.x, point->pose.y, point->pose.theta,
                  point->sin_theta, point->cos_theta);
  L = std::fabs(t) + std::fabs(u) + std::fabs(v);
  if (pass && ifly_fgreater(*Lmin, L)) {
    SetRSPathParam(path, ReedsSheppPathype[8], t, -0.5 * M_PI, u, v, 0);

    *Lmin = L;
  }

  pass = LpRmSmRm(&t, &u, &v, -point->pose.x, point->pose.y, -point->pose.theta,
                  point->sin_minus_theta, point->cos_minus_theta);
  L = std::fabs(t) + std::fabs(u) + std::fabs(v);
  if (pass && ifly_fgreater(*Lmin, L)) {
    SetRSPathParam(path, ReedsSheppPathype[8], -t, 0.5 * M_PI, -u, -v, 0);

    *Lmin = L;
  }

  pass = LpRmSmRm(&t, &u, &v, point->pose.x, -point->pose.y, -point->pose.theta,
                  point->sin_minus_theta, point->cos_minus_theta);
  L = std::fabs(t) + std::fabs(u) + std::fabs(v);
  if (pass && ifly_fgreater(*Lmin, L)) {
    SetRSPathParam(path, ReedsSheppPathype[9], t, -0.5 * M_PI, u, v, 0);

    *Lmin = L;
  }

  pass = LpRmSmRm(&t, &u, &v, -point->pose.x, -point->pose.y, point->pose.theta,
                  point->sin_theta, point->cos_theta);
  L = std::fabs(t) + std::fabs(u) + std::fabs(v);
  if (pass && ifly_fgreater(*Lmin, L)) {
    SetRSPathParam(path, ReedsSheppPathype[9], -t, 0.5 * M_PI, -u, -v, 0);

    *Lmin = L;
  }

  /* backwards */
  xb = point->pose.x * point->cos_theta + point->pose.y * point->sin_theta;
  yb = point->pose.x * point->sin_theta - point->pose.y * point->cos_theta;
  pass = LpRmSmLm(&t, &u, &v, xb, yb, point->pose.theta, point->sin_theta,
                  point->cos_theta);
  L = std::fabs(t) + std::fabs(u) + std::fabs(v);
  if (pass && ifly_fgreater(*Lmin, L)) {
    SetRSPathParam(path, ReedsSheppPathype[6], v, u, -0.5 * M_PI, t, 0);

    *Lmin = L;
  }

  pass = LpRmSmLm(&t, &u, &v, -xb, yb, -point->pose.theta,
                  point->sin_minus_theta, point->cos_minus_theta);
  L = std::fabs(t) + std::fabs(u) + std::fabs(v);
  if (pass && ifly_fgreater(*Lmin, L)) {
    SetRSPathParam(path, ReedsSheppPathype[6], -v, -u, 0.5 * M_PI, -t, 0);

    *Lmin = L;
  }

  pass = LpRmSmLm(&t, &u, &v, xb, -yb, -point->pose.theta,
                  point->sin_minus_theta, point->cos_minus_theta);
  L = std::fabs(t) + std::fabs(u) + std::fabs(v);
  if (pass && ifly_fgreater(*Lmin, L)) {
    SetRSPathParam(path, ReedsSheppPathype[7], v, u, -0.5 * M_PI, t, 0);

    *Lmin = L;
  }

  pass = LpRmSmLm(&t, &u, &v, -xb, -yb, point->pose.theta, point->sin_theta,
                  point->cos_theta);
  L = std::fabs(t) + std::fabs(u) + std::fabs(v);
  if (pass && ifly_fgreater(*Lmin, L)) {
    SetRSPathParam(path, ReedsSheppPathype[7], -v, -u, 0.5 * M_PI, -t, 0);

    *Lmin = L;
  }

  pass = LpRmSmRm(&t, &u, &v, xb, yb, point->pose.theta, point->sin_theta,
                  point->cos_theta);
  L = std::fabs(t) + std::fabs(u) + std::fabs(v);
  if (pass && ifly_fgreater(*Lmin, L)) {
    SetRSPathParam(path, ReedsSheppPathype[10], v, u, -0.5 * M_PI, t, 0);

    *Lmin = L;
  }

  pass = LpRmSmRm(&t, &u, &v, -xb, yb, -point->pose.theta,
                  point->sin_minus_theta, point->cos_minus_theta);
  L = std::fabs(t) + std::fabs(u) + std::fabs(v);
  if (pass && ifly_fgreater(*Lmin, L)) {
    SetRSPathParam(path, ReedsSheppPathype[10], -v, -u, 0.5 * M_PI, -t, 0);

    *Lmin = L;
  }

  pass = LpRmSmRm(&t, &u, &v, xb, -yb, -point->pose.theta,
                  point->sin_minus_theta, point->cos_minus_theta);
  L = std::fabs(t) + std::fabs(u) + std::fabs(v);
  if (pass && ifly_fgreater(*Lmin, L)) {
    SetRSPathParam(path, ReedsSheppPathype[11], v, u, -0.5 * M_PI, t, 0);

    *Lmin = L;
  }

  pass = LpRmSmRm(&t, &u, &v, -xb, -yb, point->pose.theta, point->sin_theta,
                  point->cos_theta);
  L = std::fabs(t) + std::fabs(u) + std::fabs(v);
  if (pass && ifly_fgreater(*Lmin, L)) {
    SetRSPathParam(path, ReedsSheppPathype[11], -v, -u, 0.5 * M_PI, -t, 0);

    *Lmin = L;
  }

  return 0;
}

/* formula 8.11 */
static bool LpRmSLmRp(float *t, float *u, float *v, float x, float y,
                      float phi, float sin_phi, float cos_phi) {
  float xi, eta, rho, theta, u1;

  xi = x + sin_phi;
  eta = y - 1.0 - cos_phi;
  CartesianToPolar(&rho, &theta, xi, eta);

  if (ifly_fless(rho, 2.0)) {
    return false;
  } else {
    u1 = 4.0 - ifly_sqrt(rho * rho - 4.0);
    if (!ifly_fgreater(u1, 0.0)) {
      *t = IflyUnifyTheta(
          std::atan2((4.0 - u1) * xi - 2.0 * eta, -2 * xi + (u1 - 4.0) * eta),
          M_PI);

      *v = IflyUnifyTheta(*t - phi, M_PI);
      *u = u1;

      if (ifly_fless(*t, 0.0) || ifly_fless(*v, 0.0)) {
        return false;
      }
    }
  }

  return true;
}

static int CCSCC(RSPathParam *path, float *Lmin, RSEndPoint *point) {
  bool pass;
  float t, u, v, L;

  if (re_request_type_ == RSPathRequestType::GEAR_SWITCH_LESS_THAN_TWICE) {
    return 0;
  }

  *Lmin = *Lmin - M_PI;
  pass = LpRmSLmRp(&t, &u, &v, point->pose.x, point->pose.y, point->pose.theta,
                   point->sin_theta, point->cos_theta);
  L = std::fabs(t) + std::fabs(u) + std::fabs(v);
  if (pass && ifly_fgreater(*Lmin, L)) {
    SetRSPathParam(path, ReedsSheppPathype[16], t, -0.5 * M_PI, u, -0.5 * M_PI,
                   v);

    *Lmin = L;
  }

  pass =
      LpRmSLmRp(&t, &u, &v, -point->pose.x, point->pose.y, -point->pose.theta,
                point->sin_minus_theta, point->cos_minus_theta);
  L = std::fabs(t) + std::fabs(u) + std::fabs(v);
  if (pass && ifly_fgreater(*Lmin, L)) {
    SetRSPathParam(path, ReedsSheppPathype[16], -t, 0.5 * M_PI, -u, 0.5 * M_PI,
                   -v);

    *Lmin = L;
  }

  pass =
      LpRmSLmRp(&t, &u, &v, point->pose.x, -point->pose.y, -point->pose.theta,
                point->sin_minus_theta, point->cos_minus_theta);
  L = std::fabs(t) + std::fabs(u) + std::fabs(v);
  if (pass && ifly_fgreater(*Lmin, L)) {
    SetRSPathParam(path, ReedsSheppPathype[17], t, -0.5 * M_PI, u, -0.5 * M_PI,
                   v);

    *Lmin = L;
  }

  pass = LpRmSLmRp(&t, &u, &v, -point->pose.x, -point->pose.y,
                   point->pose.theta, point->sin_theta, point->cos_theta);
  L = std::fabs(t) + std::fabs(u) + std::fabs(v);
  if (pass && ifly_fgreater(*Lmin, L)) {
    SetRSPathParam(path, ReedsSheppPathype[17], -t, 0.5 * M_PI, -u, 0.5 * M_PI,
                   -v);

    *Lmin = L;
  }

  return 0;
}

int GetShortestRSPathParam(RSPathParam *path, const Pose2D *start_pose,
                           const Pose2D *goal_pose, float min_turn_radius,
                           float inverse_radius,
                           const RSPathRequestType request_type) {
  float dx, dy, dtheta, cos_theta, sin_theta, x, y, Lmin;

  if (path == nullptr || start_pose == nullptr || goal_pose == nullptr) {
    return 0;
  }

  if (IsPointEqual(start_pose, goal_pose)) {
    SetRSPathParam(path, ReedsSheppPathype[0], 0, 0, 0, 0, 0);
  } else {
    dx = goal_pose->x - start_pose->x;
    dy = goal_pose->y - start_pose->y;
    dtheta = goal_pose->theta - start_pose->theta;

    cos_theta = ifly_cos(start_pose->theta);
    sin_theta = ifly_sin(start_pose->theta);
    x = (cos_theta * dx + sin_theta * dy) * inverse_radius;
    y = (-sin_theta * dx + cos_theta * dy) * inverse_radius;
    Lmin = 100000.0;

    RSEndPoint end;
    end.pose.x = x;
    end.pose.y = y;
    end.pose.theta = dtheta;

    end.sin_theta = ifly_sin(dtheta);
    end.cos_theta = ifly_cos(dtheta);
    end.cos_minus_theta = end.cos_theta;
    end.sin_minus_theta = -end.sin_theta;

    re_request_type_ = request_type;

    // SCS(path, &Lmin, &end);

    // type 3/4
    CSC(path, &Lmin, &end);

    // type 1/2
    CCC(path, &Lmin, &end);

    // type 5/6
    CCCC(path, &Lmin, &end);

    // // type 7/8
    CCSC(path, &Lmin, &end);

    // // type 9
    CCSCC(path, &Lmin, &end);
  }

  path->length[0] = path->t * min_turn_radius;
  path->length[1] = path->u * min_turn_radius;
  path->length[2] = path->v * min_turn_radius;
  path->length[3] = path->w * min_turn_radius;
  path->length[4] = path->x * min_turn_radius;

  path->total_length = std::fabs(path->length[0]) + std::fabs(path->length[1]) +
                       std::fabs(path->length[2]) + std::fabs(path->length[3]) +
                       std::fabs(path->length[4]);

#if 0
    int i;
    for (i = 0; i < MAX_RS_PATH_NUM; i++) {
      printf("path.length[%d], %.3f\n", i, path->length[i]);
    }
#endif

  return 0;
}

static int reeds_shepp_init(void) {
  cached_path_info.start.x = 100000.0;
  cached_path_info.start.y = 100000.0;
  cached_path_info.start.theta = 100000.0;

  cached_path_info.end.x = 100000.0;
  cached_path_info.end.y = 100000.0;
  cached_path_info.end.theta = 100000.0;

  cached_path_info.min_radius = 100000.0;

  global_data_inited = true;

  ILOG_INFO << "init rs path";

  return 0;
}

RSPathInfo *GetRSPathGlobalInfo(void) {
  if (!global_data_inited) {
    reeds_shepp_init();
  }

  return &cached_path_info;
}

int GetRSPathGearSwitchNum(int *gear_switch_num, const Pose2D *start_pose,
                           const Pose2D *goal_pose, float min_turn_radius,
                           AstarPathGear initial_pose_dir) {
  int i, size;
  float length[MAX_RS_PATH_NUM];
  AstarPathGear dir[MAX_RS_PATH_NUM];

  RSPathParam *path;
  RSPathInfo *path_info = GetRSPathGlobalInfo();

  if (gear_switch_num == nullptr || start_pose == nullptr ||
      goal_pose == nullptr) {
    ILOG_INFO << "nullptr";
    return 0;
  }

  *gear_switch_num = 0;
  path = &path_info->path;
  if (start_pose->IsSame(goal_pose)) {
    ILOG_INFO << "same point";
    return 0;
  }

  RSPathRequestType rs_request = RSPathRequestType::NONE;
  GetShortestRSPathParam(path, start_pose, goal_pose, min_turn_radius,
                         1.0 / min_turn_radius, rs_request);

  path_info->is_path_valid = false;

  path_info->start = *start_pose;
  path_info->end = *goal_pose;
  path_info->min_radius = min_turn_radius;

  size = 0;
  for (i = 0; i < MAX_RS_PATH_NUM; i++) {
    if (ifly_fgreater(std::fabs(path->length[i]), 0.0)) {
      length[size] = path->length[i];
      size++;
    }
  }

  for (i = 0; i < size; i++) {
    dir[i] = get_signed_segment_dir(length[i]);
  }

  for (i = 1; i < size; i++) {
    if (dir[i] != dir[i - 1]) {
      (*gear_switch_num)++;
    }
  }

  if (initial_pose_dir != AstarPathGear::NONE && initial_pose_dir != dir[0]) {
    (*gear_switch_num)++;
  }

#if 0
    printf("gear_switch_num: %d\n", *gear_switch_num);
#endif

  return 0;
}

int GetRSPathDist(float *distance, const Pose2D *start_pose,
                  const Pose2D *goal_pose, float min_turn_radius) {
  RSPathParam *path;
  RSPathInfo *path_info = GetRSPathGlobalInfo();

  if (distance == nullptr || start_pose == nullptr || goal_pose == nullptr) {
    ILOG_INFO << "nullptr";
    return 0;
  }

  *distance = 0;
  path = &path_info->path;

  if (start_pose->IsSame(goal_pose)) {
    ILOG_INFO << "same point";
    return 0;
  }

  RSPathRequestType rs_request = RSPathRequestType::NONE;
  GetShortestRSPathParam(path, start_pose, goal_pose, min_turn_radius,
                         1.0 / min_turn_radius, rs_request);

  path_info->is_path_valid = false;

  path_info->start = *start_pose;
  path_info->end = *goal_pose;
  path_info->min_radius = min_turn_radius;

  *distance = path->total_length;

  return 0;
}

int TestSCS(RSPathParam *path, const Pose2D *start_pose,
            const Pose2D *goal_pose, float min_turn_radius,
            float inverse_radius, const RSPathRequestType request_type) {
  float dx, dy, dtheta, cos_theta, sin_theta, x, y, Lmin;

  if (path == nullptr || start_pose == nullptr || goal_pose == nullptr) {
    return 0;
  }

  if (IsPointEqual(start_pose, goal_pose)) {
    SetRSPathParam(path, ReedsSheppPathype[0], 0, 0, 0, 0, 0);
  } else {
    dx = goal_pose->x - start_pose->x;
    dy = goal_pose->y - start_pose->y;
    dtheta = goal_pose->theta - start_pose->theta;

    cos_theta = ifly_cos(start_pose->theta);
    sin_theta = ifly_sin(start_pose->theta);
    x = (cos_theta * dx + sin_theta * dy) * inverse_radius;
    y = (-sin_theta * dx + cos_theta * dy) * inverse_radius;
    Lmin = 100000.0;

    RSEndPoint end;
    end.pose.x = x;
    end.pose.y = y;
    end.pose.theta = dtheta;

    end.sin_theta = ifly_sin(dtheta);
    end.cos_theta = ifly_cos(dtheta);
    end.cos_minus_theta = end.cos_theta;
    end.sin_minus_theta = -end.sin_theta;

    re_request_type_ = request_type;

    SCS(path, &Lmin, &end);
  }

  path->length[0] = path->t * min_turn_radius;
  path->length[1] = path->u * min_turn_radius;
  path->length[2] = path->v * min_turn_radius;
  path->length[3] = path->w * min_turn_radius;
  path->length[4] = path->x * min_turn_radius;

  path->total_length = std::fabs(path->length[0]) + std::fabs(path->length[1]) +
                       std::fabs(path->length[2]) + std::fabs(path->length[3]) +
                       std::fabs(path->length[4]);

#if 0
    int i;
    for (i = 0; i < MAX_RS_PATH_NUM; i++) {
      printf("path.length[%d], %.3f\n", i, path->length[i]);
    }
#endif

  return 0;
}

}  // namespace planning