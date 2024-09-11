#include "reeds_shepp_path.h"

#include <cstddef>

#include "log_glog.h"
#include "pose2d.h"
#include "transform2d.h"

namespace planning {

#define RS_PATH_POINT_DIST (0.1)
#define DEBUG_RS_PATH (0)

ReedShepp::ReedShepp(const VehicleParam& vehicle_param,
                     const PlannerOpenSpaceConfig& open_space_conf)
    : vehicle_param_(vehicle_param),
      planner_open_space_config_(open_space_conf) {
  max_kappa_ =
      std::tan(vehicle_param_.max_steer_angle / vehicle_param_.steer_ratio) /
      vehicle_param_.wheel_base;

  min_radius_ = 1 / max_kappa_;
}

std::pair<double, double> ReedShepp::calc_tau_omega(const double u,
                                                    const double v,
                                                    const double xi,
                                                    const double eta,
                                                    const double phi) {
  double delta = ad_common::math::NormalizeAngle(u - v);
  double A = std::sin(u) - std::sin(delta);
  double B = std::cos(u) - std::cos(delta) - 1.0;

  double t1 = std::atan2(eta * A - xi * B, xi * A + eta * B);
  double t2 = 2.0 * (std::cos(delta) - std::cos(v) - std::cos(u)) + 3.0;
  double tau = 0.0;
  if (t2 < 0) {
    tau = ad_common::math::NormalizeAngle(t1 + M_PI);
  } else {
    tau = ad_common::math::NormalizeAngle(t1);
  }
  double omega = ad_common::math::NormalizeAngle(tau - u + v - phi);
  return std::make_pair(tau, omega);
}

bool ReedShepp::ShortestRSPath(const Pose2D* start_node, const Pose2D* end_node,
                               ReedSheppPath* optimal_path) {
  std::vector<ReedSheppPath> all_possible_paths;
  if (!GenerateRSPathList(start_node, end_node, &all_possible_paths)) {
    ILOG_INFO << "Fail to generate different combination of Reed Shepp "
                 "paths";
    return false;
  }

  double optimal_path_length = std::numeric_limits<double>::infinity();
  size_t optimal_path_index = 0;
  size_t paths_size = all_possible_paths.size();

  double path_length;

  for (size_t i = 0; i < paths_size; ++i) {
    path_length = all_possible_paths.at(i).total_length;

    if (path_length > 0 && path_length < optimal_path_length) {
      optimal_path_index = i;
      optimal_path_length = path_length;
    }
  }

  ReedSheppPath* shortest_path = &all_possible_paths[optimal_path_index];

  if (!GenerateGlobalRSPath(start_node, end_node, shortest_path)) {
    ILOG_INFO << "Fail to generate local configurations(x, y, phi) in SetRSP";

    return false;
  }

  // check end node
  if (std::fabs(shortest_path->x.back() - end_node->GetX()) > 1e-3 ||
      std::fabs(shortest_path->y.back() - end_node->GetY()) > 1e-3 ||
      std::fabs(shortest_path->phi.back() - end_node->GetPhi()) > 1e-3) {
    ILOG_INFO << "RSP end position not right";
    ILOG_INFO << "x, y, phi are: " << shortest_path->x.back() << ", "
              << shortest_path->y.back() << ", " << shortest_path->phi.back();
    ILOG_INFO << "end x, y, phi are: " << end_node->GetX() << ", "
              << end_node->GetY() << ", " << end_node->GetPhi();

    return false;
  }

  optimal_path->x = shortest_path->x;
  optimal_path->y = shortest_path->y;
  optimal_path->phi = shortest_path->phi;
  optimal_path->gear = shortest_path->gear;
  optimal_path->total_length = shortest_path->total_length;
  optimal_path->seg_list_steer_type = shortest_path->seg_list_steer_type;
  optimal_path->segs_lengths = shortest_path->segs_lengths;

  return true;
}

bool ReedShepp::GenerateRSPathList(
    const Pose2D* start_node, const Pose2D* end_node,
    std::vector<ReedSheppPath>* all_possible_paths) {
#if DEBUG_RS_PATH
// ILOG_INFO << "one thread to generate rs path";
#endif

  if (!GenerateRSP(start_node, end_node, all_possible_paths)) {
    ILOG_ERROR << "Fail to generate general profile of different RSPs";

    // start_node->DebugString();
    // end_node->DebugString();
    return false;
  }

  return true;
}

bool ReedShepp::GenerateRSP(const Pose2D* start_node, const Pose2D* end_node,
                            std::vector<ReedSheppPath>* all_possible_paths) {
  double dx = end_node->GetX() - start_node->GetX();
  double dy = end_node->GetY() - start_node->GetY();
  double dphi = end_node->GetPhi() - start_node->GetPhi();
  double c = std::cos(start_node->GetPhi());
  double s = std::sin(start_node->GetPhi());
  // normalize the initial point to (0,0,0)
  double x = (c * dx + s * dy) * max_kappa_;
  double y = (-s * dx + c * dy) * max_kappa_;
  if (!SCS(x, y, dphi, all_possible_paths)) {
    ILOG_DEBUG << "Fail at SCS";
  }
  if (!CSC(x, y, dphi, all_possible_paths)) {
    ILOG_DEBUG << "Fail at CSC";
  }
  if (!CCC(x, y, dphi, all_possible_paths)) {
    ILOG_DEBUG << "Fail at CCC";
  }
  if (!CCCC(x, y, dphi, all_possible_paths)) {
    ILOG_DEBUG << "Fail at CCCC";
  }
  if (!CCSC(x, y, dphi, all_possible_paths)) {
    ILOG_DEBUG << "Fail at CCSC";
  }
  if (!CCSCC(x, y, dphi, all_possible_paths)) {
    ILOG_DEBUG << "Fail at CCSCC";
  }

  if (all_possible_paths->empty()) {
    ILOG_DEBUG << "No path generated by certain two configurations";
    return false;
  }

  return true;
}

bool ReedShepp::SCS(const double x, const double y, const double phi,
                    std::vector<ReedSheppPath>* all_possible_paths) {
  RSPParam SLS_param;
  SLS(x, y, phi, &SLS_param);
  double SLS_lengths[3] = {SLS_param.t, SLS_param.u, SLS_param.v};
  // char SLS_types[] = "SLS";

  AstarPathSteer SLS_types[3];
  SLS_types[0] = AstarPathSteer::straight;
  SLS_types[1] = AstarPathSteer::left;
  SLS_types[2] = AstarPathSteer::straight;

  if (SLS_param.flag &&
      !SetRSP(3, SLS_lengths, SLS_types, all_possible_paths)) {
    ILOG_DEBUG << "Fail at SetRSP with SLS_param";
    return false;
  }

  RSPParam SRS_param;
  SLS(x, -y, -phi, &SRS_param);
  double SRS_lengths[3] = {SRS_param.t, SRS_param.u, SRS_param.v};
  // char SRS_types[] = "SRS";

  AstarPathSteer SRS_types[3];
  SRS_types[0] = AstarPathSteer::straight;
  SRS_types[1] = AstarPathSteer::right;
  SRS_types[2] = AstarPathSteer::straight;

  if (SRS_param.flag &&
      !SetRSP(3, SRS_lengths, SRS_types, all_possible_paths)) {
    ILOG_DEBUG << "Fail at SetRSP with SRS_param";
    return false;
  }
  return true;
}

bool ReedShepp::CSC(const double x, const double y, const double phi,
                    std::vector<ReedSheppPath>* all_possible_paths) {
  RSPParam LSL1_param;
  LSL(x, y, phi, &LSL1_param);
  double LSL1_lengths[3] = {LSL1_param.t, LSL1_param.u, LSL1_param.v};
  // char LSL1_types[] = "LSL";

  AstarPathSteer LSL1_types[3];
  LSL1_types[0] = AstarPathSteer::left;
  LSL1_types[1] = AstarPathSteer::straight;
  LSL1_types[2] = AstarPathSteer::left;

  if (LSL1_param.flag &&
      !SetRSP(3, LSL1_lengths, LSL1_types, all_possible_paths)) {
    ILOG_DEBUG << "Fail at SetRSP with LSL_param";
    return false;
  }

  RSPParam LSL2_param;
  LSL(-x, y, -phi, &LSL2_param);
  double LSL2_lengths[3] = {-LSL2_param.t, -LSL2_param.u, -LSL2_param.v};
  // char LSL2_types[] = "LSL";

  AstarPathSteer LSL2_types[3];
  LSL2_types[0] = AstarPathSteer::left;
  LSL2_types[1] = AstarPathSteer::straight;
  LSL2_types[2] = AstarPathSteer::left;

  if (LSL2_param.flag &&
      !SetRSP(3, LSL2_lengths, LSL2_types, all_possible_paths)) {
    ILOG_DEBUG << "Fail at SetRSP with LSL2_param";
    return false;
  }

  RSPParam LSL3_param;
  LSL(x, -y, -phi, &LSL3_param);
  double LSL3_lengths[3] = {LSL3_param.t, LSL3_param.u, LSL3_param.v};
  // char LSL3_types[] = "RSR";

  AstarPathSteer LSL3_types[3];
  LSL3_types[0] = AstarPathSteer::right;
  LSL3_types[1] = AstarPathSteer::straight;
  LSL3_types[2] = AstarPathSteer::right;

  if (LSL3_param.flag &&
      !SetRSP(3, LSL3_lengths, LSL3_types, all_possible_paths)) {
    ILOG_DEBUG << "Fail at SetRSP with LSL3_param";
    return false;
  }

  RSPParam LSL4_param;
  LSL(-x, -y, phi, &LSL4_param);
  double LSL4_lengths[3] = {-LSL4_param.t, -LSL4_param.u, -LSL4_param.v};
  // char LSL4_types[] = "RSR";

  AstarPathSteer LSL4_types[3];
  LSL4_types[0] = AstarPathSteer::right;
  LSL4_types[1] = AstarPathSteer::straight;
  LSL4_types[2] = AstarPathSteer::right;

  if (LSL4_param.flag &&
      !SetRSP(3, LSL4_lengths, LSL4_types, all_possible_paths)) {
    ILOG_DEBUG << "Fail at SetRSP with LSL4_param";
    return false;
  }

  RSPParam LSR1_param;
  LSR(x, y, phi, &LSR1_param);
  double LSR1_lengths[3] = {LSR1_param.t, LSR1_param.u, LSR1_param.v};
  // char LSR1_types[] = "LSR";

  AstarPathSteer LSR1_types[3];
  LSR1_types[0] = AstarPathSteer::left;
  LSR1_types[1] = AstarPathSteer::straight;
  LSR1_types[2] = AstarPathSteer::right;

  if (LSR1_param.flag &&
      !SetRSP(3, LSR1_lengths, LSR1_types, all_possible_paths)) {
    ILOG_DEBUG << "Fail at SetRSP with LSR1_param";
    return false;
  }

  RSPParam LSR2_param;
  LSR(-x, y, -phi, &LSR2_param);
  double LSR2_lengths[3] = {-LSR2_param.t, -LSR2_param.u, -LSR2_param.v};
  // char LSR2_types[] = "LSR";

  AstarPathSteer LSR2_types[3];
  LSR2_types[0] = AstarPathSteer::left;
  LSR2_types[1] = AstarPathSteer::straight;
  LSR2_types[2] = AstarPathSteer::right;

  if (LSR2_param.flag &&
      !SetRSP(3, LSR2_lengths, LSR2_types, all_possible_paths)) {
    ILOG_DEBUG << "Fail at SetRSP with LSR2_param";
    return false;
  }

  RSPParam LSR3_param;
  LSR(x, -y, -phi, &LSR3_param);
  double LSR3_lengths[3] = {LSR3_param.t, LSR3_param.u, LSR3_param.v};
  // char LSR3_types[] = "RSL";

  AstarPathSteer LSR3_types[3];
  LSR3_types[0] = AstarPathSteer::right;
  LSR3_types[1] = AstarPathSteer::straight;
  LSR3_types[2] = AstarPathSteer::left;

  if (LSR3_param.flag &&
      !SetRSP(3, LSR3_lengths, LSR3_types, all_possible_paths)) {
    ILOG_DEBUG << "Fail at SetRSP with LSR3_param";
    return false;
  }

  RSPParam LSR4_param;
  LSR(-x, -y, phi, &LSR4_param);
  double LSR4_lengths[3] = {-LSR4_param.t, -LSR4_param.u, -LSR4_param.v};
  // char LSR4_types[] = "RSL";

  AstarPathSteer LSR4_types[3];
  LSR4_types[0] = AstarPathSteer::right;
  LSR4_types[1] = AstarPathSteer::straight;
  LSR4_types[2] = AstarPathSteer::left;

  if (LSR4_param.flag &&
      !SetRSP(3, LSR4_lengths, LSR4_types, all_possible_paths)) {
    ILOG_DEBUG << "Fail at SetRSP with LSR4_param";
    return false;
  }
  return true;
}

bool ReedShepp::CCC(const double x, const double y, const double phi,
                    std::vector<ReedSheppPath>* all_possible_paths) {
  RSPParam LRL1_param;
  LRL(x, y, phi, &LRL1_param);
  double LRL1_lengths[3] = {LRL1_param.t, LRL1_param.u, LRL1_param.v};
  // char LRL1_types[] = "LRL";

  AstarPathSteer LRL1_types[3];
  LRL1_types[0] = AstarPathSteer::left;
  LRL1_types[1] = AstarPathSteer::right;
  LRL1_types[2] = AstarPathSteer::left;

  if (LRL1_param.flag &&
      !SetRSP(3, LRL1_lengths, LRL1_types, all_possible_paths)) {
    ILOG_DEBUG << "Fail at SetRSP with LRL_param";
    return false;
  }

  RSPParam LRL2_param;
  LRL(-x, y, -phi, &LRL2_param);
  double LRL2_lengths[3] = {-LRL2_param.t, -LRL2_param.u, -LRL2_param.v};
  // char LRL2_types[] = "LRL";

  AstarPathSteer LRL2_types[3];
  LRL2_types[0] = AstarPathSteer::left;
  LRL2_types[1] = AstarPathSteer::right;
  LRL2_types[2] = AstarPathSteer::left;

  if (LRL2_param.flag &&
      !SetRSP(3, LRL2_lengths, LRL2_types, all_possible_paths)) {
    ILOG_DEBUG << "Fail at SetRSP with LRL2_param";
    return false;
  }

  RSPParam LRL3_param;
  LRL(x, -y, -phi, &LRL3_param);
  double LRL3_lengths[3] = {LRL3_param.t, LRL3_param.u, LRL3_param.v};
  // char LRL3_types[] = "RLR";

  AstarPathSteer LRL3_types[3];
  LRL3_types[0] = AstarPathSteer::right;
  LRL3_types[1] = AstarPathSteer::left;
  LRL3_types[2] = AstarPathSteer::right;
  if (LRL3_param.flag &&
      !SetRSP(3, LRL3_lengths, LRL3_types, all_possible_paths)) {
    ILOG_DEBUG << "Fail at SetRSP with LRL3_param";
    return false;
  }

  RSPParam LRL4_param;
  LRL(-x, -y, phi, &LRL4_param);
  double LRL4_lengths[3] = {-LRL4_param.t, -LRL4_param.u, -LRL4_param.v};
  // char LRL4_types[] = "RLR";

  AstarPathSteer LRL4_types[3];
  LRL4_types[0] = AstarPathSteer::right;
  LRL4_types[1] = AstarPathSteer::left;
  LRL4_types[2] = AstarPathSteer::right;

  if (LRL4_param.flag &&
      !SetRSP(3, LRL4_lengths, LRL4_types, all_possible_paths)) {
    ILOG_DEBUG << "Fail at SetRSP with LRL4_param";
    return false;
  }

  // backward
  double xb = x * std::cos(phi) + y * std::sin(phi);
  double yb = x * std::sin(phi) - y * std::cos(phi);

  RSPParam LRL5_param;
  LRL(xb, yb, phi, &LRL5_param);
  double LRL5_lengths[3] = {LRL5_param.v, LRL5_param.u, LRL5_param.t};
  // char LRL5_types[] = "LRL";

  AstarPathSteer LRL5_types[3];
  LRL5_types[0] = AstarPathSteer::left;
  LRL5_types[1] = AstarPathSteer::right;
  LRL5_types[2] = AstarPathSteer::left;

  if (LRL5_param.flag &&
      !SetRSP(3, LRL5_lengths, LRL5_types, all_possible_paths)) {
    ILOG_DEBUG << "Fail at SetRSP with LRL5_param";
    return false;
  }

  RSPParam LRL6_param;
  LRL(-xb, yb, -phi, &LRL6_param);
  double LRL6_lengths[3] = {-LRL6_param.v, -LRL6_param.u, -LRL6_param.t};
  // char LRL6_types[] = "LRL";

  AstarPathSteer LRL6_types[3];
  LRL6_types[0] = AstarPathSteer::left;
  LRL6_types[1] = AstarPathSteer::right;
  LRL6_types[2] = AstarPathSteer::left;

  if (LRL6_param.flag &&
      !SetRSP(3, LRL6_lengths, LRL6_types, all_possible_paths)) {
    ILOG_DEBUG << "Fail at SetRSP with LRL6_param";
    return false;
  }

  RSPParam LRL7_param;
  LRL(xb, -yb, -phi, &LRL7_param);
  double LRL7_lengths[3] = {LRL7_param.v, LRL7_param.u, LRL7_param.t};
  // char LRL7_types[] = "RLR";

  AstarPathSteer LRL7_types[3];
  LRL7_types[0] = AstarPathSteer::right;
  LRL7_types[1] = AstarPathSteer::left;
  LRL7_types[2] = AstarPathSteer::right;
  if (LRL7_param.flag &&
      !SetRSP(3, LRL7_lengths, LRL7_types, all_possible_paths)) {
    ILOG_DEBUG << "Fail at SetRSP with LRL7_param";
    return false;
  }

  RSPParam LRL8_param;
  LRL(-xb, -yb, phi, &LRL8_param);
  double LRL8_lengths[3] = {-LRL8_param.v, -LRL8_param.u, -LRL8_param.t};
  // char LRL8_types[] = "RLR";

  AstarPathSteer LRL8_types[3];
  LRL8_types[0] = AstarPathSteer::right;
  LRL8_types[1] = AstarPathSteer::left;
  LRL8_types[2] = AstarPathSteer::right;
  if (LRL8_param.flag &&
      !SetRSP(3, LRL8_lengths, LRL8_types, all_possible_paths)) {
    ILOG_DEBUG << "Fail at SetRSP with LRL8_param";
    return false;
  }
  return true;
}

bool ReedShepp::CCCC(const double x, const double y, const double phi,
                     std::vector<ReedSheppPath>* all_possible_paths) {
  RSPParam LRLRn1_param;
  LRLRn(x, y, phi, &LRLRn1_param);
  double LRLRn1_lengths[4] = {LRLRn1_param.t, LRLRn1_param.u, -LRLRn1_param.u,
                              LRLRn1_param.v};
  // char LRLRn1_types[] = "LRLR";

  AstarPathSteer LRLRn1_types[4];
  LRLRn1_types[0] = AstarPathSteer::left;
  LRLRn1_types[1] = AstarPathSteer::right;
  LRLRn1_types[2] = AstarPathSteer::left;
  LRLRn1_types[3] = AstarPathSteer::right;

  if (LRLRn1_param.flag &&
      !SetRSP(4, LRLRn1_lengths, LRLRn1_types, all_possible_paths)) {
    ILOG_DEBUG << "Fail at SetRSP with LRLRn_param";
    return false;
  }

  RSPParam LRLRn2_param;
  LRLRn(-x, y, -phi, &LRLRn2_param);
  double LRLRn2_lengths[4] = {-LRLRn2_param.t, -LRLRn2_param.u, LRLRn2_param.u,
                              -LRLRn2_param.v};
  // char LRLRn2_types[] = "LRLR";

  AstarPathSteer LRLRn2_types[4];
  LRLRn2_types[0] = AstarPathSteer::left;
  LRLRn2_types[1] = AstarPathSteer::right;
  LRLRn2_types[2] = AstarPathSteer::left;
  LRLRn2_types[3] = AstarPathSteer::right;

  if (LRLRn2_param.flag &&
      !SetRSP(4, LRLRn2_lengths, LRLRn2_types, all_possible_paths)) {
    ILOG_DEBUG << "Fail at SetRSP with LRLRn2_param";
    return false;
  }

  RSPParam LRLRn3_param;
  LRLRn(x, -y, -phi, &LRLRn3_param);
  double LRLRn3_lengths[4] = {LRLRn3_param.t, LRLRn3_param.u, -LRLRn3_param.u,
                              LRLRn3_param.v};
  // char LRLRn3_types[] = "RLRL";

  AstarPathSteer LRLRn3_types[4];
  LRLRn3_types[0] = AstarPathSteer::right;
  LRLRn3_types[1] = AstarPathSteer::left;
  LRLRn3_types[2] = AstarPathSteer::right;
  LRLRn3_types[3] = AstarPathSteer::left;

  if (LRLRn3_param.flag &&
      !SetRSP(4, LRLRn3_lengths, LRLRn3_types, all_possible_paths)) {
    ILOG_DEBUG << "Fail at SetRSP with LRLRn3_param";
    return false;
  }

  RSPParam LRLRn4_param;
  LRLRn(-x, -y, phi, &LRLRn4_param);
  double LRLRn4_lengths[4] = {-LRLRn4_param.t, -LRLRn4_param.u, LRLRn4_param.u,
                              -LRLRn4_param.v};
  // char LRLRn4_types[] = "RLRL";

  AstarPathSteer LRLRn4_types[4];
  LRLRn4_types[0] = AstarPathSteer::right;
  LRLRn4_types[1] = AstarPathSteer::left;
  LRLRn4_types[2] = AstarPathSteer::right;
  LRLRn4_types[3] = AstarPathSteer::left;

  if (LRLRn4_param.flag &&
      !SetRSP(4, LRLRn4_lengths, LRLRn4_types, all_possible_paths)) {
    ILOG_DEBUG << "Fail at SetRSP with LRLRn4_param";
    return false;
  }

  RSPParam LRLRp1_param;
  LRLRp(x, y, phi, &LRLRp1_param);
  double LRLRp1_lengths[4] = {LRLRp1_param.t, LRLRp1_param.u, LRLRp1_param.u,
                              LRLRp1_param.v};
  // char LRLRp1_types[] = "LRLR";

  AstarPathSteer LRLRp1_types[4];
  LRLRp1_types[0] = AstarPathSteer::left;
  LRLRp1_types[1] = AstarPathSteer::right;
  LRLRp1_types[2] = AstarPathSteer::left;
  LRLRp1_types[3] = AstarPathSteer::right;
  if (LRLRp1_param.flag &&
      !SetRSP(4, LRLRp1_lengths, LRLRp1_types, all_possible_paths)) {
    ILOG_DEBUG << "Fail at SetRSP with LRLRp1_param";
    return false;
  }

  RSPParam LRLRp2_param;
  LRLRp(-x, y, -phi, &LRLRp2_param);
  double LRLRp2_lengths[4] = {-LRLRp2_param.t, -LRLRp2_param.u, -LRLRp2_param.u,
                              -LRLRp2_param.v};
  // char LRLRp2_types[] = "LRLR";

  AstarPathSteer LRLRp2_types[4];
  LRLRp2_types[0] = AstarPathSteer::left;
  LRLRp2_types[1] = AstarPathSteer::right;
  LRLRp2_types[2] = AstarPathSteer::left;
  LRLRp2_types[3] = AstarPathSteer::right;

  if (LRLRp2_param.flag &&
      !SetRSP(4, LRLRp2_lengths, LRLRp2_types, all_possible_paths)) {
    ILOG_DEBUG << "Fail at SetRSP with LRLRp2_param";
    return false;
  }

  RSPParam LRLRp3_param;
  LRLRp(x, -y, -phi, &LRLRp3_param);
  double LRLRp3_lengths[4] = {LRLRp3_param.t, LRLRp3_param.u, LRLRp3_param.u,
                              LRLRp3_param.v};
  // char LRLRp3_types[] = "RLRL";

  AstarPathSteer LRLRp3_types[4];
  LRLRp3_types[0] = AstarPathSteer::right;
  LRLRp3_types[1] = AstarPathSteer::left;
  LRLRp3_types[2] = AstarPathSteer::right;
  LRLRp3_types[3] = AstarPathSteer::left;
  if (LRLRp3_param.flag &&
      !SetRSP(4, LRLRp3_lengths, LRLRp3_types, all_possible_paths)) {
    ILOG_DEBUG << "Fail at SetRSP with LRLRp3_param";
    return false;
  }

  RSPParam LRLRp4_param;
  LRLRp(-x, -y, phi, &LRLRp4_param);
  double LRLRp4_lengths[4] = {-LRLRp4_param.t, -LRLRp4_param.u, -LRLRp4_param.u,
                              -LRLRp4_param.v};
  // char LRLRp4_types[] = "RLRL";

  AstarPathSteer LRLRp4_types[4];
  LRLRp4_types[0] = AstarPathSteer::right;
  LRLRp4_types[1] = AstarPathSteer::left;
  LRLRp4_types[2] = AstarPathSteer::right;
  LRLRp4_types[3] = AstarPathSteer::left;
  if (LRLRp4_param.flag &&
      !SetRSP(4, LRLRp4_lengths, LRLRp4_types, all_possible_paths)) {
    ILOG_DEBUG << "Fail at SetRSP with LRLRp4_param";
    return false;
  }
  return true;
}

bool ReedShepp::CCSC(const double x, const double y, const double phi,
                     std::vector<ReedSheppPath>* all_possible_paths) {
  RSPParam LRSL1_param;
  LRSL(x, y, phi, &LRSL1_param);
  double LRSL1_lengths[4] = {LRSL1_param.t, -0.5 * M_PI, LRSL1_param.u,
                             LRSL1_param.v};
  // char LRSL1_types[] = "LRSL";

  AstarPathSteer LRSL1_types[4];
  LRSL1_types[0] = AstarPathSteer::left;
  LRSL1_types[1] = AstarPathSteer::right;
  LRSL1_types[2] = AstarPathSteer::straight;
  LRSL1_types[3] = AstarPathSteer::left;

  if (LRSL1_param.flag &&
      !SetRSP(4, LRSL1_lengths, LRSL1_types, all_possible_paths)) {
    ILOG_DEBUG << "Fail at SetRSP with LRSL1_param";
    return false;
  }

  RSPParam LRSL2_param;
  LRSL(-x, y, -phi, &LRSL2_param);
  double LRSL2_lengths[4] = {-LRSL2_param.t, 0.5 * M_PI, -LRSL2_param.u,
                             -LRSL2_param.v};
  // char LRSL2_types[] = "LRSL";

  AstarPathSteer LRSL2_types[4];
  LRSL2_types[0] = AstarPathSteer::left;
  LRSL2_types[1] = AstarPathSteer::right;
  LRSL2_types[2] = AstarPathSteer::straight;
  LRSL2_types[3] = AstarPathSteer::left;

  if (LRSL2_param.flag &&
      !SetRSP(4, LRSL2_lengths, LRSL2_types, all_possible_paths)) {
    ILOG_DEBUG << "Fail at SetRSP with LRSL2_param";
    return false;
  }

  RSPParam LRSL3_param;
  LRSL(x, -y, -phi, &LRSL3_param);
  double LRSL3_lengths[4] = {LRSL3_param.t, -0.5 * M_PI, LRSL3_param.u,
                             LRSL3_param.v};
  // char LRSL3_types[] = "RLSR";

  AstarPathSteer LRSL3_types[4];
  LRSL3_types[0] = AstarPathSteer::right;
  LRSL3_types[1] = AstarPathSteer::left;
  LRSL3_types[2] = AstarPathSteer::straight;
  LRSL3_types[3] = AstarPathSteer::right;

  if (LRSL3_param.flag &&
      !SetRSP(4, LRSL3_lengths, LRSL3_types, all_possible_paths)) {
    ILOG_DEBUG << "Fail at SetRSP with LRSL3_param";
    return false;
  }

  RSPParam LRSL4_param;
  LRSL(-x, -y, phi, &LRSL4_param);
  double LRSL4_lengths[4] = {-LRSL4_param.t, 0.5 * M_PI, -LRSL4_param.u,
                             -LRSL4_param.v};
  // char LRSL4_types[] = "RLSR";
  AstarPathSteer LRSL4_types[4];
  LRSL4_types[0] = AstarPathSteer::right;
  LRSL4_types[1] = AstarPathSteer::left;
  LRSL4_types[2] = AstarPathSteer::straight;
  LRSL4_types[3] = AstarPathSteer::right;

  if (LRSL4_param.flag &&
      !SetRSP(4, LRSL4_lengths, LRSL4_types, all_possible_paths)) {
    ILOG_DEBUG << "Fail at SetRSP with LRSL4_param";
    return false;
  }

  RSPParam LRSR1_param;
  LRSR(x, y, phi, &LRSR1_param);
  double LRSR1_lengths[4] = {LRSR1_param.t, -0.5 * M_PI, LRSR1_param.u,
                             LRSR1_param.v};
  // char LRSR1_types[] = "LRSR";

  AstarPathSteer LRSR1_types[4];
  LRSR1_types[0] = AstarPathSteer::left;
  LRSR1_types[1] = AstarPathSteer::right;
  LRSR1_types[2] = AstarPathSteer::straight;
  LRSR1_types[3] = AstarPathSteer::right;
  if (LRSR1_param.flag &&
      !SetRSP(4, LRSR1_lengths, LRSR1_types, all_possible_paths)) {
    ILOG_DEBUG << "Fail at SetRSP with LRSR1_param";
    return false;
  }

  RSPParam LRSR2_param;
  LRSR(-x, y, -phi, &LRSR2_param);
  double LRSR2_lengths[4] = {-LRSR2_param.t, 0.5 * M_PI, -LRSR2_param.u,
                             -LRSR2_param.v};
  // char LRSR2_types[] = "LRSR";

  AstarPathSteer LRSR2_types[4];
  LRSR2_types[0] = AstarPathSteer::left;
  LRSR2_types[1] = AstarPathSteer::right;
  LRSR2_types[2] = AstarPathSteer::straight;
  LRSR2_types[3] = AstarPathSteer::right;
  if (LRSR2_param.flag &&
      !SetRSP(4, LRSR2_lengths, LRSR2_types, all_possible_paths)) {
    ILOG_DEBUG << "Fail at SetRSP with LRSR2_param";
    return false;
  }

  RSPParam LRSR3_param;
  LRSR(x, -y, -phi, &LRSR3_param);
  double LRSR3_lengths[4] = {LRSR3_param.t, -0.5 * M_PI, LRSR3_param.u,
                             LRSR3_param.v};
  // char LRSR3_types[] = "RLSL";
  AstarPathSteer LRSR3_types[4];
  LRSR3_types[0] = AstarPathSteer::right;
  LRSR3_types[1] = AstarPathSteer::left;
  LRSR3_types[2] = AstarPathSteer::straight;
  LRSR3_types[3] = AstarPathSteer::left;

  if (LRSR3_param.flag &&
      !SetRSP(4, LRSR3_lengths, LRSR3_types, all_possible_paths)) {
    ILOG_DEBUG << "Fail at SetRSP with LRSR3_param";
    return false;
  }

  RSPParam LRSR4_param;
  LRSR(-x, -y, phi, &LRSR4_param);
  double LRSR4_lengths[4] = {-LRSR4_param.t, 0.5 * M_PI, -LRSR4_param.u,
                             -LRSR4_param.v};
  // char LRSR4_types[] = "RLSL";

  AstarPathSteer LRSR4_types[4];
  LRSR4_types[0] = AstarPathSteer::right;
  LRSR4_types[1] = AstarPathSteer::left;
  LRSR4_types[2] = AstarPathSteer::straight;
  LRSR4_types[3] = AstarPathSteer::left;

  if (LRSR4_param.flag &&
      !SetRSP(4, LRSR4_lengths, LRSR4_types, all_possible_paths)) {
    ILOG_DEBUG << "Fail at SetRSP with LRSR4_param";
    return false;
  }

  // backward
  double xb = x * std::cos(phi) + y * std::sin(phi);
  double yb = x * std::sin(phi) - y * std::cos(phi);

  RSPParam LRSL5_param;
  LRSL(xb, yb, phi, &LRSL5_param);
  double LRSL5_lengths[4] = {LRSL5_param.v, LRSL5_param.u, -0.5 * M_PI,
                             LRSL5_param.t};
  // char LRSL5_types[] = "LSRL";

  AstarPathSteer LRSL5_types[4];
  LRSL5_types[0] = AstarPathSteer::left;
  LRSL5_types[1] = AstarPathSteer::straight;
  LRSL5_types[2] = AstarPathSteer::right;
  LRSL5_types[3] = AstarPathSteer::left;
  if (LRSL5_param.flag &&
      !SetRSP(4, LRSL5_lengths, LRSL5_types, all_possible_paths)) {
    ILOG_DEBUG << "Fail at SetRSP with LRLRn_param";
    return false;
  }

  RSPParam LRSL6_param;
  LRSL(-xb, yb, -phi, &LRSL6_param);
  double LRSL6_lengths[4] = {-LRSL6_param.v, -LRSL6_param.u, 0.5 * M_PI,
                             -LRSL6_param.t};
  // char LRSL6_types[] = "LSRL";

  AstarPathSteer LRSL6_types[4];
  LRSL6_types[0] = AstarPathSteer::left;
  LRSL6_types[1] = AstarPathSteer::straight;
  LRSL6_types[2] = AstarPathSteer::right;
  LRSL6_types[3] = AstarPathSteer::left;

  if (LRSL6_param.flag &&
      !SetRSP(4, LRSL6_lengths, LRSL6_types, all_possible_paths)) {
    ILOG_DEBUG << "Fail at SetRSP with LRSL6_param";
    return false;
  }

  RSPParam LRSL7_param;
  LRSL(xb, -yb, -phi, &LRSL7_param);
  double LRSL7_lengths[4] = {LRSL7_param.v, LRSL7_param.u, -0.5 * M_PI,
                             LRSL7_param.t};
  // char LRSL7_types[] = "RSLR";

  AstarPathSteer LRSL7_types[4];
  LRSL7_types[0] = AstarPathSteer::right;
  LRSL7_types[1] = AstarPathSteer::straight;
  LRSL7_types[2] = AstarPathSteer::left;
  LRSL7_types[3] = AstarPathSteer::right;

  if (LRSL7_param.flag &&
      !SetRSP(4, LRSL7_lengths, LRSL7_types, all_possible_paths)) {
    ILOG_DEBUG << "Fail at SetRSP with LRSL7_param";
    return false;
  }

  RSPParam LRSL8_param;
  LRSL(-xb, -yb, phi, &LRSL8_param);
  double LRSL8_lengths[4] = {-LRSL8_param.v, -LRSL8_param.u, 0.5 * M_PI,
                             -LRSL8_param.t};
  // char LRSL8_types[] = "RSLR";

  AstarPathSteer LRSL8_types[4];
  LRSL8_types[0] = AstarPathSteer::right;
  LRSL8_types[1] = AstarPathSteer::straight;
  LRSL8_types[2] = AstarPathSteer::left;
  LRSL8_types[3] = AstarPathSteer::right;

  if (LRSL8_param.flag &&
      !SetRSP(4, LRSL8_lengths, LRSL8_types, all_possible_paths)) {
    ILOG_DEBUG << "Fail at SetRSP with LRSL8_param";
    return false;
  }

  RSPParam LRSR5_param;
  LRSR(xb, yb, phi, &LRSR5_param);
  double LRSR5_lengths[4] = {LRSR5_param.v, LRSR5_param.u, -0.5 * M_PI,
                             LRSR5_param.t};
  // char LRSR5_types[] = "RSRL";

  AstarPathSteer LRSR5_types[4];
  LRSR5_types[0] = AstarPathSteer::right;
  LRSR5_types[1] = AstarPathSteer::straight;
  LRSR5_types[2] = AstarPathSteer::right;
  LRSR5_types[3] = AstarPathSteer::left;

  if (LRSR5_param.flag &&
      !SetRSP(4, LRSR5_lengths, LRSR5_types, all_possible_paths)) {
    ILOG_DEBUG << "Fail at SetRSP with LRSR5_param";
    return false;
  }

  RSPParam LRSR6_param;
  LRSR(-xb, yb, -phi, &LRSR6_param);
  double LRSR6_lengths[4] = {-LRSR6_param.v, -LRSR6_param.u, 0.5 * M_PI,
                             -LRSR6_param.t};
  // char LRSR6_types[] = "RSRL";

  AstarPathSteer LRSR6_types[4];
  LRSR6_types[0] = AstarPathSteer::right;
  LRSR6_types[1] = AstarPathSteer::straight;
  LRSR6_types[2] = AstarPathSteer::right;
  LRSR6_types[3] = AstarPathSteer::left;

  if (LRSR6_param.flag &&
      !SetRSP(4, LRSR6_lengths, LRSR6_types, all_possible_paths)) {
    ILOG_DEBUG << "Fail at SetRSP with LRSR6_param";
    return false;
  }

  RSPParam LRSR7_param;
  LRSR(xb, -yb, -phi, &LRSR7_param);
  double LRSR7_lengths[4] = {LRSR7_param.v, LRSR7_param.u, -0.5 * M_PI,
                             LRSR7_param.t};
  // char LRSR7_types[] = "LSLR";

  AstarPathSteer LRSR7_types[4];
  LRSR7_types[0] = AstarPathSteer::left;
  LRSR7_types[1] = AstarPathSteer::straight;
  LRSR7_types[2] = AstarPathSteer::left;
  LRSR7_types[3] = AstarPathSteer::right;

  if (LRSR7_param.flag &&
      !SetRSP(4, LRSR7_lengths, LRSR7_types, all_possible_paths)) {
    ILOG_DEBUG << "Fail at SetRSP with LRSR7_param";
    return false;
  }

  RSPParam LRSR8_param;
  LRSR(-xb, -yb, phi, &LRSR8_param);
  double LRSR8_lengths[4] = {-LRSR8_param.v, -LRSR8_param.u, 0.5 * M_PI,
                             -LRSR8_param.t};
  // char LRSR8_types[] = "LSLR";

  AstarPathSteer LRSR8_types[4];
  LRSR8_types[0] = AstarPathSteer::left;
  LRSR8_types[1] = AstarPathSteer::straight;
  LRSR8_types[2] = AstarPathSteer::left;
  LRSR8_types[3] = AstarPathSteer::right;

  if (LRSR8_param.flag &&
      !SetRSP(4, LRSR8_lengths, LRSR8_types, all_possible_paths)) {
    ILOG_DEBUG << "Fail at SetRSP with LRSR8_param";
    return false;
  }
  return true;
}

bool ReedShepp::CCSCC(const double x, const double y, const double phi,
                      std::vector<ReedSheppPath>* all_possible_paths) {
  RSPParam LRSLR1_param;
  LRSLR(x, y, phi, &LRSLR1_param);
  double LRSLR1_lengths[5] = {LRSLR1_param.t, -0.5 * M_PI, LRSLR1_param.u,
                              -0.5 * M_PI, LRSLR1_param.v};
  // char LRSLR1_types[] = "LRSLR";

  AstarPathSteer LRSLR1_types[5];
  LRSLR1_types[0] = AstarPathSteer::left;
  LRSLR1_types[1] = AstarPathSteer::right;
  LRSLR1_types[2] = AstarPathSteer::straight;
  LRSLR1_types[3] = AstarPathSteer::left;
  LRSLR1_types[4] = AstarPathSteer::right;

  if (LRSLR1_param.flag &&
      !SetRSP(5, LRSLR1_lengths, LRSLR1_types, all_possible_paths)) {
    ILOG_DEBUG << "Fail at SetRSP with LRSLR1_param";
    return false;
  }

  RSPParam LRSLR2_param;
  LRSLR(-x, y, -phi, &LRSLR2_param);
  double LRSLR2_lengths[5] = {-LRSLR2_param.t, 0.5 * M_PI, -LRSLR2_param.u,
                              0.5 * M_PI, -LRSLR2_param.v};
  // char LRSLR2_types[] = "LRSLR";

  AstarPathSteer LRSLR2_types[5];
  LRSLR2_types[0] = AstarPathSteer::left;
  LRSLR2_types[1] = AstarPathSteer::right;
  LRSLR2_types[2] = AstarPathSteer::straight;
  LRSLR2_types[3] = AstarPathSteer::left;
  LRSLR2_types[4] = AstarPathSteer::right;

  if (LRSLR2_param.flag &&
      !SetRSP(5, LRSLR2_lengths, LRSLR2_types, all_possible_paths)) {
    ILOG_DEBUG << "Fail at SetRSP with LRSLR2_param";
    return false;
  }

  RSPParam LRSLR3_param;
  LRSLR(x, -y, -phi, &LRSLR3_param);
  double LRSLR3_lengths[5] = {LRSLR3_param.t, -0.5 * M_PI, LRSLR3_param.u,
                              -0.5 * M_PI, LRSLR3_param.v};
  // char LRSLR3_types[] = "RLSRL";

  AstarPathSteer LRSLR3_types[5];
  LRSLR3_types[0] = AstarPathSteer::right;
  LRSLR3_types[1] = AstarPathSteer::left;
  LRSLR3_types[2] = AstarPathSteer::straight;
  LRSLR3_types[3] = AstarPathSteer::right;
  LRSLR3_types[4] = AstarPathSteer::left;

  if (LRSLR3_param.flag &&
      !SetRSP(5, LRSLR3_lengths, LRSLR3_types, all_possible_paths)) {
    ILOG_DEBUG << "Fail at SetRSP with LRSLR3_param";
    return false;
  }

  RSPParam LRSLR4_param;
  LRSLR(-x, -y, phi, &LRSLR4_param);
  double LRSLR4_lengths[5] = {-LRSLR4_param.t, 0.5 * M_PI, -LRSLR4_param.u,
                              0.5 * M_PI, -LRSLR4_param.v};
  // char LRSLR4_types[] = "RLSRL";

  AstarPathSteer LRSLR4_types[5];
  LRSLR4_types[0] = AstarPathSteer::right;
  LRSLR4_types[1] = AstarPathSteer::left;
  LRSLR4_types[2] = AstarPathSteer::straight;
  LRSLR4_types[3] = AstarPathSteer::right;
  LRSLR4_types[4] = AstarPathSteer::left;

  if (LRSLR4_param.flag &&
      !SetRSP(5, LRSLR4_lengths, LRSLR4_types, all_possible_paths)) {
    ILOG_DEBUG << "Fail at SetRSP with LRSLR4_param";
    return false;
  }
  return true;
}

void ReedShepp::LSL(const double x, const double y, const double phi,
                    RSPParam* param) {
  std::pair<double, double> polar = ad_common::math::Cartesian2Polar(
      x - std::sin(phi), y - 1.0 + std::cos(phi));
  double u = polar.first;
  double t = polar.second;
  double v = 0.0;
  if (t >= 0.0) {
    v = ad_common::math::NormalizeAngle(phi - t);
    if (v >= 0.0) {
      param->flag = true;
      param->u = u;
      param->t = t;
      param->v = v;
    }
  }
}

void ReedShepp::LSR(const double x, const double y, const double phi,
                    RSPParam* param) {
  std::pair<double, double> polar = ad_common::math::Cartesian2Polar(
      x + std::sin(phi), y - 1.0 - std::cos(phi));
  double u1 = polar.first * polar.first;
  double t1 = polar.second;
  double u = 0.0;
  double theta = 0.0;
  double t = 0.0;
  double v = 0.0;
  if (u1 >= 4.0) {
    u = std::sqrt(u1 - 4.0);
    theta = std::atan2(2.0, u);
    t = ad_common::math::NormalizeAngle(t1 + theta);
    v = ad_common::math::NormalizeAngle(t - phi);
    if (t >= 0.0 && v >= 0.0) {
      param->flag = true;
      param->u = u;
      param->t = t;
      param->v = v;
    }
  }
}

void ReedShepp::LRL(const double x, const double y, const double phi,
                    RSPParam* param) {
  std::pair<double, double> polar = ad_common::math::Cartesian2Polar(
      x - std::sin(phi), y - 1.0 + std::cos(phi));
  double u1 = polar.first;
  double t1 = polar.second;
  double u = 0.0;
  double t = 0.0;
  double v = 0.0;
  if (u1 <= 4.0) {
    u = -2.0 * std::asin(0.25 * u1);
    t = ad_common::math::NormalizeAngle(t1 + 0.5 * u + M_PI);
    v = ad_common::math::NormalizeAngle(phi - t + u);
    if (t >= 0.0 && u <= 0.0) {
      param->flag = true;
      param->u = u;
      param->t = t;
      param->v = v;
    }
  }
}

void ReedShepp::SLS(const double x, const double y, const double phi,
                    RSPParam* param) {
  double phi_mod = ad_common::math::NormalizeAngle(phi);
  double xd = 0.0;
  double u = 0.0;
  double t = 0.0;
  double v = 0.0;
  double epsilon = 1e-1;
  if (y > 0.0 && phi_mod > epsilon && phi_mod < M_PI) {
    xd = -y / std::tan(phi_mod) + x;
    t = xd - std::tan(phi_mod / 2.0);
    u = phi_mod;
    v = std::sqrt((x - xd) * (x - xd) + y * y) - tan(phi_mod / 2.0);
    param->flag = true;
    param->u = u;
    param->t = t;
    param->v = v;
  } else if (y < 0.0 && phi_mod > epsilon && phi_mod < M_PI) {
    xd = -y / std::tan(phi_mod) + x;
    t = xd - std::tan(phi_mod / 2.0);
    u = phi_mod;
    v = -std::sqrt((x - xd) * (x - xd) + y * y) - std::tan(phi_mod / 2.0);
    param->flag = true;
    param->u = u;
    param->t = t;
    param->v = v;
  }
}

void ReedShepp::LRLRn(const double x, const double y, const double phi,
                      RSPParam* param) {
  double xi = x + std::sin(phi);
  double eta = y - 1.0 - std::cos(phi);
  double rho = 0.25 * (2.0 + std::sqrt(xi * xi + eta * eta));
  double u = 0.0;
  if (rho <= 1.0 && rho >= 0.0) {
    u = std::acos(rho);
    if (u >= 0 && u <= 0.5 * M_PI) {
      std::pair<double, double> tau_omega = calc_tau_omega(u, -u, xi, eta, phi);
      if (tau_omega.first >= 0.0 && tau_omega.second <= 0.0) {
        param->flag = true;
        param->u = u;
        param->t = tau_omega.first;
        param->v = tau_omega.second;
      }
    }
  }
}

void ReedShepp::LRLRp(const double x, const double y, const double phi,
                      RSPParam* param) {
  double xi = x + std::sin(phi);
  double eta = y - 1.0 - std::cos(phi);
  double rho = (20.0 - xi * xi - eta * eta) / 16.0;
  double u = 0.0;
  if (rho <= 1.0 && rho >= 0.0) {
    u = -std::acos(rho);
    if (u >= 0 && u <= 0.5 * M_PI) {
      std::pair<double, double> tau_omega = calc_tau_omega(u, u, xi, eta, phi);
      if (tau_omega.first >= 0.0 && tau_omega.second >= 0.0) {
        param->flag = true;
        param->u = u;
        param->t = tau_omega.first;
        param->v = tau_omega.second;
      }
    }
  }
}

void ReedShepp::LRSR(const double x, const double y, const double phi,
                     RSPParam* param) {
  double xi = x + std::sin(phi);
  double eta = y - 1.0 - std::cos(phi);
  std::pair<double, double> polar = ad_common::math::Cartesian2Polar(-eta, xi);
  double rho = polar.first;
  double theta = polar.second;
  double t = 0.0;
  double u = 0.0;
  double v = 0.0;
  if (rho >= 2.0) {
    t = theta;
    u = 2.0 - rho;
    v = ad_common::math::NormalizeAngle(t + 0.5 * M_PI - phi);
    if (t >= 0.0 && u <= 0.0 && v <= 0.0) {
      param->flag = true;
      param->u = u;
      param->t = t;
      param->v = v;
    }
  }
}

void ReedShepp::LRSL(const double x, const double y, const double phi,
                     RSPParam* param) {
  double xi = x - std::sin(phi);
  double eta = y - 1.0 + std::cos(phi);
  std::pair<double, double> polar = ad_common::math::Cartesian2Polar(xi, eta);
  double rho = polar.first;
  double theta = polar.second;
  double r = 0.0;
  double t = 0.0;
  double u = 0.0;
  double v = 0.0;

  if (rho >= 2.0) {
    r = std::sqrt(rho * rho - 4.0);
    u = 2.0 - r;
    t = ad_common::math::NormalizeAngle(theta + std::atan2(r, -2.0));
    v = ad_common::math::NormalizeAngle(phi - 0.5 * M_PI - t);
    if (t >= 0.0 && u <= 0.0 && v <= 0.0) {
      param->flag = true;
      param->u = u;
      param->t = t;
      param->v = v;
    }
  }
}

void ReedShepp::LRSLR(const double x, const double y, const double phi,
                      RSPParam* param) {
  double xi = x + std::sin(phi);
  double eta = y - 1.0 - std::cos(phi);
  std::pair<double, double> polar = ad_common::math::Cartesian2Polar(xi, eta);
  double rho = polar.first;
  double t = 0.0;
  double u = 0.0;
  double v = 0.0;
  if (rho >= 2.0) {
    u = 4.0 - std::sqrt(rho * rho - 4.0);
    if (u <= 0.0) {
      t = ad_common::math::NormalizeAngle(
          atan2((4.0 - u) * xi - 2.0 * eta, -2.0 * xi + (u - 4.0) * eta));
      v = ad_common::math::NormalizeAngle(t - phi);

      if (t >= 0.0 && v >= 0.0) {
        param->flag = true;
        param->u = u;
        param->t = t;
        param->v = v;
      }
    }
  }
}

bool ReedShepp::SetRSP(const int size, const double* lengths,
                       const AstarPathSteer* types,
                       std::vector<ReedSheppPath>* all_possible_paths) {
  ReedSheppPath path;
  std::vector<double> length_vec;
  std::vector<AstarPathSteer> steer_vec;

  for (int i = 0; i < size; i++) {
    length_vec.emplace_back(lengths[i]);
    steer_vec.emplace_back(types[i]);
  }

  path.segs_lengths = length_vec;
  path.seg_list_steer_type = steer_vec;

  double sum = 0.0;
  for (int i = 0; i < size; ++i) {
    sum += std::fabs(lengths[i]);
  }

  path.total_length = sum;
  if (path.total_length <= 0.0) {
    ILOG_ERROR << "total length smaller than 0, len is:" << path.total_length;
    return false;
  }

  all_possible_paths->emplace_back(path);
  return true;
}

// TODO(Jinyun) : reformulate GenerateGlobalRSPath.
bool ReedShepp::GenerateGlobalRSPath(const Pose2D* start_node,
                                     const Pose2D* end_node,
                                     ReedSheppPath* shortest_path) {
  // todo
  // double step_scaled = planner_open_space_config_.step_size * max_kappa_;

  // 尺度缩放的间隔
  double point_interval_scaled = RS_PATH_POINT_DIST * max_kappa_;

  int point_num = std::floor(shortest_path->total_length * min_radius_ /
                             RS_PATH_POINT_DIST) +
                  shortest_path->segs_lengths.size() + 4;

  // push all rs path points to this
  std::vector<RSPathPoint> rs_path(point_num);

  // 缩放空间的2点间隔距离
  double interval_dist = 0.0;
  // 每一段path的累计距离，经过缩放的
  double path_seg_accu_dist = 0.0;

  // first point in px is start point
  int next_point_id = 0;
  Pose2D seg_base_pose;

  for (size_t i = 0; i < shortest_path->seg_list_steer_type.size(); ++i) {
    AstarPathSteer steer = shortest_path->seg_list_steer_type.at(i);

    double length = shortest_path->segs_lengths.at(i);
    if (length > 0.0) {
      interval_dist = point_interval_scaled;
    } else {
      interval_dist = -point_interval_scaled;
    }

    // 上一段path的最后一个点
    if (i == 0) {
      seg_base_pose.x = 0.0;
      seg_base_pose.y = 0.0;
      seg_base_pose.theta = 0.0;
    } else {
      seg_base_pose.x = rs_path[next_point_id - 1].x;
      seg_base_pose.y = rs_path[next_point_id - 1].y;
      seg_base_pose.theta = rs_path[next_point_id - 1].theta;
    }

    path_seg_accu_dist = 0.0;

    // if same gear, skip same point
    if (i > 1 && shortest_path->segs_lengths.at(i - 1) * length > 0.0) {
      path_seg_accu_dist = interval_dist;
    }

    while (std::fabs(path_seg_accu_dist) < std::fabs(length)) {
      Interpolation(next_point_id, path_seg_accu_dist, steer, seg_base_pose,
                    interval_dist, rs_path);

      path_seg_accu_dist += interval_dist;
      next_point_id++;
    }

    // 最后一个点
    if (std::fabs(path_seg_accu_dist - interval_dist) < std::fabs(length)) {
      Interpolation(next_point_id, length, steer, seg_base_pose, interval_dist,
                    rs_path);

      path_seg_accu_dist = length;
      next_point_id++;
    }
  }

  point_num = next_point_id;

  // update original pose
  planning::Transform2d tf;
  Pose2D start_pose;
  Pose2D local_pose;
  Pose2D global_pose;
  start_pose.x = start_node->GetX();
  start_pose.y = start_node->GetY();
  start_pose.theta = start_node->GetPhi();

  tf.SetBasePose(start_pose);

  for (int i = 0; i < point_num; ++i) {
    local_pose.x = rs_path[i].x;
    local_pose.y = rs_path[i].y;
    local_pose.theta = rs_path[i].theta;

    tf.ULFLocalPoseToGlobal(&global_pose, local_pose);

    shortest_path->x.push_back(global_pose.x);
    shortest_path->y.push_back(global_pose.y);
    shortest_path->phi.push_back(global_pose.theta);
    shortest_path->gear.push_back(rs_path[i].gear);
  }

  // update original length
  for (size_t i = 0; i < shortest_path->segs_lengths.size(); ++i) {
    shortest_path->segs_lengths.at(i) =
        shortest_path->segs_lengths.at(i) * min_radius_;
  }

  shortest_path->total_length = shortest_path->total_length * min_radius_;

  return true;
}

void ReedShepp::Interpolation(const int index, const double path_dist,
                              const AstarPathSteer steer,
                              const Pose2D& base_pose,
                              const double interval_dist,
                              std::vector<RSPathPoint>& rs_path) {
  Position2D local;
  Position2D global;

  // ILOG_INFO << "index " << index;

  // 半径为1
  double rotate_theta = path_dist;

  if (steer == AstarPathSteer::straight) {
    rs_path[index].x =
        base_pose.x + path_dist * std::cos(base_pose.theta) * min_radius_;
    rs_path[index].y =
        base_pose.y + path_dist * std::sin(base_pose.theta) * min_radius_;
    rs_path[index].theta = base_pose.theta;
  } else {
    local.x = std::sin(rotate_theta) * min_radius_;

    if (steer == AstarPathSteer::left) {
      local.y = (1.0 - std::cos(rotate_theta)) * min_radius_;
    } else if (steer == AstarPathSteer::right) {
      local.y = -(1.0 - std::cos(rotate_theta)) * min_radius_;
    }

    planning::Transform2d tf;
    tf.SetBasePose(base_pose);

    // 相对于start node的global pose
    tf.ULFLocalPointToGlobal(&global, local);

    rs_path[index].x = global.x;
    rs_path[index].y = global.y;
  }

  if (interval_dist > 0.0) {
    rs_path[index].gear = AstarPathGear::drive;
  } else {
    rs_path[index].gear = AstarPathGear::reverse;
  }

  if (steer == AstarPathSteer::left) {
    rs_path[index].theta = base_pose.theta + rotate_theta;
  } else if (steer == AstarPathSteer::right) {
    rs_path[index].theta = base_pose.theta - rotate_theta;
  }
}

}  // namespace planning
