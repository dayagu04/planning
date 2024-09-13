#pragma once

#include <string>
#include <vector>

#include "./../../modules/common/config/vehicle_param.h"
#include "ad_common/math/math_utils.h"
#include "hybrid_astar_common.h"
#include "hybrid_astar_config.h"
#include "log_glog.h"
#include "planning_gflags.h"
#include "pose2d.h"
#include "src/common/macro.h"

namespace planning {

// retired
struct ReedSheppPath {
  // if value > 0, gear is forward
  std::vector<double> segs_lengths;

  // left or right, or straight
  std::vector<AstarPathSteer> seg_list_steer_type;

  // all path points
  std::vector<AstarPathGear> gear;
  // value > 0
  double total_length = 0.0;

  std::vector<double> x;
  std::vector<double> y;
  std::vector<double> phi;
  // left is positive
  std::vector<double> kappa;

  void Clear() {
    segs_lengths.clear();
    seg_list_steer_type.clear();

    gear.clear();
    total_length = 0.0;
    x.clear();
    y.clear();
    phi.clear();
    kappa.clear();
  }
};

struct RSPathPoint {
  double x;
  double y;
  double theta;
  AstarPathGear gear;
};

struct RSPParam {
  bool flag = false;
  double t = 0.0;
  double u = 0.0;
  double v = 0.0;
};

// retired
class ReedShepp {
 public:
  ReedShepp(const VehicleParam& vehicle_param,
            const PlannerOpenSpaceConfig& open_space_conf);

  virtual ~ReedShepp() = default;

  // Pick the shortest path from all possible combination of movement
  // primitives by Reed Shepp
  bool ShortestRSPath(const Pose2D* start_node, const Pose2D* end_node,
                      ReedSheppPath* optimal_path);

 protected:
  // Generate all possible combination of movement primitives by Reed Shepp
  // and interpolate them
  bool GenerateRSPathList(const Pose2D* start_node, const Pose2D* end_node,
                          std::vector<ReedSheppPath>* all_possible_paths);

  // Set the general profile of the movement primitives
  bool GenerateRSP(const Pose2D* start_node, const Pose2D* end_node,
                   std::vector<ReedSheppPath>* all_possible_paths);

  // Set local exact configurations profile of each movement primitive
  bool GenerateGlobalRSPath(const Pose2D* start_node, const Pose2D* end_node,
                            ReedSheppPath* shortest_path);

  // Interpolation usde in GenetateLocalConfiguration
  void Interpolation(const int index, const double path_dist,
                     const AstarPathSteer steer, const Pose2D& base_pose,
                     const double interval_dist,
                     std::vector<RSPathPoint>& rs_path);

  // motion primitives combination setup function
  bool SetRSP(const int size, const double* lengths,
              const AstarPathSteer* types,
              std::vector<ReedSheppPath>* all_possible_paths);

  // Six different combination of motion primitive in Reed Shepp path used in
  // GenerateRSP()
  bool SCS(const double x, const double y, const double phi,
           std::vector<ReedSheppPath>* all_possible_paths);

  bool CSC(const double x, const double y, const double phi,
           std::vector<ReedSheppPath>* all_possible_paths);

  bool CCC(const double x, const double y, const double phi,
           std::vector<ReedSheppPath>* all_possible_paths);

  bool CCCC(const double x, const double y, const double phi,
            std::vector<ReedSheppPath>* all_possible_paths);

  bool CCSC(const double x, const double y, const double phi,
            std::vector<ReedSheppPath>* all_possible_paths);

  bool CCSCC(const double x, const double y, const double phi,
             std::vector<ReedSheppPath>* all_possible_paths);

  // different options for different combination of motion primitives
  void LSL(const double x, const double y, const double phi, RSPParam* param);

  void LSR(const double x, const double y, const double phi, RSPParam* param);

  void LRL(const double x, const double y, const double phi, RSPParam* param);

  void SLS(const double x, const double y, const double phi, RSPParam* param);

  void LRLRn(const double x, const double y, const double phi, RSPParam* param);

  void LRLRp(const double x, const double y, const double phi, RSPParam* param);

  void LRSR(const double x, const double y, const double phi, RSPParam* param);

  void LRSL(const double x, const double y, const double phi, RSPParam* param);

  void LRSLR(const double x, const double y, const double phi, RSPParam* param);

  std::pair<double, double> calc_tau_omega(const double u, const double v,
                                           const double xi, const double eta,
                                           const double phi);

 protected:
  VehicleParam vehicle_param_;
  PlannerOpenSpaceConfig planner_open_space_config_;

  // for scaling
  double max_kappa_;
  double min_radius_;
};

}  // namespace planning
