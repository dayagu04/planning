#ifndef __DUBINS_LIB_H__
#define __DUBINS_LIB_H__

#include <sys/types.h>

#include <array>
#include <cstddef>
#include <cstdint>
#include <utility>
#include <vector>

#include "Eigen/Core"
#include "geometry_math.h"

namespace pnc {

namespace dubins_lib {
class DubinsLibrary {
 public:
  enum DubinsType {
    L_S_R,
    R_S_L,
    L_S_L,
    R_S_R,
    DUBINS_TYPE_COUNT,
  };

  enum LineArcType {
    S_L,
    S_R,
    L_S,
    R_S,
    LINEARC_TYPE_COUNT,
  };

  enum CaseType {
    CASE_A,
    CASE_B,
    CASE_COUNT,
  };

  enum GearType {
    EMPTY,
    NORMAL,
    REVERSE,
  };
  struct Input {
    Eigen::Vector2d p1 = Eigen::Vector2d::Zero();
    Eigen::Vector2d p2 = Eigen::Vector2d::Zero();
    double heading1 = 0.0;
    double heading2 = 0.0;
    double radius = 0.0;

    void Set(const Eigen::Vector2d& p_start, const Eigen::Vector2d& p_target,
             const double heading_start, const double heading_target) {
      p1 = p_start;
      p2 = p_target;
      heading1 = heading_start;
      heading2 = heading_target;
    }
  };

  struct PathPoint {
    void Set(const Eigen::Vector2d& pos_in, const double heading_in) {
      pos = pos_in, heading = heading_in;
    }

    Eigen::Vector2d pos = Eigen::Vector2d::Zero();
    double heading = 0.0;
  };

  struct Output {
    bool path_available = false;
    bool is_line_arc = false;
    double dtheta_arc_AB = 0.0;
    uint8_t dubins_type = 0;
    uint8_t line_arc_type = 0;
    uint8_t current_gear_cmd = EMPTY;
    uint8_t gear_change_count = 0;
    uint8_t gear_change_index = 0;
    uint8_t path_seg_count = 0;
    std::vector<uint8_t> gear_cmd_vec = {0, 0, 0};
    double length = 0.0;
    geometry_lib::Arc arc_AB;
    geometry_lib::LineSegment line_BC;
    geometry_lib::Arc arc_CD;
    std::vector<PathPoint> path_point_vec;
  };

  struct GeometryResult {
    bool is_line_arc = false;
    uint8_t dubins_type = 0;
    uint8_t line_arc_type = 0;

    geometry_lib::Circle c1;
    geometry_lib::Circle c2;
    geometry_lib::TangentOutput tangent_result;
    Eigen::Vector2d t1;
    Eigen::Vector2d t2;
    Eigen::Vector2d n1;
    Eigen::Vector2d n2;

    double l = 0.0;
    double r = 0.0;
  };

 public:
  void SetInput(Input& input) { input_ = input; }

  void SetStart(const Eigen::Vector2d& p_start, const double heading_start) {
    input_.p1 = p_start;
    input_.heading1 = heading_start;
  }

  void SetTarget(const Eigen::Vector2d& p_target, const double heading_target) {
    input_.p2 = p_target;
    input_.heading2 = heading_target;
  }

  void SetRadius(const double radius) { input_.radius = radius; }

  // solve by dubins
  bool Solve(uint8_t dubins_type, uint8_t case_type);

  // solve by line arc
  bool Solve(uint8_t line_arc_type);

  void Sampling(const double ds, const bool is_complete_path);
  void Extend(const double extend_s);
  void Transform(const geometry_lib::LocalToGlobalTf& l2g_tf);

  void SetOutput(const Output& Output) { output_ = Output; }
  const Output& GetOutput() const { return output_; }
  const Output* GetOutputPtr() const { return &output_; }
  const double GetThetaBC() const;
  const double GetThetaD() const;
  const std::vector<double> GetPathEle(size_t index) const;

  const std::vector<PathPoint>& GetPathPointVec() const {
    return output_.path_point_vec;
  }

 private:
  void SetOutputByCaseType(Output& output,
                           const DubinsLibrary::GeometryResult& result,
                           const uint8_t case_type);

  void DubinsCalculate(DubinsLibrary::GeometryResult& result,
                       const uint8_t dubins_type);

  void LineArcCalculate(DubinsLibrary::GeometryResult& result,
                        const uint8_t line_arc_type);

  void SetOutputByLineArcType(Output& output,
                              const DubinsLibrary::GeometryResult& result,
                              const uint8_t line_arc_type);

  const bool GenDubinsOutput(Output& output,
                             const DubinsLibrary::GeometryResult& result);

  Input input_;
  Output output_;
  uint8_t dubins_type_ = 0;
  uint8_t line_arc_type_ = 0;
};
}  // namespace dubins_lib
}  // namespace pnc

#endif