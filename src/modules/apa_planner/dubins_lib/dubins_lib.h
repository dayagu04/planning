#ifndef __DUBINS_LIB_H__
#define __DUBINS_LIB_H__

#include <sys/types.h>

#include <array>
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
  };

  struct Output {
    bool path_available = false;
    uint8_t gear_change_count = 0;
    std::vector<uint8_t> gear_cmd_vec;
    double length = 0.0;
    geometry_lib::Arc arc_AB;
    geometry_lib::LineSegment line_BC;
    geometry_lib::Arc arc_CD;
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

  const bool DubinsCalculate(DubinsLibrary::GeometryResult& result,
                             const uint8_t dubins_type);

  const bool LineArcCalculate(DubinsLibrary::GeometryResult& result,
                              const uint8_t line_arc_type);

  bool Solve(uint8_t dubins_type, uint8_t case_type);
  bool Solve(uint8_t line_arc_type);
  bool SolveAll();
  void Sampling(double ds);
  const Output GetOutput() const { return output_; }
  const double GetThetaBC() const;
  const double GetThetaD() const;

 private:
  void SetOutputByCaseType(Output& output,
                           const DubinsLibrary::GeometryResult& result,
                           const uint8_t case_type);

  void SetOutputByLineArcType(Output& output,
                              const DubinsLibrary::GeometryResult& result,
                              const uint8_t line_arc_type);

  void GenDubinsOutput(Output& output,
                       const DubinsLibrary::GeometryResult& result);

  Input input_;
  Output output_;
  std::array<Output, DUBINS_TYPE_COUNT * CASE_COUNT> output_arr;
  uint8_t dubins_type_ = 0;
  uint8_t line_arc_type_ = 0;
};
}  // namespace dubins_lib
}  // namespace pnc

#endif