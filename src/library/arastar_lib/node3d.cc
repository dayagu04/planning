#include "node3d.h"

namespace planning {

Node3D::Node3D(double x, double y, double phi) {
  x_ = x;
  y_ = y;
  phi_ = phi;
}

Node3D::Node3D(double x, double y, double phi,
               const std::vector<double>& XYbounds, double x_grid_resolution,
               double y_grid_resolution, double phi_grid_resolution) {
  // CHECK_EQ(XYbounds.size(), 4U)
  //     << "XYbounds size is not 4, but" << XYbounds.size();

  x_ = x;
  y_ = y;
  phi_ = phi;

  x_grid_ = static_cast<int>((x_ - XYbounds[0]) / x_grid_resolution);
  y_grid_ = static_cast<int>((y_ - XYbounds[2]) / y_grid_resolution);

  phi_grid_ = static_cast<int>((phi_ - (-M_PI)) / phi_grid_resolution);

  traversed_x_.push_back(x);
  traversed_y_.push_back(y);
  traversed_phi_.push_back(phi);

  index_ = ComputeStringIndex(x_grid_, y_grid_, phi_grid_);
}

Node3D::Node3D(const std::vector<double>& traversed_x,
               const std::vector<double>& traversed_y,
               const std::vector<double>& traversed_phi,
               const std::vector<double>& XYbounds, double x_grid_resolution,
               double y_grid_resolution, double phi_grid_resolution) {
  // CHECK_EQ(XYbounds.size(), 4U)
  //     << "XYbounds size is not 4, but" << XYbounds.size();
  // CHECK_EQ(traversed_x.size(), traversed_y.size());
  // CHECK_EQ(traversed_x.size(), traversed_phi.size());

  x_ = traversed_x.back();
  y_ = traversed_y.back();
  phi_ = traversed_phi.back();

  // XYbounds in xmin, xmax, ymin, ymax
  x_grid_ = static_cast<int>((x_ - XYbounds[0]) / x_grid_resolution);
  y_grid_ = static_cast<int>((y_ - XYbounds[2]) / y_grid_resolution);
  phi_grid_ = static_cast<int>((phi_ - (-M_PI)) / phi_grid_resolution);

  traversed_x_ = traversed_x;
  traversed_y_ = traversed_y;
  traversed_phi_ = traversed_phi;

  index_ = ComputeStringIndex(x_grid_, y_grid_, phi_grid_);
  step_num_ = traversed_x.size();
}

bool Node3D::operator==(const Node3D& right) const {
  return right.GetIndex() == index_;
}

std::string Node3D::ComputeStringIndex(int x_grid, int y_grid, int phi_grid) {
  std::string result;
  result.reserve(16);
  result.append(std::to_string(x_grid));
  result.append("_");
  result.append(std::to_string(y_grid));
  result.append("_");
  result.append(std::to_string(phi_grid));
  return result;
}

}  // namespace planning
