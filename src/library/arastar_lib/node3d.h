
#pragma once

#include <cmath>
#include <memory>
#include <string>
#include <vector>

namespace planning {

class Node3D {
public:
  Node3D(const double x, const double y, const double phi);
  Node3D(double x, double y, double phi, const std::vector<double>& XYbounds,
         double x_grid_resolution, double y_grid_resolution, double phi_grid_resolution);
  Node3D(const std::vector<double>& traversed_x, const std::vector<double>& traversed_y,
         const std::vector<double>& traversed_phi, const std::vector<double>& XYbounds,
         double x_grid_resolution, double y_grid_resolution, double phi_grid_resolution);
  virtual ~Node3D() = default;

  double GetCost() const {
    return traj_cost_ + heuristic_cost_;
  }
  double GetCost(double epsilon) const {
    return traj_cost_ + epsilon * heuristic_cost_;
  }
  double GetTrajCost() const {
    return traj_cost_;
  }
  double GetHeuCost() const {
    return heuristic_cost_;
  }
  double GetAgentCost() const {
    return agent_cost_;
  }
  double GetBoundaryCost() const {
    return boundary_cost_;
  }
  double GetCenterCost() const {
    return center_cost_;
  }
  double GetMotionCost() const {
    return motion_cost_;
  }
  int GetGridX() const {
    return x_grid_;
  }
  int GetGridY() const {
    return y_grid_;
  }
  int GetGridPhi() const {
    return phi_grid_;
  }
  double GetX() const {
    return x_;
  }
  double GetY() const {
    return y_;
  }
  double GetPhi() const {
    return phi_;
  }
  double GetS() const {
    return s_;
  }
  double GetL() const {
    return l_;
  }
  bool operator==(const Node3D& right) const;
  const std::string& GetIndex() const {
    return index_;
  }
  size_t GetStepNum() const {
    return step_num_;
  }
  double GetSteer() const {
    return steering_;
  }
  std::shared_ptr<Node3D> GetPreNode() const {
    return pre_node_;
  }
  const std::vector<double>& GetXs() const {
    return traversed_x_;
  }
  const std::vector<double>& GetYs() const {
    return traversed_y_;
  }
  const std::vector<double>& GetPhis() const {
    return traversed_phi_;
  }
  void SetPre(const std::shared_ptr<Node3D> pre_node) {
    pre_node_ = pre_node;
  }
  void SetTrajCost(double cost) {
    traj_cost_ = cost;
  }
  void SetHeuCost(double cost) {
    heuristic_cost_ = cost;
  }
  void SetAgentCost(double cost) {
    agent_cost_ = cost;
  }
  void SetBoundaryCost(double cost) {
    boundary_cost_ = cost;
  }
  void SetCenterCost(double cost) {
    center_cost_ = cost;
  }
  void SetMotionCost(double cost) {
    motion_cost_ = cost;
  }
  void SetSteer(double steering) {
    steering_ = steering;
  }
  void SetS(double s){
    s_ = s;
  }
  void SetL(double l){
    l_ = l;
  }
  void SetReachDest(bool reach_dest) {
    reach_dest_ = reach_dest;
  }
  bool ReachDest() const {
    return reach_dest_;
  }

private:
  static std::string ComputeStringIndex(int x_grid, int y_grid, int phi_grid);

private:
  double x_ = 0.0;
  double y_ = 0.0;
  double phi_ = 0.0;
  double s_ = 0.0;
  double l_ = 0.0;
  size_t step_num_ = 1;
  std::vector<double> traversed_x_;
  std::vector<double> traversed_y_;
  std::vector<double> traversed_phi_;
  int x_grid_ = 0;
  int y_grid_ = 0;
  int phi_grid_ = 0;
  std::string index_;
  double traj_cost_ = 0.0;
  double heuristic_cost_ = 0.0;
  std::shared_ptr<Node3D> pre_node_ = nullptr;
  double steering_ = 0.0;
  bool reach_dest_ = false;
  double agent_cost_ = 0.0;
  double boundary_cost_ = 0.0;
  double center_cost_ = 0.0;
  double motion_cost_ = 0.0;
};

} // namespace planning
