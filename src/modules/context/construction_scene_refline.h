#pragma once

#include <vector>

#include "vec2d.h"
#include "construction_scene_manager.h"
#include "lane_reference_path.h"

namespace planning {

class ConstructionSceneRefline {
 public:
  ConstructionSceneRefline();

  virtual ~ConstructionSceneRefline() = default;

  bool InitInfo();

  bool Update(
      const std::shared_ptr<LaneReferencePath>& reference_path,
      const std::map<int, RoadBoundaryCluster>& road_boundaries_clusters_map,
      const std::map<int, ConstructionAgentClusterArea>& construction_agent_cluster_attribute_set);

  ReferencePathPoints& MutableConstructionRefPathPoints() {
    return construction_ref_path_;
  };

  const ReferencePathPoints& GetConstructionRefPathPoints() const {
    return construction_ref_path_;
  };

 private:
  bool ExtractRoadBoundaries(
      const std::shared_ptr<planning::planning_math::KDPath>& frenet_coord,
      const std::map<int, RoadBoundaryCluster>& road_boundary_map,
      std::map<planning::ConstructionDirection, std::map<double, double>>& boundaries);

  bool ExtractAgentBoundaries(
      const std::shared_ptr<planning::LaneReferencePath>& reference_path,
      const std::map<int, ConstructionAgentClusterArea>& agent_map,
      std::map<planning::ConstructionDirection, std::map<double, double>>& boundaries);

  bool GeneratePassableBoundary(
      const std::map<planning::ConstructionDirection, std::map<double, double>>& boundaries);

  bool GenerateCenterLines(
      const std::shared_ptr<planning::LaneReferencePath>& reference_path,
    std::vector<Point2d>& frenet_refline);

  bool GenerateConstructionRefLine(
      const std::shared_ptr<planning::LaneReferencePath>& reference_path,
      const std::vector<Point2d>& frenet_refline);

 private:
  int lane_num_;
  double min_lane_width_;
  pnc::mathlib::spline left_boundary_spline_;        // s, l
  pnc::mathlib::spline right_boundary_spline_;       // s, l
  std::map<int, std::vector<Point2d>> center_line_;  // line_id, s, l
  ReferencePathPoints construction_ref_path_;
};

}  // namespace planning