#pragma once

#include <iostream>

#include "config/basic_type.h"
#include "ego_planning_config.h"
#include "hdmap/hdmap.h"
#include "local_view.h"
#include "sdmap/sdmap.h"
#include "sdpromap/sdpromap.h"
#include "session.h"
#include "virtual_lane.h"

namespace planning{
    
// Cost structure for evaluating VirtualLane on-route probability
struct VirtualLaneRouteCost {
  uint order_id = 0;  // VirtualLane order ID. From left to right.

  int relative_id = 0;  // VirtualLane relative ID

  // Lane matching confidence score [0.0, 1.0]
  // Compares perception LaneNumMsg total lane count against
  // feasible_lane_graph's lane_nums at corresponding distances
  double lane_match_confidence = 0.0;

  // Feasible distance reward [0.0, 1.0]
  // Longer feasible distance = higher reward
  double feasible_distance_reward = 0.0;

  // Topology trace score [0.0, 1.0]
  // How many feasible groups exist along the route
  double topo_trace_score = 0.0;

  // Distance penalty [0.0, 1.0]
  // Distance from nearest feasible order (normalized)
  double distance_penalty = 0.0;

  // Total cost (lower is better)
  double total_cost = 1000.0;

  // on link route score
  double on_link_route_score = 0.0;

  // Is on route
  bool is_on_route = false;

  // Weight parameters for cost calculation
  static constexpr double kLaneMatchWeight = 0.5;
  static constexpr double kFeasibleDistanceWeight = 0.0;  // TODO: 后续完善
  static constexpr double kTopoTraceWeight = 0.1;
  static constexpr double kDistancePenaltyWeight = 0.4;

  void CalculateTotalCost() {
    total_cost = kLaneMatchWeight * (1.0 - lane_match_confidence)
               + kFeasibleDistanceWeight * (1.0 - feasible_distance_reward)
               + kTopoTraceWeight * (1.0 - topo_trace_score)
               + kDistancePenaltyWeight * distance_penalty;
  }
};

using VirtualLanesRouteCost = std::vector<VirtualLaneRouteCost>;

class RouteInfoStrategy{
public:
 RouteInfoStrategy(const MLCDeciderConfig*  mlc_decider_config,
                   const planning::framework::Session*  session);

 virtual void Update(RouteInfoOutput& route_info_output) = 0;

 virtual void CalculateMLCDecider(
     const std::vector<std::shared_ptr<VirtualLane>>& relative_id_lanes,
     RouteInfoOutput& route_info_output) = 0;

  virtual bool get_sdpromap_valid() = 0;

  virtual const ad_common::sdpromap::SDProMap& get_sdpro_map() = 0;

  virtual const iflymapdata::sdpro::LinkInfo_Link* get_current_link() = 0;

 RampDirection CalculateSplitDirection(
     const iflymapdata::sdpro::LinkInfo_Link& split_link,
     const ad_common::sdpromap::SDProMap& sdpro_map) const;

 RampDirection CalculateMergeDirection(
     const iflymapdata::sdpro::LinkInfo_Link& merge_link,
     const ad_common::sdpromap::SDProMap& sdpro_map) const;

 virtual VirtualLanesRouteCost GetVirtualLaneCostOnRoute(
     std::vector<std::shared_ptr<VirtualLane>> const& relative_id_lanes) = 0;

 // todo(ldh): 其他virutal函数。
protected:
struct RayInfo {
  char name;
  double angle;
  RayInfo(char n, double a) : name(n), angle(a) {}
};

double CalculateAngle(const Point2D& o, const Point2D& p) const;
std::vector<char> SortRaysByDirection(const std::vector<RayInfo>& rays) const;

static constexpr uint64_t kStaticMapOvertimeThreshold = 20000000;  // 20s（单位：微秒）
static constexpr double kMaxSearchLength = 7000.0;
const planning::framework::Session*  session_;
const MLCDeciderConfig*  mlc_decider_config_;
RouteInfoOutput route_info_output_;

};


}