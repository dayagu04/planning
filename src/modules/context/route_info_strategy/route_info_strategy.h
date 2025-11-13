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

 RampDirection CalculateSplitDirection(
     const iflymapdata::sdpro::LinkInfo_Link& split_link,
     const ad_common::sdpromap::SDProMap& sdpro_map) const;

 RampDirection CalculateMergeDirection(
     const iflymapdata::sdpro::LinkInfo_Link& merge_link,
     const ad_common::sdpromap::SDProMap& sdpro_map) const;

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