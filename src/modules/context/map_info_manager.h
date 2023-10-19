#ifndef ZNQC_MODULES_CONTEXT_MAP_INFO_MANAGER_H_
#define ZNQC_MODULES_CONTEXT_MAP_INFO_MANAGER_H_

namespace planning {

class MapInfoManager {
 public:
  MapInfoManager();
  ~MapInfoManager() = default;

  double dis_to_ramp() const { return dis_to_ramp_; }
  double distance_to_first_road_merge();
  double distance_to_first_road_split();

 private:
  double dis_to_ramp_;
};
}  // namespace planning

#endif