#include "speed/st_graph_data.h"

namespace planning {

void StGraphData::LoadData(const std::vector<const STBoundary*>& st_boundaries,
                           const double min_s_on_st_boundaries,
                           const PncTrajectoryPoint& init_point,
                           const SpeedLimit& speed_limit,
                           const double path_data_length,
                           const double total_time_by_conf) {
  init_ = true;
  st_boundaries_ = st_boundaries;
  min_s_on_st_boundaries_ = min_s_on_st_boundaries;
  init_point_ = init_point;
  speed_limit_ = speed_limit;
  path_data_length_ = path_data_length;
  total_time_by_conf_ = total_time_by_conf;
}

void StGraphData::load_output_data(
    const BarrierPairList& lane_keep_sv, const BarrierArray& normal_sv,
    const BarrierList& rear_sv, const BarrierList& yield_sv,
    const BarrierList& lane_keep_potential_sv,
    const ObstacleTagList& obstacle_tag_list,
    const IntObstacleTypeUmap& obstacle_first_time_state_map,
    bool dynamic_prebrake_signal, bool static_prebrake_signal,
    bool is_prebrake_target_transverse) {
  lane_keep_sv_ = lane_keep_sv;
  normal_sv_ = normal_sv;
  rear_sv_ = rear_sv;
  yield_sv_ = yield_sv;
  lane_keep_potential_sv_ = lane_keep_potential_sv;
  obstacle_tag_list_ = obstacle_tag_list;
  obstacle_first_time_state_map_ = obstacle_first_time_state_map;
  dynamic_prebrake_signal_ = dynamic_prebrake_signal;
  static_prebrake_signal_ = static_prebrake_signal;
  is_prebrake_target_transverse_ = is_prebrake_target_transverse;
}

void StGraphData::resetBoundary(
    const std::vector<const STBoundary*>& st_boundaries) {
  st_boundaries_ = st_boundaries;
}

const std::vector<const STBoundary*>& StGraphData::st_boundaries() const {
  return st_boundaries_;
}

double StGraphData::min_s_on_st_boundaries() const {
  return min_s_on_st_boundaries_;
}

const PncTrajectoryPoint& StGraphData::init_point() const {
  return init_point_;
}

const SpeedLimit& StGraphData::speed_limit() const { return speed_limit_; }

double StGraphData::path_length() const { return path_data_length_; }

double StGraphData::total_time_by_conf() const { return total_time_by_conf_; }

const StGraphData::BarrierPairList& StGraphData::get_lane_keep_sv() const {
  return lane_keep_sv_;
}

const StGraphData::BarrierArray& StGraphData::get_normal_sv() const {
  return normal_sv_;
}

const StGraphData::BarrierList& StGraphData::get_yield_sv() const {
  return yield_sv_;
}

const StGraphData::BarrierList& StGraphData::get_rear_sv() const {
  return rear_sv_;
}

const StGraphData::BarrierList& StGraphData::get_potential_sv() const {
  return lane_keep_potential_sv_;
}

const StGraphData::ObstacleTagList& StGraphData::get_obstacle_tag() const {
  return obstacle_tag_list_;
}

const StGraphData::IntObstacleTypeUmap&
StGraphData::get_obstacle_first_time_state_map() const {
  return obstacle_first_time_state_map_;
}

bool StGraphData::get_prebrake_signal_from_static() const {
  return static_prebrake_signal_;
}

bool StGraphData::get_prebrake_signal_for_transverse() const {
  return is_prebrake_target_transverse_;
}

bool StGraphData::get_prebrake_signal_from_dynamic() const {
  return dynamic_prebrake_signal_;
}

}  // namespace planning