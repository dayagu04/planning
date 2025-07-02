#ifndef LATERAL_OBSTACLE_H
#define LATERAL_OBSTACLE_H

#include "config/basic_type.h"
#include "ego_planning_config.h"
#include "prediction_object.h"
#include "task_basic_types.h"
#include "tracked_object.h"
#include "tracklet_maintainer.h"
#include "frenet_obstacle.h"
#include <cstdint>
#include <utility>

namespace planning {

class EgoStateManager;
class VirtualLaneManager;

struct ExtraObstacleInfo {
  double timestamp = 0;  // 单位：s
  double last_recv_time = 0;
  double lat_coeff = 1.0;
  double leadone_confidence_cnt = 0;
  bool oncoming = false;
  bool is_lead = false;
  double last_ttc = std::numeric_limits<double>::min();
  double cutin_confidence_cnt = 0;
  double cutinp = 0.0;
};

class LateralObstacle {
 public:
  LateralObstacle(const EgoPlanningConfigBuilder *config_builder,
                  planning::framework::Session *session);
  virtual ~LateralObstacle();

  void SetConfig(const EgoPlanningConfigBuilder *config_builder);

  bool update();

  bool fvf_dead() const { return fvf_dead_; }
  bool svf_dead() const { return svf_dead_; }
  bool sensors_okay() const { return (!fvf_dead_ && !svf_dead_); }

  const std::shared_ptr<FrenetObstacle> leadone() { return lead_one_; }
  // const std::shared_ptr<FrenetObstacle> leadtwo() { return nullptr; }
  // const std::shared_ptr<FrenetObstacle> tleadone() { return nullptr; }
  // const std::shared_ptr<FrenetObstacle> tleadtwo() { return nullptr; }

  // LeadCars &get_lead_cars() { return lead_cars_; }

  void set_prediction_update(bool value) { prediction_update_ = value; }

  const std::vector<std::shared_ptr<FrenetObstacle>> &front_tracks() const {
    return front_tracks_;
  }

  const std::vector<std::shared_ptr<FrenetObstacle>> &front_tracks_copy() const {
    return front_tracks_copy_;
  }

  const std::vector<std::shared_ptr<FrenetObstacle>> &side_tracks() const { return side_tracks_; }

  const std::vector<std::shared_ptr<FrenetObstacle>> &front_tracks_l() const {
    return front_tracks_l_;
  }

  const std::vector<std::shared_ptr<FrenetObstacle>> &front_tracks_r() const {
    return front_tracks_r_;
  }

  const std::vector<std::shared_ptr<FrenetObstacle>> &side_tracks_l() const {
    return side_tracks_l_;
  }

  const std::vector<std::shared_ptr<FrenetObstacle>> &side_tracks_r() const {
    return side_tracks_r_;
  }

  // const std::vector<std::shared_ptr<FrenetObstacle>> &all_tracks() const { return all_tracks_; }

  const std::unordered_map<int, std::shared_ptr<FrenetObstacle>> &tracks_map() const { return tracks_map_; }

  bool find_track(int track_id, std::shared_ptr<FrenetObstacle> &dest);

  const std::unordered_map<int, ExtraObstacleInfo> &extra_obstacle_info_map() const { return extra_obstacle_info_map_; }

 private:
  bool update_sensors(const std::shared_ptr<EgoStateManager> &ego_state,
                      const std::vector<PredictionObject> &predictions,
                      bool isRedLightStop, bool hdmap_valid);
  void update_tracks();
  void LateralObstacleDecision(
      const std::vector<std::shared_ptr<FrenetObstacle>> &tracked_objects);
  void update_lead_info();
  void select_lead_cars();

  double fvf_time_ = 0.0;
  bool fvf_dead_ = true;
  double svf_time_ = 0.0;
  bool svf_dead_ = true;
  double warning_timer_[5];
  bool prediction_update_ = false;
  bool is_static_avoid_scene_ = false;

  std::shared_ptr<FrenetObstacle> lead_one_;
  std::vector<std::shared_ptr<FrenetObstacle>> front_tracks_;
  std::vector<std::shared_ptr<FrenetObstacle>> front_tracks_copy_;
  std::vector<std::shared_ptr<FrenetObstacle>> front_tracks_l_;
  std::vector<std::shared_ptr<FrenetObstacle>> front_tracks_r_;
  std::vector<std::shared_ptr<FrenetObstacle>> side_tracks_;
  std::vector<std::shared_ptr<FrenetObstacle>> side_tracks_l_;
  std::vector<std::shared_ptr<FrenetObstacle>> side_tracks_r_;
  // std::vector<std::shared_ptr<FrenetObstacle>> all_tracks_;
  std::unordered_map<int, std::shared_ptr<FrenetObstacle>> tracks_map_;
  // LeadCars lead_cars_;

  // std::shared_ptr<planning::TrackletMaintainer> maintainer_ = nullptr;
  planning::framework::Session *session_ = nullptr;
  LateralObstacleConfig config_;
  std::unordered_map<uint16_t, LatObstacleDecisionType> lat_obstacle_decision_;
  std::unordered_map<int, ExtraObstacleInfo> extra_obstacle_info_map_;
};


class LaneTracksManager {
 public:
  LaneTracksManager(LateralObstacle &lateral_obstacle,
                    VirtualLaneManager &virtual_lane_mgr,
                    planning::framework::Session *session);
  virtual ~LaneTracksManager() = default;
  void update_lane_tracks();

  double get_drel_close(int side);
  double get_vrel_close(int side, int status);
  const std::vector<std::shared_ptr<FrenetObstacle>> &front_tracks_l() const {
    return front_tracks_llane_;
  }
  const std::vector<std::shared_ptr<FrenetObstacle>> &front_tracks_r() const {
    return front_tracks_rlane_;
  }
  const std::vector<std::shared_ptr<FrenetObstacle>> &front_tracks_c() const {
    return front_tracks_clane_;
  }
  const std::vector<std::shared_ptr<FrenetObstacle>> &get_front_tracks() const {
    return lateral_obstacle_.front_tracks();
  }
  const std::vector<std::shared_ptr<FrenetObstacle>> &get_side_tracks() const {
    return lateral_obstacle_.side_tracks();
  }
  const std::vector<std::shared_ptr<FrenetObstacle>> &side_tracks_l() const {
    return side_tracks_llane_;
  }
  const std::vector<std::shared_ptr<FrenetObstacle>> &side_tracks_r() const {
    return side_tracks_rlane_;
  }
  const std::vector<std::shared_ptr<FrenetObstacle>> &side_tracks_c() const {
    return side_tracks_clane_;
  }
  std::vector<std::shared_ptr<FrenetObstacle>> *get_lane_tracks(int lane, TrackType track_type);

  // std::pair<int, double> get_vavg_poi_int(int side, double bound, double
  // poi_back_limit, double poi_front_limit); std::pair<int, pair<double,
  // double>> get_vavg_poi(int side, double, double); bool front_cone_exist();

  void update() { lane_tracks_update_.clear(); }

 private:
  void reset();

  std::set<std::tuple<int, int>> lane_tracks_update_;
  std::vector<std::shared_ptr<FrenetObstacle>> front_tracks_clane_;
  std::vector<std::shared_ptr<FrenetObstacle>> front_tracks_llane_;
  std::vector<std::shared_ptr<FrenetObstacle>> front_tracks_rlane_;
  std::vector<std::shared_ptr<FrenetObstacle>> side_tracks_clane_;
  std::vector<std::shared_ptr<FrenetObstacle>> side_tracks_llane_;
  std::vector<std::shared_ptr<FrenetObstacle>> side_tracks_rlane_;
  std::vector<std::shared_ptr<FrenetObstacle>> empty_tracks_;

  LateralObstacle &lateral_obstacle_;
  VirtualLaneManager &virtual_lane_mgr_;
  planning::framework::Session *session_ = nullptr;
  // EgoStateManager ego_state_;
};

}  // namespace planning

#endif
