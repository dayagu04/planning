#ifndef LATERAL_OBSTACLE_H
#define LATERAL_OBSTACLE_H

#include "config/basic_type.h"
#include "ego_planning_config.h"
#include "prediction_object.h"
#include "tracked_object.h"
#include "tracklet_maintainer.h"
#include "task_basic_types.h"

#include <cstdint>
#include <utility>

namespace planning {

class EgoStateManager;
class VirtualLaneManager;

class LateralObstacle {
 public:
  LateralObstacle(const EgoPlanningConfigBuilder *config_builder,
                  planning::framework::Session *session);
  virtual ~LateralObstacle();

  bool update();

  bool fvf_dead() const { return fvf_dead_; }
  bool svf_dead() const { return svf_dead_; }
  bool sensors_okay() const { return (!fvf_dead_ && !svf_dead_); }

  TrackedObject *leadone() { return lead_cars_.lead_one; }
  TrackedObject *leadtwo() { return lead_cars_.lead_two; }
  TrackedObject *tleadone() { return lead_cars_.temp_lead_one; }
  TrackedObject *tleadtwo() { return lead_cars_.temp_lead_two; }

  LeadCars &get_lead_cars() { return lead_cars_; }

  void set_prediction_update(bool value) { prediction_update_ = value; }

  const std::vector<TrackedObject> &front_tracks() const {
    return front_tracks_;
  }

  const std::vector<TrackedObject> &front_tracks_copy() const {
    return front_tracks_copy_;
  }

  const std::vector<TrackedObject> &side_tracks() const { return side_tracks_; }

  const std::vector<TrackedObject> &front_tracks_l() const {
    return front_tracks_l_;
  }

  const std::vector<TrackedObject> &front_tracks_r() const {
    return front_tracks_r_;
  }

  const std::vector<TrackedObject> &side_tracks_l() const {
    return side_tracks_l_;
  }

  const std::vector<TrackedObject> &side_tracks_r() const {
    return side_tracks_r_;
  }

  const std::vector<TrackedObject> &all_tracks() const { return all_tracks_; }

  bool find_track(int track_id, TrackedObject &dest);

  const std::unordered_map<uint16_t, LatObstacleDecisionType>& lat_obstacle_decision() { return lat_obstacle_decision_; }
 private:
  bool update_sensors(const std::shared_ptr<EgoStateManager> &ego_state,
                      const std::vector<PredictionObject> &predictions,
                      bool isRedLightStop, bool hdmap_valid);
  void update_tracks(const std::vector<TrackedObject> &tracked_objects);
  void LateralObstacleDecision(
      const std::vector<TrackedObject> &tracked_objects);

  double fvf_time_ = 0.0;
  bool fvf_dead_ = true;
  double svf_time_ = 0.0;
  bool svf_dead_ = true;
  double warning_timer_[5];
  bool prediction_update_ = false;

  std::vector<TrackedObject> front_tracks_;
  std::vector<TrackedObject> front_tracks_copy_;
  std::vector<TrackedObject> front_tracks_l_;
  std::vector<TrackedObject> front_tracks_r_;
  std::vector<TrackedObject> side_tracks_;
  std::vector<TrackedObject> side_tracks_l_;
  std::vector<TrackedObject> side_tracks_r_;
  std::vector<TrackedObject> all_tracks_;
  LeadCars lead_cars_;
  std::shared_ptr<planning::TrackletMaintainer> maintainer_ = nullptr;
  planning::framework::Session *session_ = nullptr;
  LateralObstacleConfig config_;
  std::unordered_map<uint16_t, LatObstacleDecisionType> lat_obstacle_decision_;
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
  const std::vector<TrackedObject> &front_tracks_l() const {
    return front_tracks_llane_;
  }
  const std::vector<TrackedObject> &front_tracks_r() const {
    return front_tracks_rlane_;
  }
  const std::vector<TrackedObject> &front_tracks_c() const {
    return front_tracks_clane_;
  }
  const std::vector<TrackedObject> &get_front_tracks() const {
    return lateral_obstacle_.front_tracks();
  }
  const std::vector<TrackedObject> &get_side_tracks() const {
    return lateral_obstacle_.side_tracks();
  }
  const std::vector<TrackedObject> &side_tracks_l() const {
    return side_tracks_llane_;
  }
  const std::vector<TrackedObject> &side_tracks_r() const {
    return side_tracks_rlane_;
  }
  const std::vector<TrackedObject> &side_tracks_c() const {
    return side_tracks_clane_;
  }
  std::vector<TrackedObject> *get_lane_tracks(int lane, TrackType track_type);

  // std::pair<int, double> get_vavg_poi_int(int side, double bound, double
  // poi_back_limit, double poi_front_limit); std::pair<int, pair<double,
  // double>> get_vavg_poi(int side, double, double); bool front_cone_exist();

  void update() { lane_tracks_update_.clear(); }

 private:
  void reset();

  std::set<std::tuple<int, int>> lane_tracks_update_;
  std::vector<TrackedObject> front_tracks_clane_;
  std::vector<TrackedObject> front_tracks_llane_;
  std::vector<TrackedObject> front_tracks_rlane_;
  std::vector<TrackedObject> side_tracks_clane_;
  std::vector<TrackedObject> side_tracks_llane_;
  std::vector<TrackedObject> side_tracks_rlane_;
  std::vector<TrackedObject> empty_tracks_;

  LateralObstacle &lateral_obstacle_;
  VirtualLaneManager &virtual_lane_mgr_;
  planning::framework::Session *session_ = nullptr;
  // EgoStateManager ego_state_;
};

}  // namespace planning

#endif
