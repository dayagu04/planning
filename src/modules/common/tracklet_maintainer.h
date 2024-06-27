#ifndef PLANNING_PLANNER_TRACKLET_MAINTAINER_H_
#define PLANNING_PLANNER_TRACKLET_MAINTAINER_H_

#include <map>
#include <set>

#include "ego_state_manager.h"
#include "fusion_objects_c.h"
#include "prediction_object.h"
#include "refline.h"
#include "session.h"
#include "tracked_object.h"
#include "utils/kd_path.h"
#include "utils/lateral_utils.h"
#include "utils_math.h"

namespace planning {

struct TrackletSequentialState {
  double v_lat_deriv;
  double target_left_line_filter_count;
  double target_right_line_filter_count;
};

// double calc_poly1d(const std::vector<double> &coefs, double x);
// double get_dist(double x, double y, const std::vector<double> &y_x);

class LifecycleDict {
 public:
  LifecycleDict() {}
  virtual ~LifecycleDict() = default;

  TrackletSequentialState *get(int uid);

  bool set(int uid, const TrackletSequentialState &state);

  void mark_dirty(int uid);

  void remove_clean();

 private:
  std::map<int, TrackletSequentialState> data_dict_;
  std::set<int> dirty_set_;
};

class TrackletMaintainer {
 public:
  TrackletMaintainer(planning::framework::Session *session);
  virtual ~TrackletMaintainer();

  void apply_update(const std::shared_ptr<EgoStateManager> ego_state,
                    const std::vector<PredictionObject> &predictions,
                    std::vector<TrackedObject> &tracked_objects,
                    LeadCars &lead_cars, bool isRedLightStop, bool hdmap_valid);

 private:
  void recv_prediction_objects(const std::vector<PredictionObject> &predictions,
                               std::vector<TrackedObject *> &objects);
  void recv_relative_prediction_objects(
      const std::vector<PredictionObject> &predictions,
      std::vector<TrackedObject *> &objects);

  void fisheye_helper(const PredictionObject &prediction,
                      TrackedObject &object);

  void calc(std::vector<TrackedObject *> &tracked_objects, int scenario,
            double lane_width, double lat_offset, bool borrow_bicycle_lane,
            bool enable_intersection_planner, double dist_rblane,
            bool tleft_lane, bool rightest_lane, double dist_intersect,
            double intersect_length, bool left_faster, bool right_faster,
            LeadCars &lead_cars, bool isRedLightStop, bool isFasterStaticAvd,
            bool isOnHighway, const std::vector<double> &d_poly,
            const std::vector<double> &c_poly);

  bool fill_info_with_refline(TrackedObject &item, double lat_offset);

  void fill_deriv_info(TrackedObject &item);

  void calc_intersection_with_refline(TrackedObject &item,
                                      bool enable_intersection_planner);

  double calc_ignorance_threshold(TrackedObject &item, int idx, int sgn,
                                  bool enable_intersection_planner);

  void check_accident_car(TrackedObject &item, double v_ego, int scenario,
                          double dist_intersect, double intersect_length,
                          bool left_faster, bool right_faster,
                          bool isRedLightStop, bool isFasterStaticAvd,
                          bool isOnHighway);

  void check_prebrk_object(TrackedObject &item, double v_ego,
                           double lane_width);

  bool is_potential_lead_one(TrackedObject &item, double v_ego);

  bool is_potential_lead_two(TrackedObject &item, TrackedObject *lead_one);

  bool is_potential_temp_lead_one(TrackedObject &item, double v_ego,
                                  bool refline_update);

  bool is_potential_temp_lead_two(TrackedObject &item,
                                  TrackedObject *temp_lead_one);

  bool is_potential_avoiding_car(TrackedObject &item, TrackedObject *lead_one,
                                 TrackedObject *lead_two, double v_ego,
                                 double lane_width, int scenario,
                                 bool borrow_bicycle_lane, double dist_rblane,
                                 bool tleft_lane, bool rightest_lane,
                                 double dist_intersect, double intersect_length,
                                 bool isRedLightStop, double farthest_distance);

  bool is_leadone_potential_avoiding_car(TrackedObject *lead_one, int scenario,
                                         double lane_width,
                                         bool borrow_bicycle_lane,
                                         bool rightest_lane,
                                         double dist_intersect,
                                         bool isRedLightStop);

  void select_lead_cars(const std::vector<TrackedObject *> &tracked_objects,
                        LeadCars &lead_cars);

  void set_default_value(const std::vector<TrackedObject *> &tracked_objects);

  void obstacle_reset(TrackedObject &item, bool frenet_transform_valid);

  bool is_oversize_vehicle(const int type);

  bool is_VRU(const int type);

  bool is_traffic_facilities(const int type);

  bool is_car(const int type);

  planning::framework::Session *session_ = nullptr;
  std::shared_ptr<planning_math::KDPath> frenet_coord_;
  LifecycleDict seq_state_;
  std::map<int, TrackedObject *> object_map_;
  std::map<int, TrackedObject *> fusion_object_history_map_;
  std::map<int, TrackedObject *> lt_fusion_object_history_map_;
  std::shared_ptr<EgoStateManager> ego_state_;
  bool hdmap_valid_{false};

  double s_ego_;
  double l_ego_;
  double theta_ego_;
  double vl_ego_;
  double vs_ego_;
};

}  // namespace planning

#endif
