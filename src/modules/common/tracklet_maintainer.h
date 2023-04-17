#ifndef PLANNING_PLANNER_TRACKLET_MAINTAINER_H_
#define PLANNING_PLANNER_TRACKLET_MAINTAINER_H_

// #include "common/ego_state_manager.h"
// #include "common/map_info_manager.h"

#include <map>
#include <set>

#include "src/modules/common/prediction_object.h"
#include "src/modules/common/refline.h"
#include "src/modules/context/ego_state_manager.h"
// #include "src/modules/common/define/path_point.h"
#include "src/modules/common/tracked_object.h"
#include "src/modules/common/utils/lateral_utils.h"
#include "framework/session.h"
#include "res/include/proto/fusion_objects.pb.h"

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

class SimpleRefLine {
public:
  SimpleRefLine();
  virtual ~SimpleRefLine() = default;

  void update_pathpoints(const std::vector<PathPoint> &path_points);

  void cartesian_frenet(double x, double y, double &s, double &l, double &v_s,
                        double &v_l, double &theta, bool get_theta = false,
                        double *v = nullptr, double *yaw = nullptr);

  void frenet_cartesian(double s, double l, double &x, double &y);

  bool has_update() const { return update_; }

private:
  std::vector<PathPoint> path_points_;
  bool update_;
};

class TrackletMaintainer {
public:
  TrackletMaintainer(planning::framework::Session *session);
  virtual ~TrackletMaintainer();

  void apply_update(const EgoStateManager &ego_state,
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

  void calc(std::vector<TrackedObject *> &tracked_objects,
            const std::vector<PathPoint> &path_points, int scenario,
            double lane_width, double lat_offset, bool borrow_bicycle_lane,
            bool enable_intersection_planner, double dist_rblane,
            bool tleft_lane, bool rightest_lane, double dist_intersect,
            double intersect_length, bool left_faster, bool right_faster,
            LeadCars &lead_cars, bool isRedLightStop, bool isFasterStaticAvd,
            bool isOnHighway, std::vector<double> d_poly,
            std::vector<double> c_poly);

  void fill_info_with_refline(TrackedObject &item,
                              SimpleRefLine &simple_refline, double lat_offset);

  void fill_deriv_info(TrackedObject &item);

  void fill_possibility_of_cutin(TrackedObject &item);

  void calc_intersection_with_refline(TrackedObject &item,
                                      bool enable_intersection_planner,
                                      SimpleRefLine &simple_refline);

  double calc_ignorance_threshold(TrackedObject &item, int idx, int sgn,
                                  bool enable_intersection_planner);

  void check_accident_car(TrackedObject &item, double v_ego, int scenario,
                          double dist_intersect, double intersect_length,
                          bool left_faster, bool right_faster,
                          bool isRedLightStop, bool isFasterStaticAvd, bool isOnHighway);

  void check_prebrk_object(TrackedObject &item, double v_ego,
                           double lane_width);

  bool is_potential_lead_one(TrackedObject &item, double v_ego);

  bool is_potential_lead_two(TrackedObject &item, TrackedObject *lead_one);

  bool is_potential_temp_lead_one(TrackedObject &item, double v_ego,
                                  bool refline_update);

  bool is_potential_temp_lead_two(TrackedObject &item,
                                  TrackedObject *temp_lead_one);

  bool is_potential_avoiding_car(TrackedObject &item, TrackedObject *lead_one, double v_ego,
                                 double lane_width, int scenario,
                                 bool borrow_bicycle_lane, double dist_rblane,
                                 bool tleft_lane, bool rightest_lane, double dist_intersect, double intersect_length, bool isRedLightStop);

  bool is_leadone_potential_avoiding_car(TrackedObject *lead_one, int scenario,
                                         double lane_width,
                                         bool borrow_bicycle_lane,
                                         bool rightest_lane, double dist_intersect, bool isRedLightStop);

  void select_lead_cars(const std::vector<TrackedObject *> &tracked_objects,
                        LeadCars &lead_cars);

  void set_default_value(const std::vector<TrackedObject *> &tracked_objects);

  planning::framework::Session *session_ = nullptr;
  LifecycleDict seq_state_;
  SimpleRefLine simple_refline_;
  std::map<int, TrackedObject *> object_map_;
  // EgoStateManager ego_state_;
  bool hdmap_valid_{true};

  double s0_;
  double l0_;
  double theta_ego_;
  double vl_ego_;
  double vs_ego_;
};

} // namespace planning

#endif
