#ifndef SCENARIO_MANAGER_OBJECT_SELECTOR_H_
#define SCENARIO_MANAGER_OBJECT_SELECTOR_H_

#include "config/basic_type.h"
#include "ego_planning_config.h"
#include "ego_state_manager.h"
#include "ifly_time.h"
#include "lateral_obstacle.h"
#include "obstacle_manager.h"
#include "reference_path_manager.h"
#include "session.h"
#include "virtual_lane_manager.h"

namespace planning {

class ObjectSelector {
 public:
  ObjectSelector(const EgoPlanningConfigBuilder *config_builder,
                 framework::Session *session);

  virtual ~ObjectSelector() = default;

  bool in_alc_range();

  bool in_alc_status(int status, double start_move_distolane);

  bool check_map_alc_enable(int direction, bool accident_ahead);

  bool update(int status, double start_move_distolane, bool accident_ahead,
              double perception_range, bool disable_l, bool disable_r,
              bool upstream_enable_l, bool upstream_enable_r,
              bool upstream_enable_lb, int upstream_enable_id);

  bool enable_l() const { return enable_l_; }
  bool enable_r() const { return enable_r_; }

  double v_rel_l() const { return v_rel_l_; }
  double v_rel_r() const { return v_rel_r_; }
  double v_rel_f() const { return v_rel_f_; }

  double d_stop_l() const { return d_stop_l_; }
  double d_stop_r() const { return d_stop_r_; }

  double d_lb_car_l() const { return d_lb_car_l_; }
  double d_lb_car_r() const { return d_lb_car_r_; }

  double t_surpass_l() const { return t_surpass_l_; }
  double t_surpass_r() const { return t_surpass_r_; }

  bool left_is_faster() const { return left_is_faster_; }
  bool right_is_faster() const { return right_is_faster_; }

  double premove_dist() const { return premove_dist_; }
  bool premovel() const { return premovel_; }
  bool premover() const { return premover_; }
  int premoved_id() const { return premoved_id_; }
  int neg_premoved_id() const { return neg_premoved_id_; }

  int left_is_faster_cnt() const { return left_is_faster_cnt_; }
  int right_is_faster_cnt() const { return right_is_faster_cnt_; }
  int front_tracks_l_cnt() const { return front_tracks_l_cnt_; }
  int front_tracks_r_cnt() const { return front_tracks_r_cnt_; }

  bool neg_left_lb_car() const { return neg_left_lb_car_; }
  bool neg_right_lb_car() const { return neg_right_lb_car_; }
  bool neg_left_alc_car() const { return neg_left_alc_car_; }
  bool neg_right_alc_car() const { return neg_right_alc_car_; }
  bool jam_cancel() const { return jam_cancel_; }

  std::vector<int> &left_lb_car() { return left_lb_car_; }
  std::vector<int> &left_alc_car() { return left_alc_car_; }
  std::vector<int> &right_lb_car() { return right_lb_car_; }
  std::vector<int> &right_alc_car() { return right_alc_car_; }

  std::map<int, CarCount> &left_lb_car_cnt() { return left_lb_car_cnt_; }

  std::map<int, CarCount> &left_alc_car_cnt() { return left_alc_car_cnt_; }

  std::map<int, CarCount> &right_lb_car_cnt() { return right_lb_car_cnt_; }

  std::map<int, CarCount> &right_alc_car_cnt() { return right_alc_car_cnt_; }

  const std::vector<int> &left_close_objs() const { return left_close_objs_; }
  const std::vector<int> &right_close_objs() const { return right_close_objs_; }
  const std::vector<int> &current_close_objs() const {
    return current_close_objs_;
  }

  double get_vrel_close(int side, int status);

  // void restore_context(const ObjectSelectorContext &context);
  // void save_context(ObjectSelectorContext &context) const;

 private:
  planning::framework::Session *session_ = nullptr;
  EgoPlanningObjectSelectorManagerConfig config_;
  bool enable_l_ = true;
  bool enable_r_ = true;

  double v_rel_l_ = 0.0;
  double v_rel_r_ = 0.0;
  double v_rel_f_ = 15.0;
  double d_stop_l_ = 0;
  double d_stop_r_ = 0;
  double d_lb_car_l_ = 0;
  double d_lb_car_r_ = 0;
  double t_surpass_l_ = -1000;
  double t_surpass_r_ = -1000;
  double premove_dist_ = 0.0;

  bool neg_left_lb_car_ = false;
  bool neg_right_lb_car_ = false;
  bool neg_left_alc_car_ = false;
  bool neg_right_alc_car_ = false;
  bool jam_cancel_ = false;

  bool left_is_faster_ = false;
  bool premovel_ = false;
  bool premover_ = false;
  bool right_is_faster_ = false;
  int left_is_faster_cnt_ = 0;
  int right_is_faster_cnt_ = 0;
  int premoved_id_ = -1000;
  int neg_premoved_id_ = -1000;

  int l_accident_cnt_ = 0;
  int r_accident_cnt_ = 0;
  int front_tracks_l_cnt_ = 0;
  int front_tracks_r_cnt_ = 0;

  std::vector<int> left_lb_car_;
  std::vector<int> left_alc_car_;
  std::vector<int> right_lb_car_;
  std::vector<int> right_alc_car_;

  std::map<int, CarCount> left_lb_car_cnt_;
  std::map<int, CarCount> left_alc_car_cnt_;
  std::map<int, CarCount> right_lb_car_cnt_;
  std::map<int, CarCount> right_alc_car_cnt_;

  std::vector<int> left_close_objs_;
  std::vector<int> right_close_objs_;
  std::vector<int> current_close_objs_;

  // TrackedObject *tleadtwo_ = nullptr;
  // EgoState ego_state_;
  // MapInfoManager &map_info_mgr_;
  // LateralObstacle &lateral_obstacle_;
  // VirtualLaneManager &virtual_lane_mgr_;
  // LaneTracksManager &lane_tracks_mgr_;
};

}  // namespace planning

#endif
