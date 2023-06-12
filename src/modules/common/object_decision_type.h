#ifndef MODULES_PLANNING_OPTIMIZERS_OBJECT_DECISION_TYPE_H_
#define MODULES_PLANNING_OPTIMIZERS_OBJECT_DECISION_TYPE_H_

#include "vec2d.h"
// #include <boost/optional.hpp>
#include <string.h>
#include <string>

namespace planning {

// using namespace boost;

typedef struct {
  std::string info;
  bool set_flag;
} ObjectIgnore;

typedef struct {
  planning_math::Vec2d stop_point;
  double distance_s;
  double stop_heading;
  bool set_flag;
} ObjectStop;

typedef struct {
  enum Type { LEFT_NUDGE, RIGHT_NUDGE, NO_NUDGE };
  Type type;
  bool is_longitunidal_ignored;
  double distance_l;
  double start_time;
  double time_buffer;
  int priority = 1;
  bool set_flag;
} ObjectNudge;

typedef struct {
  planning_math::Vec2d fence_point;
  double distance_s;
  double fence_heading;
  double start_time;
  double time_buffer;
  bool set_flag;
} ObjectYield;

typedef struct {
  planning_math::Vec2d fence_point;
  double distance_s;
  double fence_heading;
  double start_time;
  double time_buffer;
  int ObsTag;
  bool set_flag;
} ObjectFollow;

typedef struct {
  planning_math::Vec2d fence_point;
  double distance_s;
  double fence_heading;
  double start_time;
  double time_buffer;
  int ObsTag;
  bool set_flag;
} ObjectOvertake;

typedef struct {
  enum Type { LEFT, RIGHT };
  Type type;
  bool set_flag;
} ObjectSidePass;

typedef struct {
  int level;
  bool set_flag;
} ObjectAvoid;

class ObjectDecisionType {
 public:
  explicit ObjectDecisionType() { object_tag_case_ = OBJECT_TAG_NOT_SET; }

  ~ObjectDecisionType() = default;

  bool has_stop() const { return !stop_.set_flag ? false : true; }
  bool has_nudge() const { return !nudge_.set_flag ? false : true; }
  bool has_yield() const { return !yield_.set_flag ? false : true; }
  bool has_follow() const { return !follow_.set_flag ? false : true; }
  bool has_overtake() const { return !overtake_.set_flag ? false : true; }
  bool has_avoid() const { return !avoid_.set_flag ? false : true; }
  bool has_ignore() const { return !ignore_.set_flag ? false : true; }

  ObjectStop *mutable_stop() {
    if (has_stop()) {
      return &stop_;
    }
    setStopDecision(ObjectStop());
    return &stop_;
  }
  ObjectNudge *mutable_nudge() {
    if (has_nudge()) {
      return &nudge_;
    }
    setNudgeDecision(ObjectNudge());
    return &nudge_;
  }
  ObjectYield *mutable_yield() {
    if (has_yield()) {
      return &yield_;
    }
    setYieldDecision(ObjectYield());
    return &yield_;
  }
  ObjectFollow *mutable_follow() {
    if (has_follow()) {
      return &follow_;
    }
    setFollowDecision(ObjectFollow());
    return &follow_;
  }
  ObjectOvertake *mutable_overtake() {
    if (has_overtake()) {
      return &overtake_;
    }
    setOvertakeDecision(ObjectOvertake());
    return &overtake_;
  }
  ObjectAvoid *mutable_avoid() {
    if (has_avoid()) {
      return &avoid_;
    }
    setAvoidDecision(ObjectAvoid());
    return &avoid_;
  }
  ObjectIgnore *mutable_ignore() {
    if (has_ignore()) {
      return &ignore_;
    }
    setIgnoreDecision(ObjectIgnore());
    return &ignore_;
  }

  ObjectStop stop() const { return stop_; }
  ObjectNudge nudge() const { return nudge_; }
  ObjectYield yield() const { return yield_; }
  ObjectFollow follow() const { return follow_; }
  ObjectOvertake overtake() const { return overtake_; }
  ObjectAvoid avoid() const { return avoid_; }
  ObjectIgnore ignore() const { return ignore_; }
  std::string decision_source() const { return decision_source_; }

  void clear() {
    stop_.set_flag = false;
    nudge_.set_flag = false;
    yield_.set_flag = false;
    follow_.set_flag = false;
    overtake_.set_flag = false;
    avoid_.set_flag = false;
    ignore_.set_flag = false;
    memset(&stop_, 0, sizeof(stop_));
    memset(&nudge_, 0, sizeof(nudge_));
    memset(&yield_, 0, sizeof(yield_));
    memset(&follow_, 0, sizeof(follow_));
    memset(&overtake_, 0, sizeof(overtake_));
    memset(&avoid_, 0, sizeof(avoid_));
    memset(&ignore_, 0, sizeof(ignore_));
  }

  void setStopDecision(ObjectStop stop) {
    clear();
    stop_ = stop;
    object_tag_case_ = kStop;
  }
  void setNudgeDecision(ObjectNudge nudge) {
    clear();
    nudge_ = nudge;
    object_tag_case_ = kNudge;
  }
  void setYieldDecision(ObjectYield yield) {
    clear();
    yield_ = yield;
    object_tag_case_ = kYield;
  }
  void setFollowDecision(ObjectFollow follow) {
    clear();
    follow_ = follow;
    object_tag_case_ = kFollow;
  }
  void setOvertakeDecision(ObjectOvertake overtake) {
    clear();
    overtake_ = overtake;
    object_tag_case_ = kOvertake;
  }
  void setAvoidDecision(ObjectAvoid avoid) {
    clear();
    avoid_ = avoid;
    object_tag_case_ = kAvoid;
  }
  void setIgnoreDecision(ObjectIgnore ignore) {
    clear();
    ignore_ = ignore;
    object_tag_case_ = kIgnore;
  }
  void setDecisionSource(std::string module_name) { decision_source_ = module_name; }

  enum ObjectTagCase { kIgnore, kOvertake, kFollow, kYield, kStop, kNudge, kAvoid, OBJECT_TAG_NOT_SET };

  ObjectTagCase object_tag_case() const { return object_tag_case_; }

 private:
  ObjectStop stop_;
  ObjectNudge nudge_;
  ObjectYield yield_;
  ObjectFollow follow_;
  ObjectOvertake overtake_;
  ObjectAvoid avoid_;
  ObjectIgnore ignore_;
  ObjectTagCase object_tag_case_;
  std::string decision_source_;
};

}  // namespace planning
#endif /* MODULES_PLANNING_OPTIMIZERS_OBJECT_DECISION_TYPE_H_ */
