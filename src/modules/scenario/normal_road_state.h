#pragma once

#include "scenario_state.h"

namespace planning {

struct RoadBase : StateBase {
  bool is_leaf() { return true; }
  virtual void get_state_transition_candidates(
      FsmContext &context, StateTransitionContexts &transition_contexts) {}
  void prepare_for_none_state(
      std::shared_ptr<LaneChangeLaneManager> &lc_lane_manager,
      std::shared_ptr<LaneChangeRequestManager> &lc_req_manager,
      std::vector<ScenarioStateEnum> &candidate_states,
      std::vector<std::shared_ptr<LaneChangeLaneManager>> &lc_lane_managers);
  void prepare_for_change_state(
      std::shared_ptr<LaneChangeLaneManager> &lc_lane_manager,
      std::shared_ptr<LaneChangeRequestManager> &lc_req_manager,
      std::vector<ScenarioStateEnum> &candidate_states,
      std::vector<std::shared_ptr<LaneChangeLaneManager>> &lc_lane_managers);
  void prepare_for_wait_state(
      std::shared_ptr<LaneChangeLaneManager> &lc_lane_manager,
      std::shared_ptr<LaneChangeRequestManager> &lc_req_manager,
      std::vector<ScenarioStateEnum> &candidate_states,
      std::vector<std::shared_ptr<LaneChangeLaneManager>> &lc_lane_managers);
  void prepare_for_back_state(
      std::shared_ptr<LaneChangeLaneManager> &lc_lane_manager,
      std::shared_ptr<LaneChangeRequestManager> &lc_req_manager,
      std::vector<ScenarioStateEnum> &candidate_states,
      std::vector<std::shared_ptr<LaneChangeLaneManager>> &lc_lane_managers);
  void process_wait(FsmContext &context,
                    StateTransitionContexts &transition_contexts);
  void process_change(FsmContext &context,
                      StateTransitionContexts &transition_contexts);
  void process_back(FsmContext &context,
                    StateTransitionContexts &transition_contexts);
};

struct RoadState : StateBase {
  struct None : RoadBase {
    void get_state_transition_candidates(
        FsmContext &context, StateTransitionContexts &transition_contexts);
  };

  struct LC : RoadBase {
    struct LWait : RoadBase {
      void get_state_transition_candidates(
          FsmContext &context, StateTransitionContexts &transition_contexts);
    };

    struct RWait : RoadBase {
      void get_state_transition_candidates(
          FsmContext &context, StateTransitionContexts &transition_contexts);
    };

    struct LChange : RoadBase {
      void get_state_transition_candidates(
          FsmContext &context, StateTransitionContexts &transition_contexts);
    };

    struct RChange : RoadBase {
      void get_state_transition_candidates(
          FsmContext &context, StateTransitionContexts &transition_contexts);
    };

    struct LBack : RoadBase {
      void get_state_transition_candidates(
          FsmContext &context, StateTransitionContexts &transition_contexts);
    };

    struct RBack : RoadBase {
      void get_state_transition_candidates(
          FsmContext &context, StateTransitionContexts &transition_contexts);
    };
  };

  // 暂时不设计Lane Borrow：
  // struct LB : RoadBase {
  //   // static void process_borrow(Control &control, FsmContext &context);
  //   // static void process_back(Control &control, FsmContext &context);
  //   // static void process_return(Control &control, FsmContext &context);
  //   // static void process_suspend(Control &control, FsmContext &context);

  //   struct LBorrow : RoadBase {
  //     void get_state_transition_candidates(
  //       FsmContext &context, StateTransitionContexts &transition_contexts);
  //   };

  //   struct RBorrow : RoadBase {
  //     void get_state_transition_candidates(
  //       FsmContext &context, StateTransitionContexts &transition_contexts);
  //   };

  //   struct LBack : RoadBase {
  //     void get_state_transition_candidates(
  //       FsmContext &context, StateTransitionContexts &transition_contexts);
  //   };

  //   struct RBack : RoadBase {
  //     void get_state_transition_candidates(
  //       FsmContext &context, StateTransitionContexts &transition_contexts);
  //   };

  //   struct LReturn : RoadBase {
  //     void get_state_transition_candidates(
  //       FsmContext &context, StateTransitionContexts &transition_contexts);
  //   };

  //   struct RReturn : RoadBase {
  //     void get_state_transition_candidates(
  //       FsmContext &context, StateTransitionContexts &transition_contexts);
  //   };

  //   struct LSuspend : RoadBase {
  //     void get_state_transition_candidates(
  //       FsmContext &context, StateTransitionContexts &transition_contexts);
  //   };

  //   struct RSuspend : RoadBase {
  //     void get_state_transition_candidates(
  //       FsmContext &context, StateTransitionContexts &transition_contexts);
  //   };
  // };
};

template <>
struct type2int<RoadState::None> {
  enum { value = ROAD_NONE };
};

template <>
struct type2int<RoadState::LC::LWait> {
  enum { value = ROAD_LC_LWAIT };
};

template <>
struct type2int<RoadState::LC::RWait> {
  enum { value = ROAD_LC_RWAIT };
};

template <>
struct type2int<RoadState::LC::LChange> {
  enum { value = ROAD_LC_LCHANGE };
};

template <>
struct type2int<RoadState::LC::RChange> {
  enum { value = ROAD_LC_RCHANGE };
};

template <>
struct type2int<RoadState::LC::LBack> {
  enum { value = ROAD_LC_LBACK };
};

template <>
struct type2int<RoadState::LC::RBack> {
  enum { value = ROAD_LC_RBACK };
};

// template <> struct type2int<RoadState::LB::LBorrow> {
//   enum { value = ROAD_LB_LBORROW };
// };

// template <> struct type2int<RoadState::LB::RBorrow> {
//   enum { value = ROAD_LB_RBORROW };
// };

// template <> struct type2int<RoadState::LB::LBack> {
//   enum { value = ROAD_LB_LBACK };
// };

// template <> struct type2int<RoadState::LB::RBack> {
//   enum { value = ROAD_LB_RBACK };
// };

// template <> struct type2int<RoadState::LB::LReturn> {
//   enum { value = ROAD_LB_LRETURN };
// };

// template <> struct type2int<RoadState::LB::RReturn> {
//   enum { value = ROAD_LB_RRETURN };
// };

// template <> struct type2int<RoadState::LB::LSuspend> {
//   enum { value = ROAD_LB_LSUSPEND };
// };

// template <> struct type2int<RoadState::LB::RSuspend> {
//   enum { value = ROAD_LB_RSUSPEND };
// };

template <>
struct type2name<RoadState::None> {
  static constexpr auto name = "ROAD_NONE";
};

template <>
struct type2name<RoadState::LC::LWait> {
  static constexpr auto name = "ROAD_LC_LWAIT";
};

template <>
struct type2name<RoadState::LC::RWait> {
  static constexpr auto name = "ROAD_LC_RWAIT";
};

template <>
struct type2name<RoadState::LC::LChange> {
  static constexpr auto name = "ROAD_LC_LCHANGE";
};

template <>
struct type2name<RoadState::LC::RChange> {
  static constexpr auto name = "ROAD_LC_RCHANGE";
};

template <>
struct type2name<RoadState::LC::LBack> {
  static constexpr auto name = "ROAD_LC_LBACK";
};

template <>
struct type2name<RoadState::LC::RBack> {
  static constexpr auto name = "ROAD_LC_RBACK";
};

// template <> struct type2name<RoadState::LB::LBorrow> {
//   static constexpr auto name = "ROAD_LB_LBORROW";
// };

// template <> struct type2name<RoadState::LB::RBorrow> {
//   static constexpr auto name = "ROAD_LB_RBORROW";
// };

// template <> struct type2name<RoadState::LB::LBack> {
//   static constexpr auto name = "ROAD_LB_LBACK";
// };

// template <> struct type2name<RoadState::LB::RBack> {
//   static constexpr auto name = "ROAD_LB_RBACK";
// };

// template <> struct type2name<RoadState::LB::LReturn> {
//   static constexpr auto name = "ROAD_LB_LRETURN";
// };

// template <> struct type2name<RoadState::LB::RReturn> {
//   static constexpr auto name = "ROAD_LB_RRETURN";
// };

// template <> struct type2name<RoadState::LB::LSuspend> {
//   static constexpr auto name = "ROAD_LB_LSUSPEND";
// };

// template <> struct type2name<RoadState::LB::RSuspend> {
//   static constexpr auto name = "ROAD_LB_RSUSPEND";
// };

}  // namespace planning
