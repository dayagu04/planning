// #pragma once

// #include "lane_change_request.h"

// namespace planning {

// /// @brief 自主式(Active)换道请求
// class ActRequest : public LaneChangeRequest {
//  public:
//   ActRequest(planning::framework::Session* session,
//              std::shared_ptr<VirtualLaneManager> virtual_lane_mgr,
//              std::shared_ptr<LaneChangeLaneManager> lane_change_lane_mgr);
//   virtual ~ActRequest() = default;

//   void Update(std::shared_ptr<ObjectSelector> object_selector, int lc_status,
//               double start_move_distolane, double lc_int_tfinish,
//               double lc_map_tfinish, bool accident_ahead, bool not_accident);

//   void Reset(int direction = NO_CHANGE);

//   bool enforced_l() const { return enforced_l_; }
//   bool enforced_r() const { return enforced_r_; }
//   bool enable_l() const { return enable_l_; }

//   void set_enforced_l(bool value) { enforced_l_ = value; }
//   void set_enforced_r(bool value) { enforced_r_ = value; }

//   std::string act_request_source() { return act_request_source_; }

//  private:
//   int pos_cnt_l_ = 0;
//   int pos_cnt_r_ = 0;
//   int neg_cnt_l_ = 0;
//   int neg_cnt_r_ = 0;
//   bool left_faster_ = false;
//   bool right_faster_ = false;
//   bool enable_l_ = false;
//   bool enable_r_ = false;
//   bool enforced_l_ = false;
//   bool enforced_r_ = false;

//   // double vrel_f_ = DBL_MAX;
//   // double vrel_l_ = DBL_MAX;
//   // double vrel_r_ = DBL_MAX;
//   // tmp ignore DBL_MAX
//   double vrel_f_ = 1000.;
//   double vrel_l_ = 1000.;
//   double vrel_r_ = 1000.;
//   std::string act_request_source_{"none"};
//   ActRequestConfig act_request_config_;

//   void print_forbid_generating_reason(
//       const std::vector<std::string> forbid_generating_reason);
//   void check_lc_forbid_reason(
//       std::vector<std::string>& forbid_generating_left_reason,
//       std::vector<std::string>& forbid_generating_right_reason);
// };
// }  // namespace planning