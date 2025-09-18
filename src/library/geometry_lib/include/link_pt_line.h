#pragma once
#include <cstdint>
#include <vector>

#include "common_math.h"

namespace planning {
namespace link_pt_line {
using namespace common_math;

#define MAX_GEAR_NUM 2
#define MAX_REF_LINE_NUM 13
#define MAX_LPL_PATH_NUM 7
#define MAX_SUCCESS_LPL_PATH_NUM 10

template <typename T>
struct LinkPtLineInput {
  LinkPtLineInput() {}
  ~LinkPtLineInput() {}

  bool use_bigger_radius;
  bool has_length_require;
  bool link_line_start_pt;
  AstarPathGear ref_last_line_gear;

  T min_radius;
  T sturn_radius;
  T bigger_radius_no_asssign;
  T bigger_radius_asssign;
  T lat_err;    // m
  T theta_err;  // deg
  T reverse_last_line_min_length;
  T drive_last_line_min_length;

  LineSeg<T> ref_line;
  PathPt<T> pose;
};

template <typename T>
struct LinkPtLinePath {
  LinkPtLinePath() {}
  ~LinkPtLinePath() {}

  uint8_t seg_num;
  uint8_t gear_change_num;
  uint8_t gear_num;
  AstarPathGear cur_gear;
  AstarPathGear last_gear;
  AstarPathSteer last_steer;

  T total_length;
  T cur_gear_length;

  T last_line_length;

  T kappa_change;

  std::vector<std::vector<PathPt<T>>> ptss;

  AstarPathGear gears[MAX_LPL_PATH_NUM];
  AstarPathSteer steers[MAX_LPL_PATH_NUM];
  T lengths[MAX_LPL_PATH_NUM];
  T single_gear_lengths[MAX_LPL_PATH_NUM];
  T kappas[MAX_LPL_PATH_NUM];

  PathSeg<T> segs[MAX_LPL_PATH_NUM];

  void Clear() { seg_num = 0; }

  void Reset() {
    seg_num = 0;
    gear_change_num = 0;
    gear_num = 0;
    cur_gear = AstarPathGear::NONE;
    last_gear = AstarPathGear::NONE;
    last_steer = AstarPathSteer::NONE;
    total_length = T(0.0f);
    cur_gear_length = T(0.0f);
    kappa_change = T(0.0f);
    last_line_length = T(0.0f);
    ptss.clear();
  }

  void SetPathSegs(const PathSeg<T> _segs[], const uint8_t _seg_num);

  void SamplePath(const T sample_ds);

  void PrintInfo(const bool enable_log = true) const;
};

template <typename T>
struct LinkPtLineOutput {
  uint8_t lpl_path_num = 0;
  LinkPtLinePath<T> lpl_paths[MAX_SUCCESS_LPL_PATH_NUM];

  void Reset() { lpl_path_num = 0; }
};

template <typename T>
class LinkPtLine {
 public:
  LinkPtLine() {}
  ~LinkPtLine() {}

  const bool CalLPLPath(const LinkPtLineInput<T>& input);

  const LinkPtLineOutput<T>& GetLPLOutput() const { return output_; }

 private:
  const bool OneArcPath(LinkPtLinePath<T>& lpl_path,
                        const AstarPathGear ref_gear);

  const bool TwoArcPath(LinkPtLinePath<T>& lpl_path,
                        const AstarPathGear ref_gear, const bool same_gear);

  const bool LineArcPath(LinkPtLinePath<T>& lpl_path,
                         const AstarPathGear ref_gear, const bool same_gear);

  const bool AlignBodySTurnPath(LinkPtLinePath<T>& lpl_path,
                                const AstarPathGear ref_gear,
                                const bool same_gear);

 private:
  LinkPtLineInput<T> input_;
  LinkPtLineOutput<T> output_;

  LineSeg<T> ref_lines_[MAX_REF_LINE_NUM];
  uint8_t ref_line_num_ = 0;

  T virtual_circle1_dist_ = 0.0;
  T virtual_circle2_dist_ = 0.0;

  const AstarPathGear gears_[MAX_GEAR_NUM] = {AstarPathGear::DRIVE,
                                              AstarPathGear::REVERSE};
};

}  // namespace link_pt_line
}  // namespace planning

// #include "link_pt_line.tpp"