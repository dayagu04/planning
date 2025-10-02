#pragma once
#include <cstddef>
#include <cstdint>
#include <vector>

#include "geometry_math.h"
namespace planning {
using namespace pnc;

#define MAX_GEAR_NUM 2
#define MAX_REF_LINE_NUM 13
#define MAX_LPL_PATH_NUM 7
#define MAX_SUCCESS_LPL_PATH_NUM 10

struct LinkPoseLineInput {
  geometry_lib::LineSegment ref_line;
  geometry_lib::PathPoint pose;

  double min_radius;
  double sturn_radius;
  // if use, the second arc which link line and same gear with
  // ref_last_line_gear of two reverse arc path can use bigger_radius which is
  // bigger than min_radius
  bool use_bigger_radius;
  double bigger_radius_no_asssign;
  double bigger_radius_asssign;

  double lat_err;      // m
  double theta_err;  // deg

  bool link_line_start_pt = true;
  bool can_exceed_line_start_pt = true;

  uint8_t ref_last_line_gear;  // can be none, drive, reverse
  double reverse_last_line_min_length;
  double drive_last_line_min_length;

  // if false, it will definitely be successfully linked
  bool has_length_require = true;

  LinkPoseLineInput() = default;
  ~LinkPoseLineInput() = default;

  LinkPoseLineInput(const geometry_lib::LineSegment& _line,
                    const geometry_lib::PathPoint& _pose,
                    const double _min_radius, const double _sturn_radius,
                    const double _lat_err, const double _heading_err,
                    const bool _link_line_start_pt = false,
                    const bool _can_exceed_line_start_pt = true,
                    const bool _has_length_require = true,
                    const double _reverse_min_last_line_length = 1e-2,
                    const double _drive_min_last_line_length = 1e-2)
      : ref_line(_line),
        pose(_pose),
        min_radius(_min_radius),
        sturn_radius(_sturn_radius),
        lat_err(_lat_err),
        theta_err(_heading_err),
        link_line_start_pt(_link_line_start_pt),
        can_exceed_line_start_pt(_can_exceed_line_start_pt),
        has_length_require(_has_length_require),
        reverse_last_line_min_length(_reverse_min_last_line_length),
        drive_last_line_min_length(_drive_min_last_line_length) {}
};

struct LinkPoseLinePath {
  geometry_lib::PathSegment segs[MAX_LPL_PATH_NUM];
  uint8_t gears[MAX_LPL_PATH_NUM];
  uint8_t steers[MAX_LPL_PATH_NUM];
  double single_gear_lengths[MAX_LPL_PATH_NUM];
  double kappas[MAX_LPL_PATH_NUM];
  double lengths[MAX_LPL_PATH_NUM];

  uint8_t seg_num = 0;
  uint8_t gear_change_num = 0;
  uint8_t gear_num = 0;

  double total_length = 0.0;
  double cur_gear_length = 0.0;

  double last_line_length = 0.0;

  double kappa_change = 0.0;

  uint8_t cur_gear = 0;
  uint8_t last_gear = 0;
  uint8_t last_steer = 0;

  std::vector<std::vector<geometry_lib::PathPoint>> ptss;

  LinkPoseLinePath() = default;
  ~LinkPoseLinePath() = default;

  void SetPath(const geometry_lib::PathSegment& seg);

  void SetPath(const geometry_lib::PathSegment _segs[], uint8_t _seg_num);

  void SetPath(const std::vector<geometry_lib::PathSegment>& seg_vec);

  void AddPath(const geometry_lib::PathSegment& seg);

  void AddPath(const geometry_lib::PathSegment _segs[], uint8_t _seg_num);

  void AddPath(const std::vector<geometry_lib::PathSegment>& seg_vec);

  void SamplePath(const double sample_ds);

  void PrintInfo(const bool enable_log = true) const;

  void Clear() {
    seg_num = 0;
    gear_change_num = 0;
    gear_num = 0;
    total_length = 0.0;
    cur_gear_length = 0.0;
    cur_gear = 0;
    last_gear = 0;
    last_line_length = 0.0;
    kappa_change = 0.0;
    ptss.clear();
  }
};

struct LinkPoseLineOutput {
  LinkPoseLinePath lpl_paths[MAX_SUCCESS_LPL_PATH_NUM];
  uint8_t lpl_path_num = 0;

  void Clear() { lpl_path_num = 0; }
};

class LinkPoseLine {
 public:
  LinkPoseLine() {}
  ~LinkPoseLine() {}

  const bool CalLPLPath(const LinkPoseLineInput& input);

  const LinkPoseLineOutput& GetLPLOutput() const { return output_; }

 private:
  const bool OneArcPath(LinkPoseLinePath& lpl_path, const uint8_t ref_gear);

  const bool TwoArcPath(LinkPoseLinePath& lpl_path, const uint8_t ref_gear,
                        const bool same_gear);

  const bool LineArcPath(LinkPoseLinePath& lpl_path, const uint8_t ref_gear,
                         const bool same_gear);

  const bool AlignBodySTurnPath(LinkPoseLinePath& lpl_path,
                                const uint8_t ref_gear, const bool same_gear);

 private:
  LinkPoseLineInput input_;
  LinkPoseLineOutput output_;

  geometry_lib::LineSegment ref_lines_[MAX_REF_LINE_NUM];
  size_t ref_line_num_ = 0;

  double virtual_circle1_dist_ = 0.0;
  double virtual_circle2_dist_ = 0.0;

  const geometry_lib::PathSegGear gears_[MAX_GEAR_NUM] = {
      geometry_lib::SEG_GEAR_DRIVE, geometry_lib::SEG_GEAR_REVERSE};
};

}  // namespace planning
