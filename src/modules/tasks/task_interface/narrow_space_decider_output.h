#pragma once

namespace planning {


struct NarrowSpaceDeciderOutput {
  bool is_exist_narrow_space = false;
  bool is_passable_narrow_space = false;
  bool is_in_narrow_space = false;
  bool is_too_wide = false;
  bool is_too_narrow = false;
  bool is_relative_angle_too_large = false;

  void Clear() {
    is_exist_narrow_space = false;
    is_passable_narrow_space = false;
    is_in_narrow_space = false;
    is_too_wide = false;
    is_too_narrow = false;
    is_relative_angle_too_large = false;
  }
};

}  // namespace planning