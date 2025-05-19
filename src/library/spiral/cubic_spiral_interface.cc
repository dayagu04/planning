#include "cubic_spiral_interface.h"

#include "log_glog.h"

namespace planning {
const bool CubicSpiralInterface::CubicSpiralStatesSolve(
    bool *solution_usable, std::vector<spiral_path_point_t> &states,
    const spiral_path_point_t *start, const spiral_path_point_t *goal,
    float step_length, bool constrain_start_k, bool constrain_goal_k) {
  bool ret = false;
  solution_cubic_t sol;

  if (solution_usable == NULL || start == NULL || goal == NULL) {
    ILOG_ERROR << " there is an error in the input !";
    return false;
  }

  if (constrain_start_k && constrain_goal_k) {
    ret = CubicSpiralStrictSolve(&sol, start, goal);

  } else if (!constrain_start_k && constrain_goal_k) {
    ret = CubicSpiralStartkFreeSolve(&sol, start, goal);
  } else if (constrain_start_k && !constrain_goal_k) {
    ret = CubicSpiralEndkFreeSolve(&sol, start, goal);
  } else /* both end free */
  {
    ret = CubicSpiralBothkFreeSolve(&sol, start, goal);
  }

  if (!ret) {
    ILOG_ERROR << " solver calculation failed !";
    return false;
  }

  *solution_usable = (bool)(sol.solve_status);
  /* usable */
  if (*solution_usable) {
    if (!SampleCubicSpiralStatesBySol(states, &sol, step_length)) {
      // ILOG_ERROR << " cubic spiral sampling failed !";
      return false;
    }
  }

  return true;
}

const bool CubicSpiralInterface::GenerateCubicSpiralPathByStrictSolve(
    solution_cubic_t *solution, std::vector<spiral_path_point_t> &states,
    const spiral_path_point_t *start, const spiral_path_point_t *goal,
    const float step_length) {
  bool ret = false;
  solution_cubic_t sol;
  if (start == NULL || goal == NULL) {
    ILOG_ERROR << "input pointers should not be NULL!";
    return false;
  }

  ret = CubicSpiralStrictSolve(&sol, start, goal);
  if (!CubicSpiralStrictSolve(&sol, start, goal)) {
    ILOG_ERROR << "cubic spiral solve failed !";
    return false;
  }

  bool solution_usable = (bool)(sol.solve_status);
  if (solution_usable) {
    if (!SampleCubicSpiralStatesBySol(states, &sol, step_length)) {
      // ILOG_ERROR << "cubic spiral sampling failed !";
      return false;
    }
  }

  return true;
}
}  // namespace planning