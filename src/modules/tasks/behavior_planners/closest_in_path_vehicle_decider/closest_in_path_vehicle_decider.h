#pragma once

#include <map>
#include <utility>

#include "planning_context.h"
#include "tasks/task.h"

namespace planning {

class ClosestInPathVehicleDecider : public Task {
 public:
  ClosestInPathVehicleDecider(const EgoPlanningConfigBuilder* config_builder,
                              framework::Session* session);
  virtual ~ClosestInPathVehicleDecider() = default;

  bool Execute() override;

  void Reset();

  void Reset(int32_t* const cipv_id, double* const relative_s,
             double* const v_frenet, double* const cipv_ttc,
             int32_t* const dangerous_level, bool* const is_virtual);

 private:
  bool MakeDecison();
  bool CipvDecision();
  bool DetermineIfConeBucketCIPV();
  void MakeCipvInfo(const int32_t cipv_id, double* const relative_s,
                    double* const v_frenet, double* acc, double* const cipv_ttc,
                    int32_t* const dangerous_level, bool* const is_virtual);
  void DetermineCIPVInfoForHMI() const;

 private:
  //<double, std::pair<bool, int32_t>> :
  //   <distance to ego at cur time,
  //        <is virtual, agent id>>
  std::map<double, std::pair<bool, int32_t>> agents_distance_id_map_;
};

}  // namespace planning
