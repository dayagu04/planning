#pragma once

#include <cstdint>
#include <memory>
#include <vector>

#include "base_function.h"
#include "plan_data.h"
#include "session.h"
#include "apa_plan_interface.h"

namespace planning {

class ApaFunction : public BaseFunction {
 public:
  ApaFunction(framework::Session *session);

  virtual ~ApaFunction() = default;

  bool Reset() override;

  bool Plan() override;

  const std::shared_ptr<plan_interface::PlanData> GetPlanDataPtr() const {
    return plan_data_ptr_;
  }

 private:
  void Init();

 private:
  std::shared_ptr<plan_interface::PlanData> plan_data_ptr_;

  std::unique_ptr<apa_planner::ApaPlanInterface> apa_plan_interface_;

  // uint8_t last_planner_interface_type_ = 0;
};

}  // namespace planning
