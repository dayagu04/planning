#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/pytypes.h>
#include <pybind11/stl.h>
#include <sys/param.h>

#include <chrono>
#include <cmath>
#include <cstdint>
#include <ctime>
#include <iostream>
#include <limits>
#include <memory>
#include <ostream>
#include <unordered_map>
#include <utility>
#include <vector>

#include "jupyter_pybind/src/serialize_utils.h"
#include "modules/common/task_basic_types.h"
#include "modules/context/ego_planning_config.h"
#include "modules/tasks/task_interface/potential_dangerous_agent_decider_output.h"
#include "potential_dangerous_agent_decider_info.pb.h"
#include "src/modules/common/config/message_type.h"
#include "src/modules/common/rss_model/rss_model.h"

namespace py = pybind11;
using namespace planning;
// static class
// bag data processed to set class members to call functions
class RssInterface {
 public:
  // 各种配置参数
  PotentialDangerousAgentDeciderConfig::RSSModelConfig vru_rss_params;
  PotentialDangerousAgentDeciderConfig::RSSModelConfig vru_rss_params_reckless;
  PotentialDangerousAgentDeciderConfig::RSSModelConfig
      oversize_vehicle_rss_params;
  PotentialDangerousAgentDeciderConfig::RSSModelConfig
      oversize_vehicle_rss_params_reckless;
  PotentialDangerousAgentDeciderConfig::RSSModelConfig
      normal_size_vehicle_rss_params;
  PotentialDangerousAgentDeciderConfig::RSSModelConfig
      normal_size_vehicle_rss_params_reckless;

  // 更新参数接口
  bool UpdateModerateParams(int config_type = 3, double response_time = 0.2,
                            double longitudinal_acc_max = 1.2,
                            double longitudinal_brake_min = 2.0,
                            double longitudinal_brake_max = 4.0,
                            double lateral_acc_max = 5.0,
                            double lateral_brake_min = 0.6,
                            double lateral_brake_max = 1.0,
                            double lateral_miu = 1.0) {
    PotentialDangerousAgentDeciderConfig::RSSModelConfig rss_config;
    rss_config.response_time = response_time;
    rss_config.longitudinal_acc_max = longitudinal_acc_max;
    rss_config.longitudinal_brake_min = longitudinal_brake_min;
    rss_config.longitudinal_brake_max = longitudinal_brake_max;
    rss_config.lateral_acc_max = lateral_acc_max;
    rss_config.lateral_brake_min = lateral_brake_min;
    rss_config.lateral_brake_max = lateral_brake_max;
    rss_config.lateral_miu = lateral_miu;

    // if (config_type == 1)
    vru_rss_params = rss_config;
    // else if (config_type == 2)
    oversize_vehicle_rss_params = rss_config;
    // else
    normal_size_vehicle_rss_params = rss_config;

    return true;
  }

  bool UpdateRecklessParams(int config_type = 3, double response_time = 0.2,
                            double longitudinal_acc_max = 1.2,
                            double longitudinal_brake_min = 2.0,
                            double longitudinal_brake_max = 4.0,
                            double lateral_acc_max = 5.0,
                            double lateral_brake_min = 0.6,
                            double lateral_brake_max = 1.0,
                            double lateral_miu = 1.0) {
    PotentialDangerousAgentDeciderConfig::RSSModelConfig rss_config;
    rss_config.response_time = response_time;
    rss_config.longitudinal_acc_max = longitudinal_acc_max;
    rss_config.longitudinal_brake_min = longitudinal_brake_min;
    rss_config.longitudinal_brake_max = longitudinal_brake_max;
    rss_config.lateral_acc_max = lateral_acc_max;
    rss_config.lateral_brake_min = lateral_brake_min;
    rss_config.lateral_brake_max = lateral_brake_max;
    rss_config.lateral_miu = lateral_miu;

    // if (config_type == 1)
    vru_rss_params_reckless = rss_config;
    // else if (config_type == 2)
    oversize_vehicle_rss_params_reckless = rss_config;
    // else
    normal_size_vehicle_rss_params_reckless = rss_config;

    return true;
  }

  // 计算接口
  py::bytes CalculateAgentRSSDistance(py::bytes& input_bytes) {
    planning::common::PotentialDangerousAgentDeciderInfo pda_input =
        BytesToProto<planning::common::PotentialDangerousAgentDeciderInfo>(
            input_bytes);
    const auto& agents = pda_input.potential_dangerous_agent();
    const auto& ego_sl_info = pda_input.ego_sl_info();
    planning::common::PotentialDangerousAgentDeciderInfo output;
    output.set_ego_lateral_vel(pda_input.ego_lateral_vel());
    output.set_ego_longitudinal_vel(pda_input.ego_longitudinal_vel());
    output.set_risk_free_longitudinal_distance(
        pda_input.risk_free_longitudinal_distance());
    output.set_risk_free_lateral_distance(
        pda_input.risk_free_lateral_distance());
    // output.set_ego_sl_info(pda_input.ego_sl_info());
    if (agents.empty()) return {};

    RssModel::FrenetState ego_sl = {
        {{ego_sl_info.s(), ego_sl_info.ds(), ego_sl_info.dds()}},
        {{ego_sl_info.l(), ego_sl_info.dl(), ego_sl_info.ddl()}}};
    for (const auto& agent : agents) {
      planning::common::PotentialDangerousAgent* dangerous_info =
          output.add_potential_dangerous_agent();
      dangerous_info->CopyFrom(agent);
      std::cout << "--------" << std::endl;
      RssModel::FrenetState agent_sl = {{{agent.s(), agent.ds(), 0.0}},
                                        {{agent.l(), agent.dl(), 0.0}}};
      PotentialDangerousAgentDeciderConfig::RSSModelConfig* rss_config =
          nullptr;
      PotentialDangerousAgentDeciderConfig::RSSModelConfig* reckless_config =
          nullptr;
      double longitudinal_distance = agent.longitudinal_distance();
      double lateral_distance = agent.lateral_distance();
      // if (agent.agent_type() == 1) {
      //   rss_config = &vru_rss_params;
      //   reckless_config = &vru_rss_params_reckless;
      // } else if (agent.agent_type() == 2) {
      //   rss_config = &oversize_vehicle_rss_params;
      //   reckless_config = &oversize_vehicle_rss_params_reckless;
      // } else {
      rss_config = &normal_size_vehicle_rss_params;
      reckless_config = &normal_size_vehicle_rss_params_reckless;
      // }

      double lateral_reckless_distance = 0.;
      double longitudinal_reckless_distance = 0.;
      double lateral_moderate_distance = 0.;
      double longitudinal_moderate_distance = 0.;
      AgentPosType agent_pos_type =
          static_cast<AgentPosType>(agent.agent_pos_type());
      RssModel::CalculateSafeDistance(
          agent_sl, ego_sl, agent_pos_type, *reckless_config,
          &lateral_reckless_distance, &longitudinal_reckless_distance);
      RssModel::CalculateSafeDistance(agent_sl, ego_sl, agent_pos_type,
                                      *rss_config, &lateral_moderate_distance,
                                      &longitudinal_moderate_distance);
      std::cout << "reckless: " << reckless_config->response_time << std::endl;
      std::cout << "normal: " << rss_config->response_time << std::endl;
      std::cout << "lateral_reckless_distance: " << lateral_reckless_distance
                << std::endl;
      std::cout << "longitudinal_reckless_distance: "
                << longitudinal_reckless_distance << std::endl;
      std::cout << "lateral_moderate_distance: " << lateral_moderate_distance
                << std::endl;
      std::cout << "longitudinal_moderate_distance: "
                << longitudinal_moderate_distance << std::endl;
      bool ignore_lateral = agent_pos_type == AgentPosType::REAR ||
                            agent_pos_type == AgentPosType::FRONT;
      bool ignore_longitudinal = agent_pos_type == AgentPosType::LEFT_OVERLAP ||
                                 agent_pos_type == AgentPosType::RIGHT_OVERLAP;
      RiskLevel risk_level = RiskLevel::NO_RISK;
      RecommendedManeuver maneuver;
      dangerous_info->set_longitudinal_moderate_safe_distance(
          longitudinal_moderate_distance);
      dangerous_info->set_longitudinal_reckless_safe_distance(
          longitudinal_reckless_distance);
      dangerous_info->set_lateral_moderate_safe_distance(
          lateral_moderate_distance);
      dangerous_info->set_lateral_reckless_safe_distance(
          lateral_reckless_distance);

      if ((longitudinal_distance > longitudinal_moderate_distance ||
           ignore_longitudinal) ||
          (lateral_distance > lateral_moderate_distance || ignore_lateral)) {
        dangerous_info->set_risk_level(static_cast<int>(risk_level));
        dangerous_info->set_recommended_lateral_maneuver(
            static_cast<int>(maneuver.lateral_maneuver));
        dangerous_info->set_recommended_longitudinal_maneuver(
            static_cast<int>(maneuver.longitudinal_maneuver));
        continue;
      }

      if (!ignore_lateral && !ignore_longitudinal) {
        if (longitudinal_distance <= longitudinal_moderate_distance &&
            longitudinal_distance > longitudinal_reckless_distance) {
          risk_level = RiskLevel::LOW_RISK;
          maneuver.longitudinal_maneuver = LongitudinalManeuver::SLIGHTLY_BRAKE;
          maneuver.lateral_maneuver = LateralManeuver::IGNORE;

        } else if (longitudinal_distance <= longitudinal_reckless_distance) {
          if (lateral_distance <= lateral_moderate_distance &&
              lateral_distance > lateral_reckless_distance) {
            risk_level = RiskLevel::LOW_RISK;
            maneuver.longitudinal_maneuver =
                LongitudinalManeuver::SLIGHTLY_BRAKE;
            if (agent_pos_type == AgentPosType::LEFT_FRONT ||
                agent_pos_type == AgentPosType::LEFT_OVERLAP ||
                agent_pos_type == AgentPosType::LEFT_REAR) {
              maneuver.lateral_maneuver = LateralManeuver::RIGHT_SLIGHTLY_NUDGE;
            } else if (agent_pos_type == AgentPosType::RIGHT_FRONT ||
                       agent_pos_type == AgentPosType::RIGHT_OVERLAP ||
                       agent_pos_type == AgentPosType::RIGHT_REAR) {
              maneuver.lateral_maneuver = LateralManeuver::LEFT_SLIGHTLY_NUDGE;
            } else {
              maneuver.lateral_maneuver =
                  LateralManeuver::BOTH_SIDE_SLIGHTLY_NUDGE;
            }
          } else {
            risk_level = RiskLevel::HIGH_RISK;
            maneuver.longitudinal_maneuver =
                LongitudinalManeuver::SLIGHTLY_BRAKE;
            if (agent_pos_type == AgentPosType::LEFT_FRONT ||
                agent_pos_type == AgentPosType::LEFT_OVERLAP ||
                agent_pos_type == AgentPosType::LEFT_REAR) {
              maneuver.lateral_maneuver = LateralManeuver::RIGHT_NUDGE;
            } else if (agent_pos_type == AgentPosType::RIGHT_FRONT ||
                       agent_pos_type == AgentPosType::RIGHT_OVERLAP ||
                       agent_pos_type == AgentPosType::RIGHT_REAR) {
              maneuver.lateral_maneuver = LateralManeuver::LEFT_NUDGE;
            } else {
              maneuver.lateral_maneuver = LateralManeuver::BOTH_SIDE_NUDGE;
            }
          }
        }
      } else if (ignore_lateral) {
        if (longitudinal_distance <= longitudinal_moderate_distance &&
            longitudinal_distance > longitudinal_reckless_distance) {
          risk_level = RiskLevel::LOW_RISK;
          maneuver.longitudinal_maneuver = LongitudinalManeuver::SLIGHTLY_BRAKE;
          maneuver.lateral_maneuver = LateralManeuver::IGNORE;
        } else if (longitudinal_distance <= longitudinal_reckless_distance) {
          risk_level = RiskLevel::HIGH_RISK;
          maneuver.longitudinal_maneuver = LongitudinalManeuver::SLIGHTLY_BRAKE;
          if (agent_pos_type == AgentPosType::LEFT_FRONT ||
              agent_pos_type == AgentPosType::LEFT_OVERLAP ||
              agent_pos_type == AgentPosType::LEFT_REAR) {
            maneuver.lateral_maneuver = LateralManeuver::RIGHT_NUDGE;
          } else if (agent_pos_type == AgentPosType::RIGHT_FRONT ||
                     agent_pos_type == AgentPosType::RIGHT_OVERLAP ||
                     agent_pos_type == AgentPosType::RIGHT_REAR) {
            maneuver.lateral_maneuver = LateralManeuver::LEFT_NUDGE;
          } else {
            maneuver.lateral_maneuver = LateralManeuver::BOTH_SIDE_NUDGE;
          }
        }
      } else if (ignore_longitudinal) {
        if (lateral_distance <= lateral_moderate_distance &&
            lateral_distance > lateral_reckless_distance) {
          risk_level = RiskLevel::LOW_RISK;
          maneuver.longitudinal_maneuver = LongitudinalManeuver::IGNORE;
          if (agent_pos_type == AgentPosType::LEFT_FRONT ||
              agent_pos_type == AgentPosType::LEFT_OVERLAP ||
              agent_pos_type == AgentPosType::LEFT_REAR) {
            maneuver.lateral_maneuver = LateralManeuver::RIGHT_SLIGHTLY_NUDGE;
          } else if (agent_pos_type == AgentPosType::RIGHT_FRONT ||
                     agent_pos_type == AgentPosType::RIGHT_OVERLAP ||
                     agent_pos_type == AgentPosType::RIGHT_REAR) {
            maneuver.lateral_maneuver = LateralManeuver::LEFT_SLIGHTLY_NUDGE;
          } else {
            maneuver.lateral_maneuver =
                LateralManeuver::BOTH_SIDE_SLIGHTLY_NUDGE;
          }

        } else if (lateral_distance <= lateral_reckless_distance) {
          risk_level = RiskLevel::HIGH_RISK;
          maneuver.longitudinal_maneuver = LongitudinalManeuver::IGNORE;
          if (agent_pos_type == AgentPosType::LEFT_FRONT ||
              agent_pos_type == AgentPosType::LEFT_OVERLAP ||
              agent_pos_type == AgentPosType::LEFT_REAR) {
            maneuver.lateral_maneuver = LateralManeuver::RIGHT_NUDGE;
          } else if (agent_pos_type == AgentPosType::RIGHT_FRONT ||
                     agent_pos_type == AgentPosType::RIGHT_OVERLAP ||
                     agent_pos_type == AgentPosType::RIGHT_REAR) {
            maneuver.lateral_maneuver = LateralManeuver::LEFT_NUDGE;
          } else {
            maneuver.lateral_maneuver = LateralManeuver::BOTH_SIDE_NUDGE;
          }
        }
      }
      std::cout << "============" << std::endl;
      dangerous_info->set_risk_level(static_cast<int>(risk_level));
      dangerous_info->set_recommended_lateral_maneuver(
          static_cast<int>(maneuver.lateral_maneuver));
      dangerous_info->set_recommended_longitudinal_maneuver(
          static_cast<int>(maneuver.longitudinal_maneuver));
    }
    std::string message_string;
    output.SerializeToString(&message_string);
    return message_string;
  }
};

PYBIND11_MODULE(potential_dangerous_agent_decider_py, m) {
  py::class_<RssInterface>(m, "RssInterface")
      .def(py::init<>())
      .def("UpdateModerateParams", &RssInterface::UpdateModerateParams)
      .def("UpdateRecklessParams", &RssInterface::UpdateRecklessParams)
      .def("CalculateAgentRSSDistance",
           &RssInterface::CalculateAgentRSSDistance);
}
