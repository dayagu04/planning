#include "dp_st_cost.h"
namespace planning {

DPSTCost::DPSTCost(const DPSpeedGraphConfig& dp_speed_config,
                      const std::vector<const FrenetObstacle*>& obstacles,
                      const planning::common::TrajectoryPoint& init_point)
                      :
                      dp_speed_config_(dp_speed_config),
                      obstacles_(obstacles),
                      init_point_(init_point),
                      unit_s_(dp_speed_config_.total_path_length/dp_speed_config_.matrix_dimension_s),
                      unit_t_(dp_speed_config_.total_time/dp_speed_config_.matrix_dimension_t),
                      unit_v_(unit_s_/unit_t_) {}
double DPSTCost::GetReferenceCost(const STPoint& point, const STPoint& reference_point)const{

}
double DPSTCost::GetObstacleCost(const STPoint& point)const{}

double DPSTCost::GetSpeedCost(const STPoint& first, const STPoint& second, const double speed_limit)const{}
double DPSTCost::GetAccelCostByTwo(const double pre_speed, const STPoint& first, const STPoint& second)const{}
double DPSTCost::GetAccelCostByThree(const STPoint& first, const STPoint& second, const STPoint& third)const{}
double DPSTCost::GetJerkCostByTwo(const STPoint& pred_point,const double pre_speed, const double pre_acc, const STPoint& cur_point)const{}
double DPSTCost::GetJerkCostByThree(const STPoint& first_point,const double first_speed, const STPoint& second_point, const STPoint& third_point)const{}
double DPSTCost::GetJerkCostByFour(const STPoint& first, const STPoint& second, const STPoint& third, const STPoint& forth) const{}
double DPSTCost::GetAccelCost(const double accel)const{}
double DPSTCost::GeJerkCost(const double jerk)const{}
}