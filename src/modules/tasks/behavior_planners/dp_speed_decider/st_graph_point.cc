#include "st_graph_point.h"
#include "obstacle.h"
#include "speed/st_point.h"
namespace planning {
uint32_t STGraphPoint::index_s() const { return index_s_; }
uint32_t STGraphPoint::index_t() const { return index_t_; }
double STGraphPoint::reference_cost() const { return reference_cost_; }
double STGraphPoint::obstacle_cost() const { return obstacle_cost_; }
double STGraphPoint::total_cost() const { return total_cost_; }

const STPoint& STGraphPoint::point() const { return point_; }
const STGraphPoint* STGraphPoint::pre_point() const { return pre_point_; }
// set value
void STGraphPoint::Init(const uint32_t index_t, const uint32_t index_s,
                        const STPoint& st_point) {
  index_s_ = index_s;
  index_t_ = index_t;
  point_ = st_point;
}
void STGraphPoint::SetReferenceCost(const double reference_cost) {
  reference_cost_ = reference_cost;
}
void STGraphPoint::SetObstacleCost(const double obstacle_cost) {
  obstacle_cost_ = obstacle_cost;
}
void STGraphPoint::SetTotalCost(const double total_cost) {
  total_cost_ = total_cost;
}
void STGraphPoint::SetPrePoint(const STGraphPoint& pre_point) {
  pre_point_ = &pre_point;
}  // modify root

}  // namespace planning