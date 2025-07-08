
// set cost on st point
#include <cstdint>
#include <limits>
#include "speed/st_point.h"
namespace planning {
class STGraphPoint{
  public:
    uint32_t index_s() const;
    uint32_t index_t() const;
    double reference_cost() const;
    double obstacle_cost() const;
    double total_cost() const;
    const STPoint& point() const; // avoid copy
    const STGraphPoint* pre_point()const;
    // set const memeber
    void Init(const uint32_t index_t, const uint32_t index_s,const STPoint& st_point);
    //set mutable value
    void SetReferenceCost(const double reference_cost);
    void SetObstacleCost(const double obstacle_cost);
    void SetTotalCost(const double total_cost);
    void SetPrePoint(const STGraphPoint& pre_point);

  private:
    STPoint point_;
    const STGraphPoint* pre_point_ = nullptr; // point diff obj but not modify
    uint32_t index_s_ = 0;
    uint32_t index_t_ = 0;
    double reference_cost_ = 0;
    double obstacle_cost_= 0;
    double total_cost_ = std::numeric_limits<double>::infinity();
};
}