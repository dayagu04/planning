#include "collision_detection.h"

namespace planning {

static const std::vector<double> car_circle_x_vec = {};
static const std::vector<double> car_circle_y_vec = {};
static const std::vector<double> car_circle_radius_vec = {};

void CollisionDetector::Init() {}
void CollisionDetector::Reset() {}

void CollisionDetector::GenObstacles() {}
void CollisionDetector::GenCarCircles() {}

bool CollisionDetector::CollisionDetect(std::vector<Eigen::Vector2d> &pos_vec,
                                        std::vector<double> &theta_vec) {}

};  // namespace planning