#include "sv_graph_node.h"

#include <cmath>

namespace planning {

int32_t SVGraphNode::IndexS() const { return grid_index_.s_index; }

int32_t SVGraphNode::indexV() const { return grid_index_.v_index; }

const SVPoint& SVGraphNode::GetSVPoint() const { return sv_point_; }

const SVGraphNode* SVGraphNode::PrePoint() const { return pre_point_; }

void SVGraphNode::Init(const int32_t index_s, const int32_t index_v,
                       const SVPoint& sv_point, const double speed_limit) {
  grid_index_.v_index = index_v;
  grid_index_.s_index = index_s;
  sv_point_ = sv_point;

  pre_point_ = nullptr;
  cost_.Clear();

  speed_limit_ = speed_limit;

  return;
}

void SVGraphNode::SetTotalCost(const double total_cost) {
  cost_.total_cost = total_cost;

  return;
}

void SVGraphNode::SetPrePoint(const SVGraphNode* pre_point) {
  pre_point_ = pre_point;

  return;
}

double SVGraphNode::TotalCost() const { return cost_.total_cost; }

void SVGraphNode::EvaluateAcc(const SVGraphNode* parent_node,
                              const double dist) {
  sv_point_.acc = 0.0;

  if (parent_node == nullptr) {
    return;
  }

  if (dist > 0.001 || dist < -0.001) {
    sv_point_.acc =
        (sv_point_.v * sv_point_.v -
         parent_node->GetSVPoint().v * parent_node->GetSVPoint().v) /
        dist * 0.5;
  }

  return;
}

void SVGraphNode::EvaluateTime(const SVGraphNode* parent_node, const double dist) {
  if (parent_node == nullptr) {
    sv_point_.t = 0.0;
    return;
  }

  if (std::fabs(sv_point_.acc) < 0.00001) {
    if (std::fabs(parent_node->GetSVPoint().v) < 0.0001) {
      sv_point_.t = 1000.0;
    } else {
      sv_point_.t =
          parent_node->GetSVPoint().t + dist / parent_node->GetSVPoint().v;
    }
  } else {
    sv_point_.t = parent_node->GetSVPoint().t +
                  (sv_point_.v - parent_node->GetSVPoint().v) / sv_point_.acc;
  }

  return;
}

const bool SVGraphNode::IsZeroSpeed() {
  if (sv_point_.v < 1e-3) {
    return true;
  }

  return false;
}

void SVGraphNode::Interpolate(const double s_step, const SVGraphNode* successor,
                              std::vector<SVPoint>& speed_curve) const {
  speed_curve.clear();

  double delta_s = successor->GetSVPoint().s - sv_point_.s;
  if (delta_s < 0.01) {
    speed_curve.push_back(sv_point_);

    return;
  }

  double v_square = sv_point_.v * sv_point_.v;
  double acc =
      (successor->GetSVPoint().v * successor->GetSVPoint().v - v_square) /
      delta_s / 2;

  bool is_constant_speed = true;
  if (std::fabs(acc) > 0.00001) {
    is_constant_speed = false;
  }

  int32_t interpolate_num = std::ceil(delta_s / s_step);
  double cur_s = s_step;
  double node_start_time = sv_point_.t;
  SVPoint point;
  point.acc = acc;
  point.jerk = 0.0;

  speed_curve.push_back(sv_point_);
  for (int32_t i = 1; i < interpolate_num; i++) {
    double interpolate_v = std::sqrt(v_square + 2 * acc * cur_s);
    point.s = sv_point_.s + cur_s;
    point.v = interpolate_v;

    if (!is_constant_speed) {
      point.t = sv_point_.t + (interpolate_v - sv_point_.v) / acc;
    } else {
      if (sv_point_.v < 0.0001) {
        point.t = 1000.0;
      } else {
        point.t = sv_point_.t + cur_s / sv_point_.v;
      }
    }

    if (point.s > successor->GetSVPoint().s - 1e-2) {
      break;
    }

    speed_curve.push_back(point);

    cur_s += s_step;
  }

  return;
}

void SVGraphNode::DebugString() const {
  ILOG_INFO << "index s = " << grid_index_.s_index
            << ",index v = " << grid_index_.v_index << ",s = " << sv_point_.s
            << ",t = " << sv_point_.t << ",v = " << sv_point_.v
            << ",acc = " << sv_point_.acc;
  return;
}

void SVGraphNode::SetZeroCost() {
  cost_.acc_cost = 0.0;
  cost_.speed_limit_cost = 0.0;
  cost_.jerk_cost = 0.0;
  cost_.parent_cost = 0.0;
  cost_.stopover_cost = 0.0;
  cost_.total_cost = 0.0;
  return;
}

void SVGraphNode::EvaluateJerk(const SVGraphNode* parent_node) {

  // node is start
  if (parent_node == nullptr) {
    sv_point_.jerk = 0.0;
    return;
  }

  double delta_time = sv_point_.t - parent_node->GetSVPoint().t;
  double max_jerk = 10000.0;
  if (delta_time < 0.00001) {
    sv_point_.jerk =max_jerk;
  }

  sv_point_.jerk = (sv_point_.acc - parent_node->GetSVPoint().acc) / delta_time;

  return;
}

}  // namespace planning
