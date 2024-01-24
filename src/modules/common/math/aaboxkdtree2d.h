#pragma once

#include <algorithm>
#include <iostream>
#include <limits>
#include <memory>
#include <vector>

#include "aabox2d.h"
#include "line_segment2d.h"
#include "math_utils.h"

namespace planning {
namespace planning_math {

struct AABoxKDTreeParams {
  int max_depth = -1;
  int max_leaf_size = -1;
  double max_leaf_dimension = -1.0;
};

template <class ObjectType>
class AABoxKDTree2dNode {
 public:
  using ObjectPtr = const ObjectType*;

  AABoxKDTree2dNode(const std::vector<ObjectPtr>& objects,
                    const AABoxKDTreeParams& params, int depth)
      : depth_(depth) {
    ComputeBoundary(objects);
    ComputePartition();

    if (SplitToSubNodes(objects, params)) {
      std::vector<ObjectPtr> left_subnode_objects;
      std::vector<ObjectPtr> right_subnode_objects;
      PartitionObjects(objects, &left_subnode_objects, &right_subnode_objects);

      if (!left_subnode_objects.empty()) {
        left_subnode_.reset(new AABoxKDTree2dNode<ObjectType>(
            left_subnode_objects, params, depth + 1));
      }
      if (!right_subnode_objects.empty()) {
        right_subnode_.reset(new AABoxKDTree2dNode<ObjectType>(
            right_subnode_objects, params, depth + 1));
      }
    } else {
      InitObjects(objects);
    }
  }

  ObjectPtr GetNearestObject(const Vec2d& point) const {
    ObjectPtr nearest_object = nullptr;
    double min_distance_sqr = std::numeric_limits<double>::infinity();
    GetNearestObjectInternal(point, &min_distance_sqr, &nearest_object);
    return nearest_object;
  }

  std::vector<ObjectPtr> GetObjects(const Vec2d& point,
                                    const double distance) const {
    std::vector<ObjectPtr> result_objects;
    GetObjectsInternal(point, distance, Square(distance), &result_objects);
    return result_objects;
  }

  AABox2d GetBoundingBox() const {
    return AABox2d({min_x_, min_y_}, {max_x_, max_y_});
  }

 private:
  void InitObjects(const std::vector<ObjectPtr>& objects) {
    num_objects_ = static_cast<int>(objects.size());
    objects_sorted_by_min_ = objects;
    objects_sorted_by_max_ = objects;
    std::sort(objects_sorted_by_min_.begin(), objects_sorted_by_min_.end(),
              [&](ObjectPtr obj1, ObjectPtr obj2) {
                return partition_ == PARTITION_X
                           ? obj1->min_x() < obj2->min_x()
                           : obj1->min_y() < obj2->min_y();
              });
    std::sort(objects_sorted_by_max_.begin(), objects_sorted_by_max_.end(),
              [&](ObjectPtr obj1, ObjectPtr obj2) {
                return partition_ == PARTITION_X
                           ? obj1->max_x() > obj2->max_x()
                           : obj1->max_y() > obj2->max_y();
              });
    objects_sorted_by_min_bound_.reserve(num_objects_);
    for (ObjectPtr object : objects_sorted_by_min_) {
      objects_sorted_by_min_bound_.push_back(
          partition_ == PARTITION_X ? object->min_x() : object->min_y());
    }
    objects_sorted_by_max_bound_.reserve(num_objects_);
    for (ObjectPtr object : objects_sorted_by_max_) {
      objects_sorted_by_max_bound_.push_back(
          partition_ == PARTITION_X ? object->max_x() : object->max_y());
    }
  }

  bool SplitToSubNodes(const std::vector<ObjectPtr>& objects,
                       const AABoxKDTreeParams& params) {
    if (params.max_depth >= 0 && depth_ >= params.max_depth) {
      return false;
    }
    if (static_cast<int>(objects.size()) <= std::max(1, params.max_leaf_size)) {
      return false;
    }
    if (params.max_leaf_dimension >= 0.0 &&
        std::max(max_x_ - min_x_, max_y_ - min_y_) <=
            params.max_leaf_dimension) {
      return false;
    }
    return true;
  }

  double LowerDistanceSquareToPoint(const Vec2d& point) const {
    double dx = 0.0;
    if (point.x() < min_x_) {
      dx = min_x_ - point.x();
    } else if (point.x() > max_x_) {
      dx = point.x() - max_x_;
    }
    double dy = 0.0;
    if (point.y() < min_y_) {
      dy = min_y_ - point.y();
    } else if (point.y() > max_y_) {
      dy = point.y() - max_y_;
    }
    return dx * dx + dy * dy;
  }

  double UpperDistanceSquareToPoint(const Vec2d& point) const {
    const double dx =
        (point.x() > mid_x_ ? (point.x() - min_x_) : (point.x() - max_x_));
    const double dy =
        (point.y() > mid_y_ ? (point.y() - min_y_) : (point.y() - max_y_));
    return dx * dx + dy * dy;
  }

  void GetAllObjects(std::vector<ObjectPtr>* const result_objects) const {
    result_objects->insert(result_objects->end(),
                           objects_sorted_by_min_.begin(),
                           objects_sorted_by_min_.end());
    if (left_subnode_ != nullptr) {
      left_subnode_->GetAllObjects(result_objects);
    }
    if (right_subnode_ != nullptr) {
      right_subnode_->GetAllObjects(result_objects);
    }
  }

  void GetObjectsInternal(const Vec2d& point, const double distance,
                          const double distance_sqr,
                          std::vector<ObjectPtr>* const result_objects) const {
    if (LowerDistanceSquareToPoint(point) > distance_sqr) {
      return;
    }
    if (UpperDistanceSquareToPoint(point) <= distance_sqr) {
      GetAllObjects(result_objects);
      return;
    }
    const double pvalue = (partition_ == PARTITION_X ? point.x() : point.y());
    if (pvalue < partition_position_) {
      const double limit = pvalue + distance;
      for (int i = 0; i < num_objects_; ++i) {
        if (objects_sorted_by_min_bound_[i] > limit) {
          break;
        }
        ObjectPtr object = objects_sorted_by_min_[i];
        if (object->DistanceSquareTo(point) <= distance_sqr) {
          result_objects->push_back(object);
        }
      }
    } else {
      const double limit = pvalue - distance;
      for (int i = 0; i < num_objects_; ++i) {
        if (objects_sorted_by_max_bound_[i] < limit) {
          break;
        }
        ObjectPtr object = objects_sorted_by_max_[i];
        if (object->DistanceSquareTo(point) <= distance_sqr) {
          result_objects->push_back(object);
        }
      }
    }
    if (left_subnode_ != nullptr) {
      left_subnode_->GetObjectsInternal(point, distance, distance_sqr,
                                        result_objects);
    }
    if (right_subnode_ != nullptr) {
      right_subnode_->GetObjectsInternal(point, distance, distance_sqr,
                                         result_objects);
    }
  }

  void GetNearestObjectInternal(const Vec2d& point,
                                double* const min_distance_sqr,
                                ObjectPtr* const nearest_object) const {
    if (LowerDistanceSquareToPoint(point) >= *min_distance_sqr - kMathEpsilon) {
      return;
    }
    const double pvalue = (partition_ == PARTITION_X ? point.x() : point.y());
    const bool search_left_first = (pvalue < partition_position_);
    if (search_left_first) {
      if (left_subnode_ != nullptr) {
        left_subnode_->GetNearestObjectInternal(point, min_distance_sqr,
                                                nearest_object);
      }
    } else {
      if (right_subnode_ != nullptr) {
        right_subnode_->GetNearestObjectInternal(point, min_distance_sqr,
                                                 nearest_object);
      }
    }
    if (*min_distance_sqr <= kMathEpsilon) {
      return;
    }

    if (search_left_first) {
      for (int i = 0; i < num_objects_; ++i) {
        const double bound = objects_sorted_by_min_bound_[i];
        if (bound > pvalue && Square(bound - pvalue) > *min_distance_sqr) {
          break;
        }
        ObjectPtr object = objects_sorted_by_min_[i];
        const double distance_sqr = object->DistanceSquareTo(point);
        if (distance_sqr < *min_distance_sqr) {
          *min_distance_sqr = distance_sqr;
          *nearest_object = object;
        }
      }
    } else {
      for (int i = 0; i < num_objects_; ++i) {
        const double bound = objects_sorted_by_max_bound_[i];
        if (bound < pvalue && Square(bound - pvalue) > *min_distance_sqr) {
          break;
        }
        ObjectPtr object = objects_sorted_by_max_[i];
        const double distance_sqr = object->DistanceSquareTo(point);
        if (distance_sqr < *min_distance_sqr) {
          *min_distance_sqr = distance_sqr;
          *nearest_object = object;
        }
      }
    }
    if (*min_distance_sqr <= kMathEpsilon) {
      return;
    }
    if (search_left_first) {
      if (right_subnode_ != nullptr) {
        right_subnode_->GetNearestObjectInternal(point, min_distance_sqr,
                                                 nearest_object);
      }
    } else {
      if (left_subnode_ != nullptr) {
        left_subnode_->GetNearestObjectInternal(point, min_distance_sqr,
                                                nearest_object);
      }
    }
  }

  void ComputeBoundary(const std::vector<ObjectPtr>& objects) {
    min_x_ = std::numeric_limits<double>::infinity();
    min_y_ = std::numeric_limits<double>::infinity();
    max_x_ = -std::numeric_limits<double>::infinity();
    max_y_ = -std::numeric_limits<double>::infinity();
    for (ObjectPtr object : objects) {
      min_x_ = std::fmin(min_x_, object->min_x());
      max_x_ = std::fmax(max_x_, object->max_x());
      min_y_ = std::fmin(min_y_, object->min_y());
      max_y_ = std::fmax(max_y_, object->max_y());
    }
    mid_x_ = (min_x_ + max_x_) / 2.0;
    mid_y_ = (min_y_ + max_y_) / 2.0;
  }

  void ComputePartition() {
    if (max_x_ - min_x_ >= max_y_ - min_y_) {
      partition_ = PARTITION_X;
      partition_position_ = (min_x_ + max_x_) / 2.0;
    } else {
      partition_ = PARTITION_Y;
      partition_position_ = (min_y_ + max_y_) / 2.0;
    }
  }

  void PartitionObjects(const std::vector<ObjectPtr>& objects,
                        std::vector<ObjectPtr>* const left_subnode_objects,
                        std::vector<ObjectPtr>* const right_subnode_objects) {
    left_subnode_objects->clear();
    right_subnode_objects->clear();
    std::vector<ObjectPtr> other_objects;
    if (partition_ == PARTITION_X) {
      for (ObjectPtr object : objects) {
        if (object->max_x() <= partition_position_) {
          left_subnode_objects->push_back(object);
        } else if (object->min_x() >= partition_position_) {
          right_subnode_objects->push_back(object);
        } else {
          other_objects.push_back(object);
        }
      }
    } else {
      for (ObjectPtr object : objects) {
        if (object->max_y() <= partition_position_) {
          left_subnode_objects->push_back(object);
        } else if (object->min_y() >= partition_position_) {
          right_subnode_objects->push_back(object);
        } else {
          other_objects.push_back(object);
        }
      }
    }
    InitObjects(other_objects);
  }

 private:
  int num_objects_ = 0;
  std::vector<ObjectPtr> objects_sorted_by_min_;
  std::vector<ObjectPtr> objects_sorted_by_max_;
  std::vector<double> objects_sorted_by_min_bound_;
  std::vector<double> objects_sorted_by_max_bound_;
  int depth_ = 0;

  double min_x_ = 0.0;
  double max_x_ = 0.0;
  double min_y_ = 0.0;
  double max_y_ = 0.0;
  double mid_x_ = 0.0;
  double mid_y_ = 0.0;

  enum Partition {
    PARTITION_X = 1,
    PARTITION_Y = 2,
  };
  Partition partition_ = PARTITION_X;
  double partition_position_ = 0.0;

  std::unique_ptr<AABoxKDTree2dNode<ObjectType>> left_subnode_ = nullptr;
  std::unique_ptr<AABoxKDTree2dNode<ObjectType>> right_subnode_ = nullptr;
};

template <class ObjectType>
class AABoxKDTree2d {
 public:
  using ObjectPtr = const ObjectType*;

  AABoxKDTree2d(const std::vector<ObjectType>& objects,
                const AABoxKDTreeParams& params)
      : objects_(std::move(objects)) {
    if (!objects.empty()) {
      std::vector<ObjectPtr> object_ptrs;
      object_ptrs.reserve(objects.size());
      for (const auto& object : objects_) {
        object_ptrs.push_back(&object);
      }
      root_.reset(new AABoxKDTree2dNode<ObjectType>(object_ptrs, params, 0));
    }
  }

  ObjectPtr GetNearestObject(const Vec2d& point) const {
    return root_ == nullptr ? nullptr : root_->GetNearestObject(point);
  }

  std::vector<ObjectPtr> GetObjects(const Vec2d& point,
                                    const double distance) const {
    if (root_ == nullptr) {
      return {};
    }
    return root_->GetObjects(point, distance);
  }

  const std::vector<ObjectType>& objects() const { return objects_; }

  AABox2d GetBoundingBox() const {
    return root_ == nullptr ? AABox2d() : root_->GetBoundingBox();
  }

 private:
  std::unique_ptr<AABoxKDTree2dNode<ObjectType>> root_ = nullptr;
  const std::vector<ObjectType> objects_;
};

}  // namespace planning_math
}  // namespace planning
