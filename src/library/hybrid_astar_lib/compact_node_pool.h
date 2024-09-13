#pragma once
#include <cstddef>

#include "node3d.h"
#include "pose2d.h"

namespace planning {

#define NODE_POOL_MAX_NUM (50000)

// stack is small, so use data segment in memory to store all nodes.
// of course you can store nodes in heap by malloc function.
class CompactNodePool {
 public:
  CompactNodePool() { pool_size_ = 0; }

  static void Clear();

  static void Init();

  static void AddNode();

  static Node3d* GetBackNode();

  static void PopNode();

  static Node3d* AllocateNode();

  static Node3d* GetNode(const size_t id);

  static size_t PoolSize() { return pool_size_; }

 private:
  static size_t pool_size_;
  static Node3d pool_[NODE_POOL_MAX_NUM];
};
}  // namespace planning