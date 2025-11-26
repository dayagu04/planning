#pragma once
#include <cstddef>

#include "node3d.h"
#include "pose2d.h"

namespace planning {

// use 200000 node pool is better for x86，but for arm, 200000 will consume too
// much time.
#define NODE_POOL_MAX_NUM (60000)

// stack is small, so use data segment in memory to store all nodes.
// of course you can store nodes in heap by malloc function.
class CompactNodePool {
 public:
  CompactNodePool() { pool_size_ = 0; }

  void Clear();

  void Init();

  void AddNode();

  Node3d* GetBackNode();

  void PopNode();

  Node3d* AllocateNode();

  Node3d* GetNode(const size_t id);

  size_t PoolSize() { return pool_size_; }

 private:
  size_t pool_size_ = 0;
  Node3d pool_[NODE_POOL_MAX_NUM];
};
}  // namespace planning