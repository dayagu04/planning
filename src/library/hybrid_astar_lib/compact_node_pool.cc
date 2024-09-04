#include "compact_node_pool.h"
#include <cstddef>

namespace planning {

void CompactNodePool::AddNode() {
  pool_size_++;

  return;
}

void CompactNodePool::Clear() {
  pool_size_ = 0;
  return;
}

void CompactNodePool::Init() {
  pool_size_ = 0;
  return;
}

Node3d* CompactNodePool::GetBackNode() {
  Node3d* node = nullptr;

  if (pool_size_ < NODE_POOL_MAX_NUM) {
    node = &pool_[pool_size_];
  }

  return node;
}

void CompactNodePool::PopNode() {
  if (pool_size_ > 0) {
    pool_size_--;
  }

  return;
}

Node3d* CompactNodePool::AllocateNode() {
  Node3d* node = nullptr;

  if (pool_size_ < NODE_POOL_MAX_NUM) {
    node = &pool_[pool_size_];
    node->Clear();

    pool_size_++;
  }

  return node;
}

Node3d* CompactNodePool::GetNode(const size_t id) {
  if (id < pool_size_) {
    return &pool_[id];
  }
  return nullptr;
}

size_t CompactNodePool::pool_size_ = 0;
Node3d CompactNodePool::pool_[NODE_POOL_MAX_NUM];

}  // namespace planning