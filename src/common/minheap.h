#include <cassert>
#include <unordered_map>
#include <vector>

#include "log.h"

namespace planning {

template <typename KEY, typename VALUE>
class MinHeap {
 public:
  MinHeap() = default;
  ~MinHeap() = default;

  MinHeap(const std::vector<std::pair<KEY, VALUE>> &inputs) {
    for (size_t i = 0; i < inputs.size(); ++i) {
      index_map_[inputs[i].first] = i;
      array_.push_back(std::move(inputs[i]));
      size_++;
    }
    Heapify();
  }

  bool IsEmpty() const { return size_ == 0; }

  size_t Size() const { return size_; }

  const std::pair<KEY, VALUE> &Top() const {
    assert(!IsEmpty());
    if (IsEmpty()) {
      // throw Status(StatusCode::COMMON_ERROR, "heap is empty, cann't get
      // top");
      LOG_ERROR("heap is empty, cann't get top");
    }
    return array_[0];
  }

  void Pop() {
    assert(!IsEmpty());
    if (IsEmpty()) {
      // throw Status(StatusCode::COMMON_ERROR, "heap is empty, cann't pop");
      LOG_ERROR("heap is empty, cann't get top");
    }
    Swap(0, size_ - 1);
    index_map_.erase(array_[size_ - 1].first);
    array_.pop_back();
    size_--;
    PercolateDown(0);
  }

  bool IsInHeap(const KEY &key) const { return index_map_.count(key) != 0; }

  void Push(std::pair<KEY, VALUE> &input) { Push(input.first, input.second); }

  void Push(const KEY &key, const VALUE &value) {
    if (IsInHeap(key)) {
      Update(key, value);
    } else {
      array_.emplace_back(key, value);
      size_++;
      index_map_[key] = size_ - 1;
      PercolateUp(size_ - 1);
    }
  }

  void Update(std::pair<KEY, VALUE> &input) {
    Update(input.first, input.second);
  }

  bool Update(const KEY &key, const VALUE &value) {
    if (!IsInHeap(key)) {
      return false;
    }
    size_t index = index_map_.at(key);
    array_[index].second = value;
    if (index == 0) {
      PercolateDown(index);
      return true;
    }
    size_t parent_idx = (index - 1) / 2;
    if (array_[index].second < array_[parent_idx].second) {
      PercolateUp(index);
    } else {
      PercolateDown(index);
    }
    return true;
  }

  const std::vector<std::pair<KEY, VALUE>> &array() const { return array_; }

  void Clear() {
    array_.clear();
    index_map_.clear();
    size_ = 0;
  }

 private:
  void Heapify() {
    // node's parent index is (ele_index - 1) / 2, last node's index is (size_ -
    // 1) (size_ - 2) / 2 is the index of the last node that has children
    if (size_ < 2) {
      return;
    }
    for (int i = (size_ - 2) / 2; i >= 0; --i) {
      PercolateDown(i);
    }
  }

  void PercolateDown(size_t index) {
    while (2 * index + 1 < size_) {
      size_t left = 2 * index + 1;
      size_t right = 2 * index + 2;
      size_t swap_candidate = left;
      if (right < size_ && array_[right].second < array_[left].second) {
        swap_candidate = right;
      }
      if (array_[index].second < array_[swap_candidate].second) {
        break;
      }
      Swap(index, swap_candidate);
      index = swap_candidate;
    }
  }

  void PercolateUp(size_t index) {
    while (index > 0) {
      size_t parent_index = (index - 1) / 2;
      if (array_[parent_index].second < array_[index].second) {
        break;
      }
      Swap(index, parent_index);
      index = parent_index;
    }
  }

  void Swap(const size_t index0, const size_t index1) {
    index_map_[array_[index0].first] = index1;
    index_map_[array_[index1].first] = index0;
    std::swap(array_[index0], array_[index1]);
  }

 private:
  std::vector<std::pair<KEY, VALUE>> array_;
  std::unordered_map<KEY, size_t> index_map_;
  size_t size_ = 0;
};

}  // namespace planning