#ifndef MODULES_PLANNING_OPTIMIZERS_INDEX_LIST_H_
#define MODULES_PLANNING_OPTIMIZERS_INDEX_LIST_H_

#include <unordered_map>
#include <vector>

//#include "boost/thread/shared_mutex.hpp"
//#include "google/protobuf/stubs/map_util.h"
#include "map_util.h"
#include <iostream>

namespace planning {

// Expose some useful utils from protobuf.
// Find*()
using google::protobuf::FindCopy;
using google::protobuf::FindLinkedPtrOrDie;
using google::protobuf::FindLinkedPtrOrNull;
using google::protobuf::FindOrDie;
using google::protobuf::FindOrDieNoPrint;
using google::protobuf::FindOrNull;
using google::protobuf::FindPtrOrNull;
using google::protobuf::FindWithDefault;

// Contains*()
using google::protobuf::ContainsKey;
using google::protobuf::ContainsKeyValuePair;

// Insert*()
using google::protobuf::InsertAndDeleteExisting;
using google::protobuf::InsertIfNotPresent;
using google::protobuf::InsertKeyOrDie;
using google::protobuf::InsertOrDie;
using google::protobuf::InsertOrDieNoPrint;
using google::protobuf::InsertOrUpdate;
using google::protobuf::InsertOrUpdateMany;

// Lookup*()
using google::protobuf::AddTokenCounts;
using google::protobuf::LookupOrInsert;
using google::protobuf::LookupOrInsertNew;
using google::protobuf::LookupOrInsertNewLinkedPtr;
using google::protobuf::LookupOrInsertNewSharedPtr;

// Misc Utility Functions
using google::protobuf::AppendKeysFromMap;
using google::protobuf::AppendValuesFromMap;
using google::protobuf::EraseKeyReturnValuePtr;
using google::protobuf::InsertKeysFromMap;
using google::protobuf::InsertOrReturnExisting;
using google::protobuf::UpdateReturnCopy;

template <typename I, typename T> class IndexedList {
public:
  /**
   * @brief copy object into the container. If the id is already exist,
   * overwrite the object in the container.
   * @param id the id of the object
   * @param object the const reference of the objected to be copied to the
   * container.
   * @return The pointer to the object in the container.
   */
  T *Add(const I id, const T &object) {
    auto obs = Find(id);
    if (obs) {
      std::cout << "object " << id << " is already in container" << std::endl;
      *obs = object;
      return obs;
    } else {
      object_dict_.insert({id, object});
      auto *ptr = &object_dict_.at(id);
      object_list_.push_back(ptr);
      return ptr;
    }
  }

  /**
   * @brief Find object by id in the container
   * @param id the id of the object
   * @return the raw pointer to the object if found.
   * @return nullptr if the object is not found.
   */
  T *Find(const I id) { return FindOrNull(object_dict_, id); }

  /**
   * @brief Find object by id in the container
   * @param id the id of the object
   * @return the raw pointer to the object if found.
   * @return nullptr if the object is not found.
   */
  const T *Find(const I id) const { return FindOrNull(object_dict_, id); }

  /**
   * @brief List all the items in the container.
   * @return the list of const raw pointers of the objects in the container.
   */
  const std::vector<const T *> &Items() const { return object_list_; }

  /**
   * @brief List all the items in the container.
   * @return the unordered_map of ids and objects in the container.
   */
  const std::unordered_map<I, T> &Dict() const { return object_dict_; }

  /**
   * @brief Copy the container with objects.
   */
  IndexedList &operator=(const IndexedList &other) {
    this->object_list_.clear();
    this->object_dict_.clear();
    for (const auto &item : other.Dict()) {
      Add(item.first, item.second);
    }
    return *this;
  }

private:
  std::vector<const T *> object_list_;
  std::unordered_map<I, T> object_dict_;
};

// TODO(shike): replace boost with std
// template <typename I, typename T>
// class ThreadSafeIndexedList : public IndexedList<I, T> {
// public:
//  T *Add(const I id, const T &object) {
//    boost::unique_lock<boost::shared_mutex> writer_lock(mutex_);
//    return IndexedList<I, T>::Add(id, object);
//  }
//
//  T *Find(const I id) {
//    boost::shared_lock<boost::shared_mutex> reader_lock(mutex_);
//    return IndexedList<I, T>::Find(id);
//  }
//
//  std::vector<const T *> Items() const {
//    boost::shared_lock<boost::shared_mutex> reader_lock(mutex_);
//    return IndexedList<I, T>::Items();
//  }
//
// private:
//  mutable boost::shared_mutex mutex_;
//};

} // namespace planning

#endif /* MODULES_PLANNING_OPTIMIZERS_INDEX_LIST_H_ */
