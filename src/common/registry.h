#ifndef ZNQC_COMMON_REGISTRY_H
#define ZNQC_COMMON_REGISTRY_H

#include <cstring>
#include <iterator>
#include <map>
#include <sstream>

#include "common/log.h"

namespace planning {
namespace common {

struct StringLesser {
  bool operator()(const char *lhs, const char *rhs) const {
    return strcmp(lhs, rhs) < 0;
  }
};

// Stores all modules' factories
// Static storage so that factories can be registered early

template <typename T>
class Registry {
 public:
  using FactoryMap = std::map<const char *, T *, StringLesser>;

  Registry() = default;
  ~Registry() = default;

  // insert module factory into static map before main()
  static bool insert_factory(const char *name, T *factory) {
    // module already inserted, fail
    if (factory_map_.find(name) != factory_map_.end()) {
      LOG_ERROR("factory already exist: name=%s", name);
      return false;
    }

    factory_map_[name] = factory;
    LOG_DEBUG("factory inserted: name=%s", name);
    return true;
  }

  static bool erase_factory(const char *name, T *factory) {
    // factory never inserted, fail
    auto iter = factory_map_.find(name);
    if (iter == factory_map_.end()) {
      LOG_ERROR("factory not found: name=%s", name);
      return false;
    }

    // factory to delete doesn't match
    if (iter->second != factory) {
      LOG_ERROR("factory not match: name=%s", name);
      return false;
    }

    return factory_map_.erase(name);
  }

  static T *get_factory(const char *name) {
    auto iter = factory_map_.find(name);
    if (iter == factory_map_.end()) {
      LOG_ERROR("factory not found: name=%s", name);
      return nullptr;
    }

    return iter->second;
  }

  static std::string dump(void) {
    std::stringstream ss;
    for (auto &&ele : factory_map_) {
      ss << ele.first << ",";
    }
    return ss.str();
  }

  // support range-based for loop
  static auto begin() { return factory_map_.begin(); }
  static auto end() { return factory_map_.end(); }

 private:
  // need to initialize template where used.
  static FactoryMap factory_map_;
};

}  // namespace common
}  // namespace planning

#endif  // ZNQC_COMMON_REGISTRY_H
