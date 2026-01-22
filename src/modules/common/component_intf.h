#pragma once

#include <memory>
#include <string>
#include <unordered_map>

namespace iflyauto {
namespace interface {
class ComponentInterface {
 public:
  ComponentInterface() = default;
  virtual ~ComponentInterface() = default;

  virtual bool Init() = 0;
  virtual bool Proc() = 0;

  virtual void SetParam(std::string key, std::string value) {
    params_[key] = value;
  }

 protected:
  std::unordered_map<std::string, std::string> params_;
};

using ComponentPtr = std::shared_ptr<ComponentInterface>;
class ComponentRegistry {
 public:
  static ComponentRegistry *GetInstance() {
    static ComponentRegistry instance;
    return &instance;
  }
  ComponentPtr GetComponent(std::string name) {
    if (components_.find(name) == components_.end()) {
      return nullptr;
    }
    return components_[name];
  }
  void SetComponent(std::string name, ComponentPtr component) {
    components_[name] = component;
  }

 private:
  std::unordered_map<std::string, ComponentPtr> components_;
};

#define REG_COMPONENT(name, comp)                              \
  namespace {                                                  \
  using namespace iflyauto::interface;                         \
  struct ComponentRegister {                                   \
    ComponentRegister() {                                      \
      auto instance = ComponentRegistry::GetInstance();        \
      instance->SetComponent(#name, std::make_shared<comp>()); \
    }                                                          \
  };                                                           \
  static ComponentRegister g_##name;                           \
  }
}  // namespace interface
}  // namespace iflyauto