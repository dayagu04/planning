#ifndef ZNQC_FRAMEWORK_MODULE_H
#define znqc_FRAMEWORK_MODULE_H

#include <string>
#include <type_traits>

#include "common/class_name.h"
#include "common/macro.h"
#include "common/registry.h"
#include "framework/frame.h"
#include "framework/session.h"
#include "google/protobuf/message.h"

namespace planning {
namespace framework {

class BaseModule {
 public:
  virtual const ::google::protobuf::Message* config() const = 0;

  virtual ::google::protobuf::Message* mutable_config() = 0;

  BaseModule() = default;
  virtual ~BaseModule() = default;

  virtual BaseModule* clone() const = 0;

  virtual int init(const ::google::protobuf::Message* config,
                   Session* session) = 0;
  virtual int reset(const ::google::protobuf::Message* config) = 0;

  // Get/Set the universally unique name of the underlying module. Basically
  // thay are trivial methods, however it is not guaranteed since they would be
  // overridden by derived classes.
  virtual std::string name() const { return name_; };
  virtual void set_name(const std::string& name) { name_ = name; }

 private:
  std::string name_;

  DISALLOW_COPY_AND_ASSIGN(BaseModule);
};

class ModuleConfig : public BaseModule {
 public:
  const ::google::protobuf::Message* config() const override final {
    return config_;
  }

  ::google::protobuf::Message* mutable_config() override final {
    return config_;
  }

 protected:
  ::google::protobuf::Message* config_ = nullptr;
};

/**
 * @brief Interface for frame level planning module
 *
 */
class PlanningModule : public ModuleConfig {
 public:
  // const current cannot be done
  virtual void compute(Frame* frame) = 0;
};

/**
 * @brief Iterface for candidate level planning module
 *
 */
class CandidatePlanningModule : public ModuleConfig {
 public:
  // const current cannot be done
  virtual void compute() = 0;
};

class ModuleFactory {
 public:
  ModuleFactory() = default;
  virtual ~ModuleFactory() = default;
  virtual BaseModule* create_module(const ::google::protobuf::Message* config,
                                    Session* session) const = 0;
  virtual ::google::protobuf::Message* new_config() const = 0;

 private:
  DISALLOW_COPY_AND_ASSIGN(ModuleFactory);
};

bool register_module(const char* name, ModuleFactory* const factory);
bool deregister_module(const char* name, ModuleFactory* const factory);

}  // namespace framework

// used to access ModuleFactory instance of Registry interface
using ModuleFactoryRegistry = planning::common::Registry<planning::framework::ModuleFactory>;

}  // namespace planning

// TODO use malloc, use errno
// TODO don't init here maybe, since parameter not knowned yet
#define REGISTER_MODULE_FACTORY(T)                         \
  class T##Factory : public planning::framework::ModuleFactory {    \
   public:                                                     \
    T##Factory() {                                             \
      const char* class_name = planning::common::class_name<T>();   \
      (void)planning::framework::register_module(class_name, this); \
    }                                                          \
    planning::framework::BaseModule* create_module(                 \
        const ::google::protobuf::Message* config,             \
        planning::framework::Session* session) const override {     \
      if (session == nullptr) {                                \
        return nullptr;                                        \
      }                                                        \
      planning::framework::BaseModule* p = session->alloc<T>();     \
      p->set_name(planning::common::class_name_str<T>());           \
      if (p->init(config, session) != 0) {                     \
        return nullptr;                                        \
      }                                                        \
      return p;                                                \
    }                                                          \
    ::google::protobuf::Message* new_config() const override { \
      return nullptr;                                          \
    }                                                          \
  };                                                           \
  static T##Factory T##factory;

#define ZNQC_REGISTER_MODULE_CREATOR(F)

#endif  // ZNQC_FRAMEWORK_MODULE_H
