#pragma once

#include <set>
#include <functional>
#include <unordered_map>
#include <rosbag/bag.h>
#include <rosbag/view.h>

#define REG_CONVERT_DIRECT(name, topic)                                                             \
ConvertRegister name(topic, [](rosbag::MessageInstance const m, rosbag::Bag &dst_bag){              \
  dst_bag.write(topic, m.getTime(), m);                                                             \
})
#define REG_CONVERT_SINGLE(name, topic, type) REG_CONVERT_DOUBLE(name, topic, topic, struct_msgs_v2_10::type, struct_msgs::type)
#define REG_CONVERT_LEGACY(name, topic, type) REG_CONVERT_DOUBLE(name, topic, topic, struct_msgs_v2_10::type, struct_msgs_legacy_v2_5_0::type)
#define REG_CONVERT_DOUBLE(name, topic_src, topic_dst, type_src, type_dst)                             \
ConvertRegister name(topic_src, [](rosbag::MessageInstance const m, rosbag::Bag &dst_bag){  \
  auto src_ros = m.instantiate<type_src>();                                             \
  if (src_ros == nullptr) { return; }                                                   \
  auto dst_ros = std::make_shared<type_dst>();                                          \
  convert(*src_ros, *dst_ros, ConvertTypeInfo::TO_ROS);                                 \
  dst_bag.write(topic_dst, m.getTime(), *dst_ros);                                          \
})

using Converter = std::function<void(rosbag::MessageInstance const, rosbag::Bag &)>;
class ConvertRegistry {
public:
    static ConvertRegistry *GetInstance() {
        static ConvertRegistry instance;
        return &instance;
    }
    Converter GetConverter(std::string topic) {
        if (converters_.find(topic) == converters_.end()) {
            return nullptr;
        }
        return converters_[topic];
    }
    void SetConverter(std::string topic, Converter converter) {
        converters_[topic] = converter;
    }

private:
    std::map<std::string, Converter> converters_;
};    

class ConvertRegister {
public:
    ConvertRegister(std::string topic, Converter converter) {
        auto instance = ConvertRegistry::GetInstance();
        instance->SetConverter(topic, converter);
    }
    ~ConvertRegister(){};
    static Converter GetConverter(std::string topic) {
        auto instance = ConvertRegistry::GetInstance();
        auto converter = instance->GetConverter(topic);
        return converter;
    }
};
