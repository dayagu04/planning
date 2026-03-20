#pragma once

#include <set>
#include <algorithm>
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include "ros_convert_inc.h"

void bag_converter(std::string src_bag_path, std::string dst_bag_path) {
    ros::Time::init();
    rosbag::Bag src_bag;
    rosbag::Bag dst_bag;
    src_bag.open(src_bag_path);
    dst_bag.open(dst_bag_path, rosbag::bagmode::Write);
    std::set<std::string> total_topics;
    std::set<std::string> missed_topics;
    std::set<std::string> converted_topics;
    for(rosbag::MessageInstance const m: rosbag::View(src_bag)) {
        total_topics.insert(m.getTopic());
        auto converter = ConvertRegister::GetConverter(m.getTopic());
        if (converter != nullptr) {
            converter(m, dst_bag);
            converted_topics.insert(m.getTopic());
        }
    }
    src_bag.close();
    dst_bag.close();
    ros::Time::shutdown();
    std::cout << "\033[1;32m" << std::endl;
    std::cout << "converted topics : " << converted_topics.size() << std::endl;
    for (auto &topic : converted_topics) {
        std::cout << "\t" << topic << std::endl;
    }
    std::set_difference(total_topics.begin(), total_topics.end(),
                        converted_topics.begin(), converted_topics.end(),
                        std::inserter(missed_topics, missed_topics.end()));
    std::cout << "\033[1;31m" << std::endl;
    std::cout << "missed topics : " << missed_topics.size() << std::endl;
    for (auto &topic : missed_topics) {
        std::cout << "\t" << topic << std::endl;
    }
    std::cout << "\033[0m" << std::endl;
}