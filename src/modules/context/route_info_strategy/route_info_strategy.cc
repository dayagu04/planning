# include "route_info_strategy.h"
#include "config/basic_type.h"

namespace planning {
  RouteInfoStrategy::RouteInfoStrategy(const MLCDeciderConfig* mlc_decider_config,
                    const planning::framework::Session* session)
      : mlc_decider_config_(mlc_decider_config),  // 初始化const成员config_builder_
        session_(session) {}              // 初始化const成员session_

RampDirection RouteInfoStrategy::CalculateSplitDirection(
    const iflymapdata::sdpro::LinkInfo_Link& split_link,
    const ad_common::sdpromap::SDProMap& sdpro_map) const{
  RampDirection ramp_dir = RAMP_NONE;

  const auto& out_link_size = split_link.successor_link_ids().size();
  const auto& out_link = split_link.successor_link_ids();

  // 二分叉
  if (out_link_size == 2) {
    const auto split_next_link = sdpro_map.GetNextLinkOnRoute(split_link.id());
    if (!split_next_link) {
      return ramp_dir;
    }

    ad_common::math::Vec2d segment_in_route_dir_vec;
    ad_common::math::Vec2d segment_not_in_route_dir_vec;
    Point2D anchor_point_of_cur_seg_to_next_seg = {
        split_link.points().boot().points().rbegin()->x(),
        split_link.points().boot().points().rbegin()->y()};

    auto other_link_id =
        out_link[0] == split_next_link->id() ? out_link[1] : out_link[0];
    const auto& other_link = sdpro_map.GetLinkOnRoute(other_link_id);
    if (other_link == nullptr) {
      return ramp_dir;
    }

    const auto& split_next_segment_enu_point =
        split_next_link->points().boot().points();
    const auto& other_segment_enu_point = other_link->points().boot().points();
    if (split_next_segment_enu_point.size() > 1 &&
        other_segment_enu_point.size() > 1) {
      segment_in_route_dir_vec.set_x(split_next_segment_enu_point[1].x() -
                                     anchor_point_of_cur_seg_to_next_seg.x);
      segment_in_route_dir_vec.set_y(split_next_segment_enu_point[1].y() -
                                     anchor_point_of_cur_seg_to_next_seg.y);
      segment_not_in_route_dir_vec.set_x(other_segment_enu_point[1].x() -
                                         anchor_point_of_cur_seg_to_next_seg.x);
      segment_not_in_route_dir_vec.set_y(other_segment_enu_point[1].y() -
                                         anchor_point_of_cur_seg_to_next_seg.y);
      if (segment_in_route_dir_vec.CrossProd(segment_not_in_route_dir_vec) >
          0.0) {
        ramp_dir = RampDirection::RAMP_ON_RIGHT;
      } else {
        ramp_dir = RampDirection::RAMP_ON_LEFT;
      }
    } else {
      ILOG_WARN << "enu points error!!!!!!!!!!";
    }
  } else if (out_link_size == 3) {
    // 三分叉
    const auto split_next_link = sdpro_map.GetNextLinkOnRoute(split_link.id());
    if (!split_next_link) {
      return ramp_dir;
    }

    std::vector<uint64> other_link_ids;
    other_link_ids.reserve(2);

    for (int i = 0; i < out_link.size(); ++i) {
      if (out_link[i] != split_next_link->id()) {
        other_link_ids.emplace_back(out_link[i]);
      }
    }

    const auto& other_link1 = sdpro_map.GetLinkOnRoute(other_link_ids[0]);
    const auto& other_link2 = sdpro_map.GetLinkOnRoute(other_link_ids[1]);
    if (other_link1 == nullptr || other_link2 == nullptr ||
        other_link1->points().boot().points().size() < 2 ||
        other_link2->points().boot().points().size() < 2) {
      return ramp_dir;
    }

    // 目标车道的point
    Point2D O{split_next_link->points().boot().points()[0].x(),
              split_next_link->points().boot().points()[0].y()};
    Point2D L{split_next_link->points().boot().points()[1].x(),
              split_next_link->points().boot().points()[1].y()};

    // 另外两条link的point；
    Point2D other1{other_link1->points().boot().points()[1].x(),
                   other_link1->points().boot().points()[1].y()};
    Point2D other2{other_link2->points().boot().points()[1].x(),
                   other_link2->points().boot().points()[1].y()};

    double OL = CalculateAngle(O, L);
    double Oother1 = CalculateAngle(O, other1);
    double Oother2 = CalculateAngle(O, other2);

    std::vector<RayInfo> rays = {{'A', OL}, {'B', Oother1}, {'C', Oother2}};

    const auto& result = SortRaysByDirection(rays);

    for (int i = 0; i < result.size(); i++) {
      if (result[i] == 'A') {
        if (i == 0) {
          ramp_dir = RAMP_ON_RIGHT;
        } else if (i == 1) {
          ramp_dir = RAMP_ON_MIDDLE;
        } else if (i == 2) {
          ramp_dir = RAMP_ON_LEFT;
        }
      }
    }
  } else {
    return ramp_dir;
  }
  return ramp_dir;
}

double RouteInfoStrategy::CalculateAngle(const Point2D& o, const Point2D& p) const{
  double dx = p.x - o.x;
  double dy = p.y - o.y;
  double angle = std::atan2(dy, dx);  // 相对于x轴正方向的夹角
  if (angle < 0) {
    angle += 2 * M_PI;  // 转换到0到2π
  }
  return angle;
}

std::vector<char> RouteInfoStrategy::SortRaysByDirection(
    const std::vector<RayInfo>& rays) const{
  if (rays.empty()) return {};

  // 找到最小夹角
  double min_angle = rays[0].angle;
  for (const auto& ray : rays) {
    if (ray.angle < min_angle) {
      min_angle = ray.angle;
    }
  }

  // 转换到以min_angle为起点的坐标系
  std::vector<std::pair<double, char>> shifted_rays;
  for (const auto& ray : rays) {
    double shifted_angle = ray.angle - min_angle;
    // 如果shifted_angle超过π，转换到-π到π范围
    if (shifted_angle > M_PI) {
      shifted_angle -= 2 * M_PI;
    }
    shifted_rays.emplace_back(shifted_angle, ray.name);
  }

  // 按转换后的角度排序
  std::sort(shifted_rays.begin(), shifted_rays.end(),
            [](const auto& a, const auto& b) { return a.first < b.first; });

  // 提取排序后的射线名称
  std::vector<char> result;
  for (const auto& ray : shifted_rays) {
    result.push_back(ray.second);
  }
  return result;
}

RampDirection RouteInfoStrategy::CalculateMergeDirection(
    const iflymapdata::sdpro::LinkInfo_Link& merge_link,
    const ad_common::sdpromap::SDProMap& sdpro_map) const{
  const auto in_link_size = merge_link.predecessor_link_ids().size();
  RampDirection merge_direction = RAMP_NONE;
  const auto& in_link = merge_link.predecessor_link_ids();
  // fengwang31(TODO):暂时假设在merge处只有两个方向
  if (in_link_size == 2) {
    const auto merge_last_segment =
        sdpro_map.GetPreviousLinkOnRoute(merge_link.id());
    if (!merge_last_segment) {
      ILOG_WARN << "in segment is nullptr!!!!!!!!";
      return merge_direction;
    }

    ad_common::math::Vec2d segment_in_route_dir_vec;
    ad_common::math::Vec2d segment_not_in_route_dir_vec;
    Point2D anchor_point_of_cur_seg_to_last_seg = {
        merge_link.points().boot().points().begin()->x(),
        merge_link.points().boot().points().begin()->y()};

    auto other_link_id =
        in_link[0] == merge_last_segment->id() ? in_link[1] : in_link[0];
    const auto& other_link = sdpro_map.GetLinkOnRoute(other_link_id);
    if (other_link == nullptr) {
      return merge_direction;
    }

    const auto& merge_last_segment_enu_point =
        merge_last_segment->points().boot().points();
    const auto& other_segment_enu_point = other_link->points().boot().points();
    const int point_num = merge_last_segment_enu_point.size();
    const int other_point_num = other_segment_enu_point.size();

    if (point_num > 1 && other_point_num > 1) {
      segment_in_route_dir_vec.set_x(
          merge_last_segment_enu_point[point_num - 2].x() -
          anchor_point_of_cur_seg_to_last_seg.x);
      segment_in_route_dir_vec.set_y(
          merge_last_segment_enu_point[point_num - 2].y() -
          anchor_point_of_cur_seg_to_last_seg.y);
      segment_not_in_route_dir_vec.set_x(
          other_segment_enu_point[other_point_num - 2].x() -
          anchor_point_of_cur_seg_to_last_seg.x);
      segment_not_in_route_dir_vec.set_y(
          other_segment_enu_point[other_point_num - 2].y() -
          anchor_point_of_cur_seg_to_last_seg.y);
      if (segment_in_route_dir_vec.CrossProd(segment_not_in_route_dir_vec) >
          0.0) {
        merge_direction = RampDirection::RAMP_ON_LEFT;
      } else {
        merge_direction = RampDirection::RAMP_ON_RIGHT;
      }
    } else {
      ILOG_WARN << "enu points error!!!!!!!!!!";
    }
  } else {
    ILOG_WARN << "out_link_size != 2!!!!!!!";
  }
  return merge_direction;
}

}