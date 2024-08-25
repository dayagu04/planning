#include "proto_convert.h"

namespace planning {
namespace planning_player {

/*
void StaticMapToProto(Map::StaticMap &obj, proto_msgs::StaticMap &msg) {
  // static start
  // header
  Common::Header *header = obj.mutable_header();
  header->set_timestamp(msg.msg_header.stamp);
  header->set_version(msg.msg_header.version);
  // road_map
  Map::RoadMap *road_map = obj.mutable_road_map();
  road_map->set_timestamp(msg.road_map.timestamp);
  for (size_t i = 0; i < msg.road_map.lanes.size(); i++) {
    Map::LaneData *lanes = road_map->add_lanes();
    lanes->set_lane_id(msg.road_map.lanes[i].lane_id);
    lanes->set_sequence_id(msg.road_map.lanes[i].sequence_id);
    for (size_t j = 0; j < msg.road_map.lanes[i].points_on_central_line.size();
         j++) {
      Common::Point3d *points_on_central_line =
          lanes->add_points_on_central_line();
      points_on_central_line->set_x(
          msg.road_map.lanes[i].points_on_central_line[j].x);
      points_on_central_line->set_y(
          msg.road_map.lanes[i].points_on_central_line[j].y);
      points_on_central_line->set_z(
          msg.road_map.lanes[i].points_on_central_line[j].z);
    }

    for (size_t j = 0; j < msg.road_map.lanes[i].central_points_offset.size();
         j++) {
      lanes->add_central_points_offset(
          msg.road_map.lanes[i].central_points_offset[j]);
    }
    for (size_t j = 0; j < msg.road_map.lanes[i].width.size(); j++) {
      lanes->add_width(msg.road_map.lanes[i].width[j]);
    }
    for (size_t j = 0; j < msg.road_map.lanes[i].curvature.size(); j++) {
      lanes->add_curvature(msg.road_map.lanes[i].curvature[j]);
    }
    for (size_t j = 0; j < msg.road_map.lanes[i].heading.size(); j++) {
      lanes->add_heading(msg.road_map.lanes[i].heading[j]);
    }

    lanes->set_lane_direction(
        Map::Direction(msg.road_map.lanes[i].lane_direction));
    lanes->set_lane_type(Map::LaneType(msg.road_map.lanes[i].lane_type));
    lanes->set_left_lane_boundary_id(
        msg.road_map.lanes[i].left_lane_boundary_id);
    lanes->set_right_lane_boundary_id(
        msg.road_map.lanes[i].right_lane_boundary_id);
    lanes->set_left_road_boundary_id(
        msg.road_map.lanes[i].left_road_boundary_id);
    lanes->set_right_road_boundary_id(
        msg.road_map.lanes[i].right_road_boundary_id);
    lanes->set_min_speed_limit(msg.road_map.lanes[i].min_speed_limit);
    lanes->set_max_speed_limit(msg.road_map.lanes[i].max_speed_limit);

    for (size_t j = 0; j < msg.road_map.lanes[i].predecessor_lane_id.size();
         j++) {
      lanes->add_predecessor_lane_id(
          msg.road_map.lanes[i].predecessor_lane_id[j]);
    }
    for (size_t j = 0; j < msg.road_map.lanes[i].successor_lane_id.size();
         j++) {
      lanes->add_successor_lane_id(msg.road_map.lanes[i].successor_lane_id[j]);
    }

    lanes->set_left_neighbor_forward_lane_id(
        msg.road_map.lanes[i].left_neighbor_forward_lane_id);
    lanes->set_right_neighbor_forward_lane_id(
        msg.road_map.lanes[i].right_neighbor_forward_lane_id);
    lanes->set_left_neighbor_reverse_lane_id(
        msg.road_map.lanes[i].left_neighbor_reverse_lane_id);
    lanes->set_right_neighbor_reverse_lane_id(
        msg.road_map.lanes[i].right_neighbor_reverse_lane_id);
    lanes->set_lane_group_id(msg.road_map.lanes[i].lane_group_id);

    for (size_t j = 0; j < msg.road_map.lanes[i].overlaps.size(); j++) {
      Map::OverLap *overlaps = lanes->add_overlaps();
      overlaps->set_id(msg.road_map.lanes[i].overlaps[j].id);
      overlaps->set_type(msg.road_map.lanes[i].overlaps[j].type);
      overlaps->set_s_begin(msg.road_map.lanes[i].overlaps[j].s_begin);
      overlaps->set_s_end(msg.road_map.lanes[i].overlaps[j].s_end);
    }
  }
  for (size_t i = 0; i < msg.road_map.lane_boundaries.size(); i++) {
    Map::LaneBoundary *lane_boundaries = road_map->add_lane_boundaries();
    lane_boundaries->set_boundary_id(
        msg.road_map.lane_boundaries[i].boundary_id);
    for (size_t j = 0;
         j < msg.road_map.lane_boundaries[i].boundary_attributes.size(); j++) {
      Map::BoundaryAttributes *boundary_attributes =
          lane_boundaries->add_boundary_attributes();
      for (size_t k = 0;
           k <
           msg.road_map.lane_boundaries[i].boundary_attributes[j].points.size();
           k++) {
        Common::Point3d *points = boundary_attributes->add_points();
        points->set_x(
            msg.road_map.lane_boundaries[i].boundary_attributes[j].points[k].x);
        points->set_y(
            msg.road_map.lane_boundaries[i].boundary_attributes[j].points[k].y);
        points->set_z(
            msg.road_map.lane_boundaries[i].boundary_attributes[j].points[k].z);
      }

      for (size_t k = 0; k < msg.road_map.lane_boundaries[i]
                                 .boundary_attributes[j]
                                 .points_s.size();
           k++) {
        boundary_attributes->add_points_s(
            msg.road_map.lane_boundaries[i].boundary_attributes[j].points_s[k]);
      }

      for (size_t k = 0;
           k <
           msg.road_map.lane_boundaries[i].boundary_attributes[j].types.size();
           k++) {
        boundary_attributes->add_types(Map::BoundaryAttributes::Type(
            msg.road_map.lane_boundaries[i].boundary_attributes[j].types[k]));
      }

      for (size_t k = 0;
           k <
           msg.road_map.lane_boundaries[i].boundary_attributes[j].colors.size();
           k++) {
        boundary_attributes->add_colors(Map::BoundaryAttributes::Color(
            msg.road_map.lane_boundaries[i].boundary_attributes[j].colors[k]));
      }
    }
  }

  for (size_t i = 0; i < msg.road_map.road_boundaries.size(); i++) {
    Map::LaneBoundary *road_boundaries = road_map->add_road_boundaries();
    road_boundaries->set_boundary_id(
        msg.road_map.road_boundaries[i].boundary_id);
    for (size_t j = 0;
         j < msg.road_map.road_boundaries[i].boundary_attributes.size(); j++) {
      Map::BoundaryAttributes *boundary_attributes =
          road_boundaries->add_boundary_attributes();
      for (size_t k = 0;
           k <
           msg.road_map.road_boundaries[i].boundary_attributes[j].points.size();
           k++) {
        Common::Point3d *points = boundary_attributes->add_points();
        points->set_x(
            msg.road_map.road_boundaries[i].boundary_attributes[j].points[k].x);
        points->set_y(
            msg.road_map.road_boundaries[i].boundary_attributes[j].points[k].y);
        points->set_z(
            msg.road_map.road_boundaries[i].boundary_attributes[j].points[k].z);
      }

      for (size_t k = 0; k < msg.road_map.road_boundaries[i]
                                 .boundary_attributes[j]
                                 .points_s.size();
           k++) {
        boundary_attributes->add_points_s(
            msg.road_map.road_boundaries[i].boundary_attributes[j].points_s[k]);
      }

      for (size_t k = 0;
           k <
           msg.road_map.road_boundaries[i].boundary_attributes[j].types.size();
           k++) {
        boundary_attributes->add_types(Map::BoundaryAttributes::Type(
            msg.road_map.road_boundaries[i].boundary_attributes[j].types[k]));
      }

      for (size_t k = 0;
           k <
           msg.road_map.road_boundaries[i].boundary_attributes[j].colors.size();
           k++) {
        boundary_attributes->add_colors(Map::BoundaryAttributes::Color(
            msg.road_map.road_boundaries[i].boundary_attributes[j].colors[k]));
      }
    }
  }

  for (size_t i = 0; i < msg.road_map.lane_groups.size(); i++) {
    Map::LaneGroup *lane_groups = road_map->add_lane_groups();
    lane_groups->set_lane_group_id(msg.road_map.lane_groups[i].lane_group_id);
    lane_groups->set_length(msg.road_map.lane_groups[i].length);
    for (size_t j = 0; j < msg.road_map.lane_groups[i].way_forms.size(); j++) {
      lane_groups->add_way_forms(
          Map::FormOfWayType(msg.road_map.lane_groups[i].way_forms[j]));
    }
    for (size_t j = 0;
         j < msg.road_map.lane_groups[i].predecessor_lane_group_ids.size();
         j++) {
      lane_groups->add_predecessor_lane_group_ids(
          msg.road_map.lane_groups[i].predecessor_lane_group_ids[j]);
    }
    for (size_t j = 0;
         j < msg.road_map.lane_groups[i].successor_lane_group_ids.size(); j++) {
      lane_groups->add_successor_lane_group_ids(
          msg.road_map.lane_groups[i].successor_lane_group_ids[j]);
    }
    for (size_t j = 0; j < msg.road_map.lane_groups[i].lane_ids.size(); j++) {
      lane_groups->add_lane_ids(msg.road_map.lane_groups[i].lane_ids[j]);
    }
  }
  // current_routing
  Map::CurrentRouting *current_routing = obj.mutable_current_routing();
  current_routing->set_updated_time(msg.current_routing.updated_time);
  current_routing->set_counter(msg.current_routing.counter);
  for (size_t i = 0; i < msg.current_routing.lane_groups_in_route.size(); i++) {
    Map::LaneGroupInRoute *lane_groups_in_route =
        current_routing->add_lane_groups_in_route();
    lane_groups_in_route->set_lane_group_id(
        msg.current_routing.lane_groups_in_route[i].lane_group_id);
    lane_groups_in_route->set_length(
        msg.current_routing.lane_groups_in_route[i].length);
    for (size_t j = 0;
         j < msg.current_routing.lane_groups_in_route[i].lanes_in_route.size();
         j++) {
      Map::LaneInRoute *lanes_in_route =
          lane_groups_in_route->add_lanes_in_route();
      lanes_in_route->set_lane_id(msg.current_routing.lane_groups_in_route[i]
                                      .lanes_in_route[j]
                                      .lane_id);
      for (size_t k = 0; k < msg.current_routing.lane_groups_in_route[i]
                                 .lanes_in_route[j]
                                 .entry_lane_ids.size();
           k++) {
        lanes_in_route->add_entry_lane_ids(
            msg.current_routing.lane_groups_in_route[i]
                .lanes_in_route[j]
                .entry_lane_ids[k]);
      }
      for (size_t k = 0; k < msg.current_routing.lane_groups_in_route[i]
                                 .lanes_in_route[j]
                                 .exit_lane_ids.size();
           k++) {
        lanes_in_route->add_exit_lane_ids(
            msg.current_routing.lane_groups_in_route[i]
                .lanes_in_route[j]
                .exit_lane_ids[k]);
      }
    }
    Common::Point3d *start_point = lane_groups_in_route->mutable_start_point();
    start_point->set_x(
        msg.current_routing.lane_groups_in_route[i].start_point.x);
    start_point->set_y(
        msg.current_routing.lane_groups_in_route[i].start_point.y);
    start_point->set_z(
        msg.current_routing.lane_groups_in_route[i].start_point.z);

    Common::Point3d *end_point = lane_groups_in_route->mutable_start_point();
    end_point->set_x(msg.current_routing.lane_groups_in_route[i].end_point.x);
    end_point->set_y(msg.current_routing.lane_groups_in_route[i].end_point.y);
    end_point->set_z(msg.current_routing.lane_groups_in_route[i].end_point.z);
  }

  // static end
}
*/

}  // namespace planning_player
}  // namespace planning
