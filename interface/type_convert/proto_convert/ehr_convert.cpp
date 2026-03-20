#include "proto_convert/proto_convert.h"

/*to ros*/
void OriginDataToRos(Map::OriginData obj, proto_msgs::OriginData &msg) {
  // header
  HeaderToRos(obj.header(), msg.msg_header);
  //
  msg.seq = obj.seq();
  msg.origin_data.resize(obj.origin_data().size());
  for (int i = 0; i < obj.origin_data().size(); i++) {
    msg.origin_data[i].msg_counter = obj.origin_data(i).msg_counter();

    char *reverse_img_data = const_cast<char *>(obj.origin_data(i).data().c_str());
    // 这里需要uchar*格式，即unsigned char* 通过reinterpret_cast<unsigned
    // char*>转换
    unsigned char *img = reinterpret_cast<unsigned char *>(reverse_img_data);
    msg.origin_data[i].data.resize(obj.origin_data(i).data().size());
    memcpy(&msg.origin_data[i].data[0], img, obj.origin_data(i).data().size());
  }
}

void StaticMapToRos(Map::StaticMap obj, proto_msgs::StaticMap &msg) {
  // header
  HeaderToRos(obj.header(), msg.msg_header);
  msg.road_map.timestamp = obj.road_map().timestamp();
  msg.road_map.lanes.resize(obj.road_map().lanes().size());
  for (int i = 0; i < obj.road_map().lanes().size(); i++) {
    msg.road_map.lanes[i].lane_id = obj.road_map().lanes(i).lane_id();
    msg.road_map.lanes[i].sequence_id = obj.road_map().lanes(i).sequence_id();
    msg.road_map.lanes[i].points_on_central_line.resize(obj.road_map().lanes(i).points_on_central_line().size());
    for (int j = 0; j < obj.road_map().lanes(i).points_on_central_line().size(); j++) {
      msg.road_map.lanes[i].points_on_central_line[j].x = obj.road_map().lanes(i).points_on_central_line(j).x();
      msg.road_map.lanes[i].points_on_central_line[j].y = obj.road_map().lanes(i).points_on_central_line(j).y();
      msg.road_map.lanes[i].points_on_central_line[j].z = obj.road_map().lanes(i).points_on_central_line(j).z();
    }
    msg.road_map.lanes[i].central_points_offset.resize(obj.road_map().lanes(i).central_points_offset().size());
    for (int j = 0; j < obj.road_map().lanes(i).central_points_offset().size(); j++) {
      msg.road_map.lanes[i].central_points_offset[j] = obj.road_map().lanes(i).central_points_offset(j);
    }
    msg.road_map.lanes[i].width.resize(obj.road_map().lanes(i).width().size());
    for (int j = 0; j < obj.road_map().lanes(i).width().size(); j++) {
      msg.road_map.lanes[i].width[j] = obj.road_map().lanes(i).width(j);
    }
    msg.road_map.lanes[i].curvature.resize(obj.road_map().lanes(i).curvature().size());
    for (int j = 0; j < obj.road_map().lanes(i).curvature().size(); j++) {
      msg.road_map.lanes[i].curvature[j] = obj.road_map().lanes(i).curvature(j);
    }
    msg.road_map.lanes[i].heading.resize(obj.road_map().lanes(i).heading().size());
    for (int j = 0; j < obj.road_map().lanes(i).heading().size(); j++) {
      msg.road_map.lanes[i].heading[j] = obj.road_map().lanes(i).heading(j);
    }
    msg.road_map.lanes[i].lane_direction = obj.road_map().lanes(i).lane_direction();
    msg.road_map.lanes[i].lane_type = obj.road_map().lanes(i).lane_type();
    msg.road_map.lanes[i].left_lane_boundary_id = obj.road_map().lanes(i).left_lane_boundary_id();
    msg.road_map.lanes[i].right_lane_boundary_id = obj.road_map().lanes(i).right_lane_boundary_id();
    msg.road_map.lanes[i].left_road_boundary_id = obj.road_map().lanes(i).left_road_boundary_id();
    msg.road_map.lanes[i].right_road_boundary_id = obj.road_map().lanes(i).right_road_boundary_id();
    msg.road_map.lanes[i].min_speed_limit = obj.road_map().lanes(i).min_speed_limit();
    msg.road_map.lanes[i].max_speed_limit = obj.road_map().lanes(i).max_speed_limit();
    msg.road_map.lanes[i].predecessor_lane_id.resize(obj.road_map().lanes(i).predecessor_lane_id().size());
    for (int j = 0; j < obj.road_map().lanes(i).predecessor_lane_id().size(); j++) {
      msg.road_map.lanes[i].predecessor_lane_id[j] = obj.road_map().lanes(i).predecessor_lane_id(j);
    }
    msg.road_map.lanes[i].successor_lane_id.resize(obj.road_map().lanes(i).successor_lane_id().size());
    for (int j = 0; j < obj.road_map().lanes(i).successor_lane_id().size(); j++) {
      msg.road_map.lanes[i].successor_lane_id[j] = obj.road_map().lanes(i).successor_lane_id(j);
    }
    msg.road_map.lanes[i].left_neighbor_forward_lane_id = obj.road_map().lanes(i).left_neighbor_forward_lane_id();
    msg.road_map.lanes[i].right_neighbor_forward_lane_id = obj.road_map().lanes(i).right_neighbor_forward_lane_id();
    msg.road_map.lanes[i].left_neighbor_reverse_lane_id = obj.road_map().lanes(i).left_neighbor_reverse_lane_id();
    msg.road_map.lanes[i].right_neighbor_reverse_lane_id = obj.road_map().lanes(i).right_neighbor_reverse_lane_id();
    msg.road_map.lanes[i].lane_group_id = obj.road_map().lanes(i).lane_group_id();

    msg.road_map.lanes[i].overlaps.resize(obj.road_map().lanes(i).overlaps().size());
    for (int j = 0; j < obj.road_map().lanes(i).overlaps().size(); j++) {
      msg.road_map.lanes[i].overlaps[j].id = obj.road_map().lanes(i).overlaps(j).id();
      msg.road_map.lanes[i].overlaps[j].type = obj.road_map().lanes(i).overlaps(j).type();
      msg.road_map.lanes[i].overlaps[j].s_begin = obj.road_map().lanes(i).overlaps(j).s_begin();
      msg.road_map.lanes[i].overlaps[j].s_end = obj.road_map().lanes(i).overlaps(j).s_end();
    }
  }
  msg.road_map.lane_boundaries.resize(obj.road_map().lane_boundaries().size());
  for (int i = 0; i < obj.road_map().lane_boundaries().size(); i++) {
    msg.road_map.lane_boundaries[i].boundary_id = obj.road_map().lane_boundaries(i).boundary_id();
    msg.road_map.lane_boundaries[i].boundary_attributes.resize(
        obj.road_map().lane_boundaries(i).boundary_attributes().size());
    for (int j = 0; j < obj.road_map().lane_boundaries(i).boundary_attributes().size(); j++) {
      msg.road_map.lane_boundaries[i].boundary_attributes[j].points.resize(
          obj.road_map().lane_boundaries(i).boundary_attributes(j).points().size());
      for (int k = 0; k < obj.road_map().lane_boundaries(i).boundary_attributes(j).points().size(); k++) {
        msg.road_map.lane_boundaries[i].boundary_attributes[j].points[k].x =
            obj.road_map().lane_boundaries(i).boundary_attributes(j).points(k).x();
        msg.road_map.lane_boundaries[i].boundary_attributes[j].points[k].y =
            obj.road_map().lane_boundaries(i).boundary_attributes(j).points(k).y();
        msg.road_map.lane_boundaries[i].boundary_attributes[j].points[k].z =
            obj.road_map().lane_boundaries(i).boundary_attributes(j).points(k).z();
      }
      msg.road_map.lane_boundaries[i].boundary_attributes[j].points_s.resize(
          obj.road_map().lane_boundaries(i).boundary_attributes(j).points_s().size());
      for (int k = 0; k < obj.road_map().lane_boundaries(i).boundary_attributes(j).points_s().size(); k++) {
        msg.road_map.lane_boundaries[i].boundary_attributes[j].points_s[k] =
            obj.road_map().lane_boundaries(i).boundary_attributes(j).points_s(k);
      }
      msg.road_map.lane_boundaries[i].boundary_attributes[j].types.resize(
          obj.road_map().lane_boundaries(i).boundary_attributes(j).types().size());
      for (int k = 0; k < obj.road_map().lane_boundaries(i).boundary_attributes(j).types().size(); k++) {
        msg.road_map.lane_boundaries[i].boundary_attributes[j].types[k] =
            obj.road_map().lane_boundaries(i).boundary_attributes(j).types(k);
      }
      msg.road_map.lane_boundaries[i].boundary_attributes[j].colors.resize(
          obj.road_map().lane_boundaries(i).boundary_attributes(j).colors().size());
      for (int k = 0; k < obj.road_map().lane_boundaries(i).boundary_attributes(j).colors().size(); k++) {
        msg.road_map.lane_boundaries[i].boundary_attributes[j].colors[k] =
            obj.road_map().lane_boundaries(i).boundary_attributes(j).colors(k);
      }
    }
  }
  msg.road_map.road_boundaries.resize(obj.road_map().road_boundaries().size());
  for (int i = 0; i < obj.road_map().road_boundaries().size(); i++) {
    msg.road_map.road_boundaries[i].boundary_id = obj.road_map().road_boundaries(i).boundary_id();
    msg.road_map.road_boundaries[i].boundary_attributes.resize(
        obj.road_map().road_boundaries(i).boundary_attributes().size());
    for (int j = 0; j < obj.road_map().road_boundaries(i).boundary_attributes().size(); j++) {
      msg.road_map.road_boundaries[i].boundary_attributes[j].points.resize(
          obj.road_map().road_boundaries(i).boundary_attributes(j).points().size());
      for (int k = 0; k < obj.road_map().road_boundaries(i).boundary_attributes(j).points().size(); k++) {
        msg.road_map.road_boundaries[i].boundary_attributes[j].points[k].x =
            obj.road_map().road_boundaries(i).boundary_attributes(j).points(k).x();
        msg.road_map.road_boundaries[i].boundary_attributes[j].points[k].y =
            obj.road_map().road_boundaries(i).boundary_attributes(j).points(k).y();
        msg.road_map.road_boundaries[i].boundary_attributes[j].points[k].z =
            obj.road_map().road_boundaries(i).boundary_attributes(j).points(k).z();
      }
      msg.road_map.road_boundaries[i].boundary_attributes[j].points_s.resize(
          obj.road_map().road_boundaries(i).boundary_attributes(j).points_s().size());
      for (int k = 0; k < obj.road_map().road_boundaries(i).boundary_attributes(j).points_s().size(); k++) {
        msg.road_map.road_boundaries[i].boundary_attributes[j].points_s[k] =
            obj.road_map().road_boundaries(i).boundary_attributes(j).points_s(k);
      }
      msg.road_map.road_boundaries[i].boundary_attributes[j].types.resize(
          obj.road_map().road_boundaries(i).boundary_attributes(j).types().size());
      for (int k = 0; k < obj.road_map().road_boundaries(i).boundary_attributes(j).types().size(); k++) {
        msg.road_map.road_boundaries[i].boundary_attributes[j].types[k] =
            obj.road_map().road_boundaries(i).boundary_attributes(j).types(k);
      }
      msg.road_map.road_boundaries[i].boundary_attributes[j].colors.resize(
          obj.road_map().road_boundaries(i).boundary_attributes(j).colors().size());
      for (int k = 0; k < obj.road_map().road_boundaries(i).boundary_attributes(j).colors().size(); k++) {
        msg.road_map.road_boundaries[i].boundary_attributes[j].colors[k] =
            obj.road_map().road_boundaries(i).boundary_attributes(j).colors(k);
      }
    }
  }
  msg.road_map.lane_groups.resize(obj.road_map().lane_groups().size());
  for (int i = 0; i < obj.road_map().lane_groups().size(); i++) {
    msg.road_map.lane_groups[i].lane_group_id = obj.road_map().lane_groups(i).lane_group_id();
    msg.road_map.lane_groups[i].length = obj.road_map().lane_groups(i).length();
    msg.road_map.lane_groups[i].way_forms.resize(obj.road_map().lane_groups(i).way_forms().size());
    for (int j = 0; j < obj.road_map().lane_groups(i).way_forms().size(); j++) {
      msg.road_map.lane_groups[i].way_forms[j] = obj.road_map().lane_groups(i).way_forms(j);
    }
    msg.road_map.lane_groups[i].predecessor_lane_group_ids.resize(
        obj.road_map().lane_groups(i).predecessor_lane_group_ids().size());
    for (int j = 0; j < obj.road_map().lane_groups(i).predecessor_lane_group_ids().size(); j++) {
      msg.road_map.lane_groups[i].predecessor_lane_group_ids[j] =
          obj.road_map().lane_groups(i).predecessor_lane_group_ids(j);
    }
    msg.road_map.lane_groups[i].successor_lane_group_ids.resize(
        obj.road_map().lane_groups(i).successor_lane_group_ids().size());
    for (int j = 0; j < obj.road_map().lane_groups(i).successor_lane_group_ids().size(); j++) {
      msg.road_map.lane_groups[i].successor_lane_group_ids[j] =
          obj.road_map().lane_groups(i).successor_lane_group_ids(j);
    }
    msg.road_map.lane_groups[i].lane_ids.resize(obj.road_map().lane_groups(i).lane_ids().size());
    for (int j = 0; j < obj.road_map().lane_groups(i).lane_ids().size(); j++) {
      msg.road_map.lane_groups[i].lane_ids[j] = obj.road_map().lane_groups(i).lane_ids(j);
    }
  }

  msg.current_routing.updated_time = obj.current_routing().updated_time();
  msg.current_routing.counter = obj.current_routing().counter();
  msg.current_routing.lane_groups_in_route.resize(obj.current_routing().lane_groups_in_route().size());
  for (int i = 0; i < obj.current_routing().lane_groups_in_route().size(); i++) {
    msg.current_routing.lane_groups_in_route[i].lane_group_id =
        obj.current_routing().lane_groups_in_route(i).lane_group_id();
    msg.current_routing.lane_groups_in_route[i].length = obj.current_routing().lane_groups_in_route(i).length();
    msg.current_routing.lane_groups_in_route[i].lanes_in_route.resize(
        obj.current_routing().lane_groups_in_route(i).lanes_in_route().size());
    for (int j = 0; j < obj.current_routing().lane_groups_in_route(i).lanes_in_route().size(); j++) {
      msg.current_routing.lane_groups_in_route[i].lanes_in_route[j].lane_id =
          obj.current_routing().lane_groups_in_route(i).lanes_in_route(j).lane_id();
      msg.current_routing.lane_groups_in_route[i].lanes_in_route[j].entry_lane_ids.resize(
          obj.current_routing().lane_groups_in_route(i).lanes_in_route(j).entry_lane_ids().size());
      for (int k = 0; k < obj.current_routing().lane_groups_in_route(i).lanes_in_route(j).entry_lane_ids().size();
           k++) {
        msg.current_routing.lane_groups_in_route[i].lanes_in_route[j].entry_lane_ids[k] =
            obj.current_routing().lane_groups_in_route(i).lanes_in_route(j).entry_lane_ids(k);
      }
      msg.current_routing.lane_groups_in_route[i].lanes_in_route[j].exit_lane_ids.resize(
          obj.current_routing().lane_groups_in_route(i).lanes_in_route(j).exit_lane_ids().size());
      for (int k = 0; k < obj.current_routing().lane_groups_in_route(i).lanes_in_route(j).exit_lane_ids().size(); k++) {
        msg.current_routing.lane_groups_in_route[i].lanes_in_route[j].exit_lane_ids[k] =
            obj.current_routing().lane_groups_in_route(i).lanes_in_route(j).exit_lane_ids(k);
      }
    }
    msg.current_routing.lane_groups_in_route[i].start_point.x =
        obj.current_routing().lane_groups_in_route(i).start_point().x();
    msg.current_routing.lane_groups_in_route[i].start_point.y =
        obj.current_routing().lane_groups_in_route(i).start_point().y();
    msg.current_routing.lane_groups_in_route[i].start_point.z =
        obj.current_routing().lane_groups_in_route(i).start_point().z();
    msg.current_routing.lane_groups_in_route[i].end_point.x =
        obj.current_routing().lane_groups_in_route(i).end_point().x();
    msg.current_routing.lane_groups_in_route[i].end_point.y =
        obj.current_routing().lane_groups_in_route(i).end_point().y();
    msg.current_routing.lane_groups_in_route[i].end_point.z =
        obj.current_routing().lane_groups_in_route(i).end_point().z();
  }
}

/*to proto*/
void OriginDataToProto(Map::OriginData &obj, proto_msgs::OriginData msg) {
  // header
  Common::Header *header = obj.mutable_header();
  header->set_timestamp(msg.msg_header.timestamp);
  header->set_version(msg.msg_header.version);
  //
  obj.set_seq(msg.seq);
  for (size_t i = 0; i < msg.origin_data.size(); i++) {
    Map::EhpData *origin_data = obj.add_origin_data();
    origin_data->set_msg_counter(msg.origin_data[i].msg_counter);
    const unsigned char *temp = &(msg.origin_data[i].data)[0];
    unsigned char *chars = const_cast<unsigned char *>(temp);
    char *img = reinterpret_cast<char *>(chars);
    origin_data->set_data(img);
  }
}

void StaticMapToProto(Map::StaticMap &obj, proto_msgs::StaticMap msg) {
  // header
  Common::Header *header = obj.mutable_header();
  header->set_timestamp(msg.msg_header.timestamp);
  header->set_version(msg.msg_header.version);
  // road_map
  Map::RoadMap *road_map = obj.mutable_road_map();
  road_map->set_timestamp(msg.road_map.timestamp);
  for (size_t i = 0; i < msg.road_map.lanes.size(); i++) {
    Map::LaneData *lanes = road_map->add_lanes();
    lanes->set_lane_id(msg.road_map.lanes[i].lane_id);
    lanes->set_sequence_id(msg.road_map.lanes[i].sequence_id);
    for (size_t j = 0; j < msg.road_map.lanes[i].points_on_central_line.size(); j++) {
      Common::Point3d *points_on_central_line = lanes->add_points_on_central_line();
      points_on_central_line->set_x(msg.road_map.lanes[i].points_on_central_line[j].x);
      points_on_central_line->set_y(msg.road_map.lanes[i].points_on_central_line[j].y);
      points_on_central_line->set_z(msg.road_map.lanes[i].points_on_central_line[j].z);
    }

    for (size_t j = 0; j < msg.road_map.lanes[i].central_points_offset.size(); j++) {
      lanes->add_central_points_offset(msg.road_map.lanes[i].central_points_offset[j]);
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

    lanes->set_lane_direction(Map::Direction(msg.road_map.lanes[i].lane_direction));
    lanes->set_lane_type(Map::LaneType(msg.road_map.lanes[i].lane_type));
    lanes->set_left_lane_boundary_id(msg.road_map.lanes[i].left_lane_boundary_id);
    lanes->set_right_lane_boundary_id(msg.road_map.lanes[i].right_lane_boundary_id);
    lanes->set_left_road_boundary_id(msg.road_map.lanes[i].left_road_boundary_id);
    lanes->set_right_road_boundary_id(msg.road_map.lanes[i].right_road_boundary_id);
    lanes->set_min_speed_limit(msg.road_map.lanes[i].min_speed_limit);
    lanes->set_max_speed_limit(msg.road_map.lanes[i].max_speed_limit);

    for (size_t j = 0; j < msg.road_map.lanes[i].predecessor_lane_id.size(); j++) {
      lanes->add_predecessor_lane_id(msg.road_map.lanes[i].predecessor_lane_id[j]);
    }
    for (size_t j = 0; j < msg.road_map.lanes[i].successor_lane_id.size(); j++) {
      lanes->add_successor_lane_id(msg.road_map.lanes[i].successor_lane_id[j]);
    }

    lanes->set_left_neighbor_forward_lane_id(msg.road_map.lanes[i].left_neighbor_forward_lane_id);
    lanes->set_right_neighbor_forward_lane_id(msg.road_map.lanes[i].right_neighbor_forward_lane_id);
    lanes->set_left_neighbor_reverse_lane_id(msg.road_map.lanes[i].left_neighbor_reverse_lane_id);
    lanes->set_right_neighbor_reverse_lane_id(msg.road_map.lanes[i].right_neighbor_reverse_lane_id);
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
    lane_boundaries->set_boundary_id(msg.road_map.lane_boundaries[i].boundary_id);
    for (size_t j = 0; j < msg.road_map.lane_boundaries[i].boundary_attributes.size(); j++) {
      Map::BoundaryAttributes *boundary_attributes = lane_boundaries->add_boundary_attributes();
      for (size_t k = 0; k < msg.road_map.lane_boundaries[i].boundary_attributes[j].points.size(); k++) {
        Common::Point3d *points = boundary_attributes->add_points();
        points->set_x(msg.road_map.lane_boundaries[i].boundary_attributes[j].points[k].x);
        points->set_y(msg.road_map.lane_boundaries[i].boundary_attributes[j].points[k].y);
        points->set_z(msg.road_map.lane_boundaries[i].boundary_attributes[j].points[k].z);
      }

      for (size_t k = 0; k < msg.road_map.lane_boundaries[i].boundary_attributes[j].points_s.size(); k++) {
        boundary_attributes->add_points_s(msg.road_map.lane_boundaries[i].boundary_attributes[j].points_s[k]);
      }

      for (size_t k = 0; k < msg.road_map.lane_boundaries[i].boundary_attributes[j].types.size(); k++) {
        boundary_attributes->add_types(
            Map::BoundaryAttributes::Type(msg.road_map.lane_boundaries[i].boundary_attributes[j].types[k]));
      }

      for (size_t k = 0; k < msg.road_map.lane_boundaries[i].boundary_attributes[j].colors.size(); k++) {
        boundary_attributes->add_colors(
            Map::BoundaryAttributes::Color(msg.road_map.lane_boundaries[i].boundary_attributes[j].colors[k]));
      }
    }
  }

  for (size_t i = 0; i < msg.road_map.road_boundaries.size(); i++) {
    Map::LaneBoundary *road_boundaries = road_map->add_road_boundaries();
    road_boundaries->set_boundary_id(msg.road_map.road_boundaries[i].boundary_id);
    for (size_t j = 0; j < msg.road_map.road_boundaries[i].boundary_attributes.size(); j++) {
      Map::BoundaryAttributes *boundary_attributes = road_boundaries->add_boundary_attributes();
      for (size_t k = 0; k < msg.road_map.road_boundaries[i].boundary_attributes[j].points.size(); k++) {
        Common::Point3d *points = boundary_attributes->add_points();
        points->set_x(msg.road_map.road_boundaries[i].boundary_attributes[j].points[k].x);
        points->set_y(msg.road_map.road_boundaries[i].boundary_attributes[j].points[k].y);
        points->set_z(msg.road_map.road_boundaries[i].boundary_attributes[j].points[k].z);
      }

      for (size_t k = 0; k < msg.road_map.road_boundaries[i].boundary_attributes[j].points_s.size(); k++) {
        boundary_attributes->add_points_s(msg.road_map.road_boundaries[i].boundary_attributes[j].points_s[k]);
      }

      for (size_t k = 0; k < msg.road_map.road_boundaries[i].boundary_attributes[j].types.size(); k++) {
        boundary_attributes->add_types(
            Map::BoundaryAttributes::Type(msg.road_map.road_boundaries[i].boundary_attributes[j].types[k]));
      }

      for (size_t k = 0; k < msg.road_map.road_boundaries[i].boundary_attributes[j].colors.size(); k++) {
        boundary_attributes->add_colors(
            Map::BoundaryAttributes::Color(msg.road_map.road_boundaries[i].boundary_attributes[j].colors[k]));
      }
    }
  }

  for (size_t i = 0; i < msg.road_map.lane_groups.size(); i++) {
    Map::LaneGroup *lane_groups = road_map->add_lane_groups();
    lane_groups->set_lane_group_id(msg.road_map.lane_groups[i].lane_group_id);
    lane_groups->set_length(msg.road_map.lane_groups[i].length);
    for (size_t j = 0; j < msg.road_map.lane_groups[i].way_forms.size(); j++) {
      lane_groups->add_way_forms(Map::FormOfWayType(msg.road_map.lane_groups[i].way_forms[j]));
    }
    for (size_t j = 0; j < msg.road_map.lane_groups[i].predecessor_lane_group_ids.size(); j++) {
      lane_groups->add_predecessor_lane_group_ids(msg.road_map.lane_groups[i].predecessor_lane_group_ids[j]);
    }
    for (size_t j = 0; j < msg.road_map.lane_groups[i].successor_lane_group_ids.size(); j++) {
      lane_groups->add_successor_lane_group_ids(msg.road_map.lane_groups[i].successor_lane_group_ids[j]);
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
    Map::LaneGroupInRoute *lane_groups_in_route = current_routing->add_lane_groups_in_route();
    lane_groups_in_route->set_lane_group_id(msg.current_routing.lane_groups_in_route[i].lane_group_id);
    lane_groups_in_route->set_length(msg.current_routing.lane_groups_in_route[i].length);
    for (size_t j = 0; j < msg.current_routing.lane_groups_in_route[i].lanes_in_route.size(); j++) {
      Map::LaneInRoute *lanes_in_route = lane_groups_in_route->add_lanes_in_route();
      lanes_in_route->set_lane_id(msg.current_routing.lane_groups_in_route[i].lanes_in_route[j].lane_id);
      for (size_t k = 0; k < msg.current_routing.lane_groups_in_route[i].lanes_in_route[j].entry_lane_ids.size(); k++) {
        lanes_in_route->add_entry_lane_ids(
            msg.current_routing.lane_groups_in_route[i].lanes_in_route[j].entry_lane_ids[k]);
      }
      for (size_t k = 0; k < msg.current_routing.lane_groups_in_route[i].lanes_in_route[j].exit_lane_ids.size(); k++) {
        lanes_in_route->add_exit_lane_ids(
            msg.current_routing.lane_groups_in_route[i].lanes_in_route[j].exit_lane_ids[k]);
      }
    }
    Common::Point3d *start_point = lane_groups_in_route->mutable_start_point();
    start_point->set_x(msg.current_routing.lane_groups_in_route[i].start_point.x);
    start_point->set_y(msg.current_routing.lane_groups_in_route[i].start_point.y);
    start_point->set_z(msg.current_routing.lane_groups_in_route[i].start_point.z);

    Common::Point3d *end_point = lane_groups_in_route->mutable_start_point();
    end_point->set_x(msg.current_routing.lane_groups_in_route[i].end_point.x);
    end_point->set_y(msg.current_routing.lane_groups_in_route[i].end_point.y);
    end_point->set_z(msg.current_routing.lane_groups_in_route[i].end_point.z);
  }
}