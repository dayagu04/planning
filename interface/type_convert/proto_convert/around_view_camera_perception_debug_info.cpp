#include "proto_convert/proto_convert.h"

/*to ros*/
void AroundViewCameraPerceptionInfoToRos(AroundViewCameraPerceptionResult::AroundViewCameraPerceptionInfo obj,
                                         proto_msgs::AroundViewCameraPerceptionInfo &msg) {
  msg.id = obj.id();
  msg.header.timestamp = obj.header().timestamp();
  msg.header.version = obj.header().version();
  msg.header.seq = obj.header().seq();
  msg.avm_time_stamp = obj.avm_time_stamp();
  msg.left_camera_exposure_time_stamp = obj.left_camera_exposure_time_stamp();
  msg.front_camera_exposure_time_stamp = obj.front_camera_exposure_time_stamp();
  msg.right_camera_exposure_time_stamp = obj.right_camera_exposure_time_stamp();
  msg.rear_camera_exposure_time_stamp = obj.rear_camera_exposure_time_stamp();

  // ParkingSpace[] parking_spaces             #视觉车位线信息   <最大8个>
  msg.parking_spaces.resize(obj.parking_spaces().size());
  for (int i = 0; i < obj.parking_spaces().size(); i++) {
    msg.parking_spaces[i].point0_x = obj.parking_spaces(i).point0_x();
    msg.parking_spaces[i].point0_y = obj.parking_spaces(i).point0_y();
    msg.parking_spaces[i].point1_x = obj.parking_spaces(i).point1_x();
    msg.parking_spaces[i].point1_y = obj.parking_spaces(i).point1_y();
    msg.parking_spaces[i].point2_x = obj.parking_spaces(i).point2_x();
    msg.parking_spaces[i].point2_y = obj.parking_spaces(i).point2_y();
    msg.parking_spaces[i].point3_x = obj.parking_spaces(i).point3_x();
    msg.parking_spaces[i].point3_y = obj.parking_spaces(i).point3_y();
    msg.parking_spaces[i].direction1 = obj.parking_spaces(i).direction1();
    msg.parking_spaces[i].direction2 = obj.parking_spaces(i).direction2();
    msg.parking_spaces[i].entrance_vacant = obj.parking_spaces(i).entrance_vacant();
    msg.parking_spaces[i].entrance_score = obj.parking_spaces(i).entrance_score();
    msg.parking_spaces[i].entrance_type = obj.parking_spaces(i).entrance_type();
    msg.parking_spaces[i].id = obj.parking_spaces(i).id();
    msg.parking_spaces[i].confidence = obj.parking_spaces(i).confidence();
    msg.parking_spaces[i].confidence0 = obj.parking_spaces(i).confidence0();
    msg.parking_spaces[i].confidence1 = obj.parking_spaces(i).confidence1();
    msg.parking_spaces[i].confidence2 = obj.parking_spaces(i).confidence2();
    msg.parking_spaces[i].confidence3 = obj.parking_spaces(i).confidence3();
    msg.parking_spaces[i].life_time = obj.parking_spaces(i).life_time();
  }

  // ParkingLimiter[] parking_limiters         #视觉限位器信息
  msg.parking_limiters.resize(obj.parking_limiters().size());
  for (int i = 0; i < obj.parking_limiters().size(); i++) {
    msg.parking_limiters[i].point1_x = obj.parking_limiters(i).point1_x();
    msg.parking_limiters[i].point1_y = obj.parking_limiters(i).point1_y();
    msg.parking_limiters[i].point2_x = obj.parking_limiters(i).point2_x();
    msg.parking_limiters[i].point2_y = obj.parking_limiters(i).point2_y();
    msg.parking_limiters[i].id = obj.parking_limiters(i).id();
    msg.parking_limiters[i].confidence = obj.parking_limiters(i).confidence();
    msg.parking_limiters[i].life_time = obj.parking_limiters(i).life_time();
  }

  // ParkingColumn[] parking_columns           #视觉立柱接地线信息
  msg.parking_columns.resize(obj.parking_columns().size());
  for (int i = 0; i < obj.parking_columns().size(); i++) {
    msg.parking_columns[i].id = obj.parking_columns(i).id();
    msg.parking_columns[i].points_2d.resize(obj.parking_columns(i).points_2d().size());
    for (int j = 0; j < obj.parking_columns(i).points_2d().size(); j++) {
      msg.parking_columns[i].points_2d[j].x = obj.parking_columns(i).points_2d(j).x();
      msg.parking_columns[i].points_2d[j].y = obj.parking_columns(i).points_2d(j).y();
    }
    msg.parking_columns[i].confidence = obj.parking_columns(i).confidence();
    msg.parking_columns[i].life_time = obj.parking_columns(i).life_time();
    msg.parking_columns[i].column_dim = obj.parking_columns(i).column_dim();
    msg.parking_columns[i].column_num = obj.parking_columns(i).column_num();
    msg.parking_columns[i].column_reserved = obj.parking_columns(i).column_reserved();
  }

  // ParkingWall[] parking_walls               #视觉墙面接地线信息
  msg.parking_walls.resize(obj.parking_walls().size());
  for (int i = 0; i < obj.parking_walls().size(); i++) {
    msg.parking_walls[i].id = obj.parking_walls(i).id();
    msg.parking_walls[i].points_2d.resize(obj.parking_walls(i).points_2d().size());
    for (int j = 0; j < obj.parking_walls(i).points_2d().size(); j++) {
      msg.parking_walls[i].points_2d[j].x = obj.parking_walls(i).points_2d(j).x();
      msg.parking_walls[i].points_2d[j].y = obj.parking_walls(i).points_2d(j).y();
    }
    msg.parking_walls[i].confidence = obj.parking_walls(i).confidence();
    msg.parking_walls[i].life_time = obj.parking_walls(i).life_time();
    msg.parking_walls[i].wall_dim = obj.parking_walls(i).wall_dim();
    msg.parking_walls[i].wall_num = obj.parking_walls(i).wall_num();
    msg.parking_walls[i].wall_reserved = obj.parking_walls(i).wall_reserved();
  }

  // ParkingDeceler[] parking_decelers         #视觉减速带信息
  msg.parking_decelers.resize(obj.parking_decelers().size());
  for (int i = 0; i < obj.parking_decelers().size(); i++) {
    msg.parking_decelers[i].point1_x = obj.parking_decelers(i).point1_x();
    msg.parking_decelers[i].point1_y = obj.parking_decelers(i).point1_y();
    msg.parking_decelers[i].point2_x = obj.parking_decelers(i).point2_x();
    msg.parking_decelers[i].point2_y = obj.parking_decelers(i).point2_y();
    msg.parking_decelers[i].point3_x = obj.parking_decelers(i).point3_x();
    msg.parking_decelers[i].point3_y = obj.parking_decelers(i).point3_y();
    msg.parking_decelers[i].point4_x = obj.parking_decelers(i).point4_x();
    msg.parking_decelers[i].point4_y = obj.parking_decelers(i).point4_y();
    msg.parking_decelers[i].id = obj.parking_decelers(i).id();
    msg.parking_decelers[i].confidence = obj.parking_decelers(i).confidence();
    msg.parking_decelers[i].life_time = obj.parking_decelers(i).life_time();
  }

  // ParkingLane[] parking_lanes               #视觉车道线信息
  msg.parking_lanes.resize(obj.parking_lanes().size());
  for (int i = 0; i < obj.parking_lanes().size(); i++) {
    msg.parking_lanes[i].id = obj.parking_lanes(i).id();
    msg.parking_lanes[i].points_2d.resize(obj.parking_lanes(i).points_2d().size());
    for (int j = 0; j < obj.parking_lanes(i).points_2d().size(); j++) {
      msg.parking_lanes[i].points_2d[j].x = obj.parking_lanes(i).points_2d(j).x();
      msg.parking_lanes[i].points_2d[j].y = obj.parking_lanes(i).points_2d(j).y();
    }
    msg.parking_lanes[i].confidence = obj.parking_lanes(i).confidence();
    msg.parking_lanes[i].life_time = obj.parking_lanes(i).life_time();
    msg.parking_lanes[i].lane_dim = obj.parking_lanes(i).lane_dim();
    msg.parking_lanes[i].lane_num = obj.parking_lanes(i).lane_num();
    msg.parking_lanes[i].lane_reserved = obj.parking_lanes(i).lane_reserved();
  }

  // ParkingArrow[] parking_arrows             #视觉地面箭头信息
  msg.parking_arrows.resize(obj.parking_arrows().size());
  for (int i = 0; i < obj.parking_arrows().size(); i++) {
    msg.parking_arrows[i].id = obj.parking_arrows(i).id();
    msg.parking_arrows[i].begin_points_2d.resize(obj.parking_arrows(i).begin_points_2d().size());
    for (int j = 0; j < obj.parking_arrows(i).begin_points_2d().size(); j++) {
      msg.parking_arrows[i].begin_points_2d[j].x = obj.parking_arrows(i).begin_points_2d(j).x();
      msg.parking_arrows[i].begin_points_2d[j].y = obj.parking_arrows(i).begin_points_2d(j).y();
    }
    msg.parking_arrows[i].end_points_2d.resize(obj.parking_arrows(i).end_points_2d().size());
    for (int j = 0; j < obj.parking_arrows(i).end_points_2d().size(); j++) {
      msg.parking_arrows[i].end_points_2d[j].x = obj.parking_arrows(i).end_points_2d(j).x();
      msg.parking_arrows[i].end_points_2d[j].y = obj.parking_arrows(i).end_points_2d(j).y();
    }
    msg.parking_arrows[i].confidence = obj.parking_arrows(i).confidence();
    msg.parking_arrows[i].life_time = obj.parking_arrows(i).life_time();
    msg.parking_arrows[i].begin_scores.resize(obj.parking_arrows(i).begin_scores().size());
    for (int j = 0; j < obj.parking_arrows(i).begin_scores().size(); j++) {
      msg.parking_arrows[i].begin_scores[j] = obj.parking_arrows(i).begin_scores(j);
    }

    msg.parking_arrows[i].begin_dim = obj.parking_arrows(i).begin_dim();
    msg.parking_arrows[i].begin_num = obj.parking_arrows(i).begin_num();
    msg.parking_arrows[i].begin_reserved = obj.parking_arrows(i).begin_reserved();
    msg.parking_arrows[i].end_scores.resize(obj.parking_arrows(i).end_scores().size());
    for (int j = 0; j < obj.parking_arrows(i).end_scores().size(); j++) {
      msg.parking_arrows[i].end_scores[j] = obj.parking_arrows(i).end_scores(j);
    }
    msg.parking_arrows[i].end_dim = obj.parking_arrows(i).end_dim();
    msg.parking_arrows[i].end_num = obj.parking_arrows(i).end_num();
    msg.parking_arrows[i].end_reserved = obj.parking_arrows(i).end_reserved();
  }

  // ParkingLeftExtend[] parking_left_extends
  msg.parking_left_extends.resize(obj.parking_left_extends().size());
  for (int i = 0; i < obj.parking_left_extends().size(); i++) {
    msg.parking_left_extends[i].id = obj.parking_left_extends(i).id();
    msg.parking_left_extends[i].point1_x = obj.parking_left_extends(i).point1_x();
    msg.parking_left_extends[i].point1_y = obj.parking_left_extends(i).point1_y();
    msg.parking_left_extends[i].confidence = obj.parking_left_extends(i).confidence();
    msg.parking_left_extends[i].life_time = obj.parking_left_extends(i).life_time();
  }

  // ParkingRightExtend[] parking_right_extends
  msg.parking_right_extends.resize(obj.parking_right_extends().size());
  for (int i = 0; i < obj.parking_right_extends().size(); i++) {
    msg.parking_right_extends[i].id = obj.parking_right_extends(i).id();
    msg.parking_right_extends[i].point1_x = obj.parking_right_extends(i).point1_x();
    msg.parking_right_extends[i].point1_y = obj.parking_right_extends(i).point1_y();
    msg.parking_right_extends[i].confidence = obj.parking_right_extends(i).confidence();
    msg.parking_right_extends[i].life_time = obj.parking_right_extends(i).life_time();
  }

  msg.parking_space_num = obj.parking_space_num();
  msg.parking_limiter_num = obj.parking_limiter_num();
  msg.parking_deceler_num = obj.parking_deceler_num();
  msg.parking_column_num = obj.parking_column_num();
  msg.parking_wall_num = obj.parking_wall_num();
  msg.parking_left_extend_num = obj.parking_left_extend_num();
  msg.parking_right_extend_num = obj.parking_right_extend_num();
  msg.parking_arrow_num = obj.parking_arrow_num();
  msg.parking_lane_num = obj.parking_lane_num();
}

/*to proto*/
void AroundViewCameraPerceptionInfoToProto(AroundViewCameraPerceptionResult::AroundViewCameraPerceptionInfo &obj,
                                           proto_msgs::AroundViewCameraPerceptionInfo msg) {
  obj.set_id(msg.id);
  AroundViewCameraPerceptionResult::Header *header = obj.mutable_header();
  header->set_timestamp(msg.header.timestamp);
  header->set_version(msg.header.version);
  header->set_seq(msg.header.seq);
  obj.set_avm_time_stamp(msg.avm_time_stamp);
  obj.set_left_camera_exposure_time_stamp(msg.left_camera_exposure_time_stamp);
  obj.set_front_camera_exposure_time_stamp(msg.front_camera_exposure_time_stamp);
  obj.set_right_camera_exposure_time_stamp(msg.right_camera_exposure_time_stamp);
  obj.set_rear_camera_exposure_time_stamp(msg.rear_camera_exposure_time_stamp);

  // parking_spaces
  for (size_t i = 0; i < msg.parking_spaces.size(); i++) {
    AroundViewCameraPerceptionResult::ParkingSpace *parking_spaces = obj.add_parking_spaces();
    parking_spaces->set_point0_x(msg.parking_spaces[i].point0_x);
    parking_spaces->set_point0_y(msg.parking_spaces[i].point0_y);
    parking_spaces->set_point1_x(msg.parking_spaces[i].point1_x);
    parking_spaces->set_point1_y(msg.parking_spaces[i].point1_y);
    parking_spaces->set_point2_x(msg.parking_spaces[i].point2_x);
    parking_spaces->set_point2_y(msg.parking_spaces[i].point2_y);
    parking_spaces->set_point3_x(msg.parking_spaces[i].point3_x);
    parking_spaces->set_point3_y(msg.parking_spaces[i].point3_y);
    parking_spaces->set_direction1(msg.parking_spaces[i].direction1);
    parking_spaces->set_direction2(msg.parking_spaces[i].direction2);
    parking_spaces->set_entrance_vacant(msg.parking_spaces[i].entrance_vacant);
    parking_spaces->set_entrance_score(msg.parking_spaces[i].entrance_score);
    parking_spaces->set_entrance_type(msg.parking_spaces[i].entrance_type);
    parking_spaces->set_id(msg.parking_spaces[i].id);
    parking_spaces->set_confidence(msg.parking_spaces[i].confidence);
    parking_spaces->set_confidence0(msg.parking_spaces[i].confidence0);
    parking_spaces->set_confidence1(msg.parking_spaces[i].confidence1);
    parking_spaces->set_confidence2(msg.parking_spaces[i].confidence2);
    parking_spaces->set_confidence3(msg.parking_spaces[i].confidence3);
    parking_spaces->set_life_time(msg.parking_spaces[i].life_time);
  }

  // repeated ParkingLimiter parking_limiters
  for (size_t i = 0; i < msg.parking_limiters.size(); i++) {
    AroundViewCameraPerceptionResult::ParkingLimiter *parking_limiters = obj.add_parking_limiters();
    parking_limiters->set_point1_x(msg.parking_limiters[i].point1_x);
    parking_limiters->set_point1_y(msg.parking_limiters[i].point1_y);
    parking_limiters->set_point2_x(msg.parking_limiters[i].point2_x);
    parking_limiters->set_point2_y(msg.parking_limiters[i].point2_y);
    parking_limiters->set_id(msg.parking_limiters[i].id);
    parking_limiters->set_confidence(msg.parking_limiters[i].confidence);
    parking_limiters->set_life_time(msg.parking_limiters[i].life_time);
  }

  // repeated ParkingColumn parking_columns
  for (size_t i = 0; i < msg.parking_columns.size(); i++) {
    AroundViewCameraPerceptionResult::ParkingColumn *parking_columns = obj.add_parking_columns();
    parking_columns->set_id(msg.parking_columns[i].id);
    for (size_t j = 0; j < msg.parking_columns[i].points_2d.size(); j++) {
      AroundViewCameraPerceptionResult::Point2f *points_2d = parking_columns->add_points_2d();
      points_2d->set_x(msg.parking_columns[i].points_2d[j].x);
      points_2d->set_y(msg.parking_columns[i].points_2d[j].y);
    }
    parking_columns->set_confidence(msg.parking_columns[i].confidence);
    parking_columns->set_life_time(msg.parking_columns[i].life_time);
    parking_columns->set_column_dim(msg.parking_columns[i].column_dim);
    parking_columns->set_column_num(msg.parking_columns[i].column_num);
    parking_columns->set_column_reserved(msg.parking_columns[i].column_reserved);
  }

  // repeated ParkingWall parking_walls
  for (size_t i = 0; i < msg.parking_walls.size(); i++) {
    AroundViewCameraPerceptionResult::ParkingWall *parking_walls = obj.add_parking_walls();
    parking_walls->set_id(msg.parking_walls[i].id);
    for (size_t j = 0; j < msg.parking_walls[i].points_2d.size(); j++) {
      AroundViewCameraPerceptionResult::Point2f *points_2d = parking_walls->add_points_2d();
      points_2d->set_x(msg.parking_walls[i].points_2d[j].x);
      points_2d->set_y(msg.parking_walls[i].points_2d[j].y);
    }
    parking_walls->set_confidence(msg.parking_walls[i].confidence);
    parking_walls->set_life_time(msg.parking_walls[i].life_time);
    parking_walls->set_wall_dim(msg.parking_walls[i].wall_dim);
    parking_walls->set_wall_num(msg.parking_walls[i].wall_num);
    parking_walls->set_wall_reserved(msg.parking_walls[i].wall_reserved);
  }

  // repeated ParkingDeceler parking_decelers
  for (size_t i = 0; i < msg.parking_decelers.size(); i++) {
    AroundViewCameraPerceptionResult::ParkingDeceler *parking_decelers = obj.add_parking_decelers();
    parking_decelers->set_point1_x(msg.parking_decelers[i].point1_x);
    parking_decelers->set_point1_y(msg.parking_decelers[i].point1_y);
    parking_decelers->set_point2_x(msg.parking_decelers[i].point2_x);
    parking_decelers->set_point2_y(msg.parking_decelers[i].point2_y);
    parking_decelers->set_point3_x(msg.parking_decelers[i].point3_x);
    parking_decelers->set_point3_y(msg.parking_decelers[i].point3_y);
    parking_decelers->set_point4_x(msg.parking_decelers[i].point4_x);
    parking_decelers->set_point4_y(msg.parking_decelers[i].point4_y);
    parking_decelers->set_id(msg.parking_decelers[i].id);
    parking_decelers->set_confidence(msg.parking_decelers[i].confidence);
    parking_decelers->set_life_time(msg.parking_decelers[i].life_time);
  }

  // repeated ParkingLane parking_lanes
  for (size_t i = 0; i < msg.parking_lanes.size(); i++) {
    AroundViewCameraPerceptionResult::ParkingLane *parking_lanes = obj.add_parking_lanes();
    parking_lanes->set_id(msg.parking_lanes[i].id);
    for (size_t j = 0; j < msg.parking_lanes[i].points_2d.size(); j++) {
      AroundViewCameraPerceptionResult::Point2f *points_2d = parking_lanes->add_points_2d();
      points_2d->set_x(msg.parking_lanes[i].points_2d[j].x);
      points_2d->set_y(msg.parking_lanes[i].points_2d[j].y);
    }
    parking_lanes->set_confidence(msg.parking_lanes[i].confidence);
    parking_lanes->set_life_time(msg.parking_lanes[i].life_time);
    parking_lanes->set_lane_dim(msg.parking_lanes[i].lane_dim);
    parking_lanes->set_lane_num(msg.parking_lanes[i].lane_num);
    parking_lanes->set_lane_reserved(msg.parking_lanes[i].lane_reserved);
  }

  // repeated ParkingArrow parking_arrows
  for (size_t i = 0; i < msg.parking_arrows.size(); i++) {
    AroundViewCameraPerceptionResult::ParkingArrow *parking_arrows = obj.add_parking_arrows();
    parking_arrows->set_id(msg.parking_arrows[i].id);
    for (size_t j = 0; j < msg.parking_arrows[i].begin_points_2d.size(); j++) {
      AroundViewCameraPerceptionResult::Point2f *begin_points_2d = parking_arrows->add_begin_points_2d();
      begin_points_2d->set_x(msg.parking_arrows[i].begin_points_2d[j].x);
      begin_points_2d->set_y(msg.parking_arrows[i].begin_points_2d[j].y);
    }
    for (size_t j = 0; j < msg.parking_arrows[i].end_points_2d.size(); j++) {
      AroundViewCameraPerceptionResult::Point2f *end_points_2d = parking_arrows->add_end_points_2d();
      end_points_2d->set_x(msg.parking_arrows[i].end_points_2d[j].x);
      end_points_2d->set_y(msg.parking_arrows[i].end_points_2d[j].y);
    }
    parking_arrows->set_confidence(msg.parking_arrows[i].confidence);
    parking_arrows->set_life_time(msg.parking_arrows[i].life_time);
    for (size_t j = 0; j < msg.parking_arrows[i].begin_scores.size(); j++) {
      parking_arrows->add_begin_scores(msg.parking_arrows[i].begin_scores[j]);
    }
    parking_arrows->set_begin_dim(msg.parking_arrows[i].begin_dim);
    parking_arrows->set_begin_num(msg.parking_arrows[i].begin_num);
    parking_arrows->set_begin_reserved(msg.parking_arrows[i].begin_reserved);
    for (size_t j = 0; j < msg.parking_arrows[i].end_scores.size(); j++) {
      parking_arrows->add_end_scores(msg.parking_arrows[i].end_scores[j]);
    }
    parking_arrows->set_end_dim(msg.parking_arrows[i].end_dim);
    parking_arrows->set_end_num(msg.parking_arrows[i].end_num);
    parking_arrows->set_end_reserved(msg.parking_arrows[i].end_reserved);
  }

  // repeated ParkingLeftExtend parking_left_extends
  for (size_t i = 0; i < msg.parking_left_extends.size(); i++) {
    AroundViewCameraPerceptionResult::ParkingLeftExtend *parking_left_extends = obj.add_parking_left_extends();
    parking_left_extends->set_id(msg.parking_left_extends[i].id);
    parking_left_extends->set_point1_x(msg.parking_left_extends[i].point1_x);
    parking_left_extends->set_point1_y(msg.parking_left_extends[i].point1_y);
    parking_left_extends->set_confidence(msg.parking_left_extends[i].confidence);
    parking_left_extends->set_life_time(msg.parking_left_extends[i].life_time);
  }

  // repeated ParkingRightExtend parking_right_extends
  for (size_t i = 0; i < msg.parking_right_extends.size(); i++) {
    AroundViewCameraPerceptionResult::ParkingRightExtend *parking_right_extends = obj.add_parking_right_extends();
    parking_right_extends->set_id(msg.parking_right_extends[i].id);
    parking_right_extends->set_point1_x(msg.parking_right_extends[i].point1_x);
    parking_right_extends->set_point1_y(msg.parking_right_extends[i].point1_y);
    parking_right_extends->set_confidence(msg.parking_right_extends[i].confidence);
    parking_right_extends->set_life_time(msg.parking_right_extends[i].life_time);
  }

  // parking_space_num
  obj.set_parking_space_num(msg.parking_space_num);
  obj.set_parking_limiter_num(msg.parking_limiter_num);
  obj.set_parking_deceler_num(msg.parking_deceler_num);
  obj.set_parking_column_num(msg.parking_column_num);
  obj.set_parking_wall_num(msg.parking_wall_num);
  obj.set_parking_left_extend_num(msg.parking_left_extend_num);
  obj.set_parking_right_extend_num(msg.parking_right_extend_num);
  obj.set_parking_arrow_num(msg.parking_arrow_num);
  obj.set_parking_lane_num(msg.parking_lane_num);
}
