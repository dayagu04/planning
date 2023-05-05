#pragma once

#include <cstdint>
#include <cstdio>
#include <string>
#include <vector>
#include "log.h"

namespace planning {
struct Shape3f {
  float height;
  float width;
  float length;
};

struct Shape2f {
  float height;
  float width;
};

struct Rect4f {
  float left;
  float top;
  float right;
  float bottom;
};

struct FrameMeta {
  uint64_t timestamp_us;
  uint64_t sequence;
  std::string tag;
};

struct Point2f {
  float x;
  float y;
};

struct Point2d {
  double x;
  double y;


  Point2d() = default;
  Point2d(double xx, double yy) : x(xx), y(yy) {}
};

struct Point3f {
  float x;
  float y;
  float z;
};

struct Point3d {
  double x;
  double y;
  double z;

  Point3d() = default;
  Point3d(double xx, double yy, double zz) : x(xx), y(yy), z(zz) {}
};

struct Shape3d {
  double length;
  double width;
  double height;
};

struct Polygon3f {
  std::vector<Point3f> points;
};

struct Polyline3f {
  std::vector<Point3f> points;
};

struct Vector2f {
  float x;
  float y;
};

enum Status {
  OK,
  FAILURE,
};

struct Interval {
  double begin;
  double end;
};

struct Matrix2f {
  float x00;
  float x01;
  float x10;
  float x11;
};

enum Direction {
  DIRECTION_UNKNOWN = 0,
  DIRECTION_GO_STRAIGHT = 1,
  DIRECTION_TURN_RIGHT = 2,
  DIRECTION_TURN_LEFT = 4,
  DIRECTION_U_TURN_LEFT = 8,
  DIRECTION_U_TURN_RIGHT = 16,
};

enum MapPOIType {
  MAP_POI_TYPE_UNKNOWN = 0,
  MAP_POI_TYPE_PARKING_LOT = 1,
  MAP_POI_TYPE_HUMAN_ACCESS = 2,
  MAP_POI_TYPE_DESTINATION = 3,
  MAP_POI_TYPE_PARKING = 4,
  MAP_POI_TYPE_BARRIER_GAP = 5,
  MAP_POI_TYPE_FACILITY_ENTRANCE = 6,
  MAP_POI_TYPE_FACILITY_EXIT = 7,
  MAP_POI_TYPE_FACILITY_EXIT_AND_ENTRANCE = 8,
  MAP_POI_TYPE_BUS_STOP = 9,
  MAP_POI_TYPE_GARAGE_ENTRANCE = 10,
  MAP_POI_TYPE_GARAGE_EXIT = 11,
  MAP_POI_TYPE_SPEED_BUMP = 12,
  MAP_POI_TYPE_CROSS_WALK = 13,
  MAP_POI_TYPE_DASHED_SEGMENT = 14,
  MAP_POI_TYPE_CENTRAL_CIRCLE = 15,
  MAP_POI_TYPE_NO_PARKING_ZONE = 16,
  MAP_POI_TYPE_ROAD_MERGE = 17
};

} // namespace planning
