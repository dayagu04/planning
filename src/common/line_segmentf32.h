#pragma once
#include <cmath>
#include <string>

#include "vecf32.h"

namespace planning {

/**
 * @class LineSegmentf32
 * @brief Line segment in 2-D. todo: change it to template.
 */
class LineSegmentf32 {
 public:
  /**
   * @brief Empty constructor.
   */
  LineSegmentf32();

  /**
   * @brief Constructor with start point and end point.
   * @param start The start point of the line segment.
   * @param end The end point of the line segment.
   */
  LineSegmentf32(const Vec2df32 &start, const Vec2df32 &end);

  /**
   * @brief Get the start point.
   * @return The start point of the line segment.
   */
  const Vec2df32 &start() const { return start_; }

  /**
   * @brief Get the end point.
   * @return The end point of the line segment.
   */
  const Vec2df32 &end() const { return end_; }

  /**
   * @brief Get the unit direction from the start point to the end point.
   * @return The start point of the line segment.
   */
  const Vec2df32 &unit_direction() const { return unit_direction_; }

  /**
   * @brief Get the center of the line segment.
   * @return The center of the line segment.
   */
  Vec2df32 center() const { return (start_ + end_) / 2.0; }

  /**
   * @brief Get the heading of the line segment.
   * @return The heading, which is the angle between unit direction and x-axis.
   */
  float heading() const { return heading_; }

  void set_heading(float heading) { heading_ = heading; }
  /**
   * @brief Get the cosine of the heading.
   * @return The cosine of the heading.
   */
  float cos_heading() const { return unit_direction_.x(); }

  /**
   * @brief Get the sine of the heading.
   * @return The sine of the heading.
   */
  float sin_heading() const { return unit_direction_.y(); }
  float line_a() const { return line_a_; }

  float line_b() const { return line_b_; }

  float line_c() const { return line_c_; }

  float unit_a() const { return unit_a_; }

  float unit_b() const { return unit_b_; }

  float unit_c() const { return unit_c_; }
  /**
   * @brief Get the length of the line segment.
   * @return The length of the line segment.
   */
  float length() const;

  /**
   * @brief Get the square of length of the line segment.
   * @return The square of length of the line segment.
   */
  float length_sqr() const;

  /**
   * @brief Compute the project distance from a point in 2-D to the line
   * segment.
   * @param point The point to compute the distance to.
   * @return The project distance which is positive or negative or zero.
   */
  float RawDistanceTo(const Vec2df32 &point) const;

  /**
   * @brief Compute the shortest distance from a point on the line segment
   *        to a point in 2-D.
   * @param point The point to compute the distance to.
   * @return The shortest distance from points on the line segment to point.
   */
  float DistanceTo(const Vec2df32 &point) const;

  /**
   * @brief Compute the shortest distance from a point on the line segment
   *        to a point in 2-D, and get the nearest point on the line segment.
   * @param point The point to compute the distance to.
   * @param nearest_pt The nearest point on the line segment
   *        to the input point.
   * @return The shortest distance from points on the line segment
   *         to the input point.
   */
  float DistanceTo(const Vec2df32 &point, Vec2df32 *const nearest_pt) const;

  /**
   * @brief Compute the square of the shortest distance from a point
   *        on the line segment to a point in 2-D.
   * @param point The point to compute the squared of the distance to.
   * @return The square of the shortest distance from points
   *         on the line segment to the input point.
   */
  float DistanceSquareTo(const Vec2df32 &point) const;

  /**
   * @brief Compute the square of the shortest distance from a point
   *        on the line segment to a point in 2-D,
   *        and get the nearest point on the line segment.
   * @param point The point to compute the squared of the distance to.
   * @param nearest_pt The nearest point on the line segment
   *        to the input point.
   * @return The shortest distance from points on the line segment
   *         to the input point.
   */
  float DistanceSquareTo(const Vec2df32 &point, Vec2df32 *const nearest_pt) const;

  /**
   * @brief Check if a point is within the line segment.
   * @param point The point to check if it is within the line segment.
   * @return Whether the input point is within the line segment or not.
   */
  bool IsPointIn(const Vec2df32 &point) const;

  /**
   * @brief Check if the line segment has an intersect
   *        with another line segment in 2-D.
   * @param other_segment The line segment to check if it has an intersect.
   * @return Whether the line segment has an intersect
   *         with the input other_segment.
   */
  bool HasIntersect(const LineSegmentf32 &other_segment) const;

  /**
   * @brief Compute the intersect with another line segment in 2-D if any.
   * @param other_segment The line segment to compute the intersect.
   * @param point the computed intersect between the line segment and
   *        the input other_segment.
   * @return Whether the line segment has an intersect
   *         with the input other_segment.
   */
  bool GetIntersect(const LineSegmentf32 &other_segment,
                    Vec2df32 *const point) const;
  /**
   * @brief Compute the intersect with another straight line in 2-D if any.
   * @param other_segment The straight line to compute the intersect.
   * @param point the computed intersect between two straight lines.
   * @return Whether the two straight lines have an intersect
   */
  bool GetStraightLineIntersect(const LineSegmentf32 &other_segment,
                                Vec2df32 *const point) const;
  /**
   * @brief Compute the projection of a vector onto the line segment.
   * @param point The end of the vector (starting from the start point of the
   *        line segment) to compute the projection onto the line segment.
   * @return The projection of the vector, which is from the start point of
   *         the line segment to the input point, onto the unit direction.
   */
  float ProjectOntoUnit(const Vec2df32 &point) const;

  /**
   * @brief Compute the cross product of a vector onto the line segment.
   * @param point The end of the vector (starting from the start point of the
   *        line segment) to compute the cross product onto the line segment.
   * @return The cross product of the unit direction and
   *         the vector, which is from the start point of
   *         the line segment to the input point.
   */
  float ProductOntoUnit(const Vec2df32 &point) const;

  /**
   * @brief Compute perpendicular foot of a point in 2-D on the straight line
   *        expanded from the line segment.
   * @param point The point to compute the perpendicular foot from.
   * @param foot_point The computed perpendicular foot from the input point to
   *        the straight line expanded from the line segment.
   * @return The distance from the input point to the perpendicular foot.
   */
  float GetPerpendicularFoot(const Vec2df32 &point,
                              Vec2df32 *const foot_point) const;

  float min_x() const { return min_x_; }
  float max_x() const { return max_x_; }
  float min_y() const { return min_y_; }
  float max_y() const { return max_y_; }

  Vec2df32 GetPoint(float s) const;

 private:
  Vec2df32 start_;
  Vec2df32 end_;
  Vec2df32 unit_direction_;
  float heading_ = 0.0;
  float length_ = 0.0;

  void InitMaxMin();

  float min_x_;
  float max_x_;
  float min_y_;
  float max_y_;

  float line_a_ = 0.;
  float line_b_ = 0.;
  float line_c_ = 0.;

  float unit_a_ = 0.;
  float unit_b_ = 0.;
  float unit_c_ = 0.;
};

}  // namespace planning
