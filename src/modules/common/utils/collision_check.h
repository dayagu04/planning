#ifndef __UTILS__COLLISION_CHECK_HPP__
#define __UTILS__COLLISION_CHECK_HPP__
#include <cmath>
#include <utility>
#include <vector>
#include "cartesian_coordinate_system.h"

/*!
 * @brief Given two cycles find the distance.
 * @param cycle 1
 * @param cycle 2
 * @return distance.
 */
double GetCyclesDistance(const std::pair<Point2D, double>& cycle1,
                         const std::pair<Point2D, double>& cycle2);
/*!
 * @brief Given two sets of cycles, find the minimal distance between two sets.
 * @param cycle set 1
 * @param cycle set 2
 * @return distance.
 */
double GetCycleSetsDistance(
    const std::vector<std::pair<Point2D, double>>& cycle_set1,
    const std::vector<std::pair<Point2D, double>>& cycle_set2);

/*!
 * @brief Given two rectangles represented by 4 points . Find the distance
 * between two rectangles.
 * @param rectangle 1
 * @param rectangle 2
 * @return distance.
 */
double GetRectanglesDistance(const std::vector<Point2D>& rect1,
                             const std::vector<Point2D>& rect2);

/*!
 * @brief Given two axis aligned rectangles represented by 2 points (bottom-left
 * point and upper-right point), compute the distance represented by a tuple of
 * x-axis distance and y-axis distance
 * @param rectangle 1
 * @param rectangle 2
 * @return tuple of x axis distance and y axis distance.
 */
std::pair<double, double> GetAxisAlignedRectanglesDistances(
    const std::pair<Point2D, Point2D>& aligned_rect1,
    const std::pair<Point2D, Point2D>& aligned_rect2);
#endif
