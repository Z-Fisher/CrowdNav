#pragma once
// Copyright 2020 kvedder@seas.upenn.edu
// School of Engineering and Applied Sciences,
// University of Pennsylvania
//
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
// ========================================================================

#include <math.h>
#include <boost/functional/hash.hpp>

#include <algorithm>
#include <limits>
#include <random>
#include <string>
#include <utility>
#include <vector>
#include <list>

#include "cs/path_finding/path_finder.h"
#include "cs/util/constants.h"
#include "cs/util/visualization.h"
#include "libMultiRobotPlanning/bounded_a_star.hpp"
#include "shared/math/geometry.h"
#include "shared/math/math_util.h"

namespace cs {
namespace path_finding {

namespace rrt {
struct Node {

};
} // namespace rrt

template <int max_samples>
class RRT : public PathFinder {
  // Vector of directions
  // Each direction has a collide probability and a cost from the goal
  // 

  // Fuck astar
  // Change path finder interface in header file to take in dpw in constructor
  // change interface to take in ped detector for find path
  // 
 public:
  explicit RRT(const util::vector_map::VectorMap& map,
                 const float& robot_radius,
                 const float& safety_margin,
                 const float& inflation)
      : PathFinder(map, robot_radius, safety_margin, inflation) {}

  Path2f FindPath(const util::DynamicFeatures& dynamic_map,
                  const Eigen::Vector2f& start,
                  const Eigen::Vector2f& goal) override {
    (void) goal;
    (void) dynamic_map;
    const Eigen::Vector2f bad_goal(5.0, 5.0);

    //Path2f test;
    //test.waypoints.push_back(start);
    //const Eigen::Vector2f waypoint(15.0, 5.0);
    //test.waypoints.push_back(waypoint);
    //test.waypoints.push_back(bad_goal);

    Path2f path;
    path.waypoints.push_back(start);
    const Eigen::Vector2f waypoint(15.0, 5.0);
    path.waypoints.push_back(waypoint);
    path.waypoints.push_back(bad_goal);
    prev_path_ = path;
    return path;
    //return SmoothPath(start, dynamic_map, path);
  }
};

}  // namespace path_finding
}  // namespace cs

