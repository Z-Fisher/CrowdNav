#pragma once
// Copyright 2019 - 2020 kvedder@seas.upenn.edu
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

#include <ros/ros.h>
#include <eigen3/Eigen/Core>
#include <utility>
#include <vector>

#include "cs/obstacle_avoidance/obstacle_detector.h"
#include "cs/obstacle_avoidance/ped_detector.h"
#include "cs/util/map.h"
#include "cs/util/pose.h"

#include "config_reader/macros.h"

namespace cs {
namespace path_finding {

template <typename Position, typename Cost>
struct Path {
  std::vector<Position> waypoints;
  Cost cost;
  float dist_from_goal;
  float collision_prob;
  util::Twist v0;
  int index;

  Path() : waypoints(), cost(0), dist_from_goal(INFINITY), collision_prob(0), v0(util::Twist(0, 0, 0)), index(-1) {}
  Path(const std::vector<Position>& waypoints, const Cost& cost, const double dist_from_goal, const float collision_prob)
      : waypoints(waypoints), cost(cost), dist_from_goal(dist_from_goal), collision_prob(collision_prob), v0(util::Twist(0, 0, 0)), index(-1) {}

  Path(const Path& p) : waypoints(p.waypoints), cost(p.cost), dist_from_goal(p.dist_from_goal), collision_prob(p.collision_prob), v0(util::Twist(p.v0.tra.x(), p.v0.tra.y(), p.v0.rot)), index(p.index) {}
  Path(Path&& p) : waypoints(std::move(p.waypoints)), cost(std::move(p.cost)), v0(p.v0), index(std::move(p.index)) {}
  Path& operator=(const Path& p) {
    this->waypoints = p.waypoints;
    this->cost = p.cost;
    this->index = p.index;
    return *this;
  }
  Path& operator=(Path&& p) {
    this->waypoints = std::move(p.waypoints);
    this->cost = std::move(p.cost);
    this->index = std::move(p.index);
    return *this;
  }

  bool IsValid() const { return !waypoints.empty(); }

  void SetV0(util::Twist v) {v0 = v;}

  template <typename Transform>
  Path TransformPath(const Transform& t) const {
    Path path = *this;
    for (auto& p : path.waypoints) {
      p = t * p;
    }
    return path;
  }
};

using Path2f = Path<Eigen::Vector2f, float>;

class PathFinder {
 protected:
  const util::vector_map::VectorMap& map_;
  const float& robot_radius_;
  const float& safety_margin_;
  const float& inflation_;
  Path2f prev_path_;
  double prev_path_time_;

  bool IsLineColliding(const util::DynamicFeatures& dynamic_map,
                       const Eigen::Vector2f& p1,
                       const Eigen::Vector2f& p2) const;

  bool IsPathColliding(const util::DynamicFeatures& dynamic_map,
                       const Path2f& path) const;

  Path2f UsePrevPathOrUpdate(const util::DynamicFeatures& dynamic_map,
                             const Path2f& proposed_path);

  Path2f SmoothPath(const Eigen::Vector2f& start,
                    const util::DynamicFeatures& dynamic_map,
                    Path2f path) const;

 public:
  PathFinder(const util::vector_map::VectorMap& map,
             const float& robot_radius,
             const float& safety_margin,
             const float& inflation)
      : map_(map),
        robot_radius_(robot_radius),
        safety_margin_(safety_margin),
        inflation_(inflation) {}
        
  virtual std::vector<Path2f> GetCandidatePaths(int num_paths) = 0;

};

}  // namespace path_finding
}  // namespace cs
