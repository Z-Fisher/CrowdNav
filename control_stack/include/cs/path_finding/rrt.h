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

namespace params {
CONFIG_INT(num_samples, "rrt.num_samples");
CONFIG_FLOAT(cost_bias, "rrt.cost_bias");
CONFIG_FLOAT(path_length, "rrt.path_length");
CONFIG_FLOAT(ped_var_scale, "rrt.ped_var_scale");
CONFIG_FLOAT(ped_var_power, "rrt.ped_var_power");
CONFIG_FLOAT(ped_var_bias, "rrt.ped_var_bias");
CONFIG_FLOAT(robot_radius, "rrt.robot_radius");
CONFIG_FLOAT(collision_buffer, "rrt.collision_buffer");
CONFIG_FLOAT(t_horizon, "rrt.t_horizon");
CONFIG_FLOAT(switch_discount, "rrt.switch_discount");

// CONFIG_FLOAT(y_vel_scale, "rrt.y_vel_scale");
// CONFIG_FLOAT(x_vel_scale, "rrt.x_vel_scale");

CONFIG_FLOAT(vel_scale, "rrt.vel_scale");
CONFIG_FLOAT(atan_scale, "rrt.atan_scale");

}  // namespace params

template <int max_samples>
class RRT : public PathFinder {
 private:
  std::vector<Path2f> paths_;

  float get_angle_facing_waypoint(const Eigen::Vector2f& p,
                             const Eigen::Vector2f& waypoint) {
  const Eigen::Vector2f d = waypoint - p;
  return math_util::AngleMod(std::atan2(d.y(), d.x()));
}

  util::Pose get_local_path_pose_(const util::Pose& current_pose,
                            const Eigen::Vector2f& global_waypoint,
                            const util::Pose& goal_pose,
                            const path_finding::Path2f& path) {
  if (path.waypoints.size() <= 1) {
    const auto delta = (global_waypoint - current_pose.tra);
    const float delta_dist = delta.norm();
    const float angle = std::atan2(delta.y(), delta.x());
    return {current_pose.tra + delta.normalized() * (delta_dist * 0.66), angle};
  }

  const Eigen::Vector2f& next_waypoint = path.waypoints[1];
  if (next_waypoint == goal_pose.tra) {
    return goal_pose;
  }

  return {next_waypoint,
           get_angle_facing_waypoint(current_pose.tra, next_waypoint)};
}

  // Calculates euclidian distance from path end point to goal
  float euclid_dist_(const Path2f& path, const util::Pose& goal) {
    return sqrt(pow(path.waypoints.back()[0] - goal.tra.x(), 2) + pow(path.waypoints.back()[1] - goal.tra.y(), 2));
  }

  // Calculates to probability of colliding with a pedestrian along a path
  float find_collision_prob_(const ped_detection::PedDetector& ped_detector, 
     const util::DynamicFeatures& dynamic_features,
                  const motion_planning::PIDController& motion_planner,
                  Path2f& path,
    const Eigen::Vector2f&  start, Eigen::Vector2f  vel,
    const util::Pose& current_pose,
    const Eigen::Vector2f& global_waypoint,
    const util::Pose& goal_pose) {

    (void) start;

    Eigen::Vector2f est_vel;

    est_vel[0] = vel.x();
    est_vel[1] = vel.y();
    
    util::Pose local_waypoint =
      get_local_path_pose_(current_pose, global_waypoint, goal_pose, path);
    util::Twist command = motion_planner.DriveToPose(
      dynamic_features, local_waypoint);
    Eigen::Vector2f new_vel;
    // new_vel[0] = command.tra.x() * params::CONFIG_x_vel_scale;
    float y_diff = path.waypoints.back().y() - path.waypoints.front().y();

    float x_diff = path.waypoints.back().x() - path.waypoints.front().x();
    float theta = atan(params::CONFIG_atan_scale * y_diff/ x_diff);
    float x_vel = cos(theta) * command.tra.x();
    float y_vel = sin(theta) * command.tra.x();

    if ((x_diff < 0 && x_vel > 0) || (x_diff > 0 && x_vel < 0)) {
      x_vel *= -1;
    }

    if ((y_diff < 0 && y_vel > 0) || (y_diff > 0 && y_vel < 0)) {
      y_vel *= -1;
    }
    new_vel[0] =  params::CONFIG_vel_scale * x_vel;
    new_vel[1] = params::CONFIG_vel_scale * y_vel;

    // new_vel[1] = params::CONFIG_y_vel_scale * y_diff * command.tra.x();
    // ROS_ERROR("\n\ny diff : %f", y_diff);
    // ROS_ERROR("vel x: %f, vel y: %f", command.tra.x(), command.tra.y());
    vel = new_vel;
    path.v0 = util::Twist(vel.x(), vel.y(), command.rot);
    // ROS_ERROR("\n\nvel  vx %f, vy : %f", vel.x(), vel.y());

    // ROS_ERROR("path vel x: %f, vel y: %f", path.v0.tra.x(), path.v0.tra.y());
    // ROS_INFO("\nNew Loop");

    float prob_no_collision = 1;
    // ROS_ERROR("\n\n\n NEW PED LOOP");
    for (auto ped: ped_detector.GetPeds().peds) {
      float ped_vel_x = ped.vel.tra.x() + est_vel.x();
    
      // Calculate t_min
      float numerator_x = (ped.pose.tra.x()) * (vel.x() - ped_vel_x);
      float numerator_y = (ped.pose.tra.y()) * (vel.y() - ped.vel.tra.y());
      // float numerator_x = (ped.pose.tra.x() - start.x()) * (vel.x() - ped.vel.tra.x());
      // float numerator_y = (ped.pose.tra.y() - start.y()) * (vel.y() - ped.vel.tra.y());
      float numerator = numerator_x + numerator_y;
      float denom = (pow(vel.x() - ped_vel_x, 2) + pow(vel.y() - ped.vel.tra.y(), 2));
      float t_min = numerator / denom;

      if (t_min > params::CONFIG_t_horizon) {
        t_min = params::CONFIG_t_horizon;
      }

      if (t_min < 0) {
        t_min = 0;
      }

      // Calculate ped distrubtion params
      float sigma = sqrt(params::CONFIG_ped_var_bias + params::CONFIG_ped_var_scale * pow(t_min, params::CONFIG_ped_var_power));
      float mean_x = ped.pose.tra.x() + ped_vel_x * t_min;
      float mean_y = ped.pose.tra.y()  + ped.vel.tra.y() * t_min;

      // float x_robot = start.x() + vel.x() * t_min;
      // float y_robot = start.y() + vel.y() * t_min;

      float x_robot = vel.x() * t_min;
      float y_robot = vel.y() * t_min;
      float collision_radius = params::CONFIG_robot_radius + ped.radius + params::CONFIG_collision_buffer;

      float x_lo = x_robot - collision_radius / 2;
      float x_hi = x_robot + collision_radius / 2;
      float y_lo = y_robot - collision_radius / 2;
      float y_hi = y_robot + collision_radius / 2;


      float cdf_hi_x = 0.5 * std::erfc(-(x_hi- mean_x)/(sigma*sqrt(2)));
      float cdf_lo_x = 0.5 * std::erfc(-(x_lo- mean_x)/(sigma*sqrt(2)));
      float p_x = cdf_hi_x - cdf_lo_x;

      float cdf_hi_y = 0.5 * std::erfc(-(y_hi- mean_y)/(sigma*sqrt(2)));
      float cdf_lo_y = 0.5 * std::erfc(-(y_lo- mean_y)/(sigma*sqrt(2)));
      float p_y = cdf_hi_y - cdf_lo_y;
      float prob_single_collision = p_x * p_y;

      // if (ped.pose.tra.norm() < 4) {
      //   ROS_ERROR("\n\npedx: %f, pedy: %f, pedvx: %f, pedvy: %f,  pathx: %f, pathy: %f, robvx: %f, robvy: %f, t_min: %f, sigma: %f, coll_rad: %f,  p_coll: %f", 
      //     ped.pose.tra.x(), 
      //     ped.pose.tra.y(), 
      //     ped.vel.tra.x(),
      //     ped.vel.tra.y(),
      //     x_diff, 
      //     y_diff,
      //     vel.x(),
      //     vel.y(),
      //     t_min,
      //     sigma,
      //     collision_radius,
      //     prob_single_collision);
      // }
      
      prob_no_collision *= (1 - prob_single_collision);
    }

    return 1 - prob_no_collision;
  }


  // Calculates cost for a path based on its euclidian distance to the goal and the probability of
  // colliding with a pedestrian along the path
  void calculate_cost_(
    Path2f& path,
    const ped_detection::PedDetector& ped_detector,
    const util::DynamicFeatures& dynamic_features,
    const motion_planning::PIDController& motion_planner,
    const Eigen::Vector2f& start,
    const Eigen::Vector2f& goal,
    const Eigen::Vector2f& vel,
    const util::Pose& current_pose,
    const util::Pose& goal_pose,
    const float worst_dist
    ) {
      float collision_prob = find_collision_prob_(ped_detector, dynamic_features, motion_planner, path, start, vel, 
      current_pose, goal, goal_pose);
      float dist_from_goal = euclid_dist_(path, goal_pose) / worst_dist;
      float cost = dist_from_goal + params::CONFIG_cost_bias * collision_prob;
      path.collision_prob = collision_prob;
      path.dist_from_goal = dist_from_goal;
      path.cost  = cost;
      //ROS_ERROR("Path in paths_ : vel  x: %f, y: %f, euclid: %f, collision_prob: %f, cost: %f", path.v0.tra.x(), path.v0.tra.y(), path.dist_from_goal, path.collision_prob, path.cost);

  }
 public:
  explicit RRT(const util::vector_map::VectorMap& map,
                 const float& robot_radius,
                 const float& safety_margin,
                 const float& inflation)
      : PathFinder(map, robot_radius, safety_margin, inflation), paths_() {}

  // Calculates the cost for each sampled path and returns the lowest cost path
  Path2f FindPath(const ped_detection::PedDetector& ped_detector,
                  const util::DynamicFeatures& dynamic_features,
                  const motion_planning::PIDController& motion_planner,
                  const Eigen::Vector2f&  start,
                  const Eigen::Vector2f& goal,
                  const Eigen::Vector2f& est_vel,
                  const util::Pose& current_pose,
                  const util::Pose& goal_pose) {    
    paths_.clear();
    //ROS_ERROR("goal: %f, %f", goal[0], goal[1]);
    //ROS_ERROR("goal_pose: %f, %f", goal_pose.tra.x(), goal_pose.tra.y());
    //ROS_ERROR("prev path index %d: cost = %f", prev_path_.index, prev_path_.cost);
    // Samples paths around the clock based on number of samples

    float worst_dist = 0;
    for (int i = 0; i < params::CONFIG_num_samples; i++) {
      float theta = 2 * i * M_PI / params::CONFIG_num_samples;
      const Eigen::Vector2f sample(params::CONFIG_path_length * sin(theta), 
                                   params::CONFIG_path_length * cos(theta));
      Path2f path;
      path.index = i;

      path.waypoints.push_back(start);
      path.waypoints.push_back(start + sample);
      worst_dist = std::max(worst_dist, euclid_dist_(path, goal_pose));
    }

    for (int i = 0; i < params::CONFIG_num_samples; i++) {
      float theta = 2 * i * M_PI / params::CONFIG_num_samples;
      const Eigen::Vector2f sample(params::CONFIG_path_length * sin(theta), 
                                   params::CONFIG_path_length * cos(theta));
      Path2f path;
      path.index = i;

      path.waypoints.push_back(start);
      path.waypoints.push_back(start + sample);
      calculate_cost_(path, ped_detector, dynamic_features, motion_planner, start, goal, est_vel,
        current_pose, goal_pose, worst_dist);
      if (prev_path_.index == i) {
        path.cost *= params::CONFIG_switch_discount;
      }
      //ROS_ERROR("Candidate velocity: x: %.2f, y: %.2f, cost: %.2f", path.v0.tra.x(), path.v0.tra.y(), path.cost);
      paths_.push_back(path);
      // ROS_ERROR("Path: i: %i, x: %f, y: %f, dist: %f, p_collision: %f, cost: %f", 
      // i, path.waypoints[1].x(), path.waypoints[1].y(), path.dist_from_goal, path.collision_prob, path.cost);
    }

    // Find the minimum cost path
    // float min_ped_dist = 100;
    // for (auto ped: ped_detector.GetPeds().peds) {
    //   min_ped_dist = std::min(min_ped_dist, ped.pose.tra.norm());
    //    }
    // if (min_ped_dist <= 100) {
    //   ROS_ERROR("\n\nPRINTING PATHS at dist: %f", min_ped_dist);
    //   for (Path2f& path_: paths_) {
    //     ROS_ERROR("Path in paths_ : delta x: %f, y: %f; vel  x: %f, y: %f; euclid: %f, collision_prob: %f, cost: %f", 
    //     path_.waypoints.back().x() - path_.waypoints.front().x(),
    //     path_.waypoints.back().y() - path_.waypoints.front().y(),
    //     path_.v0.tra.x(), 
    //     path_.v0.tra.y(), 
    //     path_.dist_from_goal, path_.collision_prob, path_.cost);
    //   }
    // }
    Path2f min_cost_path = *std::min_element(begin(paths_), end(paths_),
                                [](const Path2f& a, const Path2f& b){
      return a.cost < b.cost;
    });

    // ROS_INFO("start: %f, %f", start[0], start[1]);
    // ROS_INFO("Path waypoint size: %i", (int) paths_.size());

    prev_path_ = min_cost_path;
    // if (min_ped_dist <= 100) {
    //   ROS_ERROR("\n\nMin path delta: x = %f, y = %f; velocity: vx = %f, vy = %f ; collision_prob: %f;  cost: %f", 
    //       min_cost_path.waypoints.back().x() - min_cost_path.waypoints.front().x(),
    //       min_cost_path.waypoints.back().y() - min_cost_path.waypoints.front().y(),
    //   min_cost_path.v0.tra.x(), min_cost_path.v0.tra.y(), min_cost_path.collision_prob, min_cost_path.cost);
    // }
    return min_cost_path;
    //return SmoothPath(start, dynamic_map, path);
  }

  std::vector<Path2f> GetCandidatePaths(int num_paths) override {
    std::sort(paths_.begin(), paths_.end(),[](const Path2f& a, const Path2f& b){
      return a.cost < b.cost;
    });
    return std::vector<Path2f>(paths_.begin() + 1, paths_.begin() + num_paths);
  }
};

}  // namespace path_finding
}  // namespace cs