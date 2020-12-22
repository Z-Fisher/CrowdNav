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

#include "cs/controllers/nav_controller.h"

#include <string>
#include <utility>
#include <vector>

namespace cs {
namespace controllers {

namespace params {

CONFIG_FLOAT(robot_radius, "pf.kRobotRadius");
CONFIG_FLOAT(safety_margin, "pf.kSafetyMargin");
CONFIG_FLOAT(local_inflation, "path_finding.local_robot_inflation");
CONFIG_FLOAT(global_inflation, "path_finding.global_robot_inflation");
CONFIG_INT(num_drawn_candidates, "rrt.num_paths_visualized");
CONFIG_STRING(map_tf_frame, "frames.map_tf_frame");
CONFIG_STRING(base_link_tf_frame, "frames.base_tf_frame");
CONFIG_STRING(laser_tf_frame, "frames.laser_tf_frame");
CONFIG_INT(cycles_until_refresh, "rrt.cycles_until_refresh");
CONFIG_VECTOR3FLIST(goal_poses, "pf.goal_poses");

}  // namespace params

NavController::NavController(
    cs::main::DebugPubWrapper* dpw,
    const util::LaserScan& laser,
    const util::vector_map::VectorMap& map,
    const state_estimation::StateEstimator& state_estimator,
    const obstacle_avoidance::ObstacleDetector& obstacle_detector,
    const ped_detection::PedDetector& ped_detector,
    const motion_planning::PIDController& motion_planner)
    : Controller(
          dpw, laser, map, state_estimator, obstacle_detector, ped_detector, motion_planner),
      global_path_finder_(map_,
                          params::CONFIG_robot_radius,
                          params::CONFIG_safety_margin,
                          params::CONFIG_global_inflation),
      local_path_finder_(map_,
                         params::CONFIG_robot_radius,
                         params::CONFIG_safety_margin,
                         params::CONFIG_local_inflation),
      current_goal_(params::CONFIG_goal_poses.front()),
      current_goal_index_(0) {}

void DrawPath(cs::main::DebugPubWrapper* dpw,
              const path_finding::Path2f& p,
              const std::string& ns,
              const int& color) {
  dpw->robot_path_pub_.publish(
      visualization::DrawPath(p, params::CONFIG_map_tf_frame, ns, color));
}

void DrawGoal(cs::main::DebugPubWrapper* dpw, const util::Pose& goal) {
  visualization_msgs::MarkerArray goal_marker;
  visualization::DrawPose(
      goal, params::CONFIG_map_tf_frame, "goal_pose", 0, 1, 0, 1, &goal_marker);
  dpw->goal_pub_.publish(goal_marker);
}

bool IsPointCollisionFree(const Eigen::Vector2f& p,
                          const std::vector<Eigen::Vector2f>& df,
                          const float distance_from_df) {
  for (const auto& f : df) {
    if ((p - f).squaredNorm() <= math_util::Sq(distance_from_df)) {
      return false;
    }
  }
  return true;
}

Eigen::Vector2f GetGlobalPathWaypoint(const util::Pose& current_pose,
                                      const path_finding::Path2f& path,
                                      const std::vector<Eigen::Vector2f>& obs,
                                      const float distance_from_df) {
  if (path.waypoints.size() > 1) {
    for (size_t i = 1; i < path.waypoints.size(); ++i) {
      const auto& w = path.waypoints[i];
      if (IsPointCollisionFree(w, obs, distance_from_df)) {
        return w;
      }
    }
  }
  return current_pose.tra;
}

float GetAngleFacingWaypoint(const Eigen::Vector2f& p,
                             const Eigen::Vector2f& waypoint) {
  const Eigen::Vector2f d = waypoint - p;
  return math_util::AngleMod(std::atan2(d.y(), d.x()));
}

util::Pose GetLocalPathPose(const util::Pose& current_pose,
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
          GetAngleFacingWaypoint(current_pose.tra, next_waypoint)};
}

void NavController::RefreshGoal() {
  if (motion_planner_.AtPose(current_goal_)) {
    ++current_goal_index_;
    current_goal_ =
        util::Pose(params::CONFIG_goal_poses[current_goal_index_ %
                                             params::CONFIG_goal_poses.size()]);
  }
}

std::pair<ControllerType, util::Twist> NavController::Execute() {
  const auto est_pose = state_estimator_.GetEstimatedPose();
  const auto est_vel = state_estimator_.GetEstimatedVelocity();
  ROS_INFO("Robot pose: %f, %f", est_pose.tra.x(), est_pose.tra.y());

  const auto laser_points_wf = laser_.TransformPointsFrameSparse(
      est_pose.ToAffine(), [this](const float& d) {
        return (d > laser_.ros_laser_scan_.range_min) &&
               (d <= laser_.ros_laser_scan_.range_max);
      });

  const float total_margin =
      (params::CONFIG_robot_radius + params::CONFIG_safety_margin + kEpsilon) *
      params::CONFIG_local_inflation;
      
  // UNCOMMENT FOR ESCAPE COLLISION FUNCTIONALITY
  // if (!IsPointCollisionFree(est_pose.tra, laser_points_wf, total_margin)) {
  //   return {ControllerType::ESCAPE_COLLISION, {}};
  // }

  static util::Pose local_waypoint;
  static int cycle_num = 0;
  if (cycle_num == params::CONFIG_cycles_until_refresh) { 
    RefreshGoal();

    global_path_finder_.PlanPath(est_pose.tra, current_goal_.tra);
    const auto global_path = global_path_finder_.GetPath();
    DrawPath(dpw_, global_path, "global_path", 0);
    const Eigen::Vector2f global_waypoint = GetGlobalPathWaypoint(
        est_pose, global_path, laser_points_wf, total_margin);
    
    // ROS_ERROR("!ENTERING LOCAL PATH FINDER");
    const auto local_path = local_path_finder_.FindPath(
        ped_detector_, obstacle_detector_.GetDynamicFeatures(), motion_planner_, est_pose.tra, global_waypoint, est_vel.tra,
        est_pose, current_goal_);
    local_waypoint = GetLocalPathPose(est_pose, global_waypoint, current_goal_, local_path);
    DrawPath(dpw_, local_path, "local_path", 2);
    const auto candidates = local_path_finder_.GetCandidatePaths(
      params::CONFIG_num_drawn_candidates);
    for (int i = 0; i < (int)candidates.size(); i++) {
      DrawPath(dpw_, candidates[i], "candidate_path" + std::to_string(i), 1);
    }
    if (local_path.waypoints.empty()) {
      ROS_INFO("Local path planner failed.");
    }
    DrawGoal(dpw_, local_waypoint);
    cycle_num = 0;
  }
  cycle_num++;
  // ROS_ERROR("\n\n RIGHT BEFORE DRIVE TO POSE");
  const util::Twist command = motion_planner_.DriveToPose(
      obstacle_detector_.GetDynamicFeatures(), local_waypoint);
  
  // ROS_ERROR("\n\n\n\n\n actual command: %f, %f, %f, local_waypoint: %f, %f", command.tra.x(), command.tra.y(), command.rot, local_waypoint.tra.x(), local_waypoint.tra.y());

  return {ControllerType::NAVIGATION, command};
}

void NavController::Reset() {}

}  // namespace controllers
}  // namespace cs
