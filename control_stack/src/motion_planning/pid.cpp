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

#include <algorithm>
#include <limits>
#include <utility>

#include "cs/motion_planning/pid.h"
#include "cs/util/physics.h"
#include "shared/util/array_util.h"

#include "config_reader/macros.h"

namespace cs {
namespace motion_planning {

namespace params {
CONFIG_FLOAT(rotation_drive_threshold, "control.rotation_drive_threshold");
CONFIG_FLOAT(rotation_p, "control.rotation_p");
CONFIG_FLOAT(translation_p, "control.translation_p");
CONFIG_FLOAT(goal_deadzone_tra, "control.goal_deadzone_tra");
CONFIG_FLOAT(goal_deadzone_rot, "control.goal_deadzone_rot");

CONFIG_FLOAT(stop_past_goal_threshold, "control.stop_past_goal_threshold");
CONFIG_FLOAT(stop_past_goal_dampener, "control.stop_past_goal_dampener");

CONFIG_FLOAT(kMaxTraAcc, "limits.kMaxTraAcc");
CONFIG_FLOAT(kMaxTraVel, "limits.kMaxTraVel");
CONFIG_FLOAT(kMaxRotAcc, "limits.kMaxRotAcc");
CONFIG_FLOAT(kMaxRotVel, "limits.kMaxRotVel");

CONFIG_FLOAT(robot_radius, "pf.kRobotRadius");
CONFIG_FLOAT(safety_margin, "pf.kSafetyMargin");

CONFIG_FLOAT(translational_cost_scale_factor, "od.kTranslationCostScaleFactor");

}  // namespace params

util::Pose PIDController::EscapeCollisionPose(
    const util::DynamicFeatures& dynamic_features) const {
  const auto est_world_pose_ = state_estimator_.GetEstimatedPose();
  NP_CHECK(!dynamic_features.features.empty());
  Eigen::Vector2f closest_feature = dynamic_features.features.front();
  float closest_feature_distance_sq =
      (est_world_pose_.tra - closest_feature).squaredNorm();
  for (const auto& f : dynamic_features.features) {
    const float dsq = (est_world_pose_.tra - f).squaredNorm();
    if (dsq < closest_feature_distance_sq) {
      closest_feature_distance_sq = dsq;
      closest_feature = f;
    }
  }
  NP_CHECK_LE(closest_feature_distance_sq,
              math_util::Sq(params::CONFIG_robot_radius +
                            params::CONFIG_safety_margin));
  const Eigen::Vector2f cf_delta = closest_feature - est_world_pose_.tra;
  const Eigen::Vector2f waypoint_tra =
      -cf_delta.normalized() *
          (params::CONFIG_robot_radius + params::CONFIG_safety_margin) +
      est_world_pose_.tra;
  return {waypoint_tra, est_world_pose_.rot};
}

util::Twist PIDController::EscapeCollision(const util::Pose& pose) const {
  return ApplyCommandLimits(ProposeCommand(pose));
}

util::Twist PIDController::DriveToPose(
    const util::DynamicFeatures& dynamic_features,
    const util::Pose& waypoint) const {
  const auto est_velocity_ = state_estimator_.GetEstimatedVelocity();

  const auto proposed_command = ProposeCommand(waypoint);
  const auto limited_command = ApplyCommandLimits(proposed_command);

  const auto limited_command_res =
      IsCommandColliding(limited_command, dynamic_features);
  if (!limited_command_res.first) {
    return limited_command;
  }
  const TrajectoryRollout& limited_command_tr = limited_command_res.second;

  static constexpr int kNumAlt = 11;
  auto costs = array_util::MakeArray<kNumAlt>(0.0f);
  std::array<util::Twist, kNumAlt> alternate_commands = {{
      {0, 0, 0},
      {0, 0, params::CONFIG_kMaxRotVel},
      {0, 0, -params::CONFIG_kMaxRotVel},
      {est_velocity_.tra, params::CONFIG_kMaxRotVel},
      {est_velocity_.tra, -params::CONFIG_kMaxRotVel},
      {est_velocity_.tra / 2, params::CONFIG_kMaxRotVel},
      {est_velocity_.tra / 2, -params::CONFIG_kMaxRotVel},
      {est_velocity_.tra / 2, params::CONFIG_kMaxRotVel / 2},
      {est_velocity_.tra / 2, -params::CONFIG_kMaxRotVel / 2},
      {est_velocity_.tra, params::CONFIG_kMaxRotVel / 2},
      {est_velocity_.tra, -params::CONFIG_kMaxRotVel / 2},
  }};

  for (auto& cmd : alternate_commands) {
    cmd = ApplyCommandLimits(cmd);
  }

  for (size_t i = 0; i < alternate_commands.size(); ++i) {
    costs[i] = AlternateCommandCost(
        limited_command_tr, alternate_commands[i], dynamic_features);
  }

  size_t best_idx = array_util::ArgMin(costs);

  // If every command causes a crash, we command full brakes, limited by
  // acceleration constraints.
  if (costs[best_idx] >= std::numeric_limits<float>::max()) {
    ROS_ERROR("All proposed commands colliding!");
    best_idx = 0;
  }
  return alternate_commands[best_idx];
}

float PIDController::AlternateCommandCost(
    const TrajectoryRollout& desired_tr,
    const util::Twist& alternate,
    const util::DynamicFeatures& dynamic_features) const {
  const auto collide_result = IsCommandColliding(alternate, dynamic_features);
  if (collide_result.first) {
    return std::numeric_limits<float>::max();
  }
  const TrajectoryRollout& alternate_tr = collide_result.second;
  return (desired_tr.final_pose.tra - alternate_tr.final_pose.tra)
      .squaredNorm();
}

bool PIDController::AtPoint(const Eigen::Vector2f& point) const {
  const auto est_world_pose_ = state_estimator_.GetEstimatedPose();
  const Eigen::Vector2f tra_delta = est_world_pose_.tra - point;
  return (tra_delta.squaredNorm() < Sq(params::CONFIG_goal_deadzone_tra));
}

bool PIDController::AtPose(const util::Pose& pose) const {
  const auto est_world_pose_ = state_estimator_.GetEstimatedPose();
  if (!AtPoint(pose.tra)) {
    return false;
  }
  const float waypoint_angle_delta =
      math_util::AngleDiff(est_world_pose_.rot, pose.rot);
  return (std::abs(waypoint_angle_delta) < params::CONFIG_goal_deadzone_rot);
}

util::Twist PIDController::ProposeCommand(const util::Pose& waypoint) const {
  const auto est_world_pose_ = state_estimator_.GetEstimatedPose();
  const Eigen::Vector2f tra_delta = est_world_pose_.tra - waypoint.tra;

  // Handle final turn to face waypoint's angle.
  if (tra_delta.squaredNorm() < Sq(params::CONFIG_goal_deadzone_tra)) {
    const float waypoint_angle_delta =
        math_util::AngleDiff(est_world_pose_.rot, waypoint.rot);
    if (std::abs(waypoint_angle_delta) < params::CONFIG_goal_deadzone_rot) {
      return {0, 0, 0};
    }
    // Rotate in place to final pose rotation.
    return {0, 0, -waypoint_angle_delta * params::CONFIG_rotation_p};
  }

  const float robot_angle = est_world_pose_.rot;
  const Eigen::Vector2f robot_to_waypoint_delta =
      waypoint.tra - est_world_pose_.tra;
  const float robot_to_waypoint_angle =
      std::atan2(robot_to_waypoint_delta.y(), robot_to_waypoint_delta.x());

  NP_FINITE(robot_angle);
  NP_FINITE_VEC(robot_to_waypoint_delta);
  NP_FINITE(robot_to_waypoint_angle);

  const float robot_to_waypoint_angle_delta =
      math_util::AngleDiff(robot_angle, robot_to_waypoint_angle);
  NP_FINITE_MSG(robot_to_waypoint_angle_delta,
                "Robot angle: " << robot_angle << " robot to waypoint angle: "
                                << robot_to_waypoint_angle);
  NP_CHECK_VAL(robot_to_waypoint_angle_delta >= -(kPi + kEpsilon) &&
                   robot_to_waypoint_angle_delta <= (kPi + kEpsilon),
               robot_to_waypoint_angle_delta);
  float x = 0;
  if (std::abs(robot_to_waypoint_angle_delta) <
      params::CONFIG_rotation_drive_threshold) {
    x = robot_to_waypoint_delta.norm();
  }

  util::Twist proposed_command(
      x * params::CONFIG_translation_p,
      0,
      -robot_to_waypoint_angle_delta * params::CONFIG_rotation_p);

  const TrajectoryRollout tr(state_estimator_.GetEstimatedPose(),
                             state_estimator_.GetEstimatedVelocity(),
                             proposed_command,
                             state_estimator_.GetLaserTimeDelta());

  const Eigen::Vector2f waypoint_to_stop_delta =
      tr.final_pose.tra - waypoint.tra;
  if (robot_to_waypoint_delta.dot(waypoint_to_stop_delta) > 0.0f) {
    const float dampener_distance =
        std::max(waypoint_to_stop_delta.squaredNorm() -
                     math_util::Sq(params::CONFIG_stop_past_goal_threshold),
                 0.0f);
    proposed_command.tra.x() -=
        dampener_distance * params::CONFIG_stop_past_goal_dampener;
  }

  return proposed_command;
}

util::Twist PIDController::ApplyCommandLimits(util::Twist c) const {
  const auto est_velocity_ = state_estimator_.GetEstimatedVelocity();
  return util::physics::ApplyCommandLimits(c,
                                           state_estimator_.GetLaserTimeDelta(),
                                           est_velocity_,
                                           params::CONFIG_kMaxTraVel,
                                           params::CONFIG_kMaxTraAcc,
                                           params::CONFIG_kMaxRotVel,
                                           params::CONFIG_kMaxRotAcc);
}

std::pair<bool, TrajectoryRollout> PIDController::IsCommandColliding(
    const util::Twist& commanded_velocity,
    const util::DynamicFeatures& dynamic_features) const {
  const float rollout_duration = state_estimator_.GetLaserTimeDelta();
  const float& robot_radius = params::CONFIG_robot_radius;
  const float& safety_margin = params::CONFIG_safety_margin;

  const auto est_world_pose_ = state_estimator_.GetEstimatedPose();
  const auto est_velocity_ = state_estimator_.GetEstimatedVelocity();

  static constexpr bool kDebug = false;
  const TrajectoryRollout tr(
      est_world_pose_, est_velocity_, commanded_velocity, rollout_duration);
  for (const auto& p : dynamic_features.features) {
    if (tr.IsColliding({p, p}, robot_radius + safety_margin)) {
      if (kDebug) {
        ROS_INFO("Current command: (%f, %f), %f",
                 commanded_velocity.tra.x(),
                 commanded_velocity.tra.y(),
                 commanded_velocity.rot);
        ROS_INFO("End pose: (%f, %f), %f",
                 tr.final_pose.tra.x(),
                 tr.final_pose.tra.y(),
                 tr.final_pose.rot);
        ROS_INFO("Colliding with observed point: (%f, %f)", p.x(), p.y());
      }
      return {true, tr};
    }
  }
  for (const auto& w : map_.lines) {
    if (tr.IsColliding(w, robot_radius + safety_margin)) {
      if (kDebug) {
        ROS_INFO("Current command: (%f, %f), %f",
                 commanded_velocity.tra.x(),
                 commanded_velocity.tra.y(),
                 commanded_velocity.rot);
        ROS_INFO("End pose: (%f, %f), %f",
                 tr.final_pose.tra.x(),
                 tr.final_pose.tra.y(),
                 tr.final_pose.rot);
        ROS_INFO("Colliding with observed wall: (%f, %f) <-> (%f, %f)",
                 w.p0.x(),
                 w.p0.y(),
                 w.p1.x(),
                 w.p1.y());
      }
      return {true, tr};
    }
  }
  return {false, tr};
}

}  // namespace motion_planning
}  // namespace cs
