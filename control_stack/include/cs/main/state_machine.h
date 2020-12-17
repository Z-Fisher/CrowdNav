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

#include <memory>
#include <string>
#include <vector>

#include "cs/main/debug_pub_wrapper.h"
#include "cs/util/constants.h"
#include "cs/util/laser_scan.h"

#include "config_reader/config_reader.h"
#include "cs/controllers/controller.h"
#include "cs/controllers/escape_collision_controller.h"
#include "cs/controllers/nav_controller.h"
#include "cs/localization/particle_filter.h"
#include "cs/motion_planning/command_scaler.h"
#include "cs/motion_planning/identity_command_scaler.h"
#include "cs/motion_planning/pid.h"
#include "cs/motion_planning/turtlebot_command_scaler.h"
#include "cs/obstacle_avoidance/obstacle_detector.h"
#include "cs/obstacle_avoidance/ped_detector.h"
#include "cs/path_finding/astar.h"
#include "cs/path_finding/global_path_finder.h"
#include "cs/state_estimation/pf_state_estimator.h"
#include "cs/state_estimation/sim_state_estimator.h"
#include "cs/state_estimation/state_estimator.h"
#include "cs/util/physics.h"
#include "cs/util/pose.h"
#include "cs/util/twist.h"
#include "cs/util/util.h"
#include "cs/util/ped_vector.h"
#include "cs/util/visualization.h"
#include "shared/math/geometry.h"
#include "shared/math/math_util.h"
#include "shared/util/timer.h"
#include <control_stack/Obstacles.h>

namespace cs {
namespace main {

namespace params {
CONFIG_STRING(map, "pf.map");
CONFIG_INTLIST(deadzones, "laser.deadzones");
CONFIG_VECTOR2F(laser_offset, "laser.center_offset");

CONFIG_BOOL(use_sim_ground_truth, "state_estimation.use_sim_ground_truth");
CONFIG_STRING(command_scaler, "cmd_scaler.command_scaler");
CONFIG_VECTOR3F(start_pose, "pf.start_pose");

CONFIG_STRING(map_tf_frame, "frames.map_tf_frame");
CONFIG_STRING(base_link_tf_frame, "frames.base_tf_frame");
CONFIG_STRING(laser_tf_frame, "frames.laser_tf_frame");

CONFIG_FLOAT(robot_radius, "pf.kRobotRadius");
CONFIG_FLOAT(safety_margin, "pf.kSafetyMargin");

}  // namespace params


struct ControllerList {
  using ControllerPtr = std::unique_ptr<cs::controllers::Controller>;
  std::array<ControllerPtr, cs::controllers::ControllerType::CONTROLLER_NUM>
      controller_array;

  // std::array<ControllerPtr, 1>
  //     controller_array;

  cs::controllers::ControllerType current_controller_;

  ControllerList(cs::main::DebugPubWrapper* dpw,
                 const util::LaserScan& laser,
                 const util::vector_map::VectorMap& map,
                 const state_estimation::StateEstimator& state_estimator,
                 const obstacle_avoidance::ObstacleDetector& obstacle_detector,
                 const ped_detection::PedDetector& ped_detector,
                 const motion_planning::PIDController& motion_planner,
                 cs::controllers::ControllerType current_controller)
      : current_controller_(current_controller) {
    controller_array[cs::controllers::ControllerType::NAVIGATION] =
        ControllerPtr(new cs::controllers::NavController(dpw,
                                                         laser,
                                                         map,
                                                         state_estimator,
                                                         obstacle_detector,
                                                         ped_detector,
                                                         motion_planner));
    controller_array[cs::controllers::ControllerType::ESCAPE_COLLISION] =
        ControllerPtr(
            new cs::controllers::EscapeCollisionController(dpw,
                                                           laser,
                                                           map,
                                                           state_estimator,
                                                           obstacle_detector,
                                                           ped_detector,
                                                           motion_planner));

    for (const auto& p : controller_array) {
      NP_CHECK(p != nullptr);
    }
  }

  void Reset() {
    NP_NOT_NULL(controller_array[current_controller_]);
    return controller_array[current_controller_]->Reset();
  }

  util::Twist Execute() {
    static constexpr bool kDebug = true;
    for (int num_transitions = 0; num_transitions < 8; ++num_transitions) {
      NP_NOT_NULL(controller_array[current_controller_]);
      if (kDebug) {
        ROS_INFO("Controller: %s",
                 cs::controllers::to_string(current_controller_).c_str());
      }
      const auto execute_result =
          controller_array[current_controller_]->Execute();
      ROS_INFO("Robot speed: %f, %f", execute_result.second.tra.x(), execute_result.second.tra.y());

      if (execute_result.first == current_controller_) {
        return execute_result.second;
      }
      Reset();
      current_controller_ = execute_result.first;
    }
    ROS_ERROR("Controller exceeded max transitions!");
    return {};
  }
};

class StateMachine {
 private:
  util::LaserScan laser_;
  util::PedVector ped_vector_;
  ros::Time laser_update_time_;
  ros::Time ped_update_time_;
  util::Twist odom_;
  ros::Time odom_update_time_;
  cs::main::DebugPubWrapper* dpw_;
  util::vector_map::VectorMap map_;
  std::unique_ptr<state_estimation::StateEstimator> state_estimator_;
  std::unique_ptr<ped_detection::PedDetector> ped_detector_;
  obstacle_avoidance::ObstacleDetector obstacle_detector_;
  motion_planning::PIDController motion_planner_;
  std::unique_ptr<motion_planning::CommandScaler> command_scaler_;
  tf::TransformBroadcaster br_;
  ControllerList controller_list_;
  std::string pub_sub_prefix_;

  // ===========================================================================

  cs::state_estimation::StateEstimator* MakeStateEstimator(ros::NodeHandle* n) {
    if (params::CONFIG_use_sim_ground_truth) {
      ROS_INFO("Using sim ground truth for state estimation");
      return new cs::state_estimation::SimStateEstimator(n);
    }

    ROS_INFO("Using PF for state estimation initialized at (%f, %f), %f",
             params::CONFIG_start_pose.x(),
             params::CONFIG_start_pose.y(),
             params::CONFIG_start_pose.z());
    return new cs::state_estimation::PFStateEstimator(
        map_, util::Pose(params::CONFIG_start_pose));
  }

    
  cs::ped_detection::PedDetector* MakePedDetector() {
      return new cs::ped_detection::PedDetector();
    }
  cs::motion_planning::CommandScaler* MakeCommandScaler() {
    if (params::CONFIG_command_scaler == "turtlebot") {
      return new cs::motion_planning::TurtlebotCommandScaler();
    }
    return new cs::motion_planning::IdentityCommandScaler();
  }

  void DrawRobot(const util::vector_map::VectorMap& full_map,
                 util::Twist command) {
    static constexpr bool kShowRotateCenter = false;
    // clang-format off
    dpw_->robot_size_pub_.publish(visualization::MakeCylinder(
        state_estimator_->GetEstimatedPose().tra,
        params::CONFIG_robot_radius, 0.1,
        params::CONFIG_map_tf_frame,
    "robot_size", 0, 1, 0, 1, 0.05));
    const Eigen::Vector2f front_offset =
        Eigen::Rotation2Df(state_estimator_->GetEstimatedPose().rot) *
        Eigen::Vector2f(
        params::CONFIG_robot_radius + params::CONFIG_safety_margin, 0);
    dpw_->robot_size_pub_.publish(visualization::MakeCylinder(
        state_estimator_->GetEstimatedPose().tra + front_offset, 0.05,
        0.1, params::CONFIG_map_tf_frame,
        "forward_bump", 1, 0, 0, 1,
        0.05));
    dpw_->robot_size_pub_.publish(visualization::MakeCylinder(
        state_estimator_->GetEstimatedPose().tra,
        params::CONFIG_robot_radius + params::CONFIG_safety_margin,
        0.1, params::CONFIG_map_tf_frame,
        "safety_size", 0, 0, 1, 0.1, 0.05));

    const auto cd = util::physics::ComputeCommandDelta(
        state_estimator_->GetEstimatedPose(),
        state_estimator_->GetEstimatedVelocity(), command,
        state_estimator_->GetLaserTimeDelta());

    if (kShowRotateCenter &&
            cd.type == util::physics::CommandDelta::Type::CURVE) {
      dpw_->robot_size_pub_.publish(visualization::MakeCylinder(
          cd.curve.rotate_circle_center_wf, 0.1, 0.1,
          params::CONFIG_map_tf_frame, "rotatecenter", 1, 0, 0, 1));
    }

    const motion_planning::TrajectoryRollout tr(
        state_estimator_->GetEstimatedPose(),
        state_estimator_->GetEstimatedVelocity(), command,
        state_estimator_->GetLaserTimeDelta());

    dpw_->robot_size_pub_.publish(visualization::MakeCylinder(
        tr.final_pose.tra,
        params::CONFIG_robot_radius + params::CONFIG_safety_margin, 0.1,
        params::CONFIG_map_tf_frame, "final_safety", 1, 0, 0, 0.1, 0.05));

    std::vector<util::Wall> colliding_walls;
    for (const auto& w : full_map.lines) {
      if (tr.IsColliding(
              w, params::CONFIG_robot_radius + params::CONFIG_safety_margin)) {
        colliding_walls.push_back(w);
      }
    }
    dpw_->robot_size_pub_.publish(visualization::DrawWalls(
        colliding_walls, params::CONFIG_map_tf_frame, "colliding_walls", 0.3));

    dpw_->robot_size_pub_.publish(visualization::MakeCylinder(
        tr.final_pose.tra,
        params::CONFIG_robot_radius + params::CONFIG_safety_margin, 0.1,
        params::CONFIG_map_tf_frame, "final_safety", 1, 0, 0, 0.1, 0.05));
    // clang-format on
  }

  void PublishTransforms() {
    tf::Transform laser_transform = tf::Transform::getIdentity();
    laser_transform.setOrigin({-params::CONFIG_laser_offset.x(),
                               -params::CONFIG_laser_offset.y(),
                               0});
    br_.sendTransform(tf::StampedTransform(
        laser_transform,
        ros::Time::now(),
        pub_sub_prefix_ + params::CONFIG_laser_tf_frame,
        pub_sub_prefix_ + params::CONFIG_base_link_tf_frame));

    const auto est_pose = state_estimator_->GetEstimatedPose();
    NP_FINITE_VEC(est_pose.tra);
    NP_FINITE(est_pose.rot);
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(est_pose.tra.x(), est_pose.tra.y(), 0.0));
    tf::Quaternion q;
    q.setRPY(0, 0, est_pose.rot);
    transform.setRotation(q);
    br_.sendTransform(tf::StampedTransform(
        transform.inverse(),
        ros::Time::now(),
        pub_sub_prefix_ + params::CONFIG_base_link_tf_frame,
        params::CONFIG_map_tf_frame));
  }

 public:
  StateMachine() = delete;
  StateMachine(cs::main::DebugPubWrapper* dpw,
               ros::NodeHandle* n,
               const std::string& pub_sub_prefix)
      : dpw_(dpw),
        map_(params::CONFIG_map),
        state_estimator_(MakeStateEstimator(n)),
        ped_detector_(MakePedDetector()),
        obstacle_detector_(),
        motion_planner_(map_, *state_estimator_),
        command_scaler_(MakeCommandScaler()),
        controller_list_(dpw,
                         laser_,
                         map_,
                         *state_estimator_,
                         obstacle_detector_,
                         *ped_detector_,
                         motion_planner_,
                         cs::controllers::ControllerType::NAVIGATION),
        pub_sub_prefix_(pub_sub_prefix) {}


  Eigen::Affine2f GetLaserOffset() {
    Eigen::Affine2f a = Eigen::Affine2f::Identity();
    a.translate(params::CONFIG_laser_offset);
    return a;
  }

  void RemoveDeadzones(util::LaserScan* laser) {
    NP_CHECK_EQ(params::CONFIG_deadzones.size() % 2, 0);
    for (size_t i = 0; i < params::CONFIG_deadzones.size(); i += 2) {
      laser->ClearDataInIndexRange(params::CONFIG_deadzones[i],
                                   params::CONFIG_deadzones[i + 1]);
    }
  }

  void UpdateLaser(const sensor_msgs::LaserScan& msg) {
    util::LaserScan laser(msg, GetLaserOffset());
    RemoveDeadzones(&laser);
    dpw_->modified_laser_pub_.publish(laser.ros_laser_scan_);
    laser_ = laser;
    laser_update_time_ = msg.header.stamp;
    state_estimator_->UpdateLaser(laser_, laser_update_time_);
  }

  void UpdatePeds(const control_stack::Obstacles::ConstPtr& msg) {
    ped_update_time_ = msg->header.stamp;
    ped_vector_ = util::PedVector(msg);
    ped_detector_->UpdatePeds(ped_vector_, ped_update_time_);
  }

  void UpdateOdom(const nav_msgs::Odometry& msg) {
    odom_ = util::Twist(msg.twist.twist);
    odom_update_time_ = msg.header.stamp;
    state_estimator_->UpdateOdom(odom_, odom_update_time_);
  }
  
  util::Twist ExecuteController() {
    const auto est_pose = state_estimator_->GetEstimatedPose();
    dpw_->position_pub_.publish(est_pose.ToTwist());
    obstacle_detector_.UpdateObservation(
        est_pose, laser_, &(dpw_->detected_walls_pub_));
    const util::Twist command = controller_list_.Execute();
    // ROS_ERROR("Actual velocity: vx = %f, vy = %f", command.tra.x(), command.tra.y());
    state_estimator_->UpdateLastCommand(command);
    PublishTransforms();
    state_estimator_->Visualize(&(dpw_->particle_pub_));
    dpw_->map_pub_.publish(
        visualization::DrawWalls(map_.lines, "map", "map_ns"));
    DrawRobot(map_, command);
    
    // TODO: causes robot to stay still - delete this later
    // (void) command;
    // const util::Twist bad_command(0, 0, 0);

    // return command_scaler_->ScaleCommand(bad_command);

    return command_scaler_->ScaleCommand(command);
  }
};

}  // namespace main
}  // namespace cs
