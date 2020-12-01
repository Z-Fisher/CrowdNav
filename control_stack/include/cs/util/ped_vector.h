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

#include <sensor_msgs/LaserScan.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <limits>
#include <vector>

#include "cs/util/constants.h"
#include "shared/math/math_util.h"

#include <control_stack/Obstacles.h>

#include "pose.h"
#include "twist.h"

#include "geometry_msgs/Point.h"
#include "geometry_msgs/Vector3.h"

#include <ros/ros.h>

namespace util {
    struct Pedestrian {
        Pose pose;
        Twist vel;
        float radius;
        float true_radius;
        Pedestrian(const geometry_msgs::Point& point, const geometry_msgs::Vector3& vel,
            const float rad, const float true_rad) : pose(point.x, point.y, 0), vel(vel.x, vel.y, 0) {
                radius = rad;
                true_radius = true_rad;
            }
    };


class PedVector {
 public:
    std::vector<Pedestrian> peds;
    // control_stack::Obstacles ros_msg;

    PedVector(const control_stack::Obstacles::ConstPtr& msg) {
        for (auto circle: msg->circles) {
            peds.push_back(Pedestrian(circle.center, circle.velocity, circle.radius, circle.true_radius));
        }

        //  ROS_INFO("x_pos: [%f]", peds[0].pose.tra[0]);
    }

    PedVector() {}
  
};
}  // namespace util
