#pragma once
// ========================================================================
#include <ros/ros.h>
#include <array>
#include <random>

#include "cs/util/laser_scan.h"
#include "cs/util/map.h"
#include "cs/util/pose.h"
#include "cs/util/twist.h"
#include "cs/util/datastructures/circular_buffer.h"

namespace cs {
namespace ped_detection {

class PedDetector {
 private:
  float obstacle_dummy_ = 31;
  static constexpr size_t kTimeBufferSize = 5;
  cs::datastructures::CircularBuffer<ros::Time, kTimeBufferSize> laser_times_;

  float GetTimeDelta(
      const cs::datastructures::CircularBuffer<ros::Time, kTimeBufferSize>& b)
      const {
    if (b.size() <= 1) {
      return kEpsilon;
    }
    const double total_time_delta = (b.back() - b.front()).toSec();
    const double iterations = static_cast<double>(b.size() - 1);
    return static_cast<float>(total_time_delta / iterations);
  }

 public:
  // PedDetector() = delete;
  // PedDetector() {}
  // ~PedDetector() = default;

  void UpdatePeds(const util::LaserScan& laser, const ros::Time& time) {
    NP_CHECK(laser.ros_laser_scan_.header.stamp == time);
    laser_times_.push_back(time);
  }

  float GetPeds() const {
    return obstacle_dummy_;
  }

  float GetTimeDelta() const { return GetTimeDelta(laser_times_); }
};

}  // namespace ped_detector
}  // namespace cs

