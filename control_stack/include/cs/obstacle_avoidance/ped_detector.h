#pragma once
// ========================================================================
#include <ros/ros.h>
#include <array>
#include <random>
#include <control_stack/Obstacles.h>

#include "cs/util/laser_scan.h"
#include "cs/util/map.h"
#include "cs/util/pose.h"
#include "cs/util/twist.h"
#include "cs/util/ped_vector.h"
#include "cs/util/datastructures/circular_buffer.h"

namespace cs {
namespace ped_detection {

class PedDetector {
 private:
  util::PedVector obstacles_;
  static constexpr size_t kTimeBufferSize = 5;
  cs::datastructures::CircularBuffer<ros::Time, kTimeBufferSize> ped_times_;

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
  PedDetector() : obstacles_() { }
  // ~PedDetector() = default;

  void UpdatePeds(const util::PedVector ped_vect, const ros::Time& time) {
    // NP_CHECK(ped_vect.ros_msg.header.stamp == time);
    ped_times_.push_back(time);
    obstacles_ = ped_vect;

  }

  util::PedVector GetPeds() const {
    return obstacles_;
  }

  float GetTimeDelta() const { return GetTimeDelta(ped_times_); }
};

}  // namespace ped_detector
}  // namespace cs

