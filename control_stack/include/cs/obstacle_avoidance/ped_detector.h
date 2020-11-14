#pragma once
// ========================================================================
#include <ros/ros.h>
#include <array>
#include <random>

#include "cs/util/laser_scan.h"
#include "cs/util/map.h"
#include "cs/util/pose.h"
#include "cs/util/twist.h"

namespace cs {
namespace ped_detection {

class PedDetector {
 public:
  PedDetector() = default;
  virtual ~PedDetector() = default;

  virtual void UpdatePeds(const util::LaserScan& laser,
                           const ros::Time& time) = 0;  // currently takes in laser but needs to take in Obstacle


  virtual util::Pose GetEstimatedPeds() const = 0; // currently returns Pose but needs to return ped data

  virtual float GetPedsTimeDelta() const = 0;
};

}  // namespace state_estimation
}  // namespace cs
