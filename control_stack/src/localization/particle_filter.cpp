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
#include <visualization_msgs/MarkerArray.h>
#include <eigen3/Eigen/Geometry>

#include <algorithm>
#include <iomanip>
#include <limits>
#include <vector>

#include "config_reader/config_reader.h"
#include "cs/localization/particle_filter.h"
#include "cs/motion_planning/trajectory_rollout.h"
#include "cs/util/constants.h"
#include "cs/util/visualization.h"
#include "shared/math/math_util.h"
#include "shared/math/statistics.h"
#include "shared/util/timer.h"

namespace pf {
CONFIG_FLOAT(kLaserStdDev, "pf.kLaserStdDev");
CONFIG_FLOAT(kArcStdDev, "pf.kArcStdDev");
CONFIG_FLOAT(kRotateStdDev, "pf.kRotateStdDev");
CONFIG_FLOAT(kTemporalConsistencyWeight, "pf.kTemporalConsistencyWeight");
}  // namespace pf

namespace cs {
namespace localization {

MotionModel::MotionModel() : rd_(), gen_(0) {}

// util::Pose FollowTrajectory(const util::Pose& pose_global_frame,
//                            const float& delta_translation_rf,
//                            const float& delta_rotation_rf) {
//  const util::Twist command_rf(delta_translation_rf, 0, delta_rotation_rf);
//  // Command delta t is hardcoded to 1 because the delta rotation and
//  // translation have already been scaled to the single timestep.
//  cs::motion_planning::TrajectoryRollout tr(
//      pose_global_frame, command_rf, command_rf, 1);
//  return tr.achieved_vel_pose;
//}

util::Pose FollowTrajectory(const util::Pose& pose_global_frame,
                            const float& distance_along_arc,
                            const float& rotation) {
  const Eigen::Rotation2Df robot_to_global_frame(pose_global_frame.rot);
  const Eigen::Vector2f robot_forward_global_frame =
      robot_to_global_frame * Eigen::Vector2f(1, 0);

  if (rotation == 0) {
    util::Pose updated_pose = pose_global_frame;
    updated_pose.tra += robot_forward_global_frame * distance_along_arc;
    return updated_pose;
  }

  const float circle_radius = distance_along_arc / rotation;

  const float move_x_dist = std::sin(rotation) * circle_radius;
  const float move_y_dist =
      (std::cos(fabs(rotation)) * circle_radius - circle_radius);

  const Eigen::Vector2f movement_arc_robot_frame(move_x_dist, move_y_dist);
  const Eigen::Vector2f movement_arc_global_frame =
      robot_to_global_frame * movement_arc_robot_frame;

  return {movement_arc_global_frame + pose_global_frame.tra,
          math_util::AngleMod(rotation + pose_global_frame.rot)};
}

util::Pose MotionModel::ForwardPredict(const util::Pose& pose_global_frame,
                                       const float translation_robot_frame,
                                       const float rotation_robot_frame) {
  NP_CHECK(std::isfinite(translation_robot_frame));
  NP_CHECK(std::isfinite(rotation_robot_frame));
  std::normal_distribution<> distance_along_arc_dist(translation_robot_frame,
                                                     pf::CONFIG_kArcStdDev);
  std::normal_distribution<> rotation_dist(rotation_robot_frame,
                                           pf::CONFIG_kRotateStdDev);

  const float distance_along_arc = distance_along_arc_dist(gen_);
  const float rotation = rotation_dist(gen_);

  return FollowTrajectory(pose_global_frame, distance_along_arc, rotation);
}

SensorModel::SensorModel(const util::vector_map::VectorMap& map) : map_(map) {}

float GetDepthProbability(const float& sensor_reading,
                          const float& map_reading,
                          const float& ray_min,
                          const float& ray_max) {
  NP_CHECK(ray_min <= ray_max);
  NP_CHECK(sensor_reading <= ray_max);
  NP_CHECK(sensor_reading >= ray_min);

  const float people_noise = 0;
  // math_util::ProbabilityDensityExp(sensor_reading, 0.01f) * 0.0f;
  const float sensor_reading_noise = statistics::ProbabilityDensityGaussian(
      sensor_reading, map_reading, pf::CONFIG_kLaserStdDev);
  const float sensor_random_reading = 0;
  // math_util::ProbabilityDensityUniform(sensor_reading, ray_min, ray_max)
  const float sensor_random_ray_max = 0;
  // math_util::ProbabilityDensityUniform(sensor_reading, ray_max - 0.01f,
  //                                     ray_max)
  return people_noise + sensor_reading_noise + sensor_random_reading +
         sensor_random_ray_max;
}

float SensorModel::GetProbability(const util::Pose& pose_global_frame,
                                  const util::LaserScan& laser_scan) const {
  if (laser_scan.ros_laser_scan_.ranges.empty()) {
    return 0;
  }

  const float& angle_min = laser_scan.ros_laser_scan_.angle_min;
  const float& angle_max = laser_scan.ros_laser_scan_.angle_max;
  const float& range_min = laser_scan.ros_laser_scan_.range_min;
  const float& range_max = laser_scan.ros_laser_scan_.range_max;
  const int num_rays = laser_scan.ros_laser_scan_.ranges.size();

  NP_CHECK_GT(num_rays, 0);
  NP_CHECK_LE(range_min, range_max);

  std::vector<float> scan;
  scan.reserve(num_rays);
  map_.GetPredictedScan(pose_global_frame.tra,
                        range_min,
                        range_max,
                        angle_min + pose_global_frame.rot,
                        angle_max + pose_global_frame.rot,
                        num_rays,
                        &scan);
  NP_CHECK(static_cast<int>(scan.size()) == num_rays);

  float probability_sum = 0;
  int num_valid_observations = 0;
  for (int i = 0; i < num_rays; ++i) {
    float observed_depth = laser_scan.ros_laser_scan_.ranges[i];
    if (!std::isfinite(observed_depth)) {
      continue;
    }
    ++num_valid_observations;
    observed_depth = math_util::Clamp(observed_depth, range_min, range_max);
    const float& cast_depth = scan[i];
    const float depth_probability =
        GetDepthProbability(observed_depth, cast_depth, range_min, range_max);
    probability_sum += depth_probability;
  }
  if (num_valid_observations == 0) {
    return 0;
  }
  return probability_sum / num_valid_observations;
}

ParticleFilter::ParticleFilter(const util::vector_map::VectorMap& map)
    : initialized_(false), rd_(), gen_(0), sensor_model_(map) {}

ParticleFilter::ParticleFilter(const util::vector_map::VectorMap& map,
                               const util::Pose& start_pose)
    : initialized_(true), rd_(), gen_(0), sensor_model_(map) {
  InitalizePose(start_pose);
}

bool ParticleFilter::IsInitialized() const { return initialized_; }

void ParticleFilter::InitalizePose(const util::Pose& start_pose) {
  for (Particle& p : particles_) {
    p = Particle(start_pose, 1.0f);
  }
  initialized_ = true;
}

void ParticleFilter::UpdateOdom(const float& translation,
                                const float& rotation) {
  if (!initialized_) {
    ROS_WARN("Particle filter not initialized yet!");
    return;
  }

  for (Particle& p : particles_) {
    p.pose = motion_model_.ForwardPredict(p.pose, translation, rotation);
  }
}

float ScanSimilarity(const util::LaserScan& scan1,
                     const util::Pose& pose1,
                     const util::LaserScan& scan2,
                     const util::Pose& pose2) {
  const auto global_points_1 =
      scan1.TransformPointsFrameSparse(pose1.ToAffine());
  const auto global_points_2 =
      scan2.TransformPointsFrameSparse(pose2.ToAffine());

  if (global_points_1.empty() || global_points_2.empty()) {
    return 0.0f;
  }

  float total_error = 0;
  for (const Eigen::Vector2f& p1 : global_points_1) {
    float min_sq_distance = std::numeric_limits<float>::max();
    for (const Eigen::Vector2f& p2 : global_points_2) {
      const float candidate_sq = (p1 - p2).squaredNorm();
      NP_FINITE(p1.x());
      NP_FINITE(p1.y());
      NP_FINITE(p2.x());
      NP_FINITE(p2.y());
      if (!std::isfinite(candidate_sq)) {
        std::cout << std::setprecision(20) << std::fixed << p1.x() << p1.y()
                  << p2.x() << p2.y() << candidate_sq << std::endl;
      }
      NP_FINITE(candidate_sq);
      min_sq_distance = std::min(candidate_sq, min_sq_distance);
    }
    NP_FINITE(min_sq_distance);
    if (min_sq_distance < std::numeric_limits<float>::max()) {
      total_error += min_sq_distance;
    }
    NP_FINITE(total_error);
  }
  NP_FINITE(total_error);
  const float average_error =
      total_error / static_cast<float>(global_points_1.size());
  // std::cout << "Average error: " << average_error << std::endl;
  if (average_error < 0.01f) {
    return 100.0f;
  }
  return 1.0f / average_error;
}

float ParticleFilter::ScoreObservation(
    const util::Pose& pose, const util::LaserScan& laser_scan) const {
  return sensor_model_.GetProbability(pose, laser_scan);
}

void ParticleFilter::ReweightParticles(const util::LaserScan& laser_scan) {
  for (auto it = particles_.begin(); it != particles_.end(); ++it) {
    Particle& p = *it;
    NP_FINITE(p.pose.tra.x());
    NP_FINITE(p.pose.tra.y());
    NP_FINITE(p.pose.rot);
    p.weight = ScoreObservation(p.pose, laser_scan);
  }
}

void ParticleFilter::ResampleParticles() {
  const float total_weights = [this]() -> float {
    float sum = 0;
    for (const auto& p : particles_) {
      sum += p.weight;
    }
    return sum;
  }();

  static_assert(kNumParticles > 0, "Must have at least one particle.");
  const float low_variance_step_size = total_weights / kNumParticles;
  std::uniform_real_distribution<> distribution(0.0f, low_variance_step_size);
  const float low_variance_start = distribution(gen_);

  auto resampled_particles = particles_;
  float current_weight = low_variance_start;
  size_t old_particle_index = 0;
  for (Particle& new_p : resampled_particles) {
    for (size_t i = old_particle_index; i < particles_.size(); ++i) {
      NP_CHECK(i < particles_.size());
      const Particle& old_p = particles_[i];
      if (current_weight <= old_p.weight) {
        new_p = old_p;
        old_particle_index = i;
        current_weight += low_variance_step_size;
        break;
      }
      current_weight -= old_p.weight;
    }
  }

  particles_ = resampled_particles;
}

void ParticleFilter::UpdateObservation(const util::LaserScan& laser_scan) {
  return UpdateObservation(laser_scan, nullptr);
}

void ParticleFilter::UpdateObservation(const util::LaserScan& laser_scan,
                                       ros::Publisher* sampled_scan_pub) {
  if (!initialized_) {
    ROS_WARN("Particle filter not initialized yet!");
    return;
  }

  ReweightParticles(laser_scan);

  if (sampled_scan_pub != nullptr) {
    sampled_scan_pub->publish(laser_scan.ros_laser_scan_);
  }

  ResampleParticles();
}

util::Pose ParticleFilter::MaxWeight() const {
  Particle const* max_particle = &particles_[0];
  for (Particle const& p : particles_) {
    if (max_particle->weight < p.weight) {
      max_particle = &p;
    }
  }
  return max_particle->pose;
}

util::Pose ParticleFilter::WeightedCentroid() const {
  NP_CHECK_GT(particles_.size(), 0);
  float total_weight = 0;
  for (const Particle& p : particles_) {
    total_weight += p.weight;
  }

  util::Pose weighted_centroid({0, 0}, 0);
  if (total_weight == 0.0f) {
    ROS_WARN(
        "Total particle weight 0; observations have diverged from estimates.");
    return weighted_centroid;
  }

  float sum_of_sins = 0.0f;
  float sum_of_coss = 0.0f;
  for (const Particle& p : particles_) {
    const float scale = p.weight / total_weight;
    weighted_centroid.tra += (p.pose.tra * scale);
    sum_of_sins += std::sin(p.pose.rot) * scale;
    sum_of_coss += std::cos(p.pose.rot) * scale;
  }

  weighted_centroid.rot = math_util::AngleMod(atan2(sum_of_sins, sum_of_coss));
  return weighted_centroid;
}

void ParticleFilter::DrawParticles(ros::Publisher* particle_pub) const {
  if (!initialized_) {
    ROS_WARN("Particle filter not initialized yet!");
    return;
  }
  static visualization_msgs::MarkerArray particle_markers;
  for (visualization_msgs::Marker& marker : particle_markers.markers) {
    marker.action = marker.DELETE;
  }
  particle_pub->publish(particle_markers);
  particle_markers.markers.clear();
  float max_weight = 0;
  for (const Particle& p : particles_) {
    max_weight = std::max(p.weight, max_weight);
  }

  for (const Particle& p : particles_) {
    const float alpha =
        ((max_weight > 0.0f) ? (p.weight / max_weight) : 0.0f) / 2;
    visualization::DrawPose(
        p.pose, "map", "particles", 1, 0, 0, alpha, &particle_markers);
  }

  const util::Pose estimate = WeightedCentroid();

  visualization::DrawPose(
      estimate, "map", "estimate", 0, 0, 1, 1, &particle_markers);

  particle_pub->publish(particle_markers);
}
}  // namespace localization
}  // namespace cs
