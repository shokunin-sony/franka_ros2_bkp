// Copyright (c) 2021 Franka Emika GmbH
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <franka_example_controllers/speed_generator.hpp>

#include <algorithm>
#include <array>
#include <cassert>
#include <cmath>
#include <utility>

#include <Eigen/Core>

/**
 * @brief Construct a joint velocity controlled motion generator
 *
 * @param speed_factor - The ratio of the max acceleration and velocities for joints
 * @param q_start - The start position of the robot
 * @param q_vel - Desired velocities for joints
 * @param duration_vel - the total duration of the velocity based motion
 */
SpeedGenerator::SpeedGenerator(double speed_factor,
                               const Vector7d& q_start,
                               const Vector7d& q_vel,
                               double duration_vel)
    : q_start_(q_start) {
  assert(speed_factor > 0);
  assert(speed_factor <= 1);
  dq_max_ *= speed_factor;
  duration_vel_ = duration_vel;
  q_vel_ = q_vel;
  // assert(abs(q_vel.maxCoeff()) <= dq_max_);
  calculateSynchronizedValues();
}

bool SpeedGenerator::calculateDesiredValues(double t, Vector7d* delta_q_d) const {
  Vector7i sign_q_vel;
  sign_q_vel << q_vel_.cwiseSign().cast<int>();
  Vector7d t_d = t_2_ - t_1_;
  Vector7d delta_t_2 = t_f_ - t_2_;
  std::array<bool, kJoints> joint_motion_finished{};

  for (auto i = 0; i < kJoints; i++) {
    if (t < t_1_[i]) {
      // double omega[i] = q_vel_[i] / 2.0 * (1.0 - std::cos(2.0 * M_PI * t));
      (*delta_q_d)[i] = q_vel_[i] / 2.0 * t - q_vel_[i] / 4.0 / M_PI * std::sin(2.0 * M_PI * t);
    } else if (t >= t_1_[i] && t < t_2_[i]) {
      (*delta_q_d)[i] =
          q_vel_[i] / 2.0 * (1.0 - std::cos(2.0 * M_PI * t_1_[i])) + (t - t_1_[i]) * q_vel_[i];
    } else if (t >= t_2_[i] && t < t_f_[i]) {
      (*delta_q_d)[i] = (t_2_[i] - t_1_[i]) * q_vel_[i] +
                        q_vel_[i] / 2.0 * (1.0 - std::cos(2.0 * M_PI * (t - t_d[i])));
    } else {
      (*delta_q_d)[i] = (t_2_[i] - t_1_[i]) * q_vel_[i] +
                        q_vel_[i] / 2.0 * (1.0 - std::cos(2.0 * M_PI * (t_1_[i] + delta_t_2[i])));
      joint_motion_finished.at(i) = true;
    }
  }
  return std::all_of(joint_motion_finished.cbegin(), joint_motion_finished.cend(),
                     [](bool x) { return x; });
}

void SpeedGenerator::calculateSynchronizedValues() {
  Vector7d delta_t_2 = Vector7d::Zero();

  for (auto i = 0; i < kJoints; i++) {
    t_1_[i] = 0.5;
    delta_t_2[i] = 0.5;
    t_f_[i] = duration_vel_;
    t_2_[i] = t_f_[i] - delta_t_2[i];
  }
}

std::pair<SpeedGenerator::Vector7d, bool> SpeedGenerator::getDesiredJointPositions(
    const rclcpp::Duration& trajectory_time) {
  time_ = trajectory_time.seconds();

  Vector7d delta_q_d;
  bool motion_finished = calculateDesiredValues(time_, &delta_q_d);

  std::array<double, kJoints> joint_positions{};
  Eigen::VectorXd::Map(&joint_positions[0], kJoints) = (q_start_ + delta_q_d);
  return std::make_pair(q_start_ + delta_q_d, motion_finished);
}