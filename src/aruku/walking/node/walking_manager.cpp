// Copyright (c) 2021 Ichiro ITS
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
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#include "aruku/walking/node/walking_manager.hpp"

#include <fstream>
#include <string>
#include <vector>

#include "aruku/walking/process/kinematic.hpp"
#include "jitsuyo/config.hpp"
#include "keisan/angle/angle.hpp"
#include "nlohmann/json.hpp"
#include "tachimawari/joint/model/joint.hpp"
#include "tachimawari/joint/model/joint_id.hpp"

using keisan::literals::operator""_deg;

namespace aruku
{

WalkingManager::WalkingManager()
: kinematic(),
  orientation(0_deg),
  inital_joints({0.0}),
  joints_direction({1}),
  position(0.0, 0.0),
  gyro(keisan::Vector<3>::zero())
{
  using tachimawari::joint::Joint;
  using tachimawari::joint::JointId;

  for (auto id : JointId::list) {
    if (id != JointId::NECK_YAW && id != JointId::NECK_PITCH) {
      joints.push_back(Joint(id, 0.0));
    } else {
      break;
    }
  }
}

void WalkingManager::set_config(
  const nlohmann::json & walking_data, const nlohmann::json & kinematic_data)
{
  bool valid_config = true;

  nlohmann::json balance_section;
  if (jitsuyo::assign_val(walking_data, "balance", balance_section)) {
    bool valid_section = true;
    valid_section &= jitsuyo::assign_val(balance_section, "enable", balance_enable);
    valid_section &= jitsuyo::assign_val(balance_section, "balance_knee_gain", balance_knee_gain);
    valid_section &=
      jitsuyo::assign_val(balance_section, "balance_ankle_pitch_gain", balance_ankle_pitch_gain);
    valid_section &=
      jitsuyo::assign_val(balance_section, "balance_hip_roll_gain", balance_hip_roll_gain);
    valid_section &=
      jitsuyo::assign_val(balance_section, "balance_ankle_roll_gain", balance_ankle_roll_gain);
    if (!valid_section) {
      std::cout << "Error found at section `balance`" << std::endl;
      valid_config = false;
    }
  } else {
    valid_config = false;
  }

  nlohmann::json pid_section;
  if (jitsuyo::assign_val(walking_data, "pid", pid_section)) {
    bool valid_section = true;
    valid_section &= jitsuyo::assign_val(pid_section, "p_gain", p_gain);
    valid_section &= jitsuyo::assign_val(pid_section, "i_gain", i_gain);
    valid_section &= jitsuyo::assign_val(pid_section, "d_gain", d_gain);
    if (!valid_section) {
      std::cout << "Error found at section `pid`" << std::endl;
      valid_config = false;
    }
  } else {
    valid_config = false;
  }

  nlohmann::json odometry_section;
  if (jitsuyo::assign_val(walking_data, "odometry", odometry_section)) {
    bool valid_section = true;
    valid_section &=
      jitsuyo::assign_val(odometry_section, "fx_coefficient", odometry_fx_coefficient);
    valid_section &=
      jitsuyo::assign_val(odometry_section, "ly_coefficient", odometry_ly_coefficient);
    valid_section &=
      jitsuyo::assign_val(odometry_section, "ry_coefficient", odometry_ry_coefficient);
    valid_section &=
      jitsuyo::assign_val(odometry_section, "bx_coefficient", odometry_bx_coefficient);
    if (!valid_section) {
      std::cout << "Error found at section `odometry`" << std::endl;
      valid_config = false;
    }
  } else {
    valid_config = false;
  }

  nlohmann::json init_angles_section;
  if (jitsuyo::assign_val(walking_data, "init_angles", init_angles_section)) {
    bool valid_section = true;

    using tachimawari::joint::JointId;

    valid_section &= jitsuyo::assign_val(
      init_angles_section, "right_hip_yaw", inital_joints[JointId::RIGHT_HIP_YAW]);
    valid_section &= jitsuyo::assign_val(
      init_angles_section, "right_hip_pitch", inital_joints[JointId::RIGHT_HIP_PITCH]);
    valid_section &= jitsuyo::assign_val(
      init_angles_section, "right_hip_roll", inital_joints[JointId::RIGHT_HIP_ROLL]);
    valid_section &=
      jitsuyo::assign_val(init_angles_section, "right_knee", inital_joints[JointId::RIGHT_KNEE]);
    valid_section &= jitsuyo::assign_val(
      init_angles_section, "right_ankle_pitch", inital_joints[JointId::RIGHT_ANKLE_PITCH]);
    valid_section &= jitsuyo::assign_val(
      init_angles_section, "right_ankle_roll", inital_joints[JointId::RIGHT_ANKLE_ROLL]);
    valid_section &= jitsuyo::assign_val(
      init_angles_section, "left_hip_yaw", inital_joints[JointId::LEFT_HIP_YAW]);
    valid_section &= jitsuyo::assign_val(
      init_angles_section, "left_hip_pitch", inital_joints[JointId::LEFT_HIP_PITCH]);
    valid_section &= jitsuyo::assign_val(
      init_angles_section, "left_hip_roll", inital_joints[JointId::LEFT_HIP_ROLL]);
    valid_section &=
      jitsuyo::assign_val(init_angles_section, "left_knee", inital_joints[JointId::LEFT_KNEE]);
    valid_section &= jitsuyo::assign_val(
      init_angles_section, "left_ankle_pitch", inital_joints[JointId::LEFT_ANKLE_PITCH]);
    valid_section &= jitsuyo::assign_val(
      init_angles_section, "left_ankle_roll", inital_joints[JointId::LEFT_ANKLE_ROLL]);
    valid_section &= jitsuyo::assign_val(
      init_angles_section, "right_shoulder_pitch", inital_joints[JointId::RIGHT_SHOULDER_PITCH]);
    valid_section &= jitsuyo::assign_val(
      init_angles_section, "right_shoulder_roll", inital_joints[JointId::RIGHT_SHOULDER_ROLL]);
    valid_section &= jitsuyo::assign_val(
      init_angles_section, "right_shoulder_yaw", inital_joints[JointId::RIGHT_SHOULDER_YAW]);
    valid_section &=
      jitsuyo::assign_val(init_angles_section, "right_elbow", inital_joints[JointId::RIGHT_ELBOW]);
    valid_section &= jitsuyo::assign_val(
      init_angles_section, "right_gripper", inital_joints[JointId::RIGHT_GRIPPER]);
    valid_section &= jitsuyo::assign_val(
      init_angles_section, "left_shoulder_pitch", inital_joints[JointId::LEFT_SHOULDER_PITCH]);
    valid_section &= jitsuyo::assign_val(
      init_angles_section, "left_shoulder_roll", inital_joints[JointId::LEFT_SHOULDER_ROLL]);
    valid_section &= jitsuyo::assign_val(
      init_angles_section, "left_shoulder_yaw", inital_joints[JointId::LEFT_SHOULDER_YAW]);
    valid_section &=
      jitsuyo::assign_val(init_angles_section, "left_elbow", inital_joints[JointId::LEFT_ELBOW]);
    valid_section &= jitsuyo::assign_val(
      init_angles_section, "left_gripper", inital_joints[JointId::LEFT_GRIPPER]);

    if (!valid_section) {
      std::cout << "Error found at section `init_angles`" << std::endl;
      valid_config = false;
    }
  } else {
    valid_config = false;
  }

  nlohmann::json angles_direction_section;
  if (jitsuyo::assign_val(walking_data, "angles_direction", angles_direction_section)) {
    bool valid_section = true;

    using tachimawari::joint::JointId;

    valid_section &= jitsuyo::assign_val(
      angles_direction_section, "right_hip_yaw", joints_direction[JointId::RIGHT_HIP_YAW]);
    valid_section &= jitsuyo::assign_val(
      angles_direction_section, "right_hip_pitch", joints_direction[JointId::RIGHT_HIP_PITCH]);
    valid_section &= jitsuyo::assign_val(
      angles_direction_section, "right_hip_roll", joints_direction[JointId::RIGHT_HIP_ROLL]);
    valid_section &= jitsuyo::assign_val(
      angles_direction_section, "right_knee", joints_direction[JointId::RIGHT_KNEE]);
    valid_section &= jitsuyo::assign_val(
      angles_direction_section, "right_ankle_pitch", joints_direction[JointId::RIGHT_ANKLE_PITCH]);
    valid_section &= jitsuyo::assign_val(
      angles_direction_section, "right_ankle_roll", joints_direction[JointId::RIGHT_ANKLE_ROLL]);
    valid_section &= jitsuyo::assign_val(
      angles_direction_section, "left_hip_yaw", joints_direction[JointId::LEFT_HIP_YAW]);
    valid_section &= jitsuyo::assign_val(
      angles_direction_section, "left_hip_pitch", joints_direction[JointId::LEFT_HIP_PITCH]);
    valid_section &= jitsuyo::assign_val(
      angles_direction_section, "left_hip_roll", joints_direction[JointId::LEFT_HIP_ROLL]);
    valid_section &= jitsuyo::assign_val(
      angles_direction_section, "left_knee", joints_direction[JointId::LEFT_KNEE]);
    valid_section &= jitsuyo::assign_val(
      angles_direction_section, "left_ankle_pitch", joints_direction[JointId::LEFT_ANKLE_PITCH]);
    valid_section &= jitsuyo::assign_val(
      angles_direction_section, "left_ankle_roll", joints_direction[JointId::LEFT_ANKLE_ROLL]);
    valid_section &= jitsuyo::assign_val(
      angles_direction_section, "right_shoulder_pitch",
      joints_direction[JointId::RIGHT_SHOULDER_PITCH]);
    valid_section &= jitsuyo::assign_val(
      angles_direction_section, "right_shoulder_roll",
      joints_direction[JointId::RIGHT_SHOULDER_ROLL]);
    valid_section &= jitsuyo::assign_val(
      angles_direction_section, "right_elbow", joints_direction[JointId::RIGHT_ELBOW]);
    valid_section &= jitsuyo::assign_val(
      angles_direction_section, "left_shoulder_pitch",
      joints_direction[JointId::LEFT_SHOULDER_PITCH]);
    valid_section &= jitsuyo::assign_val(
      angles_direction_section, "left_shoulder_roll",
      joints_direction[JointId::LEFT_SHOULDER_ROLL]);
    valid_section &= jitsuyo::assign_val(
      angles_direction_section, "left_elbow", joints_direction[JointId::LEFT_ELBOW]);
    valid_section &= jitsuyo::assign_val(
      angles_direction_section, "right_gripper", joints_direction[JointId::RIGHT_GRIPPER]);
    valid_section &= jitsuyo::assign_val(
      angles_direction_section, "left_gripper", joints_direction[JointId::LEFT_GRIPPER]);
    valid_section &= jitsuyo::assign_val(
      angles_direction_section, "right_shoulder_yaw",
      joints_direction[JointId::RIGHT_SHOULDER_YAW]);
    valid_section &= jitsuyo::assign_val(
      angles_direction_section, "left_shoulder_yaw", joints_direction[JointId::LEFT_SHOULDER_YAW]);

    if (!valid_section) {
      std::cout << "Error found at section `angles_direction`" << std::endl;
      valid_config = false;
    }
  } else {
    valid_config = false;
  }

  if (!valid_config) {
    throw std::runtime_error("Failed to load config file `walking.json`");
  }

  kinematic.set_config(kinematic_data);
}

void WalkingManager::load_config(const std::string & path)
{
  std::ifstream walking_file(path + "walking.json");
  nlohmann::json walking_data = nlohmann::json::parse(walking_file);

  std::ifstream kinematic_file(path + "kinematic.json");
  nlohmann::json kinematic_data = nlohmann::json::parse(kinematic_file);

  set_config(walking_data, kinematic_data);

  walking_file.close();
  kinematic_file.close();

  reinit_joints();
}

void WalkingManager::update_orientation(const keisan::Angle<double> & orientation)
{
  this->orientation = orientation;
}

void WalkingManager::update_gyro(const keisan::Vector<3> & gyro) { this->gyro = gyro; }

void WalkingManager::reinit_joints()
{
  for (auto & joint : joints) {
    uint8_t joint_id = joint.get_id();
    joint.set_position(inital_joints[joint_id]);
  }
}

void WalkingManager::set_position(const keisan::Point2 & position) { this->position = position; }

const keisan::Point2 & WalkingManager::get_position() const { return position; }

void WalkingManager::run(double x_move, double y_move, double a_move, bool aim_on)
{
  kinematic.set_move_amplitude(x_move, y_move, keisan::make_degree(a_move), aim_on);
  kinematic.set_running_state(true);
}

void WalkingManager::stop() { kinematic.set_running_state(false); }

bool WalkingManager::process()
{
  if (kinematic.run_kinematic()) {
    if (kinematic.time_to_compute_odometry()) {
      double x_amplitude = kinematic.get_x_move_amplitude();
      double y_amplitude = kinematic.get_y_move_amplitude();

      if (fabs(x_amplitude) >= 5 || fabs(y_amplitude) >= 5) {
        float dx = 0.0;
        if (x_amplitude > 0.0) {
          dx = x_amplitude * odometry_fx_coefficient / 30.0;
        } else {
          dx = x_amplitude * odometry_bx_coefficient / 30.0;
        }

        double dy = 0.0;
        if (y_amplitude > 0.0) {
          dy = -y_amplitude * odometry_ly_coefficient / 30.0;
        } else {
          dy = -y_amplitude * odometry_ry_coefficient / 30.0;
        }

        position.x += dx * orientation.cos() - dy * orientation.sin();
        position.y += dx * orientation.sin() + dy * orientation.cos();
      }
    }

    {
      using tachimawari::joint::Joint;
      using tachimawari::joint::JointId;

      auto angles = kinematic.get_angles();
      for (auto & joint : joints) {
        uint8_t joint_id = joint.get_id();

        double offset = joints_direction[joint_id] * Joint::angle_to_value(angles[joint_id]);

        joint.set_position(inital_joints[joint_id]);

        if (joint_id == JointId::LEFT_HIP_PITCH || joint_id == JointId::RIGHT_HIP_PITCH) {
          offset -= joints_direction[joint_id] * Joint::angle_to_value(kinematic.get_hip_offset());
        }

        offset += joint.get_position_value();

        if (balance_enable) {
          if (joint_id == JointId::LEFT_HIP_ROLL || joint_id == JointId::RIGHT_HIP_ROLL) {
            offset += joints_direction[joint_id] * balance_hip_roll_gain * gyro[0] * 4;
          }

          if (joint_id == JointId::LEFT_ANKLE_ROLL || joint_id == JointId::RIGHT_ANKLE_ROLL) {
            offset -= joints_direction[joint_id] * balance_ankle_roll_gain * gyro[0] * 4;
          }

          if (joint_id == JointId::LEFT_KNEE || joint_id == JointId::RIGHT_KNEE) {
            offset -= joints_direction[joint_id] * balance_knee_gain * gyro[1] * 4;
          }

          if (joint_id == JointId::LEFT_ANKLE_PITCH || joint_id == JointId::RIGHT_ANKLE_PITCH) {
            offset -= joints_direction[joint_id] * balance_ankle_pitch_gain * gyro[1] * 4;
          }
        }

        joint.set_position_value(offset);
      }
    }

    return true;
  }

  return false;
}

bool WalkingManager::is_running() const { return kinematic.get_running_state(); }

std::vector<tachimawari::joint::Joint> WalkingManager::get_joints() const { return joints; }

void WalkingManager::set_initial_joint(uint8_t id, const keisan::Angle<double> & angle)
{
  inital_joints[id] = angle.degree();
}

void WalkingManager::set_hip_pitch_offset(const keisan::Angle<double> & offset)
{
  kinematic.hip_pitch_offset = offset;
}

const Kinematic & WalkingManager::get_kinematic() const { return kinematic; }

}  // namespace aruku
