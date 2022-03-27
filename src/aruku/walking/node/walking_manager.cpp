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

#include <fstream>
#include <string>
#include <vector>

#include "aruku/walking/node/walking_manager.hpp"

#include "aruku/walking/process/kinematic.hpp"
#include "keisan/angle/angle.hpp"
#include "nlohmann/json.hpp"
#include "tachimawari/joint/model/joint_id.hpp"
#include "tachimawari/joint/model/joint.hpp"

namespace aruku
{

WalkingManager::WalkingManager()
: kinematic(), orientation(0.0), inital_joints({0.0}), joints_direction({1}),
  position(0.0, 0.0), fb_gyro(0.0), rl_gyro(0.0)
{
  using tachimawari::joint::JointId;
  using tachimawari::joint::Joint;

  for (auto id : JointId::list) {
    if (id < JointId::NECK_YAW) {
      joints.push_back(Joint(id, 0.0));
    } else {
      break;
    }
  }
}

void WalkingManager::set_config(nlohmann::json kinematic_data, nlohmann::json walking_data)
{
  for (auto &[key, val] : walking_data.items()) {
    if (key == "balance") {
      try {
        val.at("enable").get_to(balance_enable);
        val.at("balance_knee_gain").get_to(balance_knee_gain);
        val.at("balance_ankle_pitch_gain").get_to(balance_ankle_pitch_gain);
        val.at("balance_hip_roll_gain").get_to(balance_hip_roll_gain);
        val.at("balance_ankle_roll_gain").get_to(balance_ankle_roll_gain);
      } catch (nlohmann::json::parse_error & ex) {
        std::cerr << "parse error at byte " << ex.byte << std::endl;
      }
    } else if (key == "odometry") {
      try {
        val.at("fx_coefficient").get_to(odometry_fx_coefficient);
        val.at("ly_coefficient").get_to(odometry_ly_coefficient);
        val.at("ry_coefficient").get_to(odometry_ry_coefficient);
      } catch (nlohmann::json::parse_error & ex) {
        std::cerr << "parse error at byte " << ex.byte << std::endl;
      }
    } else if (key == "init_angles") {
      try {
        {
          using tachimawari::joint::JointId;

          val.at("right_hip_yaw").get_to(inital_joints[JointId::RIGHT_HIP_YAW]);
          val.at("right_hip_pitch").get_to(inital_joints[JointId::RIGHT_HIP_PITCH]);
          val.at("right_hip_roll").get_to(inital_joints[JointId::RIGHT_HIP_ROLL]);
          val.at("right_knee").get_to(inital_joints[JointId::RIGHT_KNEE]);
          val.at("right_ankle_pitch").get_to(inital_joints[JointId::RIGHT_ANKLE_PITCH]);
          val.at("right_ankle_roll").get_to(inital_joints[JointId::RIGHT_ANKLE_ROLL]);
          val.at("left_hip_yaw").get_to(inital_joints[JointId::LEFT_HIP_YAW]);
          val.at("left_hip_pitch").get_to(inital_joints[JointId::LEFT_HIP_PITCH]);
          val.at("left_hip_roll").get_to(inital_joints[JointId::LEFT_HIP_ROLL]);
          val.at("left_knee").get_to(inital_joints[JointId::LEFT_KNEE]);
          val.at("left_ankle_pitch").get_to(inital_joints[JointId::LEFT_ANKLE_PITCH]);
          val.at("left_ankle_roll").get_to(inital_joints[JointId::LEFT_ANKLE_ROLL]);
          val.at("right_shoulder_pitch").get_to(inital_joints[JointId::RIGHT_SHOULDER_PITCH]);
          val.at("right_shoulder_roll").get_to(inital_joints[JointId::RIGHT_SHOULDER_ROLL]);
          val.at("right_elbow").get_to(inital_joints[JointId::RIGHT_ELBOW]);
          val.at("left_shoulder_pitch").get_to(inital_joints[JointId::LEFT_SHOULDER_PITCH]);
          val.at("left_shoulder_roll").get_to(inital_joints[JointId::LEFT_SHOULDER_ROLL]);
          val.at("left_elbow").get_to(inital_joints[JointId::LEFT_ELBOW]);
        }
      } catch (nlohmann::json::parse_error & ex) {
        std::cerr << "parse error at byte " << ex.byte << std::endl;
      }
    }
  }

  kinematic.set_config(kinematic_data);
}

void WalkingManager::load_data(const std::string & path)
{
  std::ifstream file(path + "walking.json");
  nlohmann::json walking_data = nlohmann::json::parse(file);

  for (auto &[key, val] : walking_data.items()) {
    if (key == "balance") {
      try {
        val.at("enable").get_to(balance_enable);
        val.at("balance_knee_gain").get_to(balance_knee_gain);
        val.at("balance_ankle_pitch_gain").get_to(balance_ankle_pitch_gain);
        val.at("balance_hip_roll_gain").get_to(balance_hip_roll_gain);
        val.at("balance_ankle_roll_gain").get_to(balance_ankle_roll_gain);
      } catch (nlohmann::json::parse_error & ex) {
        std::cerr << "parse error at byte " << ex.byte << std::endl;
      }
    } else if (key == "pid") {
      try {
        val.at("p_gain").get_to(p_gain);
        val.at("i_gain").get_to(i_gain);
        val.at("d_gain").get_to(d_gain);
      } catch (nlohmann::json::parse_error & ex) {
        std::cerr << "parse error at byte " << ex.byte << std::endl;
      }
    } else if (key == "odometry") {
      try {
        val.at("fx_coefficient").get_to(odometry_fx_coefficient);
        val.at("ly_coefficient").get_to(odometry_ly_coefficient);
        val.at("ry_coefficient").get_to(odometry_ry_coefficient);
      } catch (nlohmann::json::parse_error & ex) {
        std::cerr << "parse error at byte " << ex.byte << std::endl;
      }
    } else if (key == "init_angles") {
      try {
        {
          using tachimawari::joint::JointId;

          val.at("right_hip_yaw").get_to(inital_joints[JointId::RIGHT_HIP_YAW]);
          val.at("right_hip_pitch").get_to(inital_joints[JointId::RIGHT_HIP_PITCH]);
          val.at("right_hip_roll").get_to(inital_joints[JointId::RIGHT_HIP_ROLL]);
          val.at("right_knee").get_to(inital_joints[JointId::RIGHT_KNEE]);
          val.at("right_ankle_pitch").get_to(inital_joints[JointId::RIGHT_ANKLE_PITCH]);
          val.at("right_ankle_roll").get_to(inital_joints[JointId::RIGHT_ANKLE_ROLL]);
          val.at("left_hip_yaw").get_to(inital_joints[JointId::LEFT_HIP_YAW]);
          val.at("left_hip_pitch").get_to(inital_joints[JointId::LEFT_HIP_PITCH]);
          val.at("left_hip_roll").get_to(inital_joints[JointId::LEFT_HIP_ROLL]);
          val.at("left_knee").get_to(inital_joints[JointId::LEFT_KNEE]);
          val.at("left_ankle_pitch").get_to(inital_joints[JointId::LEFT_ANKLE_PITCH]);
          val.at("left_ankle_roll").get_to(inital_joints[JointId::LEFT_ANKLE_ROLL]);
          val.at("right_shoulder_pitch").get_to(inital_joints[JointId::RIGHT_SHOULDER_PITCH]);
          val.at("right_shoulder_roll").get_to(inital_joints[JointId::RIGHT_SHOULDER_ROLL]);
          val.at("right_elbow").get_to(inital_joints[JointId::RIGHT_ELBOW]);
          val.at("left_shoulder_pitch").get_to(inital_joints[JointId::LEFT_SHOULDER_PITCH]);
          val.at("left_shoulder_roll").get_to(inital_joints[JointId::LEFT_SHOULDER_ROLL]);
          val.at("left_elbow").get_to(inital_joints[JointId::LEFT_ELBOW]);
        }
      } catch (nlohmann::json::parse_error & ex) {
        std::cerr << "parse error at byte " << ex.byte << std::endl;
      }
    } else if (key == "angles_direction") {
      try {
        {
          using tachimawari::joint::JointId;

          val.at("right_hip_yaw").get_to(joints_direction[JointId::RIGHT_HIP_YAW]);
          val.at("right_hip_pitch").get_to(joints_direction[JointId::RIGHT_HIP_PITCH]);
          val.at("right_hip_roll").get_to(joints_direction[JointId::RIGHT_HIP_ROLL]);
          val.at("right_knee").get_to(joints_direction[JointId::RIGHT_KNEE]);
          val.at("right_ankle_pitch").get_to(joints_direction[JointId::RIGHT_ANKLE_PITCH]);
          val.at("right_ankle_roll").get_to(joints_direction[JointId::RIGHT_ANKLE_ROLL]);
          val.at("left_hip_yaw").get_to(joints_direction[JointId::LEFT_HIP_YAW]);
          val.at("left_hip_pitch").get_to(joints_direction[JointId::LEFT_HIP_PITCH]);
          val.at("left_hip_roll").get_to(joints_direction[JointId::LEFT_HIP_ROLL]);
          val.at("left_knee").get_to(joints_direction[JointId::LEFT_KNEE]);
          val.at("left_ankle_pitch").get_to(joints_direction[JointId::LEFT_ANKLE_PITCH]);
          val.at("left_ankle_roll").get_to(joints_direction[JointId::LEFT_ANKLE_ROLL]);
          val.at("right_shoulder_pitch").get_to(joints_direction[JointId::RIGHT_SHOULDER_PITCH]);
          val.at("right_shoulder_roll").get_to(joints_direction[JointId::RIGHT_SHOULDER_ROLL]);
          val.at("right_elbow").get_to(joints_direction[JointId::RIGHT_ELBOW]);
          val.at("left_shoulder_pitch").get_to(joints_direction[JointId::LEFT_SHOULDER_PITCH]);
          val.at("left_shoulder_roll").get_to(joints_direction[JointId::LEFT_SHOULDER_ROLL]);
          val.at("left_elbow").get_to(joints_direction[JointId::LEFT_ELBOW]);
        }
      } catch (nlohmann::json::parse_error & ex) {
        std::cerr << "parse error at byte " << ex.byte << std::endl;
      }
    }
  }

  file.close();

  kinematic.load_data(path);
}

void WalkingManager::update_imu(double orientation)
{
  this->orientation = orientation;
}

void WalkingManager::update_imu(double fb_gyro, double rl_gyro)
{
  this->fb_gyro = fb_gyro;
  this->rl_gyro = rl_gyro;
}

const keisan::Point2 & WalkingManager::get_position() const
{
  return position;
}

void WalkingManager::run(double x_move, double y_move, double a_move, bool aim_on)
{
  kinematic.set_move_amplitude(x_move, y_move, a_move, aim_on);
  kinematic.set_running_state(true);
}

void WalkingManager::stop()
{
  kinematic.set_running_state(false);
}

bool WalkingManager::process()
{
  if (kinematic.run_kinematic()) {
    if (kinematic.time_to_compute_odometry()) {
      double x_amplitude = kinematic.get_x_move_amplitude();
      double y_amplitude = kinematic.get_y_move_amplitude();

      if (fabs(x_amplitude) >= 5 || fabs(y_amplitude) >= 5) {
        double dx = x_amplitude * odometry_fx_coefficient / 30.0;

        double dy = 0.0;
        if (y_amplitude > 0.0) {
          dy = -y_amplitude * odometry_ly_coefficient / 30.0;
        } else {
          dy = -y_amplitude * odometry_ry_coefficient / 30.0;
        }

        auto theta = keisan::make_degree(orientation);

        position.x += dx * theta.cos() - dy * theta.sin();
        position.y += dx * theta.sin() + dy * theta.cos();
      }
    }

    {
      using tachimawari::joint::JointId;
      using tachimawari::joint::Joint;

      auto angles = kinematic.get_angles();
      for (auto & joint : joints) {
        uint8_t joint_id = joint.get_id();
        double offset = joints_direction[joint_id] *
          keisan::make_radian(angles[joint_id]).degree() *
          Joint::TO_VALUE_RATIO;

        joint.set_position(inital_joints[joint_id]);

        if (joint_id == JointId::LEFT_HIP_PITCH || joint_id == JointId::RIGHT_HIP_PITCH) {
          offset -= joints_direction[joint_id] * kinematic.get_hip_offest() * Joint::TO_VALUE_RATIO;
        }

        offset += joint.get_position_value();

        if (balance_enable) {
          if (joint_id == JointId::LEFT_HIP_ROLL || joint_id == JointId::RIGHT_HIP_ROLL) {
            offset += joints_direction[joint_id] * balance_hip_roll_gain * rl_gyro * 4;
          }

          if (joint_id == JointId::LEFT_ANKLE_ROLL || joint_id == JointId::RIGHT_ANKLE_ROLL) {
            offset -= joints_direction[joint_id] * balance_ankle_roll_gain * rl_gyro * 4;
          }

          if (joint_id == JointId::LEFT_KNEE || joint_id == JointId::RIGHT_KNEE) {
            offset -= joints_direction[joint_id] * balance_knee_gain * fb_gyro * 4;
          }

          if (joint_id == JointId::LEFT_ANKLE_PITCH || joint_id == JointId::RIGHT_ANKLE_PITCH) {
            offset -= joints_direction[joint_id] * balance_ankle_pitch_gain * fb_gyro * 4;
          }
        }

        joint.set_position_value(offset);
      }
    }

    return true;
  }

  return false;
}

bool WalkingManager::is_runing() const
{
  return kinematic.get_running_state();
}

std::vector<tachimawari::joint::Joint> WalkingManager::get_joints() const
{
  return joints;
}


}  // namespace aruku
