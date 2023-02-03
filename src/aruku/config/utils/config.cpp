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
#include <iomanip>
#include <string>

#include "aruku/config/utils/config.hpp"
#include "aruku/walking/process/kinematic.hpp"

#include "nlohmann/json.hpp"

namespace aruku
{

Config::Config(const std::string & path)
: path(path)
{
}

std::string Config::get_config(const std::string & key) const
{
  if (key == "walking") {
    std::ifstream walking_file(path + "walking.json");
    nlohmann::json walking_data = nlohmann::json::parse(walking_file);
    walking_file.close();

    walking_data.erase("angles_direction");
    walking_data.erase("pid");

    return walking_data.dump();
  } else if (key == "kinematic") {
    std::ifstream kinematic_file(path + "kinematic.json");
    nlohmann::json kinematic_data = nlohmann::json::parse(kinematic_file);

    kinematic_data.erase("length");

    return kinematic_data.dump();
  }

  return "";
}

void Config::save_config(
  const nlohmann::json & kinematic_data, const nlohmann::json & walking_data)
{
  std::ofstream kinematic_file(path + "kinematic.json", std::ios::out | std::ios::trunc);
  kinematic_file << std::setw(2) << kinematic_data << std::endl;
  kinematic_file.close();

  std::ofstream walking_file(path + "walking.json", std::ios::out | std::ios::trunc);
  walking_file << std::setw(2) << walking_data << std::endl;
  walking_file.close();
}

// void Config::set_config(const nlohmann::json & walking_data)
// {
//   using tachimawari::joint::JointId;
//   using tachimawari::joint::Joint;

//   for (auto &[key, val] : walking_data.items()) {
//     // if (key == "balance") {
//     //   try {
//     //     val.at("enable").get_to(balance_enable);
//     //     val.at("balance_knee_gain").get_to(balance_knee_gain);
//     //     val.at("balance_ankle_pitch_gain").get_to(balance_ankle_pitch_gain);
//     //     val.at("balance_hip_roll_gain").get_to(balance_hip_roll_gain);
//     //     val.at("balance_ankle_roll_gain").get_to(balance_ankle_roll_gain);
//     //   } catch (nlohmann::json::parse_error & ex) {
//     //     std::cerr << "parse error at byte " << ex.byte << std::endl;
//     //   }
//     // } else if (key == "pid") {
//     //   try {
//     //     val.at("p_gain").get_to(p_gain);
//     //     val.at("i_gain").get_to(i_gain);
//     //     val.at("d_gain").get_to(d_gain);
//     //   } catch (nlohmann::json::parse_error & ex) {
//     //     std::cerr << "parse error at byte " << ex.byte << std::endl;
//     //   }
//     // } else if (key == "odometry") {
//     //   try {
//     //     val.at("fx_coefficient").get_to(odometry_fx_coefficient);
//     //     val.at("ly_coefficient").get_to(odometry_ly_coefficient);
//     //     val.at("ry_coefficient").get_to(odometry_ry_coefficient);
//     //   } catch (nlohmann::json::parse_error & ex) {
//     //     std::cerr << "parse error at byte " << ex.byte << std::endl;
//     //   }
//     if (key == "init_angles") {
//       try {
//         {
//           val.at("right_hip_yaw").get_to(inital_joints[JointId::RIGHT_HIP_YAW]);
//           val.at("right_hip_pitch").get_to(inital_joints[JointId::RIGHT_HIP_PITCH]);
//           val.at("right_hip_roll").get_to(inital_joints[JointId::RIGHT_HIP_ROLL]);
//           val.at("right_knee").get_to(inital_joints[JointId::RIGHT_KNEE]);
//           val.at("right_ankle_pitch").get_to(inital_joints[JointId::RIGHT_ANKLE_PITCH]);
//           val.at("right_ankle_roll").get_to(inital_joints[JointId::RIGHT_ANKLE_ROLL]);
//           val.at("left_hip_yaw").get_to(inital_joints[JointId::LEFT_HIP_YAW]);
//           val.at("left_hip_pitch").get_to(inital_joints[JointId::LEFT_HIP_PITCH]);
//           val.at("left_hip_roll").get_to(inital_joints[JointId::LEFT_HIP_ROLL]);
//           val.at("left_knee").get_to(inital_joints[JointId::LEFT_KNEE]);
//           val.at("left_ankle_pitch").get_to(inital_joints[JointId::LEFT_ANKLE_PITCH]);
//           val.at("left_ankle_roll").get_to(inital_joints[JointId::LEFT_ANKLE_ROLL]);
//           val.at("right_shoulder_pitch").get_to(inital_joints[JointId::RIGHT_SHOULDER_PITCH]);
//           val.at("right_shoulder_roll").get_to(inital_joints[JointId::RIGHT_SHOULDER_ROLL]);
//           val.at("right_elbow").get_to(inital_joints[JointId::RIGHT_ELBOW]);
//           val.at("left_shoulder_pitch").get_to(inital_joints[JointId::LEFT_SHOULDER_PITCH]);
//           val.at("left_shoulder_roll").get_to(inital_joints[JointId::LEFT_SHOULDER_ROLL]);
//           val.at("left_elbow").get_to(inital_joints[JointId::LEFT_ELBOW]);
//         }
//       } catch (nlohmann::json::parse_error & ex) {
//         std::cerr << "parse error at byte " << ex.byte << std::endl;
//       }
//     } else if (key == "angles_direction") {
//       try {
//         {
//           val.at("right_hip_yaw").get_to(joints_direction[JointId::RIGHT_HIP_YAW]);
//           val.at("right_hip_pitch").get_to(joints_direction[JointId::RIGHT_HIP_PITCH]);
//           val.at("right_hip_roll").get_to(joints_direction[JointId::RIGHT_HIP_ROLL]);
//           val.at("right_knee").get_to(joints_direction[JointId::RIGHT_KNEE]);
//           val.at("right_ankle_pitch").get_to(joints_direction[JointId::RIGHT_ANKLE_PITCH]);
//           val.at("right_ankle_roll").get_to(joints_direction[JointId::RIGHT_ANKLE_ROLL]);
//           val.at("left_hip_yaw").get_to(joints_direction[JointId::LEFT_HIP_YAW]);
//           val.at("left_hip_pitch").get_to(joints_direction[JointId::LEFT_HIP_PITCH]);
//           val.at("left_hip_roll").get_to(joints_direction[JointId::LEFT_HIP_ROLL]);
//           val.at("left_knee").get_to(joints_direction[JointId::LEFT_KNEE]);
//           val.at("left_ankle_pitch").get_to(joints_direction[JointId::LEFT_ANKLE_PITCH]);
//           val.at("left_ankle_roll").get_to(joints_direction[JointId::LEFT_ANKLE_ROLL]);
//           val.at("right_shoulder_pitch").get_to(joints_direction[JointId::RIGHT_SHOULDER_PITCH]);
//           val.at("right_shoulder_roll").get_to(joints_direction[JointId::RIGHT_SHOULDER_ROLL]);
//           val.at("right_elbow").get_to(joints_direction[JointId::RIGHT_ELBOW]);
//           val.at("left_shoulder_pitch").get_to(joints_direction[JointId::LEFT_SHOULDER_PITCH]);
//           val.at("left_shoulder_roll").get_to(joints_direction[JointId::LEFT_SHOULDER_ROLL]);
//           val.at("left_elbow").get_to(joints_direction[JointId::LEFT_ELBOW]);
//         }
//       } catch (nlohmann::json::parse_error & ex) {
//         std::cerr << "parse error at byte " << ex.byte << std::endl;
//       }
//     }
//   }

//   for (auto & joint : joints) {
//     uint8_t joint_id = joint.get_id();

//     joint.set_position(inital_joints[joint_id]);
//   }
// }

}  // namespace aruku
