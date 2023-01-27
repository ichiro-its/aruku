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

#ifndef ARUKU__CONFIG__UTILS__CONFIG_HPP_
#define ARUKU__CONFIG__UTILS__CONFIG_HPP_

#include <fstream>
#include <map>
#include <string>

#include "nlohmann/json.hpp"
#include "tachimawari/joint/model/joint.hpp"
#include "aruku/walking/process/kinematic.hpp"
#include "tachimawari/joint/model/joint.hpp"

namespace aruku
{

class Config
{
public:
  explicit Config(const std::string & path);

  std::string get_config(const std::string & key) const;
  void save_config(
    const nlohmann::json & kinematic_data, const nlohmann::json & walking_data);
  void set_config(const nlohmann::json & walking_data);

private:
  std::string path;

  // config member
  // bool balance_enable;
  // double balance_knee_gain;
  // double balance_ankle_pitch_gain;
  // double balance_hip_roll_gain;
  // double balance_ankle_roll_gain;

  // int p_gain;
  // int i_gain;
  // int d_gain;

  // double odometry_fx_coefficient;
  // double odometry_ly_coefficient;
  // double odometry_ry_coefficient;

  // Kinematic kinematic;

  std::vector<tachimawari::joint::Joint> joints;
  std::array<double, 19> inital_joints;
  std::array<double, 19> joints_direction;

  // keisan::Point2 position;

  // keisan::Angle<double> orientation;
};

}  // namespace aruku

#endif  // ARUKU__CONFIG__UTILS__CONFIG_HPP_
