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

#ifndef ARUKU__WALKING__NODE__WALKING_MANAGER_HPP_
#define ARUKU__WALKING__NODE__WALKING_MANAGER_HPP_

#include <array>
#include <string>
#include <vector>

#include "aruku/walking/process/kinematic.hpp"
#include "keisan/geometry/point_2.hpp"
#include "nlohmann/json.hpp"
#include "tachimawari/joint/model/joint.hpp"

namespace aruku
{

class WalkingManager
{
public:
  WalkingManager();

  void set_config(const nlohmann::json & walking_data, const nlohmann::json & kinematic_data);
  void load_config(const std::string & path);

  void update_orientation(const keisan::Angle<double> & orientation);
  void update_gyro(const keisan::Vector<3> & gyro);
  void reinit_joints();

  void set_position(const keisan::Point2 & position);
  const keisan::Point2 & get_position() const;

  void run(double x_move, double y_move, double a_move, bool aim_on = false, bool inverse_a_move = false);
  void stop();
  bool process();

  bool is_running() const;

  std::vector<tachimawari::joint::Joint> get_joints() const;
  const Kinematic & get_kinematic() const;

private:
  // config member
  bool balance_enable;
  double balance_knee_gain;
  double balance_ankle_pitch_gain;
  double balance_hip_roll_gain;
  double balance_ankle_roll_gain;

  int p_gain;
  int i_gain;
  int d_gain;

  double odometry_fx_coefficient;
  double odometry_ly_coefficient;
  double odometry_ry_coefficient;

  bool using_antibacklash;

  // output member
  Kinematic kinematic;

  std::vector<tachimawari::joint::Joint> joints;
  std::array<double, 19> inital_joints;
  std::array<double, 19> joints_direction;

  keisan::Point2 position;

  keisan::Angle<double> orientation;
  keisan::Vector<3> gyro;
};

}  // namespace aruku

#endif  // ARUKU__WALKING__NODE__WALKING_MANAGER_HPP_
