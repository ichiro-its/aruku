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
#include "tachimawari/joint/model/joint.hpp"

namespace aruku
{

class WalkingManager
{
public:
  WalkingManager();

  void load_data(const std::string & path);

  void update_orientation(double orientation);

  void run(double x_move, double y_move, double a_move, bool aim_on = false);
  void stop();
  void process();

  bool is_runing() const;

  std::vector<tachimawari::joint::Joint> get_joints() const;

private:
  void compute_odometry();

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

  // output member
  Kinematic kinematic;

  std::vector<tachimawari::joint::Joint> joints;
  std::array<double, 18> inital_joints;
  std::array<int, 18> joints_direction;

  double position_x;
  double position_y;
  double orientation;
  double fb_gyro;
  double rl_gyro;
};

}  // namespace aruku

#endif  // ARUKU__WALKING__NODE__WALKING_MANAGER_HPP_
