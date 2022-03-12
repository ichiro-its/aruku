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

#ifndef ARUKU__MOVEMENT__MODEL__MOVEMENT_HPP_
#define ARUKU__MOVEMENT__MODEL__MOVEMENT_HPP_

#include <algorithm>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "tachimawari/joint/model/joint.hpp"

namespace aruku
{

class Movement
{
public:
  Movement();

  void initialize();

  void load_data(const std::string & path);

  double x_offset;
  double y_offset;
  double z_offset;
  double a_offset;
  double p_offset;
  double r_offset;

  double period_time;
  double dsp_ratio;

  bool balance_enable;
  double balance_knee_gain;
  double balance_ankle_pitch_gain;
  double balance_hip_roll_gain;
  double balance_ankle_roll_gain;

  double y_swap_amplitude;
  double z_swap_amplitude;
  double arm_swing_gain;
  double hip_pitch_offset;

  double initial_r_hip_yaw;
  double initial_r_hip_roll;
  double initial_r_hip_pitch;
  double initial_r_knee;
  double initial_r_ankle_pitch;
  double initial_r_ankle_roll;
  double initial_r_shoulder_pitch;
  double initial_r_shoulder_roll;
  double initial_r_elbow;
  double initial_r_gripper;

  double initial_l_hip_yaw;
  double initial_l_hip_roll;
  double initial_l_hip_pitch;
  double initial_l_knee;
  double initial_l_ankle_pitch;
  double initial_l_ankle_roll;
  double initial_l_shoulder_pitch;
  double initial_l_shoulder_roll;
  double initial_l_elbow;
  double initial_l_gripper;

  double thigh_length;
  double calf_length;
  double ankle_length;
  double leg_length;

  int p_gain;
  int i_gain;
  int d_gain;

  double position_x;
  double position_y;
  double orientation;
  double odometry_fx_coefficient;
  double odometry_ly_coefficient;
  double odometry_ry_coefficient;

  double dsp_comp_ratio;
  double hip_comp_ratio;
  double period_comp_ratio;
  double backward_hip_comp_ratio;
  double forward_hip_comp_ratio;

  double foot_comp_ratio;
  double move_accel_ratio;
  double foot_accel_ratio;

private:
  std::vector<tachimawari::joint::Joint> joints;
};

}  // namespace aruku

#endif  // ARUKU__MOVEMENT__MODEL__MOVEMENT_HPP_
