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

#include <aruku/walking.hpp>
#include <tachimawari/joint.hpp>

#include "common/algebra.h"
#include "math/matrix.h"
#include "math/vector.h"

#include <map>
#include <iostream>
#include <string>
#include <vector>
#include <cmath>
#include <cstdlib>
#include <memory>
#include <fstream>
#include <nlohmann/json.hpp>

namespace aruku
{

Walking::Walking()
{
  m_PeriodTime = 0;
  m_DSP_Ratio = 0;
  m_SSP_Ratio = 0;
  m_X_Swap_PeriodTime = 0;
  m_X_Move_PeriodTime = 0;
  m_Y_Swap_PeriodTime = 0;
  m_Y_Move_PeriodTime = 0;
  m_Z_Swap_PeriodTime = 0;
  m_Z_Move_PeriodTime = 0;
  m_A_Move_PeriodTime = 0;
  m_SSP_Time = 0;
  m_SSP_Time_Start_L = 0;
  m_SSP_Time_End_L = 0;
  m_SSP_Time_Start_R = 0;
  m_SSP_Time_End_R = 0;
  m_Phase_Time1 = 0;
  m_Phase_Time2 = 0;
  m_Phase_Time3 = 0;

  m_X_Offset = 0;
  m_Y_Offset = 0;
  m_Z_Offset = 0;
  m_R_Offset = 0;
  m_P_Offset = 0;
  m_A_Offset = 0;

  m_X_Swap_Phase_Shift = 0;
  m_X_Swap_Amplitude = 0;
  m_X_Swap_Amplitude_Shift = 0;
  m_X_Move_Phase_Shift = 0;
  m_X_Move_Amplitude = 0;
  m_X_Move_Amplitude_Shift = 0;
  m_Y_Swap_Phase_Shift = 0;
  m_Y_Swap_Amplitude = 0;
  m_Y_Swap_Amplitude_Shift = 0;
  m_Y_Move_Phase_Shift = 0;
  m_Y_Move_Amplitude = 0;
  m_Y_Move_Amplitude_Shift = 0;
  m_Z_Swap_Phase_Shift = 0;
  m_Z_Swap_Amplitude = 0;
  m_Z_Swap_Amplitude_Shift = 0;
  m_Z_Move_Phase_Shift = 0;
  m_Z_Move_Amplitude = 0;
  m_Z_Move_Amplitude_Goal = 0;
  m_Z_Move_Amplitude_Shift = 0;
  m_A_Move_Phase_Shift = 0;
  m_A_Move_Amplitude = 0;
  m_A_Move_Amplitude_Shift = 0;

  m_Pelvis_Offset = 0;
  m_Pelvis_Swing = 0;
  m_Arm_Swing_Gain = 0;
  m_Arm_Roll_Gain = 0;

  X_MOVE_AMPLITUDE = 0;
  Y_MOVE_AMPLITUDE = 0;
  A_MOVE_AMPLITUDE = 0;
  A_MOVE_AIM_ON = false;

  BALANCE_ENABLE = true;

  POSITION_X = 0;
  POSITION_Y = 0;
  ORIENTATION = 0;

  HIP_COMP = 0.0;
  FOOT_COMP = 0.0;

  TIME_UNIT = 8.0;

  dynamic_left_kick_ = 0.0;
  dynamic_right_kick_ = 0.0;

  std::vector<std::string> ids = {
      // right leg motors
      "right_hip_yaw",
      "right_hip_roll",
      "right_hip_pitch",
      "right_knee",
      "right_ankle_roll",
      "right_ankle_pitch",

      // left leg motors
      "left_hip_yaw",
      "left_hip_roll",
      "left_hip_pitch",
      "left_knee",
      "left_ankle_roll",
      "left_ankle_pitch",

      // right arm motors
      "right_shoulder_pitch",
      "right_shoulder_roll",
      "right_elbow",

      // left arm motors
      "left_shoulder_pitch",
      "left_shoulder_roll",
      "left_elbow",
  };

  for (auto id : ids)
  {
    tachimawari::Joint joint(id);
    joints.push_back(joint);
  }
}

double Walking::wsin(double time, double period, double period_shift, double mag, double mag_shift)
{
  return mag * sin(2 * 3.141592 / period * time - period_shift) + mag_shift;
}

bool Walking::compute_ik(double *out, double x, double y, double z, double a, double b, double c)
{
  Robot::Matrix3D Tad, Tda, Tcd, Tdc, Tac;
  Robot::Vector3D vec;
  double _Rac, _Acos, _Atan, _k, _l, _m, _n, _s, _c, _theta;

  Tad.SetTransform(Robot::Point3D(x, y, z - LEG_LENGTH), Robot::Vector3D(a * alg::rad2Deg(), b * alg::rad2Deg(), c * alg::rad2Deg()));

  vec.X = x + Tad.m[2] * ANKLE_LENGTH;
  vec.Y = y + Tad.m[6] * ANKLE_LENGTH;
  vec.Z = (z - LEG_LENGTH) + Tad.m[10] * ANKLE_LENGTH;

  _Rac = vec.Length();
  _Acos = acos((_Rac * _Rac - THIGH_LENGTH * THIGH_LENGTH - CALF_LENGTH * CALF_LENGTH) / (2 * THIGH_LENGTH * CALF_LENGTH));
  if (std::isnan(_Acos) == 1)
    return false;
  *(out + 3) = _Acos;

  Tda = Tad;
  if (Tda.Inverse() == false)
    return false;
  _k = sqrt(Tda.m[7] * Tda.m[7] + Tda.m[11] * Tda.m[11]);
  _l = sqrt(Tda.m[7] * Tda.m[7] + (Tda.m[11] - ANKLE_LENGTH) * (Tda.m[11] - ANKLE_LENGTH));
  _m = (_k * _k - _l * _l - ANKLE_LENGTH * ANKLE_LENGTH) / (2 * _l * ANKLE_LENGTH);
  if (_m > 1.0)
    _m = 1.0;
  else if (_m < -1.0)
    _m = -1.0;
  _Acos = acos(_m);
  if (std::isnan(_Acos) == 1)
    return false;
  if (Tda.m[7] < 0.0)
    *(out + 5) = -_Acos;
  else
    *(out + 5) = _Acos;

  Tcd.SetTransform(Robot::Point3D(0, 0, -ANKLE_LENGTH), Robot::Vector3D(*(out + 5) * alg::rad2Deg(), 0, 0));
  Tdc = Tcd;
  if (Tdc.Inverse() == false)
    return false;
  Tac = Tad * Tdc;
  _Atan = atan2(-Tac.m[1], Tac.m[5]);
  if (std::isinf(_Atan) == 1)
    return false;
  *(out) = _Atan;

  // Get Hip Roll
  _Atan = atan2(Tac.m[9], -Tac.m[1] * sin(*(out)) + Tac.m[5] * cos(*(out)));
  if (std::isinf(_Atan) == 1)
    return false;
  *(out + 1) = _Atan;

  // Get Hip Pitch and Ankle Pitch
  _Atan = atan2(Tac.m[2] * cos(*(out)) + Tac.m[6] * sin(*(out)), Tac.m[0] * cos(*(out)) + Tac.m[4] * sin(*(out)));
  if (std::isinf(_Atan) == 1)
    return false;
  _theta = _Atan;
  _k = sin(*(out + 3)) * CALF_LENGTH;
  _l = -THIGH_LENGTH - cos(*(out + 3)) * CALF_LENGTH;
  _m = cos(*(out)) * vec.X + sin(*(out)) * vec.Y;
  _n = cos(*(out + 1)) * vec.Z + sin(*(out)) * sin(*(out + 1)) * vec.X - cos(*(out)) * sin(*(out + 1)) * vec.Y;
  _s = (_k * _n + _l * _m) / (_k * _k + _l * _l);
  _c = (_n - _k * _s) / _l;
  _Atan = atan2(_s, _c);
  if (std::isinf(_Atan) == 1)
    return false;
  *(out + 2) = _Atan;
  *(out + 4) = _theta - *(out + 3) - *(out + 2);

  return true;
}

void Walking::compute_odometry()
{
  // ORIENTATION = MPU::getInstance()->getAngle();

  if (fabs(m_X_Move_Amplitude) >= 5 || fabs(m_Y_Move_Amplitude) >= 5)
  {
    float dx = m_X_Move_Amplitude * ODOMETRY_FX_COEFFICIENT / 30.0;

    float dy = 0.0;
    if (m_Y_Move_Amplitude > 0.0)
      dy = -m_Y_Move_Amplitude * ODOMETRY_LY_COEFFICIENT / 30.0;
    else
      dy = -m_Y_Move_Amplitude * ODOMETRY_RY_COEFFICIENT / 30.0;

    float theta = ORIENTATION * alg::deg2Rad();

    POSITION_X += dx * cos(theta) - dy * sin(theta);
    POSITION_Y += dx * sin(theta) + dy * cos(theta);
  }
}

void Walking::update_param_time()
{
  DSP_COMP = fabs(m_X_Move_Amplitude) * DSP_COMP_RATIO * 0.001;

  m_PeriodTime = PERIOD_TIME - (fabs(m_X_Move_Amplitude) * PERIOD_COMP_RATIO);

  m_DSP_Ratio = DSP_RATIO + DSP_COMP;
  m_SSP_Ratio = 1 - m_DSP_Ratio;

  m_X_Swap_PeriodTime = m_PeriodTime / 2;
  m_X_Move_PeriodTime = m_PeriodTime * m_SSP_Ratio;
  m_Y_Swap_PeriodTime = m_PeriodTime;
  m_Y_Move_PeriodTime = m_PeriodTime * m_SSP_Ratio;
  m_Z_Swap_PeriodTime = m_PeriodTime / 2;
  m_Z_Move_PeriodTime = m_PeriodTime * m_SSP_Ratio / 2;
  m_A_Move_PeriodTime = m_PeriodTime * m_SSP_Ratio;

  m_SSP_Time = m_PeriodTime * m_SSP_Ratio;
  m_SSP_Time_Start_L = (1 - m_SSP_Ratio) * m_PeriodTime / 4;
  m_SSP_Time_End_L = (1 + m_SSP_Ratio) * m_PeriodTime / 4;
  m_SSP_Time_Start_R = (3 - m_SSP_Ratio) * m_PeriodTime / 4;
  m_SSP_Time_End_R = (3 + m_SSP_Ratio) * m_PeriodTime / 4;

  m_Phase_Time1 = (m_SSP_Time_Start_L + m_SSP_Time_End_L) / 2;
  m_Phase_Time2 = (m_SSP_Time_End_L + m_SSP_Time_Start_R) / 2;
  m_Phase_Time3 = (m_SSP_Time_Start_R + m_SSP_Time_End_R) / 2;

  m_Pelvis_Offset = PELVIS_OFFSET * mx.RATIO_ANGLE2VALUE;
  m_Pelvis_Swing = m_Pelvis_Offset * 0.35;
  m_Arm_Swing_Gain = ARM_SWING_GAIN;

  if (m_X_Move_Amplitude > 0)
    HIP_COMP = m_X_Move_Amplitude * FORWARD_HIP_COMP_RATIO;
  else
    HIP_COMP = m_X_Move_Amplitude * BACKWARD_HIP_COMP_RATIO;

  FOOT_COMP = fabs(m_X_Move_Amplitude) * FOOT_COMP_RATIO;
}

void Walking::update_param_move()
{
  // Forward/Back
  double x_input = X_MOVE_AMPLITUDE;
  double y_input = Y_MOVE_AMPLITUDE * 0.5;
  double a_input = A_MOVE_AMPLITUDE;

  if (m_Z_Move_Amplitude < (Z_MOVE_AMPLITUDE * 0.45))
  {
    x_input = 0.0;
    y_input = 0.0;
    a_input = 0.0;
  }

  m_X_Move_Amplitude = alg::smoothValue(m_X_Move_Amplitude, x_input, MOVE_ACCEL_RATIO);
  m_X_Swap_Amplitude = x_input * STEP_FB_RATIO;

  m_Y_Move_Amplitude = alg::smoothValue(m_Y_Move_Amplitude, y_input, MOVE_ACCEL_RATIO);
  m_Y_Move_Amplitude_Shift = fabs(m_Y_Move_Amplitude);
  m_Y_Swap_Amplitude = Y_SWAP_AMPLITUDE + m_Y_Move_Amplitude_Shift * 0.04;

  if (m_Ctrl_Running)
    m_Z_Move_Amplitude = alg::smoothValue(m_Z_Move_Amplitude, (Z_MOVE_AMPLITUDE + FOOT_COMP) * 0.5, FOOT_ACCEL_RATIO);
  else
    m_Z_Move_Amplitude = 0.0;

  m_Z_Move_Amplitude_Shift = m_Z_Move_Amplitude * 0.5;
  m_Z_Swap_Amplitude = Z_SWAP_AMPLITUDE;

  if (A_MOVE_AIM_ON == false)
  {
    m_A_Move_Amplitude = alg::smoothValue(m_A_Move_Amplitude, a_input * alg::deg2Rad() * 0.5, MOVE_ACCEL_RATIO);
    m_A_Move_Amplitude_Shift = fabs(m_A_Move_Amplitude);
  }
  else
  {
    m_A_Move_Amplitude = alg::smoothValue(m_A_Move_Amplitude, (-a_input) * alg::deg2Rad() * 0.5, MOVE_ACCEL_RATIO);
    m_A_Move_Amplitude_Shift = -fabs(m_A_Move_Amplitude);
  }
}

void Walking::update_orientation(double orientation)
{
  ORIENTATION = orientation;
}

void Walking::load_data()
{
  std::cout << "load_data" << std::endl;
  std::string file_name =
      "/home/maroqi/Projects/RobotProjects/ROS2Projects/ros2_workspace/src/aruku/config/aruku.json";
  std::ifstream file(file_name);
  nlohmann::json walking_data = nlohmann::json::parse(file);

  for (auto &[key, val] : walking_data.items())
  {
    if (key == "Ratio")
    {
      try
      {
        PERIOD_TIME = static_cast<double>(walking_data[key]["period_time"].get<double>());
        DSP_RATIO = static_cast<double>(walking_data[key]["dsp_ratio"].get<double>());
        Z_MOVE_AMPLITUDE = static_cast<double>(walking_data[key]["foot_height"].get<double>());
        Y_SWAP_AMPLITUDE = static_cast<double>(walking_data[key]["swing_right_left"].get<double>());
        Z_SWAP_AMPLITUDE = static_cast<double>(walking_data[key]["swing_up_down"].get<double>());
        ARM_SWING_GAIN = static_cast<double>(walking_data[key]["arm_swing_gain"].get<double>());
        BACKWARD_HIP_COMP_RATIO = static_cast<double>(walking_data[key]["backward_hip_comp_ratio"].get<double>());
        FORWARD_HIP_COMP_RATIO = static_cast<double>(walking_data[key]["forward_hip_comp_ratio"].get<double>());
        FOOT_COMP_RATIO = static_cast<double>(walking_data[key]["foot_comp_ratio"].get<double>());
        DSP_COMP_RATIO = static_cast<double>(walking_data[key]["dsp_comp_ratio"].get<double>());
        PERIOD_COMP_RATIO = static_cast<double>(walking_data[key]["period_comp_ratio"].get<double>());
        MOVE_ACCEL_RATIO = static_cast<double>(walking_data[key]["move_accel_ratio"].get<double>());
        FOOT_ACCEL_RATIO = static_cast<double>(walking_data[key]["foot_accel_ratio"].get<double>());
      }
      catch (nlohmann::json::parse_error &ex)
      {
        std::cerr << "parse error at byte " << ex.byte << std::endl;
      }
    }
    else if (key == "Balance")
    {
      try
      {
        BALANCE_KNEE_GAIN = static_cast<double>(walking_data[key]["balance_knee_gain"].get<double>());
        BALANCE_ANKLE_PITCH_GAIN = static_cast<double>(walking_data[key]["balance_ankle_pitch_gain"].get<double>());
        BALANCE_HIP_ROLL_GAIN = static_cast<double>(walking_data[key]["balance_hip_roll_gain"].get<double>());
        BALANCE_ANKLE_ROLL_GAIN = static_cast<double>(walking_data[key]["balance_ankle_roll_gain"].get<double>());
      }
      catch (nlohmann::json::parse_error &ex)
      {
        std::cerr << "parse error at byte " << ex.byte << std::endl;
      }
    }
    else if (key == "PID")
    {
      try
      {
        P_GAIN = static_cast<int>(walking_data[key]["p_gain"].get<int>());
        I_GAIN = static_cast<int>(walking_data[key]["i_gain"].get<int>());
        D_GAIN = static_cast<int>(walking_data[key]["d_gain"].get<int>());
      }
      catch (nlohmann::json::parse_error &ex)
      {
        std::cerr << "parse error at byte " << ex.byte << std::endl;
      }
    }
    else if (key == "Odometry")
    {
      try
      {
        ODOMETRY_FX_COEFFICIENT = static_cast<double>(walking_data[key]["fx_coefficient"].get<double>());
        ODOMETRY_LY_COEFFICIENT = static_cast<double>(walking_data[key]["ly_coefficient"].get<double>());
        ODOMETRY_RY_COEFFICIENT = static_cast<double>(walking_data[key]["ry_coefficient"].get<double>());
      }
      catch (nlohmann::json::parse_error &ex)
      {
        std::cerr << "parse error at byte " << ex.byte << std::endl;
      }
    }
    else if (key == "Kinematic")
    {
      try
      {
        THIGH_LENGTH = static_cast<double>(walking_data[key]["thigh_length"].get<double>());
        CALF_LENGTH = static_cast<double>(walking_data[key]["calf_length"].get<double>());
        ANKLE_LENGTH = static_cast<double>(walking_data[key]["ankle_length"].get<double>());
        LEG_LENGTH = static_cast<double>(walking_data[key]["leg_length"].get<double>());
      }
      catch (nlohmann::json::parse_error &ex)
      {
        std::cerr << "parse error at byte " << ex.byte << std::endl;
      }
    }
    else if (key == "InitAngles")
    {
      try
      {
        INIT_R_HIP_YAW = static_cast<double>(walking_data[key]["right_hip_yaw"].get<double>());
        INIT_R_HIP_PITCH = static_cast<double>(walking_data[key]["right_hip_pitch"].get<double>());
        INIT_R_HIP_ROLL = static_cast<double>(walking_data[key]["right_hip_roll"].get<double>());
        INIT_R_KNEE = static_cast<double>(walking_data[key]["right_knee"].get<double>());
        INIT_R_ANKLE_PITCH = static_cast<double>(walking_data[key]["right_ankle_pitch"].get<double>());
        INIT_R_ANKLE_ROLL = static_cast<double>(walking_data[key]["right_ankle_roll"].get<double>());
        INIT_L_HIP_YAW = static_cast<double>(walking_data[key]["left_hip_yaw"].get<double>());
        INIT_L_HIP_PITCH = static_cast<double>(walking_data[key]["left_hip_pitch"].get<double>());
        INIT_L_HIP_ROLL = static_cast<double>(walking_data[key]["left_hip_roll"].get<double>());
        INIT_L_KNEE = static_cast<double>(walking_data[key]["left_knee"].get<double>());
        INIT_L_ANKLE_PITCH = static_cast<double>(walking_data[key]["left_ankle_pitch"].get<double>());
        INIT_L_ANKLE_ROLL = static_cast<double>(walking_data[key]["left_ankle_roll"].get<double>());
        INIT_R_SHOULDER_PITCH = static_cast<double>(walking_data[key]["right_shoulder_pitch"].get<double>());
        INIT_R_SHOULDER_ROLL = static_cast<double>(walking_data[key]["right_shoulder_roll"].get<double>());
        INIT_R_ELBOW = static_cast<double>(walking_data[key]["right_elbow"].get<double>());
        INIT_L_SHOULDER_PITCH = static_cast<double>(walking_data[key]["left_shoulder_pitch"].get<double>());
        INIT_L_SHOULDER_ROLL = static_cast<double>(walking_data[key]["left_shoulder_roll"].get<double>());
        INIT_L_ELBOW = static_cast<double>(walking_data[key]["left_elbow"].get<double>());
      }
      catch (nlohmann::json::parse_error &ex)
      {
        std::cerr << "parse error at byte " << ex.byte << std::endl;
      }
    }
  }
}

void Walking::initialize()
{
  X_MOVE_AMPLITUDE = 0;
  Y_MOVE_AMPLITUDE = 0;
  A_MOVE_AMPLITUDE = 0;

  m_X_Move_Phase_Shift = alg::piValue() / 2;
  m_X_Move_Amplitude_Shift = 0;
  m_Y_Move_Phase_Shift = alg::piValue() / 2;
  m_Z_Move_Phase_Shift = alg::piValue() / 2;
  m_A_Move_Phase_Shift = alg::piValue() / 2;

  m_Ctrl_Running = false;
  m_Real_Running = false;
  m_Time = 0;

  load_data();

  update_param_time();
  update_param_move();

  process();
}

void Walking::start()
{
  m_Ctrl_Running = true;
  m_Real_Running = true;
}

void Walking::stop()
{
  m_Ctrl_Running = false;
}

void Walking::force_stop()
{
  m_Ctrl_Running = false;
  m_Real_Running = false;

  m_X_Move_Amplitude = 0.0;
  m_Y_Move_Amplitude = 0.0;
  m_A_Move_Amplitude = 0.0;
  m_Z_Move_Amplitude = 0.0;

  update_param_time();
  update_param_move();
}

std::vector<tachimawari::Joint> Walking::get_joints()
{
  return joints;
}

void Walking::process()
{
  if (m_Time == 0)
  {
    update_param_time();

    if (m_Ctrl_Running == false)
    {
      bool walk_in_position = true;
      walk_in_position &= (fabs(m_X_Move_Amplitude) <= 5.0);
      walk_in_position &= (fabs(m_Y_Move_Amplitude) <= 5.0);
      walk_in_position &= (fabs(m_A_Move_Amplitude) <= 5.0);

      m_Z_Move_Amplitude = 0;

      if (walk_in_position)
      {
        m_Real_Running = false;
      }
      else
      {
        X_MOVE_AMPLITUDE = 0;
        Y_MOVE_AMPLITUDE = 0;
        A_MOVE_AMPLITUDE = 0;

        dynamic_left_kick_ = 0.0;
        dynamic_right_kick_ = 0.0;
      }
    }
  }
  else if (m_Time >= (m_Phase_Time1 - TIME_UNIT / 2) && m_Time < (m_Phase_Time1 + TIME_UNIT / 2))
  {
    update_param_move();

    if (dynamic_left_kick_ > 5.0)
    {
      m_X_Move_Amplitude = dynamic_left_kick_;
      dynamic_left_kick_ = dynamic_left_kick_ * 0.9;
    }

    compute_odometry();
  }
  else if (m_Time >= (m_Phase_Time2 - TIME_UNIT / 2) && m_Time < (m_Phase_Time2 + TIME_UNIT / 2))
  {
    update_param_time();
    m_Time = m_Phase_Time2;

    if (m_Ctrl_Running == false)
    {
      bool walk_in_position = true;
      walk_in_position &= (fabs(m_X_Move_Amplitude) <= 5.0);
      walk_in_position &= (fabs(m_Y_Move_Amplitude) <= 5.0);
      walk_in_position &= (fabs(m_A_Move_Amplitude) <= 5.0);

      m_Z_Move_Amplitude = 0;

      if (walk_in_position)
      {
        m_Real_Running = false;
      }
      else
      {
        X_MOVE_AMPLITUDE = 0;
        Y_MOVE_AMPLITUDE = 0;
        A_MOVE_AMPLITUDE = 0;

        dynamic_left_kick_ = 0.0;
        dynamic_right_kick_ = 0.0;
      }
    }
  }
  else if (m_Time >= (m_Phase_Time3 - TIME_UNIT / 2) && m_Time < (m_Phase_Time3 + TIME_UNIT / 2))
  {
    update_param_move();

    if (dynamic_right_kick_ > 5.0)
    {
      m_X_Move_Amplitude = dynamic_right_kick_;
      dynamic_right_kick_ = dynamic_right_kick_ * 0.9;
    }

    compute_odometry();
  }

  m_X_Offset = X_OFFSET;
  m_Y_Offset = Y_OFFSET;
  m_Z_Offset = Z_OFFSET;
  m_R_Offset = R_OFFSET * alg::deg2Rad();
  m_P_Offset = P_OFFSET * alg::deg2Rad();
  m_A_Offset = A_OFFSET * alg::deg2Rad();

  // Compute endpoints
  double x_swap = wsin(m_Time, m_X_Swap_PeriodTime, alg::piValue(), m_X_Swap_Amplitude, 0);
  double y_swap = wsin(m_Time, m_Y_Swap_PeriodTime, 0, m_Y_Swap_Amplitude, 0);
  double z_swap = wsin(m_Time, m_Z_Swap_PeriodTime, alg::piValue() * 1.5, m_Z_Swap_Amplitude, 0);
  double a_swap = 0;
  double b_swap = 0;
  double c_swap = 0;

  double x_move_r, y_move_r, z_move_r, a_move_r, b_move_r, c_move_r;
  double x_move_l, y_move_l, z_move_l, a_move_l, b_move_l, c_move_l;
  double pelvis_offset_r, pelvis_offset_l;

  if (m_Time <= m_SSP_Time_Start_L)
  {
    x_move_l = wsin(m_SSP_Time_Start_L, m_X_Move_PeriodTime, m_X_Move_Phase_Shift + 2 * alg::piValue() / m_X_Move_PeriodTime * m_SSP_Time_Start_L, m_X_Move_Amplitude, m_X_Move_Amplitude_Shift);
    y_move_l = wsin(m_SSP_Time_Start_L, m_Y_Move_PeriodTime, m_Y_Move_Phase_Shift + 2 * alg::piValue() / m_Y_Move_PeriodTime * m_SSP_Time_Start_L, m_Y_Move_Amplitude, m_Y_Move_Amplitude_Shift);
    z_move_l = wsin(m_SSP_Time_Start_L, m_Z_Move_PeriodTime, m_Z_Move_Phase_Shift + 2 * alg::piValue() / m_Z_Move_PeriodTime * m_SSP_Time_Start_L, m_Z_Move_Amplitude, m_Z_Move_Amplitude_Shift);
    c_move_l = wsin(m_SSP_Time_Start_L, m_A_Move_PeriodTime, m_A_Move_Phase_Shift + 2 * alg::piValue() / m_A_Move_PeriodTime * m_SSP_Time_Start_L, m_A_Move_Amplitude, m_A_Move_Amplitude_Shift);
    x_move_r = wsin(m_SSP_Time_Start_L, m_X_Move_PeriodTime, m_X_Move_Phase_Shift + 2 * alg::piValue() / m_X_Move_PeriodTime * m_SSP_Time_Start_L, -m_X_Move_Amplitude, -m_X_Move_Amplitude_Shift);
    y_move_r = wsin(m_SSP_Time_Start_L, m_Y_Move_PeriodTime, m_Y_Move_Phase_Shift + 2 * alg::piValue() / m_Y_Move_PeriodTime * m_SSP_Time_Start_L, -m_Y_Move_Amplitude, -m_Y_Move_Amplitude_Shift);
    z_move_r = wsin(m_SSP_Time_Start_R, m_Z_Move_PeriodTime, m_Z_Move_Phase_Shift + 2 * alg::piValue() / m_Z_Move_PeriodTime * m_SSP_Time_Start_R, m_Z_Move_Amplitude, m_Z_Move_Amplitude_Shift);
    c_move_r = wsin(m_SSP_Time_Start_L, m_A_Move_PeriodTime, m_A_Move_Phase_Shift + 2 * alg::piValue() / m_A_Move_PeriodTime * m_SSP_Time_Start_L, -m_A_Move_Amplitude, -m_A_Move_Amplitude_Shift);
    pelvis_offset_l = 0;
    pelvis_offset_r = 0;
  }
  else if (m_Time <= m_SSP_Time_End_L)
  {
    x_move_l = wsin(m_Time, m_X_Move_PeriodTime, m_X_Move_Phase_Shift + 2 * alg::piValue() / m_X_Move_PeriodTime * m_SSP_Time_Start_L, m_X_Move_Amplitude, m_X_Move_Amplitude_Shift);
    y_move_l = wsin(m_Time, m_Y_Move_PeriodTime, m_Y_Move_Phase_Shift + 2 * alg::piValue() / m_Y_Move_PeriodTime * m_SSP_Time_Start_L, m_Y_Move_Amplitude, m_Y_Move_Amplitude_Shift);
    z_move_l = wsin(m_Time, m_Z_Move_PeriodTime, m_Z_Move_Phase_Shift + 2 * alg::piValue() / m_Z_Move_PeriodTime * m_SSP_Time_Start_L, m_Z_Move_Amplitude, m_Z_Move_Amplitude_Shift);
    c_move_l = wsin(m_Time, m_A_Move_PeriodTime, m_A_Move_Phase_Shift + 2 * alg::piValue() / m_A_Move_PeriodTime * m_SSP_Time_Start_L, m_A_Move_Amplitude, m_A_Move_Amplitude_Shift);
    x_move_r = wsin(m_Time, m_X_Move_PeriodTime, m_X_Move_Phase_Shift + 2 * alg::piValue() / m_X_Move_PeriodTime * m_SSP_Time_Start_L, -m_X_Move_Amplitude, -m_X_Move_Amplitude_Shift);
    y_move_r = wsin(m_Time, m_Y_Move_PeriodTime, m_Y_Move_Phase_Shift + 2 * alg::piValue() / m_Y_Move_PeriodTime * m_SSP_Time_Start_L, -m_Y_Move_Amplitude, -m_Y_Move_Amplitude_Shift);
    z_move_r = wsin(m_SSP_Time_Start_R, m_Z_Move_PeriodTime, m_Z_Move_Phase_Shift + 2 * alg::piValue() / m_Z_Move_PeriodTime * m_SSP_Time_Start_R, m_Z_Move_Amplitude, m_Z_Move_Amplitude_Shift);
    c_move_r = wsin(m_Time, m_A_Move_PeriodTime, m_A_Move_Phase_Shift + 2 * alg::piValue() / m_A_Move_PeriodTime * m_SSP_Time_Start_L, -m_A_Move_Amplitude, -m_A_Move_Amplitude_Shift);
    pelvis_offset_l = wsin(m_Time, m_Z_Move_PeriodTime, m_Z_Move_Phase_Shift + 2 * alg::piValue() / m_Z_Move_PeriodTime * m_SSP_Time_Start_L, m_Pelvis_Swing / 2, m_Pelvis_Swing / 2);
    pelvis_offset_r = wsin(m_Time, m_Z_Move_PeriodTime, m_Z_Move_Phase_Shift + 2 * alg::piValue() / m_Z_Move_PeriodTime * m_SSP_Time_Start_L, -m_Pelvis_Offset / 2, -m_Pelvis_Offset / 2);
  }
  else if (m_Time <= m_SSP_Time_Start_R)
  {
    x_move_l = wsin(m_SSP_Time_End_L, m_X_Move_PeriodTime, m_X_Move_Phase_Shift + 2 * alg::piValue() / m_X_Move_PeriodTime * m_SSP_Time_Start_L, m_X_Move_Amplitude, m_X_Move_Amplitude_Shift);
    y_move_l = wsin(m_SSP_Time_End_L, m_Y_Move_PeriodTime, m_Y_Move_Phase_Shift + 2 * alg::piValue() / m_Y_Move_PeriodTime * m_SSP_Time_Start_L, m_Y_Move_Amplitude, m_Y_Move_Amplitude_Shift);
    z_move_l = wsin(m_SSP_Time_End_L, m_Z_Move_PeriodTime, m_Z_Move_Phase_Shift + 2 * alg::piValue() / m_Z_Move_PeriodTime * m_SSP_Time_Start_L, m_Z_Move_Amplitude, m_Z_Move_Amplitude_Shift);
    c_move_l = wsin(m_SSP_Time_End_L, m_A_Move_PeriodTime, m_A_Move_Phase_Shift + 2 * alg::piValue() / m_A_Move_PeriodTime * m_SSP_Time_Start_L, m_A_Move_Amplitude, m_A_Move_Amplitude_Shift);
    x_move_r = wsin(m_SSP_Time_End_L, m_X_Move_PeriodTime, m_X_Move_Phase_Shift + 2 * alg::piValue() / m_X_Move_PeriodTime * m_SSP_Time_Start_L, -m_X_Move_Amplitude, -m_X_Move_Amplitude_Shift);
    y_move_r = wsin(m_SSP_Time_End_L, m_Y_Move_PeriodTime, m_Y_Move_Phase_Shift + 2 * alg::piValue() / m_Y_Move_PeriodTime * m_SSP_Time_Start_L, -m_Y_Move_Amplitude, -m_Y_Move_Amplitude_Shift);
    z_move_r = wsin(m_SSP_Time_Start_R, m_Z_Move_PeriodTime, m_Z_Move_Phase_Shift + 2 * alg::piValue() / m_Z_Move_PeriodTime * m_SSP_Time_Start_R, m_Z_Move_Amplitude, m_Z_Move_Amplitude_Shift);
    c_move_r = wsin(m_SSP_Time_End_L, m_A_Move_PeriodTime, m_A_Move_Phase_Shift + 2 * alg::piValue() / m_A_Move_PeriodTime * m_SSP_Time_Start_L, -m_A_Move_Amplitude, -m_A_Move_Amplitude_Shift);
    pelvis_offset_l = 0;
    pelvis_offset_r = 0;
  }
  else if (m_Time <= m_SSP_Time_End_R)
  {
    x_move_l = wsin(m_Time, m_X_Move_PeriodTime, m_X_Move_Phase_Shift + 2 * alg::piValue() / m_X_Move_PeriodTime * m_SSP_Time_Start_R + alg::piValue(), m_X_Move_Amplitude, m_X_Move_Amplitude_Shift);
    y_move_l = wsin(m_Time, m_Y_Move_PeriodTime, m_Y_Move_Phase_Shift + 2 * alg::piValue() / m_Y_Move_PeriodTime * m_SSP_Time_Start_R + alg::piValue(), m_Y_Move_Amplitude, m_Y_Move_Amplitude_Shift);
    z_move_l = wsin(m_SSP_Time_End_L, m_Z_Move_PeriodTime, m_Z_Move_Phase_Shift + 2 * alg::piValue() / m_Z_Move_PeriodTime * m_SSP_Time_Start_L, m_Z_Move_Amplitude, m_Z_Move_Amplitude_Shift);
    c_move_l = wsin(m_Time, m_A_Move_PeriodTime, m_A_Move_Phase_Shift + 2 * alg::piValue() / m_A_Move_PeriodTime * m_SSP_Time_Start_R + alg::piValue(), m_A_Move_Amplitude, m_A_Move_Amplitude_Shift);
    x_move_r = wsin(m_Time, m_X_Move_PeriodTime, m_X_Move_Phase_Shift + 2 * alg::piValue() / m_X_Move_PeriodTime * m_SSP_Time_Start_R + alg::piValue(), -m_X_Move_Amplitude, -m_X_Move_Amplitude_Shift);
    y_move_r = wsin(m_Time, m_Y_Move_PeriodTime, m_Y_Move_Phase_Shift + 2 * alg::piValue() / m_Y_Move_PeriodTime * m_SSP_Time_Start_R + alg::piValue(), -m_Y_Move_Amplitude, -m_Y_Move_Amplitude_Shift);
    z_move_r = wsin(m_Time, m_Z_Move_PeriodTime, m_Z_Move_Phase_Shift + 2 * alg::piValue() / m_Z_Move_PeriodTime * m_SSP_Time_Start_R, m_Z_Move_Amplitude, m_Z_Move_Amplitude_Shift);
    c_move_r = wsin(m_Time, m_A_Move_PeriodTime, m_A_Move_Phase_Shift + 2 * alg::piValue() / m_A_Move_PeriodTime * m_SSP_Time_Start_R + alg::piValue(), -m_A_Move_Amplitude, -m_A_Move_Amplitude_Shift);
    pelvis_offset_l = wsin(m_Time, m_Z_Move_PeriodTime, m_Z_Move_Phase_Shift + 2 * alg::piValue() / m_Z_Move_PeriodTime * m_SSP_Time_Start_R, m_Pelvis_Offset / 2, m_Pelvis_Offset / 2);
    pelvis_offset_r = wsin(m_Time, m_Z_Move_PeriodTime, m_Z_Move_Phase_Shift + 2 * alg::piValue() / m_Z_Move_PeriodTime * m_SSP_Time_Start_R, -m_Pelvis_Swing / 2, -m_Pelvis_Swing / 2);
  }
  else
  {
    x_move_l = wsin(m_SSP_Time_End_R, m_X_Move_PeriodTime, m_X_Move_Phase_Shift + 2 * alg::piValue() / m_X_Move_PeriodTime * m_SSP_Time_Start_R + alg::piValue(), m_X_Move_Amplitude, m_X_Move_Amplitude_Shift);
    y_move_l = wsin(m_SSP_Time_End_R, m_Y_Move_PeriodTime, m_Y_Move_Phase_Shift + 2 * alg::piValue() / m_Y_Move_PeriodTime * m_SSP_Time_Start_R + alg::piValue(), m_Y_Move_Amplitude, m_Y_Move_Amplitude_Shift);
    z_move_l = wsin(m_SSP_Time_End_L, m_Z_Move_PeriodTime, m_Z_Move_Phase_Shift + 2 * alg::piValue() / m_Z_Move_PeriodTime * m_SSP_Time_Start_L, m_Z_Move_Amplitude, m_Z_Move_Amplitude_Shift);
    c_move_l = wsin(m_SSP_Time_End_R, m_A_Move_PeriodTime, m_A_Move_Phase_Shift + 2 * alg::piValue() / m_A_Move_PeriodTime * m_SSP_Time_Start_R + alg::piValue(), m_A_Move_Amplitude, m_A_Move_Amplitude_Shift);
    x_move_r = wsin(m_SSP_Time_End_R, m_X_Move_PeriodTime, m_X_Move_Phase_Shift + 2 * alg::piValue() / m_X_Move_PeriodTime * m_SSP_Time_Start_R + alg::piValue(), -m_X_Move_Amplitude, -m_X_Move_Amplitude_Shift);
    y_move_r = wsin(m_SSP_Time_End_R, m_Y_Move_PeriodTime, m_Y_Move_Phase_Shift + 2 * alg::piValue() / m_Y_Move_PeriodTime * m_SSP_Time_Start_R + alg::piValue(), -m_Y_Move_Amplitude, -m_Y_Move_Amplitude_Shift);
    z_move_r = wsin(m_SSP_Time_End_R, m_Z_Move_PeriodTime, m_Z_Move_Phase_Shift + 2 * alg::piValue() / m_Z_Move_PeriodTime * m_SSP_Time_Start_R, m_Z_Move_Amplitude, m_Z_Move_Amplitude_Shift);
    c_move_r = wsin(m_SSP_Time_End_R, m_A_Move_PeriodTime, m_A_Move_Phase_Shift + 2 * alg::piValue() / m_A_Move_PeriodTime * m_SSP_Time_Start_R + alg::piValue(), -m_A_Move_Amplitude, -m_A_Move_Amplitude_Shift);
    pelvis_offset_l = 0;
    pelvis_offset_r = 0;
  }

  a_move_l = 0;
  b_move_l = 0;
  a_move_r = 0;
  b_move_r = 0;

  double r_x = x_swap + x_move_r + m_X_Offset;
  double r_y = y_swap + y_move_r - m_Y_Offset / 2;
  double r_z = z_swap + z_move_r + m_Z_Offset;
  double r_a = a_swap + a_move_r - m_R_Offset / 2;
  double r_b = b_swap + b_move_r + m_P_Offset;
  double r_c = c_swap + c_move_r - m_A_Offset / 2;

  double l_x = x_swap + x_move_l + m_X_Offset;
  double l_y = y_swap + y_move_l + m_Y_Offset / 2;
  double l_z = z_swap + z_move_l + m_Z_Offset;
  double l_a = a_swap + a_move_l + m_R_Offset / 2;
  double l_b = b_swap + b_move_l + m_P_Offset;
  double l_c = c_swap + c_move_l + m_A_Offset / 2;

  double angle[22];
  for (int i = 0; i < 22; i++)
  {
    angle[i] = 0;
  }

  if (m_X_Move_Amplitude == 0)
  {
    angle[12] = 0; // Right
    angle[15] = 0; // Left
  }
  else
  {
    angle[12] = wsin(m_Time, m_PeriodTime, alg::piValue() * 1.5, -m_X_Move_Amplitude * m_Arm_Swing_Gain, 0);
    angle[15] = wsin(m_Time, m_PeriodTime, alg::piValue() * 1.5, m_X_Move_Amplitude * m_Arm_Swing_Gain, 0);
  }

  if (m_Real_Running == true)
  {
    m_Time += TIME_UNIT;
    if (m_Time >= m_PeriodTime)
    {
      m_Time = 0;
    }
  }
  else
  {
    m_Time = 0;
  }

  // Compute angles
  if (compute_ik(&angle[0], r_x, r_y, r_z, r_a, r_b, r_c) == false)
  {
    return;
  }

  if (compute_ik(&angle[6], l_x, l_y, l_z, l_a, l_b, l_c) == false)
  {
    return;
  }

  for (int i = 0; i < 12; i++)
  {
    angle[i] *= alg::rad2Deg();
  }

  double initAngle[22];
  initAngle[0] = INIT_R_HIP_YAW;
  initAngle[1] = INIT_R_HIP_ROLL;
  initAngle[2] = INIT_R_HIP_PITCH;
  initAngle[3] = INIT_R_KNEE;
  initAngle[4] = INIT_R_ANKLE_PITCH;
  initAngle[5] = INIT_R_ANKLE_ROLL;
  initAngle[6] = INIT_L_HIP_YAW;
  initAngle[7] = INIT_L_HIP_ROLL;
  initAngle[8] = INIT_L_HIP_PITCH;
  initAngle[9] = INIT_L_KNEE;
  initAngle[10] = INIT_L_ANKLE_PITCH;
  initAngle[11] = INIT_L_ANKLE_ROLL;
  initAngle[12] = INIT_R_SHOULDER_PITCH;
  initAngle[13] = INIT_R_SHOULDER_ROLL;
  initAngle[14] = INIT_R_ELBOW;
  initAngle[15] = INIT_L_SHOULDER_PITCH;
  initAngle[16] = INIT_L_SHOULDER_ROLL;
  initAngle[17] = INIT_L_ELBOW;
  initAngle[18] = 0.0;
  initAngle[19] = 0.0;
  initAngle[20] = INIT_R_GRIPPER;
  initAngle[21] = INIT_L_GRIPPER;

  int dir[22];
  dir[0] = -1;
  dir[1] = -1;
  dir[2] = 1;
  dir[3] = 1;
  dir[4] = -1;
  dir[5] = 1;
  dir[6] = -1;
  dir[7] = -1;
  dir[8] = -1;
  dir[9] = -1;
  dir[10] = 1;
  dir[11] = 1;
  dir[12] = 1;
  dir[13] = 1;
  dir[14] = 1;
  dir[15] = -1;
  dir[16] = -1;
  dir[17] = -1;
  dir[18] = 1;
  dir[19] = 1;
  dir[20] = 1;
  dir[21] = 1;

  int outValue[22];
  for (int i = 0; i < 22; i++)
  {
    double offset = (double)dir[i] * angle[i] * mx.RATIO_ANGLE2VALUE;

    switch (i)
    {
    case 1: // R_HIP_ROLL
      offset += (double)dir[i] * pelvis_offset_r;
      break;

    case 7: // L_HIP_ROLL
      offset += (double)dir[i] * pelvis_offset_l;
      break;

    case 2:
    case 8:
      offset -= (double)dir[i] * (HIP_PITCH_OFFSET + HIP_COMP) * mx.RATIO_ANGLE2VALUE;
    }

    outValue[i] = mx.angle_to_value(initAngle[i]) + (int)offset;
  }

  // adjust balance offset
  if (BALANCE_ENABLE == true)
  {
    double rlGyroErr = 0.0; // MotionStatus::RL_GYRO
    double fbGyroErr = 0.0; // MotionStatus::FB_GYRO

    outValue[1] += (int)(dir[1] * rlGyroErr * BALANCE_HIP_ROLL_GAIN * 4);
    outValue[7] += (int)(dir[7] * rlGyroErr * BALANCE_HIP_ROLL_GAIN * 4);

    outValue[3] -= (int)(dir[3] * fbGyroErr * BALANCE_KNEE_GAIN * 4);
    outValue[9] -= (int)(dir[9] * fbGyroErr * BALANCE_KNEE_GAIN * 4);

    outValue[4] -= (int)(dir[4] * fbGyroErr * BALANCE_ANKLE_PITCH_GAIN * 4);
    outValue[10] -= (int)(dir[10] * fbGyroErr * BALANCE_ANKLE_PITCH_GAIN * 4);

    outValue[5] -= (int)(dir[5] * rlGyroErr * BALANCE_ANKLE_ROLL_GAIN * 4);
    outValue[11] -= (int)(dir[11] * rlGyroErr * BALANCE_ANKLE_ROLL_GAIN * 4);
  }

  for (int id = 0; id < static_cast<int>(joints.size()); id++)
  {
    joints.at(id).set_target_position(outValue[id]);
    joints.at(id).set_pid_gain(P_GAIN, I_GAIN, D_GAIN);
  }
}

} // namespace aruku
