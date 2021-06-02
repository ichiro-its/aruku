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

#ifndef ARUKU__WALKING_HPP_
#define ARUKU__WALKING_HPP_

#include <tachimawari/joint.hpp>

#include "common/algebra.h"

#include <map>
#include <string>
#include <vector>
#include <memory>

namespace aruku
{

class Walking
{
public:
  struct mx28
  {
    const int CENTER_VALUE = 2048;
    const double RATIO_ANGLE2VALUE = 11.378;

    int angle_to_value(double angle)
    {
      return static_cast<int>((angle * RATIO_ANGLE2VALUE) + CENTER_VALUE);
    }
  };

  explicit Walking();

  void initialize();
  void start();
  void stop();
  void force_stop();

  void process();
  bool is_running() { return m_Real_Running; }

  double get_mx_move_amplitude() { return m_X_Move_Amplitude; }
  double get_my_move_amplitude() { return m_Y_Move_Amplitude; }
  double get_ma_move_amplitude() { return m_A_Move_Amplitude; }

  void set_dynamic_left_kick(double speed) { dynamic_left_kick_ = speed; }
  void set_dynamic_right_kick(double speed) { dynamic_right_kick_ = speed; }
  double set_dynamic_kick() { return alg::maxValue(dynamic_left_kick_, dynamic_right_kick_); }

  void load_data();
  void update_orientation(double orientation);

  std::vector<tachimawari::Joint> get_joints();

  mx28 mx;

  double X_OFFSET;
  double Y_OFFSET;
  double Z_OFFSET;
  double A_OFFSET;
  double P_OFFSET;
  double R_OFFSET;

  double PERIOD_TIME;
  double DSP_RATIO;
  double STEP_FB_RATIO;
  double X_MOVE_AMPLITUDE;
  double Y_MOVE_AMPLITUDE;
  double Z_MOVE_AMPLITUDE;
  double A_MOVE_AMPLITUDE;
  bool A_MOVE_AIM_ON;

  bool BALANCE_ENABLE;
  double BALANCE_KNEE_GAIN;
  double BALANCE_ANKLE_PITCH_GAIN;
  double BALANCE_HIP_ROLL_GAIN;
  double BALANCE_ANKLE_ROLL_GAIN;

  double Y_SWAP_AMPLITUDE;
  double Z_SWAP_AMPLITUDE;
  double ARM_SWING_GAIN;
  double PELVIS_OFFSET;
  double HIP_PITCH_OFFSET;

  double INIT_R_HIP_YAW;
  double INIT_R_HIP_ROLL;
  double INIT_R_HIP_PITCH;
  double INIT_R_KNEE;
  double INIT_R_ANKLE_PITCH;
  double INIT_R_ANKLE_ROLL;
  double INIT_R_SHOULDER_PITCH;
  double INIT_R_SHOULDER_ROLL;
  double INIT_R_ELBOW;
  double INIT_R_GRIPPER;

  double INIT_L_HIP_YAW;
  double INIT_L_HIP_ROLL;
  double INIT_L_HIP_PITCH;
  double INIT_L_KNEE;
  double INIT_L_ANKLE_PITCH;
  double INIT_L_ANKLE_ROLL;
  double INIT_L_SHOULDER_PITCH;
  double INIT_L_SHOULDER_ROLL;
  double INIT_L_ELBOW;
  double INIT_L_GRIPPER;

  double THIGH_LENGTH;
  double CALF_LENGTH;
  double ANKLE_LENGTH;
  double LEG_LENGTH;

  int P_GAIN;
  int I_GAIN;
  int D_GAIN;

  double POSITION_X;
  double POSITION_Y;
  double ORIENTATION;
  double ODOMETRY_FX_COEFFICIENT;
  double ODOMETRY_LY_COEFFICIENT;
  double ODOMETRY_RY_COEFFICIENT;

  double HIP_COMP;
  double FOOT_COMP;
  double DSP_COMP;

  double DSP_COMP_RATIO;
  double HIP_COMP_RATIO;
  double PERIOD_COMP_RATIO;
  double BACKWARD_HIP_COMP_RATIO;
  double FORWARD_HIP_COMP_RATIO;

  double FOOT_COMP_RATIO;
  double MOVE_ACCEL_RATIO;
  double FOOT_ACCEL_RATIO;

private:
  double wsin(double time, double period, double period_shift, double mag, double mag_shift);
  bool compute_ik(double *out, double x, double y, double z, double a, double b, double c);

  void compute_odometry();
  void update_param_time();
  void update_param_move();
  void update_param_balance();

  double m_PeriodTime;
  double m_DSP_Ratio;
  double m_SSP_Ratio;

  double m_X_Swap_PeriodTime;
  double m_X_Move_PeriodTime;

  double m_Y_Swap_PeriodTime;
  double m_Y_Move_PeriodTime;

  double m_Z_Swap_PeriodTime;
  double m_Z_Move_PeriodTime;

  double m_A_Move_PeriodTime;

  double m_SSP_Time;

  double m_SSP_Time_Start_L;
  double m_SSP_Time_End_L;

  double m_SSP_Time_Start_R;
  double m_SSP_Time_End_R;

  double m_Phase_Time1;
  double m_Phase_Time2;
  double m_Phase_Time3;

  double m_X_Offset;
  double m_Y_Offset;
  double m_Z_Offset;

  double m_R_Offset;
  double m_P_Offset;
  double m_A_Offset;

  double m_X_Swap_Phase_Shift;
  double m_X_Swap_Amplitude;
  double m_X_Swap_Amplitude_Shift;

  double m_X_Move_Phase_Shift;
  double m_X_Move_Amplitude;
  double m_X_Move_Amplitude_Shift;

  double m_Y_Swap_Phase_Shift;
  double m_Y_Swap_Amplitude;
  double m_Y_Swap_Amplitude_Shift;

  double m_Y_Move_Phase_Shift;
  double m_Y_Move_Amplitude;
  double m_Y_Move_Amplitude_Shift;

  double m_Z_Swap_Phase_Shift;
  double m_Z_Swap_Amplitude;
  double m_Z_Swap_Amplitude_Shift;

  double m_Z_Move_Phase_Shift;
  double m_Z_Move_Amplitude;
  double m_Z_Move_Amplitude_Goal;
  double m_Z_Move_Amplitude_Shift;

  double m_A_Move_Phase_Shift;
  double m_A_Move_Amplitude;
  double m_A_Move_Amplitude_Shift;

  double m_Pelvis_Offset;
  double m_Pelvis_Swing;

  double m_Arm_Swing_Gain;
  double m_Arm_Roll_Gain;

  double dynamic_left_kick_;
  double dynamic_right_kick_;

  bool m_Ctrl_Running;
  bool m_Real_Running;

  double m_Time;
  double TIME_UNIT;

  std::vector<tachimawari::Joint> joints;
};

} // namespace aruku

#endif // ARUKU__WALKING_HPP_
