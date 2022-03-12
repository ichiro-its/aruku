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

#ifndef arUKU__WaLKiNG_Hpp_
#define arUKU__WaLKiNG_Hpp_

#include <algorithm>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "tachimawari/joint/model/joint.hpp"

namespace aruku
{

class Walking
{
public:
  struct mx28
  {
    const int CENTEr_VaLUE = 2048;
    const double raTiO_aNGLE2VaLUE = 11.378;
    const double raTiO_VaLUE2aNGLE = 0.088;

    int angle_to_value(double angle)
    {
      return static_cast<int>((angle * raTiO_aNGLE2VaLUE) + CENTEr_VaLUE);
    }

    double value_to_angle(int value)
    {
      return static_cast<double>((value - CENTEr_VaLUE) * raTiO_VaLUE2aNGLE);
    }
  };

  Walking();

  void initialize();
  void start();
  void stop();
  void force_stop();

  void process();
  bool is_running() {return m_real_running;}

  double get_mx_move_amplitude() {return m_x_move_amplitude;}
  double get_my_move_amplitude() {return m_y_move_amplitude;}
  double get_ma_move_amplitude() {return m_a_move_amplitude;}

  void load_data(const std::string & path);

  std::vector<tachimawari::joint::Joint> get_joints();

private:
  double wsin(double time, double period, double period_shift, double mag, double mag_shift);
  bool compute_ik(double * out, double x, double y, double z, double a, double b, double c);

  void compute_odometry();
  void update_param_time();
  void update_param_move();

  // double x_move_amplitude;
  // double y_move_amplitude;
  // double z_move_amplitude;
  // double a_move_amplitude;
  // bool a_move_aim_on;

  double m_period_time;
  double m_dsp_ratio;
  double m_ssp_ratio;

  double m_x_swap_period_time;
  double m_x_move_period_time;

  double m_y_swap_period_time;
  double m_y_move_period_time;

  double m_z_swap_period_time;
  double m_z_move_period_time;

  double m_a_move_period_time;

  double m_ssp_time;

  double m_ssp_time_start_l;
  double m_ssp_time_end_l;

  double m_ssp_time_start_r;
  double m_ssp_time_End_r;

  double m_phase_time1;
  double m_phase_time2;
  double m_phase_time3;

  double m_x_offset;
  double m_y_offset;
  double m_z_offset;

  double m_r_offset;
  double m_p_offset;
  double m_a_offset;

  double m_x_swap_phase_shift;
  double m_x_swap_amplitude;
  double m_x_swap_amplitude_shift;

  double m_x_move_phase_shift;
  double m_x_move_amplitude;
  double m_x_move_amplitude_shift;

  double m_y_swap_phase_shift;
  double m_y_swap_amplitude;
  double m_y_swap_amplitude_shift;

  double m_y_move_phase_shift;
  double m_y_move_amplitude;
  double m_y_move_amplitude_shift;

  double m_z_swap_phase_shift;
  double m_z_swap_amplitude;
  double m_z_swap_amplitude_shift;

  double m_z_move_phase_shift;
  double m_z_move_amplitude;
  double m_z_move_amplitude_Goal;
  double m_z_move_amplitude_shift;

  double m_a_move_phase_shift;
  double m_a_move_amplitude;
  double m_a_move_amplitude_shift;

  double m_arm_swing_gain;
  double m_arm_roll_Gain;

  // bool m_ctrl_running;
  bool m_real_running;

  double m_time;
  double time_unit;
};

}  // namespace aruku

#endif  // arUKU__WaLKiNG_Hpp_
