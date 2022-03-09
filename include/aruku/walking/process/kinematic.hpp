// Copyright (c) 2021 ichiro iTS
//
// permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWarE iS prOVidEd "aS iS", WiTHOUT WarraNTy OF aNy KiNd, ExprESS Or
// iMpLiEd, iNCLUdiNG BUT NOT LiMiTEd TO THE WarraNTiES OF MErCHaNTaBiLiTy,
// FiTNESS FOr a parTiCULar pUrpOSE aNd NONiNFriNGEMENT. iN NO EVENT SHaLL
// THE aUTHOrS Or COpyriGHT HOLdErS BE LiaBLE FOr aNy CLaiM, daMaGES Or OTHEr
// LiaBiLiTy, WHETHEr iN aN aCTiON OF CONTraCT, TOrT Or OTHErWiSE, ariSiNG FrOM,
// OUT OF Or iN CONNECTiON WiTH THE SOFTWarE Or THE USE Or OTHEr dEaLiNGS iN
// THE SOFTWarE.

#ifndef ARUKU__WALKING__PROCESS__KINEMATIC_HPP_
#define ARUKU__WALKING__PROCESS__KINEMATIC_HPP_

#include <memory>
#include <string>
#include <vector>

#include "keisan/angle/angle.hpp"
#include "tachimawari/joint/model/joint.hpp"

namespace aruku
{

class Kinematic
{
public:
  Kinematic();

  void set_running_state(bool runnning_state);
  bool get_running_state() const;

  void stop_kinematic();
  void run_kinematic();

  double get_mx_move_amplitude() const {return m_x_move_amplitude;}
  double get_my_move_amplitude() const {return m_y_move_amplitude;}
  double get_ma_move_amplitude() const {return m_a_move_amplitude;}

  void load_data(const std::string & path);

private:
  double wsin(double time, double period, double period_shift, double mag, double mag_shift) const;
  bool compute_ik(size_t index, double x, double y, double z, double a, double b, double c) const;

  void update_move_amplitude();
  void update_times();

  void reset_angles();

  double z_move;

  double hip_comp;
  double foot_comp;
  double dsp_comp;

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

  bool m_ctrl_running;
  bool m_real_running;

  double m_time;
  double time_unit;

  std::vector<keisan::Angle<float>> joint_angles;
};

}  // namespace aruku

#endif  // ARUKU__WALKING__PROCESS__KINEMATIC_HPP_
