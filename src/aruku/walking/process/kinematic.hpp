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

#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "aruku/walking/process/kinematic.hpp"

#include "keisan/keisan.hpp"
#include "nlohmann/json.hpp"
#include "tachimawari/joint/model/joint.hpp"
#include "tachimawari/joint/model/joint_id.hpp"

using keisan::literals::operator""_pi;

namespace aruku
{

Kinematic::Kinematic()
: m_period_time(0), m_dsp_ratio(0), m_ssp_ratio(0), m_x_swap_period_time(0), m_x_move_period_time(0), m_y_swap_period_time(0), m_y_move_period_time(0), m_z_swap_period_time(0), m_z_move_period_time(0), m_a_move_period_time(0), m_ssp_time(0), m_ssp_time_start_l(0), m_ssp_time_end_l(0), m_ssp_time_start_r(0), m_ssp_time_End_r(0), m_phase_time1(0), m_phase_time2(0), m_phase_time3(0), m_x_swap_phase_shift(0), m_x_swap_amplitude(0), m_x_swap_amplitude_shift(0), m_x_move_amplitude(0), m_y_swap_phase_shift(0), m_y_swap_amplitude(0), m_y_swap_amplitude_shift(0), m_y_move_amplitude(0), m_y_move_amplitude_shift(0), m_z_swap_phase_shift(0), m_z_swap_amplitude(0), m_z_swap_amplitude_shift(0), m_z_move_amplitude(0), m_z_move_amplitude_Goal(0), m_z_move_amplitude_shift(0), m_a_move_amplitude(0), m_a_move_amplitude_shift(0), m_arm_swing_gain(0), m_arm_roll_Gain(0), hip_comp(0.0), foot_comp(0.0), time_unit(8.0), m_x_move_phase_shift(0.5_pi), m_x_move_amplitude_shift(0), m_y_move_phase_shift(0.5_pi), m_z_move_phase_shift(0.5_pi), m_a_move_phase_shift(0.5_pi), m_ctrl_running(false), m_real_running(false), m_time(0), x_move(0.0), y_move(0.0), a_move(0.0), a_move_aim_on(false), is_compute_odometry(false)
{
  reset_angles();
}

void Kinematic::reset_angles()
{
  for (auto & joint : joint_angles) {
    joint = keisan::make_degree(0.0);
  }
}

void Kinematic::set_running_state(bool running_state)
{
  m_ctrl_running = running_state;

  if (running_state) {
    m_real_running = running_state;
  }
}

bool Kinematic::get_running_state() const
{
  return m_ctrl_running;
}

double Kinematic::get_x_move_amplitude() const
{
  return m_x_move_amplitude;
}

double Kinematic::get_y_move_amplitude() const
{
  return m_y_move_amplitude;
}

double Kinematic::get_hip_offest() const
{
  return hip_pitch_offset + hip_comp;
}

bool Kinematic::is_time_compute_odometry() const
{
  return is_compute_odometry;
}

void Kinematic::stop_kinematic()
{
  m_ctrl_running = false;
  m_real_running = false;

  m_x_move_amplitude = 0.0;
  m_y_move_amplitude = 0.0;
  m_a_move_amplitude = 0.0;
  m_z_move_amplitude = 0.0;

  update_times();
  update_move_amplitude();
}

double Kinematic::wsin(double time, double period, double period_shift, double mag, double mag_shift) const
{
  return mag * sin(2_pi / period * time - period_shift) + mag_shift;
}

bool Kinematic::compute_inverse_kinematic(size_t index, double x, double y, double z, double a, double b, double c)
{
  {
    using keisan::Point3;
    using keisan::Euler;
    using keisan::Matrix;

    auto matrix_translation = keisan::translation_matrix(Point3(x, y, z - leg_length));
    auto matrix_rotation = keisan::rotation_matrix(Euler(keisan::make_radian(a),
      keisan::make_radian(b), keisan::make_radian(c)));

    auto matrix_transformation = matrix_translation * matrix_rotation;

    auto vector = Point3(
      x + (matrix_transformation[0][2] * ankle_length),
      y + (matrix_transformation[1][2] * ankle_length),
      (z - leg_length) + (matrix_transformation[2][2] * ankle_length));

    // get knee
    double vector_magnitude = vector.magnitude();
    double acos_result = acos(
      (vector_magnitude * vector_magnitude - thigh_length * thigh_length - calf_length * calf_length) /
      (2 * thigh_length * calf_length));

    if (std::isnan(acos_result)) {
      return false;
    }
    joint_angles[index + 3] = keisan::make_radian(acos_result);

    // get ankle roll
    auto matrix = matrix_transformation;
    if (!matrix.inverse()) {
      return false;
    }

    double k = sqrt(matrix[1][3] * matrix[1][3] + matrix[2][3] * matrix[2][3]);
    double l = sqrt(matrix[1][3] * matrix[1][3] + (matrix[2][3] - ankle_length) * (matrix[2][3] - ankle_length));
    double m = (k * k - l * l - ankle_length * ankle_length) / (2 * l * ankle_length);
    if (m > 1.0) {
      m = 1.0;
    } else if (m < -1.0) {
      m = -1.0;
    }

    acos_result = acos(m);
    if (std::isnan(acos_result)) {
      return false;
    }
    if (matrix[1][3] < 0.0) {
      joint_angles[index + 5] = keisan::make_radian(-acos_result);
    } else {
      joint_angles[index + 5] = keisan::make_radian(acos_result);
    }

    // get hip yaw
    matrix_translation = keisan::translation_matrix(Point3(0, 0, ankle_length));
    matrix_rotation = keisan::rotation_matrix(Euler(joint_angles[index + 5], keisan::make_radian(0.0), keisan::make_radian(0.0)));

    matrix = matrix_translation * matrix_rotation;
    if (!matrix.inverse()) {
      return false;
    }

    matrix = matrix_transformation * matrix;
    double atan_result = atan2(-matrix[0][1], matrix[1][1]);
    if (std::isinf(atan_result)) {
      return false;
    }
    joint_angles[index] = keisan::make_radian(atan_result);

    // get hip roll
    atan_result = atan2(matrix[2][1], -matrix[0][1] * sin(joint_angles[index].radian()) + matrix[1][1] * cos(joint_angles[index].radian()));
    if (std::isinf(atan_result)) {
      return false;
    }
    joint_angles[index + 1] = keisan::make_radian(atan_result);

    // get hip pitch and ankle pitch
    atan_result = atan2(matrix[0][2] * cos(joint_angles[index].radian()) + matrix[1][2] * sin(joint_angles[index].radian()), matrix[0][0] *
      cos(joint_angles[index].radian()) + matrix[1][0] * sin(joint_angles[index].radian()));
    if (std::isinf(atan_result)) {
      return false;
    }

    k = sin(joint_angles[index + 3].radian()) * calf_length;
    l = -thigh_length - cos(joint_angles[index + 3].radian()) * calf_length;
    m = cos(joint_angles[index].radian()) * vector.x + sin(joint_angles[index].radian()) * vector.y;

    double n = cos(joint_angles[index + 1].radian()) * vector.z + sin(joint_angles[index].radian()) * sin(joint_angles[index + 1].radian()) * vector.x - cos(joint_angles[index].radian()) *
      sin(joint_angles[index + 1].radian()) * vector.y;
    double s = (k * n + l * m) / (k * k + l * l);
    double c = (n - k * s) / l;

    double theta_result = atan_result;
    atan_result = atan2(s, c);
    if (std::isinf(atan_result)) {
      return false;
    }
    joint_angles[index + 2] = keisan::make_radian(atan_result);
    joint_angles[index + 4] = keisan::make_radian(theta_result) - joint_angles[index + 3] - joint_angles[index + 2];

    return true;
  }
}

void Kinematic::update_times()
{
  double dsp_comp = fabs(m_x_move_amplitude) * dsp_comp_ratio * 0.001;

  m_period_time = period_time - (fabs(m_x_move_amplitude) * period_comp_ratio);

  m_dsp_ratio = dsp_ratio + dsp_comp;
  m_ssp_ratio = 1 - m_dsp_ratio;

  m_x_swap_period_time = m_period_time / 2;
  m_x_move_period_time = m_period_time * m_ssp_ratio;
  m_y_swap_period_time = m_period_time;
  m_y_move_period_time = m_period_time * m_ssp_ratio;
  m_z_swap_period_time = m_period_time / 2;
  m_z_move_period_time = m_period_time * m_ssp_ratio / 2;
  m_a_move_period_time = m_period_time * m_ssp_ratio;

  m_ssp_time = m_period_time * m_ssp_ratio;
  m_ssp_time_start_l = (1 - m_ssp_ratio) * m_period_time / 4;
  m_ssp_time_end_l = (1 + m_ssp_ratio) * m_period_time / 4;
  m_ssp_time_start_r = (3 - m_ssp_ratio) * m_period_time / 4;
  m_ssp_time_End_r = (3 + m_ssp_ratio) * m_period_time / 4;

  m_phase_time1 = (m_ssp_time_start_l + m_ssp_time_end_l) / 2;
  m_phase_time2 = (m_ssp_time_end_l + m_ssp_time_start_r) / 2;
  m_phase_time3 = (m_ssp_time_start_r + m_ssp_time_End_r) / 2;

  m_arm_swing_gain = arm_swing_gain;

  if (m_x_move_amplitude > 0) {
    hip_comp = m_x_move_amplitude * forward_hip_comp_ratio;
  } else {
    hip_comp = m_x_move_amplitude * backward_hip_comp_ratio;
  }

  foot_comp = fabs(m_x_move_amplitude) * foot_comp_ratio;
}

void Kinematic::update_move_amplitude()
{
  double x_input = x_move;
  double y_input = y_move * 0.5;
  double a_input = a_move;

  if (m_z_move_amplitude < (z_move * 0.45)) {
    x_input = 0.0;
    y_input = 0.0;
    a_input = 0.0;
  }

  m_x_move_amplitude = keisan::smooth(m_x_move_amplitude, x_input, move_accel_ratio);
  m_y_move_amplitude = keisan::smooth(m_y_move_amplitude, y_input, move_accel_ratio);
  m_y_move_amplitude_shift = fabs(m_y_move_amplitude);
  m_y_swap_amplitude = y_swap_amplitude + m_y_move_amplitude_shift * 0.04;

  if (m_ctrl_running) {
    m_z_move_amplitude = keisan::smooth(
      m_z_move_amplitude, (z_move + foot_comp) * 0.5, foot_accel_ratio);
  } else {
    m_z_move_amplitude = 0.0;
  }

  m_z_move_amplitude_shift = m_z_move_amplitude;
  m_z_swap_amplitude = z_swap_amplitude;

  if (!a_move_aim_on) {
    m_a_move_amplitude = keisan::smooth(
      m_a_move_amplitude, keisan::make_degree(a_input).radian() / 2, move_accel_ratio);
    m_a_move_amplitude_shift = fabs(m_a_move_amplitude);
  } else {
    m_a_move_amplitude = keisan::smooth(
      m_a_move_amplitude, keisan::make_degree(-a_input).radian() / 2, move_accel_ratio);
    m_a_move_amplitude_shift = -fabs(m_a_move_amplitude);
  }
}

void Kinematic::load_data(const std::string & path)
{
  std::string file_name =
    path + "walking/" + "aruku.json";
  std::ifstream file(file_name);
  nlohmann::json walking_data = nlohmann::json::parse(file);

  for (auto &[key, val] : walking_data.items()) {
    if (key == "Ratio") {
      try {
        val.at("period_time").get_to(period_time);
        val.at("dsp_ratio").get_to(dsp_ratio);
        val.at("foot_height").get_to(z_move);
        val.at("swing_right_left").get_to(y_swap_amplitude);
        val.at("swing_up_down").get_to(z_swap_amplitude);
        val.at("arm_swing_gain").get_to(arm_swing_gain);
        val.at("backward_hip_comp_ratio").get_to(backward_hip_comp_ratio);
        val.at("forward_hip_comp_ratio").get_to(forward_hip_comp_ratio);
        val.at("foot_comp_ratio").get_to(foot_comp_ratio);
        val.at("dsp_comp_ratio").get_to(dsp_comp_ratio);
        val.at("period_comp_ratio").get_to(period_comp_ratio);
        val.at("move_accel_ratio").get_to(move_accel_ratio);
        val.at("foot_accel_ratio").get_to(foot_accel_ratio);
      } catch (nlohmann::json::parse_error & ex) {
        std::cerr << "parse error at byte " << ex.byte << std::endl;
      }
    } else if (key == "Kinematic") {
      try {
        val.at("thigh_length").get_to(thigh_length);
        val.at("calf_length").get_to(calf_length);
        val.at("ankle_length").get_to(ankle_length);
        val.at("leg_length").get_to(leg_length);
      } catch (nlohmann::json::parse_error & ex) {
        std::cerr << "parse error at byte " << ex.byte << std::endl;
      }
    } else if (key == "Offset") {
      try {
        val.at("x_offset").get_to(x_offset);
        val.at("y_offset").get_to(y_offset);
        val.at("z_offset").get_to(z_offset);
        val.at("yaw_offset").get_to(yaw_offset);
        val.at("roll_offset").get_to(roll_offset);
        val.at("pitch_offset").get_to(pitch_offset);
        val.at("hip_pitch_offset").get_to(hip_pitch_offset);
      } catch (nlohmann::json::parse_error & ex) {
        std::cerr << "parse error at byte " << ex.byte << std::endl;
      }
    }
  }

  update_times();
  update_move_amplitude();

  run_kinematic();
}

void Kinematic::run_kinematic()
{
  is_compute_odometry = false;

  if (m_time == 0) {
    update_move_amplitude();
    update_times();

    if (!m_ctrl_running) {
      bool walk_in_position = true;
      walk_in_position &= (fabs(m_x_move_amplitude) <= 5.0);
      walk_in_position &= (fabs(m_y_move_amplitude) <= 5.0);
      walk_in_position &= (fabs(m_a_move_amplitude) <= 5.0);

      m_z_move_amplitude = 0;

      if (walk_in_position) {
        m_real_running = false;
      } else {
        x_move = 0;
        y_move = 0;
        a_move = 0;
      }
    }
  } else if (m_time >= (m_phase_time1 - time_unit / 2) &&  // NOLINT
    m_time < (m_phase_time1 + time_unit / 2))
  {
    // left leg
    update_move_amplitude();
    is_compute_odometry = true;
  } else if (m_time >= (m_phase_time2 - time_unit / 2) &&  // NOLINT
    m_time < (m_phase_time2 + time_unit / 2))
  {
    update_move_amplitude();
    update_times();

    m_time = m_phase_time2;

    if (!m_ctrl_running) {
      bool walk_in_position = true;
      walk_in_position &= (fabs(m_x_move_amplitude) <= 5.0);
      walk_in_position &= (fabs(m_y_move_amplitude) <= 5.0);
      walk_in_position &= (fabs(m_a_move_amplitude) <= 5.0);

      m_z_move_amplitude = 0;

      if (walk_in_position) {
        m_real_running = false;
      } else {
        x_move = 0;
        y_move = 0;
        a_move = 0;
      }
    }
  } else if (m_time >= (m_phase_time3 - time_unit / 2) &&  // NOLINT
    m_time < (m_phase_time3 + time_unit / 2))
  {
    // right leg
    update_move_amplitude();
    is_compute_odometry = true;
  }

  // compute endpoints
  double x_swap = 0;
  double y_swap = wsin(m_time, m_y_swap_period_time, 0, m_y_swap_amplitude, 0);
  double z_swap = wsin(m_time, m_z_swap_period_time, 1.5_pi, m_z_swap_amplitude, 0);
  double a_swap = 0;
  double b_swap = 0;
  double c_swap = 0;

  double x_move_r, y_move_r, z_move_r, a_move_r, b_move_r, c_move_r;
  double x_move_l, y_move_l, z_move_l, a_move_l, b_move_l, c_move_l;

  a_move_l = 0;
  b_move_l = 0;
  a_move_r = 0;
  b_move_r = 0;

  if (m_time <= m_ssp_time_start_l) {
    x_move_l = wsin(m_ssp_time_start_l, m_x_move_period_time, m_x_move_phase_shift + 2_pi / m_x_move_period_time * m_ssp_time_start_l, m_x_move_amplitude, m_x_move_amplitude_shift);
    y_move_l = wsin(m_ssp_time_start_l, m_y_move_period_time, m_y_move_phase_shift + 2_pi / m_y_move_period_time * m_ssp_time_start_l, m_y_move_amplitude, m_y_move_amplitude_shift);
    z_move_l = wsin(m_ssp_time_start_l, m_z_move_period_time, m_z_move_phase_shift + 2_pi / m_z_move_period_time * m_ssp_time_start_l, m_z_move_amplitude, m_z_move_amplitude_shift);
    c_move_l = wsin(m_ssp_time_start_l, m_a_move_period_time, m_a_move_phase_shift + 2_pi / m_a_move_period_time * m_ssp_time_start_l, m_a_move_amplitude, m_a_move_amplitude_shift);
    x_move_r = wsin(m_ssp_time_start_l, m_x_move_period_time, m_x_move_phase_shift + 2_pi / m_x_move_period_time * m_ssp_time_start_l, -m_x_move_amplitude, -m_x_move_amplitude_shift);
    y_move_r = wsin(m_ssp_time_start_l, m_y_move_period_time, m_y_move_phase_shift + 2_pi / m_y_move_period_time * m_ssp_time_start_l, -m_y_move_amplitude, -m_y_move_amplitude_shift);
    z_move_r = wsin(m_ssp_time_start_r, m_z_move_period_time, m_z_move_phase_shift + 2_pi / m_z_move_period_time * m_ssp_time_start_r, m_z_move_amplitude, m_z_move_amplitude_shift);
    c_move_r = wsin(m_ssp_time_start_l, m_a_move_period_time, m_a_move_phase_shift + 2_pi / m_a_move_period_time * m_ssp_time_start_l, -m_a_move_amplitude, -m_a_move_amplitude_shift);
  } else if (m_time <= m_ssp_time_end_l) {
    x_move_l = wsin(m_time, m_x_move_period_time, m_x_move_phase_shift + 2_pi / m_x_move_period_time * m_ssp_time_start_l, m_x_move_amplitude, m_x_move_amplitude_shift);
    y_move_l = wsin(m_time, m_y_move_period_time, m_y_move_phase_shift + 2_pi / m_y_move_period_time * m_ssp_time_start_l, m_y_move_amplitude, m_y_move_amplitude_shift);
    z_move_l = wsin(m_time, m_z_move_period_time, m_z_move_phase_shift + 2_pi / m_z_move_period_time * m_ssp_time_start_l, m_z_move_amplitude, m_z_move_amplitude_shift);
    c_move_l = wsin(m_time, m_a_move_period_time, m_a_move_phase_shift + 2_pi / m_a_move_period_time * m_ssp_time_start_l, m_a_move_amplitude, m_a_move_amplitude_shift);
    x_move_r = wsin(m_time, m_x_move_period_time, m_x_move_phase_shift + 2_pi / m_x_move_period_time * m_ssp_time_start_l, -m_x_move_amplitude, -m_x_move_amplitude_shift);
    y_move_r = wsin(m_time, m_y_move_period_time, m_y_move_phase_shift + 2_pi / m_y_move_period_time * m_ssp_time_start_l, -m_y_move_amplitude, -m_y_move_amplitude_shift);
    z_move_r = wsin(m_ssp_time_start_r, m_z_move_period_time, m_z_move_phase_shift + 2_pi / m_z_move_period_time * m_ssp_time_start_r, m_z_move_amplitude, m_z_move_amplitude_shift);
    c_move_r = wsin(m_time, m_a_move_period_time, m_a_move_phase_shift + 2_pi / m_a_move_period_time * m_ssp_time_start_l, -m_a_move_amplitude, -m_a_move_amplitude_shift);
  } else if (m_time <= m_ssp_time_start_r) {
    x_move_l = wsin(m_ssp_time_end_l, m_x_move_period_time, m_x_move_phase_shift + 2_pi / m_x_move_period_time * m_ssp_time_start_l, m_x_move_amplitude, m_x_move_amplitude_shift);
    y_move_l = wsin(m_ssp_time_end_l, m_y_move_period_time, m_y_move_phase_shift + 2_pi / m_y_move_period_time * m_ssp_time_start_l, m_y_move_amplitude, m_y_move_amplitude_shift);
    z_move_l = wsin(m_ssp_time_end_l, m_z_move_period_time, m_z_move_phase_shift + 2_pi / m_z_move_period_time * m_ssp_time_start_l, m_z_move_amplitude, m_z_move_amplitude_shift);
    c_move_l = wsin(m_ssp_time_end_l, m_a_move_period_time, m_a_move_phase_shift + 2_pi / m_a_move_period_time * m_ssp_time_start_l, m_a_move_amplitude, m_a_move_amplitude_shift);
    x_move_r = wsin(m_ssp_time_end_l, m_x_move_period_time, m_x_move_phase_shift + 2_pi / m_x_move_period_time * m_ssp_time_start_l, -m_x_move_amplitude, -m_x_move_amplitude_shift);
    y_move_r = wsin(m_ssp_time_end_l, m_y_move_period_time, m_y_move_phase_shift + 2_pi / m_y_move_period_time * m_ssp_time_start_l, -m_y_move_amplitude, -m_y_move_amplitude_shift);
    z_move_r = wsin(m_ssp_time_start_r, m_z_move_period_time, m_z_move_phase_shift + 2_pi / m_z_move_period_time * m_ssp_time_start_r, m_z_move_amplitude, m_z_move_amplitude_shift);
    c_move_r = wsin(m_ssp_time_end_l, m_a_move_period_time, m_a_move_phase_shift + 2_pi / m_a_move_period_time * m_ssp_time_start_l, -m_a_move_amplitude, -m_a_move_amplitude_shift);
  } else if (m_time <= m_ssp_time_End_r) {
    x_move_l = wsin(m_time, m_x_move_period_time, m_x_move_phase_shift + 2_pi / m_x_move_period_time * m_ssp_time_start_r + 1_pi, m_x_move_amplitude, m_x_move_amplitude_shift);
    y_move_l = wsin(m_time, m_y_move_period_time, m_y_move_phase_shift + 2_pi / m_y_move_period_time * m_ssp_time_start_r + 1_pi, m_y_move_amplitude, m_y_move_amplitude_shift);
    z_move_l = wsin(m_ssp_time_end_l, m_z_move_period_time, m_z_move_phase_shift + 2_pi / m_z_move_period_time * m_ssp_time_start_l, m_z_move_amplitude, m_z_move_amplitude_shift);
    c_move_l = wsin(m_time, m_a_move_period_time, m_a_move_phase_shift + 2_pi / m_a_move_period_time * m_ssp_time_start_r + 1_pi, m_a_move_amplitude, m_a_move_amplitude_shift);
    x_move_r = wsin(m_time, m_x_move_period_time, m_x_move_phase_shift + 2_pi / m_x_move_period_time * m_ssp_time_start_r + 1_pi, -m_x_move_amplitude, -m_x_move_amplitude_shift);
    y_move_r = wsin(m_time, m_y_move_period_time, m_y_move_phase_shift + 2_pi / m_y_move_period_time * m_ssp_time_start_r + 1_pi, -m_y_move_amplitude, -m_y_move_amplitude_shift);
    z_move_r = wsin(m_time, m_z_move_period_time, m_z_move_phase_shift + 2_pi / m_z_move_period_time * m_ssp_time_start_r, m_z_move_amplitude, m_z_move_amplitude_shift);
    c_move_r = wsin(m_time, m_a_move_period_time, m_a_move_phase_shift + 2_pi / m_a_move_period_time * m_ssp_time_start_r + 1_pi, -m_a_move_amplitude, -m_a_move_amplitude_shift);
  } else {
    x_move_l = wsin(m_ssp_time_End_r, m_x_move_period_time, m_x_move_phase_shift + 2_pi / m_x_move_period_time * m_ssp_time_start_r + 1_pi, m_x_move_amplitude, m_x_move_amplitude_shift);
    y_move_l = wsin(m_ssp_time_End_r, m_y_move_period_time, m_y_move_phase_shift + 2_pi / m_y_move_period_time * m_ssp_time_start_r + 1_pi, m_y_move_amplitude, m_y_move_amplitude_shift);
    z_move_l = wsin(m_ssp_time_end_l, m_z_move_period_time, m_z_move_phase_shift + 2_pi / m_z_move_period_time * m_ssp_time_start_l, m_z_move_amplitude, m_z_move_amplitude_shift);
    c_move_l = wsin(m_ssp_time_End_r, m_a_move_period_time, m_a_move_phase_shift + 2_pi / m_a_move_period_time * m_ssp_time_start_r + 1_pi, m_a_move_amplitude, m_a_move_amplitude_shift);
    x_move_r = wsin(m_ssp_time_End_r, m_x_move_period_time, m_x_move_phase_shift + 2_pi / m_x_move_period_time * m_ssp_time_start_r + 1_pi, -m_x_move_amplitude, -m_x_move_amplitude_shift);
    y_move_r = wsin(m_ssp_time_End_r, m_y_move_period_time, m_y_move_phase_shift + 2_pi / m_y_move_period_time * m_ssp_time_start_r + 1_pi, -m_y_move_amplitude, -m_y_move_amplitude_shift);
    z_move_r = wsin(m_ssp_time_End_r, m_z_move_period_time, m_z_move_phase_shift + 2_pi / m_z_move_period_time * m_ssp_time_start_r, m_z_move_amplitude, m_z_move_amplitude_shift);
    c_move_r = wsin(m_ssp_time_End_r, m_a_move_period_time, m_a_move_phase_shift + 2_pi / m_a_move_period_time * m_ssp_time_start_r + 1_pi, -m_a_move_amplitude, -m_a_move_amplitude_shift);
  }

  double angle[22];
  for (int i = 0; i < 22; i++) {
    angle[i] = 0;
  }

  if (m_x_move_amplitude != 0) {
    angle[12] = wsin(
      m_time, m_period_time, 1.5_pi, -m_x_move_amplitude *
      m_arm_swing_gain, 0);
    angle[15] = wsin(
      m_time, m_period_time, 1.5_pi, m_x_move_amplitude *
      m_arm_swing_gain, 0);
  }

  if (m_real_running) {
    m_time += time_unit;

    if (m_time >= m_period_time) {
      m_time = 0;
    }
  } else {
    m_time = 0;
  }

  double r_x = x_swap + x_move_r + x_offset;
  double r_y = y_swap + y_move_r - y_offset / 2;
  double r_z = z_swap + z_move_r + z_offset;
  double r_a = a_swap + a_move_r - keisan::make_degree(roll_offset).radian() / 2;
  double r_b = b_swap + b_move_r + keisan::make_degree(pitch_offset).radian();
  double r_c = c_swap + c_move_r - keisan::make_degree(yaw_offset).radian() / 2;

  // compute angles
  if (!compute_inverse_kinematic(0, r_x, r_y, r_z, r_a, r_b, r_c)) {
    return;
  }

  double l_x = x_swap + x_move_l + x_offset;
  double l_y = y_swap + y_move_l + y_offset / 2;
  double l_z = z_swap + z_move_l + z_offset;
  double l_a = a_swap + a_move_l + keisan::make_degree(roll_offset).radian() / 2;
  double l_b = b_swap + b_move_l + keisan::make_degree(pitch_offset).radian();
  double l_c = c_swap + c_move_l + keisan::make_degree(yaw_offset).radian() / 2;

  if (!compute_inverse_kinematic(6, l_x, l_y, l_z, l_a, l_b, l_c)) {
    return;
  }
}

}  // namespace aruku
