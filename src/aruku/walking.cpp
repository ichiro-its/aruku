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

#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "aruku/walking.hpp"

#include "keisan/keisan.hpp"
#include "nlohmann/json.hpp"
#include "tachimawari/joint/model/joint.hpp"
#include "tachimawari/joint/model/joint_id.hpp"

using keisan::literals::operator""_pi;

namespace aruku
{

Walking::Walking()
{
  m_period_time = 0;
  m_dsp_ratio = 0;
  m_ssp_ratio = 0;
  m_x_swap_period_time = 0;
  m_x_move_period_time = 0;
  m_y_swap_period_time = 0;
  m_y_move_period_time = 0;
  m_z_swap_period_time = 0;
  m_z_move_period_time = 0;
  m_a_move_period_time = 0;
  m_ssp_time = 0;
  m_ssp_time_start_l = 0;
  m_ssp_time_end_l = 0;
  m_ssp_time_start_r = 0;
  m_ssp_time_End_r = 0;
  m_phase_time1 = 0;
  m_phase_time2 = 0;
  m_phase_time3 = 0;

  m_x_offset = 0;
  m_y_offset = 0;
  m_z_offset = 0;
  m_r_offset = 0;
  m_p_offset = 0;
  m_a_offset = 0;

  m_x_swap_phase_shift = 0;
  m_x_swap_amplitude = 0;
  m_x_swap_amplitude_shift = 0;
  m_x_move_phase_shift = 0;
  m_x_move_amplitude = 0;
  m_x_move_amplitude_shift = 0;
  m_y_swap_phase_shift = 0;
  m_y_swap_amplitude = 0;
  m_y_swap_amplitude_shift = 0;
  m_y_move_phase_shift = 0;
  m_y_move_amplitude = 0;
  m_y_move_amplitude_shift = 0;
  m_z_swap_phase_shift = 0;
  m_z_swap_amplitude = 0;
  m_z_swap_amplitude_shift = 0;
  m_z_move_phase_shift = 0;
  m_z_move_amplitude = 0;
  m_z_move_amplitude_Goal = 0;
  m_z_move_amplitude_shift = 0;
  m_a_move_phase_shift = 0;
  m_a_move_amplitude = 0;
  m_a_move_amplitude_shift = 0;

  m_arm_swing_gain = 0;
  m_arm_roll_Gain = 0;

  x_move_amplitude = 0;
  y_move_amplitude = 0;
  a_move_amplitude = 0;
  a_move_aim_on = false;

  balance_enable = false;

  position_x = 0;
  position_y = 0;
  orientation = 0;

  hip_comp = 0.0;
  foot_comp = 0.0;

  time_unit = 8.0;

  for (auto id : tachimawari::joint::JointId::list) {
    if (id <= tachimawari::joint::JointId::LEFT_ANKLE_PITCH) {
      joints.push_back(tachimawari::joint::Joint(id));
    }
  }
}

double Walking::wsin(double time, double period, double period_shift, double mag, double mag_shift)
{
  return mag * sin(2_pi / period * time - period_shift) + mag_shift;
}

bool Walking::compute_ik(double * out, double x, double y, double z, double a, double b, double c)
{
  {
    using keisan::Point3;
    using keisan::Euler;
    using keisan::Matrix;

    auto matrix_translation = keisan::translation_matrix(Point3(x, y, z - leg_length));
    auto matrix_rotation = keisan::rotation_matrix(Euler(keisan::make_radian(a),
      keisan::make_radian(b), keisan::make_radian(c));

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
    *(out + 3) = acos_result;

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
      *(out + 5) = -acos_result;
    } else {
      *(out + 5) = acos_result;
    }

    // get hip yaw
    matrix_translation = keisan::translation_matrix(Point3(0, 0, ankle_length));
    matrix_rotation = keisan::rotation_matrix(Euler(keisan::make_radian(*(out + 5)), 0, 0);

    matrix = matrix_translation * matrix_rotation;
    if (!matrix.inverse()) {
      return false;
    }

    matrix = matrix_transformation * matrix;
    double atan_result = atan2(-matrix[0][1], matrix[1][1]);
    if (std::isinf(atan_result)) {
      return false;
    }
    *(out) = atan_result;

    // get hip roll
    atan_result = atan2(matrix[2][1], -matrix[0][1] * sin(*(out)) + matrix[1][1] * cos(*(out)));
    if (std::isinf(atan_result)) {
      return false;
    }
    *(out + 1) = atan_result;

    // get hip pitch and ankle pitch
    atan_result = atan2(matrix[0][2] * cos(*(out)) + matrix[1][2] * sin(*(out)), matrix[0][0] *
      cos(*(out)) + matrix[1][0] * sin(*(out)));
    if (std::isinf(atan_result)) {
      return false;
    }

    k = sin(*(out + 3)) * calf_length;
    l = -thigh_length - cos(*(out + 3)) * calf_length;
    m = cos(*(out)) * vector.x + sin(*(out)) * vector.y;

    double n = cos(*(out + 1)) * vector.z + sin(*(out)) * sin(*(out + 1)) * vector.x - cos(*(out)) *
      sin(*(out + 1)) * vector.y;
    double s = (k * n + l * m) / (k * k + l * l);
    double c = (n - k * s) / l;

    double theta_result = atan_result;
    atan_result = atan2(s, c);
    if (std::isinf(atan_result)) {
      return false;
    }
    *(out + 2) = atan_result;
    *(out + 4) = theta_result - *(out + 3) - *(out + 2);

    return true;
  }
}

void Walking::compute_odometry()
{
  if (fabs(m_x_move_amplitude) >= 5 || fabs(m_y_move_amplitude) >= 5) {
    float dx = m_x_move_amplitude * odometry_fx_coefficient / 30.0;

    float dy = 0.0;
    if (m_y_move_amplitude > 0.0) {
      dy = -m_y_move_amplitude * odometry_ly_coefficient / 30.0;
    } else {
      dy = -m_y_move_amplitude * odometry_ry_coefficient / 30.0;
    }

    float theta = keisan::make_degree(orientation).radian();

    position_x += dx * cos(theta) - dy * sin(theta);
    position_y += dx * sin(theta) + dy * cos(theta);
  }
}

void Walking::update_param_time()
{
  dsp_comp = fabs(m_x_move_amplitude) * dsp_comp_ratio * 0.001;

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

void Walking::update_param_move()
{
  double x_input = x_move_amplitude;
  double y_input = y_move_amplitude * 0.5;
  double a_input = a_move_amplitude;

  if (m_z_move_amplitude < (z_move_amplitude * 0.45)) {
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
      m_z_move_amplitude, (z_move_amplitude + foot_comp) * 0.5, foot_accel_ratio);
  } else {
    m_z_move_amplitude = 0.0;
  }

  m_z_move_amplitude_shift = m_z_move_amplitude;
  m_z_swap_amplitude = z_swap_amplitude;

  if (a_move_aim_on == false) {
    m_a_move_amplitude = keisan::smooth(
      m_a_move_amplitude, keisan::make_degree(a_input).radian() / 2, move_accel_ratio);
    m_a_move_amplitude_shift = fabs(m_a_move_amplitude);
  } else {
    m_a_move_amplitude = keisan::smooth(
      m_a_move_amplitude, keisan::make_degree(-a_input).radian() / 2, move_accel_ratio);
    m_a_move_amplitude_shift = -fabs(m_a_move_amplitude);
  }
}

void Walking::load_data(const std::string & path)
{
  std::string file_name =
    path + "walking/" + "aruku.json";
  std::ifstream file(file_name);
  nlohmann::json walking_data = nlohmann::json::parse(file);

  for (auto &[key, val] : walking_data.items()) {
    if (key == "ratio") {
      try {
        val.at("period_time").get_to(period_time);
        val.at("dsp_ratio").get_to(dsp_ratio);
        val.at("foot_height").get_to(z_move_amplitude);
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
    } else if (key == "Balance") {
      try {
        val.at("balance_knee_gain").get_to(balance_knee_gain);
        val.at("balance_ankle_pitch_gain").get_to(balance_ankle_pitch_gain);
        val.at("balance_hip_roll_gain").get_to(balance_hip_roll_gain);
        val.at("balance_ankle_roll_gain").get_to(balance_ankle_roll_gain);
      } catch (nlohmann::json::parse_error & ex) {
        std::cerr << "parse error at byte " << ex.byte << std::endl;
      }
    } else if (key == "pid") {
      try {
        val.at("p_gain").get_to(p_gain);
        val.at("i_gain").get_to(i_gain);
        val.at("d_gain").get_to(d_gain);
      } catch (nlohmann::json::parse_error & ex) {
        std::cerr << "parse error at byte " << ex.byte << std::endl;
      }
    } else if (key == "Odometry") {
      try {
        val.at("fx_coefficient").get_to(odometry_fx_coefficient);
        val.at("ly_coefficient").get_to(odometry_ly_coefficient);
        val.at("ry_coefficient").get_to(odometry_ry_coefficient);
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
    } else if (key == "initangles") {
      try {
        val.at("right_hip_yaw").get_to(initial_r_hip_yaw);
        val.at("right_hip_pitch").get_to(initial_r_hip_pitch);
        val.at("right_hip_roll").get_to(initial_r_hip_roll);
        val.at("right_knee").get_to(initial_r_knee);
        val.at("right_ankle_pitch").get_to(initial_r_ankle_pitch);
        val.at("right_ankle_roll").get_to(initial_r_ankle_roll);
        val.at("left_hip_yaw").get_to(initial_l_hip_yaw);
        val.at("left_hip_pitch").get_to(initial_l_hip_pitch);
        val.at("left_hip_roll").get_to(initial_l_hip_roll);
        val.at("left_knee").get_to(initial_l_knee);
        val.at("left_ankle_pitch").get_to(initial_l_ankle_pitch);
        val.at("left_ankle_roll").get_to(initial_l_ankle_roll);
        val.at("right_shoulder_pitch").get_to(initial_r_shoulder_pitch);
        val.at("right_shoulder_roll").get_to(initial_r_shoulder_roll);
        val.at("right_elbow").get_to(initial_r_elbow);
        val.at("left_shoulder_pitch").get_to(initial_l_shoulder_pitch);
        val.at("left_shoulder_roll").get_to(initial_l_shoulder_roll);
        val.at("left_elbow").get_to(initial_l_elbow);
      } catch (nlohmann::json::parse_error & ex) {
        std::cerr << "parse error at byte " << ex.byte << std::endl;
      }
    }
  }
}

void Walking::initialize()
{
  x_move_amplitude = 0;
  y_move_amplitude = 0;
  a_move_amplitude = 0;

  m_x_move_phase_shift = keisan::pi<double> / 2;
  m_x_move_amplitude_shift = 0;
  m_y_move_phase_shift = keisan::pi<double> / 2;
  m_z_move_phase_shift = keisan::pi<double> / 2;
  m_a_move_phase_shift = keisan::pi<double> / 2;

  m_ctrl_running = false;
  m_real_running = false;
  m_time = 0;

  joints.at(0).set_target_position(initial_r_hip_yaw);
  joints.at(2).set_target_position(initial_r_hip_pitch);
  joints.at(1).set_target_position(initial_r_hip_roll);
  joints.at(3).set_target_position(initial_r_knee);
  joints.at(4).set_target_position(initial_r_ankle_pitch);
  joints.at(5).set_target_position(initial_r_ankle_roll);
  joints.at(6).set_target_position(initial_l_hip_yaw);
  joints.at(8).set_target_position(initial_l_hip_pitch);
  joints.at(7).set_target_position(initial_l_hip_roll);
  joints.at(9).set_target_position(initial_l_knee);
  joints.at(10).set_target_position(initial_l_ankle_pitch);
  joints.at(11).set_target_position(initial_l_ankle_roll);
  joints.at(12).set_target_position(initial_r_shoulder_pitch);
  joints.at(13).set_target_position(initial_r_shoulder_roll);
  joints.at(14).set_target_position(initial_r_elbow);
  joints.at(15).set_target_position(initial_l_shoulder_pitch);
  joints.at(16).set_target_position(initial_l_shoulder_roll);
  joints.at(17).set_target_position(initial_l_elbow);

  update_param_time();
  update_param_move();

  process();
}

void Walking::start()
{
  m_ctrl_running = true;
  m_real_running = true;
}

void Walking::stop()
{
  m_ctrl_running = false;
}

void Walking::force_stop()
{
  m_ctrl_running = false;
  m_real_running = false;

  m_x_move_amplitude = 0.0;
  m_y_move_amplitude = 0.0;
  m_a_move_amplitude = 0.0;
  m_z_move_amplitude = 0.0;

  update_param_time();
  update_param_move();
}

void Walking::process()
{
  if (m_time == 0) {
    update_param_move();
    update_param_time();

    if (!m_ctrl_running) {
      bool walk_in_position = true;
      walk_in_position &= (fabs(m_x_move_amplitude) <= 5.0);
      walk_in_position &= (fabs(m_y_move_amplitude) <= 5.0);
      walk_in_position &= (fabs(m_a_move_amplitude) <= 5.0);

      m_z_move_amplitude = 0;

      if (walk_in_position) {
        m_real_running = false;
      } else {
        x_move_amplitude = 0;
        y_move_amplitude = 0;
        a_move_amplitude = 0;
      }
    }
  } else if (m_time >= (m_phase_time1 - time_unit / 2) &&  // NOLINT
    m_time < (m_phase_time1 + time_unit / 2))
  {
    // left leg
    update_param_move();
    compute_odometry();
  } else if (m_time >= (m_phase_time2 - time_unit / 2) &&  // NOLINT
    m_time < (m_phase_time2 + time_unit / 2))
  {
    update_param_move();
    update_param_time();

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
        x_move_amplitude = 0;
        y_move_amplitude = 0;
        a_move_amplitude = 0;
      }
    }
  } else if (m_time >= (m_phase_time3 - time_unit / 2) &&  // NOLINT
    m_time < (m_phase_time3 + time_unit / 2))
  {
    // right leg
    update_param_move();
    compute_odometry();
  }

  // Compute endpoints
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
    x_move_l = wsin(
      m_ssp_time_start_l, m_x_move_period_time,
      m_x_move_phase_shift + 2_pi / m_x_move_period_time *
      m_ssp_time_start_l, m_x_move_amplitude,
      m_x_move_amplitude_shift);
    y_move_l = wsin(
      m_ssp_time_start_l, m_y_move_period_time,
      m_y_move_phase_shift + 2_pi / m_y_move_period_time *
      m_ssp_time_start_l, m_y_move_amplitude,
      m_y_move_amplitude_shift);
    z_move_l = wsin(
      m_ssp_time_start_l, m_z_move_period_time,
      m_z_move_phase_shift + 2_pi / m_z_move_period_time *
      m_ssp_time_start_l, m_z_move_amplitude,
      m_z_move_amplitude_shift);
    c_move_l = wsin(
      m_ssp_time_start_l, m_a_move_period_time,
      m_a_move_phase_shift + 2_pi / m_a_move_period_time *
      m_ssp_time_start_l, m_a_move_amplitude,
      m_a_move_amplitude_shift);
    x_move_r = wsin(
      m_ssp_time_start_l, m_x_move_period_time,
      m_x_move_phase_shift + 2_pi / m_x_move_period_time *
      m_ssp_time_start_l, -m_x_move_amplitude,
      -m_x_move_amplitude_shift);
    y_move_r = wsin(
      m_ssp_time_start_l, m_y_move_period_time,
      m_y_move_phase_shift + 2_pi / m_y_move_period_time *
      m_ssp_time_start_l, -m_y_move_amplitude,
      -m_y_move_amplitude_shift);
    z_move_r = wsin(
      m_ssp_time_start_r, m_z_move_period_time,
      m_z_move_phase_shift + 2_pi / m_z_move_period_time *
      m_ssp_time_start_r, m_z_move_amplitude,
      m_z_move_amplitude_shift);
    c_move_r = wsin(
      m_ssp_time_start_l, m_a_move_period_time,
      m_a_move_phase_shift + 2_pi / m_a_move_period_time *
      m_ssp_time_start_l, -m_a_move_amplitude,
      -m_a_move_amplitude_shift);
  } else if (m_time <= m_ssp_time_end_l) {
    x_move_l = wsin(
      m_time, m_x_move_period_time,
      m_x_move_phase_shift + 2_pi / m_x_move_period_time *
      m_ssp_time_start_l, m_x_move_amplitude,
      m_x_move_amplitude_shift);
    y_move_l = wsin(
      m_time, m_y_move_period_time,
      m_y_move_phase_shift + 2_pi / m_y_move_period_time *
      m_ssp_time_start_l, m_y_move_amplitude,
      m_y_move_amplitude_shift);
    z_move_l = wsin(
      m_time, m_z_move_period_time,
      m_z_move_phase_shift + 2_pi / m_z_move_period_time *
      m_ssp_time_start_l, m_z_move_amplitude,
      m_z_move_amplitude_shift);
    c_move_l = wsin(
      m_time, m_a_move_period_time,
      m_a_move_phase_shift + 2_pi / m_a_move_period_time *
      m_ssp_time_start_l, m_a_move_amplitude,
      m_a_move_amplitude_shift);
    x_move_r = wsin(
      m_time, m_x_move_period_time,
      m_x_move_phase_shift + 2_pi / m_x_move_period_time *
      m_ssp_time_start_l, -m_x_move_amplitude,
      -m_x_move_amplitude_shift);
    y_move_r = wsin(
      m_time, m_y_move_period_time,
      m_y_move_phase_shift + 2_pi / m_y_move_period_time *
      m_ssp_time_start_l, -m_y_move_amplitude,
      -m_y_move_amplitude_shift);
    z_move_r = wsin(
      m_ssp_time_start_r, m_z_move_period_time,
      m_z_move_phase_shift + 2_pi / m_z_move_period_time *
      m_ssp_time_start_r, m_z_move_amplitude,
      m_z_move_amplitude_shift);
    c_move_r = wsin(
      m_time, m_a_move_period_time,
      m_a_move_phase_shift + 2_pi / m_a_move_period_time *
      m_ssp_time_start_l, -m_a_move_amplitude,
      -m_a_move_amplitude_shift);
  } else if (m_time <= m_ssp_time_start_r) {
    x_move_l = wsin(
      m_ssp_time_end_l, m_x_move_period_time,
      m_x_move_phase_shift + 2_pi / m_x_move_period_time *
      m_ssp_time_start_l, m_x_move_amplitude,
      m_x_move_amplitude_shift);
    y_move_l = wsin(
      m_ssp_time_end_l, m_y_move_period_time,
      m_y_move_phase_shift + 2_pi / m_y_move_period_time *
      m_ssp_time_start_l, m_y_move_amplitude,
      m_y_move_amplitude_shift);
    z_move_l = wsin(
      m_ssp_time_end_l, m_z_move_period_time,
      m_z_move_phase_shift + 2_pi / m_z_move_period_time *
      m_ssp_time_start_l, m_z_move_amplitude,
      m_z_move_amplitude_shift);
    c_move_l = wsin(
      m_ssp_time_end_l, m_a_move_period_time,
      m_a_move_phase_shift + 2_pi / m_a_move_period_time *
      m_ssp_time_start_l, m_a_move_amplitude,
      m_a_move_amplitude_shift);
    x_move_r = wsin(
      m_ssp_time_end_l, m_x_move_period_time,
      m_x_move_phase_shift + 2_pi / m_x_move_period_time *
      m_ssp_time_start_l, -m_x_move_amplitude,
      -m_x_move_amplitude_shift);
    y_move_r = wsin(
      m_ssp_time_end_l, m_y_move_period_time,
      m_y_move_phase_shift + 2_pi / m_y_move_period_time *
      m_ssp_time_start_l, -m_y_move_amplitude,
      -m_y_move_amplitude_shift);
    z_move_r = wsin(
      m_ssp_time_start_r, m_z_move_period_time,
      m_z_move_phase_shift + 2_pi / m_z_move_period_time *
      m_ssp_time_start_r, m_z_move_amplitude,
      m_z_move_amplitude_shift);
    c_move_r = wsin(
      m_ssp_time_end_l, m_a_move_period_time,
      m_a_move_phase_shift + 2_pi / m_a_move_period_time *
      m_ssp_time_start_l, -m_a_move_amplitude,
      -m_a_move_amplitude_shift);
  } else if (m_time <= m_ssp_time_End_r) {
    x_move_l = wsin(
      m_time, m_x_move_period_time,
      m_x_move_phase_shift + 2_pi / m_x_move_period_time *
      m_ssp_time_start_r + keisan::pi<double>, m_x_move_amplitude,
      m_x_move_amplitude_shift);
    y_move_l = wsin(
      m_time, m_y_move_period_time,
      m_y_move_phase_shift + 2_pi / m_y_move_period_time *
      m_ssp_time_start_r + keisan::pi<double>, m_y_move_amplitude,
      m_y_move_amplitude_shift);
    z_move_l = wsin(
      m_ssp_time_end_l, m_z_move_period_time,
      m_z_move_phase_shift + 2_pi / m_z_move_period_time *
      m_ssp_time_start_l, m_z_move_amplitude,
      m_z_move_amplitude_shift);
    c_move_l = wsin(
      m_time, m_a_move_period_time,
      m_a_move_phase_shift + 2_pi / m_a_move_period_time *
      m_ssp_time_start_r + keisan::pi<double>, m_a_move_amplitude,
      m_a_move_amplitude_shift);
    x_move_r = wsin(
      m_time, m_x_move_period_time,
      m_x_move_phase_shift + 2_pi / m_x_move_period_time *
      m_ssp_time_start_r + keisan::pi<double>, -m_x_move_amplitude,
      -m_x_move_amplitude_shift);
    y_move_r = wsin(
      m_time, m_y_move_period_time,
      m_y_move_phase_shift + 2_pi / m_y_move_period_time *
      m_ssp_time_start_r + keisan::pi<double>, -m_y_move_amplitude,
      -m_y_move_amplitude_shift);
    z_move_r = wsin(
      m_time, m_z_move_period_time,
      m_z_move_phase_shift + 2_pi / m_z_move_period_time *
      m_ssp_time_start_r, m_z_move_amplitude,
      m_z_move_amplitude_shift);
    c_move_r = wsin(
      m_time, m_a_move_period_time,
      m_a_move_phase_shift + 2_pi / m_a_move_period_time *
      m_ssp_time_start_r + keisan::pi<double>, -m_a_move_amplitude,
      -m_a_move_amplitude_shift);
  } else {
    x_move_l = wsin(
      m_ssp_time_End_r, m_x_move_period_time,
      m_x_move_phase_shift + 2_pi / m_x_move_period_time *
      m_ssp_time_start_r + keisan::pi<double>, m_x_move_amplitude,
      m_x_move_amplitude_shift);
    y_move_l = wsin(
      m_ssp_time_End_r, m_y_move_period_time,
      m_y_move_phase_shift + 2_pi / m_y_move_period_time *
      m_ssp_time_start_r + keisan::pi<double>, m_y_move_amplitude,
      m_y_move_amplitude_shift);
    z_move_l = wsin(
      m_ssp_time_end_l, m_z_move_period_time,
      m_z_move_phase_shift + 2_pi / m_z_move_period_time *
      m_ssp_time_start_l, m_z_move_amplitude,
      m_z_move_amplitude_shift);
    c_move_l = wsin(
      m_ssp_time_End_r, m_a_move_period_time,
      m_a_move_phase_shift + 2_pi / m_a_move_period_time *
      m_ssp_time_start_r + keisan::pi<double>, m_a_move_amplitude,
      m_a_move_amplitude_shift);
    x_move_r = wsin(
      m_ssp_time_End_r, m_x_move_period_time,
      m_x_move_phase_shift + 2_pi / m_x_move_period_time *
      m_ssp_time_start_r + keisan::pi<double>, -m_x_move_amplitude,
      -m_x_move_amplitude_shift);
    y_move_r = wsin(
      m_ssp_time_End_r, m_y_move_period_time,
      m_y_move_phase_shift + 2_pi / m_y_move_period_time *
      m_ssp_time_start_r + keisan::pi<double>, -m_y_move_amplitude,
      -m_y_move_amplitude_shift);
    z_move_r = wsin(
      m_ssp_time_End_r, m_z_move_period_time,
      m_z_move_phase_shift + 2_pi / m_z_move_period_time *
      m_ssp_time_start_r, m_z_move_amplitude,
      m_z_move_amplitude_shift);
    c_move_r = wsin(
      m_ssp_time_End_r, m_a_move_period_time,
      m_a_move_phase_shift + 2_pi / m_a_move_period_time *
      m_ssp_time_start_r + keisan::pi<double>, -m_a_move_amplitude,
      -m_a_move_amplitude_shift);
  }

  m_x_offset = x_offset;
  m_y_offset = y_offset;
  m_z_offset = z_offset;
  m_r_offset = keisan::make_degree(r_offset).radian();
  m_p_offset = keisan::make_degree(p_offset).radian();
  m_a_offset = keisan::make_degree(a_offset).radian();

  double r_x = x_swap + x_move_r + m_x_offset;
  double r_y = y_swap + y_move_r - m_y_offset / 2;
  double r_z = z_swap + z_move_r + m_z_offset;
  double r_a = a_swap + a_move_r - m_r_offset / 2;
  double r_b = b_swap + b_move_r + m_p_offset;
  double r_c = c_swap + c_move_r - m_a_offset / 2;

  double l_x = x_swap + x_move_l + m_x_offset;
  double l_y = y_swap + y_move_l + m_y_offset / 2;
  double l_z = z_swap + z_move_l + m_z_offset;
  double l_a = a_swap + a_move_l + m_r_offset / 2;
  double l_b = b_swap + b_move_l + m_p_offset;
  double l_c = c_swap + c_move_l + m_a_offset / 2;

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

  // compute angles
  if (!compute_ik(&angle[0], r_x, r_y, r_z, r_a, r_b, r_c)) {
    return;
  }

  if (!compute_ik(&angle[6], l_x, l_y, l_z, l_a, l_b, l_c)) {
    return;
  }

  for (int i = 0; i < 12; i++) {
    angle[i] = keisan::make_radian(angle[i]).degree();
  }

  double initangle[22];
  initangle[0] = initial_r_hip_yaw;
  initangle[1] = initial_r_hip_roll;
  initangle[2] = initial_r_hip_pitch;
  initangle[3] = initial_r_knee;
  initangle[4] = initial_r_ankle_pitch;
  initangle[5] = initial_r_ankle_roll;
  initangle[6] = initial_l_hip_yaw;
  initangle[7] = initial_l_hip_roll;
  initangle[8] = initial_l_hip_pitch;
  initangle[9] = initial_l_knee;
  initangle[10] = initial_l_ankle_pitch;
  initangle[11] = initial_l_ankle_roll;
  initangle[12] = initial_r_shoulder_pitch;
  initangle[13] = initial_r_shoulder_roll;
  initangle[14] = initial_r_elbow;
  initangle[15] = initial_l_shoulder_pitch;
  initangle[16] = initial_l_shoulder_roll;
  initangle[17] = initial_l_elbow;
  initangle[18] = 0.0;
  initangle[19] = 0.0;
  initangle[20] = initial_r_gripper;
  initangle[21] = initial_l_gripper;

  int dir[22];
  dir[0] = 1;
  dir[1] = -1;
  dir[2] = -1;
  dir[3] = -1;
  dir[4] = -1;
  dir[5] = 1;
  dir[6] = 1;
  dir[7] = -1;
  dir[8] = -1;
  dir[9] = -1;
  dir[10] = -1;
  dir[11] = 1;
  dir[12] = -1;
  dir[13] = 1;
  dir[14] = 1;
  dir[15] = -1;
  dir[16] = 1;
  dir[17] = 1;
  dir[18] = 1;
  dir[19] = 1;
  dir[20] = 1;
  dir[21] = 1;

  int outValue[22];
  for (int i = 0; i < 22; i++) {
    double offset = static_cast<double>(dir[i]) * angle[i] * mx.raTiO_aNGLE2VaLUE;

    switch (i) {
      case 2:
      case 8:
        offset -= static_cast<double>(dir[i]) * (hip_pitch_offset + hip_comp) *
          mx.raTiO_aNGLE2VaLUE;
    }

    outValue[i] = mx.angle_to_value(initangle[i]) + static_cast<int>(offset);
  }

  // adjust balance offset
  if (balance_enable) {
    double rlGyroErr = imu->get_rl_gyro();
    double fbGyroErr = imu->get_fb_gyro();

    outValue[1] += static_cast<int>(dir[1] * rlGyroErr * balance_hip_roll_gain * 4);
    outValue[7] += static_cast<int>(dir[7] * rlGyroErr * balance_hip_roll_gain * 4);

    outValue[3] -= static_cast<int>(dir[3] * fbGyroErr * balance_knee_gain * 4);
    outValue[9] -= static_cast<int>(dir[9] * fbGyroErr * balance_knee_gain * 4);

    outValue[4] -= static_cast<int>(dir[4] * fbGyroErr * balance_ankle_pitch_gain * 4);
    outValue[10] -= static_cast<int>(dir[10] * fbGyroErr * balance_ankle_pitch_gain * 4);

    outValue[5] -= static_cast<int>(dir[5] * rlGyroErr * balance_ankle_roll_gain * 4);
    outValue[11] -= static_cast<int>(dir[11] * rlGyroErr * balance_ankle_roll_gain * 4);
  }

  for (int id = 0; id < static_cast<int>(joints.size()); id++) {
    joints.at(id).set_target_position(mx.value_to_angle(outValue[id]));
    joints.at(id).set_pid_gain(p_gain, i_gain, d_gain);
  }
}

}  // namespace aruku
