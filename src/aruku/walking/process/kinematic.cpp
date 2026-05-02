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

#include "aruku/walking/process/kinematic.hpp"

#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "jitsuyo/config.hpp"
#include "keisan/keisan.hpp"
#include "nlohmann/json.hpp"
#include "tachimawari/joint/model/joint.hpp"
#include "tachimawari/joint/model/joint_id.hpp"

using keisan::literals::operator""_deg;
using keisan::literals::operator""_pi;
using keisan::literals::operator""_pi_rad;
using WalkPhase = aruku_interfaces::msg::WalkPhase;

namespace aruku
{

Kinematic::Kinematic()
: m_period_time(0),
  m_dsp_ratio(0),
  m_ssp_ratio(0),
  m_x_swap_period_time(0),
  m_x_move_period_time(0),
  m_y_swap_period_time(0),
  m_y_move_period_time(0),
  m_z_swap_period_time(0),
  m_z_move_period_time(0),
  m_a_move_period_time(0),
  m_ssp_time(0),
  m_ssp_time_start_l(0),
  m_ssp_time_end_l(0),
  m_ssp_time_start_r(0),
  m_ssp_time_end_r(0),
  m_phase_time1(0),
  m_phase_time2(0),
  m_phase_time3(0),
  m_x_swap_phase_shift(0),
  m_x_swap_amplitude(0),
  m_x_swap_amplitude_shift(0),
  m_x_move_amplitude(0),
  m_y_swap_phase_shift(0),
  m_y_swap_amplitude(0),
  m_y_swap_amplitude_shift(0),
  m_y_move_amplitude(0),
  m_y_move_amplitude_shift(0),
  m_z_swap_phase_shift(0),
  m_z_swap_amplitude(0),
  m_z_swap_amplitude_shift(0),
  m_z_move_amplitude(0),
  m_z_move_amplitude_Goal(0),
  m_z_move_amplitude_shift(0),
  m_a_move_amplitude(0),
  m_a_move_amplitude_shift(0),
  m_arm_swing_gain(0),
  m_arm_roll_Gain(0),
  hip_comp(0_deg),
  foot_comp(0.0),
  time_unit(8.0),
  m_x_move_phase_shift(0.5_pi),
  m_x_move_amplitude_shift(0),
  m_y_move_phase_shift(0.5_pi),
  m_z_move_phase_shift(0.5_pi),
  m_a_move_phase_shift(0.5_pi),
  m_ctrl_running(false),
  m_real_running(false),
  m_time(0),
  x_move(0.0),
  y_move(0.0),
  a_move(0_deg),
  a_move_aim_on(false),
  is_compute_odometry(false),
  is_paused(false),
  pause_counter(0),
  phase_on_pause(WalkPhase::DOUBLE_SUPPORT),
  do_walk_in_place(false),
  has_paused_this_cycle(false),
  kick_state(KickState::IDLE),
  kick_leg(KickLeg::RIGHT),
  m_kick_period_scale(1.0),
  kick_x_amplitude(0.0),
  kick_y_amplitude(0.0),
  kick_z_amplitude(0.0),
  kick_start_ratio(1.0),
  kick_return_ratio(0.0),
  kick_period_scale(0.0)
{
  reset_angles();
}

void Kinematic::reset_angles()
{
  for (auto & angle : angles) {
    angle = 0_deg;
  }
}

const std::array<keisan::Angle<double>, 19> & Kinematic::get_angles() const { return angles; }

void Kinematic::set_running_state(bool running_state)
{
  m_ctrl_running = running_state;

  if (running_state) {
    m_real_running = running_state;
  }
}

bool Kinematic::get_running_state() const { return m_real_running; }

void Kinematic::set_move_amplitude(double x, double y, const keisan::Angle<double> & a, bool aim_on)
{
  x_move = x;
  y_move = y;
  a_move = a;
  a_move_aim_on = aim_on;
}

void Kinematic::set_actual_walk_phase(uint8_t current_phase){
  actual_walk_phase = current_phase;
}

void Kinematic::update_imu_roll(const keisan::Angle<double> & roll){
  imu_roll = roll;
}

uint8_t Kinematic::get_expected_walk_phase(){
  if (m_time > m_ssp_time_start_l && m_time <= m_ssp_time_end_l)
    return WalkPhase::RIGHT_SUPPORT;
  if (m_time > m_ssp_time_start_r && m_time <= m_ssp_time_end_r)
    return WalkPhase::LEFT_SUPPORT;

  return WalkPhase::DOUBLE_SUPPORT;
}


double Kinematic::get_x_move_amplitude() const { return m_x_move_amplitude; }

double Kinematic::get_y_move_amplitude() const { return m_y_move_amplitude; }

double Kinematic::get_a_move_amplitude() const { return m_a_move_amplitude; }

keisan::Angle<double> Kinematic::get_raw_hip_offset() const { return hip_pitch_offset; }

keisan::Angle<double> Kinematic::get_hip_offset() const { return hip_pitch_offset + hip_comp; }

bool Kinematic::time_to_compute_odometry() const { return is_compute_odometry; }

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

double Kinematic::wsin(
  double time, double period, double period_shift, double mag, double mag_shift) const
{
  return mag * sin(2_pi / period * time - period_shift) + mag_shift;
}

bool Kinematic::compute_inverse_kinematic(
  int index_addition, keisan::Point3 translation_target, keisan::Euler<double> rotation_target)
{
  using keisan::Euler;
  using keisan::Matrix;
  using keisan::Point3;
  using tachimawari::joint::JointId;

  translation_target.z -= leg_length;
  auto matrix_translation = keisan::translation_matrix(translation_target);
  auto matrix_rotation = keisan::rotation_matrix(rotation_target);

  auto matrix_transformation = matrix_translation * matrix_rotation;

  auto vector = Point3(
    translation_target.x + matrix_transformation[0][2] * ankle_length,
    translation_target.y + matrix_transformation[1][2] * ankle_length,
    translation_target.z + matrix_transformation[2][2] * ankle_length);

  // get knee
  double vector_magnitude = vector.magnitude();
  double acos_result = acos(
    (pow(vector_magnitude, 2) - pow(thigh_length, 2) - pow(calf_length, 2)) /
    (2 * thigh_length * calf_length));

  if (std::isnan(acos_result)) {
    return false;
  }
  angles[JointId::RIGHT_KNEE + index_addition] = keisan::make_radian(acos_result);

  // get ankle roll
  auto matrix = matrix_transformation;
  if (!matrix.inverse()) {
    return false;
  }

  double k = std::hypot(matrix[1][3], matrix[2][3]);
  double l = std::hypot(matrix[1][3], (matrix[2][3] - ankle_length));
  double m = (k * k - l * l - pow(ankle_length, 2)) / (2 * l * ankle_length);
  m = keisan::clamp(m, -1.0, 1.0);

  acos_result = acos(m);
  if (std::isnan(acos_result)) {
    return false;
  }
  if (matrix[1][3] < 0.0) {
    angles[JointId::RIGHT_ANKLE_ROLL + index_addition] = keisan::make_radian(-acos_result);
  } else {
    angles[JointId::RIGHT_ANKLE_ROLL + index_addition] = keisan::make_radian(acos_result);
  }

  // get hip yaw
  matrix_translation = keisan::translation_matrix(Point3(0, 0, -ankle_length));
  matrix_rotation = keisan::rotation_matrix(
    Euler(angles[JointId::RIGHT_ANKLE_ROLL + index_addition], 0.0_pi_rad, 0.0_pi_rad));

  matrix = matrix_translation * matrix_rotation;
  if (!matrix.inverse()) {
    return false;
  }

  matrix = matrix_transformation * matrix;
  double atan_result = atan2(-matrix[0][1], matrix[1][1]);
  if (std::isinf(atan_result)) {
    return false;
  }
  angles[JointId::RIGHT_HIP_YAW + index_addition] = keisan::make_radian(atan_result);

  // get hip roll
  k = -matrix[0][1] * angles[JointId::RIGHT_HIP_YAW + index_addition].sin();
  l = matrix[1][1] * angles[JointId::RIGHT_HIP_YAW + index_addition].cos();
  atan_result = atan2(matrix[2][1], k + l);

  if (std::isinf(atan_result)) {
    return false;
  }
  angles[JointId::RIGHT_HIP_ROLL + index_addition] = keisan::make_radian(atan_result);

  // get hip pitch and ankle pitch
  atan_result = atan2(
    matrix[0][2] * angles[JointId::RIGHT_HIP_YAW + index_addition].cos() +
      matrix[1][2] * angles[JointId::RIGHT_HIP_YAW + index_addition].sin(),
    matrix[0][0] * angles[JointId::RIGHT_HIP_YAW + index_addition].cos() +
      matrix[1][0] * angles[JointId::RIGHT_HIP_YAW + index_addition].sin());
  if (std::isinf(atan_result)) {
    return false;
  }

  k = angles[JointId::RIGHT_KNEE + index_addition].sin() * calf_length;
  l = -thigh_length - angles[JointId::RIGHT_KNEE + index_addition].cos() * calf_length;

  double n = angles[JointId::RIGHT_HIP_YAW + index_addition].cos() * vector.x;
  double o = angles[JointId::RIGHT_HIP_YAW + index_addition].sin() * vector.y;
  m = n + o;

  o = angles[JointId::RIGHT_HIP_ROLL + index_addition].cos() * vector.z;
  double p = angles[JointId::RIGHT_HIP_YAW + index_addition].sin() *
             angles[JointId::RIGHT_HIP_ROLL + index_addition].sin() * vector.x;
  double q = angles[JointId::RIGHT_HIP_YAW + index_addition].cos() *
             angles[JointId::RIGHT_HIP_ROLL + index_addition].sin() * vector.y;
  n = o + p - q;

  double s = (k * n + l * m) / (k * k + l * l);
  double t = (n - k * s) / l;

  double theta_result = atan_result;
  atan_result = atan2(s, t);
  if (std::isinf(atan_result)) {
    return false;
  }
  angles[JointId::RIGHT_HIP_PITCH + index_addition] = keisan::make_radian(atan_result);
  angles[JointId::RIGHT_ANKLE_PITCH + index_addition] =
    keisan::make_radian(theta_result) - angles[JointId::RIGHT_KNEE + index_addition] -
    angles[JointId::RIGHT_HIP_PITCH + index_addition];

  return true;
}

void Kinematic::update_times()
{
  double dsp_comp = fabs(m_x_move_amplitude) * dsp_comp_ratio * 0.001;
  double period_comp_ratio = (m_x_move_amplitude > 0)
                            ? forward_period_comp_ratio
                            : backward_period_comp_ratio;

  m_period_time = (period_time - (fabs(m_x_move_amplitude) * period_comp_ratio))
                * m_kick_period_scale;

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
  m_ssp_time_end_r = (3 + m_ssp_ratio) * m_period_time / 4;

  m_phase_time1 = (m_ssp_time_start_l + m_ssp_time_end_l) / 2;
  m_phase_time2 = (m_ssp_time_end_l + m_ssp_time_start_r) / 2;
  m_phase_time3 = (m_ssp_time_start_r + m_ssp_time_end_r) / 2;

  m_arm_swing_gain = arm_swing_gain;

  hip_comp = keisan::make_degree(
    m_x_move_amplitude *
    ((m_x_move_amplitude > 0) ? forward_hip_comp_ratio : backward_hip_comp_ratio));

  foot_comp = fabs(m_x_move_amplitude) * foot_comp_ratio;
}

void Kinematic::update_move_amplitude()
{
  double x_input = x_move;
  double y_input = y_move * 0.5;
  auto a_input   = a_move;

  if (m_z_move_amplitude < (z_move * 0.45)) {
    x_input = 0.0;
    y_input = 0.0;
    a_input = 0_deg;
  }

  m_x_move_amplitude = keisan::smooth(m_x_move_amplitude, x_input, move_accel_ratio);
  m_y_move_amplitude = keisan::smooth(m_y_move_amplitude, y_input, move_accel_ratio);
  m_y_move_amplitude_shift = fabs(m_y_move_amplitude);
  m_y_swap_amplitude = y_swap_amplitude + m_y_move_amplitude_shift * 0.04;

  if (m_ctrl_running) {
    m_z_move_amplitude =
      keisan::smooth(m_z_move_amplitude, (z_move + foot_comp) * 0.5, foot_accel_ratio);
  } else {
    m_z_move_amplitude = 0.0;
  }

  m_z_move_amplitude_shift = m_z_move_amplitude;
  m_z_swap_amplitude = z_swap_amplitude;

  if (!a_move_aim_on) {
    if (m_a_move_amplitude_shift < -0.005) {
      m_a_move_amplitude = keisan::smooth(m_a_move_amplitude, 0.0, move_accel_ratio);
      m_a_move_amplitude_shift = -fabs(m_a_move_amplitude);
    } else {
      m_a_move_amplitude =
        keisan::smooth(m_a_move_amplitude, a_input.radian() / 2, move_accel_ratio);
      m_a_move_amplitude_shift = fabs(m_a_move_amplitude);
    }
  } else {
    if (m_a_move_amplitude_shift > 0.005) {
      m_a_move_amplitude = keisan::smooth(m_a_move_amplitude, 0.0, move_accel_ratio);
      m_a_move_amplitude_shift = fabs(m_a_move_amplitude);
    } else {
      m_a_move_amplitude =
        keisan::smooth(m_a_move_amplitude, -a_input.radian() / 2, move_accel_ratio);
      m_a_move_amplitude_shift = -fabs(m_a_move_amplitude);
    }
  }
}

void Kinematic::set_config(const nlohmann::json & kinematic_data)
{
  bool valid_config = true;

  nlohmann::json ratio_section;
  if (jitsuyo::assign_val(kinematic_data, "ratio", ratio_section)) {
    bool valid_section = true;
    valid_section &= jitsuyo::assign_val(ratio_section, "period_time", period_time);
    valid_section &= jitsuyo::assign_val(ratio_section, "dsp_ratio", dsp_ratio);
    valid_section &= jitsuyo::assign_val(ratio_section, "foot_height", z_move);
    valid_section &= jitsuyo::assign_val(ratio_section, "swing_right_left", y_swap_amplitude);
    valid_section &= jitsuyo::assign_val(ratio_section, "swing_up_down", z_swap_amplitude);
    valid_section &= jitsuyo::assign_val(ratio_section, "arm_swing_gain", arm_swing_gain);
    valid_section &=
      jitsuyo::assign_val(ratio_section, "backward_hip_comp_ratio", backward_hip_comp_ratio);
    valid_section &=
      jitsuyo::assign_val(ratio_section, "forward_hip_comp_ratio", forward_hip_comp_ratio);
    valid_section &= jitsuyo::assign_val(ratio_section, "foot_comp_ratio", foot_comp_ratio);
    valid_section &= jitsuyo::assign_val(ratio_section, "dsp_comp_ratio", dsp_comp_ratio);
    valid_section &=
      jitsuyo::assign_val(ratio_section, "forward_period_comp_ratio", forward_period_comp_ratio);
    valid_section &=
      jitsuyo::assign_val(ratio_section, "backward_period_comp_ratio", backward_period_comp_ratio);
    valid_section &= jitsuyo::assign_val(ratio_section, "move_accel_ratio", move_accel_ratio);
    valid_section &= jitsuyo::assign_val(ratio_section, "foot_accel_ratio", foot_accel_ratio);

    if (!valid_section) {
      std::cout << "Error found at section `ratio`" << std::endl;
      valid_config = false;
    }
  } else {
    valid_config = false;
  }

  nlohmann::json length_section;
  if (jitsuyo::assign_val(kinematic_data, "length", length_section)) {
    bool valid_section = true;
    valid_section &= jitsuyo::assign_val(length_section, "thigh_length", thigh_length);
    valid_section &= jitsuyo::assign_val(length_section, "calf_length", calf_length);
    valid_section &= jitsuyo::assign_val(length_section, "ankle_length", ankle_length);
    valid_section &= jitsuyo::assign_val(length_section, "leg_length", leg_length);

    if (!valid_section) {
      std::cout << "Error found at section `length`" << std::endl;
      valid_config = false;
    }
  } else {
    valid_config = false;
  }

  nlohmann::json offset_section;
  if (jitsuyo::assign_val(kinematic_data, "offset", offset_section)) {
    bool valid_section = true;

    double yaw_offset_double;
    double roll_offset_double;
    double pitch_offset_double;
    double hip_pitch_offset_double;

    valid_section &= jitsuyo::assign_val(offset_section, "x_offset", x_offset);
    valid_section &= jitsuyo::assign_val(offset_section, "y_offset", y_offset);
    valid_section &= jitsuyo::assign_val(offset_section, "z_offset", z_offset);
    valid_section &= jitsuyo::assign_val(offset_section, "yaw_offset", yaw_offset_double);
    valid_section &= jitsuyo::assign_val(offset_section, "roll_offset", roll_offset_double);
    valid_section &= jitsuyo::assign_val(offset_section, "pitch_offset", pitch_offset_double);
    valid_section &=
      jitsuyo::assign_val(offset_section, "hip_pitch_offset", hip_pitch_offset_double);

    yaw_offset = keisan::make_degree(yaw_offset_double);
    roll_offset = keisan::make_degree(roll_offset_double);
    pitch_offset = keisan::make_degree(pitch_offset_double);
    hip_pitch_offset = keisan::make_degree(hip_pitch_offset_double);

    if (!valid_section) {
      std::cout << "Error found at section `offset`" << std::endl;
      valid_config = false;
    }
  } else {
    valid_config = false;
  }

  nlohmann::json balance_section;
  if(jitsuyo::assign_val(kinematic_data, "balance_pause", balance_section)){
    bool valid_section = true;
    valid_section &= jitsuyo::assign_val(balance_section, "enable", pause_enable);
    valid_section &= jitsuyo::assign_val(balance_section, "roll_pause_threshold", roll_pause_threshold);
    valid_section &= jitsuyo::assign_val(balance_section, "roll_resume_threshold", roll_resume_threshold);
    valid_section &= jitsuyo::assign_val(balance_section, "max_pause_counter", max_pause_counter);
    valid_section &= jitsuyo::assign_val(balance_section, "max_pause_speed", max_pause_speed);

    if (!valid_section) {
      std::cout << "Error found at section `balance`" << std::endl;
      valid_config = false;
    }
  } else {
    valid_config = false;
  }

  nlohmann::json kick_section;
  if(jitsuyo::assign_val(kinematic_data, "in_walk_kick", kick_section)){
    bool valid_section = true;
    valid_section &= jitsuyo::assign_val(kick_section, "x_amplitude", kick_x_amplitude);
    valid_section &= jitsuyo::assign_val(kick_section, "y_amplitude", kick_y_amplitude);
    valid_section &= jitsuyo::assign_val(kick_section, "z_amplitude", kick_z_amplitude);
    valid_section &= jitsuyo::assign_val(kick_section, "start_ratio", kick_start_ratio);
    valid_section &= jitsuyo::assign_val(kick_section, "return_ratio", kick_return_ratio);
    valid_section &= jitsuyo::assign_val(kick_section, "period_scale", kick_period_scale);

    if (!valid_section) {
      std::cout << "Error found at section `in_walk_kick`" << std::endl;
      valid_config = false;
    }
  } else {
    valid_config = false;
  }

  if (!valid_config) {
    throw std::runtime_error("Failed to load config file `kinematic.json`");
  }

  update_times();
  update_move_amplitude();

  run_kinematic();
}

bool Kinematic::run_kinematic()
{
  is_compute_odometry = false;

  if (m_time == 0) {
    // Right kick finished, reset kick period scale
    if (kick_state == KickState::KICKING && (kick_leg == KickLeg::RIGHT || kick_leg == KickLeg::RIGHT_CENTER)) {
      m_kick_period_scale = 1.0;
      kick_state = KickState::DONE;
      printf("[KICK KICKING→DONE] Right swing ended at phase boundary\n");
    }
    // Left kick ready, apply kick period scale before left SSP
    if (kick_state == KickState::PREPARE && (kick_leg == KickLeg::LEFT || kick_leg == KickLeg::LEFT_CENTER)) {
      m_kick_period_scale = kick_period_scale;
      kick_state = KickState::KICKING;
      printf("[KICK PREPARE→KICKING] Left kick armed, scale applied\n");
    }

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
        a_move = 0_deg;
      }
    }
  } else if (
    m_time >= (m_phase_time1 - time_unit / 2) &&  // NOLINT
    m_time < (m_phase_time1 + time_unit / 2)) {
    // left leg
    update_move_amplitude();
    is_compute_odometry = true;
    do_walk_in_place = false;
  } else if (
    m_time >= (m_phase_time2 - time_unit / 2) &&  // NOLINT
    m_time < (m_phase_time2 + time_unit / 2)) {

    // Left kick finished, reset kick period scale
    if (kick_state == KickState::KICKING && (kick_leg == KickLeg::LEFT || kick_leg == KickLeg::LEFT_CENTER)) {
      m_kick_period_scale = 1.0;
      kick_state = KickState::DONE;
      printf("[KICK KICKING→DONE] Left swing ended at phase boundary\n");
    }
    // Right kick ready, apply kick period scale before right SSP
    if (kick_state == KickState::PREPARE && (kick_leg == KickLeg::RIGHT || kick_leg == KickLeg::RIGHT_CENTER)) {
      m_kick_period_scale = kick_period_scale;
      kick_state = KickState::KICKING;
      printf("[KICK PREPARE→KICKING] Right kick armed, scale applied\n");
    }

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
        a_move = 0_deg;
      }
    }
  } else if (
    m_time >= (m_phase_time3 - time_unit / 2) &&  // NOLINT
    m_time < (m_phase_time3 + time_unit / 2)) {
    // right leg
    update_move_amplitude();
    is_compute_odometry = true;
    do_walk_in_place = false;
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
    x_move_l = wsin(
      m_ssp_time_start_l, m_x_move_period_time,
      m_x_move_phase_shift + 2_pi / m_x_move_period_time * m_ssp_time_start_l, m_x_move_amplitude,
      m_x_move_amplitude_shift);
    y_move_l = wsin(
      m_ssp_time_start_l, m_y_move_period_time,
      m_y_move_phase_shift + 2_pi / m_y_move_period_time * m_ssp_time_start_l, m_y_move_amplitude,
      m_y_move_amplitude_shift);
    z_move_l = wsin(
      m_ssp_time_start_l, m_z_move_period_time,
      m_z_move_phase_shift + 2_pi / m_z_move_period_time * m_ssp_time_start_l, m_z_move_amplitude,
      m_z_move_amplitude_shift);
    c_move_l = wsin(
      m_ssp_time_start_l, m_a_move_period_time,
      m_a_move_phase_shift + 2_pi / m_a_move_period_time * m_ssp_time_start_l, m_a_move_amplitude,
      m_a_move_amplitude_shift);
    x_move_r = wsin(
      m_ssp_time_start_l, m_x_move_period_time,
      m_x_move_phase_shift + 2_pi / m_x_move_period_time * m_ssp_time_start_l, -m_x_move_amplitude,
      -m_x_move_amplitude_shift);
    y_move_r = wsin(
      m_ssp_time_start_l, m_y_move_period_time,
      m_y_move_phase_shift + 2_pi / m_y_move_period_time * m_ssp_time_start_l, -m_y_move_amplitude,
      -m_y_move_amplitude_shift);
    z_move_r = wsin(
      m_ssp_time_start_r, m_z_move_period_time,
      m_z_move_phase_shift + 2_pi / m_z_move_period_time * m_ssp_time_start_r, m_z_move_amplitude,
      m_z_move_amplitude_shift);
    c_move_r = wsin(
      m_ssp_time_start_l, m_a_move_period_time,
      m_a_move_phase_shift + 2_pi / m_a_move_period_time * m_ssp_time_start_l, -m_a_move_amplitude,
      -m_a_move_amplitude_shift);
  } else if (m_time <= m_ssp_time_end_l) {
    x_move_l = wsin(
      m_time, m_x_move_period_time,
      m_x_move_phase_shift + 2_pi / m_x_move_period_time * m_ssp_time_start_l, m_x_move_amplitude,
      m_x_move_amplitude_shift);
    y_move_l = wsin(
      m_time, m_y_move_period_time,
      m_y_move_phase_shift + 2_pi / m_y_move_period_time * m_ssp_time_start_l, m_y_move_amplitude,
      m_y_move_amplitude_shift);
    z_move_l = wsin(
      m_time, m_z_move_period_time,
      m_z_move_phase_shift + 2_pi / m_z_move_period_time * m_ssp_time_start_l, m_z_move_amplitude,
      m_z_move_amplitude_shift);
    c_move_l = wsin(
      m_time, m_a_move_period_time,
      m_a_move_phase_shift + 2_pi / m_a_move_period_time * m_ssp_time_start_l, m_a_move_amplitude,
      m_a_move_amplitude_shift);
    x_move_r = wsin(
      m_time, m_x_move_period_time,
      m_x_move_phase_shift + 2_pi / m_x_move_period_time * m_ssp_time_start_l, -m_x_move_amplitude,
      -m_x_move_amplitude_shift);
    y_move_r = wsin(
      m_time, m_y_move_period_time,
      m_y_move_phase_shift + 2_pi / m_y_move_period_time * m_ssp_time_start_l, -m_y_move_amplitude,
      -m_y_move_amplitude_shift);
    z_move_r = wsin(
      m_ssp_time_start_r, m_z_move_period_time,
      m_z_move_phase_shift + 2_pi / m_z_move_period_time * m_ssp_time_start_r, m_z_move_amplitude,
      m_z_move_amplitude_shift);
    c_move_r = wsin(
      m_time, m_a_move_period_time,
      m_a_move_phase_shift + 2_pi / m_a_move_period_time * m_ssp_time_start_l, -m_a_move_amplitude,
      -m_a_move_amplitude_shift);
  } else if (m_time <= m_ssp_time_start_r) {
    x_move_l = wsin(
      m_ssp_time_end_l, m_x_move_period_time,
      m_x_move_phase_shift + 2_pi / m_x_move_period_time * m_ssp_time_start_l, m_x_move_amplitude,
      m_x_move_amplitude_shift);
    y_move_l = wsin(
      m_ssp_time_end_l, m_y_move_period_time,
      m_y_move_phase_shift + 2_pi / m_y_move_period_time * m_ssp_time_start_l, m_y_move_amplitude,
      m_y_move_amplitude_shift);
    z_move_l = wsin(
      m_ssp_time_end_l, m_z_move_period_time,
      m_z_move_phase_shift + 2_pi / m_z_move_period_time * m_ssp_time_start_l, m_z_move_amplitude,
      m_z_move_amplitude_shift);
    c_move_l = wsin(
      m_ssp_time_end_l, m_a_move_period_time,
      m_a_move_phase_shift + 2_pi / m_a_move_period_time * m_ssp_time_start_l, m_a_move_amplitude,
      m_a_move_amplitude_shift);
    x_move_r = wsin(
      m_ssp_time_end_l, m_x_move_period_time,
      m_x_move_phase_shift + 2_pi / m_x_move_period_time * m_ssp_time_start_l, -m_x_move_amplitude,
      -m_x_move_amplitude_shift);
    y_move_r = wsin(
      m_ssp_time_end_l, m_y_move_period_time,
      m_y_move_phase_shift + 2_pi / m_y_move_period_time * m_ssp_time_start_l, -m_y_move_amplitude,
      -m_y_move_amplitude_shift);
    z_move_r = wsin(
      m_ssp_time_start_r, m_z_move_period_time,
      m_z_move_phase_shift + 2_pi / m_z_move_period_time * m_ssp_time_start_r, m_z_move_amplitude,
      m_z_move_amplitude_shift);
    c_move_r = wsin(
      m_ssp_time_end_l, m_a_move_period_time,
      m_a_move_phase_shift + 2_pi / m_a_move_period_time * m_ssp_time_start_l, -m_a_move_amplitude,
      -m_a_move_amplitude_shift);
  } else if (m_time <= m_ssp_time_end_r) {
    x_move_l = wsin(
      m_time, m_x_move_period_time,
      m_x_move_phase_shift + 2_pi / m_x_move_period_time * m_ssp_time_start_r + 1_pi,
      m_x_move_amplitude, m_x_move_amplitude_shift);
    y_move_l = wsin(
      m_time, m_y_move_period_time,
      m_y_move_phase_shift + 2_pi / m_y_move_period_time * m_ssp_time_start_r + 1_pi,
      m_y_move_amplitude, m_y_move_amplitude_shift);
    z_move_l = wsin(
      m_ssp_time_end_l, m_z_move_period_time,
      m_z_move_phase_shift + 2_pi / m_z_move_period_time * m_ssp_time_start_l, m_z_move_amplitude,
      m_z_move_amplitude_shift);
    c_move_l = wsin(
      m_time, m_a_move_period_time,
      m_a_move_phase_shift + 2_pi / m_a_move_period_time * m_ssp_time_start_r + 1_pi,
      m_a_move_amplitude, m_a_move_amplitude_shift);
    x_move_r = wsin(
      m_time, m_x_move_period_time,
      m_x_move_phase_shift + 2_pi / m_x_move_period_time * m_ssp_time_start_r + 1_pi,
      -m_x_move_amplitude, -m_x_move_amplitude_shift);
    y_move_r = wsin(
      m_time, m_y_move_period_time,
      m_y_move_phase_shift + 2_pi / m_y_move_period_time * m_ssp_time_start_r + 1_pi,
      -m_y_move_amplitude, -m_y_move_amplitude_shift);
    z_move_r = wsin(
      m_time, m_z_move_period_time,
      m_z_move_phase_shift + 2_pi / m_z_move_period_time * m_ssp_time_start_r, m_z_move_amplitude,
      m_z_move_amplitude_shift);
    c_move_r = wsin(
      m_time, m_a_move_period_time,
      m_a_move_phase_shift + 2_pi / m_a_move_period_time * m_ssp_time_start_r + 1_pi,
      -m_a_move_amplitude, -m_a_move_amplitude_shift);
  } else {
    x_move_l = wsin(
      m_ssp_time_end_r, m_x_move_period_time,
      m_x_move_phase_shift + 2_pi / m_x_move_period_time * m_ssp_time_start_r + 1_pi,
      m_x_move_amplitude, m_x_move_amplitude_shift);
    y_move_l = wsin(
      m_ssp_time_end_r, m_y_move_period_time,
      m_y_move_phase_shift + 2_pi / m_y_move_period_time * m_ssp_time_start_r + 1_pi,
      m_y_move_amplitude, m_y_move_amplitude_shift);
    z_move_l = wsin(
      m_ssp_time_end_l, m_z_move_period_time,
      m_z_move_phase_shift + 2_pi / m_z_move_period_time * m_ssp_time_start_l, m_z_move_amplitude,
      m_z_move_amplitude_shift);
    c_move_l = wsin(
      m_ssp_time_end_r, m_a_move_period_time,
      m_a_move_phase_shift + 2_pi / m_a_move_period_time * m_ssp_time_start_r + 1_pi,
      m_a_move_amplitude, m_a_move_amplitude_shift);
    x_move_r = wsin(
      m_ssp_time_end_r, m_x_move_period_time,
      m_x_move_phase_shift + 2_pi / m_x_move_period_time * m_ssp_time_start_r + 1_pi,
      -m_x_move_amplitude, -m_x_move_amplitude_shift);
    y_move_r = wsin(
      m_ssp_time_end_r, m_y_move_period_time,
      m_y_move_phase_shift + 2_pi / m_y_move_period_time * m_ssp_time_start_r + 1_pi,
      -m_y_move_amplitude, -m_y_move_amplitude_shift);
    z_move_r = wsin(
      m_ssp_time_end_r, m_z_move_period_time,
      m_z_move_phase_shift + 2_pi / m_z_move_period_time * m_ssp_time_start_r, m_z_move_amplitude,
      m_z_move_amplitude_shift);
    c_move_r = wsin(
      m_ssp_time_end_r, m_a_move_period_time,
      m_a_move_phase_shift + 2_pi / m_a_move_period_time * m_ssp_time_start_r + 1_pi,
      -m_a_move_amplitude, -m_a_move_amplitude_shift);
  }

  reset_angles();

  if (m_x_move_amplitude != 0) {
    angles[tachimawari::joint::JointId::RIGHT_SHOULDER_PITCH] = keisan::make_radian(
      wsin(m_time, m_period_time, 1.5_pi, -m_x_move_amplitude * m_arm_swing_gain, 0));
    angles[tachimawari::joint::JointId::LEFT_SHOULDER_PITCH] = keisan::make_radian(
      wsin(m_time, m_period_time, 1.5_pi, m_x_move_amplitude * m_arm_swing_gain, 0));
  }

  // reset pause state on new walk cycle
  if (m_time == 0) {
    has_paused_this_cycle = false;
    is_paused = false;
    pause_counter = 0;
    phase_on_pause = WalkPhase::DOUBLE_SUPPORT;
    do_walk_in_place = false;
  }

  double roll_abs = std::fabs(imu_roll.degree());

  if (!is_paused && should_enable_roll_pause(roll_abs)) {
    is_paused = true;
    phase_on_pause = actual_walk_phase;
    pause_counter = 0;
    has_paused_this_cycle = true;
    do_walk_in_place = true;
  }

  if (is_paused) {
    // before resuming, move period time according to the supporting foot during pause
    if (roll_abs <= roll_resume_threshold && actual_walk_phase != phase_on_pause) {
      if (phase_on_pause == WalkPhase::RIGHT_SUPPORT) {
        m_time = m_ssp_time_start_r - time_unit;
      } else if (phase_on_pause == WalkPhase::LEFT_SUPPORT) {
        m_time = m_ssp_time_start_l - time_unit;
      } else {
        m_time = 0;
      }
      is_paused = false;
      pause_counter = 0;
    } else if (pause_counter >= max_pause_counter) {
      is_paused = false;
      pause_counter = 0;
    }
  }

  if (m_real_running) {
    if(!is_paused || !pause_enable){
      m_time += time_unit;
      if (m_time >= m_period_time) m_time = 0;
    } else {
      pause_counter++;
    }
  } else {
    m_time = 0;
  }

  // equalize both leg's height during pause to ensure stability
  if (is_paused && pause_enable){
    double landing_factor = 1.0 - (std::min(pause_counter, 10) / 10.0);
    z_move_l *= landing_factor;
    z_move_r *= landing_factor;
  }

  if (do_walk_in_place) {
    x_move_l = 0.0;  x_move_r = 0.0;
    y_move_l = 0.0;  y_move_r = 0.0;
    c_move_l = 0.0;  c_move_r = 0.0;
  }

  // In walk kick inject
  bool in_left_swing  = (m_time > m_ssp_time_start_l && m_time <= m_ssp_time_end_l);
  bool in_right_swing = (m_time > m_ssp_time_start_r && m_time <= m_ssp_time_end_r);

  bool kick_leg_is_left  = (kick_leg == KickLeg::LEFT || kick_leg == KickLeg::LEFT_CENTER);
  bool kick_leg_is_right = (kick_leg == KickLeg::RIGHT || kick_leg == KickLeg::RIGHT_CENTER);

  bool is_center_kick = (kick_leg == KickLeg::LEFT_CENTER || kick_leg == KickLeg::RIGHT_CENTER);

  if (kick_state == KickState::KICKING) {
    bool still_in_swing = (kick_leg_is_left && in_left_swing) || (kick_leg_is_right && in_right_swing);

    if (still_in_swing) {
      double swing_start = kick_leg_is_left ? m_ssp_time_start_l : m_ssp_time_start_r;
      double swing_end = kick_leg_is_left ? m_ssp_time_end_l : m_ssp_time_end_r;
      double swing_progress = (m_time - swing_start) / (swing_end - swing_start);

      double kick_z_profile = sin(swing_progress * M_PI);

      double x_progress;
      if (swing_progress < kick_start_ratio) {
        x_progress = 0.0;
      } else if (swing_progress > kick_return_ratio) {
        x_progress = 1.0;
      } else {
        x_progress = (swing_progress - kick_start_ratio) / (kick_return_ratio - kick_start_ratio);
      }
      double kick_x_profile = sin(x_progress * M_PI);

      double kick_y_profile = 0.0;
      if (is_center_kick) {
        if (swing_progress < kick_start_ratio) {
          double t = swing_progress / kick_start_ratio;
          kick_y_profile = sin(t * M_PI / 2.0);
        } else if (swing_progress <= kick_return_ratio) {
          kick_y_profile = 1.0;
        } else {
          double t = (swing_progress - kick_return_ratio) / (1.0 - kick_return_ratio);
          kick_y_profile = cos(t * M_PI / 2.0);
        }
      }

      double kick_x = kick_x_amplitude * kick_x_profile;
      double kick_y = kick_y_amplitude * kick_y_profile;
      double kick_z = kick_z_amplitude * kick_z_profile;

      printf(
        "[KICK KICKING] leg=%s | progress=%.3f | x_prog=%.3f | x_prof=%.3f | y_prof=%.3f | z_prof=%.3f | kick_x=%.2f kick_y=%.2f kick_z=%.2f\n",
        kick_leg_is_right ? "RIGHT" : "LEFT",
        swing_progress, x_progress, kick_x_profile, kick_y_profile, kick_z_profile,
        kick_x, kick_y, kick_z);

      if (kick_leg_is_right) {
        x_move_r = kick_x;
        z_move_r = kick_z;
        if (is_center_kick) y_move_r =  kick_y;
      } else {
        x_move_l = kick_x;
        z_move_l = kick_z;
        if (is_center_kick) y_move_l = -kick_y;
      }
    }
  }

  if (kick_state == KickState::DONE && m_time == 0) {
    kick_state = KickState::IDLE;
    printf("[KICK DONE→IDLE]\n");
  }

  keisan::Point3 translation_target;
  translation_target.x = x_swap + x_move_r + x_offset;
  translation_target.y = y_swap + y_move_r - y_offset / 2;
  translation_target.z = z_swap + z_move_r + z_offset;

  keisan::Euler<double> rotation_target;
  rotation_target.roll = keisan::make_radian(a_swap + a_move_r - roll_offset.radian() / 2);
  rotation_target.pitch = keisan::make_radian(b_swap + b_move_r + pitch_offset.radian());
  rotation_target.yaw = keisan::make_radian(c_swap + c_move_r - yaw_offset.radian() / 2);

  // compute angles
  if (!compute_inverse_kinematic(RIGHT_LEG, translation_target, rotation_target)) {
    return false;
  }

  translation_target.x = x_swap + x_move_l + x_offset;
  translation_target.y = y_swap + y_move_l + y_offset / 2;
  translation_target.z = z_swap + z_move_l + z_offset;
  rotation_target.roll = keisan::make_radian(a_swap + a_move_l + roll_offset.radian() / 2);
  rotation_target.pitch = keisan::make_radian(b_swap + b_move_l + pitch_offset.radian());
  rotation_target.yaw = keisan::make_radian(c_swap + c_move_l + yaw_offset.radian() / 2);

  if (!compute_inverse_kinematic(LEFT_LEG, translation_target, rotation_target)) {
    return false;
  }

  return true;
}

bool Kinematic::should_enable_roll_pause(double roll_abs) const
{
  return pause_enable &&
         roll_abs > roll_pause_threshold &&
         actual_walk_phase != WalkPhase::DOUBLE_SUPPORT &&
         x_move <= max_pause_speed &&
         !has_paused_this_cycle;
}

void Kinematic::trigger_kick(KickLeg leg)
{
  if (kick_state != KickState::IDLE && kick_state != KickState::DONE) {
    return;
  }
  kick_leg = leg;
  kick_state = KickState::PREPARE;
}

}  // namespace aruku
