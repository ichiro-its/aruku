/*
 *  Copyright (C) 2010, CCNY Robotics Lab
 *  Ivan Dryanovski <ivan.dryanovski@gmail.com>
 *
 *  http://robotics.ccny.cuny.edu
 *
 *  Based on implementation of Madgwick's IMU and AHRS algorithms.
 *  http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
 *
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <aruku/imu.hpp>
#include <aruku/stateless_orientation.hpp>

#include "math/vector.h"

#include <iostream>
#include <string>
#include <memory>
#include <chrono>
#include <cmath>


namespace imu
{

Imu::Imu() {

  initialized = false;
  filter.setWorldFrame(WorldFrame::ENU);
  filter.setAlgorithmGain(0.1);
  filter.setDriftBiasGain(0.0);
}

void Imu::compute_imu(Robot::Vector3D gy, Robot::Vector3D acc, double dt)
{
  geometry_msgs::msg::Vector3 ang_vel;
  ang_vel.x = gy.X;
  ang_vel.y = gy.Y;
  ang_vel.z = gy.Z;

  geometry_msgs::msg::Vector3 lin_acc;
  lin_acc.x = acc.X;
  lin_acc.y = acc.Y;
  lin_acc.z = acc.Z;

  if (!initialized) {
    geometry_msgs::msg::Quaternion init_q;
    if (!StatelessOrientation::computeOrientation(WorldFrame::ENU, lin_acc, init_q)) {
      std::cout << "The IMU seems to be in free fall, cannot determine gravity direction!" <<
        std::endl;
      return;
    }

    filter.setOrientation(init_q.w, init_q.x, init_q.y, init_q.z);
    initialized = true;
  }

  filter.madgwickAHRSupdateIMU(
    ang_vel.x, ang_vel.y, ang_vel.z,
    lin_acc.x, lin_acc.y, lin_acc.z,
    dt);

  double q0;
  double q1;
  double q2;
  double q3;
  filter.getOrientation(q0, q1, q2, q3);
  tf2::Matrix3x3(tf2::Quaternion(q1,q2,q3,q0)).getRPY(roll, pitch, yaw);
}

double Imu::get_roll()
{
  return roll;
}

double Imu::get_pitch()
{
  return pitch;
}

double Imu::get_yaw()
{
  return yaw;
}

}  // namespace imu
