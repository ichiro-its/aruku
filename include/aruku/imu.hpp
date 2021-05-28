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

#ifndef IMU__IMU_HPP
#define IMU__IMU_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <aruku/imu_filter.hpp>

#include "math/vector.h"

#include <string>
#include <memory>

namespace imu
{

class Imu
{
public:
  explicit Imu();

  void compute_imu(Robot::Vector3D gy, Robot::Vector3D acc, double dt);

  double get_roll();
  double get_pitch();
  double get_yaw();

private:
  bool initialized;
  ImuFilter filter;

  double roll;
  double pitch;
  double yaw;
};

}  // namespace imu

#endif // IMU__IMU_HPP
