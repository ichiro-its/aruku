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

#include <google/protobuf/io/zero_copy_stream_impl.h>
#include <google/protobuf/text_format.h>
#include <unistd.h>

#include <aruku/walking.hpp>
#include <kansei/imu.hpp>
#include <robocup_client/robocup_client.hpp>

#include <iostream>
#include <memory>
#include <string>

int main(int argc, char * argv[])
{
  if (argc < 3) {
    std::cerr << "Please specify the host and the port!" << std::endl;
    return 0;
  }

  std::string host = argv[1];
  int port = std::stoi(argv[2]);
  robocup_client::RobotClient client(host, port);
  if (!client.connect()) {
    std::cerr << "Failed to connect to server on port " << client.get_port() << "!" << std::endl;
    return 1;
  }

  robocup_client::MessageHandler message;
  message.add_sensor_time_step("gyro", 8);
  message.add_sensor_time_step("accelerometer", 8);
  client.send(*message.get_actuator_request());

  std::shared_ptr<aruku::Walking> walking = std::make_shared<aruku::Walking>();
  walking->initialize();
  walking->start();

  std::shared_ptr<kansei::Imu> imu = std::make_shared<kansei::Imu>();

  while (client.get_tcp_socket()->is_connected()) {
    try {
      auto sensors = client.receive();

      float gy[3];
      if (sensors.get()->gyros_size() > 0) {
        auto gyro = sensors.get()->gyros(0);
        gy[0] = gyro.value().x();
        gy[1] = gyro.value().y();
        gy[2] = gyro.value().z();
      }

      float acc[3];
      if (sensors.get()->accelerometers_size() > 0) {
        auto accelerometer = sensors.get()->accelerometers(0);
        acc[0] = accelerometer.value().x();
        acc[1] = accelerometer.value().y();
        acc[2] = accelerometer.value().z();
      }

      float seconds = sensors.get()->time();

      imu->compute_rpy(gy, acc, seconds);
      walking->update_orientation(imu->get_yaw());
      walking->process();

      message.clear_actuator_request();
      for (auto joint : walking->get_joints()) {
        if (joint.get_joint_name().find("shoulder_pitch") != std::string::npos) {
          message.add_motor_position_in_radian(
            joint.get_joint_name() + " [shouder]", joint.get_goal_position());
        } else if (joint.get_joint_name().find("hip_yaw") != std::string::npos) {
          message.add_motor_position_in_radian(
            joint.get_joint_name() + " [hip]", joint.get_goal_position());
        } else {
          message.add_motor_position_in_radian(joint.get_joint_name(), joint.get_goal_position());
        }
      }
      client.send(*message.get_actuator_request());
    } catch (const std::runtime_error & exc) {
      std::cerr << "Runtime error: " << exc.what() << std::endl;
    }
  }

  walking->stop();
}
