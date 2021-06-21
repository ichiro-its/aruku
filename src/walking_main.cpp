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

#include <nlohmann/json.hpp>

#include <fstream>
#include <iostream>
#include <memory>
#include <string>

int main(int argc, char * argv[])
{
  if (argc < 4) {
    std::cerr << "Please specify the host, port, and the path!" << std::endl;
    return 0;
  }

  std::string host = argv[1];
  int port = std::stoi(argv[2]);
  std::string path = argv[3];
  robocup_client::RobotClient client(host, port);
  if (!client.connect()) {
    std::cerr << "Failed to connect to server on port " << client.get_port() << "!" << std::endl;
    return 1;
  }

  robocup_client::MessageHandler message;
  message.add_sensor_time_step("gyro", 8);
  message.add_sensor_time_step("accelerometer", 8);
  client.send(*message.get_actuator_request());

  auto imu = std::make_shared<kansei::Imu>();
  auto walking = std::make_shared<aruku::Walking>(imu);

  walking->load_data(path);
  walking->initialize();
  walking->start();

  float counter = 1.0;
  bool init_orientation = true;

  while (client.get_tcp_socket()->is_connected()) {
    try {
      std::string file_name =
        path + "walking/" + "main.json";
      std::ifstream file(file_name);
      nlohmann::json main_data = nlohmann::json::parse(file);

      for (auto &[key, val] : main_data.items()) {
        if (key == "Start") {
          if (val == true) {
            if (imu->is_calibrated()) {
              walking->start();

              if (init_orientation) {
                imu->reset_orientation();
                init_orientation = false;
              }
            }
          } else {
            walking->stop();
            init_orientation = true;
          }
        }
        if (key == "X") {
          walking->X_MOVE_AMPLITUDE = val;
        }
        if (key == "Y") {
          walking->Y_MOVE_AMPLITUDE = val;
        }
        if (key == "A") {
          walking->A_MOVE_AMPLITUDE = val;
        }
        if (key == "Aim") {
          walking->A_MOVE_AIM_ON = val;
        }
      }
      walking->load_data(path);

      auto sensors = client.receive();

      double gy[3];
      if (sensors.get()->gyros_size() > 0) {
        auto gyro = sensors.get()->gyros(0);
        gy[0] = gyro.value().x();
        gy[1] = gyro.value().y();
        gy[2] = gyro.value().z();
      }

      double acc[3];
      if (sensors.get()->accelerometers_size() > 0) {
        auto accelerometer = sensors.get()->accelerometers(0);
        acc[0] = accelerometer.value().x();
        acc[1] = accelerometer.value().y();
        acc[2] = accelerometer.value().z();
      }

      float seconds = (sensors.get()->time() + 0.0) / 1000;

      imu->compute_rpy(gy, acc, seconds);
      walking->process();

      std::cout << "pos_x " << walking->POSITION_X << ", pos_y " << walking->POSITION_Y << std::endl;
      std::cout << "orientation " << imu->get_yaw() << std::endl;

      message.clear_actuator_request();
      for (auto joint : walking->get_joints()) {
        std::string joint_name = joint.get_joint_name();
        float position = joint.get_goal_position();

        if (joint_name.find("shoulder_pitch") != std::string::npos) {
          joint_name += " [shoulder]";
        } else if (joint_name.find("hip_yaw") != std::string::npos) {
          joint_name += " [hip]", joint.get_goal_position();
        }

        message.add_motor_position_in_degree(joint_name, position);
      }
      client.send(*message.get_actuator_request());
    } catch (const std::runtime_error & exc) {
      std::cerr << "Runtime error: " << exc.what() << std::endl;
    }
  }

  walking->stop();
}
