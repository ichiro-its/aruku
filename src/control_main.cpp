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

#include <fstream>
#include <string>
#include <memory>

#include "aruku/walking/node/walking_manager.hpp"
#include "aruku/walking/node/walking_node.hpp"
#include "nlohmann/json.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  if (argc < 2) {
    std::cerr << "Please specify the path!" << std::endl;
    return 0;
  }

  std::string path = argv[1];
  auto node = std::make_shared<rclcpp::Node>("aruku_node");

  auto walking_manager = std::make_shared<aruku::WalkingManager>();
  walking_manager->load_config(path);

  aruku::WalkingNode walking_node(node, walking_manager);

  rclcpp::Rate rcl_rate(8ms);
  while (rclcpp::ok()) {
    rcl_rate.sleep();

    rclcpp::spin_some(node);

    std::ifstream file(path + "control.json");
    nlohmann::json data = nlohmann::json::parse(file);

    bool enable = false;
    double x_speed = 0.0;
    double y_speed = 0.0;
    double a_speed = 0.0;

    for (auto &[key, val] : data.items()) {
      if (key == "enable") {
        enable = val;
      } else if (key == "x") {
        x_speed = val;
      } else if (key == "y") {
        y_speed = val;
      } else if (key == "a") {
        a_speed = val;
      }
    }

    file.close();

    if (enable) {
      walking_manager->run(x_speed, y_speed, a_speed);
    } else {
      walking_manager->stop();
    }

    walking_manager->process();

    if (walking_manager->is_runing()) {
      walking_node.update();

      auto joints = walking_manager->get_joints();

      for (const auto & joint : joints) {
        std::cout << "id " << static_cast<int>(joint.get_id()) << ": " <<
          joint.get_position() << "\n";
      }
    } else {
      std::cout << "kinematic failed!\n";
    }
  }

  rclcpp::shutdown();

  return 0;
}
