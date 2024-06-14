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

#include <chrono>
#include <memory>
#include <string>

#include "aruku/node/aruku_node.hpp"

#include "aruku/config/node/config_node.hpp"
#include "aruku/walking/node/walking_manager.hpp"
#include "aruku/walking/node/walking_node.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

namespace aruku
{

ArukuNode::ArukuNode(rclcpp::Node::SharedPtr node)
: node(node), walking_manager(nullptr), walking_node(nullptr), config_node(nullptr)
{
  node_timer = node->create_wall_timer(
    8ms,
    [this]() {
      if (this->walking_manager->process()) {
        this->walking_node->update();
      }
      this->walking_node->publish_status();
    }
  );
}

void ArukuNode::set_walking_manager(std::shared_ptr<WalkingManager> walking_manager)
{
  this->walking_manager = walking_manager;
  walking_node = std::make_shared<WalkingNode>(node, walking_manager);
}

void ArukuNode::run_config_service(const std::string & path)
{
  config_node = std::make_shared<ConfigNode>(node, path, walking_node, walking_manager);

  if (walking_manager) {
    config_node->set_config_callback(
      [this](const aruku_interfaces::msg::SetConfig::SharedPtr message) {
        nlohmann::json kinematic_data = nlohmann::json::parse(message->json_kinematic);
        nlohmann::json walking_data = nlohmann::json::parse(message->json_walking);

        this->walking_manager->set_config(walking_data, kinematic_data);
        this->walking_manager->reinit_joints();
        this->walking_node->update();
      });
  }
}

}  // namespace aruku
