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

#ifndef ARUKU__CONFIG__NODE__CONFIG_NODE_HPP_
#define ARUKU__CONFIG__NODE__CONFIG_NODE_HPP_

#include <memory>
#include <string>

#include "aruku/config/utils/config.hpp"
#include "aruku_interfaces/msg/set_config.hpp"
#include "aruku_interfaces/srv/get_config.hpp"
#include "aruku_interfaces/srv/save_config.hpp"
#include "rclcpp/rclcpp.hpp"

namespace aruku
{

class ConfigNode
{
public:
  using SaveConfig = aruku_interfaces::srv::SaveConfig;
  using GetConfig = aruku_interfaces::srv::GetConfig;

  explicit ConfigNode(rclcpp::Node::SharedPtr node, const std::string & path);

private:
  std::string get_node_prefix() const;

  Config config;

  rclcpp::Service<SaveConfig>::SharedPtr save_config_server;
  rclcpp::Service<GetConfig>::SharedPtr get_config_server;
};

}  // namespace aruku

#endif  // ARUKU__CONFIG__NODE__CONFIG_NODE_HPP_
