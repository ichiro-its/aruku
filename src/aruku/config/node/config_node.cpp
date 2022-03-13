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

#include <memory>
#include <string>

#include "aruku/config/node/config_node.hpp"

#include "aruku/config/utils/config_util.hpp"
#include "aruku_interfaces/msg/set_config.hpp"
#include "aruku_interfaces/srv/get_config.hpp"
#include "rclcpp/rclcpp.hpp"

namespace aruku
{

ConfigNode::ConfigNode(rclcpp::Node::SharedPtr node, const std::string & path)
: config_util(path)
{
  {
    using aruku_interfaces::srv::GetConfig;

    get_config_server = node->create_service<GetConfig>(
      get_node_prefix() + "/get_config",
      [this](GetConfig::Request::SharedPtr request, GetConfig::Response::SharedPtr response) {
        response->json = this->config_util.get_config();
      });
  }

  {
    using aruku_interfaces::msg::SetConfig;

    set_config_subscriber = node->create_subscription<SetConfig>(
      get_node_prefix() + "/set_config", 10,
      [this](SetConfig::SharedPtr message) {
        this->config_util.set_config(message->name, message->key, message->value);
      });
  }
}

std::string ConfigNode::get_node_prefix() const
{
  return "aruku/config";
}

}  // namespace aruku
