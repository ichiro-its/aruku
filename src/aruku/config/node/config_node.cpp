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
#include <memory>
#include <string>

#include "aruku/config/node/config_node.hpp"
#include "aruku/config/grpc/config.hpp"
#include "aruku/node/aruku_node.hpp"
#include "aruku/config/utils/config.hpp"
#include "aruku/config/grpc/config.hpp"
#include "aruku_interfaces/srv/save_config.hpp"
#include "aruku_interfaces/srv/get_config.hpp"
#include "nlohmann/json.hpp"
#include "rclcpp/rclcpp.hpp"

namespace aruku
{

ConfigNode::ConfigNode(rclcpp::Node::SharedPtr node, const std::string & path)
: node(node), config(path), set_config_subscriber(nullptr)
{
  // get_config_server = node->create_service<GetConfig>(
  //   get_node_prefix() + "/get_config",
  //   [this](GetConfig::Request::SharedPtr request, GetConfig::Response::SharedPtr response) {
  //     response->json_walking = this->config.get_config("walking");
  //     response->json_kinematic = this->config.get_config("kinematic");
  //   });

  // save_config_server = node->create_service<SaveConfig>(
  //   get_node_prefix() + "/save_config",
  //   [this](SaveConfig::Request::SharedPtr request, SaveConfig::Response::SharedPtr response) {
  //     try {
  //       nlohmann::json kinematic_data = nlohmann::json::parse(request->json_kinematic);
  //       nlohmann::json walking_data = nlohmann::json::parse(request->json_walking);

  //       this->config.save_config(kinematic_data, walking_data);
  //       response->status = true;
  //     } catch (std::ofstream::failure) {
  //       // TODO(maroqijalil): log it
  //       response->status = false;
  //     } catch (nlohmann::json::exception) {
  //       // TODO(maroqijalil): log it
  //       response->status = false;
  //     }
  //   });

    config_grpc.Run(5050, path);
}

void ConfigNode::set_config_callback(
  const std::function<void(const SetConfig::SharedPtr)> & callback)
{
  set_config_subscriber = node->create_subscription<SetConfig>(
    get_node_prefix() + "/set_config", 10, callback);
}

std::string ConfigNode::get_node_prefix() const
{
  return "aruku/config";
}

}  // namespace aruku
