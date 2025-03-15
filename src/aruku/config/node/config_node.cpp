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
#include "aruku/node/aruku_node.hpp"
#include "aruku_interfaces/srv/save_config.hpp"
#include "aruku_interfaces/srv/get_config.hpp"
#include "jitsuyo/config.hpp"
#include "nlohmann/json.hpp"
#include "rclcpp/rclcpp.hpp"

namespace aruku
{

ConfigNode::ConfigNode(rclcpp::Node::SharedPtr node, const std::string & path,
  const std::shared_ptr<WalkingNode> & walking_node, const std::shared_ptr<WalkingManager> & walking_manager)
: node(node), set_config_subscriber(nullptr), walking_node(walking_node), walking_manager(walking_manager)
{
  get_config_server = node->create_service<GetConfig>(
    get_node_prefix() + "/get_config",
    [this, path](GetConfig::Request::SharedPtr request, GetConfig::Response::SharedPtr response) {
      nlohmann::ordered_json walking_data;
      nlohmann::ordered_json kinematic_data;
      if (!jitsuyo::load_config(path, "walking.json", walking_data)) {
        RCLCPP_ERROR(rclcpp::get_logger("Get config server"), "Failed to load walking config");
        return;
      }

      if (!jitsuyo::load_config(path, "kinematic.json", kinematic_data)) {
        RCLCPP_ERROR(rclcpp::get_logger("Get config server"), "Failed to load kinematic config");
        return;
      }

      response->json_walking = walking_data.dump();
      response->json_kinematic = kinematic_data.dump();
    });

  save_config_server = node->create_service<SaveConfig>(
    get_node_prefix() + "/save_config",
    [this, path](SaveConfig::Request::SharedPtr request, SaveConfig::Response::SharedPtr response) {
      nlohmann::ordered_json kinematic_data = nlohmann::ordered_json::parse(request->json_kinematic);
      nlohmann::ordered_json walking_data = nlohmann::ordered_json::parse(request->json_walking);
      response->status = false;

      if (!jitsuyo::save_config(path, "kinematic.json", kinematic_data)) {
        RCLCPP_ERROR(rclcpp::get_logger("Save config server"), "Failed to save kinematic config");
        return;
      }

      if (!jitsuyo::save_config(path, "walking.json", walking_data)) {
        RCLCPP_ERROR(rclcpp::get_logger("Save config server"), "Failed to save walking config");
        return;
      }

      response->status = true;
    });

  app_status_subscriber = node->create_subscription<AppStatus>(
    get_node_prefix() + "/app_status", 10,
    [this, walking_node](const AppStatus::SharedPtr message) {
      walking_node->set_action_manager_is_open(message->action_manager_status);
    });
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
