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
#include "aruku/config/grpc/config.hpp"
#include "aruku/walking/node/walking_manager.hpp"
#include "aruku/walking/node/walking_node.hpp"
#include "aruku_interfaces/msg/set_config.hpp"
#include "aruku_interfaces/srv/get_config.hpp"
#include "aruku_interfaces/srv/save_config.hpp"
#include "aruku.grpc.pb.h"
#include "aruku.pb.h"
#include "rclcpp/rclcpp.hpp"

#include "grpc/support/log.h"
#include "grpcpp/grpcpp.h"

using aruku_interfaces::proto::Config;


namespace aruku
{

class ConfigNode
{
public:
  using GetConfig = aruku_interfaces::srv::GetConfig;
  using SaveConfig = aruku_interfaces::srv::SaveConfig;
  using SetConfig = aruku_interfaces::msg::SetConfig;

  explicit ConfigNode(rclcpp::Node::SharedPtr node, const std::string & path);

  void set_config_callback(
    const std::function<void(const SetConfig::SharedPtr)> & callback);

private:
  std::string get_node_prefix() const;

  std::shared_ptr<WalkingManager> walking_manager;
  std::shared_ptr<WalkingNode> walking_node;

  std::shared_ptr<std::thread> thread_;
  std::unique_ptr<grpc::ServerCompletionQueue> cq_;
  std::unique_ptr<grpc::Server> server_;

  aruku_interfaces::proto::Config::AsyncService service_;

  Config config;
  ConfigGrpc config_grpc;
  rclcpp::Node::SharedPtr node;
  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Service<GetConfig>::SharedPtr get_config_server;
  rclcpp::Service<SaveConfig>::SharedPtr save_config_server;
  rclcpp::Subscription<SetConfig>::SharedPtr set_config_subscriber;
};

}  // namespace aruku

#endif  // ARUKU__CONFIG__NODE__CONFIG_NODE_HPP_
