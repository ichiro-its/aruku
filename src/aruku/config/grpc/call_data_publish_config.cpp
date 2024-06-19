// Copyright (c) 2024 Ichiro ITS
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

#include "aruku/config/grpc/call_data_publish_config.hpp"

#include "aruku_interfaces/aruku.grpc.pb.h"
#include "aruku_interfaces/aruku.pb.h"
#include "nlohmann/json.hpp"
#include "rclcpp/rclcpp.hpp"

namespace aruku {
CallDataPublishConfig::CallDataPublishConfig(
    aruku_interfaces::proto::Config::AsyncService *service,
    grpc::ServerCompletionQueue *cq, const std::string &path,
    rclcpp::Node::SharedPtr node)
    : CallData(service, cq, path), node_(node) {
  set_config_publisher_ =
      node_->create_publisher<aruku_interfaces::msg::SetConfig>(
          "aruku/config/set_config", 10);
  Proceed();
}

void CallDataPublishConfig::AddNextToCompletionQueue() {
  new CallDataPublishConfig(service_, cq_, path_, node_);
}

void CallDataPublishConfig::WaitForRequest() {
  service_->RequestPublishConfig(&ctx_, &request_, &responder_, cq_, cq_, this);
}

void CallDataPublishConfig::HandleRequest() {
  try {
    aruku_interfaces::msg::SetConfig msg;
    msg.json_kinematic = request_.json_kinematic();
    msg.json_walking = request_.json_walking();
    set_config_publisher_->publish(msg);
    RCLCPP_INFO(rclcpp::get_logger("Publish config"),
                "config has been published!  ");
  } catch (const nlohmann::json::exception &e) {
    RCLCPP_ERROR(rclcpp::get_logger("Publish config"), e.what());
  }
}
} // namespace aruku
