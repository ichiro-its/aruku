// Copyright (c) 2024 ICHIRO ITS
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

#include <aruku/config/grpc/call_data_set_app_status.hpp>
#include <aruku_interfaces/aruku.grpc.pb.h>
#include <aruku_interfaces/aruku.pb.h>
#include <rclcpp/rclcpp.hpp>

namespace aruku
{
CallDataSetAppStatus::CallDataSetAppStatus(
  aruku_interfaces::proto::Config::AsyncService * service, grpc::ServerCompletionQueue * cq,
  const std::string & path, const std::shared_ptr<aruku::WalkingNode> & walking_node)
: CallData(service, cq, path), walking_node_(walking_node)
{
  Proceed();
}

void CallDataSetAppStatus::AddNextToCompletionQueue()
{
  new CallDataSetAppStatus(service_, cq_, path_, walking_node_);
}

void CallDataSetAppStatus::WaitForRequest()
{
  service_->RequestSetAppStatus(&ctx_, &request_, &responder_, cq_, cq_, this);
}

void CallDataSetAppStatus::HandleRequest()
{
  try {
    walking_node_->set_action_manager_is_open(request_.action_manager_status());
    walking_node_->set_walk_setting_is_open(request_.walk_setting_status());

    RCLCPP_INFO(
      rclcpp::get_logger("Set App Status"), "Received app status!"
    );
  } catch (std::exception e) {
    RCLCPP_ERROR(rclcpp::get_logger("Publish config"), e.what());
  }
}
}  // namespace aruku
