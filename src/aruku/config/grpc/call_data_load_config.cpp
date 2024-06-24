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

#include "aruku/config/grpc/call_data_load_config.hpp"

#include "aruku_interfaces/aruku.grpc.pb.h"
#include "aruku_interfaces/aruku.pb.h"
#include "nlohmann/json.hpp"
#include "rclcpp/rclcpp.hpp"

namespace aruku
{
CallDataLoadConfig::CallDataLoadConfig(
  aruku_interfaces::proto::Config::AsyncService * service, grpc::ServerCompletionQueue * cq,
  const std::string & path, const std::shared_ptr<aruku::WalkingManager> & walking_manager)
: CallData(service, cq, path), walking_manager_(walking_manager)
{
  Proceed();
}

void CallDataLoadConfig::AddNextToCompletionQueue()
{
  new CallDataLoadConfig(service_, cq_, path_, walking_manager_);
}

void CallDataLoadConfig::WaitForRequest()
{
  service_->RequestLoadConfig(&ctx_, &request_, &responder_, cq_, cq_, this);
}

void CallDataLoadConfig::HandleRequest()
{
  try {
    walking_manager_->load_config(path_);
    RCLCPP_INFO(rclcpp::get_logger("Load config"), "config has been loaded!");
  } catch (const nlohmann::json::exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger("Load config"), e.what());
  }
}
}  // namespace aruku
