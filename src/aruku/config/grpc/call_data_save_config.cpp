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

#include "aruku/config/grpc/call_data_save_config.hpp"

#include "aruku/config/utils/config.hpp"
#include "aruku_interfaces/aruku.grpc.pb.h"
#include "aruku_interfaces/aruku.pb.h"
#include "rclcpp/rclcpp.hpp"

namespace aruku
{
CallDataSaveConfig::CallDataSaveConfig(
  aruku_interfaces::proto::Config::AsyncService * service, grpc::ServerCompletionQueue * cq,
  const std::string & path, const std::shared_ptr<aruku::WalkingManager> & walking_manager)
: CallData(service, cq, path), walking_manager_(walking_manager)
{
  Proceed();
}

void CallDataSaveConfig::AddNextToCompletionQueue()
{
  new CallDataSaveConfig(service_, cq_, path_, walking_manager_);
}

void CallDataSaveConfig::WaitForRequest()
{
  service_->RequestSaveConfig(&ctx_, &request_, &responder_, cq_, cq_, this);
}

void CallDataSaveConfig::HandleRequest()
{
  Config config(path_);
  try {
    nlohmann::ordered_json kinematic_data = nlohmann::ordered_json::parse(request_.json_kinematic());
    nlohmann::ordered_json walking_data = nlohmann::ordered_json::parse(request_.json_walking());
    config.save_config(kinematic_data, walking_data);
    walking_manager_->load_config(path_);
    RCLCPP_INFO(rclcpp::get_logger("Save config"), "config has been saved!");
  } catch (const nlohmann::json::exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger("Save config"), e.what());
  }
}
}  // namespace aruku
