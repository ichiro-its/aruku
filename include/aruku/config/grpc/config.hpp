// Copyright (c) 2023 Ichiro ITS
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

#ifndef ARUKU__CONFIG__GRPC__CONFIG_HPP_
#define ARUKU__CONFIG__GRPC__CONFIG_HPP_

#include <chrono>
#include <fstream>
#include <iostream>
#include <map>
#include <memory>
#include <string>
#include <thread>
#include <future>

#include "absl/flags/flag.h"
#include "absl/flags/parse.h"
#include "absl/strings/str_format.h"
#include "aruku_interfaces/aruku.grpc.pb.h"
#include "aruku_interfaces/aruku.pb.h"
#include "aruku/walking/process/kinematic.hpp"
#include "grpc/support/log.h"
#include "grpcpp/grpcpp.h"
#include "nlohmann/json.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tachimawari/joint/model/joint.hpp"
#include "aruku_interfaces/msg/set_config.hpp"
#include "aruku_interfaces/msg/set_walking.hpp"
#include "aruku/config/grpc/call_data_base.hpp"
#include "aruku/config/grpc/call_data.hpp"

using aruku_interfaces::proto::Config;

namespace aruku
{
class ConfigGrpc
{
public:
  explicit ConfigGrpc();
  explicit ConfigGrpc(const std::string & path);

  ~ConfigGrpc();

  void Run(uint16_t port, const std::string path, rclcpp::Node::SharedPtr node);

private:
  std::string path;  
  static void SignIntHandler(int signum);
  
  static inline std::unique_ptr<grpc::ServerCompletionQueue> cq_;
  static inline std::unique_ptr<grpc::Server> server_;
  std::thread thread_;
  aruku_interfaces::proto::Config::AsyncService service_;  
};

}  // namespace aruku

#endif  // ARUKU__CONFIG__GRPC__CONFIG_HPP_

