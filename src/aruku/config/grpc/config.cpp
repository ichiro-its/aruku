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


#include <chrono>
#include <csignal>
#include <future>
#include <string>

#include "aruku/config/utils/config.hpp"
#include "aruku/config/grpc/config.hpp"
#include "aruku/config/grpc/call_data_base.hpp"
#include "aruku/config/grpc/call_data_get_config.hpp"
#include "aruku/config/grpc/call_data_set_config.hpp"
#include "aruku/config/grpc/call_data_save_config.hpp"
#include "aruku/config/grpc/call_data_publish_config.hpp"
#include "rclcpp/rclcpp.hpp"

using grpc::ServerBuilder;
using namespace std::chrono_literals;

namespace aruku
{
ConfigGrpc::ConfigGrpc() {}
ConfigGrpc::ConfigGrpc(const std::string & path) : path(path) {}

ConfigGrpc::~ConfigGrpc()
{
  server_->Shutdown();
  cq_->Shutdown();
}

void ConfigGrpc::SignIntHandler(int signum)
{
  server_->Shutdown();
  cq_->Shutdown();  
  exit(signum);
}

void ConfigGrpc::Run(uint16_t port, const std::string path, rclcpp::Node::SharedPtr node)
{
  Config config(path);
  std::string server_address = absl::StrFormat("0.0.0.0:%d", config.get_grpc_config()["port"].get<uint16_t>());

  ServerBuilder builder;
  builder.AddListeningPort(server_address, grpc::InsecureServerCredentials());
  builder.RegisterService(&service_);

  cq_ = builder.AddCompletionQueue();
  server_ = builder.BuildAndStart();
  std::cout << "Server listening on " << server_address << std::endl;

  signal(SIGINT, SignIntHandler);
  thread_ = std::thread([&path, &node, this]() {
    new CallDataGetConfig(&service_, cq_.get(), path);
    new CallDataSaveConfig(&service_, cq_.get(), path);
    new CallDataPublishConfig(&service_, cq_.get(), path, node);
    new CallDataSetConfig(&service_, cq_.get(), path, node);
    void * tag;  // uniquely identifies a request.
    bool ok = true;
    while (true) {
      this->cq_->Next(&tag, &ok);
      if (ok) {
        static_cast<CallDataBase *>(tag)->Proceed();
      }
    }
  });
  std::this_thread::sleep_for(200ms);
}


}  // namespace aruku
