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

#include "aruku/config/grpc/config.hpp"

#include <chrono>
#include <csignal>
#include <string>
#include <future>

#include "aruku/config/utils/config.hpp"
#include "aruku_interfaces/msg/set_config.hpp"
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
  // async_server.
  exit(signum);
}

void ConfigGrpc::Run(uint16_t port, const std::string path, rclcpp::Node::SharedPtr node)
{
  std::string server_address = absl::StrFormat("0.0.0.0:%d", port);

  ServerBuilder builder;
  builder.AddListeningPort(server_address, grpc::InsecureServerCredentials());
  builder.RegisterService(&service_);

  cq_ = builder.AddCompletionQueue();
  server_ = builder.BuildAndStart();
  std::cout << "Server listening on " << server_address << std::endl;

  signal(SIGINT, SignIntHandler);
  thread_ = std::thread([&path, &node, this]() {
    new ConfigGrpc::CallDataGetConfig(&service_, cq_.get(), path);
    new ConfigGrpc::CallDataSaveConfig(&service_, cq_.get(), path);
    new ConfigGrpc::CallDataPublishConfig(&service_, cq_.get(), path, node);
    void * tag;  // uniquely identifies a request.
    bool ok = true;
    while (true) {      
      this->cq_->Next(&tag, &ok);
      if (ok) {
        static_cast<ConfigGrpc::CallDataBase *>(tag)->Proceed();
      }
    }
  });
  std::this_thread::sleep_for(200ms);  
  
  
}

ConfigGrpc::CallDataBase::CallDataBase() {}

template <class ConfigRequest, class ConfigReply>
ConfigGrpc::CallData<ConfigRequest, ConfigReply>::CallData(
  aruku_interfaces::proto::Config::AsyncService * service, grpc::ServerCompletionQueue * cq,
  const std::string path)
: status_(CREATE), service_(service), cq_(cq), responder_(&ctx_), path_(path)
{
}

template <class ConfigRequest, class ConfigReply>
void ConfigGrpc::CallData<ConfigRequest, ConfigReply>::Proceed()
{
  if (status_ == CREATE) {
    status_ = PROCESS;
    WaitForRequest();
  } else if (status_ == PROCESS) {
    AddNextToCompletionQueue();
    HandleRequest();
    status_ = FINISH;
    responder_.Finish(reply_, grpc::Status::OK, this);
  } else {
    GPR_ASSERT(status_ == FINISH);
    delete this;
  }
}

ConfigGrpc::CallDataGetConfig::CallDataGetConfig(
  aruku_interfaces::proto::Config::AsyncService * service, grpc::ServerCompletionQueue * cq,
  const std::string path)
: CallData(service, cq, path)
{
  Proceed();
}

void ConfigGrpc::CallDataGetConfig::AddNextToCompletionQueue()
{
  new ConfigGrpc::CallDataGetConfig(service_, cq_, path_);
}

void ConfigGrpc::CallDataGetConfig::WaitForRequest()
{
  service_->RequestGetConfig(&ctx_, &request_, &responder_, cq_, cq_, this);
}

void ConfigGrpc::CallDataGetConfig::HandleRequest()
{
  Config config(path_);

  reply_.set_json_kinematic(config.get_config("kinematic"));
  reply_.set_json_walking(config.get_config("walking"));
  RCLCPP_INFO(rclcpp::get_logger("GetConfig"), "config has been sent!");
}

ConfigGrpc::CallDataSaveConfig::CallDataSaveConfig(
  aruku_interfaces::proto::Config::AsyncService * service, grpc::ServerCompletionQueue * cq,
  const std::string path)
: CallData(service, cq, path)
{
  Proceed();
}

void ConfigGrpc::CallDataSaveConfig::AddNextToCompletionQueue()
{
  new ConfigGrpc::CallDataSaveConfig(service_, cq_, path_);
}

void ConfigGrpc::CallDataSaveConfig::WaitForRequest()
{
  service_->RequestSaveConfig(&ctx_, &request_, &responder_, cq_, cq_, this);
}

void ConfigGrpc::CallDataSaveConfig::HandleRequest()
{
  Config config(path_);
  try {
    nlohmann::json kinematic_data = nlohmann::json::parse(request_.json_kinematic());
    nlohmann::json walking_data = nlohmann::json::parse(request_.json_walking());

    config.save_config(kinematic_data, walking_data);
    RCLCPP_INFO(rclcpp::get_logger("SaveConfig"), "config has been saved!  ");

  } catch (std::ofstream::failure f) {
    RCLCPP_ERROR(rclcpp::get_logger("SaveConfig"), f.what());

  } catch (nlohmann::json::exception e) {
    RCLCPP_ERROR(rclcpp::get_logger("SaveConfig"), e.what());
  }
}

ConfigGrpc::CallDataPublishConfig::CallDataPublishConfig(
  aruku_interfaces::proto::Config::AsyncService * service, grpc::ServerCompletionQueue * cq,
  const std::string path, rclcpp::Node::SharedPtr node)
: CallData(service, cq, path), node_(node)
{
  set_config_publisher_ =
    node_->create_publisher<aruku_interfaces::msg::SetConfig>("aruku/config/set_config", 10);
  // set_config_publisher_ = node_->create_publisher<aruku_interfaces::msg::SetConfig>("set_config", 10);
  Proceed();
}

void ConfigGrpc::CallDataPublishConfig::AddNextToCompletionQueue()
{
  new ConfigGrpc::CallDataPublishConfig(service_, cq_, path_, node_);
}

void ConfigGrpc::CallDataPublishConfig::WaitForRequest()
{
  service_->RequestPublishConfig(&ctx_, &request_, &responder_, cq_, cq_, this);
}

void ConfigGrpc::CallDataPublishConfig::HandleRequest()
{
  // Config config(path_);
  try {
    nlohmann::json kinematic_data = nlohmann::json::parse(request_.json_kinematic());
    nlohmann::json walking_data = nlohmann::json::parse(request_.json_walking());

    aruku_interfaces::msg::SetConfig msg;
    msg.json_kinematic = kinematic_data.dump();
    msg.json_walking = walking_data.dump();
    set_config_publisher_->publish(msg);
    RCLCPP_INFO(rclcpp::get_logger("Publish Config"), "config has been published!  ");

  } catch (nlohmann::json::exception e) {
    RCLCPP_ERROR(rclcpp::get_logger("Publish Config"), e.what());
  }
}

ConfigGrpc::CallDataSetConfig::CallDataSetConfig(
  aruku_interfaces::proto::Config::AsyncService * service, grpc::ServerCompletionQueue * cq,
  const std::string path)
: CallData(service, cq, path)
{
  set_config_publisher_ =
    node_->create_publisher<aruku_interfaces::msg::SetWalking>("/walking/set_walking/", 10);
  Proceed();
}

void ConfigGrpc::CallDataSetConfig::AddNextToCompletionQueue()
{
  new ConfigGrpc::CallDataSetConfig(service_, cq_, path_);
}

void ConfigGrpc::CallDataSetConfig::WaitForRequest()
{
  service_->RequestSetMainConfig(&ctx_, &request_, &responder_, cq_, cq_, this);
}

void ConfigGrpc::CallDataSetConfig::HandleRequest()
{
  Config config(path_);
  // config.save_config
  try {
    bool run = request_.run();
    double x_move = request_.x_move();
    double y_move = request_.y_move();
    double a_move = request_.a_move();
    bool aim_on = request_.aim_on();
    nlohmann::json set_config_data = {
      {"enable", run},
      {"x_move", x_move},
      {"y_move", y_move},
      {"a_move", a_move},
      {"aim_on", aim_on}
    };
    config.save_control_config(set_config_data);
  
    aruku_interfaces::msg::SetWalking msg;
    msg.a_move = a_move;
    msg.x_move = x_move;
    msg.y_move = y_move;
    msg.aim_on = aim_on;
    msg.run = run;
    set_config_publisher_->publish(msg);

    RCLCPP_INFO(rclcpp::get_logger("SaveConfig"), "control config has been saved!  ");

  } catch (std::ofstream::failure f) {
    RCLCPP_ERROR(rclcpp::get_logger("SaveConfig"), f.what());

  } catch (nlohmann::json::exception e) {
    RCLCPP_ERROR(rclcpp::get_logger("SaveConfig"), e.what());
  }
}

}  // namespace aruku