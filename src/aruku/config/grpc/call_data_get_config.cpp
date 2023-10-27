#include "aruku/config/grpc/call_data_get_config.hpp"

#include "aruku.grpc.pb.h"
#include "aruku.pb.h"
#include "aruku/config/utils/config.hpp"
#include "rclcpp/rclcpp.hpp"

namespace aruku
{
CallDataGetConfig::CallDataGetConfig(
  aruku_interfaces::proto::Config::AsyncService * service, grpc::ServerCompletionQueue * cq,
  const std::string path)
: CallData(service, cq, path)
{
  Proceed();
}

void CallDataGetConfig::AddNextToCompletionQueue() { new CallDataGetConfig(service_, cq_, path_); }

void CallDataGetConfig::WaitForRequest()
{
  service_->RequestGetConfig(&ctx_, &request_, &responder_, cq_, cq_, this);
}

void CallDataGetConfig::HandleRequest()
{
  Config config(path_);

  reply_.set_json_kinematic(config.get_config("kinematic"));
  reply_.set_json_walking(config.get_config("walking"));
  RCLCPP_INFO(rclcpp::get_logger("Get config"), "config has been sent!");
}
}  // namespace aruku
