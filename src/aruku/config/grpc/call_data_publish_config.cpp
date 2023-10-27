#include "aruku/config/grpc/call_data_publish_config.hpp"
#include "aruku.grpc.pb.h"
#include "aruku.pb.h"
#include "aruku/config/utils/config.hpp"
#include "rclcpp/rclcpp.hpp"

namespace aruku {
CallDataPublishConfig::CallDataPublishConfig(
  aruku_interfaces::proto::Config::AsyncService * service, grpc::ServerCompletionQueue * cq,
  const std::string path, rclcpp::Node::SharedPtr node)
: CallData(service, cq, path), node_(node)
{
  set_config_publisher_ =
    node_->create_publisher<aruku_interfaces::msg::SetConfig>("aruku/config/set_config", 10);
  // set_config_publisher_ = node_->create_publisher<aruku_interfaces::msg::SetConfig>("set_config", 10);
  Proceed();
}

void CallDataPublishConfig::AddNextToCompletionQueue()
{
  new CallDataPublishConfig(service_, cq_, path_, node_);
}

void CallDataPublishConfig::WaitForRequest()
{
  service_->RequestPublishConfig(&ctx_, &request_, &responder_, cq_, cq_, this);
}

void CallDataPublishConfig::HandleRequest()
{
  // Config config(path_);
  try {
    aruku_interfaces::msg::SetConfig msg;
    msg.json_kinematic = request_.json_kinematic();
    msg.json_walking = request_.json_walking();
    set_config_publisher_->publish(msg);
    RCLCPP_INFO(rclcpp::get_logger("Publish config"), "config has been published!  ");

  } catch (nlohmann::json::exception e) {
    RCLCPP_ERROR(rclcpp::get_logger("Publish config"), e.what());
  }
}
} // namespace aruku