#include "aruku/config/grpc/call_data_save_config.hpp"

#include "aruku.grpc.pb.h"
#include "aruku.pb.h"
#include "aruku/config/utils/config.hpp"
#include "rclcpp/rclcpp.hpp"

namespace aruku {
CallDataSaveConfig::CallDataSaveConfig(
  aruku_interfaces::proto::Config::AsyncService * service, grpc::ServerCompletionQueue * cq,
  const std::string path)
: CallData(service, cq, path)
{
  Proceed();
}

void CallDataSaveConfig::AddNextToCompletionQueue()
{
  new CallDataSaveConfig(service_, cq_, path_);
}

void CallDataSaveConfig::WaitForRequest()
{
  service_->RequestSaveConfig(&ctx_, &request_, &responder_, cq_, cq_, this);
}

void CallDataSaveConfig::HandleRequest()
{
  Config config(path_);
  try {
    nlohmann::json kinematic_data = nlohmann::json::parse(request_.json_kinematic());
    nlohmann::json walking_data = nlohmann::json::parse(request_.json_walking());

    config.save_config(kinematic_data, walking_data);
    RCLCPP_INFO(rclcpp::get_logger("Save config"), " config has been saved!  ");

  } catch (std::ofstream::failure f) {
    RCLCPP_ERROR(rclcpp::get_logger("Save config"), f.what());

  } catch (nlohmann::json::exception e) {
    RCLCPP_ERROR(rclcpp::get_logger("Save config"), e.what());
  }
}
} // namespace aruku