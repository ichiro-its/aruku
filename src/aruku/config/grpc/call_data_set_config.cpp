#include "aruku/config/grpc/call_data_set_config.hpp"

#include "aruku.grpc.pb.h"
#include "aruku.pb.h"
#include "aruku/config/utils/config.hpp"

namespace aruku {
CallDataSetConfig::CallDataSetConfig(
  aruku_interfaces::proto::Config::AsyncService * service, grpc::ServerCompletionQueue * cq,
  const std::string path, rclcpp::Node::SharedPtr node)
: CallData(service, cq, path), node_(node)
{
  set_config_publisher_ =
    node_->create_publisher<aruku_interfaces::msg::SetWalking>("walking/set_walking", 10);
  Proceed();
}

void CallDataSetConfig::AddNextToCompletionQueue()
{
  new CallDataSetConfig(service_, cq_, path_, node_);
}

void CallDataSetConfig::WaitForRequest()
{
  service_->RequestSetMainConfig(&ctx_, &request_, &responder_, cq_, cq_, this);
}

void CallDataSetConfig::HandleRequest()
{
  Config config(path_);
  // config.save_config
  try {
    bool run = request_.run();
    double x_move = request_.x_move();
    double y_move = request_.y_move();
    double a_move = request_.a_move();
    bool aim_on = request_.aim_on();
    std::cout << "run: " << run << std::endl;
    std::cout << "x_move: " << x_move << std::endl;
    std::cout << "y_move: " << y_move << std::endl;
    std::cout << "a_move: " << a_move << std::endl;
    std::cout << "aim_on: " << aim_on << std::endl;
    aruku_interfaces::msg::SetWalking msg;
    msg.a_move = a_move;
    msg.x_move = x_move;
    msg.y_move = y_move;
    msg.aim_on = aim_on;
    msg.run = run;
    set_config_publisher_->publish(msg);

    RCLCPP_INFO(rclcpp::get_logger("Publish control config"), "control config has been saved!  ");

  } catch (std::ofstream::failure f) {
    RCLCPP_ERROR(rclcpp::get_logger("Publish control config"), f.what());

  } catch (nlohmann::json::exception e) {
    RCLCPP_ERROR(rclcpp::get_logger("Publish control config"), e.what());
  }
}
} // namespace aruku
