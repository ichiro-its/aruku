// Copyright (c) 2021 Ichiro ITS
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

#ifndef ARUKU__WALKING__NODE__WALKING_NODE_HPP_
#define ARUKU__WALKING__NODE__WALKING_NODE_HPP_

#include <memory>
#include <string>

#include "aruku/walking/node/walking_manager.hpp"
#include "aruku/walking/process/kinematic.hpp"
#include "aruku_interfaces/msg/point2.hpp"
#include "aruku_interfaces/msg/set_walking.hpp"
#include "aruku_interfaces/msg/status.hpp"
#include "rclcpp/rclcpp.hpp"
#include "kansei_interfaces/msg/status.hpp"
#include "kansei_interfaces/msg/unit.hpp"
#include "tachimawari_interfaces/msg/set_joints.hpp"

namespace aruku
{

class WalkingNode
{
public:
  using Point2 = aruku_interfaces::msg::Point2;
  using SetJoints = tachimawari_interfaces::msg::SetJoints;
  using SetWalking = aruku_interfaces::msg::SetWalking;
  using MeasurementStatus = kansei_interfaces::msg::Status;
  using WalkingStatus = aruku_interfaces::msg::Status;
  using Unit = kansei_interfaces::msg::Unit;

  static std::string get_node_prefix();
  static std::string set_walking_topic();
  static std::string status_topic();
  static std::string set_odometry_topic();

  explicit WalkingNode(
    rclcpp::Node::SharedPtr node, std::shared_ptr<WalkingManager> walking_manager);

  void update();

  void set_action_manager_is_open(bool is_open) { action_manager_is_open = is_open;}

  void publish_joints();
  void publish_status();
private:

  rclcpp::Node::SharedPtr node;

  std::shared_ptr<WalkingManager> walking_manager;

  rclcpp::Subscription<SetWalking>::SharedPtr set_walking_subscriber;
  rclcpp::Publisher<SetJoints>::SharedPtr set_joints_publisher;

  rclcpp::Subscription<Point2>::SharedPtr set_odometry_subscriber;
  rclcpp::Publisher<WalkingStatus>::SharedPtr status_publisher;

  rclcpp::Subscription<MeasurementStatus>::SharedPtr measurement_status_subscriber;
  rclcpp::Subscription<Unit>::SharedPtr unit_subscriber;

  int status;

  bool action_manager_is_open = false;
};

}  // namespace aruku

#endif  // ARUKU__WALKING__NODE__WALKING_NODE_HPP_
