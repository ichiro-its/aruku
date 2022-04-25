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
#include "aruku_interfaces/msg/odometry.hpp"
#include "aruku_interfaces/msg/set_walking.hpp"
#include "aruku_interfaces/msg/status.hpp"
#include "rclcpp/rclcpp.hpp"
#include "kansei_interfaces/msg/axis.hpp"
#include "kansei_interfaces/msg/unit.hpp"
#include "tachimawari_interfaces/msg/set_joints.hpp"

namespace aruku
{

class WalkingNode
{
public:
  using Axis = kansei_interfaces::msg::Axis;
  using Odometry = aruku_interfaces::msg::Odometry;
  using SetJoints = tachimawari_interfaces::msg::SetJoints;
  using SetWalking = aruku_interfaces::msg::SetWalking;
  using Status = aruku_interfaces::msg::Status;
  using Unit = kansei_interfaces::msg::Unit;

  explicit WalkingNode(
    rclcpp::Node::SharedPtr node, std::shared_ptr<WalkingManager> walking_manager);

  void update();

private:
  std::string get_node_prefix() const;

  void publish_joints();
  void publish_odometry();
  void publish_status();

  rclcpp::Node::SharedPtr node;

  std::shared_ptr<WalkingManager> walking_manager;

  rclcpp::Subscription<SetWalking>::SharedPtr set_walking_subscriber;
  rclcpp::Publisher<SetJoints>::SharedPtr set_joints_publisher;

  rclcpp::Publisher<Odometry>::SharedPtr odometry_publisher;
  rclcpp::Publisher<Status>::SharedPtr status_publisher;

  rclcpp::Subscription<Axis>::SharedPtr orientation_subscriber;
  rclcpp::Subscription<Unit>::SharedPtr unit_subscriber;

  int status;
};

}  // namespace aruku

#endif  // ARUKU__WALKING__NODE__WALKING_NODE_HPP_
