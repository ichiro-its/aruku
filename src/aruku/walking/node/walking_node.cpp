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

#include <memory>
#include <string>

#include "aruku/walking/node/walking_node.hpp"

#include "aruku/walking/node/walking_manager.hpp"
#include "aruku_interfaces/msg/odometry.hpp"
#include "aruku_interfaces/msg/set_walking.hpp"
#include "kansei_interfaces/msg/orientation.hpp"
#include "kansei_interfaces/msg/unit.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tachimawari/joint/joint.hpp"
#include "tachimawari_interfaces/msg/set_joints.hpp"

namespace aruku
{

WalkingNode::WalkingNode(
  rclcpp::Node::SharedPtr node, std::shared_ptr<WalkingManager> walking_manager)
{
  {
    using aruku_interfaces::msg::SetWalking;

    set_walking_subscriber = node->create_subscription<SetWalking>(
      get_node_prefix() + "/set_walking", 10,
      [this](const SetWalking::SharedPtr message) {
        if (message->run) {
          this->walking_manager->run(
            message->x_move, message->y_move, message->a_move,
            message->aim_on);
        } else {
          this->walking_manager->stop();
        }
      });
  }

  {
    using kansei_interfaces::msg::Orientation;

    orientation_subscriber = node->create_subscription<Orientation>(
      "/measurement/orientation", 10,
      [this](const Orientation::SharedPtr message) {
        this->walking_manager->update_imu(message->orientation.yaw);
      });
  }

  {
    using kansei_interfaces::msg::Unit;

    unit_subscriber = node->create_subscription<Unit>(
      "/imu/unit", 10,
      [this](const Unit::SharedPtr message) {
        this->walking_manager->update_imu(message->gyro.pitch, message->gyro.roll);
      });
  }

  odometry_publisher = node->create_publisher<aruku_interfaces::msg::Odometry>(
    get_node_prefix() + "/odometry", 10);

  set_joints_publisher = node->create_publisher<tachimawari_interfaces::msg::SetJoints>(
    "/joint/set_joints", 10);
}

void WalkingNode::process()
{
  if (walking_manager->process()) {
    if (walking_manager->is_runing()) {
      publish_joints();
      publish_odometry();
    }
  }
}

std::string WalkingNode::get_node_prefix() const
{
  return "walking";
}

void WalkingNode::publish_joints()
{
  auto joints_msg = tachimawari_interfaces::msg::SetJoints();
  joints_msg.control_type = tachimawari::joint::Middleware::FOR_WALKING;

  const auto & joints = walking_manager->get_joints();
  auto & joint_msgs = joints_msg.joints;

  joint_msgs.resize(joints.size());
  for (size_t i = 0; i < joints.size() && i < joint_msgs.size(); ++i) {
    joint_msgs[i].id = joints[i].get_id();
    joint_msgs[i].position = joints[i].get_position();
  }

  set_joints_publisher->publish(joints_msg);
}

void WalkingNode::publish_odometry()
{
  auto odometry_msg = aruku_interfaces::msg::Odometry();

  odometry_msg.position_x = walking_manager->get_position().x;
  odometry_msg.position_y = walking_manager->get_position().y;

  odometry_publisher->publish(odometry_msg);
}

}  // namespace aruku
