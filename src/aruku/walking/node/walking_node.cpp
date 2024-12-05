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
#include "aruku/walking/process/kinematic.hpp"
#include "kansei/measurement/measurement.hpp"
#include "keisan/keisan.hpp"
#include "nlohmann/json.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tachimawari/joint/joint.hpp"

namespace aruku
{

std::string WalkingNode::get_node_prefix()
{
  return "walking";
}

std::string WalkingNode::set_walking_topic()
{
  return get_node_prefix() + "/set_walking";
}

std::string WalkingNode::status_topic()
{
  return get_node_prefix() + "/status";
}

std::string WalkingNode::set_odometry_topic()
{
  return get_node_prefix() + "/set_odometry";
}

std::string WalkingNode::delta_position_topic()
{
  return get_node_prefix() + "/delta_position";
}

WalkingNode::WalkingNode(
  rclcpp::Node::SharedPtr node, std::shared_ptr<WalkingManager> walking_manager)
: walking_manager(walking_manager)
{
  set_walking_subscriber = node->create_subscription<SetWalking>(
    set_walking_topic(), 10,
    [this](const SetWalking::SharedPtr message) {
      if (message->run) {
        this->walking_manager->run(
          message->x_move, message->y_move, message->a_move,
          message->aim_on);
      } else {
        this->walking_manager->stop();
      }
    });


  measurement_status_subscriber = node->create_subscription<MeasurementStatus>(
    kansei::measurement::MeasurementNode::status_topic(), 10,
    [this](const MeasurementStatus::SharedPtr message) {
      this->walking_manager->update_orientation(
        keisan::make_degree(message->orientation.yaw));
    });

  status_publisher = node->create_publisher<WalkingStatus>(status_topic(), 10);

  unit_subscriber = node->create_subscription<Unit>(
    "/imu/unit", 10,
    [this](const Unit::SharedPtr message) {
      this->walking_manager->update_gyro(
        keisan::Vector<3>(
          message->gyro.roll, message->gyro.pitch, message->gyro.yaw));
    });

  set_odometry_subscriber = node->create_subscription<Point2>(
    set_odometry_topic(), 10,
    [this](const Point2::SharedPtr message) {
      this->walking_manager->set_position(
        keisan::Point2(message->x, message->y));
    });

  delta_position_publisher = node->create_publisher<Point2>(
    delta_position_topic(), 10);

  set_joints_publisher = node->create_publisher<SetJoints>(
    "/joint/set_joints", 10);
}

void WalkingNode::update()
{
  if (action_manager_is_open) {
    return;
  }

  publish_joints();
  publish_status();
  publish_delta_position();
}

void WalkingNode::publish_joints()
{
  auto joints_msg = SetJoints();

  const auto & joints = walking_manager->get_joints();
  auto & joint_msgs = joints_msg.joints;

  joint_msgs.resize(joints.size());
  for (size_t i = 0; i < joints.size() && i < joint_msgs.size(); ++i) {
    joint_msgs[i].id = joints[i].get_id();
    joint_msgs[i].position = joints[i].get_position();
  }

  joints_msg.control_type = tachimawari::joint::Middleware::FOR_WALKING;

  set_joints_publisher->publish(joints_msg);
}

void WalkingNode::publish_status()
{
  auto kinematic = walking_manager->get_kinematic();
  auto status_msg = WalkingStatus();

  status_msg.is_running = walking_manager->is_running();

  status_msg.x_amplitude = kinematic.get_x_move_amplitude();
  status_msg.y_amplitude = kinematic.get_y_move_amplitude();
  status_msg.a_amplitude = kinematic.get_a_move_amplitude();

  status_msg.odometry.x = walking_manager->get_position().x;
  status_msg.odometry.y = walking_manager->get_position().y;

  status_publisher->publish(status_msg);
}

void WalkingNode::publish_delta_position()
{
  auto delta_position_msg = Point2();

  delta_position_msg.x = walking_manager->get_delta_position().x;
  delta_position_msg.y = walking_manager->get_delta_position().y;

  delta_position_publisher->publish(delta_position_msg);
}

}  // namespace aruku
