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

#ifndef ARUKU__NODE__ARUKU_NODE_HPP_
#define ARUKU__NODE__ARUKU_NODE_HPP_

#include <memory>

#include "aruku/walking/node/walking_manager.hpp"
#include "aruku/walking/node/walking_node.hpp"
#include "rclcpp/rclcpp.hpp"

namespace aruku
{

class ArukuNode
{
public:
  explicit ArukuNode(rclcpp::Node::SharedPtr node);

  void set_walking_manager(std::shared_ptr<WalkingManager> walking_manager);

private:
  rclcpp::Node::SharedPtr node;
  rclcpp::TimerBase::SharedPtr node_timer;

  std::shared_ptr<WalkingNode> walking_node;
};

}  // namespace aruku

#endif  // ARUKU__NODE__ARUKU_NODE_HPP_
