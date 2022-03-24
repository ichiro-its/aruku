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

#include <string>
#include <unordered_map>

#include "aruku/walking/model/kinematic_id.hpp"

#include "tachimawari/joint/model/joint_id.hpp"

namespace aruku
{

using tachimawari::joint::JointId;

const std::unordered_map<uint8_t, size_t> KinematicId::map = {
  {JointId::RIGHT_HIP_YAW, 0},
  {JointId::RIGHT_HIP_ROLL, 1},
  {JointId::RIGHT_HIP_PITCH, 2},
  {JointId::RIGHT_KNEE, 3},
  {JointId::RIGHT_ANKLE_ROLL, 4},
  {JointId::RIGHT_ANKLE_PITCH, 5},
  {JointId::LEFT_HIP_YAW, 6},
  {JointId::LEFT_HIP_ROLL, 7},
  {JointId::LEFT_HIP_PITCH, 8},
  {JointId::LEFT_KNEE, 9},
  {JointId::LEFT_ANKLE_ROLL, 10},
  {JointId::LEFT_ANKLE_PITCH, 11},
  {JointId::RIGHT_SHOULDER_PITCH, 12},
  {JointId::RIGHT_SHOULDER_ROLL, 13},
  {JointId::RIGHT_ELBOW, 14},
  {JointId::LEFT_SHOULDER_PITCH, 15},
  {JointId::LEFT_SHOULDER_ROLL, 16},
  {JointId::LEFT_ELBOW, 17},
};

}  // namespace aruku
