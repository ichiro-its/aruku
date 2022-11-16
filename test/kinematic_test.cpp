// Copyright (c) 2021 ICHIRO ITS
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

#include "gtest/gtest.h"

#include "kinematic_test.hpp"

namespace ar = aruku;

TEST(KinematicTest, get_angles)
{
  float max_error = 0.5;

  float current_angles[20] = ar::Kinematic::get_angles();

  for (int i = 0; i < 20; i++) {
    EXPECT_NEAR(0.0, current_angles[i], max_error);
  }
}

TEST(KinematicTest, stop_kinematic)
{
  ar::Kinematic::stop_kinematic();
  ASSERT_EQ(ar::Kinematic::get_running_state(), false);
  ASSERT_EQ(ar::Kinematic::get_x_move_amplitude(), 0.0);
  ASSERT_EQ(ar::Kinematic::get_y_move_amplitude(), 0.0);
}
