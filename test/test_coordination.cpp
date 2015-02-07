/*********************************************************************
* test_hand.cpp
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2015, University of Patras
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of University of Patras nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Authors: Aris Synodinos
*********************************************************************/

#include <gtest/gtest.h>
#include <hand_publisher/Coordination.h>
#include <hand_publisher/config.h>

using namespace raad2015;
using namespace std;

TEST(Coordination, findMinValue) {
  Coordination coord;
  std::vector<double> test;
  test.push_back(23);
  test.push_back(12);
  test.push_back(54);
  test.push_back(3);
  test.push_back(8);
  double value;
  EXPECT_EQ(3, coord.findMinValue(test,value));
  EXPECT_FLOAT_EQ(3.0, value);
}

TEST(Coordination, findMaxValue) {
  Coordination coord;
  std::vector<double> test;
  test.push_back(-23);
  test.push_back(-12);
  test.push_back(-54);
  test.push_back(-3);
  test.push_back(-8);
  double value;
  EXPECT_EQ(3, coord.findMaxValue(test,value));
  EXPECT_FLOAT_EQ(-3, value);
}

TEST(Coordination, l2distance) {
  Coordination coord;
  Point p1;
  p1.x = 1;
  p1.y = 1;
  p1.z = 0;
  Point p2;
  p2.x = 1;
  p2.y = 3;
  p2.z = 2;
  EXPECT_FLOAT_EQ(std::sqrt(8), coord.l2distance(p1,p2));
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "google_test_coord");
  return RUN_ALL_TESTS();
}
