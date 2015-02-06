/*********************************************************************
* Coordination.h
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

#ifndef COORDINATION_H
#define COORDINATION_H

#include <ros/ros.h>
#include <deque>
#include <cmath>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>

using geometry_msgs::Point;
using geometry_msgs::PointStamped;
using std_msgs::Float64MultiArray;
using std_msgs::String;

namespace raad2015{

typedef enum {
  IDLE,
  HUMAN,
  COOP
}State_t;

class Coordination
{
public:
  Coordination();
  void runLoop();
private:
  void fabric(const Float64MultiArray &msg);
  void human(const PointStamped &msg);
  void serial(const String &msg);
  bool nearby(double distance);
  double l2distance(const Point &p1, const Point &p2);

  State_t current_state;
  ros::NodeHandle node_;
  // ROS Subscribers
  ros::Subscriber vision_sub_;
  ros::Subscriber hand_sub_;
  ros::Subscriber serial_rsp_;
  // ROS Publishers
  ros::Publisher vision_req_;
  ros::Publisher serial_cmd_;

  std::deque<PointStamped> hand_positions_;
};

}

#endif // COORDINATION_H