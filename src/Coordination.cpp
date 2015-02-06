/*********************************************************************
* Coordination.cpp
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

#include <hand_publisher/Coordination.h>

namespace raad2015{

Coordination::Coordination()
{
  current_state = IDLE;
  // Subscribe to Vision
  vision_sub_ = node_.subscribe("/fabric_vertices",
                                10,
                                &Coordination::fabric,
                                this);

  // Subscribe to hand_sub_
  hand_sub_ = node_.subscribe("/human_points",
                              10,
                              &Coordination::human,
                              this);

  // Subscribe to serial comms
  serial_rsp_ = node_.subscribe("/serial_incoming_messages",
                                10,
                                &Coordination::serial,
                                this);

  // Advertise Publishers
  vision_req_ = node_.advertise<std_msgs::Bool>("/fabric_localization_request",
                                                10);
  serial_cmd_ = node_.advertise<std_msgs::String>("/serial_outgoing_messages",
                                                  10);
}

void Coordination::runLoop()
{
  ros::Rate rate(30);
  while (ros::ok())
  {
    switch (current_state)
    {
    case IDLE:



      break;
    case HUMAN:


      break;
    case COOP:


      break;
    }
    ros::spinOnce();
    rate.sleep();
  }
}

void Coordination::fabric(const Float64MultiArray &msg)
{

}

void Coordination::human(const PointStamped &msg)
{
  // Only keep the positions of the last 1 second
  hand_positions_.push_front(msg);
  std::deque<PointStamped>::iterator iter;
  for (iter = hand_positions_.begin(); iter != hand_positions_.end(); ++iter)
  {
    if (ros::Time::now() - iter->header.stamp < ros::Duration(1.0))
      hand_positions_.erase(iter);
  }
}

void Coordination::serial(const String &msg)
{

}

bool Coordination::nearby(double distance)
{
  // check if hand is near any of the fabric vertices
  // Copy the deque for thread safety!!!
  std::deque<PointStamped> positions = hand_positions_;
  std::deque<PointStamped>::iterator iter;
  for (iter = positions.front(); iter != positions.back(); ++iter)
  {
    double dist_p1 = 0;
  }

}

double Coordination::l2distance(const Point &p1,
                                const Point &p2)
{
  double dx = p1.x - p2.x;
  double dy = p1.y - p2.y;
  double dz = p1.z - p2.z;
  double distance = std::sqrt( std::pow(dx,2) + std::pow(dy,2) + std::pow(dz,2));
  return distance;
}

}
