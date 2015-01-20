/*********************************************************************
* HandPublisher.cpp
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

#include <hand_publisher/HandPublisher.h>

namespace raad2015 {

void HandPublisher::init()
{
  try
  {
    listener_.waitForTransform("/world",
                               "/right_hand",
                               ros::Time(0),
                               ros::Duration(10));
  }
  catch (const tf::TransformException &e)
  {
    ROS_ERROR("%s", e.what());
  }
}

void HandPublisher::publishTopic(const std::string &topic_name)
{
  publisher_ = node_.advertise<std_msgs::String>(topic_name,10);
}

void HandPublisher::runLoop()
{
  ros::Rate rate(10.0);
  while (ros::ok())
  {
    this->updateTransform();
    this->sendTransform();
    ros::spinOnce();
    rate.sleep();
  }
}

void HandPublisher::updateTransform()
{
  try
  {
    listener_.lookupTransform("/world",
                              "/right_hand",
                              ros::Time(0),
                              tf_world_to_right_hand_);
  }
  catch (const tf::TransformException &e)
  {
    ROS_ERROR("%s", e.what());
  }
}

void HandPublisher::sendTransform()
{
  char tf_string[200];
  std_msgs::String msg;
  double x = tf_world_to_right_hand_.getOrigin().x();
  double y = tf_world_to_right_hand_.getOrigin().y();
  double z = tf_world_to_right_hand_.getOrigin().z();
  sprintf(tf_string, "x=%f y=%f z=%f\n", x, y, z);
  msg.data = tf_string;
  publisher_.publish(msg);
}

}
