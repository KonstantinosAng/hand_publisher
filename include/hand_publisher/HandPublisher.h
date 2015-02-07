/*********************************************************************
* HandPublisher.h
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

#ifndef HAND_PUBLISHER_H
#define HAND_PUBLISHER_H

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float64MultiArray.h>
#include <eigen3/Eigen/Geometry>

using geometry_msgs::Point;
using geometry_msgs::PointStamped;
using geometry_msgs::Vector3Stamped;
using geometry_msgs::Pose;
using geometry_msgs::PoseStamped;

using tf::Vector3;
using std_msgs::Float64MultiArray;

namespace raad2015 {

class HandPublisher {
public:
  void init();
  void publishTopic(const std::string &topic_name);
  void runLoop();
  void addAverage(Point &hand);
  Point hand() const;
  void setHand(const Point& hand);
  void calcOrientation();

private:
  void updateTransform();

  void sendTransform();

  ros::NodeHandle node_;
  ros::Publisher publisher_;
  ros::Publisher vector_pub_;
  ros::Publisher human_orientation_pub_;
  tf::TransformListener listener_;
  tf::StampedTransform tf_tracker_to_right_hand_;
  tf::StampedTransform tf_tracker_to_right_shoulder_;
  tf::StampedTransform tf_tracker_to_left_shoulder_;
  tf::StampedTransform tf_tracker_to_torso_;

  Point hand_;
  std::deque<Point> hand_points_;
};
}

#endif
