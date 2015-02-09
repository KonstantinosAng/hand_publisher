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

void HandPublisher::init() {
  try {
    listener_.waitForTransform("/tracker_depth_frame",
                               "/right_hand",
                               ros::Time(0),
                               ros::Duration(10));
  }
  catch (const tf::TransformException &e) {
    ROS_ERROR("%s", e.what());
  }
}

void HandPublisher::publishTopic(const std::string &topic_name) {
  publisher_ = node_.advertise<PointStamped>(topic_name, 10);
//  vector_pub_ = node_.advertise<Vector3Stamped>("/normal_human", 10);
  human_orientation_pub_ = node_.advertise<PointStamped>("/human_normal",10);
}

void HandPublisher::runLoop() {
  ros::Rate rate(30.0);
  while (ros::ok()) {
    this->updateTransform();
    this->sendTransform();
    ros::spinOnce();
    rate.sleep();
  }
}

void HandPublisher::updateTransform() {
  try {
    listener_.lookupTransform("/tracker_depth_frame",
                              "/right_hand",
                              ros::Time(0),
                              tf_tracker_to_right_hand_);
    listener_.lookupTransform("/tracker_depth_frame",
                              "/right_shoulder",
                              ros::Time(0),
                              tf_tracker_to_right_shoulder_);
    listener_.lookupTransform("/tracker_depth_frame",
                              "/left_shoulder",
                              ros::Time(0),
                              tf_tracker_to_left_shoulder_);
    listener_.lookupTransform("/tracker_depth_frame",
                              "/torso",
                              ros::Time(0),
                              tf_tracker_to_torso_);
  }
  catch (const tf::TransformException &e) {
    //ROS_ERROR("%s", e.what());
  }

  Point hand;
  hand.x = tf_tracker_to_right_hand_.getOrigin().getX();
  hand.y = tf_tracker_to_right_hand_.getOrigin().getY();
  hand.z = tf_tracker_to_right_hand_.getOrigin().getZ();

  this->addAverage(hand);
}

void HandPublisher::sendTransform() {
  PointStamped msg;
  msg.header.frame_id = "/tracker_depth_frame";
  msg.header.stamp = ros::Time::now();
  msg.point.x = hand_.x;
  msg.point.y = hand_.y;
  msg.point.z = hand_.z;
  publisher_.publish(msg);
  this->calcOrientation();
}

Point HandPublisher::hand() const
{
  return hand_;
}

void HandPublisher::setHand(const Point& hand)
{
  hand_ = hand;
}

void HandPublisher::addAverage(Point &hand)
{;
  hand_points_.push_front(hand);
  if (hand_points_.size() >= 51)
    hand_points_.pop_back();

  double x = 0, y = 0, z = 0;
  for (size_t i = 0; i < hand_points_.size(); ++i)
  {
    x += hand_points_.at(i).x;
    y += hand_points_.at(i).y;
    z += hand_points_.at(i).z;
  }
  hand_.x = x / hand_points_.size();
  hand_.y = y / hand_points_.size();
  hand_.z = z / hand_points_.size();
}

void HandPublisher::calcOrientation()
{
//  tf_tracker_to_right_shoulder_.getOrigin().getX() - tf_tracker_to_torso_.getOrigin().getX();
  Vector3 torso_to_rshoulder(
        tf_tracker_to_right_shoulder_.getOrigin().getX()-tf_tracker_to_torso_.getOrigin().getX(),
        tf_tracker_to_right_shoulder_.getOrigin().getY()-tf_tracker_to_torso_.getOrigin().getY(),
        tf_tracker_to_right_shoulder_.getOrigin().getZ()-tf_tracker_to_torso_.getOrigin().getZ());

  Vector3 torso_to_lshoulder(
        tf_tracker_to_left_shoulder_.getOrigin().getX()-tf_tracker_to_torso_.getOrigin().getX(),
        tf_tracker_to_left_shoulder_.getOrigin().getY()-tf_tracker_to_torso_.getOrigin().getY(),
        tf_tracker_to_left_shoulder_.getOrigin().getZ()-tf_tracker_to_torso_.getOrigin().getZ());

  Vector3 normal = tf::tfCross(torso_to_lshoulder, torso_to_rshoulder);
  normal.normalize();
//  Vector3 k(0.0, 1.0, 0.0);
//  Vector3 xy_projected = normal - tf::tfDot(normal,k)*k;
//  Vector3Stamped message;
//  message.header.frame_id = "/tracker_depth_frame";
//  message.header.stamp = ros::Time::now();
//  tf::vector3TFToMsg(xy_projected.normalize(),message.vector);
//  vector_pub_.publish(message);

  PointStamped test;
  test.header.frame_id = "/torso";
  test.header.stamp = ros::Time::now();
  test.point.x = normal.getX();
  test.point.y = normal.getY();
  test.point.z = normal.getZ();
  human_orientation_pub_.publish(test);

//  PoseStamped rviz_pose;
//  rviz_pose.header.frame_id = "/torso";
//  rviz_pose.header.stamp = ros::Time::now();
//  rviz_pose.pose.position.x = 0.0;
//  rviz_pose.pose.position.y = 0.0;
//  rviz_pose.pose.position.z = 0.0;
//  rviz_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(normal.angle(Vector3(1,0,0)), normal.angle(Vector3(0,1,0)), normal.angle(Vector3(0,0,1)));
//  rviz_pub_.publish(rviz_pose);
}

}
