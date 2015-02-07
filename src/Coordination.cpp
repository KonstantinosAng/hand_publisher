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
#include <limits>


namespace raad2015{

Coordination::Coordination()
{
  current_state_ = IDLE;
  // Subscribe to Vision
  vision_sub_ = node_.subscribe("/fabric_vertices",
                                10,
                                &Coordination::fabric,
                                this);

  // Subscribe to hand_sub_
  hand_sub_ = node_.subscribe("/human_points",
                              10,
                              &Coordination::humanHand,
                              this);

  // Subscribe to serial comms
  serial_rsp_ = node_.subscribe("/serial_incoming_messages",
                                10,
                                &Coordination::serial,
                                this);

  // Subscribe to human_orientation_
  human_orientation_sub_ = node_.subscribe("/human_normal",
                                       10,
                                       &Coordination::humanOrientation,
                                       this);
  // Advertise Publishers
  vision_req_ = node_.advertise<Bool>("/fabric_localization_request",
                                                10);
  serial_cmd_ = node_.advertise<String>("/serial_outgoing_messages",
                                                  10);
  fabric_point0_pub_ = node_.advertise<PointStamped>("/fabric_point0",
                                                    10);
  fabric_point1_pub_ = node_.advertise<PointStamped>("/fabric_point1",
                                                    10);
  fabric_point2_pub_ = node_.advertise<PointStamped>("/fabric_point2",
                                                    10);
  fabric_point3_pub_ = node_.advertise<PointStamped>("/fabric_point3",
                                                    10);
  human_gripped_point_pub_ = node_.advertise<PointStamped>("/human_gripped_point",
                                                    10);
  robot_gripped_point_pub_ = node_.advertise<PointStamped>("/robot_gripped_point",
                                                    10);
  robot_ready_ = false;
  vision_sent_ = false;
  gripped_ = false;
//  ros::Duration(5).sleep();
  // Request from fabric vision node
  while(!vision_sent_ && ros::ok())
  {
    this->requestFabricData();
    ros::Duration(1).sleep();
    ros::spinOnce();
    ros::Duration(1).sleep();
  }
}

void Coordination::runLoop()
{
  ros::Rate rate(10);
  while (ros::ok())
  {
    switch (current_state_)
    {
    case IDLE:
      if (!isHandMoving(0.05) && isNearFabric(0.1))
      {
        this->sendRobotGoal();
        ROS_ERROR("Human Grabbed Cloth, Waiting for Robot!");
        current_state_ = HUMAN;
      }
      break;
    case HUMAN:
      if (robot_ready_) {
        ROS_ERROR("Robot Grabbed Cloth, Starting Cooperation!");
        current_state_ = COOP;
      }
      robot_ready_ = true; // FOR TESTING ONLY!!
      break;
    case COOP:
      if (gripped_)
        this->sendRobotCurrent();
      else
      {
        ROS_ERROR("Human Released Cloth, Stopping!");
        current_state_ = IDLE;
      }
      break;
    }
    this->publishFabricVertices();
    ros::spinOnce();
    rate.sleep();
  }
}

void Coordination::fabric(const Float64MultiArray &msg)
{
  fabric_vertices_.clear();
  for (size_t i = 0; i < msg.data.size() - 1; i = i+2)
  {
    PointStamped temp;
    temp.header.frame_id = "/camera_checkerboard";
    temp.header.stamp = ros::Time::now();
    temp.point.x = msg.data.at(i)/100.0;
    temp.point.y = msg.data.at(i+1)/100.0;
    temp.point.z = 0;
    ROS_DEBUG("Point is %f, %f", temp.point.x, temp.point.y);
    fabric_vertices_.push_back(temp);
  }
  vision_sent_ = true;
}

void Coordination::humanHand(const PointStamped &msg)
{
  if (msg.point.x == 0.0 || msg.point.y == 0.0 || msg.point.z == 0.0)
  {
    ROS_INFO("Lost Human");
  }
  else
  {
    // Only keep the 10 last positions
    hand_positions_.push_front(msg);
    if (hand_positions_.size() >= 30)
      hand_positions_.pop_back();
  }
}

void Coordination::humanOrientation(const PointStamped &msg)
{
  human_orientation_ = msg;
}

void Coordination::serial(const String &msg)
{
  if (msg.data == "REACHED")
    robot_ready_ = true;
  if (msg.data == "RELEASED")
    gripped_ = false;
}

bool Coordination::isNearFabric(double distance)
{
  std::vector<double> distances;
  PointStamped hand_position_tracker = hand_positions_.front();
  PointStamped hand_position_camera;

  // Transform hand position to the same frame
  listener_.transformPoint(fabric_vertices_.at(0).header.frame_id, hand_position_tracker, hand_position_camera);

  for (size_t i = 0; i < fabric_vertices_.size(); ++i)
  {
    double distance = this->l2distance(fabric_vertices_[i].point, hand_position_camera.point);
    distances.push_back(distance);
  }
  int close_element;
  double close_element_distance;
  close_element = this->findMinValue(distances, close_element_distance);
//  ROS_ERROR("Distance X: %f, Distance Y: %f, Distance Z: %f",
//            fabric_vertices_[close_element].point.x - hand_position_camera.point.x,
//            fabric_vertices_[close_element].point.y - hand_position_camera.point.y,
//            fabric_vertices_[close_element].point.z - hand_position_camera.point.z);
  if (close_element_distance < distance)
  {
    human_gripped_point_index_ = close_element;
    gripped_ = true;
    return true;
  }
  return false;
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

int Coordination::findMinValue(const std::vector<double> &data, double &value)
{
  int min_element;
  value = std::numeric_limits<double>::max();
  for (size_t i = 0; i < data.size(); ++i)
  {
    if (data[i] < value)
    {
      value = data[i];
      min_element = i;
    }
  }
  return min_element;
}

int Coordination::findMaxValue(const std::vector<double> &data, double &value)
{
  int max_element;
  value = - std::numeric_limits<double>::max();
  for (size_t i = 0; i < data.size(); ++i)
  {
    if (data[i] > value)
    {
      value = data[i];
      max_element = i;
    }
  }
  return max_element;
}

bool Coordination::isHandMoving(const double &radius)
{
  if (hand_positions_.size() > 5) {
    double min_x, max_x, min_y, max_y, min_z, max_z;
    std::vector<double> hand_x;
    std::vector<double> hand_y;
    std::vector<double> hand_z;
    for (size_t i = 0; i < hand_positions_.size(); ++i)
    {
      hand_x.push_back(hand_positions_[i].point.x);
      hand_y.push_back(hand_positions_[i].point.y);
      hand_z.push_back(hand_positions_[i].point.z);
    }
    this->findMinValue(hand_x, min_x);
    this->findMinValue(hand_y, min_y);
    this->findMinValue(hand_z, min_z);

    this->findMaxValue(hand_x, max_x);
    this->findMaxValue(hand_y, max_y);
    this->findMaxValue(hand_z, max_z);

    double dx = max_x - min_x;
    double dy = max_y - min_y;
    double dz = max_z - min_z;

    if ( dx < radius && dy < radius && dz < radius) {
//      ROS_ERROR("HAND IS STABLE");
      return false;
    }
  }
  return true;
}

void Coordination::sendRobotGoal()
{
  // Publish to serial the Vertex
  PointStamped grip_point_camera = calculateGrippingPoint();
  grip_point_camera.header.stamp = ros::Time::now();
  PointStamped grip_point_robot;
  // Transform point to the robot frame
  listener_.transformPoint("/robot_frame", grip_point_camera, grip_point_robot);

  std_msgs::String msg;
  char char_msg[200];
  sprintf(char_msg, "%f %f %f", grip_point_robot.point.x, grip_point_robot.point.y, grip_point_robot.point.z );
  msg.data = char_msg;
  serial_cmd_.publish(msg);
}

void Coordination::sendRobotCurrent()
{
  ros::Time timecall = ros::Time::now();
  ros::Duration(0.01).sleep();
  // Hand from kinect
  PointStamped current_hand = hand_positions_.front();
  // Human Gripped point from Camera
  PointStamped human_gripped_point = fabric_vertices_[human_gripped_point_index_];
  human_gripped_point.header.stamp = timecall;
  // Hand from Camera
  PointStamped hand_to_camera;
  listener_.transformPoint("/camera_checkerboard", current_hand, hand_to_camera);

  double dx = hand_to_camera.point.x - human_gripped_point.point.x;
  double dy = hand_to_camera.point.y - human_gripped_point.point.y;
  double dz = hand_to_camera.point.z - human_gripped_point.point.z;

  // Robot Gripped point from Camera
  PointStamped robot_gripped_point_cam = fabric_vertices_[robot_gripped_point_index_];
  robot_gripped_point_cam.header.stamp = timecall;
  // Robot Gripped point from robot
  PointStamped robot_gripped_point;
  listener_.transformPoint("/robot_frame", robot_gripped_point_cam, robot_gripped_point);

  robot_cmd_.header.frame_id = "/robot_frame";
  robot_cmd_.header.stamp = timecall;
  robot_cmd_.point.x = robot_gripped_point.point.x - dy;
  robot_cmd_.point.y = robot_gripped_point.point.y + dx;
  robot_cmd_.point.z = robot_gripped_point.point.z + dz;

  std_msgs::String msg;
  char char_msg[200];
  sprintf(char_msg, "%f %f %f", robot_cmd_.point.x, robot_cmd_.point.y, robot_cmd_.point.z );
  msg.data = char_msg;
  serial_cmd_.publish(msg);
}

void Coordination::requestFabricData()
{
  ROS_INFO("Requesting Fabric Localization");
  std_msgs::Bool msg;
  msg.data = true;
  vision_req_.publish(msg);
}

PointStamped Coordination::calculateGrippingPoint()
{
  std::vector<double> angles;
  for (size_t i = 0; i < fabric_vertices_.size(); ++i) {
    if ( i == human_gripped_point_index_)
    {
      angles.push_back(std::numeric_limits<double>::max());
      continue;
    }
    Vector3 edge;
    edge.setX(std::abs(fabric_vertices_[i].point.x - fabric_vertices_[human_gripped_point_index_].point.x));
    edge.setY(std::abs(fabric_vertices_[i].point.y - fabric_vertices_[human_gripped_point_index_].point.y));
    edge.setZ(std::abs(fabric_vertices_[i].point.z - fabric_vertices_[human_gripped_point_index_].point.z));
    Vector3 normal;
    normal.setX(human_orientation_.point.x);
    normal.setY(human_orientation_.point.y);
    normal.setZ(human_orientation_.point.z);
    double angle = tf::tfAngle(edge,normal);
    angles.push_back(angle);
  }
  double angle;
  robot_gripped_point_index_ = this->findMinValue(angles,angle);
  return (fabric_vertices_[robot_gripped_point_index_]);
}

void Coordination::publishFabricVertices()
{
  if (vision_sent_) {
    fabric_point0_pub_.publish(fabric_vertices_[0]);
    fabric_point1_pub_.publish(fabric_vertices_[1]);
    fabric_point2_pub_.publish(fabric_vertices_[2]);
    fabric_point3_pub_.publish(fabric_vertices_[3]);
  }
  if (gripped_) {
    human_gripped_point_pub_.publish(fabric_vertices_[human_gripped_point_index_]);
    if (current_state_ == HUMAN) {
      robot_gripped_point_pub_.publish(fabric_vertices_[robot_gripped_point_index_]);
    }
    if (current_state_ == COOP)
      robot_gripped_point_pub_.publish(robot_cmd_);
  }
}

}
