/*********************************************************************
* KinectVision.h
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

#ifndef KINECT_VISION_H
#define KINECT_VISION_H

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <tf/transform_broadcaster.h>

namespace raad2015 {

class Color
{
public:
  static cv::Scalar ColorBlue;
  static cv::Scalar ColorRed;
  static cv::Scalar ColorGreen;
  static cv::Scalar ColorWhite;
  static cv::Scalar ColorBlack;
};

class KinectVision
{
public:
  KinectVision();
  cv::Mat openFile(const std::string &filename);
  void showImage(const cv::Mat &image,
                 const std::string &window_name = "Display Inage") const;
  void loadIntrinsic(const std::string &filename);
  void calibrateExtrinsic(const cv::Mat &image);
  void embedOrigin(cv::Mat &image);
  void saveFile(const cv::Mat &image,
                const std::string &filename) const;
  void publishTransform();
private:
  void imageCallBack(const sensor_msgs::ImageConstPtr& msg);
  void calculateTransformation();

  ros::NodeHandle node_;
  ros::Subscriber sub_;
  tf::TransformBroadcaster br_;
  cv::Mat camera_matrix_;
  cv::Mat distortion_matrix_;
  cv::Mat rmat_;
  cv::Mat tvec_;
  cv::Mat rvec_;
  tf::Transform tf_;
  double scale_;
};

}

#endif // KINECT_VISION_H
