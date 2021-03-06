/*********************************************************************
* FabricVision.h
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

#ifndef FABRIC_VISION_H
#define FABRIC_VISION_H

#include <string>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64MultiArray.h>
#include <opencv2/opencv.hpp>
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


typedef struct {
  int low_hue;
  int high_hue;
  int low_saturation;
  int high_saturation;
  int low_value;
  int high_value;
} FilterHSV;

class FabricVision {
public:
  FabricVision();
  void subscribeTopic(const std::string &topic_name);
  void publishTopic(const std::string &topic_name);
  void calibrate();
  void calibrate(int i);
  void calculateVertices();
  cv::VideoCapture openCamera(int device = 0);
  void showCamera(cv::VideoCapture &camera,
                  const std::string &window_name = "Camera Image") const;
  void getFrame();
  void applyFilters();
  void applyFilters(cv::Mat &image);
  void showFrame(const std::string &window_name = "Filtered Image") const;
  void showFrame(const cv::Mat &image,
                 const std::string &window_name = "Image") const;
  cv::Mat openFile(const std::string &filename);
  void showImage(const cv::Mat &image,
                 const std::string &window_name = "Display Image") const;
  void saveFile(const cv::Mat &image, const std::string &filename) const;
  void loadCalibration(const std::string &filename);
  std::vector<std::vector<cv::Point> > findContours(cv::Mat &image) const;
  cv::Mat setLabels(const cv::Mat &src,
                    const std::vector<std::vector<cv::Point> > &contours) const;
  FilterHSV filter() const;
  void setFilter(const FilterHSV &filter);
  void thresholdGUI(const std::string &window_name = "HSV Control");

  cv::Mat image() const;
  void setImage(const cv::Mat& image);

  void embedOrigin(cv::Mat &drawing);
  cv::Mat real_image() const;
  void setReal_image(const cv::Mat& real_image);

private:
  void morphologicalOpening(cv::Mat &image, int radius = 5);
  void morphologicalClosing(cv::Mat &image, int radius = 5);
  void toGray(cv::Mat &image) const;
  void toHSV(cv::Mat &image) const;
  void toBGR(cv::Mat &image) const;
  void toCanny(cv::Mat &image) const;
  void threshold(cv::Mat &image, const FilterHSV &filter) const;
  cv::RotatedRect
  findRectangles(const cv::Mat &image,
                 const std::vector<std::vector<cv::Point> > &contours) const;
  void setLabel(cv::Mat &im, const std::string label,
                const std::vector<cv::Point> &contour) const;
  void undistort(cv::Mat &image);
  double angle(cv::Point pt1, cv::Point pt2, cv::Point pt0) const;
  void calculateVertices(const std_msgs::Bool &msg);

  FilterHSV filter_;
  cv::VideoCapture video_;
  cv::Mat real_image_;
  cv::Mat image_;
  cv::Mat camera_matrix_;
  cv::Mat distortion_matrix_;
  cv::Mat rmat_;
  cv::Mat tvec_;
  double scale_;
  ros::NodeHandle node_;
  ros::Publisher publisher_;
  ros::Subscriber subscriber_;
};
}

#endif
