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

typedef enum {
  up    = 0,
  down  = 1,
  right = 2,
  left  = 3
}Location;

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
  cv::Mat calibrateExtrinsic(const cv::Mat &image);
  cv::Mat embedOrigin(const cv::Mat &image);
  cv::Mat embedPoints(const cv::Mat &image,
                      const std::vector<cv::Point2f> &pixels,
                      const std::vector<cv::Point3f> &points) const;
  void calculateVertices(const cv::Mat &image, cv::Mat &draw);
  std::vector<cv::Point2f> calculateVertices(const cv::Mat &image);
  std::vector<cv::Point3f> projectTo3D(const std::vector<cv::Point2f> &image_points);
  cv::VideoCapture openCamera(int device = 0);
  void showCamera(cv::VideoCapture &camera,
                  const std::string &window_name = "Camera Image") const;
  cv::Mat getFrame(cv::VideoCapture &camera);
  cv::Mat applyFilters(const cv::Mat &image);
  void showFrame(const cv::Mat &image,
                 const std::string &window_name = "Image") const;
  cv::Mat openFile(const std::string &filename);
  void saveFile(const cv::Mat &image, const std::string &filename) const;
  void loadCalibration(const std::string &filename);
  std::vector<std::vector<cv::Point> > findContours(cv::Mat &image) const;
  cv::Mat setLabels(const cv::Mat &src,
                    const std::vector<std::vector<cv::Point> > &contours) const;
  void thresholdGUI(const std::string &window_name = "HSV Control");
  cv::Mat undistort(const cv::Mat &image);
  void publishTransformation();
private:
  void morphologicalOpening(cv::Mat &image, int radius = 5);
  void morphologicalClosing(cv::Mat &image, int radius = 5);
  void toGray(cv::Mat &image) const;
  void toHSV(cv::Mat &image) const;
  void toBGR(cv::Mat &image) const;
  void toCanny(cv::Mat &image) const;
  void threshold(cv::Mat &image, const FilterHSV &filter) const;
  cv::RotatedRect findRectangles(const cv::Mat &image,
                                 const std::vector<std::vector<cv::Point> > &contours) const;
  void setLabel(cv::Mat &image, const std::string label,
                const Location &loc,
                const cv::Point &point) const;
  void setLabel(cv::Mat &im, const std::string label,
                const std::vector<cv::Point> &contour) const;
  double angle(cv::Point pt1, cv::Point pt2, cv::Point pt0) const;
  void fabricRequest(const std_msgs::Bool &msg);
  void publishResults(const std::vector<cv::Point3f> &vertices);
  cv::Point2f transformPoint(cv::Point2f current, cv::Mat transformation);
  void calculateTransformation();

  FilterHSV filter_;
  cv::Mat camera_matrix_;
  cv::Mat distortion_matrix_;
  cv::Mat image_to_checkerboard_;
  cv::Mat rmat_;
  cv::Mat tvec_;
  cv::Mat rvec_;
  double scale_;
  tf::Transform tf_;

  ros::NodeHandle node_;
  ros::Publisher publisher_;
  ros::Subscriber subscriber_;
  tf::TransformBroadcaster br_;
  bool intrinsic_calibrated_;
  bool extrinsic_calibrated_;
};
}

#endif
