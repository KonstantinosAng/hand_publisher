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
#include <opencv2/opencv.hpp>

namespace raad2015 {

typedef struct
{
  int low_hue;
  int high_hue;
  int low_saturation;
  int high_saturation;
  int low_value;
  int high_value;
} FilterHSV;


class FabricVision
{
public:
  FabricVision();
  cv::VideoCapture openCamera(int device = 0) const;
  void showCamera(cv::VideoCapture &camera,
                  const std::string &window_name = "Camera Image") const;
  cv::Mat openFile(const std::string &filename);
  void showImage(const cv::Mat &image,
                 const std::string &window_name = "Display Image") const;
  void saveFile(const cv::Mat &image,
                const std::string &filename) const;
  void toGray(cv::Mat &image) const;
  void toHSV(cv::Mat &image) const;
  void toBGR(cv::Mat &image) const;
  void toCanny(cv::Mat &image) const;
  std::vector<std::vector<cv::Point> > findContours(cv::Mat &image) const;
  cv::Mat setLabels(const cv::Mat& src,
                 const std::vector<std::vector<cv::Point> >& contours) const;
  void threshold(cv::Mat &image,
                 const FilterHSV& filter) const;

  void morphologicalOpening(cv::Mat& image, int radius = 5);
  void morphologicalClosing(cv::Mat& image, int radius = 5);
  FilterHSV filter() const;
  void setFilter(const FilterHSV &filter);
  void thresholdGUI(const std::string &window_name = "HSV Control");
private:
  void setLabel(cv::Mat& im,
                const std::string label,
                const std::vector<cv::Point>& contour) const;
  double angle(cv::Point pt1, cv::Point pt2, cv::Point pt0) const;

  FilterHSV filter_;
};

}

#endif
