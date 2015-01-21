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


class FabricVision
{
public:
  FabricVision();
  void openCamera(int device = 0);
  void showCamera(const std::string &window_name = "Camera Image");
  void openFile(const std::string &filename);
  void showImage(const std::string &window_name = "Display Image");
  void saveFile(const std::string &filename);
  void toGray();
  void toHSV();
  void toBGR();
  void threshold(int lowH, int lowS, int lowV,
                 int highH, int highS, int highV);
  void thresholdGUI(const std::string &window_name = "HSV Control");
  void morphologicalOpening(int radius = 5);
  void morphologicalClosing(int radius = 5);
private:
  cv::Mat image_;
  cv::VideoCapture video_;
  bool apply_threshold_;
  int lowH_, lowS_, lowV_, highH_, highS_, highV_;
};

}

#endif
