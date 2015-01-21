/*********************************************************************
* FabricVision.cpp
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

#include <hand_publisher/FabricVision.h>

namespace raad2015 {

FabricVision::FabricVision() :
  apply_threshold_(false)
{
  lowH_ = 22;
  lowS_ = 105;
  lowV_ = 45;
  highH_ = 38;
  highS_ = 255;
  highV_ = 255;
}

void FabricVision::openCamera(int device)
{
  video_ = cv::VideoCapture(device);
  if (!video_.isOpened())
    ROS_ERROR("Device %d could not be found", device);
}

void FabricVision::showCamera(const std::string &window_name)
{
  cv::namedWindow(window_name.c_str());
  while (true)
  {
    video_.read(image_);
    if (apply_threshold_)
    {
      toHSV();
      threshold(lowH_, lowS_, lowV_, highH_, highS_, highV_);
//      morphologicalOpening(2);
//      morphologicalClosing(2);
    }

    cv::imshow(window_name.c_str(), image_);
    if (cv::waitKey(30) == 1048603) // Escape key
      break;
  }
}

void FabricVision::openFile(const std::string &filename)
{
  image_ = cv::imread(filename);
  if (!image_.data)
    ROS_ERROR("No image data.");
}

void FabricVision::showImage(const std::string &window_name)
{
  cv::namedWindow(window_name.c_str(), cv::WINDOW_AUTOSIZE );
  cv::imshow(window_name.c_str(), image_);
  cv::waitKey(0);
}

void FabricVision::saveFile(const std::string &filename)
{
  cv::imwrite(filename, image_);
}

void FabricVision::toGray()
{
  cv::cvtColor(image_, image_, cv::COLOR_BGR2GRAY );
}

void FabricVision::toHSV()
{
  cv::cvtColor(image_, image_, cv::COLOR_BGR2HSV);
}

void FabricVision::toBGR()
{
  cv::cvtColor(image_, image_, cv::COLOR_HSV2BGR);
}

void FabricVision::threshold(int lowH, int lowS, int lowV,
                             int highH, int highS, int highV)
{
  cv::inRange(image_,
              cv::Scalar(lowH, lowS, lowV),
              cv::Scalar(highH, highS, highV),
              image_);
}

void FabricVision::thresholdGUI(const std::string &window_name)
{
  cv::namedWindow(window_name.c_str(), cv::WINDOW_AUTOSIZE);
  cv::createTrackbar("Low Hue", window_name.c_str(), &lowH_, 179);
  cv::createTrackbar("High Hue", window_name.c_str(), &highH_, 179);
  cv::createTrackbar("Low Saturation", window_name.c_str(), &lowS_, 255);
  cv::createTrackbar("High Saturation", window_name.c_str(), &highS_, 255);
  cv::createTrackbar("Low Value", window_name.c_str(), &lowV_, 255);
  cv::createTrackbar("High Value", window_name.c_str(), &highV_, 255);
  apply_threshold_ = true;
}

void FabricVision::morphologicalOpening(int radius)
{
  cv::erode(image_,
            image_,
            cv::getStructuringElement(cv::MORPH_ELLIPSE,
                                      cv::Size(radius, radius)) );

  cv::dilate( image_,
              image_,
              cv::getStructuringElement(cv::MORPH_ELLIPSE,
                                        cv::Size(radius, radius)) );
}

void FabricVision::morphologicalClosing(int radius)
{
  cv::dilate( image_,
              image_,
              cv::getStructuringElement(cv::MORPH_ELLIPSE,
                                        cv::Size(radius, radius)) );

  cv::erode(image_,
            image_,
            cv::getStructuringElement(cv::MORPH_ELLIPSE,
                                      cv::Size(radius, radius)) );
}

}
