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

FabricVision::FabricVision()
{
  lowH_ = 22;
  lowS_ = 105;
  lowV_ = 45;
  highH_ = 38;
  highS_ = 255;
  highV_ = 255;
}

cv::VideoCapture FabricVision::openCamera(int device) const
{
  cv::VideoCapture video = cv::VideoCapture(device);
  if (!video.isOpened())
    ROS_ERROR("Device %d could not be found", device);
  return (video);
}

void FabricVision::showCamera(cv::VideoCapture &camera,
                              const std::string &window_name) const
{
  cv::Mat image;
  cv::namedWindow(window_name.c_str());
  while (true)
  {
    camera.read(image);
    cv::imshow(window_name.c_str(), image);
    if (cv::waitKey(30) == 1048603) // Escape key
      break;
  }
}

cv::Mat FabricVision::openFile(const std::string &filename)
{
  cv::Mat image = cv::imread(filename);
  if (!image.data)
    ROS_ERROR("No image data.");
  return (image);
}

void FabricVision::showImage(const cv::Mat &image,
                             const std::string &window_name) const
{
  cv::namedWindow(window_name.c_str(), cv::WINDOW_AUTOSIZE );
  cv::imshow(window_name.c_str(), image);
  cv::waitKey(0);
}

void FabricVision::saveFile(const cv::Mat &image,
                            const std::string &filename) const
{
  cv::imwrite(filename, image);
}

void FabricVision::toGray(cv::Mat& image) const
{
  cv::cvtColor(image, image, cv::COLOR_BGR2GRAY );
}

void FabricVision::toHSV(cv::Mat& image) const
{
  cv::cvtColor(image, image, cv::COLOR_BGR2HSV);
}

void FabricVision::toBGR(cv::Mat& image) const
{
  cv::cvtColor(image, image, cv::COLOR_HSV2BGR);
}

void FabricVision::toCanny(cv::Mat& image) const
{
  cv::Canny(image, image, 0, 50, 5);
}

std::vector<std::vector<cv::Point> > FabricVision::findContours(cv::Mat& image) const
{
  std::vector<std::vector<cv::Point> > contours;
  cv::findContours(image, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
  return (contours);
}

void FabricVision::threshold(cv::Mat& image,
                             int lowH, int lowS, int lowV,
                             int highH, int highS, int highV) const
{
  cv::inRange(image,
              cv::Scalar(lowH, lowS, lowV),
              cv::Scalar(highH, highS, highV),
              image);
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
}

void FabricVision::morphologicalOpening(cv::Mat& image,
                                        int radius)
{
  cv::erode(image,
            image,
            cv::getStructuringElement(cv::MORPH_ELLIPSE,
                                      cv::Size(radius, radius)) );

  cv::dilate( image,
              image,
              cv::getStructuringElement(cv::MORPH_ELLIPSE,
                                        cv::Size(radius, radius)) );
}

void FabricVision::morphologicalClosing(cv::Mat& image,
                                        int radius)
{
  cv::dilate( image,
              image,
              cv::getStructuringElement(cv::MORPH_ELLIPSE,
                                        cv::Size(radius, radius)) );

  cv::erode(image,
            image,
            cv::getStructuringElement(cv::MORPH_ELLIPSE,
                                      cv::Size(radius, radius)) );
}

}
