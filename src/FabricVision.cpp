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
#include <cmath>

namespace raad2015 {

FabricVision::FabricVision()
{
  filter_.low_hue = 18;
  filter_.low_saturation = 85;
  filter_.low_value = 45;
  filter_.high_hue = 38;
  filter_.high_saturation = 255;
  filter_.high_value = 255;
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
  cv::findContours(image.clone(), contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
  return (contours);
}

cv::Mat FabricVision::setLabels(const cv::Mat& src,
                             const std::vector<std::vector<cv::Point> > &contours) const
{
  // The array for storing the approximation curve
  std::vector<cv::Point> approx;

  // We'll put the labels in this destination image
  cv::Mat dst = src.clone();

  for (int i = 0; i < contours.size(); i++)
  {
    // Approximate contour with accuracy proportional
    // to the contour perimeter
    cv::approxPolyDP(cv::Mat(contours[i]),
                     approx,
                     cv::arcLength(cv::Mat(contours[i]), true) * 0.02,
                     true);

    // Skip small or non-convex objects
    if (std::fabs(cv::contourArea(contours[i])) < 100 || !cv::isContourConvex(approx))
      continue;

    if (approx.size() == 3)
      setLabel(dst, "TRI", contours[i]);    // Triangles

    else if (approx.size() >= 4 && approx.size() <= 6)
    {
      // Number of vertices of polygonal curve
      int vtc = approx.size();

      // Get the degree (in cosines) of all corners
      std::vector<double> cos;
      for (int j = 2; j < vtc+1; j++)
        cos.push_back(angle(approx[j%vtc], approx[j-2], approx[j-1]));

      // Sort ascending the corner degree values
      std::sort(cos.begin(), cos.end());

      // Get the lowest and the highest degree
      double mincos = cos.front();
      double maxcos = cos.back();

      // Use the degrees obtained above and the number of vertices
      // to determine the shape of the contour
      if (vtc == 4 && mincos >= -0.1 && maxcos <= 0.3)
      {
        // Detect rectangle or square
        cv::Rect r = cv::boundingRect(contours[i]);
        double ratio = std::abs(1 - (double)r.width / r.height);

        setLabel(dst, ratio <= 0.02 ? "SQU" : "RECT", contours[i]);
      }
      else if (vtc == 5 && mincos >= -0.34 && maxcos <= -0.27)
        setLabel(dst, "PENTA", contours[i]);
      else if (vtc == 6 && mincos >= -0.55 && maxcos <= -0.45)
        setLabel(dst, "HEXA", contours[i]);
    }
    else
    {
      // Detect and label circles
      double area = cv::contourArea(contours[i]);
      cv::Rect r = cv::boundingRect(contours[i]);
      int radius = r.width / 2;

      if (std::abs(1 - ((double)r.width / r.height)) <= 0.2 &&
          std::abs(1 - (area / (CV_PI * std::pow(radius, 2)))) <= 0.2)
      {
        setLabel(dst, "CIR", contours[i]);
      }
    }
  }

  return (dst);
}

void FabricVision::threshold(cv::Mat& image,
                             const FilterHSV& filter) const
{
  cv::inRange(image,
              cv::Scalar(filter.low_hue, filter.low_saturation, filter.low_value),
              cv::Scalar(filter.high_hue, filter.high_saturation, filter.high_value),
              image);
}

void FabricVision::thresholdGUI(const std::string &window_name)
{
  cv::namedWindow(window_name.c_str(), cv::WINDOW_AUTOSIZE);
  cv::createTrackbar("Low Hue", window_name.c_str(), &filter_.low_hue, 179);
  cv::createTrackbar("High Hue", window_name.c_str(), &filter_.high_hue, 179);
  cv::createTrackbar("Low Saturation", window_name.c_str(), &filter_.low_saturation, 255);
  cv::createTrackbar("High Saturation", window_name.c_str(), &filter_.high_saturation, 255);
  cv::createTrackbar("Low Value", window_name.c_str(), &filter_.low_value, 255);
  cv::createTrackbar("High Value", window_name.c_str(), &filter_.high_value, 255);
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

void FabricVision::setLabel(cv::Mat& im,
                            const std::string label,
                            const std::vector<cv::Point> &contour) const
{
  int fontface = cv::FONT_HERSHEY_SIMPLEX;
  double scale = 0.4;
  int thickness = 1;
  int baseline = 0;

  cv::Size text = cv::getTextSize(label, fontface, scale, thickness, &baseline);
  cv::Rect r = cv::boundingRect(contour);

  cv::Point pt(r.x + ((r.width - text.width) / 2), r.y + ((r.height + text.height) / 2));
  cv::rectangle(im, pt + cv::Point(0, baseline), pt + cv::Point(text.width, -text.height), CV_RGB(255,255,255), CV_FILLED);
  cv::putText(im, label, pt, fontface, scale, CV_RGB(0,0,0), thickness, 8);
}

double FabricVision::angle(cv::Point pt1, cv::Point pt2, cv::Point pt0) const
{
  double dx1 = pt1.x - pt0.x;
  double dy1 = pt1.y - pt0.y;
  double dx2 = pt2.x - pt0.x;
  double dy2 = pt2.y - pt0.y;
  return (dx1*dx2 + dy1*dy2)/sqrt((dx1*dx1 + dy1*dy1)*(dx2*dx2 + dy2*dy2) + 1e-10);
}
FilterHSV FabricVision::filter() const
{
  return filter_;
}

void FabricVision::setFilter(const FilterHSV &filter)
{
  filter_ = filter;
}


}
