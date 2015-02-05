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
#include <hand_publisher/config.h>

namespace raad2015 {

cv::Scalar Color::ColorBlue(255, 0, 0);
cv::Scalar Color::ColorRed(0, 0, 255);
cv::Scalar Color::ColorGreen(0, 255, 0);
cv::Scalar Color::ColorWhite(255, 255, 255);
cv::Scalar Color::ColorBlack(0, 0, 0);

FabricVision::FabricVision() {
  filter_.low_hue = 18;
  filter_.low_saturation = 45;
  filter_.low_value = 45;
  filter_.high_hue = 38;
  filter_.high_saturation = 255;
  filter_.high_value = 255;
}

void FabricVision::subscribeTopic(const std::string &topic_name) {
  subscriber_ =
      node_.subscribe(topic_name, 10, &FabricVision::calculateVertices, this);
}

void FabricVision::publishTopic(const std::string &topic_name) {
  publisher_ = node_.advertise<std_msgs::Float64MultiArray>(topic_name, 10);
}

void FabricVision::calibrate() {
  cv::Size checkerboard = cv::Size(7, 10);  // A3 Checkerboard
  // Create actual board points
  std::vector<cv::Point3d> board_points;
  for (size_t i = 0; i < 10; ++i)
    for (size_t j = 0; j < 7; ++j)
      board_points.push_back(cv::Point3d(3.4 * i, 3.4 * j, 0.0));

  // Try to find checkerboard
  bool found;
  std::vector<cv::Point2d> found_points;
  ROS_INFO("Waiting for Checkerboard");
  while (ros::ok()) {
    this->getFrame();
    this->toGray(image_);
    found = cv::findChessboardCorners(image_, checkerboard, found_points,
                                      cv::CALIB_CB_FAST_CHECK);
    if (found) {
      ROS_INFO("Found Checkerboard");
      cv::Mat rvec;
      cv::solvePnP(cv::Mat(board_points), cv::Mat(found_points), camera_matrix_,
                   distortion_matrix_, rvec, tvec_, false);
      cv::Rodrigues(rvec, rmat_);
      cv::Mat scale_mat = camera_matrix_ * ( rmat_ * cv::Mat(board_points[0]) + tvec_ );
      scale_ = scale_mat.at<double>(0,2);
      break;
    }
  }
}

void FabricVision::calibrate(int i) {
  cv::Size checkerboard = cv::Size(7, 10);  // A3 Checkerboard
  // Create actual board points
  std::vector<cv::Point3d> board_points;
  for (size_t i = 0; i < 10; ++i)
    for (size_t j = 0; j < 7; ++j)
      board_points.push_back(cv::Point3d(3.4 * i, 3.4 * j, 0.0));

  // Try to find checkerboard
  bool found;
  std::vector<cv::Point2d> found_points;
  ROS_INFO("Waiting for Checkerboard");
  while (ros::ok()) {
    this->toGray(image_);
    found = cv::findChessboardCorners(image_, checkerboard, found_points,
                                      cv::CALIB_CB_FAST_CHECK);
    if (found) {
      ROS_INFO("Found Checkerboard");
      cv::Mat rvec;
      cv::solvePnP(cv::Mat(board_points), cv::Mat(found_points), camera_matrix_,
                   distortion_matrix_, rvec, tvec_);
      cv::Rodrigues(rvec, rmat_);
      cv::Mat scale_mat = camera_matrix_ * ( rmat_ * cv::Mat(board_points[0]) + tvec_ );
      scale_ = scale_mat.at<double>(0,2);
      break;
    }
  }
}

void FabricVision::embedOrigin(cv::Mat &drawing)
{
  cv::Mat projected_origin = cv::Mat(cv::Point3d(0.0,0.0,1.0));
  cv::Mat projected_x = cv::Mat(cv::Point3d(5.0, 0.0, 1.0));
  cv::Mat projected_y = cv::Mat(cv::Point3d(0.0, 5.0, 1.0));
  cv::Mat origin = camera_matrix_ * ( rmat_ * projected_origin + tvec_ );
  origin /= scale_;
  cv::Mat origin_x = camera_matrix_ * ( rmat_ * projected_x + tvec_ );
  origin_x /= scale_;
  cv::Mat origin_y = camera_matrix_ * ( rmat_ * projected_y + tvec_ );
  origin_y /= scale_;
  cv::circle(drawing, cv::Point2d(origin.at<double>(0,0),
                                  origin.at<double>(0,1)),
             3, Color::ColorRed, 5);
  cv::line(drawing,
           cv::Point2d(origin.at<double>(0,0),
                                origin.at<double>(0,1)),
           cv::Point2d(origin_x.at<double>(0,0),
                       origin_x.at<double>(0,1)),
           Color::ColorRed, 3, 8);
  cv::line(drawing,
           cv::Point2d(origin.at<double>(0,0),
                       origin.at<double>(0,1)),
           cv::Point2d(origin_y.at<double>(0,0),
                       origin_y.at<double>(0,1)),
           Color::ColorRed, 3, 8);

  tf::StampedTransform image_frame;
}

void FabricVision::calculateVertices() {
  std_msgs::Float64MultiArray vert_msg;
  cv::Mat image;
  image = image_.clone();
  std::vector<std::vector<cv::Point> > contours;
  contours = findContours(image);
  cv::RotatedRect r = findRectangles(image, contours);
  cv::Point2f vertices[4];
  r.points(vertices);

  for( int j = 0; j < 4; j++ )
    cv::line(real_image_, vertices[j], vertices[(j+1)%4],
        Color::ColorWhite, 1, 8 );
  for( int j = 0; j < 4; j++ )
    cv::circle(real_image_, vertices[j], 3, Color::ColorBlue, 5);

  embedOrigin(real_image_);

  std::string path(PACKAGE_PATH);
  path.append("/samples/identified.jpg");
  this->saveFile(real_image_,path);

  for (int i = 0; i < 4; ++i) {
    cv::Mat vert_pnt = cv::Mat(cv::Point3d(vertices[i].x, vertices[i].y, 1.0));
    cv::Mat result = rmat_.inv() * camera_matrix_.inv() * scale_ * vert_pnt -
                     rmat_.inv() * tvec_;
    vert_msg.data.push_back(result.at<double>(0,0));
    vert_msg.data.push_back(result.at<double>(0,1));
    std::cout << "Result " << result << std::endl;
  }
}

void FabricVision::calculateVertices(const std_msgs::Bool &msg) {
  if (msg.data) {
    std_msgs::Float64MultiArray vert_msg;
    cv::Mat image;
    image = image_.clone();
    std::vector<std::vector<cv::Point> > contours;
    contours = findContours(image);
    cv::RotatedRect r = findRectangles(image, contours);
    cv::Point2f vertices[4];
    r.points(vertices);

    for( int j = 0; j < 4; j++ )
      cv::line(real_image_, vertices[j], vertices[(j+1)%4],
          Color::ColorWhite, 1, 8 );
    for( int j = 0; j < 4; j++ )
      cv::circle(real_image_, vertices[j], 3, Color::ColorBlue, 5);

    embedOrigin(real_image_);

    std::string path(PACKAGE_PATH);
    path.append("/samples/identified.jpg");
    this->saveFile(real_image_,path);

    for (int i = 0; i < 4; ++i) {
      cv::Mat vert_pnt = cv::Mat(cv::Point3d(vertices[i].x, vertices[i].y, 1.0));
      cv::Mat result = rmat_.inv() * camera_matrix_.inv() * scale_ * vert_pnt -
                       rmat_.inv() * tvec_;
      vert_msg.data.push_back(result.at<double>(0,0));
      vert_msg.data.push_back(result.at<double>(0,1));
//      std::cout << "Result " << result << std::endl;
    }
    publisher_.publish(vert_msg);
  }
}

cv::VideoCapture FabricVision::openCamera(int device) {
  video_ = cv::VideoCapture(device);
  if (!video_.isOpened()) ROS_ERROR("Device %d could not be found", device);
  return (video_);
}

void FabricVision::showCamera(cv::VideoCapture &camera,
                              const std::string &window_name) const {
  cv::Mat image;
  cv::namedWindow(window_name.c_str());
  while (true) {
    camera.read(image);
    cv::imshow(window_name.c_str(), image);
    if (cv::waitKey(30) == 1048603)  // Escape key
      break;
  }
}

void FabricVision::getFrame() {
  if (!video_.isOpened()) {
    ROS_ERROR("Camera not initialized");
    return;
  }
  video_.read(image_);
}

void FabricVision::applyFilters() {
  toHSV(image_);
  threshold(image_, filter_);
  morphologicalOpening(image_);
  morphologicalClosing(image_);
  toCanny(image_);
}

void FabricVision::applyFilters(cv::Mat &image) {
  toHSV(image);
  threshold(image, filter_);
  morphologicalOpening(image);
  morphologicalClosing(image);
  toCanny(image);
}

void FabricVision::showFrame(const std::string &window_name) const {
  cv::imshow(window_name, image_);
  cv::waitKey(30);
}

void FabricVision::showFrame(const cv::Mat &image,
                             const std::string &window_name) const {
  cv::imshow(window_name, image);
  cv::waitKey(30);
}

cv::Mat FabricVision::openFile(const std::string &filename) {
  cv::Mat image = cv::imread(filename);
  if (!image.data) ROS_ERROR("No image data.");
  return (image);
}

void FabricVision::showImage(const cv::Mat &image,
                             const std::string &window_name) const {
  cv::namedWindow(window_name.c_str(), cv::WINDOW_AUTOSIZE);
  cv::imshow(window_name.c_str(), image);
  cv::waitKey(0);
}

void FabricVision::saveFile(const cv::Mat &image,
                            const std::string &filename) const {
  cv::imwrite(filename, image);
}

void FabricVision::toGray(cv::Mat &image) const {
  cv::cvtColor(image, image, cv::COLOR_BGR2GRAY);
}

void FabricVision::toHSV(cv::Mat &image) const {
  cv::cvtColor(image, image, cv::COLOR_BGR2HSV);
}

void FabricVision::toBGR(cv::Mat &image) const {
  cv::cvtColor(image, image, cv::COLOR_HSV2BGR);
}

void FabricVision::toCanny(cv::Mat &image) const {
  cv::Canny(image, image, 0, 50, 5);
}

std::vector<std::vector<cv::Point> > FabricVision::findContours(cv::Mat &image)
    const {
  std::vector<std::vector<cv::Point> > contours;
  cv::findContours(image.clone(), contours, cv::RETR_EXTERNAL,
                   cv::CHAIN_APPROX_SIMPLE);
  return (contours);
}

cv::RotatedRect FabricVision::findRectangles(
    const cv::Mat &image,
    const std::vector<std::vector<cv::Point> > &contours) const {
  std::vector<cv::Point> approx;
  for (int i = 0; i < contours.size(); i++) {
    cv::approxPolyDP(cv::Mat(contours[i]), approx,
                     cv::arcLength(cv::Mat(contours[i]), true) * 0.02, true);
    if (std::fabs(cv::contourArea(contours[i])) < 100 ||
        !cv::isContourConvex(approx))
      continue;

    if (approx.size() == 4) {
      std::vector<double> cos;
      for (int j = 2; j < 5; j++)
        cos.push_back(
            angle(approx[j % approx.size()], approx[j - 2], approx[j - 1]));
      std::sort(cos.begin(), cos.end());
      double mincos = cos.front();
      double maxcos = cos.back();
      if (mincos >= -0.1 && maxcos <= 0.3) {
        cv::RotatedRect r = cv::minAreaRect(contours[i]);
        return r;
      }
    }
  }
}

cv::Mat FabricVision::setLabels(
    const cv::Mat &src,
    const std::vector<std::vector<cv::Point> > &contours) const {
  // The array for storing the approximation curve
  std::vector<cv::Point> approx;

  // We'll put the labels in this destination image
  cv::Mat dst = src.clone();

  for (int i = 0; i < contours.size(); i++) {
    // Approximate contour with accuracy proportional
    // to the contour perimeter
    cv::approxPolyDP(cv::Mat(contours[i]), approx,
                     cv::arcLength(cv::Mat(contours[i]), true) * 0.02, true);

    // Skip small or non-convex objects
    if (std::fabs(cv::contourArea(contours[i])) < 100 ||
        !cv::isContourConvex(approx))
      continue;

    if (approx.size() == 3)
      setLabel(dst, "TRI", contours[i]);  // Triangles

    else if (approx.size() >= 4 && approx.size() <= 6) {
      // Number of vertices of polygonal curve
      int vtc = approx.size();

      // Get the degree (in cosines) of all corners
      std::vector<double> cos;
      for (int j = 2; j < vtc + 1; j++)
        cos.push_back(angle(approx[j % vtc], approx[j - 2], approx[j - 1]));

      // Sort ascending the corner degree values
      std::sort(cos.begin(), cos.end());

      // Get the lowest and the highest degree
      double mincos = cos.front();
      double maxcos = cos.back();

      // Use the degrees obtained above and the number of vertices
      // to determine the shape of the contour
      if (vtc == 4 && mincos >= -0.1 && maxcos <= 0.3) {
        // Detect rectangle or square
        cv::Rect r = cv::boundingRect(contours[i]);
        double ratio = std::abs(1 - (double)r.width / r.height);

        // setLabel(dst, ratio <= 0.02 ? "SQU" : "RECT", contours[i]);

        {
          cv::RotatedRect rot = cv::minAreaRect(contours[i]);
          std::stringstream lbl;
          lbl << rot.center << " " << rot.angle;
          setLabel(dst, lbl.str(), contours[i]);
        }
      } else if (vtc == 5 && mincos >= -0.34 && maxcos <= -0.27)
        setLabel(dst, "PENTA", contours[i]);
      else if (vtc == 6 && mincos >= -0.55 && maxcos <= -0.45)
        setLabel(dst, "HEXA", contours[i]);
    } else {
      // Detect and label circles
      double area = cv::contourArea(contours[i]);
      cv::Rect r = cv::boundingRect(contours[i]);
      int radius = r.width / 2;

      if (std::abs(1 - ((double)r.width / r.height)) <= 0.2 &&
          std::abs(1 - (area / (CV_PI * std::pow(radius, 2)))) <= 0.2) {
        setLabel(dst, "CIR", contours[i]);
      }
    }
  }

  return (dst);
}

void FabricVision::threshold(cv::Mat &image, const FilterHSV &filter) const {
  cv::inRange(
      image,
      cv::Scalar(filter.low_hue, filter.low_saturation, filter.low_value),
      cv::Scalar(filter.high_hue, filter.high_saturation, filter.high_value),
      image);
}

void FabricVision::loadCalibration(const std::string &filename) {
  try {
    cv::FileStorage fs(filename, cv::FileStorage::READ);
    fs["camera_matrix"] >> camera_matrix_;
    fs["open"] >> distortion_matrix_;
    fs.release();
  }
  catch (const cv::Exception &e) {
    std::cout << e.what() << std::endl;
  }
}

void FabricVision::thresholdGUI(const std::string &window_name) {
  cv::namedWindow(window_name.c_str(), cv::WINDOW_AUTOSIZE);
  cv::createTrackbar("Low Hue", window_name.c_str(), &filter_.low_hue, 179);
  cv::createTrackbar("High Hue", window_name.c_str(), &filter_.high_hue, 179);
  cv::createTrackbar("Low Saturation", window_name.c_str(),
                     &filter_.low_saturation, 255);
  cv::createTrackbar("High Saturation", window_name.c_str(),
                     &filter_.high_saturation, 255);
  cv::createTrackbar("Low Value", window_name.c_str(), &filter_.low_value, 255);
  cv::createTrackbar("High Value", window_name.c_str(), &filter_.high_value,
                     255);
}

void FabricVision::morphologicalOpening(cv::Mat &image, int radius) {
  cv::erode(image, image, cv::getStructuringElement(cv::MORPH_ELLIPSE,
                                                    cv::Size(radius, radius)));

  cv::dilate(image, image, cv::getStructuringElement(cv::MORPH_ELLIPSE,
                                                     cv::Size(radius, radius)));
}

void FabricVision::morphologicalClosing(cv::Mat &image, int radius) {
  cv::dilate(image, image, cv::getStructuringElement(cv::MORPH_ELLIPSE,
                                                     cv::Size(radius, radius)));

  cv::erode(image, image, cv::getStructuringElement(cv::MORPH_ELLIPSE,
                                                    cv::Size(radius, radius)));
}

void FabricVision::setLabel(cv::Mat &im, const std::string label,
                            const std::vector<cv::Point> &contour) const {
  int fontface = cv::FONT_HERSHEY_SIMPLEX;
  double scale = 0.4;
  int thickness = 1;
  int baseline = 0;

  cv::Size text = cv::getTextSize(label, fontface, scale, thickness, &baseline);
  cv::Rect r = cv::boundingRect(contour);

  cv::Point pt(r.x + ((r.width - text.width) / 2),
               r.y + ((r.height + text.height) / 2));
  cv::rectangle(im, pt + cv::Point(0, baseline),
                pt + cv::Point(text.width, -text.height), CV_RGB(255, 255, 255),
                CV_FILLED);
  cv::putText(im, label, pt, fontface, scale, CV_RGB(0, 0, 0), thickness, 8);
}

void FabricVision::undistort(cv::Mat &image) {
  cv::undistortPoints(image, image, camera_matrix_, distortion_matrix_);
}

double FabricVision::angle(cv::Point pt1, cv::Point pt2, cv::Point pt0) const {
  double dx1 = pt1.x - pt0.x;
  double dy1 = pt1.y - pt0.y;
  double dx2 = pt2.x - pt0.x;
  double dy2 = pt2.y - pt0.y;
  return (dx1 * dx2 + dy1 * dy2) /
         sqrt((dx1 * dx1 + dy1 * dy1) * (dx2 * dx2 + dy2 * dy2) + 1e-10);
}
cv::Mat FabricVision::real_image() const
{
  return real_image_;
}

void FabricVision::setReal_image(const cv::Mat& real_image)
{
  real_image_ = real_image;
}

cv::Mat FabricVision::image() const { return image_; }

void FabricVision::setImage(const cv::Mat &image) { image_ = image; }

FilterHSV FabricVision::filter() const { return filter_; }

void FabricVision::setFilter(const FilterHSV &filter) { filter_ = filter; }
}
