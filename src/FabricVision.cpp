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
  this->filter_.low_hue = 18;
  this->filter_.low_saturation = 45;
  this->filter_.low_value = 45;
  this->filter_.high_hue = 38;
  this->filter_.high_saturation = 255;
  this->filter_.high_value = 255;
  this->intrinsic_calibrated_ = false;
  this->extrinsic_calibrated_ = false;
}

void FabricVision::subscribeTopic(const std::string &topic_name) {
  subscriber_ =
      node_.subscribe(topic_name, 10, &FabricVision::fabricRequest, this);
}

void FabricVision::publishTopic(const std::string &topic_name) {
  publisher_ = node_.advertise<std_msgs::Float64MultiArray>(topic_name, 10);
}

cv::Mat FabricVision::calibrateExtrinsic(const cv::Mat &image)
{
  if (!this->intrinsic_calibrated_) {
    ROS_ERROR("Camera not calibrated. Have you loaded the calibration file?");
    exit(-1);
  }
  cv::Mat img = image.clone();
  cv::Size checkerboard = cv::Size(7, 10); // A3 Checkerboard
  // Create actual board points
  std::vector<cv::Point3d> board_points;
  for (size_t i = 0; i < 10; ++i)
    for (size_t j = 0; j < 7; ++j)
      board_points.push_back(cv::Point3d(3.4 * i, 3.4 * j, 0.0));

  // Try to find checkerboard
  bool found;
  std::vector<cv::Point2f> found_points;
  found = cv::findChessboardCorners(img,
                                    checkerboard,
                                    found_points,
                                    cv::CALIB_CB_ADAPTIVE_THRESH+
                                    cv::CALIB_CB_NORMALIZE_IMAGE+
                                    cv::CALIB_CB_FILTER_QUADS);
  if (found) {
    ROS_INFO("Found Checkerboard");
    cv::drawChessboardCorners(img,
                              checkerboard,
                              cv::Mat(found_points),
                              found);

    cv::solvePnP(cv::Mat(board_points),
                 cv::Mat(found_points),
                 this->camera_matrix_,
                 this->distortion_matrix_,
                 this->rvec_,
                 this->tvec_,
                 false,
                 CV_ITERATIVE);
    // Only use four points to get Perspective Transform
    std::vector<cv::Point2f> img_pts, wrl_pts;
    {
      wrl_pts.push_back(cv::Point2f(board_points[0].x, board_points[0].y));
      img_pts.push_back(found_points[0]);
      wrl_pts.push_back(cv::Point2f(board_points[6].x, board_points[6].y));
      img_pts.push_back(found_points[6]);
      wrl_pts.push_back(cv::Point2f(board_points[63].x, board_points[63].y));
      img_pts.push_back(found_points[63]);
      wrl_pts.push_back(cv::Point2f(board_points[69].x, board_points[69].y));
      img_pts.push_back(found_points[69]);
    }

    this->image_to_checkerboard_ = cv::getPerspectiveTransform(img_pts, wrl_pts);

    cv::Rodrigues(this->rvec_, this->rmat_);
    cv::Mat scale_mat = camera_matrix_ * ( rmat_ * cv::Mat(board_points[0]) + tvec_ );
    scale_ = scale_mat.at<double>(0,2);
    this->extrinsic_calibrated_ = true;
    return img;
  }
  else {
    ROS_ERROR("Checkerboard not found!");
    exit(-2);
  }
}

cv::Mat FabricVision::embedOrigin(const cv::Mat &image)
{
  if (!this->extrinsic_calibrated_) {
    ROS_ERROR("Camera has not been extrinsically calibrated.");
    exit(-3);
  }
  cv::Mat img = image.clone();

  std::vector<cv::Point3d> points;
  points.push_back(cv::Point3d(0.0, 0.0, 0.0));
  points.push_back(cv::Point3d(20.0, 0.0, 0.0));
  points.push_back(cv::Point3d(0.0, 20.0, 0.0));
  points.push_back(cv::Point3d(0.0, 0.0, 20.0));

  cv::Mat projected_points;
  cv::projectPoints(points,
                    this->rvec_,
                    this->tvec_,
                    this->camera_matrix_,
                    this->distortion_matrix_,
                    projected_points);
  std::vector<cv::Point2d> projected_vector;
  projected_points.copyTo(projected_vector);

  cv::circle(img, projected_vector[0],
             3, Color::ColorRed, 5);
  cv::line(img,
           projected_vector[0],
           projected_vector[1],
           Color::ColorRed, 3, 8);
  this->setLabel(img, "x", right, projected_vector[1]);
  cv::line(img,
           projected_vector[0],
           projected_vector[2],
           Color::ColorRed, 3, 8);
  this->setLabel(img, "y", down, projected_vector[2]);
  cv::line(img,
           projected_vector[0],
           projected_vector[3],
           Color::ColorRed, 3, 8);
  this->setLabel(img, "z", right, projected_vector[3]);
  return img;
}

void FabricVision::calculateVertices(const cv::Mat &image, cv::Mat &draw)
{
  cv::Mat img = image.clone();
  std::vector<std::vector<cv::Point> > contours;
  contours = this->findContours(img);
  cv::RotatedRect r = findRectangles(img, contours);
  cv::Point2f vertices[4];
  r.points(vertices);

  for (int i = 0; i < 4; i++)
    cv::line(draw, vertices[i], vertices[(i+1)%4], Color::ColorWhite, 1, 8);
  for (int i = 0; i < 4; i++ )
    cv::circle(draw, vertices[i], 3, Color::ColorBlue, 5);
}

std::vector<cv::Point2f> FabricVision::calculateVertices(const cv::Mat &image)
{
  cv::Mat img = image.clone();
  std::vector<std::vector<cv::Point> > contours;
  contours = this->findContours(img);
  cv::RotatedRect r = findRectangles(img, contours);
  cv::Point2f vertices[4];
  r.points(vertices);
  std::vector<cv::Point2f> vec_ver;
  for (int i = 0; i < 4; i++) {
    vec_ver.push_back(vertices[i]);
  }
  return vec_ver;
}

cv::VideoCapture FabricVision::openCamera(int device)
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
  while (true) {
    camera.read(image);
    cv::imshow(window_name.c_str(), image);
    if (cv::waitKey(30) == 1048603)  // Escape key
      break;
  }
}

cv::Mat FabricVision::getFrame(cv::VideoCapture &camera)
{
  if (!camera.isOpened()) {
    ROS_ERROR("Camera not initialized");
    exit(-4);
  }
  cv::Mat img;
  camera.read(img);
  return img;
}

cv::Mat FabricVision::applyFilters(const cv::Mat &image) {
  cv::Mat img = image.clone();
  toHSV(img);
  threshold(img, filter_);
  morphologicalOpening(img);
  morphologicalClosing(img);
  toCanny(img);
  return img;
}

void FabricVision::showFrame(const cv::Mat &image,
                             const std::string &window_name) const
{
  cv::imshow(window_name, image);
  cv::waitKey(30);
}

cv::Mat FabricVision::openFile(const std::string &filename) {
  cv::Mat image = cv::imread(filename);
  if (!image.data)
    ROS_ERROR("No image data.");
  return (image);
}

void FabricVision::saveFile(const cv::Mat &image,
                            const std::string &filename) const
{
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
    const
{
  std::vector<std::vector<cv::Point> > contours;
  cv::findContours(image.clone(), contours, cv::RETR_EXTERNAL,
                   cv::CHAIN_APPROX_SIMPLE);
  return (contours);
}

cv::RotatedRect FabricVision::findRectangles(
    const cv::Mat &image,
    const std::vector<std::vector<cv::Point> > &contours) const
{
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

cv::Mat FabricVision::setLabels(const cv::Mat &src,
                                const std::vector<std::vector<cv::Point> > &contours) const
{
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

void FabricVision::threshold(cv::Mat &image, const FilterHSV &filter) const
{
  cv::inRange(image,
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
    this->intrinsic_calibrated_ = true;
  }
  catch (const cv::Exception &e) {
    ROS_ERROR("%s", e.what());
    this->intrinsic_calibrated_ = false;
  }
}

void FabricVision::thresholdGUI(const std::string &window_name)
{
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

void FabricVision::morphologicalOpening(cv::Mat &image, int radius)
{
  cv::Mat el = cv::getStructuringElement(cv::MORPH_ELLIPSE,cv::Size(radius, radius));
  cv::erode(image, image, el);
  cv::dilate(image, image, el);
}

void FabricVision::morphologicalClosing(cv::Mat &image, int radius)
{
  cv::Mat el = cv::getStructuringElement(cv::MORPH_ELLIPSE,cv::Size(radius, radius));
  cv::dilate(image, image, el);
  cv::erode(image, image, el);
}

void FabricVision::setLabel(cv::Mat &image, const std::string label,
                            const Location &loc,
                            const cv::Point &point) const {
  int fontface = cv::FONT_HERSHEY_TRIPLEX;
  double scale = 1;
  int thickness = 2;

  double x,y;

  switch (loc) {
    case up:
      x = 0.0;
      y = -20.0;
      break;
    case down:
      x = 0.0;
      y = 20.0;
      break;
    case right:
      x = 10.0;
      y = 0.0;
      break;
    case left:
      x = -10.0;
      y = 0.0;
      break;
  }

  cv::Point pt = cv::Point(point.x + x, point.y + y);
  cv::putText(image, label, pt, fontface, scale, Color::ColorRed, thickness, 8);
}

void FabricVision::setLabel(cv::Mat &im, const std::string label,
                            const std::vector<cv::Point> &contour) const {
  int fontface = cv::FONT_HERSHEY_TRIPLEX;
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

cv::Mat FabricVision::undistort(const cv::Mat &image)
{
  cv::Mat img = image.clone();
  cv::undistort(image, img, this->camera_matrix_, this->distortion_matrix_);
  return img;
}

double FabricVision::angle(cv::Point pt1, cv::Point pt2, cv::Point pt0) const
{
  double dx1 = pt1.x - pt0.x;
  double dy1 = pt1.y - pt0.y;
  double dx2 = pt2.x - pt0.x;
  double dy2 = pt2.y - pt0.y;
  return (dx1 * dx2 + dy1 * dy2) /
         sqrt((dx1 * dx1 + dy1 * dy1) * (dx2 * dx2 + dy2 * dy2) + 1e-10);
}

void FabricVision::fabricRequest(const std_msgs::Bool &msg)
{
  // Requested localization
  cv::VideoCapture camera = this->openCamera(0);
  cv::Mat img = this->getFrame(camera);
  img = this->calibrateExtrinsic(img);
  img = this->undistort(img);
  img = this->applyFilters(img);
  std::vector<cv::Point2f> vertices = this->calculateVertices(img);
  std::vector<cv::Point3f> world = this->projectTo3D(vertices);
  this->publishResults(world);
}

std::vector<cv::Point3f> FabricVision::projectTo3D(const std::vector<cv::Point2f> &image_points)
{

  std::vector<cv::Point3f> projected_points;
  for (size_t i = 0; i < image_points.size(); i++) {
    cv::Point2f world_point = this->transformPoint(image_points[i], image_to_checkerboard_);
    cv::Point3f proj_pnt = cv::Point3f(world_point.x,
                                       world_point.y,
                                       0.0);
    projected_points.push_back(proj_pnt);
  }
  return projected_points;
}

void FabricVision::publishResults(const std::vector<cv::Point3f> &vertices)
{
  std_msgs::Float64MultiArray msg;
  for (size_t i = 0; i < vertices.size(); i++) {
    msg.data.push_back(vertices.at(i).x);
    msg.data.push_back(vertices.at(i).y);
  }
  publisher_.publish(msg);
}

cv::Point2f FabricVision::transformPoint(cv::Point2f current, cv::Mat transformation)
{
    cv::Point2f transformedPoint;
    transformedPoint.x = current.x * transformation.at<double>(0,0) + current.y * transformation.at<double>(0,1) + transformation.at<double>(0,2);
    transformedPoint.y = current.x * transformation.at<double>(1,0) + current.y * transformation.at<double>(1,1) + transformation.at<double>(1,2);
    float z = current.x * transformation.at<double>(2,0) + current.y * transformation.at<double>(2,1) + transformation.at<double>(2,2);
    transformedPoint.x /= z;
    transformedPoint.y /= z;

    return transformedPoint;
}

cv::Mat FabricVision::embedPoints(const cv::Mat &image,
                                  const std::vector<cv::Point2f> &pixels,
                                  const std::vector<cv::Point3f> &points) const
{
  cv::Mat img = image.clone();
  {
    std::stringstream label;
    label << std::setprecision(2);
    label << "(" << points[0].x << "," << points[0].y << ")";
    this->setLabel(img, label.str(), down, pixels[0]);
  }
  {
    std::stringstream label;
    label << std::setprecision(2);
    label << "(" << points[1].x << "," << points[1].y << ")";
    this->setLabel(img, label.str(), down, pixels[1]);
  }
  {
    std::stringstream label;
    label << std::setprecision(2);
    label << "(" << points[2].x << "," << points[2].y << ")";
    this->setLabel(img, label.str(), up, pixels[2]);
  }
  {
    std::stringstream label;
    label << std::setprecision(2);
    label << "(" << points[3].x << "," << points[3].y << ")";
    this->setLabel(img, label.str(), up, pixels[3]);
  }

  return img;
}

}
