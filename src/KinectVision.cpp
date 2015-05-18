/*********************************************************************
* KinectVision.cpp
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

#include <hand_publisher/KinectVision.h>

namespace raad2015 {

cv::Scalar Color::ColorBlue(255, 0, 0);
cv::Scalar Color::ColorRed(0, 0, 255);
cv::Scalar Color::ColorGreen(0, 255, 0);
cv::Scalar Color::ColorWhite(255, 255, 255);
cv::Scalar Color::ColorBlack(0, 0, 0);

KinectVision::KinectVision()
{
}

void KinectVision::subscribeTopic(const std::string &topic_name)
{
  node_.subscribe(topic_name, 10, &KinectVision::imageCallBack, this);
}

void KinectVision::imageCallBack(const sensor_msgs::ImageConstPtr& msg)
{
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGRA8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  calibrateExtrinsic(cv_ptr->image);
}

void KinectVision::calibrateExtrinsic(const cv::Mat &image)
{
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
  cv::cvtColor(img,img, cv::COLOR_BGRA2BGR);
  found = cv::findChessboardCorners(img, checkerboard, found_points,
                                    cv::CALIB_CB_ADAPTIVE_THRESH+
                                    cv::CALIB_CB_NORMALIZE_IMAGE+
                                    cv::CALIB_CB_FILTER_QUADS);
  cv::Mat draw_img = img.clone();
  cv::drawChessboardCorners(draw_img, checkerboard, cv::Mat(found_points), found);
  cv::imwrite("/home/aris/source/indigo/raad2015_ws/src/hand_publisher/samples/kinect2_chess.jpg",
              draw_img);
  cv::Mat undist_img = img.clone();
  cv::undistort(img, undist_img, camera_matrix_, distortion_matrix_);
  cv::imwrite("/home/aris/source/indigo/raad2015_ws/src/hand_publisher/samples/kinect2_undistorted.jpg",
              undist_img);

  if (found) {
    ROS_INFO("Found Checkerboard");
    cv::solvePnP(cv::Mat(board_points), cv::Mat(found_points), camera_matrix_,
                 distortion_matrix_, rvec_, tvec_, false, CV_ITERATIVE);
    cv::Rodrigues(rvec_, rmat_);
    cv::Mat scale_mat = camera_matrix_ * ( rmat_ * cv::Mat(board_points[0]) + tvec_ );
    scale_ = scale_mat.at<double>(0,2);
    this->calculateTransformation();
  }
  else {
    ROS_ERROR("Checkerboard not found!");
  }
}

cv::Mat KinectVision::openFile(const std::string &filename)
{
  cv::Mat image = cv::imread(filename);
  if (!image.data) ROS_ERROR("No image data.");
  return (image);
}

void KinectVision::showImage(const cv::Mat &image, const std::string &window_name) const
{
  cv::imshow(window_name, image);
  cv::waitKey(30);
}

void KinectVision::loadIntrinsic(const std::string &filename)
{
  try {
    cv::FileStorage fs(filename, cv::FileStorage::READ);
    fs["camera_matrix"] >> camera_matrix_;
    fs["open"] >> distortion_matrix_;
    fs.release();
  }
  catch (const cv::Exception &e) {
    ROS_ERROR("%s", e.what());
  }
}

void KinectVision::embedOrigin(cv::Mat &image)
{
  std::vector<cv::Point3d> points;
  points.push_back(cv::Point3d(0.0, 0.0, 0.0));
  points.push_back(cv::Point3d(20.0, 0.0, 0.0));
  points.push_back(cv::Point3d(0.0, 20.0, 0.0));
  points.push_back(cv::Point3d(0.0, 0.0, 20.0));
  cv::Mat projected_points;
  cv::projectPoints(points, rvec_, tvec_, camera_matrix_, distortion_matrix_,
                    projected_points);
  std::vector<cv::Point2d> projected_vector;
  projected_points.copyTo(projected_vector);

  cv::circle(image, projected_vector[0],
             3, Color::ColorRed, 5);
  cv::line(image,
           projected_vector[0],
           projected_vector[1],
           Color::ColorRed, 3, 8);
  cv::line(image,
           projected_vector[0],
           projected_vector[2],
           Color::ColorRed, 3, 8);
  cv::line(image,
           projected_vector[0],
           projected_vector[3],
           Color::ColorRed, 3, 8);
}

void KinectVision::saveFile(const cv::Mat &image,
                            const std::string &filename) const {
  cv::imwrite(filename, image);
}

void KinectVision::publishTransform()
{
  try {
    tf::StampedTransform stmp_trn(tf_, ros::Time::now(), "/camera_checkerboard", "/tracker_rgb_frame");
    br_.sendTransform(stmp_trn);
  }
  catch (...) {
    ROS_ERROR("Exception thrown while publishing transformation");
  }
}

void KinectVision::calculateTransformation()
{
  tf::Matrix3x3 rot;
  rot[0][0] = rmat_.at<double>(0,0);
  rot[0][1] = rmat_.at<double>(0,1);
  rot[0][2] = rmat_.at<double>(0,2);
  rot[1][0] = rmat_.at<double>(1,0);
  rot[1][1] = rmat_.at<double>(1,1);
  rot[1][2] = rmat_.at<double>(1,2);
  rot[2][0] = rmat_.at<double>(2,0);
  rot[2][1] = rmat_.at<double>(2,1);
  rot[2][2] = rmat_.at<double>(2,2);
  tf::Vector3 tran(tvec_.at<double>(0,0)/100.0,
                   tvec_.at<double>(1,0)/100.0,
                   tvec_.at<double>(2,0)/100.0);
  tf::Transform trans = tf::Transform(rot, tran);
  tf_ = trans.inverse();
}

}
