/*********************************************************************
* test_vision.cpp
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

#include <gtest/gtest.h>
#include <boost/lexical_cast.hpp>
#include <hand_publisher/FabricVision.h>
#include <hand_publisher/config.h>

using namespace raad2015;
using namespace std;

TEST(Hand_Publisher, Fabric_Calibration) {
  FabricVision vision;
  std::string package_path(PACKAGE_PATH);
  std::string calibration_path(package_path);
  calibration_path.append("/config/camera_calibration.yaml");
  std::string open_path(package_path);
  open_path.append("/samples/testing.jpg");
  vision.loadCalibration(calibration_path);
  cv::Mat img = vision.openFile(open_path);
  cv::Mat checkerboard = vision.calibrateExtrinsic(img);
  cv::Mat origin = vision.undistort(img);
  origin = vision.embedOrigin(origin);

  std::string save_path(package_path);
  save_path.append("/samples/");
  std::string save_checker(save_path);
  save_checker.append("checker_fabric.jpg");
  std::string save_origin(save_path);
  save_origin.append("origin_fabric.jpg");
  vision.saveFile(checkerboard, save_checker);
  vision.saveFile(origin, save_origin);
}

TEST(Hand_Publisher, Fabric_Vision) {
  FabricVision vision;
  std::string package_path(PACKAGE_PATH);
  std::string calibration_path(package_path);
  calibration_path.append("/config/camera_calibration.yaml");
  vision.loadCalibration(calibration_path);

  std::vector<std::vector<cv::Point> > contours;
  for (int i = 1; i < 4; ++i) {
    std::string open_path(package_path);
    open_path.append("/samples/img");
    open_path.append(boost::lexical_cast<std::string>(i));
    open_path.append(".jpg");
    cv::Mat image = vision.openFile(open_path);
    image = vision.undistort(image);
    image = vision.applyFilters(image);
    contours = vision.findContours(image);
    image = vision.setLabels(image, contours);
    std::string save_path(package_path);
    save_path.append("/samples/edited");
    save_path.append(boost::lexical_cast<std::string>(i));
    save_path.append(".jpg");
    vision.saveFile(image, save_path);
  }
}

TEST(Hand_Publisher, Localization) {
  FabricVision vision;
  std::string package_path(PACKAGE_PATH);
  std::string calibration_path(package_path);
  calibration_path.append("/config/camera_calibration.yaml");
  std::string open_path(package_path);
  open_path.append("/samples/received_image.jpg");
  vision.loadCalibration(calibration_path);
  cv::Mat img = vision.openFile(open_path);
  img = vision.calibrateExtrinsic(img);
  cv::Mat und = vision.undistort(img);
  img = vision.embedOrigin(img);
  cv::Mat filtered = vision.applyFilters(img);
  vision.calculateVertices(filtered, img);
  std::string save_path(package_path);
  save_path.append("/samples/identified.jpg");

  std::vector<cv::Point2f> vert = vision.calculateVertices(filtered);
  std::cout << "Vertices on the image are:" << vert << std::endl;
  std::vector<cv::Point3f> wrld = vision.projectTo3D(vert);
  std::cout << "Vertices on the world are:" << wrld << std::endl;
  img = vision.embedPoints(img, vert, wrld);
  vision.saveFile(img, save_path);
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "google_test_vision");
  return RUN_ALL_TESTS();
}
