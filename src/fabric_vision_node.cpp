/*********************************************************************
* fabric_vision_node.cpp
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

#include <ros/ros.h>
#include <hand_publisher/config.h>
#include <hand_publisher/FabricVision.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "fabric_vision_node");
  ros::NodeHandle n;
  std::string package_path(PACKAGE_PATH);
  std::string calibration_path(package_path);
  calibration_path.append("/config/camera_calibration.yml");

  raad2015::FabricVision vision;
  cv::Point3f trans(0.0, 0.0, 1.0);
  vision.setCamera_translation(trans);
  vision.publishTopic("fabric_vertices");
  vision.subscribeTopic("fabric_localization_request");
  vision.loadCalibration(calibration_path);
  vision.openCamera(1);
  vision.calibrate();
//  vision.thresholdGUI();
  ros::Rate rate(30);

  while(ros::ok())
  {
    vision.getFrame();
    vision.applyFilters();
    vision.showFrame();
    ros::spinOnce();
    rate.sleep();
  }
  return 0;
}
