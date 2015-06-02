# Hand Publisher [![Build Status](https://img.shields.io/travis/roboticsgroup/hand_publisher.svg)](https://travis-ci.org/roboticsgroup/hand_publisher)
[![Open Issues](https://img.shields.io/github/issues/roboticsgroup/hand_publisher.svg)](https://github.com/roboticsgroup/hand_publisher/issues)
[![Licence](https://img.shields.io/github/license/roboticsgroup/hand_publisher.svg)](https://github.com/roboticsgroup/hand_publisher/blob/master/LICENCE.md)

This package contains the source code for the skeleton tracking and communication for fabric folding using the Adept Scara robot, the Kinect openni2_tracker ROS package and a vision sensor.

## How to setup ##

This package requires ROS Hydro or Indigo, Ubuntu 12.04 or 14.04, the openni2_tracker package with an appropriate sensor (Asus Xtion PRO Live). Also for the serial communication the LibSerial library is required (which can be found in the Debian/Ubuntu Repositories).

To install all the required modules:

- Install ROS.
- Create a catkin workspace
```
#!bash
mkdir -p ~/raad2015_ws/src
cd ~/raad2015_ws/src
source /opt/ros/hydro/setup.bash
catkin_init_workspace
```
- Clone the required repositories
```
#!bash
git clone git@github.com:roboticsgroup/openni2_tracker.git
git clone git@bitbucket.org:roboticsgroup/hand_publisher.git
cd .. && catkin_make
```
- Configure the transformations and launch the required launch files