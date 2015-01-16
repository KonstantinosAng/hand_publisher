/*********************************************************************
* SerialCommunication.cpp
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
#include <string>
#include <hand_publisher/SerialCommunication.h>

using namespace LibSerial;

namespace raad2015 {

SerialCommunication::SerialCommunication(const std::string &port)
{
  serial_.Open(port);
  serial_.SetBaudRate(SerialStreamBuf::BAUD_38400);
  serial_.SetCharSize(SerialStreamBuf::CHAR_SIZE_8);
  serial_.SetNumOfStopBits(1);
  serial_.SetParity(SerialStreamBuf::PARITY_NONE);
  serial_.SetFlowControl(SerialStreamBuf::FLOW_CONTROL_NONE);
}

void SerialCommunication::send(const std::string &msg)
{
  char output_buffer[BUFFER_SIZE];
  strncpy(output_buffer, msg.c_str(), BUFFER_SIZE);
  output_buffer[BUFFER_SIZE - 1] = 0;
  serial_.write(output_buffer,BUFFER_SIZE);
}

std::string SerialCommunication::receive()
{
  char input_buffer[BUFFER_SIZE];
  serial_.read(input_buffer,BUFFER_SIZE);
  std::string msg(input_buffer);
  return msg;
}

void SerialCommunication::checkSerial(const tf::StampedTransform &transform)
{
  std::string msg = receive();

  if (msg == "OK")
    sendTransform(transform);
  else
    ROS_ERROR("Received %s", msg.c_str());
}

void SerialCommunication::sendTransform(const tf::StampedTransform &transform)
{
  double x = transform.getOrigin().getX();
  double y = transform.getOrigin().getY();
  double z = transform.getOrigin().getZ();

  std::stringstream stream;
  stream << std::setprecision(5) << x << " - " << y << " - " << z;

  send(stream.str());
}

}
