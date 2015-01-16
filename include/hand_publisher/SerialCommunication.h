/*********************************************************************
* SerialCommunication.h
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

#ifndef SERIAL_COMMUNICATION_H
#define SERIAL_COMMUNICATION_H

#include <iostream>
#include <string>
#include <tf/transform_datatypes.h>

namespace raad2015 {
/**
 * @brief
 */

class SerialCommunication
{
public:
  SerialCommunication(const int latency = 10);
  ~SerialCommunication();
  bool isConnected();
  void connect(const std::string &port);
  void disconnect();
  void send(const std::string &msg);
  std::string receive();
  void checkSerial(const tf::StampedTransform &transform);
private:
  void initPort();
  void writeMessage(const std::string &msg);
  void sendTransform(const tf::StampedTransform &transform);
  std::string readMessage();
  int handle_;
  int latency_;
};

}

#endif
