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

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cerrno>
#include <exception>

#include <ros/ros.h>
#include <hand_publisher/SerialCommunication.h>

namespace raad2015 {

SerialCommunication::SerialCommunication(const int latency)
{
  handle_ = -1;
  latency_ = latency;
}

SerialCommunication::~SerialCommunication()
{
  disconnect();
}

bool SerialCommunication::isConnected()
{
  return (handle_ == 0);
}

void SerialCommunication::connect(const std::string &port)
{
  while (isConnected())
  {
    ROS_ERROR("Device is connected, attempting to disconnect.");
    disconnect();
  }

  ROS_INFO("Opening port: %s ...", port.c_str());
  // O_RDWR     Open for reading and writing.
  // O_NOCTTY   Do not assign controlling terminal.
  // O_NONBLOCK Non-blocking mode.
  handle_ = open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (handle_ == -1)
  {
    ROS_ERROR("Failed getting the handle.");
    throw std::runtime_error("Error openning the port.");
  }

  // F_SETFL    Set file status flag.
  // 0_APPEND   Set append mode.
  // O_NONBLOCK Non-blocking mode.
  ROS_INFO("Succeded connecting to device.");
  fcntl(handle_, F_SETFL, O_APPEND | O_NONBLOCK);

  ROS_INFO("Initializing port settings...");
  initPort();
  ROS_INFO("...initialized");

  ROS_INFO("Sending handshake...");
  try
  {
    send("Initialize");
  }
  catch (std::exception &e)
  {
    ROS_ERROR("Failed to receive responce. %s", e.what());
    disconnect();
    throw std::runtime_error("Error connecting with device.");
  }
  ROS_INFO("Successfully connected to the device.");
}

void SerialCommunication::disconnect()
{
  if (isConnected())
  {
    close(handle_);
    handle_ = -1;
    ROS_INFO("Successfully disconnected from the device.");
  }
}

void SerialCommunication::send(const std::string &msg)
{
  std::string reply;

  try
  {
    writeMessage(msg);
  }
  catch (std::exception &e)
  {
    ROS_ERROR("Failed to send message %s. %s.", msg.c_str(), e.what());
  }

  usleep(latency_);

  try
  {
    reply = readMessage();
  }
  catch (std::exception &e)
  {
    ROS_ERROR("Failed to read message from device. %s", e.what());
  }

  if (reply != "OK")
  {
    ROS_ERROR("Reply from device was %s", reply.c_str());
    throw std::runtime_error("Reply from device was not OK");
  }
}

std::string SerialCommunication::receive()
{
  std::string msg;
  try
  {
    msg = readMessage();
  }
  catch (std::exception &e)
  {
    ROS_ERROR("Failed to read message from device. %s", e.what());
  }

  usleep(latency_);

  try
  {
    writeMessage("OK");
  }
  catch (std::exception &e)
  {
    ROS_ERROR("Failed to send OK. %s.", e.what());
  }

  return msg;
}

void SerialCommunication::checkSerial(const tf::StampedTransform &transform)
{
  std::string msg;
  try
  {
    msg = readMessage();
  }
  catch (std::exception &e)
  {
    ROS_ERROR("Failed to read message from device. %s", e.what());
  }

  usleep(latency_);

  if (msg == "OK")
  {
    try
    {
      sendTransform(transform);
    }
    catch (std::exception &e)
    {
      ROS_ERROR("Failed to send transform. %s.", e.what());
    }
  }
}

void SerialCommunication::sendTransform(const tf::StampedTransform &transform)
{
  double x = transform.getOrigin().getX();
  double y = transform.getOrigin().getY();
  double z = transform.getOrigin().getZ();

  std::stringstream stream;
  stream << std::setprecision(5) << x << " - " << y << " - " << z;

  writeMessage(stream.str());
}

void SerialCommunication::writeMessage(const std::string &msg)
{
  int countSent = write(handle_, msg.c_str(), msg.length());
  if (countSent < 0)
    throw std::runtime_error("Error sending data to serial.");
}

std::string SerialCommunication::readMessage()
{
  std::string msg;
  int countReceived;
  char buf[1025] = "";

  while ( ( countReceived = read(handle_, buf, 1024) ) > 0 )
  {
    msg.append(buf, countReceived);
    if (countReceived < 1024)
      break;
  }

  if (countReceived < 0)
  {
    if (errno == EAGAIN)
      throw std::runtime_error("Serial IO error.");
    else
      throw std::runtime_error("Error recieving data from serial.");
  }

  return msg;
}

void SerialCommunication::initPort()
{
  int BAUDRATE = B115200;
  struct termios newtio;
  tcgetattr(handle_, &newtio);
  cfsetospeed(&newtio, (speed_t)BAUDRATE);
  cfsetispeed(&newtio, (speed_t)BAUDRATE);

  //Enable the Receiver and  Set local Mode
  // Ignore Break Condition & no processing under input options
  newtio.c_iflag = IGNBRK;
  // Select the RAW Input Mode through Local options
  newtio.c_lflag = 0;
  // Select the RAW Output Mode through Local options
  newtio.c_oflag = 0;
  // Select the Local Mode & Enable Receiver through Control options
  newtio.c_cflag |= (CLOCAL | CREAD);

  //Set Data format to 7E1
  // Mask the Character Size Bits through Control options
  newtio.c_cflag &= ~CSIZE;
  // Select Character Size to 7-Bits through Control options
  newtio.c_cflag |= CS7;
  // Select the Parity Enable through Control options
  newtio.c_cflag |= PARENB;
  // Select the Even Parity through Control options*/
  newtio.c_cflag &= ~PARODD;

  /**********************
   * Different Settings *
   **********************
  // Local Line (do not change the owner of the port)
  newtio.c_cflag |= CLOCAL;
  // Enable receiver
  newtio.c_cflag |= CREAD;
  // Disable echoing of input characters
  newtio.c_cflag &= ~ECHO;
  newtio.c_cflag &= ~ECHOE;

  // Set Data formatl to 8n1
  // No parity byte
  newtio.c_cflag &= ~PARENB;
  // 1 stop bit
  newtio.c_cflag &= ~CSTOPB;
  // Mask the character size bits
  newtio.c_cflag &= ~CSIZE;
  // 8 data bits
  newtio.c_cflag |= CS8;
  */

  // Set the attribute NOW without waiting for Data to Complete
  tcsetattr (handle_, TCSANOW, &newtio);
}

}
