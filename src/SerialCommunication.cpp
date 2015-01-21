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

#include <hand_publisher/SerialCommunication.h>

namespace raad2015 {

SerialCommunication::SerialCommunication(const std::string &port,
                                         int baudrate)
{
  int fd = -1;
  struct termios newtio;
//  FILE *fp_ptr = &fp_serial_;

  fd = open(port.c_str(), O_RDWR | O_NOCTTY | O_NDELAY );
  if (fd < 0)
    ROS_ERROR("Could not open serial device %s", port.c_str());

  memset(&newtio, 0, sizeof(newtio));

  newtio.c_cflag =  CS8 | CLOCAL | CREAD; //no parity, 1 stop bit
  newtio.c_iflag = IGNCR;                 //ignore CR, other options off
  newtio.c_iflag |= IGNBRK;               //ignore break condition
  newtio.c_oflag = 0;                     //all options off
  newtio.c_lflag = ICANON;                //process input as lines

  // Activate the settings
  tcflush(fd, TCIFLUSH);

  if (cfsetispeed(&newtio, baudrate) < 0 || cfsetospeed(&newtio, baudrate) < 0)
  {
    ROS_ERROR("Failed to set serial baud rate: %d", baudrate);
    close(fd);
  }

  // Set the baud rate
  tcsetattr(fd, TCSANOW, &newtio);
  tcflush(fd, TCIOFLUSH);

  //Open file as a standard I/O stream
  fp_serial_ = fdopen(fd, "r+");
  if (!fp_serial_)
    ROS_ERROR("Failed to open serial stream %s", port.c_str());
//  fp_serial_ = *fp_ptr;
}

SerialCommunication::~SerialCommunication()
{
  rcv_.join();
  fclose(fp_serial_);
}

void SerialCommunication::subscribeTopic(const std::string &topic_name)
{
  subscriber_ = node_.subscribe(topic_name,
                                10,
                                &SerialCommunication::sendMessage,
                                this);
}

void SerialCommunication::publishTopic(const std::string &topic_name)
{
  publisher_ = node_.advertise<std_msgs::String>(topic_name,
                                                 10);
}

void SerialCommunication::sendMessage(const std_msgs::String &msg)
{
  ROS_INFO("%s", msg.data.c_str());
  fprintf(fp_serial_, "%s\r\n", msg.data.c_str());
}

void SerialCommunication::receiveThread()
{
  char buffer[BUFFER_SIZE] = "";
  char *buffer_ptr;
  std_msgs::String msg;

  ROS_INFO("Receive thread started");
  while (ros::ok())
  {
    buffer_ptr = fgets(buffer, BUFFER_SIZE, fp_serial_);
    if (buffer_ptr != 0)
    {
      ROS_INFO("Received %s", buffer);
      msg.data = buffer;
      publisher_.publish(msg);
    }
  }
}

void SerialCommunication::runLoop()
{
  // Create receive thread
  rcv_ = boost::thread(boost::bind(&SerialCommunication::receiveThread, this));
  ros::spin();
}

}
