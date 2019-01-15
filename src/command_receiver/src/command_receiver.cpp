/*
 * command_receiver.cpp
 * Copyright (C) 2019 willdle <willdle@willdle-ThinkPad-X1-Carbon>
 *
 * Distributed under terms of the MIT license.
 */

#include <command_receiver/command_receiver.h>

CommandReceiver::CommandReceiver(ros::NodeHandle n, int port, char* main_addr)
{
  //UDP setup
  sock = socket(AF_INET, SOCK_DGAM, IPPROTO_UDP);
  if (sock < 0 )
  {
    ROS_ERROR("Failed to make socket on line: %d in %s", __LINE__, __FILE__);
  }
  addr.sin_family = AF_INET;
  // figure out the port and main computer IP
  addr.sin_port = htons(port);
  addr.sin_addr.s_addr = inet_addr(main_addr);
  //publisher setup
  cmd_pub = n.advertise<robocup_msgs::Command>("command", 30);
}

CommandReceiver::~CommandReceiver()
{
}

void CommandReceiver::run()
{
  //UDP recv
  char* pkt;
  if (recvFrom(socket, pkt, 1024, 0, addr, (socklen_t *) addrLen)
  {
    ROS_ERROR("Receive error on line: %d in %s", __LINE__, __FILE__);
  }
  //pkt fill
  robocup_msgs::Command msg = fillMsg(pkt);
  //publish
  cmd_pub.publish(msg);
}

Command CommandReceiver::fillMsg(char* pkt)
{
  // gather data
  uint16_t x = pkt;
  uint16_t y = pkt+2;
  uint16_t theta = pkt+4;
  uint8_t kick = pkt+6;
  uint8_t dribble = pkt+7;
  
  //fill msg
  robocup_msgs::Command cmd;
  cmd->speed.linear.x = (double) (x & 0x7FFF);
  cmd->speed.linear.y = (double) (y & 0x7FFF);
  cmd->speed.angular.z = (double) (theta & 0x7FFF);
  
  if (x & 0x8000)
  {
    cmd->speed.linear.x *= -1;
  }
  if (y & 0x8000)
  {
    cmd->speed.linear.y *= -1;
  }
  if (theta & 0x8000)
  {
    cmd->speed.angular.z *= -1;
  }
  
  cmd->kick = (kick > 0);
  cmd->dribble = (dribble > 0);

  return cmd;
}


