/*
 * command_node.cpp
 * Copyright (C) 2019 willdle <willdle@willdle-ThinkPad-X1-Carbon>
 *
 * Distributed under terms of the MIT license.
 */

#include <ros/ros.h>
#include <command_receiver/command_receiver.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "command_receiver");
  ros::NodeHandle h;
  char* addr = "192.0.0.1";
  int port = 30;
  command_receiver::CommandReceiver cmd_recv(h, port, addr);
  ros::Rate loop_rate(60);
  while(ros::ok())
  {
    cmd_recv.run();
    ros::spinOnce();
    loop_rate.sleep();
  }
}
