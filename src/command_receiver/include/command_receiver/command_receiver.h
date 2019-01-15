/*
 * command_receiver.h
 * Copyright (C) 2019 willdle <willdle@willdle-ThinkPad-X1-Carbon>
 *
 * Distributed under terms of the MIT license.
 */

#ifndef COMMAND_RECEIVER_H
#define COMMAND_RECEIVER_H
#include <ros/ros.h>
#include <robocup_msgs/Command.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>

class CommandReceiver
{
  public:
    CommandReceiver();
    ~CommandReceiver();
    void run();
  private:
    ros::NodeHandle n;
    ros::Publisher cmd_pub;
    int socket;
    struct sockaddr_in addr;
    robocup_msgs::Command fillMsg(char* pkt);
}


#endif /* !COMMAND_RECEIVER_H */
