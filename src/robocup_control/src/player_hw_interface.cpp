/*
 * player_hw_interface.cpp
 * Copyright (C) 2018 willdle <willdle@willdle-ThinkPad-X1-Carbon>
 *
 * Distributed under terms of the MIT license.
 */

#include <robocup_control/player_hw_interface.h>
#include <wiringPi.h>

namespace robocup_control
{
PlayerHWInterface::PlayerHWInterface(ros::NodeHandle nh)
{
  ROS_INFO("Initializng Player Hardware Interface");
  nh.getParam("/robocup_control/drive_controller/joints", joint_names);
  ROS_INFO("Done getting joint parameters");

  num_joints = joint_names.size();

  cmd_client = nh.serviceClient<robocup_control::Insn>("player_insns");
  data_client = nh.serviceClient<robocup_control::Data>("player_data");
}

bool PlayerHWInterface::init()
{
  vel_cmd.resize(num_joints);
  pos.resize(num_joints);
  vel.resize(num_joints);
  eff.resize(num_joints);

  ROS_INFO("Resized joints");
  ROS_INFO("Number of joints: %zd", num_joints);

  for (int i = 0; i < num_joints; i++)
  {
    ROS_INFO("Registering joint to joint state interface: %s", joint_names[i].c_str());
    // connect and register the joint state interface
    hardware_interface::JointStateHandle jnt_st_handle(joint_names[i], &pos[i], &vel[i], &eff[i]);
    jnt_st_interface.registerHandle(jnt_st_handle);

    ROS_INFO("Registering joint to joint velocity interface: %s", joint_names[i].c_str());
    // connect and register the joint velocity interface
    hardware_interface::JointHandle jnt_vel_handle(jnt_st_interface.getHandle(joint_names[i]), &vel_cmd[i]);
    jnt_vel_interface.registerHandle(jnt_vel_handle);
  }

  ROS_INFO("Registering interfaces");
  registerInterface(&jnt_st_interface);
  registerInterface(&jnt_vel_interface);

  ROS_INFO("Getting motor handles");
  motor_1 = jnt_vel_interface.getHandle(joint_names[0]);
  motor_2 = jnt_vel_interface.getHandle(joint_names[1]);
  motor_3 = jnt_vel_interface.getHandle(joint_names[2]);
  motor_4 = jnt_vel_interface.getHandle(joint_names[3]);

  if(wiringPiSPISetup(0, 100000) < 0 )
  {
    ROS_ERROR();
    //die
  }
  return true;
}

void PlayerHWInterface::read()
{
  char req[12];
  if ( wiringPiSPIDataRW(0, req, 12) < 0 )
  {
    ROS_ERROR();
  }
  updateVelocity(req);
}

void PlayerHWInterface::write()
{
  char pkt = fillPkt(motor_1.getCommand(), motor_2.getCommand(), motor3.getCommand(), motor_4.getCommand(), true, true);
  if ( wiringPiSPIDataRW(0, pkt, 12) < 0 )
  {
    ROS_ERROR();
  }
}

void PlayerHWInterface::updateVelocity(char* req)
{
  bool sign_motors[4];
  uint16_t motor_speeds[4];
  sign_motors[0] = req[2] & 0x80;
  motor_speeds[0] = (req[2] << 4) & 0x7F;
  motor_speeds[0] &= req[3];

  sign_motors[1] = req[4] & 0x80;
  motor_speeds[1] = (req[4] << 4) & 0x7F;
  motor_speeds[1] &= req[5];

  sign_motors[2] = req[6] & 0x80;
  motor_speeds[2] = (req[6] << 4) & 0x7F;
  motor_speeds[2] &= req[7];

  sign_motors[3] = req[8] & 0x80;
  motor_speeds[3] = (req[8] << 4) & 0x7F;
  motor_speeds[3] &= req[9];

  // update velocities
  for (int i = 0; i < 4; i++)
  {
    if (sign_motors[i])
    {
      vel[i] = (double) motor_speeds[i];
    }
    else
    {
      vel[i] = (double) motor_speeds[i];
      vel[i] *= -1;
    }
  }
}

char* fillPkt(double motor_1, double motor_2, double motor_3, double motor_4, bool kick, bool dribble)
{
  char pkt[12];
  pkt[0] = 0x2A;
  pkt[1] = 0xB7;
  for (int i = 0; i < 4; i++)
  {
    if (motor_speed[i] < 0)
    {
      pkt[(i+1)*2] = 0x80;
    }
    pkt[(i+1)*2] = (motor_speed[i] & 0x70) >> 4;
    pkt[(i+1) * 2 + 1] = motor_speed[i] & 0x0F;
  }
  if (kick)
  {
    pkt[10] = 1;
  }
  if (dribble)
  {
    pkt[11] = 1;
  }
  return pkt;
}

}
