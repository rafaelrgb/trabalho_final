/**
 *  This source file implements the ControladorDeDeslocamento class, which is also a
 *  Node class via enhancement.
 *
 *  Version: 1.0.0
 *  Created on: 24/10/2016
 *  Modified on: 24/10/2016
 *  Author: Rafael Gomes Braga (faerugb@gmail.com)
 *  Maintainer: Rafael Gomes Braga (faerugb@gmail.com)
 */

#include "trabalho_final/ControladorDeDeslocamento.h"

namespace trabalho_final
{

ControladorDeDeslocamento::ControladorDeDeslocamento(ros::NodeHandle *nh)
  : Node(nh, 10)
{
  pose_2d_sub_ = nh->subscribe("RosAria/pose2D", 1, &ControladorDeDeslocamento::pose2DCb, this);
  cmd_vel_pub_ = nh->advertise<geometry_msgs::Twist>("cmd_vel", 1);
  //ROS_INFO("2015100688 - Rafael Gomes Braga"); //exibe na tela o numero de matricula e o nome
  //ROS_INFO("25510 - Christian Tossani Pedroso de Souza"); //exibe na tela o numero de matricula e o nome
}

ControladorDeDeslocamento::~ControladorDeDeslocamento()
{
  pose_2d_sub_.shutdown();
  cmd_vel_pub_.shutdown();
}

void ControladorDeDeslocamento::pose2DCb(const geometry_msgs::Pose2DConstPtr& msg)
{
  /*
  for (int i = 0; i < msg->ranges.size(); i++)
  {
    ROS_INFO("Range medido: [%f]", msg->ranges[i]);
  }*/
}

void ControladorDeDeslocamento::controlLoop()
{
  publishVelocity(150.0, 1.5);
}

void ControladorDeDeslocamento::publishVelocity(double vel_x, double vel_theta)
{
  geometry_msgs::Twist msg;
  msg.linear.x = vel_x;
  msg.angular.z = vel_theta;
  cmd_vel_pub_.publish(msg);
}

}

