/**
 *  This source file implements the EvitadorDeColisoes class, which is also a
 *  Node class via enhancement.
 *
 *  Version: 1.0.0
 *  Created on: 24/10/2016
 *  Modified on: 24/10/2016
 *  Author: Rafael Gomes Braga (faerugb@gmail.com)
 *  Maintainer: Rafael Gomes Braga (faerugb@gmail.com)
 */

#include "trabalho_final/EvitadorDeColisoes.h"

namespace trabalho_final
{

EvitadorDeColisoes::EvitadorDeColisoes(ros::NodeHandle *nh)
  : Node(nh, 10)
{
  sonar_sub_ = nh->subscribe("RosAria/sonar_range", 1, &EvitadorDeColisoes::sonarCb, this);
  cmd_vel_sub_ = nh->subscribe("cmd_vel", 1, &EvitadorDeColisoes::cmdVelCb, this);
  cmd_vel_pub_ = nh->advertise<geometry_msgs::Twist>("RosAria/cmd_vel", 1);
  //ROS_INFO("2015100688 - Rafael Gomes Braga"); //exibe na tela o numero de matricula e o nome
  //ROS_INFO("25510 - Christian Tossani Pedroso de Souza"); //exibe na tela o numero de matricula e o nome
}

EvitadorDeColisoes::~EvitadorDeColisoes()
{
  sonar_sub_.shutdown();
  cmd_vel_sub_.shutdown();
  cmd_vel_pub_.shutdown();
}

void EvitadorDeColisoes::sonarCb(const sensor_msgs::LaserScanConstPtr& msg)
{
  /*
  for (int i = 0; i < msg->ranges.size(); i++)
  {
    ROS_INFO("Range medido: [%f]", msg->ranges[i]);
  }*/
}

void EvitadorDeColisoes::cmdVelCb(const geometry_msgs::TwistConstPtr& msg)
{
  vel_x_ = msg->linear.x;
  vel_theta_ = msg->angular.z;
}

void EvitadorDeColisoes::controlLoop()
{
  publishVelocity(vel_x_, vel_theta_);
}

void EvitadorDeColisoes::publishVelocity(double vel_x, double vel_theta)
{
  geometry_msgs::Twist msg;
  msg.linear.x = vel_x;
  msg.angular.z = vel_theta;
  cmd_vel_pub_.publish(msg);
}

}

