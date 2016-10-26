/**
 *  This source file implements the Conversor class, which is also a
 *  Node class via enhancement.
 *
 *  Version: 1.0.0
 *  Created on: 24/10/2016
 *  Modified on: 24/10/2016
 *  Author: Rafael Gomes Braga (faerugb@gmail.com)
 *  Maintainer: Rafael Gomes Braga (faerugb@gmail.com)
 */

#include "trabalho_final/Conversor.h"

namespace trabalho_final
{

Conversor::Conversor(ros::NodeHandle *nh)
  : Node(nh, 10)
{
  pose_sub_ = nh->subscribe("RosAria/pose", 1, &Conversor::poseCb, this);
  pose_2d_pub_ = nh->advertise<geometry_msgs::Pose2D>("RosAria/pose2D", 1);
  //ROS_INFO("2015100688 - Rafael Gomes Braga"); //exibe na tela o numero de matricula e o nome
  //ROS_INFO("25510 - Christian Tossani Pedroso de Souza"); //exibe na tela o numero de matricula e o nome
  x_ = 0.0;
  y_ = 0.0;
  theta_ = 0.0;
}

Conversor::~Conversor()
{
  pose_sub_.shutdown();
  pose_2d_pub_.shutdown();
}

void Conversor::poseCb(const nav_msgs::Odometry::ConstPtr& msg)
{
  x_ = msg->pose.pose.position.x;
  y_ = msg->pose.pose.position.y;
  theta_ = tf::getYaw(msg->pose.pose.orientation);
}

void Conversor::controlLoop()
{
  publishPose2D(x_, y_, theta_);
}

void Conversor::publishPose2D(double x, double y, double theta)
{
  geometry_msgs::Pose2D msg;
  msg.x = x;
  msg.y = y;
  msg.theta = theta;
  pose_2d_pub_.publish(msg);
}

}

