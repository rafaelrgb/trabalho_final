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
  objective_sub_ = nh->subscribe("objective", 1, &ControladorDeDeslocamento::objectiveCb, this);
  control_effort_sub_ = nh->subscribe("/control_effort", 1, &ControladorDeDeslocamento::controlEffortCb, this);
  cmd_vel_pub_ = nh->advertise<geometry_msgs::Twist>("cmd_vel", 1);
  setpoint_pub_ = nh->advertise<std_msgs::Float64>("/setpoint", 1);
  theta_pub_ = nh->advertise<std_msgs::Float64>("/state", 1);

  vel_x_ = 0.0;
  objectiveX_ = 0.0;
  objectiveY_ = 0.0;
  vel_theta_ = 0.0;
  min_tolerance_ = 0.4;
  max_tolerance_ = 2.0;
}

ControladorDeDeslocamento::~ControladorDeDeslocamento()
{
  pose_2d_sub_.shutdown();
  objective_sub_.shutdown();
  control_effort_sub_.shutdown();
  cmd_vel_pub_.shutdown();
  setpoint_pub_.shutdown();
  theta_pub_.shutdown();
}

void ControladorDeDeslocamento::controlLoop()
{
  //publishVelocity(1.0, 0.0);
}

void ControladorDeDeslocamento::pose2DCb(const geometry_msgs::Pose2DConstPtr& msg)
{
  // Get robot's current pose
  double x = msg->x;
  double y = msg->y;
  double theta = msg->theta;

  // Find distance between current position and objective position
  double distance = sqrt( pow((objectiveX_ - x), 2) + pow((objectiveY_ - y), 2) );

  // Calculate the angle that points to the objective
  double objective_theta = atan2( (objectiveY_ - y), (objectiveX_ - x) );

  double setpoint = objective_theta;
  double t1 = objective_theta - 2 * PI;
  double t2 = objective_theta + 2 * PI;

  double min_dist = 1000;

  if ( abs(theta - t1) < min_dist )
  {
    min_dist = abs(theta - t1);
    setpoint = t1;
  }

  if ( abs(theta - objective_theta) < min_dist )
  {
    min_dist = abs(theta - objective_theta);
    setpoint = objective_theta;
  }

  if ( abs(theta - t2) < min_dist )
  {
    min_dist = abs(theta - t2);
    setpoint = t2;
  }

  // Control theta by publishing a setpoint and state to the pid node
  publishTheta(theta);
  publishSetpoint(setpoint);

  ROS_INFO("\nx = %f | y = %f | theta = %f \n objective_X = %f | objective_Y = %f | objective_theta = %f \n distance = %f",
           x, y, theta, objectiveX_, objectiveY_, objective_theta, distance);

  // Robot should not move if it's close enough to the objective
  if ( distance > max_tolerance_ )
  {
    // Move the robot
    vel_x_ = 1.0;

    // Publish velocity to move the robot
    publishVelocity(vel_x_, vel_theta_);
  }
  else if ( distance <= max_tolerance_ && distance > min_tolerance_ )
  {
    // Move the robot
    vel_x_ = 0.5;

    // Publish velocity to move the robot
    publishVelocity(vel_x_, vel_theta_);
  }
  else
  {
    // Stop the robot
    publishVelocity(0.0, 0.0);
  }
}

void ControladorDeDeslocamento::objectiveCb(const geometry_msgs::Point32ConstPtr& msg)
{
  objectiveX_ = msg->x;
  objectiveY_ = msg->y;
}

void ControladorDeDeslocamento::controlEffortCb(const std_msgs::Float64ConstPtr& msg)
{
  vel_theta_ = msg->data;
}

void ControladorDeDeslocamento::publishVelocity(double vel_x, double vel_theta)
{
  geometry_msgs::Twist msg;
  msg.linear.x = vel_x;
  msg.angular.z = vel_theta;
  cmd_vel_pub_.publish(msg);
}

void ControladorDeDeslocamento::publishSetpoint(double theta)
{
  std_msgs::Float64 msg;
  msg.data = theta;
  setpoint_pub_.publish(msg);
}

void ControladorDeDeslocamento::publishTheta(double theta)
{
  std_msgs::Float64 msg;
  msg.data = theta;
  theta_pub_.publish(msg);
}

}

