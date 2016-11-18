/**
 *  This header file defines the ControladorDeDeslocamento class, which is also a
 *  Node class via enhancement.
 *
 *  Version: 1.0.0
 *  Created on: 24/10/2016
 *  Modified on: 24/10/2016
 *  Author: Rafael Gomes Braga (faerugb@gmail.com)
 *  Maintainer: Rafael Gomes Braga (faerugb@gmail.com)
 */

#ifndef _CONTROLADOR_DE_DESLOCAMENTO_H_
#define _CONTROLADOR_DE_DESLOCAMENTO_H_

#include "Node.h"
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>

namespace trabalho_final
{

class ControladorDeDeslocamento : public Node
{
public:
  ControladorDeDeslocamento(ros::NodeHandle *nh);
  virtual ~ControladorDeDeslocamento();

private:
  virtual void controlLoop();
  void pose2DCb(const geometry_msgs::Pose2DConstPtr& msg);
  void objectiveCb(const geometry_msgs::Point32ConstPtr& msg);
  void controlEffortCb(const std_msgs::Float64ConstPtr& msg);
  void publishVelocity(double vel_x, double vel_theta);
  void publishSetpoint(double theta);
  ros::Subscriber pose_2d_sub_;
  ros::Subscriber objective_sub_;
  ros::Subscriber control_effort_sub_;
  ros::Publisher cmd_vel_pub_;
  ros::Publisher setpoint_pub_;

  double objectiveX_;
  double objectiveY_;
  double tolerance_;
  double vel_x_;
  double vel_theta_;
};

}

#endif // _CONTROLADOR_DE_DESLOCAMENTO_H_
