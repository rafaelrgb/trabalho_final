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
#include <geometry_msgs/Twist.h>

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
  void publishVelocity(double vel_x, double vel_theta);
  ros::Subscriber pose_2d_sub_;
  ros::Publisher cmd_vel_pub_;
};

}

#endif // _CONTROLADOR_DE_DESLOCAMENTO_H_
