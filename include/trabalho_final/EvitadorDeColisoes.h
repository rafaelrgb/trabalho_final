/**
 *  This header file defines the EvitadorDeColisoes class, which is also a
 *  Node class via enhancement.
 *
 *  Version: 1.0.0
 *  Created on: 24/10/2016
 *  Modified on: 24/10/2016
 *  Author: Rafael Gomes Braga (faerugb@gmail.com)
 *  Maintainer: Rafael Gomes Braga (faerugb@gmail.com)
 */

#ifndef _EVITADOR_DE_COLISOES_H_
#define _EVITADOR_DE_COLISOES_H_

#include "Node.h"
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/Twist.h>
#include <fl/Headers.h>

namespace trabalho_final
{

class EvitadorDeColisoes : public Node
{
public:
  EvitadorDeColisoes(ros::NodeHandle *nh);
  virtual ~EvitadorDeColisoes();

private:
  virtual void controlLoop();
  void sonarCb(const std_msgs::Float32MultiArrayConstPtr& msg);
  void cmdVelCb(const geometry_msgs::TwistConstPtr& msg);
  void publishVelocity(double vel_x, double vel_theta);
  ros::Subscriber sonar_sub_;
  ros::Subscriber cmd_vel_sub_;
  ros::Publisher cmd_vel_pub_;
  double vel_x_;
  double vel_theta_;
  double range_sensor_[8];
  fl::Engine* engine_;
  fl::InputVariable* sensor0_;
  fl::InputVariable* sensor1_;
  fl::InputVariable* sensor2_;
  fl::InputVariable* sensor3_;
  fl::InputVariable* sensor4_;
  fl::InputVariable* sensor5_;
  fl::OutputVariable* vel_ang_;
  fl::OutputVariable* vel_lin_;
  fl::RuleBlock* rule_block_;
  int counter_;
};

}

#endif // _EVITADOR_DE_COLISOES_H_
