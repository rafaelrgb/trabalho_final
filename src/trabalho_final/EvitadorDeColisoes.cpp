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
using namespace fl;

namespace trabalho_final
{

EvitadorDeColisoes::EvitadorDeColisoes(ros::NodeHandle *nh)
  : Node(nh, 10)
{
  // ----------------------------------------
  int a = 0;
  int b = 1;
  int c = 1.5;
  int d = 2;
  int e = 5;

  using namespace fl;

  engine_ = new fl::Engine("Evitador");

  sensor0_ = new fl::InputVariable;
  sensor0_->setEnabled(true);
  sensor0_->setName("Sensor0");
  sensor0_->setRange(0, 5);
  sensor0_->addTerm(new fl::Trapezoid("PERTO", a, a, b, c));
  sensor0_->addTerm(new fl::Triangle("MEDIO", b, d));
  sensor0_->addTerm(new fl::Trapezoid("LONGE", c, d, e, e));
  engine_->addInputVariable(sensor0_);

  sensor1_ = new fl::InputVariable;
  sensor1_->setEnabled(true);
  sensor1_->setName("Sensor1");
  sensor1_->setRange(0, 5);
  sensor1_->addTerm(new fl::Trapezoid("PERTO", a, a, b, c));
  sensor1_->addTerm(new fl::Triangle("MEDIO", b, d));
  sensor1_->addTerm(new fl::Trapezoid("LONGE", c, d, e, e));
  engine_->addInputVariable(sensor1_);

  sensor2_ = new fl::InputVariable;
  sensor2_->setEnabled(true);
  sensor2_->setName("Sensor2");
  sensor2_->setRange(0, 5);
  sensor2_->addTerm(new fl::Trapezoid("PERTO", a, a, b, c));
  sensor2_->addTerm(new fl::Triangle("MEDIO", b, d));
  sensor2_->addTerm(new fl::Trapezoid("LONGE", c, d, e, e));
  engine_->addInputVariable(sensor2_);

  sensor3_ = new fl::InputVariable;
  sensor3_->setEnabled(true);
  sensor3_->setName("Sensor3");
  sensor3_->setRange(0, 5);
  sensor3_->addTerm(new fl::Trapezoid("PERTO", a, a, b, c));
  sensor3_->addTerm(new fl::Triangle("MEDIO", b, d));
  sensor3_->addTerm(new fl::Trapezoid("LONGE", c, d, e, e));
  engine_->addInputVariable(sensor3_);

  sensor4_ = new fl::InputVariable;
  sensor4_->setEnabled(true);
  sensor4_->setName("Sensor4");
  sensor4_->setRange(0, 5);
  sensor4_->addTerm(new fl::Trapezoid("PERTO", a, a, b, c));
  sensor4_->addTerm(new fl::Triangle("MEDIO", b, d));
  sensor4_->addTerm(new fl::Trapezoid("LONGE", c, d, e, e));
  engine_->addInputVariable(sensor4_);

  sensor5_ = new fl::InputVariable;
  sensor5_->setEnabled(true);
  sensor5_->setName("Sensor5");
  sensor5_->setRange(0, 5);
  sensor5_->addTerm(new fl::Trapezoid("PERTO", a, a, b, c));
  sensor5_->addTerm(new fl::Triangle("MEDIO", b, d));
  sensor5_->addTerm(new fl::Trapezoid("LONGE", c, d, e, e));
  engine_->addInputVariable(sensor5_);

  vel_ang_ = new fl::OutputVariable;
  vel_ang_->setEnabled(true);
  vel_ang_->setName("VelAngular");
  vel_ang_->setRange(-2, 2);
  vel_ang_->setDefaultValue(fl::nan);
  vel_ang_->addTerm(new fl::Trapezoid("MDIREITA", -2, -2, -1, -0.5));
  vel_ang_->addTerm(new fl::Triangle("PDIREITA", -1, 0));
  vel_ang_->addTerm(new fl::Triangle("ZERO", -0.5, 0.5));
  vel_ang_->addTerm(new fl::Triangle("PESQUERDA", 0, 1));
  vel_ang_->addTerm(new fl::Trapezoid("MESQUERDA", 0.5, 1, 2, 2));
  engine_->addOutputVariable(vel_ang_);

  vel_lin_ = new fl::OutputVariable;
  vel_lin_->setEnabled(true);
  vel_lin_->setName("VelLinear");
  vel_lin_->setRange(0, 1);
  vel_lin_->setDefaultValue(fl::nan);
  vel_lin_->addTerm(new fl::Trapezoid("QPARADO", 0, 0, 0.125, 0,375));
  vel_lin_->addTerm(new fl::Triangle("DEVAGAR", 0.125, 0.625));
  vel_lin_->addTerm(new fl::Triangle("MEDIO", 0.375, 0.875));
  vel_lin_->addTerm(new fl::Trapezoid("RAPIDO", 0.625, 0.875, 1, 1));
  engine_->addOutputVariable(vel_lin_);

  rule_block_ = new fl::RuleBlock;
  rule_block_->setEnabled(true);
  rule_block_->setName("");

  rule_block_->addRule(fl::Rule::parse("if Sensor3 is PERTO then VelAngular is MESQUERDA", engine_));
  rule_block_->addRule(fl::Rule::parse("if Sensor3 is MEDIO and Sensor4 is PERTO then VelAngular is MESQUERDA", engine_));
  rule_block_->addRule(fl::Rule::parse("if Sensor3 is MEDIO and Sensor4 is MEDIO then VelAngular is PESQUERDA", engine_));
  rule_block_->addRule(fl::Rule::parse("if Sensor3 is MEDIO and Sensor4 is LONGE then VelAngular is PESQUERDA", engine_));
  rule_block_->addRule(fl::Rule::parse("if Sensor3 is LONGE and Sensor4 is PERTO then VelAngular is MESQUERDA", engine_));
  rule_block_->addRule(fl::Rule::parse("if Sensor3 is LONGE and Sensor4 is MEDIO then VelAngular is PESQUERDA", engine_));
  rule_block_->addRule(fl::Rule::parse("if Sensor3 is LONGE and Sensor4 is LONGE then VelAngular is ZERO", engine_));

  rule_block_->addRule(fl::Rule::parse("if Sensor2 is PERTO then VelAngular is MDIREITA", engine_));
  rule_block_->addRule(fl::Rule::parse("if Sensor2 is MEDIO and Sensor1 is PERTO then VelAngular is MDIREITA", engine_));
  rule_block_->addRule(fl::Rule::parse("if Sensor2 is MEDIO and Sensor1 is MEDIO then VelAngular is PDIREITA", engine_));
  rule_block_->addRule(fl::Rule::parse("if Sensor2 is MEDIO and Sensor1 is LONGE then VelAngular is PDIREITA", engine_));
  rule_block_->addRule(fl::Rule::parse("if Sensor2 is LONGE and Sensor1 is PERTO then VelAngular is MDIREITA", engine_));
  rule_block_->addRule(fl::Rule::parse("if Sensor2 is LONGE and Sensor1 is MEDIO then VelAngular is PDIREITA", engine_));
  rule_block_->addRule(fl::Rule::parse("if Sensor2 is LONGE and Sensor1 is LONGE then VelAngular is ZERO", engine_));


  rule_block_->addRule(fl::Rule::parse("if Sensor2 is MEDIO and Sensor3 is MEDIO and Sensor5 is LONGE "
                                       "then VelAngular is MDIREITA", engine_));
  rule_block_->addRule(fl::Rule::parse("if Sensor2 is MEDIO and Sensor3 is MEDIO and Sensor1 is LONGE "
                                       "then VelAngular is MESQUERDA", engine_));
  rule_block_->addRule(fl::Rule::parse("if Sensor2 is MEDIO and Sensor3 is MEDIO and Sensor5 is LONGE and Sensor1 is LONGE "
                                       "then VelAngular is MDIREITA", engine_));

  rule_block_->addRule(fl::Rule::parse("if Sensor2 is PERTO or Sensor3 is PERTO then VelLinear is QPARADO", engine_));
  rule_block_->addRule(fl::Rule::parse("if Sensor1 is PERTO or Sensor5 is PERTO then VelLinear is DEVAGAR", engine_));
  rule_block_->addRule(fl::Rule::parse("if Sensor1 is MEDIO then VelLinear is MEDIO", engine_));
  rule_block_->addRule(fl::Rule::parse("if Sensor2 is MEDIO then VelLinear is MEDIO", engine_));
  rule_block_->addRule(fl::Rule::parse("if Sensor3 is MEDIO then VelLinear is MEDIO", engine_));
  rule_block_->addRule(fl::Rule::parse("if Sensor4 is MEDIO then VelLinear is MEDIO", engine_));
  rule_block_->addRule(fl::Rule::parse("if Sensor1 is LONGE then VelLinear is RAPIDO", engine_));
  rule_block_->addRule(fl::Rule::parse("if Sensor2 is LONGE then VelLinear is RAPIDO", engine_));
  rule_block_->addRule(fl::Rule::parse("if Sensor3 is LONGE then VelLinear is RAPIDO", engine_));
  rule_block_->addRule(fl::Rule::parse("if Sensor4 is LONGE then VelLinear is RAPIDO", engine_));

  engine_->addRuleBlock(rule_block_);

  engine_->configure("Minimum", "Maximum", "Minimum", "Maximum", "Centroid");
  //conjunction, disjunction, activation, accumulation, defuzzifier

//  std::string status;
//  if (!engine_->isReady(&status))
//  {
//    throw fl::Exception("Engine not ready. The following errors were encountered: " + status, FL_AT);
//  }

  sonar_sub_ = nh->subscribe("RosAria/sonar_range", 1, &EvitadorDeColisoes::sonarCb, this);
  cmd_vel_sub_ = nh->subscribe("cmd_vel", 1, &EvitadorDeColisoes::cmdVelCb, this);
  cmd_vel_pub_ = nh->advertise<geometry_msgs::Twist>("RosAria/cmd_vel", 1);
  //ROS_INFO("2015100688 - Rafael Gomes Braga"); //exibe na tela o numero de matricula e o nome
  //ROS_INFO("25510 - Christian Tossani Pedroso de Souza"); //exibe na tela o numero de matricula e o nome
  vel_x_ = 0.0;
  vel_theta_ = 0.0;
}
// ----------------------------------------

EvitadorDeColisoes::~EvitadorDeColisoes()
{
  sonar_sub_.shutdown();
  cmd_vel_sub_.shutdown();
  cmd_vel_pub_.shutdown();

  if (engine_)
  {
    delete engine_;
  }
  if (sensor0_)
  {
    delete sensor0_;
  }
  if (sensor1_)
  {
    delete sensor1_;
  }
  if (sensor2_)
  {
    delete sensor2_;
  }
  if (sensor3_)
  {
    delete sensor3_;
  }
  if (sensor4_)
  {
    delete sensor4_;
  }
  if (sensor5_)
  {
    delete sensor5_;
  }
  if (vel_ang_)
  {
    delete vel_ang_;
  }
  if (vel_lin_)
  {
    delete vel_lin_;
  }
  if (rule_block_)
  {
    delete rule_block_;
  }

}

void EvitadorDeColisoes::sonarCb(const std_msgs::Float32MultiArrayConstPtr& msg)
{
  //  for (int i = 0; i < msg->data.size(); i++)
  //  {
  //    ROS_INFO("Range %d medido: [%f]", i, msg->data[i]);
  //  }
    for (int i = 0; i < msg->data.size(); i++)
    {
      range_sensor_[i] = msg->data[i];
    }

}

void EvitadorDeColisoes::cmdVelCb(const geometry_msgs::TwistConstPtr& msg)
{
  vel_x_ = msg->linear.x;
  vel_theta_ = msg->angular.z;
}

void EvitadorDeColisoes::controlLoop()
{
  sensor0_->setInputValue(range_sensor_[0]);
  sensor1_->setInputValue(range_sensor_[1]);
  sensor2_->setInputValue(range_sensor_[2]);
  sensor3_->setInputValue(range_sensor_[3]);
  sensor4_->setInputValue(range_sensor_[4]);
  sensor5_->setInputValue(range_sensor_[5]);
  engine_->process();
  double output_ang(vel_ang_->getOutputValue());
  double output_lin(vel_lin_->getOutputValue());
  ROS_INFO("Output angular: [%f]", output_ang);
  ROS_INFO("Output linear: [%f]", output_lin);
  vel_x_ =  vel_x_*output_lin;
  vel_theta_ = vel_theta_ + output_ang;
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

