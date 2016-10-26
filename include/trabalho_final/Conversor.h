/**
 *  This header file defines the Conversor class, which is also a
 *  Node class via enhancement.
 *
 *  Version: 1.0.0
 *  Created on: 24/10/2016
 *  Modified on: 24/10/2016
 *  Author: Rafael Gomes Braga (faerugb@gmail.com)
 *  Maintainer: Rafael Gomes Braga (faerugb@gmail.com)
 */

#ifndef _CONVERSOR_H_
#define _CONVERSOR_H_

#include "Node.h"
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose2D.h>
#include <tf/tf.h>

namespace trabalho_final
{

class Conversor : public Node
{
public:
  Conversor(ros::NodeHandle *nh);
  virtual ~Conversor();

private:
  virtual void controlLoop();
  void poseCb(const nav_msgs::OdometryConstPtr& msg);
  void publishPose2D(double x, double y, double theta);
  ros::Subscriber pose_sub_;
  ros::Publisher pose_2d_pub_;
  double x_;
  double y_;
  double theta_;
};

}

#endif // _CONVERSOR_H_
