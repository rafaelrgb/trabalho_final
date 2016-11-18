#include <ros/ros.h>
#include <geometry_msgs/Point32.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "setpoint_node");
  ROS_INFO("Starting setpoint publisher");
  ros::NodeHandle setpoint_node;

  while (ros::Time(0) == ros::Time::now())
  {
    ROS_INFO("Setpoint_node spinning waiting for time to become non-zero");
    sleep(1);
  }

  geometry_msgs::Point32 objective;
  objective.x = 2.0;
  objective.y = 2.0;
  objective.z = 0.0;
  ros::Publisher setpoint_pub = setpoint_node.advertise<geometry_msgs::Point32>("objective", 1);

  ros::Rate loop_rate(0.05);   // change setpoint every 20 seconds

  while (ros::ok())
  {
    ros::spinOnce();

    setpoint_pub.publish(objective);     // publish twice so graph gets it as a step
    objective.y = 0 - objective.y;
    setpoint_pub.publish(objective);

    loop_rate.sleep();
  }
}
