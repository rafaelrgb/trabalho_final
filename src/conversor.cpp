#include <stdlib.h>
#include "trabalho_final/Conversor.h"

int main(int argc, char **argv)
{
  // Set up ROS.
  ros::init(argc, argv, "conversor");
  trabalho_final::Conversor node(new ros::NodeHandle());
  node.spin();
  return EXIT_SUCCESS;
}
