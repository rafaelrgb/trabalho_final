#include <stdlib.h>
#include "trabalho_final/EvitadorDeColisoes.h"

int main(int argc, char **argv)
{
  // Set up ROS.
  ros::init(argc, argv, "evitador_de_colisoes");
  trabalho_final::EvitadorDeColisoes node(new ros::NodeHandle());
  node.spin();
  return EXIT_SUCCESS;
}
