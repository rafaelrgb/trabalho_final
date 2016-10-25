#include <stdlib.h>
#include "trabalho_final/ControladorDeDeslocamento.h"

int main(int argc, char **argv)
{
  // Set up ROS.
  ros::init(argc, argv, "controlador_de_deslocamento");
  trabalho_final::ControladorDeDeslocamento node(new ros::NodeHandle());
  node.spin();
  return EXIT_SUCCESS;
}
