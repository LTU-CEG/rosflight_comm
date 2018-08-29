#include "rosflight_comm/rosflight_comm.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "rosflight_comm_node");

  ros::NodeHandle n;
  ros::NodeHandle pn("~");

  rosflight_communication comm(n, pn);

  comm.ros_loop();

  return 0;
}
