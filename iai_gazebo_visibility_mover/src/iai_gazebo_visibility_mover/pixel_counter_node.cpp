#include <iai_gazebo_visibility_mover/pixel_counter.hpp>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pixel_counter");
  ros::NodeHandle n("~");

  iai_gazebo::PixelCounter pc(n);
  ROS_INFO("[%s] Starting up.", n.getNamespace().c_str());
  if(pc.start())
    ros::spin();
  else
    ROS_ERROR("[%s] Error during startup.", n.getNamespace().c_str());

  return 0;
}
