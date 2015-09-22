#include <iai_gazebo_visibility_mover/visibility_mover.hpp>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "visibility_mover");
  ros::NodeHandle n("~");
  iai_gazebo::VisibilityMover vm(n);
  ROS_INFO("[%s] Starting up.", n.getNamespace().c_str());
  if(vm.start())
    ros::spin();
  else
    ROS_ERROR("[%s] Error during startup.", n.getNamespace().c_str());

  return 0;
}
