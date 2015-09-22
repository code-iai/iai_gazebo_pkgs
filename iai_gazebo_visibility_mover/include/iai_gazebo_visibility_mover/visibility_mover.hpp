#ifndef IAI_GAZEBO_VISIBILITY_MOVER_HPP
#define IAI_GAZEBO_VISIBILITY_MOVER_HPP

#include <ros/ros.h>

namespace iai_gazebo
{
  class VisibilityMover
  {
    public:
      VisibilityMover(const ros::NodeHandle& nh);
      ~VisibilityMover();
      bool start();

    private:
      ros::NodeHandle nh_;
      ros::ServiceClient spawn_urdf_client_;
      std::string robot_description_;

      bool spawnUrdf();
  };
}

#endif // IAI_GAZEBO_VISIBILITY_MOVER_HPP
