#ifndef IAI_GAZEBO_VISIBILITY_MOVER_HPP
#define IAI_GAZEBO_VISIBILITY_MOVER_HPP

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

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
      ros::Subscriber joint_state_subscriber_;
      std::string robot_description_;
      sensor_msgs::JointState last_q_;

      void joint_state_callback(const sensor_msgs::JointState::ConstPtr& msg);
      bool spawnUrdf();
  };
}

#endif // IAI_GAZEBO_VISIBILITY_MOVER_HPP