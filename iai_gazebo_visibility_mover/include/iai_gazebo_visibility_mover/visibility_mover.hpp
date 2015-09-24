#ifndef IAI_GAZEBO_VISIBILITY_MOVER_HPP
#define IAI_GAZEBO_VISIBILITY_MOVER_HPP

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_srvs/Trigger.h>
#include <gazebo/transport/transport.hh>

namespace iai_gazebo
{
  class VisibilityMover
  {
    public:
      VisibilityMover(const ros::NodeHandle& nh);
      ~VisibilityMover();
      bool start();

    private:
      // Communications
      ros::NodeHandle nh_;
      ros::ServiceClient delete_model_client_, spawn_urdf_client_, 
          set_joint_states_client_, model_state_client_;
      ros::ServiceServer trigger_server_;
      ros::Subscriber joint_state_subscriber_;
      gazebo::transport::PublisherPtr world_control_publisher_;

      // Internal memory of things
      std::string robot_description_;
      sensor_msgs::JointState last_q_;

      // Callbacks
      void joint_state_callback(const sensor_msgs::JointState::ConstPtr& msg);
      bool trigger_callback(std_srvs::Trigger::Request& request,
          std_srvs::Trigger::Response& response);

      // Aux
      bool find_model(const std::string& robot_name);
      bool spawn_urdf(const std::string& urdf, const std::string& robot_name);
      bool delete_model(const std::string& robot_name);
      bool set_joint_states(const sensor_msgs::JointState& q, const std::string& robot_name);
      bool step_simulation(size_t steps = 10);
      bool target_visible(const sensor_msgs::JointState& q, const std::string& robot_name);
  };
}

#endif // IAI_GAZEBO_VISIBILITY_MOVER_HPP
