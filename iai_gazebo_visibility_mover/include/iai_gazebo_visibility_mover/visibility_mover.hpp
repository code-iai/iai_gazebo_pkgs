#ifndef IAI_GAZEBO_VISIBILITY_MOVER_HPP
#define IAI_GAZEBO_VISIBILITY_MOVER_HPP

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/UInt64.h>
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
          set_joint_states_client_, model_state_client_, clear_body_wrench_client_,
          set_model_state_client_;
      ros::ServiceServer trigger_server_;
      ros::Subscriber joint_state_subscriber_, pixel_count_subscriber_;
      ros::Publisher neck_command_publisher_;
      gazebo::transport::PublisherPtr world_control_publisher_;

      // Internal memory of things
      std::string robot_description_;
      sensor_msgs::JointState last_q_;
      size_t pixel_count_;
      bool has_new_pixel_count_;
      std::vector< std::map<std::string, double> > alternative_configs_;

      // Callbacks
      void joint_state_callback(const sensor_msgs::JointState::ConstPtr& msg);
      bool trigger_callback(std_srvs::Trigger::Request& request,
          std_srvs::Trigger::Response& response);
      void pixel_count_callback(const std_msgs::UInt64::ConstPtr& msg);

      // Aux
      bool find_model(const std::string& robot_name);
      bool spawn_urdf(const std::string& urdf, const std::string& robot_name);
      bool delete_model(const std::string& robot_name);
      bool set_joint_states(const std::map< std::string, double >& q, 
          const std::string& robot_name, size_t iterations);
      bool clear_body_wrenches();
      bool reset_base_pose(const std::string& robot_name);
      bool step_simulation(size_t steps);
      bool target_visible(const std::map< std::string, double>& q, const std::string& robot_name,
          double threshold);
      void wait_for_pixel_count();
      bool read_alternative_configs();
      std::map<std::string, double> to_map(const sensor_msgs::JointState& q) const;
      sensor_msgs::JointState to_msg(const std::map<std::string, double>& q) const;
      std::map<std::string, double> merge_configs(const std::map<std::string, double>& base,
          const std::map<std::string, double>& replacement) const;
  };
}

#endif // IAI_GAZEBO_VISIBILITY_MOVER_HPP
