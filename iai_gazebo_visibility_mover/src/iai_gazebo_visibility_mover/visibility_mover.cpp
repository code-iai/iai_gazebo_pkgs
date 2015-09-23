#include <iai_gazebo_visibility_mover/visibility_mover.hpp>
#include <gazebo_msgs/SpawnModel.h>
#include <gazebo_msgs/SetModelConfiguration.h>
#include <gazebo/msgs/msgs.hh>

using namespace iai_gazebo;

VisibilityMover::VisibilityMover(const ros::NodeHandle& nh) :
    nh_( nh )
{
}

VisibilityMover::~VisibilityMover()
{
}

bool VisibilityMover::start()
{
  if(!nh_.getParam("robot_description", robot_description_))
  {
    ROS_ERROR("[%s] Could not find param 'robot_description' in namespace '%s'",
        nh_.getNamespace().c_str(), nh_.getNamespace().c_str());
    return false;
  }

  spawn_urdf_client_ = nh_.serviceClient<gazebo_msgs::SpawnModel>("/gazebo/spawn_urdf_model");
  if(!spawn_urdf_client_.waitForExistence(ros::Duration(2.0)))
  {
    ROS_ERROR("[%s] Could not connect to service '/gazebo/spawn_urdf_model'",
        nh_.getNamespace().c_str());
    return false;
  }

  set_joint_states_client_ = nh_.serviceClient<gazebo_msgs::SetModelConfiguration>("/gazebo/set_model_configuration");
  if(!set_joint_states_client_.waitForExistence(ros::Duration(2.0)))
  {
    ROS_ERROR("[%s] Could not connect to service '/gazebo/set_model_configuration'",
        nh_.getNamespace().c_str());
    return false;
  }

  gazebo::transport::NodePtr node = gazebo::transport::NodePtr(new gazebo::transport::Node());
  node->Init();
  world_control_publisher_ = node->Advertise<gazebo::msgs::WorldControl>("/gazebo/default/world_control");

  trigger_server_ = nh_.advertiseService("trigger", 
      &VisibilityMover::trigger_callback, this);

  joint_state_subscriber_ = nh_.subscribe("joint_states", 1, 
      &VisibilityMover::joint_state_callback, this);

  if(!spawnUrdf())
    return false;
 
  return true;
}

bool VisibilityMover::spawnUrdf()
{
  gazebo_msgs::SpawnModel srv;
  srv.request.model_name = "Boxy";
  srv.request.model_xml = robot_description_;
  srv.request.robot_namespace = "boxy";

  if(spawn_urdf_client_.call(srv))
  {
    if(!srv.response.success)
    {
      ROS_ERROR("[%s] Spawn Urdf unsuccessful: %s", nh_.getNamespace().c_str(),
        srv.response.status_message.c_str()); 
      return false;
    }
    else 
      return true;
  }
  else
  {
    ROS_ERROR("[%s] Failed to call service '/gazebo/spawn_urdf_model'.",
        nh_.getNamespace().c_str());
    return false;
  }
}

bool VisibilityMover::set_joint_states()
{
  gazebo_msgs::SetModelConfiguration srv;
  srv.request.model_name = "Boxy";
  srv.request.urdf_param_name = nh_.getNamespace() + "/robot_description";
  srv.request.joint_names = last_q_.name;
  srv.request.joint_positions = last_q_.position;

  // DEBUG PRINTOUT
  //std::string joint_names, joint_positions;
  //for(size_t i=0; i<last_q_.name.size(); ++i)
  //  joint_names += " " + last_q_.name[i];
  //for(size_t i=0; i<last_q_.position.size(); ++i)
  //  joint_positions += " " + boost::lexical_cast<std::string>(last_q_.position[i]);
  //ROS_INFO_STREAM("Setting joint states\n" << joint_names << "\n" << joint_positions);

  if(set_joint_states_client_.call(srv))
  {
    if(!srv.response.success)
    {
      ROS_ERROR("[%s] Set Joint States unsuccessful: %s", nh_.getNamespace().c_str(),
        srv.response.status_message.c_str()); 
      return false;
    }
    else 
      return true;
  }
  else
  {
    ROS_ERROR("[%s] Failed to call service '/gazebo/set_model_configuration'.",
        nh_.getNamespace().c_str());
    return false;
  }
}
void VisibilityMover::joint_state_callback(const sensor_msgs::JointState::ConstPtr& msg)
{
  last_q_ = *msg;
}

bool VisibilityMover::trigger_callback(std_srvs::Trigger::Request& request,
    std_srvs::Trigger::Response& response)
{
  response.success = false;

  if(!set_joint_states())
    return false;
  // FIXME: find out why we sometimes need 2 of these
  if(!set_joint_states())
    return false;

  if(!step_simulation(1))
    return false;

  response.success = true;
  return true;
}

bool VisibilityMover::step_simulation(size_t steps)
{
  gazebo::msgs::WorldControl msg;
  msg.set_multi_step(steps);
  world_control_publisher_->Publish(msg);

  return true;
}
