#include <iai_gazebo_visibility_mover/visibility_mover.hpp>
#include <gazebo_msgs/SpawnModel.h>
#include <gazebo_msgs/DeleteModel.h>
#include <gazebo_msgs/SetModelConfiguration.h>
#include <gazebo_msgs/GetModelState.h>
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

  delete_model_client_ = nh_.serviceClient<gazebo_msgs::DeleteModel>("/gazebo/delete_model");
  if(!delete_model_client_.waitForExistence(ros::Duration(2.0)))
  {
    ROS_ERROR("[%s] Could not connect to service '/gazebo/delete_model'",
        nh_.getNamespace().c_str());
    return false;
  }

  model_state_client_ = nh_.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
  if(!model_state_client_.waitForExistence(ros::Duration(2.0)))
  {
    ROS_ERROR("[%s] Could not connect to service '/gazebo/get_model_state'",
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

  if(!spawn_urdf(robot_description_, "boxy"))
    return false;

  return true;
}

bool VisibilityMover::spawn_urdf(const std::string& urdf, const std::string& robot_name)
{
  gazebo_msgs::SpawnModel srv;
  srv.request.model_name = robot_name;
  srv.request.model_xml = urdf;
  srv.request.robot_namespace = robot_name;

  bool result = false;

  if(spawn_urdf_client_.call(srv))
  {
    if(!srv.response.success)
    {
      ROS_ERROR("[%s] Spawn Urdf unsuccessful: %s", nh_.getNamespace().c_str(),
        srv.response.status_message.c_str()); 
      result = false;
    }
    else 
      result = true;
  }
  else
  {
    ROS_ERROR("[%s] Failed to call service '/gazebo/spawn_urdf_model'.",
        nh_.getNamespace().c_str());
    result = false;
  }

  // HACK: The plugin API of gazebo_ros unconditionally sets /use_sim_time=true.
  //       We do not want this because it srews with the ROS time. This happens
  //       everytime we spawn a robot with such a plugin inside. Our robot has such
  //       plugins. Hence, we undo it here, everytime.
  nh_.setParam("/use_sim_time", false);

  return result;
}

bool VisibilityMover::delete_model(const std::string& robot_name)
{
  gazebo_msgs::DeleteModel srv;
  srv.request.model_name = robot_name;

  bool result = false;

  if(find_model(robot_name))
  {
    if(delete_model_client_.call(srv))
    {
      if(!srv.response.success)
      {
        ROS_ERROR("[%s] Delete Model unsuccessful: %s", nh_.getNamespace().c_str(),
          srv.response.status_message.c_str()); 
        result = false;
      }
      else 
        result = true;
    }
    else
    {
      ROS_ERROR("[%s] Failed to call service '/gazebo/delete_model'.",
          nh_.getNamespace().c_str());
      result = false;
    }
  }
  else
    result = true;

  return result;
}

bool VisibilityMover::find_model(const std::string& robot_name)
{
  gazebo_msgs::GetModelState srv;
  srv.request.model_name = robot_name;

  bool result = false;

  if(model_state_client_.call(srv))
  {
    if(!srv.response.success)
    {
      ROS_ERROR("[%s] Get Model State unsuccessful: %s", nh_.getNamespace().c_str(),
        srv.response.status_message.c_str()); 
      result = false;
    }
    else 
      result = true;
  }
  else
  {
    ROS_ERROR("[%s] Failed to call service '/gazebo/get_model_state'.",
        nh_.getNamespace().c_str());
    result = false;
  }

  if(!spawn_urdf(robot_description_, robot_name))
    return false;

  return result;
}

bool VisibilityMover::set_joint_states(const sensor_msgs::JointState& q, const std::string& robot_name)
{
  gazebo_msgs::SetModelConfiguration srv;
  srv.request.model_name = robot_name;
  srv.request.urdf_param_name = nh_.getNamespace() + "/robot_description";
  srv.request.joint_names = q.name;
  srv.request.joint_positions = q.position;

  // DEBUG PRINTOUT
  std::string joint_names, joint_positions;
  for(size_t i=0; i<q.name.size(); ++i)
    joint_names += " " + q.name[i];
  for(size_t i=0; i<q.position.size(); ++i)
    joint_positions += " " + boost::lexical_cast<std::string>(q.position[i]);
  ROS_INFO_STREAM("Setting joint states\n" << joint_names << "\n" << joint_positions);

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

  if(target_visible(last_q_, "boxy"))
  {
    ROS_INFO("[%s] Target already visible. Not moving head.",
        nh_.getNamespace().c_str());
    response.success = true;
    return true;
  }

  // FIXME: keep on implementing me

  return true;
}

bool VisibilityMover::step_simulation(size_t steps)
{
  gazebo::msgs::WorldControl msg;
  msg.set_multi_step(steps);
  world_control_publisher_->Publish(msg);

  return true;
}

bool VisibilityMover::target_visible(const sensor_msgs::JointState& q, const std::string& robot_name)
{
  size_t ticks = 5;
  for(size_t tick=0; tick<ticks; ++tick)
  {
    if(!set_joint_states(q, robot_name))
      return false;
  
    if(!step_simulation(1))
      return false;
  }

  sensor_msgs::JointState left_arm_away;
  for(size_t i=0; i<7; ++i)
  {
    left_arm_away.name.push_back("left_arm_" + boost::lexical_cast<std::string>(i) + "_joint");
    left_arm_away.position.push_back(0.0);
  }
  // FIXME: keep on implementing me

  for(size_t tick=0; tick<ticks; ++tick)
  {
    if(!set_joint_states(left_arm_away, robot_name))
      return false;
  
    if(!step_simulation(1))
      return false;
  }

  return true;
}
