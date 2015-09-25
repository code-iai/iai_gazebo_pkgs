#include <iai_gazebo_visibility_mover/visibility_mover.hpp>
#include <gazebo_msgs/BodyRequest.h>
#include <gazebo_msgs/SpawnModel.h>
#include <gazebo_msgs/DeleteModel.h>
#include <gazebo_msgs/SetModelConfiguration.h>
#include <gazebo_msgs/SetModelState.h>
#include <gazebo_msgs/GetModelState.h>
#include <gazebo/msgs/msgs.hh>
#include <algorithm>

using namespace iai_gazebo;

VisibilityMover::VisibilityMover(const ros::NodeHandle& nh) :
    nh_( nh ), robot_description_( "" ), pixel_count_( 0 ),
    has_new_pixel_count_( false )
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

  if(!read_alternative_configs())
  {
    ROS_ERROR("[%s] Could not read alternative head configs",
        nh_.getNamespace().c_str());
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

  clear_body_wrench_client_ = nh_.serviceClient<gazebo_msgs::BodyRequest>("/gazebo/clear_body_wrenches");
  if(!clear_body_wrench_client_.waitForExistence(ros::Duration(2.0)))
  {
    ROS_ERROR("[%s] Could not connect to service '/gazebo/clear_body_wrench'",
        nh_.getNamespace().c_str());
    return false;
  }

  set_model_state_client_ = nh_.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
  if(!set_model_state_client_.waitForExistence(ros::Duration(2.0)))
  {
    ROS_ERROR("[%s] Could not connect to service '/gazebo/set_model_state'",
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

  pixel_count_subscriber_ = nh_.subscribe("pixel_count", 1, 
      &VisibilityMover::pixel_count_callback, this);

  neck_command_publisher_ = nh_.advertise<sensor_msgs::JointState>("neck_command", 1);

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
      {
        ROS_INFO("[%s] Delete Model successful.", nh_.getNamespace().c_str());
        result = true;
      }
    }
    else
    {
      ROS_ERROR("[%s] Failed to call service '/gazebo/delete_model'.",
          nh_.getNamespace().c_str());
      result = false;
    }
  }
  else
  {
    ROS_INFO("[%s] Delete Model unnecessary.", nh_.getNamespace().c_str());
    result = true;
  }

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
    {
      ROS_INFO("[%s] Found model %s", nh_.getNamespace().c_str(), robot_name.c_str());
      result = true;
    }
  }
  else
  {
    ROS_ERROR("[%s] Failed to call service '/gazebo/get_model_state'.",
        nh_.getNamespace().c_str());
    result = false;
  }

  return result;
}

bool VisibilityMover::set_joint_states(const std::map<std::string, double>& q, 
    const std::string& robot_name, size_t iterations)
{
  sensor_msgs::JointState qmsg = to_msg(q);
  gazebo_msgs::SetModelConfiguration srv;
  srv.request.model_name = robot_name;
  srv.request.urdf_param_name = nh_.getNamespace() + "/robot_description";
  srv.request.joint_names = qmsg.name;
  srv.request.joint_positions = qmsg.position;

  // DEBUG PRINTOUT
  //std::string joint_names, joint_positions;
  //for(size_t i=0; i<q.name.size(); ++i)
  //  joint_names += " " + q.name[i];
  //for(size_t i=0; i<q.position.size(); ++i)
  //  joint_positions += " " + boost::lexical_cast<std::string>(q.position[i]);
  //ROS_INFO_STREAM("Setting joint states\n" << joint_names << "\n" << joint_positions);

  for(size_t i=0; i<iterations; ++i)
  {
    if(set_joint_states_client_.call(srv))
    {
      if(!srv.response.success)
      {
        ROS_ERROR("[%s] Set Joint States unsuccessful: %s", nh_.getNamespace().c_str(),
          srv.response.status_message.c_str()); 
        return false;
      }
    }
    else
    {
      ROS_ERROR("[%s] Failed to call service '/gazebo/set_model_configuration'.",
          nh_.getNamespace().c_str());
      return false;
    }
  }

  if(!clear_body_wrenches())
    return false;

  if(!reset_base_pose(robot_name))
   return false;

  return true;
}

void VisibilityMover::joint_state_callback(const sensor_msgs::JointState::ConstPtr& msg)
{
  last_q_ = *msg;
}

void VisibilityMover::pixel_count_callback(const std_msgs::UInt64::ConstPtr& msg)
{
  has_new_pixel_count_ = true;
  pixel_count_ = msg->data;
}

bool VisibilityMover::trigger_callback(std_srvs::Trigger::Request& request,
    std_srvs::Trigger::Response& response)
{
  std::string robot_name = "boxy";
  double threshold = 0.1;
  size_t setting_iterations = 2;

  response.success = false;

  std::map<std::string, double> last_q = to_map(last_q_);

  if(target_visible(last_q, robot_name, threshold))
  {
    ROS_INFO("[%s] Target already visible. Not moving head.",
        nh_.getNamespace().c_str());
    response.success = true;
    return true;
  }

  std::random_shuffle ( alternative_configs_.begin(), alternative_configs_.end() );
  for(size_t i=0; i<alternative_configs_.size(); ++i)
  {
    if(target_visible(merge_configs(last_q, alternative_configs_[i]), robot_name, threshold))
    {
      ROS_INFO("[%s] Target already visible in alternative config. Moving head.",
          nh_.getNamespace().c_str());
      neck_command_publisher_.publish(to_msg(alternative_configs_[i]));
      response.success = true;
      return true;
    }
  }
 
  return true;
}

bool VisibilityMover::step_simulation(size_t steps)
{
  gazebo::msgs::WorldControl msg;
  msg.set_multi_step(steps);
  world_control_publisher_->Publish(msg);

  return true;
}

bool VisibilityMover::target_visible(const std::map<std::string, double>& q, const std::string& robot_name,
    double threshold)
{
  // ACQUIRE PRE-PIXEL-COUNT
  size_t setting_iterations = 2;
  size_t sim_steps = 1;

  has_new_pixel_count_ = false;

  if(!set_joint_states(q, robot_name, setting_iterations))
    return false;

  if(!step_simulation(sim_steps))
    return false;

  wait_for_pixel_count();
  size_t pre_pixel_count = pixel_count_;

  std::map<std::string, double> left_arm_away;
  for(size_t i=0; i<7; ++i)
  {
    left_arm_away["left_arm_" + boost::lexical_cast<std::string>(i) + "_joint"] = 0.0;
  }

  // ACQUIRE POST-PIXEL-COUNT
  if(!set_joint_states(merge_configs(q, left_arm_away), robot_name, setting_iterations))
    return false;

  if(!step_simulation(sim_steps))
    return false;

  has_new_pixel_count_ = false;
  wait_for_pixel_count();
  size_t post_pixel_count = pixel_count_;

  // DECISION
  int delta = static_cast<int>(post_pixel_count) - static_cast<int>(pre_pixel_count);
  double ratio = static_cast<double>(delta) / static_cast<double>(post_pixel_count);
  ROS_INFO_STREAM("pre: " << pre_pixel_count << ", post: " << post_pixel_count << 
      ", delta: " << delta << ", ratio:" << ratio << ", treshold: " << threshold);
  return (post_pixel_count > 0) && (ratio < threshold);
}

bool VisibilityMover::clear_body_wrenches()
{
  std::vector<std::string> body_names;
  body_names.push_back("ground_plane::link");
  body_names.push_back("boxy::base_footprint");
  body_names.push_back("boxy::neck_shoulder_link");
  body_names.push_back("boxy::neck_upper_arm_link");
  body_names.push_back("boxy::neck_forearm_link");
  body_names.push_back("boxy::neck_wrist_1_link");
  body_names.push_back("boxy::neck_wrist_2_link");
  body_names.push_back("boxy::neck_wrist_3_link");
  body_names.push_back("boxy::triangle_base_link");
  body_names.push_back("boxy::left_arm_1_link");
  body_names.push_back("boxy::left_arm_2_link");
  body_names.push_back("boxy::left_arm_3_link");
  body_names.push_back("boxy::left_arm_4_link");
  body_names.push_back("boxy::left_arm_5_link");
  body_names.push_back("boxy::left_arm_6_link");
  body_names.push_back("boxy::left_arm_7_link");
  body_names.push_back("boxy::left_gripper_gripper_left_link");
  body_names.push_back("boxy::left_gripper_gripper_right_link");
  body_names.push_back("boxy::right_arm_1_link");
  body_names.push_back("boxy::right_arm_2_link");
  body_names.push_back("boxy::right_arm_3_link");
  body_names.push_back("boxy::right_arm_4_link");
  body_names.push_back("boxy::right_arm_5_link");
  body_names.push_back("boxy::right_arm_6_link");
  body_names.push_back("boxy::right_arm_7_link");
  body_names.push_back("boxy::right_gripper_gripper_left_link");
  body_names.push_back("boxy::right_gripper_gripper_right_link");

  gazebo_msgs::BodyRequest srv;
  for(size_t i=0; i<body_names.size(); ++i)
  {
    srv.request.body_name = body_names[i];
    if(!clear_body_wrench_client_.call(srv))
    {
      ROS_ERROR("[%s] Failed to call service '/gazebo/clear_body_wrench' for body '%s'.",
          nh_.getNamespace().c_str(), body_names[i].c_str());
      return false;
    }
  }

  return true;
}

void VisibilityMover::wait_for_pixel_count()
{
  ROS_DEBUG("Waiting for pixel count...");
  
  while(ros::ok() && !has_new_pixel_count_)
    ros::spinOnce();
  ROS_DEBUG("Done.");
}

bool VisibilityMover::read_alternative_configs()
{

  std::vector<std::string> joint_names;
  joint_names.push_back("neck_shoulder_pan_joint");
  joint_names.push_back("neck_shoulder_lift_joint");
  joint_names.push_back("neck_elbow_joint");
  joint_names.push_back("neck_wrist_1_joint");
  joint_names.push_back("neck_wrist_2_joint");
  joint_names.push_back("neck_wrist_3_joint");

  std::vector<double> config1, config2, config3, config4, config5,
      config6, config7, config8, config9, config10, config11,
      config12, config13, config14, config15;
  config1.push_back(1.6133809977777778);
  config1.push_back(-1.1410953011111111);
  config1.push_back(1.635197595);
  config1.push_back(-1.4261073272222222);
  config1.push_back(1.5552615827777778);
  config1.push_back(3.0162754655555553);

  config2.push_back(0.2483704537153244);
  config2.push_back(-0.2301581541644495);
  config2.push_back(0.768549919128418);
  config2.push_back(-0.8555524984942835);
  config2.push_back(0.561461329460144);
  config2.push_back(4.1270880699157715);

  config3.push_back(2.637866258621216);
  config3.push_back(-0.7665770689593714);
  config3.push_back(1.292036533355713);
  config3.push_back(-0.40053111711610967);
  config3.push_back(1.8659439086914062);
  config3.push_back(2.288059711456299);

  config4.push_back(1.2661278247833252);
  config4.push_back(-0.17604047456850225);
  config4.push_back(0.48613643646240234);
  config4.push_back(-0.17137366930116826);
  config4.push_back(1.5050984621047974);
  config4.push_back(3.1847846508026123);

  config5.push_back(1.4161126613616943);
  config5.push_back(-1.0655291716205042);
  config5.push_back(2.187502861022949);
  config5.push_back(-1.2248061339007776);
  config5.push_back(1.5577870607376099);
  config5.push_back(3.015765428543091);

  config6.push_back(1.272057294845581);
  config6.push_back(-1.348339859639303);
  config6.push_back(1.421757698059082);
  config6.push_back(-0.2135232130633753);
  config6.push_back(1.3236674070358276);
  config6.push_back(3.1781420707702637);

  config7.push_back(1.2720932960510254);
  config7.push_back(-2.349339310322897);
  config7.push_back(2.1557388305664062);
  config7.push_back(-0.12168628374208623);
  config7.push_back(1.3236074447631836);
  config7.push_back(3.1781539916992188);

  config8.push_back(2.006105422973633);
  config8.push_back(-1.2148321310626429);
  config8.push_back(2.0223541259765625);
  config8.push_back(-0.4892209211932581);
  config8.push_back(1.3235955238342285);
  config8.push_back(2.259474039077759);

  config9.push_back(2.0060935020446777);
  config9.push_back(-0.7477543989764612);
  config9.push_back(1.5552387237548828);
  config9.push_back(-0.5810282866107386);
  config9.push_back(1.3236314058303833);
  config9.push_back(2.259485960006714);

  config10.push_back(2.006129264831543);
  config10.push_back(-0.2138903776751917);
  config10.push_back(0.4875149726867676);
  config10.push_back(0.062090277671813965);
  config10.push_back(1.5991644859313965);
  config10.push_back(2.2594261169433594);

  config11.push_back(-0.12931424776186162);
  config11.push_back(-0.41399985948671514);
  config11.push_back(0.4874911308288574);
  config11.push_back(0.42949092388153076);
  config11.push_back(0.9561718702316284);
  config11.push_back(3.821215867996216);
  
  config12.push_back(0.20434153079986572);
  config12.push_back(-0.41409570375551397);
  config12.push_back(0.4875149726867676);
  config12.push_back(0.42950308322906494);
  config12.push_back(1.3236314058303833);
  config12.push_back(3.821215867996216);

  config13.push_back(0.8049560785293579);
  config13.push_back(-0.3473094145404261);
  config13.push_back(0.48746681213378906);
  config13.push_back(-0.12171060243715459);
  config13.push_back(1.3236194849014282);
  config13.push_back(3.821239948272705);

  config14.push_back(0.33774617314338684);
  config14.push_back(-1.1480568091021937);
  config14.push_back(1.688767433166504);
  config14.push_back(-0.48914891878236944);
  config14.push_back(1.1398762464523315);
  config14.push_back(3.8212039470672607);

  config15.push_back(1.4722658395767212);
  config15.push_back(-1.1481288115130823);
  config15.push_back(1.6887316703796387);
  config15.push_back(-0.48914891878236944);
  config15.push_back(1.6911078691482544);
  config15.push_back(3.0863242149353027);

  sensor_msgs::JointState msg;
  msg.name = joint_names;
  msg.position = config1;
  alternative_configs_.push_back(to_map(msg));
  msg.position = config2;
  alternative_configs_.push_back(to_map(msg));
  msg.position = config3;
  alternative_configs_.push_back(to_map(msg));
  msg.position = config4;
  alternative_configs_.push_back(to_map(msg));
  msg.position = config5;
  alternative_configs_.push_back(to_map(msg));
  msg.position = config5;
  alternative_configs_.push_back(to_map(msg));
  msg.position = config6;
  alternative_configs_.push_back(to_map(msg));
  msg.position = config7;
  alternative_configs_.push_back(to_map(msg));
  msg.position = config8;
  alternative_configs_.push_back(to_map(msg));
  msg.position = config9;
  alternative_configs_.push_back(to_map(msg));
  msg.position = config10;
  alternative_configs_.push_back(to_map(msg));
  msg.position = config11;
  alternative_configs_.push_back(to_map(msg));
  msg.position = config12;
  alternative_configs_.push_back(to_map(msg));
  msg.position = config13;
  alternative_configs_.push_back(to_map(msg));
  msg.position = config14;
  alternative_configs_.push_back(to_map(msg));
  msg.position = config15;
  alternative_configs_.push_back(to_map(msg));

  return true;
}

bool VisibilityMover::reset_base_pose(const std::string& robot_name)
{
  gazebo_msgs::SetModelState srv;
  srv.request.model_state.model_name = robot_name;
  srv.request.model_state.pose.orientation.w = 1.0;

  if(set_model_state_client_.call(srv))
  {
    if(!srv.response.success)
    {
      ROS_ERROR("[%s] Set Model State unsuccessful: %s", nh_.getNamespace().c_str(),
        srv.response.status_message.c_str()); 
      return false;
    }
  }
  else
  {
    ROS_ERROR("[%s] Failed to call service '/gazebo/set_model_state'.",
        nh_.getNamespace().c_str());
    return false;
  }

  return true;
}

std::map<std::string, double> VisibilityMover::to_map(const sensor_msgs::JointState& q) const
{
  std::map<std::string, double> result;

  assert(q.position.size() == q.name.size());
  for(size_t i=0; i< q.position.size(); ++i)
    result[q.name[i]] = q.position[i];

  return result;
}

sensor_msgs::JointState VisibilityMover::to_msg(const std::map<std::string, double>& q) const
{
  sensor_msgs::JointState result;

  for (std::map<std::string, double>::const_iterator it=q.begin(); it!=q.end(); ++it)
  {
    result.name.push_back(it->first);
    result.position.push_back(it->second);
  }
  return result;
}

std::map<std::string, double> VisibilityMover::merge_configs(const std::map<std::string, double>& base,
    const std::map<std::string, double>& replacement) const
{
  std::map<std::string, double> result = base;

  for (std::map<std::string, double>::const_iterator it=replacement.begin(); 
       it!=replacement.end(); ++it)
    result[it->first] = it->second;

  return result;
}

