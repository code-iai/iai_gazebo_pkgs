/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, Andrei Haidu, Georg Bartels
 *  Institute for Artificial Intelligence, Universität Bremen.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Institute for Artificial Intelligence,
 *     Universität Bremen, nor the names of its contributors may be
 *     used to endorse or promote products derived from this software
 *     without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/
#include <gazebo/msgs/msgs.hh>
#include <boost/bind.hpp>

#include "iai_gazebo_controllers/WorldControlPlugin.hh"
#include "iai_gazebo_controllers/gazebo_utils.hh"
#include "iai_gazebo_controllers/conversions.hh"


using namespace iai_gazebo_controllers;
using namespace gazebo;

// Register this plugin with the simulator
GZ_REGISTER_WORLD_PLUGIN(WorldControlPlugin)

//////////////////////////////////////////////////
WorldControlPlugin::WorldControlPlugin()
{
}

//////////////////////////////////////////////////
WorldControlPlugin::~WorldControlPlugin()
{
}

//////////////////////////////////////////////////
void WorldControlPlugin::Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf)
{
  self_ = _parent;
  self_description_ = _sdf;
   
  GetControlledModel();

  SetupConnections();

  ReadMotionDescriptions();

  GetStartDelay();
  // start thread which delays start of simulation
  checkStartDelay = new boost::thread(&WorldControlPlugin::DelaySimulationStart, this);
}

//////////////////////////////////////////////////
void WorldControlPlugin::DelaySimulationStart()
{
  self_->SetPaused(true);
  std::cout << "Paused the world, starting sim in " <<  startDelay << " sec..\n";
  usleep(startDelay * 1000000);
  self_->SetPaused(false);
}

//////////////////////////////////////////////////
void WorldControlPlugin::GetControlledModel()
{
  std::string controlledModelName;
  assert(GetSDFValue("controlledModel", self_description_, controlledModelName));
  controlled_model_ = self_->GetModel(controlledModelName);
  assert(controlled_model_);
}

//////////////////////////////////////////////////
void WorldControlPlugin::GetStartDelay()
{
  GetSDFValue("startDelay", self_description_, startDelay, 0.0);
  assert(startDelay >= 0.0);
}

//////////////////////////////////////////////////

void WorldControlPlugin::UpdateCallback(const common::UpdateInfo& info)
{
  // TODO: get me from the model plugin
}

//////////////////////////////////////////////////

void WorldControlPlugin::SetupConnections()
{
  // regular update callback
  updateConnection_ = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&WorldControlPlugin::UpdateCallback, this, _1));

  // publisher to request gazebo shutdown
  transport::NodePtr node = transport::NodePtr(new transport::Node());
  node->Init(self_->GetName());
  serverControlPublisher_ = node->Advertise<msgs::ServerControl>("/gazebo/server/control");
}

//////////////////////////////////////////////////

void WorldControlPlugin::ReadMotionDescriptions()
{
  std::string motion_file = "motions/sample-motion1.yaml";

  std::ifstream file_in(motion_file.c_str());
  assert(file_in.good());

  motions_.clear();
  YAML::Parser parser(file_in);
  YAML::Node doc;
  parser.GetNextDocument(doc);
  const YAML::Node& cmd_node = doc["motion-command"];
  cmd_node["simulation-start-delay"] >> simulation_start_delay_;
  assert(simulation_start_delay_ >= 0.0);
  const YAML::Node& motions_node = cmd_node["motions"];

  for(unsigned int i=0; i<motions_node.size(); ++i)
  {
    MotionDescription tmp_motion;
    motions_node[i] >> tmp_motion;
    motions_.push_back(tmp_motion);

    assert(tmp_motion.constraints_.isValid());
  }
}

//////////////////////////////////////////////////

void WorldControlPlugin::PerformVelocityControl(const Twist& twist)
{
  gazebo::physics::LinkPtr link = controlled_model_->GetLinks()[0];
  link->SetLinearVel(twist.linear_velocity_);
  link->SetAngularVel(twist.angular_velocity_);
}

//////////////////////////////////////////////////

void WorldControlPlugin::FillTransformMap()
{
  physics::ModelPtr stove = self_->GetModel("PancakeMaker");
  math::Pose stove_pose = stove->GetLinks()[0]->GetWorldPose();
  math::Pose cup_pose = controlled_model_->GetLinks()[0]->GetWorldPose();
 
  Transform cup_transform, stove_transform; 
  cup_transform.semantics().reference().setName("World");
  cup_transform.semantics().target().setName("Cup");
  stove_transform.semantics().reference().setName("World");
  stove_transform.semantics().target().setName("PancakeMaker");
  cup_transform.numerics() = toKDL(cup_pose);
  stove_transform.numerics() = toKDL(stove_pose);
  transforms_.setTransform(cup_transform);
  transforms_.setTransform(stove_transform);
}

//////////////////////////////////////////////////

void WorldControlPlugin::SwitchToNextMotionPhase()
{
  std::cout << "Starting new motion at time: " << self_->GetSimTime().Double() << "\n";
  current_motion_index_ += 1;
  last_control_time_ = self_->GetSimTime();
  InitController(motions_, current_motion_index_);
} 

//////////////////////////////////////////////////

bool WorldControlPlugin::CurrentMotionPhaseOver() const
{
  return controller_.constraints().areFulfilled() &&
      (accumulated_convergence_time_.Double() >= motions_[current_motion_index_].finish_delay_);
}

//////////////////////////////////////////////////

bool WorldControlPlugin::MoreMotionPhasesRemaining() const
{
  return (current_motion_index_ + 1 < motions_.size());
}


//////////////////////////////////////////////////

void WorldControlPlugin::InitController(const std::vector<MotionDescription>& motions, unsigned int index)
{
  controlled_model_->SetGravityMode(false);

  accumulated_convergence_time_.Set(0.0);

  controller_.init(motions[index].constraints_);
  fccl::control::PIDGains gains;
  gains.init(controller_.constraints().names());
  for(unsigned int i=0; i<gains.size(); ++i)
    gains.p().numerics()(i) = 50.0;
  controller_.setGains(gains); 
  FillTransformMap();
  controller_.start(transforms_, GetCycleTime(DEFAULT_CYCLE_TIME).Double());
}
 
//////////////////////////////////////////////////

gazebo::common::Time WorldControlPlugin::GetCycleTime(double default_cycle_time) const
{
  common::Time cycle_time = self_->GetSimTime() - last_control_time_;

  if(cycle_time.Double() <= 0.0)
    cycle_time = common::Time(default_cycle_time);

  return cycle_time;
}

//////////////////////////////////////////////////


//////////////////////////////////////////////////


//////////////////////////////////////////////////


//////////////////////////////////////////////////


//////////////////////////////////////////////////


//////////////////////////////////////////////////


