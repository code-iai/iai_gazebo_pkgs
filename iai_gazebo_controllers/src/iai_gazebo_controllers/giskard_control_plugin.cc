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
#include "iai_gazebo_controllers/giskard_control_plugin.hh"
#include "iai_gazebo_controllers/gazebo_utils.hh"
#include <gazebo/msgs/msgs.hh>

using namespace iai_gazebo_controllers;
using namespace gazebo;

// Register this plugin with the simulator
GZ_REGISTER_WORLD_PLUGIN(GiskardControlPlugin)

//////////////////////////////////////////////////

GiskardControlPlugin::GiskardControlPlugin()
{
}

//////////////////////////////////////////////////

GiskardControlPlugin::~GiskardControlPlugin()
{
}

//////////////////////////////////////////////////

void GiskardControlPlugin::Load(physics::WorldPtr world, sdf::ElementPtr self_description)
{
  InitInternals(world, self_description);

  InitGazeboCommunication();
}

//////////////////////////////////////////////////

void GiskardControlPlugin::UpdateCallback(const common::UpdateInfo& info)
{
  if(MotionFinished())
    if(controller_specs_.empty())
      RequestGazeboShutdown();  
    else
      InitNextController();

  // TODO: get this number from somewhere
  // TODO: to sth smarter than just dying
  assert(controller_.update(GetObservables(), 10));

  SetCommand(controller_.get_command());
}

//////////////////////////////////////////////////

void GiskardControlPlugin::InitInternals(gazebo::physics::WorldPtr world, sdf::ElementPtr self_description)
{
  world_ = world;
  self_description_ = self_description;

  InitControlledModel();
  InitObservedModel();
  ReadMotionDescriptions();
  InitNextController();
}

//////////////////////////////////////////////////

void GiskardControlPlugin::InitControlledModel()
{
  std::string controlledModelName;
  assert(GetSDFValue("controlledModel", self_description_, controlledModelName));
  controlled_model_ = world_->GetModel(controlledModelName);
  assert(controlled_model_.get());
  controlled_model_->SetGravityMode(false);
}

//////////////////////////////////////////////////

void GiskardControlPlugin::InitObservedModel()
{
  std::string model_name;
  assert(GetSDFValue("observedModel", self_description_, model_name));
  observed_model_ = world_->GetModel(model_name);
  assert(observed_model_.get());
}

//////////////////////////////////////////////////

void GiskardControlPlugin::InitNextController()
{
  assert(controller_specs_.size() > 0);
  controller_ = giskard::generate(controller_specs_[0]);
  controller_specs_.erase(controller_specs_.begin());
  // TODO: get this number from somewhere
  assert(controller_.start(GetObservables(), 10));

  cmdBuffer_.clear();
  // TODO: get this number from somewhere
  maxCmdBufferSize_ = 100;
}

//////////////////////////////////////////////////

void GiskardControlPlugin::ReadMotionDescriptions()
{
  // getting file name out of SDF
  std::string motion_file;
  assert(GetSDFValue("motionFile", self_description_, motion_file));

  // parsing yaml-file
  YAML::Node node = YAML::LoadFile(motion_file);
  controller_specs_ = node.as< std::vector< giskard::QPControllerSpec > >();

  // making sure that we got at least one spec, and that all of them compile
  assert(controller_specs_.size() > 0);
  for(size_t i=0; i<controller_specs_.size(); ++i)
    giskard::generate(controller_specs_[i]);
}

//////////////////////////////////////////////////

void GiskardControlPlugin::InitGazeboCommunication()
{
  updateConnection_ = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&GiskardControlPlugin::UpdateCallback, this, _1));

  transport::NodePtr node = transport::NodePtr(new transport::Node());
  node->Init(world_->GetName());
  serverControlPublisher_ = node->Advertise<msgs::ServerControl>("/gazebo/server/control");
}

//////////////////////////////////////////////////

bool GiskardControlPlugin::MotionFinished() const
{
  assert(maxCmdBufferSize_ > 0);

  if(cmdBuffer_.size() != maxCmdBufferSize_)
    return false;

  for(std::deque<Eigen::VectorXd>::const_iterator it = cmdBuffer_.begin(); it!=cmdBuffer_.end(); ++it)
    for(size_t row = 0; row < it->rows(); ++row)
      // TODO: sth more sophisticated here, please
      // TODO: get this number from somewhere
      if(it->operator()(row) > 0.03)
        return false;

  return true;
}

//////////////////////////////////////////////////

void GiskardControlPlugin::RequestGazeboShutdown()
{
  std::cout << "\n\nREQUESTING GAZEBO SERVER SHUTDOWN\n\n";
  msgs::ServerControl server_msg;
  server_msg.set_stop(true);
  serverControlPublisher_->Publish(server_msg);

}

//////////////////////////////////////////////////

Eigen::VectorXd GiskardControlPlugin::GetObservables()
{
  Eigen::VectorXd result(12);

  result.segment(0, 6) = PoseToGiskardInputs(controlled_model_->GetLinks()[0]->GetWorldPose());
  result.segment(6, 6) = PoseToGiskardInputs(observed_model_->GetLinks()[0]->GetWorldPose());

  return result;
}

//////////////////////////////////////////////////

void GiskardControlPlugin::SetCommand(const Eigen::VectorXd& command)
{
  // actually setting command
  controlled_model_->GetLinks()[0]->SetLinearVel(gazebo::math::Vector3(command(0), command(1), command(2)));
  controlled_model_->GetLinks()[0]->SetAngularVel(gazebo::math::Vector3(command(3), command(4), command(5)));

  // remembering command
  assert(maxCmdBufferSize_ > 0);
  if(cmdBuffer_.size() >= maxCmdBufferSize_)
    cmdBuffer_.pop_back();

  cmdBuffer_.push_front(command);
}
